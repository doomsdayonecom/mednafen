/* Super Nintendo (snes_faust module) control backend for the Retro Remote
 * Debug Controller.
 *
 * Maps the portable RRDC backend (the vendored shared core under
 * src/extern/retro-remote-debug-controller) onto Mednafen's snes_faust core —
 * the fast SNES module (shortname "snes_faust"), NOT the bsnes-derived "snes"
 * module. Mirrors pce_control.cpp / md_control.cpp: emulator-specific state is
 * reached through tiny extern "C" accessors added to snes_faust/snes.cpp
 * (SNES_GetWRAM / SNES_GetRegs / SNES_ControlReset) and snes_faust/input.cpp
 * (SNES_SetPad / SNES_GetPad), and the framebuffer is pushed each frame from
 * the frontend hook (snes_control_frame).
 *
 * /mem speaks BUS addresses: the 128 KB work RAM sits at $7E0000-$7FFFFF, so a
 * WRAM symbol address from a test's linker map reads directly. The low 8 KB is
 * also served at $000000-$001FFF, the bank-0 system-area mirror every LoROM
 * program actually addresses it through. A byte outside both windows ends the
 * transfer, so the returned count is honest.
 *
 * Enabled by setting MEDNAFEN_CONTROLPORT=<port> in the environment. */

#include "../extern/retro-remote-debug-controller/core/retro_control.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>

/* --- accessors implemented in snes_faust/snes.cpp / input.cpp ------------ */
extern "C" uint8_t *SNES_GetWRAM(uint32_t *size_out);  /* 128 KB at bus $7E0000 */
extern "C" void     SNES_GetRegs(uint32_t *out9);      /* [0]=pc(pbr:pc) [1]=dbr [2]=s [3]=d [4]=a [5]=x [6]=y [7]=p [8]=e */
extern "C" void     SNES_ControlReset(void);           /* Reset(true) — power cycle */
extern "C" int      SNES_SetPad(int index, unsigned buttons, int connected);
extern "C" int      SNES_GetPad(int index, unsigned *buttons, int *connected);

/* --- captured framebuffer (filled by the frontend hook each frame) ------- *
 * Variable-size, pre-converted to packed RGB888 so /screenshot is correct
 * regardless of Mednafen's surface channel order. snes_faust renders 256 or
 * 512 wide and up to 239 (478 interlaced) lines; the buffer covers the lot. */
#define FB_MAX_W 512
#define FB_MAX_H 480
static uint8_t  g_fb[FB_MAX_W * FB_MAX_H * 3];
static int      g_fb_w = 0, g_fb_h = 0;
static uint64_t g_frames = 0;

/* --- captured audio ring (single-threaded, as in pce_control.cpp) -------- */
#define AUD_RING 65536
static int16_t  g_aud[AUD_RING];
static uint32_t g_aud_head = 0, g_aud_count = 0, g_aud_dropped = 0;
static int      g_aud_rate = 0, g_aud_chans = 2;

/* --- backend callbacks --------------------------------------------------- */

/* Translate one SNES bus address to a host pointer into WRAM, or NULL if it
 * lands outside the served windows ($7E0000-$7FFFFF, and the bank-0 low-RAM
 * mirror $000000-$001FFF). */
static uint8_t *snes_bus_ptr(uint32_t addr)
{
    uint32_t size = 0;
    uint8_t *wram = SNES_GetWRAM(&size);
    if (!wram || !size) return NULL;
    if (addr >= 0x7E0000 && addr <= 0x7FFFFF)
        return wram + ((addr - 0x7E0000) & (size - 1));
    if (addr <= 0x001FFF)                        /* bank-0 system-area mirror */
        return wram + addr;
    return NULL;
}

static uint32_t snes_read_mem(uint32_t addr, int32_t bank, uint32_t len,
                              uint8_t *out, uint32_t cap)
{
    uint32_t n = 0;
    (void)bank;
    for (; n < len && n < cap; n++) {
        uint8_t *p = snes_bus_ptr(addr + n);
        if (!p) break;                           /* honest short read        */
        out[n] = *p;
    }
    return n;
}

static uint32_t snes_write_mem(uint32_t addr, int32_t bank, uint32_t len,
                               const uint8_t *in)
{
    uint32_t n = 0;
    (void)bank;
    for (; n < len; n++) {
        uint8_t *p = snes_bus_ptr(addr + n);
        if (!p) break;
        *p = in[n];
    }
    return n;
}

static void snes_get_regs_json(char *buf, size_t cap)
{
    uint32_t r[9];
    SNES_GetRegs(r);
    snprintf(buf, cap,
             "{\"pc\":%u,\"dbr\":%u,\"s\":%u,\"d\":%u,"
             "\"a\":%u,\"x\":%u,\"y\":%u,\"p\":%u,\"e\":%u}",
             (unsigned)r[0], (unsigned)r[1], (unsigned)r[2], (unsigned)r[3],
             (unsigned)r[4], (unsigned)r[5], (unsigned)r[6], (unsigned)r[7],
             (unsigned)r[8]);
}

static void snes_get_framebuffer(retro_framebuffer_t *out)
{
    out->pixels = g_fb;                  /* already packed RGB888 */
    out->width  = g_fb_w;
    out->height = g_fb_h;
    out->fmt    = RETRO_PIX_RGB888;
}

static uint64_t snes_get_frame_count(void) { return g_frames; }

static void snes_reset(void) { SNES_ControlReset(); }

/* Key injection is not wired for the SNES (the pad is the input device; use
 * /pad); the honest stub keeps /status advertising the cumulative contract
 * while /key answers 400. */
static int snes_inject_key(int is_text, uint32_t value, int action)
{ (void)is_text; (void)value; (void)action; return 0; }

/* 0.5: virtual game controller. RRDC's CANONICAL mask (bit0 LEFT 1 RIGHT 2 UP
 * 3 DOWN 4 A 5 B 6 X 7 Y 8 START 9 SELECT 10 L 11 R — see pcfx_control.cpp)
 * remapped to snes_faust's gamepad data word, whose bit layout is the IDII
 * order in input/gamepad.cpp (git.cpp assigns BitOffset += BitSize walking the
 * array): B=0 Y=1 SELECT=2 START=3 UP=4 DOWN=5 LEFT=6 RIGHT=7 A=8 X=9 L=10
 * R=11 — the SNES controller's own serial shift order, so the auto-read
 * JOY1L/H the guest polls carries exactly these bits. input.cpp OR-merges the
 * held mask into the pad data every core-side input update — the real read
 * path, no SDL (headless CI has no joystick, which is what this exists for). */
static const int8_t snes_pad_from_canon[12] = {
    /*[0]  LEFT  */  6,
    /*[1]  RIGHT */  7,
    /*[2]  UP    */  4,
    /*[3]  DOWN  */  5,
    /*[4]  A     */  8,
    /*[5]  B     */  0,
    /*[6]  X     */  9,
    /*[7]  Y     */  1,
    /*[8]  START */  3,
    /*[9]  SELECT*/  2,
    /*[10] L     */ 10,
    /*[11] R     */ 11,
};

static unsigned snes_mask_from_canon(int canon)
{
    unsigned out = 0;
    for (int b = 0; b < 12; b++)
        if (canon & (1 << b)) out |= (1u << snes_pad_from_canon[b]);
    return out;
}

static int snes_canon_from_mask(unsigned raw)
{
    int out = 0;
    for (int b = 0; b < 12; b++)
        if (raw & (1u << snes_pad_from_canon[b])) out |= (1 << b);
    return out;
}

static int snes_set_pad(int index, int buttons, int connected)
{
    unsigned cur = 0; int curconn = 0;
    if (!SNES_GetPad(index, &cur, &curconn)) return 0;   /* bad index -> 400 */
    unsigned mask = (buttons   < 0) ? cur     : snes_mask_from_canon(buttons);
    int      conn = (connected < 0) ? curconn : connected;   /* -1 = leave as-is */
    return SNES_SetPad(index, mask, conn);
}

static int snes_get_pad(int index, int *buttons, int *connected)
{
    unsigned raw = 0; int conn = 0;
    if (!SNES_GetPad(index, &raw, &conn)) return 0;
    if (buttons)   *buttons   = snes_canon_from_mask(raw);
    if (connected) *connected = conn;
    return 1;
}

/* No pointing device wired (the mouse device exists but no test needs it);
 * honest stubs (see pcfx_control.cpp). */
static int snes_set_pointer(int absolute, int32_t x, int32_t y, int buttons)
{ (void)absolute; (void)x; (void)y; (void)buttons; return 0; }
static int snes_get_pointer(int32_t *x, int32_t *y, int *buttons)
{ (void)x; (void)y; (void)buttons; return 0; }

static uint32_t snes_capture_audio(int16_t *out, uint32_t cap,
                                   int *rate, int *channels, uint32_t *dropped)
{
    uint32_t n = 0;
    if (rate)     *rate     = g_aud_rate ? g_aud_rate : 48000;
    if (channels) *channels = g_aud_chans;
    if (dropped)  *dropped  = g_aud_dropped;
    g_aud_dropped = 0;
    while (n < cap && g_aud_count) {
        out[n++] = g_aud[g_aud_head];
        g_aud_head = (g_aud_head + 1) % AUD_RING;
        g_aud_count--;
    }
    return n;
}

static const retro_control_backend_t snes_backend = {
    /*platform*/       "snes",
    /*emulator*/       "mednafen",
    /*read_mem*/       snes_read_mem,
    /*get_regs_json*/  snes_get_regs_json,
    /*get_framebuffer*/snes_get_framebuffer,
    /*get_frame_count*/snes_get_frame_count,
    /*inject_key*/     snes_inject_key,
    /*reset*/          snes_reset,
    /*write_mem*/      snes_write_mem,
    /*capture_audio*/  snes_capture_audio,
    /*set_pointer*/    snes_set_pointer,
    /*get_pointer*/    snes_get_pointer,
    /*set_pad*/        snes_set_pad,
    /*get_pad*/        snes_get_pad,
};

/* --- frontend hooks (called from src/drivers/main.cpp) ------------------- */
extern "C" void snes_control_init(void)
{
    static int started = 0;
    if (started) return;
    started = 1;
    const char *p = getenv("MEDNAFEN_CONTROLPORT");
    if (p && *p)
        retro_control_start(atoi(p), &snes_backend);
}

extern "C" void snes_control_service(void)  { retro_control_service(); }
extern "C" int  snes_control_running(void)  { return retro_control_running(); }
extern "C" void snes_control_on_frame(void) { retro_control_on_frame(); }

/* Copy this frame's displayed pixels, decoding each 32-bit surface pixel to
 * packed RGB888 with the surface's channel shifts. snes_faust can mix 256- and
 * 512-wide lines in one frame (hires/pseudo-hires), so each line is
 * nearest-neighbour normalized to the frame's widest line via line_widths —
 * the same normalization as pcfx_control.cpp. A plain 256x224 mode-1 frame
 * passes through 1:1. */
extern "C" void snes_control_frame(const uint32_t *pixels, int w, int h, int pitch,
                                   const int32_t *line_widths, int rsh, int gsh, int bsh)
{
    if (pixels && w > 0 && h > 0) {
        if (h > FB_MAX_H) h = FB_MAX_H;
        int W = w;
        for (int y = 0; y < h; y++) {
            int lw = line_widths ? (int)line_widths[y] : w;
            if (lw > W) W = lw;
        }
        if (W > FB_MAX_W) W = FB_MAX_W;
        for (int y = 0; y < h; y++) {
            int lw = line_widths ? (int)line_widths[y] : w;
            if (lw <= 0) lw = w;
            const uint32_t *row = &pixels[(size_t)y * pitch];
            uint8_t *o = &g_fb[(size_t)y * W * 3];
            for (int x = 0; x < W; x++) {
                uint32_t px = row[(size_t)x * lw / W];
                *o++ = (uint8_t)(px >> rsh);
                *o++ = (uint8_t)(px >> gsh);
                *o++ = (uint8_t)(px >> bsh);
            }
        }
        g_fb_w = W;
        g_fb_h = h;
    }
    g_frames++;
}

extern "C" void snes_control_audio(const int16_t *samples, int frames,
                                   int channels, double rate)
{
    if (!samples || frames <= 0 || channels <= 0) return;
    g_aud_rate  = (int)(rate + 0.5);
    g_aud_chans = channels;
    uint32_t n = (uint32_t)frames * (uint32_t)channels;
    for (uint32_t i = 0; i < n; i++) {
        if (g_aud_count == AUD_RING) { g_aud_dropped++; continue; }
        g_aud[(g_aud_head + g_aud_count) % AUD_RING] = samples[i];
        g_aud_count++;
    }
}
