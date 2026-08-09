/* Game Boy Advance control backend for the Retro Remote Debug Controller.
 *
 * Maps the portable RRDC backend (the vendored shared core under
 * src/extern/retro-remote-debug-controller) onto Mednafen's GBA core. Mirrors
 * pce_control.cpp / md_control.cpp: emulator-specific state is reached through
 * tiny extern "C" accessors added to gba/GBA.cpp (GBA_GetEWRAM / GBA_GetIWRAM /
 * GBA_GetRegs / GBA_ControlReset / GBA_SetPad / GBA_GetPad), and the
 * framebuffer is pushed each frame from the frontend hook (gba_control_frame).
 *
 * /mem speaks BUS addresses, not array offsets: a GBA program's globals live in
 * EWRAM (0x02000000, 256 KB) and IWRAM (0x03000000, 32 KB), which is where the
 * linker script puts them and therefore what any symbol-address a test reads out
 * of the map file looks like. Each byte is translated region-by-region onto the
 * core's workRAM / internalRAM arrays (mirrored within its region, as the
 * hardware mirrors), and the 1 KB I/O block (0x04000000) is served read-only —
 * so a test can watch KEYINPUT (0x04000130) confirm a /pad injection exactly as
 * the guest would read it. A byte outside the served regions ends the transfer,
 * so the returned count is honest.
 *
 * The GBA LCD is a fixed 240x160 and Mednafen's DisplayRect delivers exactly
 * that, so the frame hook is the generic normalize-and-convert (pcfx-style)
 * rather than a crop: whatever the core displays is what /screenshot serves.
 *
 * Enabled by setting MEDNAFEN_CONTROLPORT=<port> in the environment. */

#include "../extern/retro-remote-debug-controller/core/retro_control.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>

/* --- accessors implemented in gba/GBA.cpp -------------------------------- */
extern "C" uint8_t *GBA_GetEWRAM(uint32_t *size_out);  /* 256 KB at bus 0x02000000 */
extern "C" uint8_t *GBA_GetIWRAM(uint32_t *size_out);  /* 32 KB at bus 0x03000000 */
extern "C" uint8_t *GBA_GetIOMEM(uint32_t *size_out);  /* 1 KB at bus 0x04000000 (read-only) */
extern "C" void     GBA_GetRegs(uint32_t *out18);      /* [0]=pc [1]=cpsr [2..17]=r0-r15 */
extern "C" void     GBA_ControlReset(void);            /* CPUReset() */
extern "C" int      GBA_SetPad(int index, unsigned buttons, int connected);
extern "C" int      GBA_GetPad(int index, unsigned *buttons, int *connected);

/* --- captured framebuffer (filled by the frontend hook each frame) ------- *
 * Variable-size, pre-converted to packed RGB888 so /screenshot is correct
 * regardless of Mednafen's surface channel order. The GBA displays 240x160;
 * the buffer is padded for the GSF-player screen just in case. */
#define FB_MAX_W 512
#define FB_MAX_H 256
static uint8_t  g_fb[FB_MAX_W * FB_MAX_H * 3];
static int      g_fb_w = 0, g_fb_h = 0;
static uint64_t g_frames = 0;

/* --- captured audio ring (single-threaded, as in pce_control.cpp) -------- */
#define AUD_RING 65536
static int16_t  g_aud[AUD_RING];
static uint32_t g_aud_head = 0, g_aud_count = 0, g_aud_dropped = 0;
static int      g_aud_rate = 0, g_aud_chans = 2;

/* --- backend callbacks --------------------------------------------------- */

/* Translate one GBA bus address to a host pointer, or NULL if it lands outside
 * the served regions. The work-RAM regions mirror across their whole 16 MB
 * slot on hardware, which the masks reproduce. The I/O block (KEYINPUT etc. as
 * the core last latched them) is served for READS only — a /mem poke there
 * would bypass every register's side effects, so writes refuse it. */
static uint8_t *gba_bus_ptr(uint32_t addr, int for_write)
{
    uint32_t size = 0;
    uint8_t *p;
    switch (addr >> 24) {
    case 0x02:                                   /* EWRAM: 256 KB, on-board  */
        p = GBA_GetEWRAM(&size);
        return (p && size) ? p + (addr & (size - 1)) : NULL;
    case 0x03:                                   /* IWRAM: 32 KB, on-chip    */
        p = GBA_GetIWRAM(&size);
        return (p && size) ? p + (addr & (size - 1)) : NULL;
    case 0x04:                                   /* I/O regs: 1 KB, read-only */
        if (for_write) return NULL;
        p = GBA_GetIOMEM(&size);
        return (p && size && (addr & 0x00FFFFFF) < size)
                   ? p + (addr & 0x00FFFFFF) : NULL;
    default:
        return NULL;
    }
}

static uint32_t gba_read_mem(uint32_t addr, int32_t bank, uint32_t len,
                             uint8_t *out, uint32_t cap)
{
    uint32_t n = 0;
    (void)bank;
    for (; n < len && n < cap; n++) {
        uint8_t *p = gba_bus_ptr(addr + n, 0);
        if (!p) break;                           /* honest short read        */
        out[n] = *p;
    }
    return n;
}

static uint32_t gba_write_mem(uint32_t addr, int32_t bank, uint32_t len,
                              const uint8_t *in)
{
    uint32_t n = 0;
    (void)bank;
    for (; n < len; n++) {
        uint8_t *p = gba_bus_ptr(addr + n, 1);
        if (!p) break;
        *p = in[n];
    }
    return n;
}

static void gba_get_regs_json(char *buf, size_t cap)
{
    uint32_t r[18];
    int off, i;
    GBA_GetRegs(r);
    off = snprintf(buf, cap, "{\"pc\":%u,\"cpsr\":%u",
                   (unsigned)r[0], (unsigned)r[1]);
    for (i = 0; i < 16 && off > 0 && (size_t)off < cap; i++)
        off += snprintf(buf + off, cap - off, ",\"r%d\":%u", i, (unsigned)r[2 + i]);
    if (off > 0 && (size_t)off < cap)
        snprintf(buf + off, cap - off, "}");
}

static void gba_get_framebuffer(retro_framebuffer_t *out)
{
    out->pixels = g_fb;                  /* already packed RGB888 */
    out->width  = g_fb_w;
    out->height = g_fb_h;
    out->fmt    = RETRO_PIX_RGB888;
}

static uint64_t gba_get_frame_count(void) { return g_frames; }

static void gba_reset(void) { GBA_ControlReset(); }

/* Key injection is not wired for the GBA (the pad is the input device; use
 * /pad); the honest stub keeps /status advertising the cumulative contract
 * while /key answers 400. */
static int gba_inject_key(int is_text, uint32_t value, int action)
{ (void)is_text; (void)value; (void)action; return 0; }

/* 0.5: virtual game controller. RRDC speaks a fixed CANONICAL button mask
 * (bit0 LEFT 1 RIGHT 2 UP 3 DOWN 4 A 5 B 6 X 7 Y 8 START 9 SELECT 10 L 11 R —
 * see pcfx_control.cpp) which we remap to the GBA pad bits as the core samples
 * them: the IDII order in gba/GBA.cpp assigns A=0 B=1 SELECT=2 START=3 RIGHT=4
 * LEFT=5 UP=6 DOWN=7 R=8 L=9 — which is exactly the hardware KEYINPUT layout.
 * We inject ACTIVE-HIGH into the pre-inversion sample (padbufblah); the core
 * itself derives the active-low KEYINPUT register (P1 = 0x03FF ^ joy), so the
 * guest reads 0 = pressed as on hardware. The GBA has no X/Y buttons: those
 * canonical bits map to nothing and are dropped (and never read back). */
static const int8_t gba_pad_from_canon[12] = {
    /*[0]  LEFT  */  5,
    /*[1]  RIGHT */  4,
    /*[2]  UP    */  6,
    /*[3]  DOWN  */  7,
    /*[4]  A     */  0,
    /*[5]  B     */  1,
    /*[6]  X     */ -1,   /* no such button */
    /*[7]  Y     */ -1,   /* no such button */
    /*[8]  START */  3,
    /*[9]  SELECT*/  2,
    /*[10] L     */  9,
    /*[11] R     */  8,
};

static unsigned gba_mask_from_canon(int canon)
{
    unsigned out = 0;
    for (int b = 0; b < 12; b++)
        if ((canon & (1 << b)) && gba_pad_from_canon[b] >= 0)
            out |= (1u << gba_pad_from_canon[b]);
    return out;
}

static int gba_canon_from_mask(unsigned raw)
{
    int out = 0;
    for (int b = 0; b < 12; b++)
        if (gba_pad_from_canon[b] >= 0 && (raw & (1u << gba_pad_from_canon[b])))
            out |= (1 << b);
    return out;
}

static int gba_set_pad(int index, int buttons, int connected)
{
    unsigned cur = 0; int curconn = 0;
    if (!GBA_GetPad(index, &cur, &curconn)) return 0;   /* bad index -> 400 */
    unsigned mask = (buttons   < 0) ? cur     : gba_mask_from_canon(buttons);
    int      conn = (connected < 0) ? curconn : connected;   /* -1 = leave as-is */
    return GBA_SetPad(index, mask, conn);
}

static int gba_get_pad(int index, int *buttons, int *connected)
{
    unsigned raw = 0; int conn = 0;
    if (!GBA_GetPad(index, &raw, &conn)) return 0;
    if (buttons)   *buttons   = gba_canon_from_mask(raw);
    if (connected) *connected = conn;
    return 1;
}

/* The GBA has no pointing device; honest stubs (see pcfx_control.cpp). */
static int gba_set_pointer(int absolute, int32_t x, int32_t y, int buttons)
{ (void)absolute; (void)x; (void)y; (void)buttons; return 0; }
static int gba_get_pointer(int32_t *x, int32_t *y, int *buttons)
{ (void)x; (void)y; (void)buttons; return 0; }

static uint32_t gba_capture_audio(int16_t *out, uint32_t cap,
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

static const retro_control_backend_t gba_backend = {
    /*platform*/       "gba",
    /*emulator*/       "mednafen",
    /*read_mem*/       gba_read_mem,
    /*get_regs_json*/  gba_get_regs_json,
    /*get_framebuffer*/gba_get_framebuffer,
    /*get_frame_count*/gba_get_frame_count,
    /*inject_key*/     gba_inject_key,
    /*reset*/          gba_reset,
    /*write_mem*/      gba_write_mem,
    /*capture_audio*/  gba_capture_audio,
    /*set_pointer*/    gba_set_pointer,
    /*get_pointer*/    gba_get_pointer,
    /*set_pad*/        gba_set_pad,
    /*get_pad*/        gba_get_pad,
};

/* --- frontend hooks (called from src/drivers/main.cpp) ------------------- */
extern "C" void gba_control_init(void)
{
    static int started = 0;
    if (started) return;
    started = 1;
    const char *p = getenv("MEDNAFEN_CONTROLPORT");
    if (p && *p)
        retro_control_start(atoi(p), &gba_backend);
}

extern "C" void gba_control_service(void)  { retro_control_service(); }
extern "C" int  gba_control_running(void)  { return retro_control_running(); }
extern "C" void gba_control_on_frame(void) { retro_control_on_frame(); }

/* Copy this frame's displayed pixels, decoding each 32-bit surface pixel to
 * packed RGB888 with the surface's channel shifts. The GBA's DisplayRect is a
 * uniform 240x160 (line_widths is NULL for a uniform frame), so this is the
 * pcfx-style pass-through normalization with no crop. */
extern "C" void gba_control_frame(const uint32_t *pixels, int w, int h, int pitch,
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

extern "C" void gba_control_audio(const int16_t *samples, int frames,
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
