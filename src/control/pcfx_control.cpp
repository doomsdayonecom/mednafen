/* PC-FX control backend for the Retro Remote Debug Controller.
 *
 * Maps the portable RRDC backend (the vendored shared core under
 * src/extern/retro-remote-debug-controller) onto Mednafen's
 * PC-FX core. To stay free of Mednafen's C++ headers, the emulator-specific
 * bits are reached through tiny extern "C" accessors added to pcfx.cpp
 * (PCFX_GetRAM / PCFX_GetRegs / PCFX_ControlReset), and the framebuffer is
 * pushed in each frame from the frontend hook (pcfx_control_frame).
 *
 * Enabled by setting MEDNAFEN_CONTROLPORT=<port> in the environment. */

#include "../extern/retro-remote-debug-controller/core/retro_control.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>

/* --- accessors implemented in pcfx/pcfx.cpp ------------------------------ */
extern "C" uint8_t *PCFX_GetRAM(uint32_t *size_out);   /* 2 MB main RAM */
extern "C" void     PCFX_GetRegs(uint32_t *out33);     /* [0]=pc, [1..32]=r0..r31 */
extern "C" void     PCFX_ControlReset(void);           /* MDFNI_Reset() */
/* --- pad-button injection, implemented in pcfx/input.cpp ----------------- */
extern "C" int      PCFX_InjectButton(unsigned bit, int action);  /* 0=tap 1=dn 2=up */
/* --- level-held virtual pad (0.5), implemented in pcfx/input.cpp --------- */
extern "C" int      PCFX_SetPad(int index, unsigned buttons, int connected);
extern "C" int      PCFX_GetPad(int index, unsigned *buttons, int *connected);

/* --- captured framebuffer (filled by the frontend hook each frame) ------- *
 * Stored pre-converted to packed RGB888 so /screenshot is correct regardless
 * of Mednafen's (display-dependent) surface channel order — the frame hook
 * decodes each pixel with the surface's R/G/B shifts.
 *
 * The PC-FX changes horizontal resolution per scene, and Mednafen reports the
 * TRUE per-line width in espec.LineWidths[] (256 for the maze, up to 1024 for
 * high-res text like the attract screen) — which can disagree with
 * DisplayRect.w. So the hook normalizes every line to a common width using the
 * per-line widths, exactly as Mednafen's own blitter does. FB_MAX_W must cover
 * the widest mode (1024) or the image is clipped. */
#define FB_MAX_W 1024
#define FB_MAX_H 256
static uint8_t  g_fb[FB_MAX_W * FB_MAX_H * 3];
static int      g_fb_w = 0, g_fb_h = 0;
static uint64_t g_frames = 0;

/* --- captured audio ring (single-threaded: producer and consumer are both
 * the emulator thread — the push hook after Emulate and capture_audio inside
 * retro_control_service() at the top of the loop — so no locking needed). --- */
#define AUD_RING 65536                 /* int16 samples (~0.37 s of stereo@44k) */
static int16_t  g_aud[AUD_RING];
static uint32_t g_aud_head = 0, g_aud_count = 0, g_aud_dropped = 0;
static int      g_aud_rate = 0, g_aud_chans = 2;

/* --- backend callbacks --------------------------------------------------- */
static uint32_t pcfx_read_mem(uint32_t addr, int32_t bank, uint32_t len,
                              uint8_t *out, uint32_t cap)
{
    uint32_t size = 0, n = 0;
    uint8_t *ram = PCFX_GetRAM(&size);
    (void)bank;
    if (!ram || !size) return 0;
    for (; n < len && n < cap; n++)
        out[n] = ram[(addr + n) & (size - 1)];   /* wrap in the 2 MB space */
    return n;
}

/* 0.3: write len bytes at addr into main RAM (debug poke). Mirrors read_mem;
 * writes through the same writable 2 MB pointer PCFX_GetRAM hands back. */
static uint32_t pcfx_write_mem(uint32_t addr, int32_t bank, uint32_t len,
                               const uint8_t *in)
{
    uint32_t size = 0, n = 0;
    uint8_t *ram = PCFX_GetRAM(&size);
    (void)bank;
    if (!ram || !size) return 0;
    for (; n < len; n++)
        ram[(addr + n) & (size - 1)] = in[n];   /* wrap in the 2 MB space */
    return n;
}

static void pcfx_get_regs_json(char *buf, size_t cap)
{
    uint32_t r[33];
    int off, i;
    PCFX_GetRegs(r);
    off = snprintf(buf, cap, "{\"pc\":%u", (unsigned)r[0]);
    for (i = 0; i < 32 && off > 0 && (size_t)off < cap; i++)
        off += snprintf(buf + off, cap - off, ",\"r%d\":%u", i, (unsigned)r[1 + i]);
    if (off > 0 && (size_t)off < cap)
        snprintf(buf + off, cap - off, "}");
}

static void pcfx_get_framebuffer(retro_framebuffer_t *out)
{
    out->pixels = g_fb;                  /* already packed RGB888 */
    out->width  = g_fb_w;
    out->height = g_fb_h;
    out->fmt    = RETRO_PIX_RGB888;
}

static uint64_t pcfx_get_frame_count(void) { return g_frames; }

static void pcfx_reset(void) { PCFX_ControlReset(); }

/* 0.2: inject a pad button. The PC-FX has a gamepad, not a keyboard, so both a
 * character (is_text=1) and a raw code (is_text=0) are interpreted as a
 * character over a WASD-style pad map, then routed to the pad bit. The bit
 * numbers are the PC-FX pad's data-buffer layout — the BitOffsets declared in
 * pcfx/input/gamepad.cpp (PCFX_GamepadIDII) and read back verbatim by the
 * device (Frame() does buttons = de16lsb(data); Read() returns it): UP=0 DOWN=1
 * LEFT=2 RIGHT=3 SELECT=4 RUN=5 IV=6 V=7 VI=8 III=9 II=10 I=11. (An earlier map
 * here used I=0…SELECT=6, which set the wrong bits — 'c' hit IV, not SELECT.)
 * Unmapped chars return -1 (the server answers 400). */
static int pcfx_char_to_bit(uint32_t c)
{
    switch (c) {
    case 'w': case 'W': return 0;    /* UP     */
    case 's': case 'S': return 1;    /* DOWN   */
    case 'a': case 'A': return 2;    /* LEFT   */
    case 'd': case 'D': return 3;    /* RIGHT  */
    case 'c': case 'C': return 4;    /* SELECT (coin)  */
    case ' ': case '\r': case '\n': return 5;  /* RUN (start) */
    case '1': return 11;             /* I   */
    case '2': return 10;             /* II  */
    case '3': return 9;              /* III */
    case '4': return 6;              /* IV  */
    case '5': return 7;              /* V   */
    case '6': return 8;              /* VI  */
    default:  return -1;
    }
}

static int pcfx_inject_key(int is_text, uint32_t value, int action)
{
    (void)is_text;                              /* pad-only: both are chars */
    int bit = pcfx_char_to_bit(value);
    if (bit < 0) return 0;                       /* unmapped -> 400 */
    return PCFX_InjectButton((unsigned)bit, action);
}

/* 0.3: drain interleaved int16 samples captured since the last call. */
static uint32_t pcfx_capture_audio(int16_t *out, uint32_t cap,
                                   int *rate, int *channels, uint32_t *dropped)
{
    uint32_t n = 0;
    if (rate)     *rate     = g_aud_rate ? g_aud_rate : 44100;
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

/* 0.5: virtual game controller. RRDC speaks a fixed CANONICAL button mask
 * (bit0 LEFT 1 RIGHT 2 UP 3 DOWN 4 A 5 B 6 X 7 Y 8 START 9 SELECT 10 L 11 R —
 * the SNES/Neo6502 order, identical on every platform) so one /pad call means
 * the same thing everywhere. We remap it to the PC-FX pad's own data-buffer
 * bits (see pcfx_char_to_bit): the d-pad maps straight across; A/B → the
 * primary pair I/II; X/Y → III/IV; L/R → V/VI; START → RUN; SELECT → SELECT.
 * pcfx/input.cpp holds the remapped mask as a LEVEL and OR-merges it into the
 * pad read every frame — the real read path, no SDL (headless CI has no
 * joystick, which is the case this exists for). */
static const int8_t pcfx_pad_from_canon[12] = {
    /*[0]  LEFT  */  2,
    /*[1]  RIGHT */  3,
    /*[2]  UP    */  0,
    /*[3]  DOWN  */  1,
    /*[4]  A     */ 11,   /* I   */
    /*[5]  B     */ 10,   /* II  */
    /*[6]  X     */  9,   /* III */
    /*[7]  Y     */  6,   /* IV  */
    /*[8]  START */  5,   /* RUN */
    /*[9]  SELECT*/  4,   /* SELECT */
    /*[10] L     */  7,   /* V   */
    /*[11] R     */  8,   /* VI  */
};

static unsigned pcfx_mask_from_canon(int canon)
{
    unsigned out = 0;
    for (int b = 0; b < 12; b++)
        if (canon & (1 << b)) out |= (1u << pcfx_pad_from_canon[b]);
    return out;
}

static int pcfx_canon_from_mask(unsigned raw)
{
    int out = 0;
    for (int b = 0; b < 12; b++)
        if (raw & (1u << pcfx_pad_from_canon[b])) out |= (1 << b);
    return out;
}

static int pcfx_set_pad(int index, int buttons, int connected)
{
    unsigned cur = 0; int curconn = 0;
    if (!PCFX_GetPad(index, &cur, &curconn)) return 0;   /* bad index -> 400 */
    unsigned mask = (buttons   < 0) ? cur     : pcfx_mask_from_canon(buttons);
    int      conn = (connected < 0) ? curconn : connected;   /* -1 = leave as-is */
    return PCFX_SetPad(index, mask, conn);
}

static int pcfx_get_pad(int index, int *buttons, int *connected)
{
    unsigned raw = 0; int conn = 0;
    if (!PCFX_GetPad(index, &raw, &conn)) return 0;
    if (buttons)   *buttons   = pcfx_canon_from_mask(raw);
    if (connected) *connected = conn;
    return 1;
}

/* The PC-FX is a gamepad console with no built-in pointer, but 0.5.0 advertises
 * CUMULATIVELY (it requires the 0.4.0 pointer verbs to be present). Honest
 * no-pointer stubs let /status report 0.5.0 while /pointer answers 400. */
static int pcfx_set_pointer(int absolute, int32_t x, int32_t y, int buttons)
{
    (void)absolute; (void)x; (void)y; (void)buttons;
    return 0;   /* no pointing device -> 400 */
}

static int pcfx_get_pointer(int32_t *x, int32_t *y, int *buttons)
{
    (void)x; (void)y; (void)buttons;
    return 0;   /* no pointing device -> 400 */
}

static const retro_control_backend_t pcfx_backend = {
    /*platform*/       "pcfx",
    /*emulator*/       "mednafen",
    /*read_mem*/       pcfx_read_mem,
    /*get_regs_json*/  pcfx_get_regs_json,
    /*get_framebuffer*/pcfx_get_framebuffer,
    /*get_frame_count*/pcfx_get_frame_count,
    /*inject_key*/     pcfx_inject_key,
    /*reset*/          pcfx_reset,
    /*write_mem*/      pcfx_write_mem,
    /*capture_audio*/  pcfx_capture_audio,
    /*set_pointer*/    pcfx_set_pointer,
    /*get_pointer*/    pcfx_get_pointer,
    /*set_pad*/        pcfx_set_pad,
    /*get_pad*/        pcfx_get_pad,
};

/* --- frontend hooks (called from src/drivers/main.cpp) ------------------- *
 * The GameLoop drives the portable core cooperatively: once per iteration it
 * services a pending request (pcfx_control_service) at a frame boundary, gates
 * advancing the machine on pcfx_control_running (so /pause and /step actually
 * halt it), and after each emulated frame pushes pixels + audio and ticks the
 * step budget (pcfx_control_on_frame). All of this runs on the emulator
 * thread, which is why the framebuffer and audio ring need no locking. */
extern "C" void pcfx_control_init(void)
{
    static int started = 0;
    if (started) return;
    started = 1;
    const char *p = getenv("MEDNAFEN_CONTROLPORT");
    if (p && *p)
        retro_control_start(atoi(p), &pcfx_backend);
}

/* Process one pending control request; call once per loop iteration, even when
 * paused (so /resume and /step can be received while the machine is halted). */
extern "C" void pcfx_control_service(void)  { retro_control_service(); }

/* Nonzero => advance the machine this iteration (free-run or steps remaining). */
extern "C" int  pcfx_control_running(void)  { return retro_control_running(); }

/* Tick the /step frame budget; call once per completed emulated frame. */
extern "C" void pcfx_control_on_frame(void) { retro_control_on_frame(); }

/* Copy this frame's displayed pixels, decoding each 32-bit surface pixel to
 * packed RGB888 with the surface's channel shifts (rsh/gsh/bsh = bit position
 * of each component's low bit; 8-bit precision). Called once per emulated frame.
 *
 * `line_widths[y]` is the TRUE pixel width of display line y (NULL => uniform,
 * use `w` = DisplayRect.w). The PC-FX varies horizontal resolution per scene
 * and DisplayRect.w can disagree with the real per-line width, so each line is
 * nearest-neighbour stretched from its own width to the frame's widest line —
 * the same normalization Mednafen's blitter applies before display. Without
 * this a high-res (1024) attract screen was captured as a clipped 256/512 slice
 * shifted off-centre. (Servicing happens at the top of the loop.) */
extern "C" void pcfx_control_frame(const uint32_t *pixels, int w, int h, int pitch,
                                   const int32_t *line_widths, int rsh, int gsh, int bsh)
{
    if (pixels && w > 0 && h > 0) {
        if (h > FB_MAX_H) h = FB_MAX_H;
        /* output width = widest true line width this frame, capped to the buffer */
        int W = w;
        for (int y = 0; y < h; y++) {
            int lw = line_widths ? (int)line_widths[y] : w;
            if (lw > W) W = lw;
        }
        if (W > FB_MAX_W) W = FB_MAX_W;
        for (int y = 0; y < h; y++) {
            int lw = line_widths ? (int)line_widths[y] : w;
            if (lw <= 0) lw = w;                       /* blank/border line */
            const uint32_t *row = &pixels[(size_t)y * pitch];
            uint8_t *o = &g_fb[(size_t)y * W * 3];
            for (int x = 0; x < W; x++) {
                uint32_t px = row[(size_t)x * lw / W]; /* stretch lw -> W */
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

/* Push this frame's synthesised audio into the drain ring (interleaved int16,
 * `frames` sample-frames of `channels`). Overflow drops newest, counted. */
extern "C" void pcfx_control_audio(const int16_t *samples, int frames,
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
