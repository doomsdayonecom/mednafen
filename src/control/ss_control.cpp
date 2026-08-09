/* Sega Saturn control backend for the Retro Remote Debug Controller.
 *
 * Same shape as the PC-FX backend it was modelled on: the portable core under
 * src/extern/retro-remote-debug-controller, driven by the frontend hooks in
 * src/drivers/main.cpp, with the emulator-specific parts behind extern "C"
 * accessors so this file needs none of Mednafen's C++ headers.
 *
 * Two things differ from the PC-FX, and both are the Saturn being the Saturn:
 *
 *  - MEMORY IS NOT ONE FLAT BLOCK. There are two work RAM areas (low at
 *    0x00200000, high at 0x06000000) plus the BIOS, and each is reachable
 *    through both a cached and an uncached view of the same bytes. So /mem
 *    takes a real Saturn address and SS_ControlReadMem decides what it lands
 *    in, rather than masking into a single array. It also means the ADDRESS
 *    YOU ASK FOR IS THE ADDRESS THE PROGRAM SEES, which is what makes this
 *    usable for debugging a floor that talks to VDP2 through 0x25F80000.
 *
 *  - REGISTERS ARE THE MASTER SH-2's ONLY. The Saturn has two, and exposing
 *    both in one JSON object would invite reading the wrong one; this floor
 *    runs the master.
 *
 * Enabled by setting MEDNAFEN_CONTROLPORT=<port> in the environment. */

#include "../extern/retro-remote-debug-controller/core/retro_control.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>

/* --- accessors implemented in ss/ss.cpp ---------------------------------- */
extern "C" uint32_t SS_ControlReadMem(uint32_t addr, uint32_t len, uint8_t *out);
extern "C" uint32_t SS_ControlWriteMem(uint32_t addr, uint32_t len, const uint8_t *in);
extern "C" void     SS_ControlGetRegs(uint32_t *out17); /* [0]=pc, [1..16]=r0..r15 */
extern "C" void     SS_ControlReset(void);

/* --- captured framebuffer (filled by the frontend hook each frame) ------- *
 * Stored pre-converted to packed RGB888 so /screenshot is correct regardless
 * of Mednafen's (display-dependent) surface channel order — the frame hook
 * decodes each pixel with the surface's R/G/B shifts.
 *
 * The Saturn's horizontal resolution is a mode, not a per-scene decision, so
 * the per-line normalisation inherited from the PC-FX is usually a no-op here.
 * It is kept anyway: Mednafen still reports per-line widths, the hi-res and
 * interlaced modes exist, and a capture that silently clipped them would be a
 * worse bug than a loop that mostly does nothing. FB_MAX_H covers 512-line
 * interlace; FB_MAX_W covers the 704-pixel hi-res modes with room to spare. */
#define FB_MAX_W 1024
#define FB_MAX_H 512
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
static uint32_t ss_read_mem(uint32_t addr, int32_t bank, uint32_t len,
                            uint8_t *out, uint32_t cap)
{
    (void)bank;
    if (len > cap) len = cap;
    /* A real Saturn address, not an offset into one array: work RAM low, work
     * RAM high and the BIOS all answer, cached or uncached. An address in none
     * of them yields a SHORT read rather than a wrapped one — reading hardware
     * registers back is not something this can honestly do, and pretending
     * otherwise would hand back plausible nonsense. */
    return SS_ControlReadMem(addr, len, out);
}

/* 0.3: write len bytes at addr (debug poke). Work RAM only — the BIOS and the
 * hardware registers refuse, and say so by returning a short count. */
static uint32_t ss_write_mem(uint32_t addr, int32_t bank, uint32_t len,
                             const uint8_t *in)
{
    (void)bank;
    return SS_ControlWriteMem(addr, len, in);
}

static void ss_get_regs_json(char *buf, size_t cap)
{
    uint32_t r[17];
    int off, i;
    SS_ControlGetRegs(r);
    off = snprintf(buf, cap, "{\"pc\":%u", (unsigned)r[0]);
    for (i = 0; i < 16 && off > 0 && (size_t)off < cap; i++)
        off += snprintf(buf + off, cap - off, ",\"r%d\":%u", i, (unsigned)r[1 + i]);
    if (off > 0 && (size_t)off < cap)
        snprintf(buf + off, cap - off, "}");
}

static void ss_get_framebuffer(retro_framebuffer_t *out)
{
    out->pixels = g_fb;                  /* already packed RGB888 */
    out->width  = g_fb_w;
    out->height = g_fb_h;
    out->fmt    = RETRO_PIX_RGB888;
}

static uint64_t ss_get_frame_count(void) { return g_frames; }

static void ss_reset(void) { SS_ControlReset(); }

/* 0.2: inject a pad button. The PC-FX has a gamepad, not a keyboard, so both a
 * character (is_text=1) and a raw code (is_text=0) are interpreted as a
 * character over a WASD-style pad map, then routed to the pad bit.
 *
 * The bit numbers are the PC-FX pad word as the guest reads it:
 *   I=0 II=1 III=2 IV=3 V=4 VI=5 SELECT=6 RUN=7 UP=8 RIGHT=9 DOWN=10 LEFT=11
 * We inject by OR-ing into data_ptr[], the same buffer the device's Frame()
 * consumes verbatim (buttons = de16lsb(data); Read() returns it), so these ARE
 * the guest-visible bits — the same ones liberis' fxpad/spr7up examples decode
 * on real hardware and the same ones pac-man-fx's PAD_* constants use.
 *
 * TRAP: this layout is implicit in the ORDER of PCFX_GamepadIDII in
 * pcfx/input/gamepad.cpp — git.cpp walks that array assigning
 * BitOffset += BitSize. The third IDIIS_Button argument is ConfigOrder (the
 * input-config prompt order), NOT a bit offset. Reading those numbers as bits
 * gives the wrong map (UP=0 DOWN=1 … SELECT=4), which silently sends 'c' to V
 * instead of SELECT so no credit is ever deposited. Do not "fix" this back.
 *
 * Unmapped chars return -1 (the server answers 400). */
static int ss_char_to_bit(uint32_t c)
{
    switch (c) {
    case 'w': case 'W': return 8;    /* UP     */
    case 's': case 'S': return 10;   /* DOWN   */
    case 'a': case 'A': return 11;   /* LEFT   */
    case 'd': case 'D': return 9;    /* RIGHT  */
    case 'c': case 'C': return 6;    /* SELECT (coin)  */
    case ' ': case '\r': case '\n': return 7;  /* RUN (start) */
    case '1': return 0;              /* I   */
    case '2': return 1;              /* II  */
    case '3': return 2;              /* III */
    case '4': return 3;              /* IV  */
    case '5': return 4;              /* V   */
    case '6': return 5;              /* VI  */
    default:  return -1;
    }
}

static int ss_inject_key(int is_text, uint32_t value, int action)
{
    (void)is_text; (void)value; (void)action;
    return 0;   /* SMPC pad injection not wired yet -> 400, honestly */
}

/* 0.3: drain interleaved int16 samples captured since the last call. */
static uint32_t ss_capture_audio(int16_t *out, uint32_t cap,
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
 * bits (see ss_char_to_bit): the d-pad maps straight across; A/B → the
 * primary pair I/II; X/Y → III/IV; L/R → V/VI; START → RUN; SELECT → SELECT.
 * pcfx/input.cpp holds the remapped mask as a LEVEL and OR-merges it into the
 * pad read every frame — the real read path, no SDL (headless CI has no
 * joystick, which is the case this exists for). */
static const int8_t ss_pad_from_canon[12] = {
    /*[0]  LEFT  */ 11,
    /*[1]  RIGHT */  9,
    /*[2]  UP    */  8,
    /*[3]  DOWN  */ 10,
    /*[4]  A     */  0,   /* I   */
    /*[5]  B     */  1,   /* II  */
    /*[6]  X     */  2,   /* III */
    /*[7]  Y     */  3,   /* IV  */
    /*[8]  START */  7,   /* RUN */
    /*[9]  SELECT*/  6,   /* SELECT */
    /*[10] L     */  4,   /* V   */
    /*[11] R     */  5,   /* VI  */
};

static unsigned ss_mask_from_canon(int canon)
{
    unsigned out = 0;
    for (int b = 0; b < 12; b++)
        if (canon & (1 << b)) out |= (1u << ss_pad_from_canon[b]);
    return out;
}

static int ss_canon_from_mask(unsigned raw)
{
    int out = 0;
    for (int b = 0; b < 12; b++)
        if (raw & (1u << ss_pad_from_canon[b])) out |= (1 << b);
    return out;
}

static int ss_set_pad(int index, int buttons, int connected)
{
    (void)index; (void)buttons; (void)connected;
    return 0;   /* no virtual pad on this floor yet -> 400 */
}

static int ss_get_pad(int index, int *buttons, int *connected)
{
    (void)index; (void)buttons; (void)connected;
    return 0;   /* no virtual pad on this floor yet -> 400 */
}

/* The PC-FX is a gamepad console with no built-in pointer, but 0.5.0 advertises
 * CUMULATIVELY (it requires the 0.4.0 pointer verbs to be present). Honest
 * no-pointer stubs let /status report 0.5.0 while /pointer answers 400. */
static int ss_set_pointer(int absolute, int32_t x, int32_t y, int buttons)
{
    (void)absolute; (void)x; (void)y; (void)buttons;
    return 0;   /* no pointing device -> 400 */
}

static int ss_get_pointer(int32_t *x, int32_t *y, int *buttons)
{
    (void)x; (void)y; (void)buttons;
    return 0;   /* no pointing device -> 400 */
}

static const retro_control_backend_t ss_backend = {
    /*platform*/       "saturn",
    /*emulator*/       "mednafen",
    /*read_mem*/       ss_read_mem,
    /*get_regs_json*/  ss_get_regs_json,
    /*get_framebuffer*/ss_get_framebuffer,
    /*get_frame_count*/ss_get_frame_count,
    /*inject_key*/     ss_inject_key,
    /*reset*/          ss_reset,
    /*write_mem*/      ss_write_mem,
    /*capture_audio*/  ss_capture_audio,
    /*set_pointer*/    ss_set_pointer,
    /*get_pointer*/    ss_get_pointer,
    /*set_pad*/        ss_set_pad,
    /*get_pad*/        ss_get_pad,
};

/* --- frontend hooks (called from src/drivers/main.cpp) ------------------- *
 * The GameLoop drives the portable core cooperatively: once per iteration it
 * services a pending request (ss_control_service) at a frame boundary, gates
 * advancing the machine on ss_control_running (so /pause and /step actually
 * halt it), and after each emulated frame pushes pixels + audio and ticks the
 * step budget (ss_control_on_frame). All of this runs on the emulator
 * thread, which is why the framebuffer and audio ring need no locking. */
extern "C" void ss_control_init(void)
{
    static int started = 0;
    if (started) return;
    started = 1;
    const char *p = getenv("MEDNAFEN_CONTROLPORT");
    if (p && *p)
        retro_control_start(atoi(p), &ss_backend);
}

/* Process one pending control request; call once per loop iteration, even when
 * paused (so /resume and /step can be received while the machine is halted). */
extern "C" void ss_control_service(void)  { retro_control_service(); }

/* Nonzero => advance the machine this iteration (free-run or steps remaining). */
extern "C" int  ss_control_running(void)  { return retro_control_running(); }

/* Tick the /step frame budget; call once per completed emulated frame. */
extern "C" void ss_control_on_frame(void) { retro_control_on_frame(); }

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
extern "C" void ss_control_frame(const uint32_t *pixels, int w, int h, int pitch,
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
extern "C" void ss_control_audio(const int16_t *samples, int frames,
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
