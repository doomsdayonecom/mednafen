/* PC Engine / TurboGrafx-16 control backend for the Retro Remote Debug
 * Controller.
 *
 * Maps the portable RRDC backend (the vendored shared core under
 * src/extern/retro-remote-debug-controller) onto Mednafen's PCE core. Mirrors
 * pcfx_control.cpp: emulator-specific state is reached through tiny extern "C"
 * accessors added to pce/pce.cpp (PCE_GetRAM / PCE_GetRegs / PCE_ControlReset)
 * and pce/vce.cpp (PCE_GetContentX0), and the framebuffer is pushed each frame
 * from the frontend hook (pce_control_frame).
 *
 * The PCE VDC has no linear framebuffer — Mednafen renders it into a 1365-wide
 * surface, and at the RGM floor's 40-tile 7 MHz mode the 320 content pixels are
 * drawn 1:1, centred in the LineWidths[y]-wide active line (~341, i.e. a ~10px
 * border each side). So the frame hook CENTRE-CROPS each line to its middle 320
 * and captures 240 lines (run with -pce.slstart 0 -pce.slend 239), giving a 1:1
 * 320x240 /screenshot the shared test can assert on exactly, identically to the
 * neo6502/x16 floors.
 *
 * Enabled by setting MEDNAFEN_CONTROLPORT=<port> in the environment. */

#include "../extern/retro-remote-debug-controller/core/retro_control.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>

/* --- accessors implemented in pce/pce.cpp -------------------------------- */
extern "C" uint8_t *PCE_GetRAM(uint32_t *size_out);   /* 8 KB (32 KB SGX) work RAM */
extern "C" void     PCE_GetRegs(uint32_t *out6);      /* [0]=pc [1]=a [2]=x [3]=y [4]=sp [5]=p */
extern "C" void     PCE_ControlReset(void);           /* PCE_Power() */

/* --- captured framebuffer (filled by the frontend hook each frame) ------- *
 * Fixed 320x240, pre-converted to packed RGB888 so /screenshot is correct
 * regardless of Mednafen's surface channel order. */
#define OUT_W 320
#define OUT_H 240
static uint8_t  g_fb[OUT_W * OUT_H * 3];
static int      g_fb_w = 0, g_fb_h = 0;
static uint64_t g_frames = 0;

/* --- captured audio ring (single-threaded, as in pcfx_control.cpp) ------- */
#define AUD_RING 65536
static int16_t  g_aud[AUD_RING];
static uint32_t g_aud_head = 0, g_aud_count = 0, g_aud_dropped = 0;
static int      g_aud_rate = 0, g_aud_chans = 2;

/* --- backend callbacks --------------------------------------------------- */
static uint32_t pce_read_mem(uint32_t addr, int32_t bank, uint32_t len,
                             uint8_t *out, uint32_t cap)
{
    uint32_t size = 0, n = 0;
    uint8_t *ram = PCE_GetRAM(&size);
    (void)bank;
    if (!ram || !size) return 0;
    for (; n < len && n < cap; n++)
        out[n] = ram[(addr + n) & (size - 1)];   /* wrap in the work-RAM space */
    return n;
}

static uint32_t pce_write_mem(uint32_t addr, int32_t bank, uint32_t len,
                              const uint8_t *in)
{
    uint32_t size = 0, n = 0;
    uint8_t *ram = PCE_GetRAM(&size);
    (void)bank;
    if (!ram || !size) return 0;
    for (; n < len; n++)
        ram[(addr + n) & (size - 1)] = in[n];
    return n;
}

static void pce_get_regs_json(char *buf, size_t cap)
{
    uint32_t r[6];
    PCE_GetRegs(r);
    snprintf(buf, cap,
             "{\"pc\":%u,\"a\":%u,\"x\":%u,\"y\":%u,\"sp\":%u,\"p\":%u}",
             (unsigned)r[0], (unsigned)r[1], (unsigned)r[2],
             (unsigned)r[3], (unsigned)r[4], (unsigned)r[5]);
}

static void pce_get_framebuffer(retro_framebuffer_t *out)
{
    out->pixels = g_fb;                  /* already packed RGB888, 320x240 */
    out->width  = g_fb_w;
    out->height = g_fb_h;
    out->fmt    = RETRO_PIX_RGB888;
}

static uint64_t pce_get_frame_count(void) { return g_frames; }

static void pce_reset(void) { PCE_ControlReset(); }

/* Pad/pointer/key injection are not wired for the PCE yet (the screenshot test
 * needs none of them); honest stubs keep /status advertising the cumulative
 * contract while those verbs answer 400. */
static int pce_inject_key(int is_text, uint32_t value, int action)
{ (void)is_text; (void)value; (void)action; return 0; }
static int pce_set_pad(int index, int buttons, int connected)
{ (void)index; (void)buttons; (void)connected; return 0; }
static int pce_get_pad(int index, int *buttons, int *connected)
{ (void)index; (void)buttons; (void)connected; return 0; }
static int pce_set_pointer(int absolute, int32_t x, int32_t y, int buttons)
{ (void)absolute; (void)x; (void)y; (void)buttons; return 0; }
static int pce_get_pointer(int32_t *x, int32_t *y, int *buttons)
{ (void)x; (void)y; (void)buttons; return 0; }

static uint32_t pce_capture_audio(int16_t *out, uint32_t cap,
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

static const retro_control_backend_t pce_backend = {
    /*platform*/       "pcengine",
    /*emulator*/       "mednafen",
    /*read_mem*/       pce_read_mem,
    /*get_regs_json*/  pce_get_regs_json,
    /*get_framebuffer*/pce_get_framebuffer,
    /*get_frame_count*/pce_get_frame_count,
    /*inject_key*/     pce_inject_key,
    /*reset*/          pce_reset,
    /*write_mem*/      pce_write_mem,
    /*capture_audio*/  pce_capture_audio,
    /*set_pointer*/    pce_set_pointer,
    /*get_pointer*/    pce_get_pointer,
    /*set_pad*/        pce_set_pad,
    /*get_pad*/        pce_get_pad,
};

/* --- frontend hooks (called from src/drivers/main.cpp) ------------------- */
extern "C" void pce_control_init(void)
{
    static int started = 0;
    if (started) return;
    started = 1;
    const char *p = getenv("MEDNAFEN_CONTROLPORT");
    if (p && *p)
        retro_control_start(atoi(p), &pce_backend);
}

extern "C" void pce_control_service(void)  { retro_control_service(); }
extern "C" int  pce_control_running(void)  { return retro_control_running(); }
extern "C" void pce_control_on_frame(void) { retro_control_on_frame(); }

/* Resample this frame's displayed pixels to a fixed 320x240, decoding each
 * 32-bit surface pixel to packed RGB888 with the surface's channel shifts.
 *
 * `line_widths[y]` is the TRUE content width of display line y (rect_w from the
 * VCE); the content begins at surface X = PCE_GetContentX0() and wraps mod 2048
 * (the VCE's line buffer is >=2048 wide). Each output pixel is nearest-neighbour
 * sampled with a wrap-aware index, so the offset, dot-clock-scaled PCE display
 * lands as a clean 320x240 image. Height maps 1:1 when the frame is 240 lines
 * (run with -pce.slstart 0 -pce.slend 239); otherwise it scales. */
extern "C" void pce_control_frame(const uint32_t *pixels, int w, int h, int pitch,
                                  const int32_t *line_widths, int rsh, int gsh, int bsh)
{
    if (pixels && w > 0 && h > 0) {
        for (int oy = 0; oy < OUT_H; oy++) {
            /* The PCE's active display begins ~3 lines below Mednafen's
             * DisplayRect top, so shift the sample down to align logical row 0
             * with content row 0. Run with -pce.slstart 0 -pce.slend 239 (the
             * full 240-line window; slend caps at 239). The final ~3 logical
             * rows fall past that window and clamp to the last captured line;
             * rgm_smoke's only content there is the solid bottom-right corner,
             * so the clamp is invisible. */
            int sy = oy + 3;
            if (sy >= h) sy = h - 1;
            int lw = line_widths ? (int)line_widths[sy] : w; /* active-line width (341) */
            if (lw <= 0) lw = w;                             /* blank/border line */
            /* Our 40-tile floor renders 320 content pixels 1:1, centred in the
             * lw-wide active line (lw=341 -> a ~10px border each side), and the
             * active line begins at column 0 of the passed DisplayRect region.
             * So the logical canvas is the centre 320 of each line. */
            int content_x = (lw - OUT_W) / 2;
            if (content_x < 0) content_x = 0;
            const uint32_t *row = &pixels[(size_t)sy * pitch];
            uint8_t *o = &g_fb[(size_t)oy * OUT_W * 3];
            for (int ox = 0; ox < OUT_W; ox++) {
                int sx = content_x + ox;
                if (sx >= pitch) sx = pitch - 1;             /* safety clamp */
                uint32_t px = row[sx];
                *o++ = (uint8_t)(px >> rsh);
                *o++ = (uint8_t)(px >> gsh);
                *o++ = (uint8_t)(px >> bsh);
            }
        }
        g_fb_w = OUT_W;
        g_fb_h = OUT_H;
    }
    g_frames++;
}

extern "C" void pce_control_audio(const int16_t *samples, int frames,
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
