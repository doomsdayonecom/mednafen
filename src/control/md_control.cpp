/* Sega Mega Drive / Genesis control backend for the Retro Remote Debug
 * Controller.
 *
 * Maps the portable RRDC backend (the vendored shared core under
 * src/extern/retro-remote-debug-controller) onto Mednafen's MD core. Mirrors
 * pce_control.cpp / pcfx_control.cpp: emulator-specific state is reached through
 * tiny extern "C" accessors added to md/genesis.cpp (MD_GetRAM / MD_GetRegs /
 * MD_ControlReset), and the framebuffer is pushed each frame from the frontend
 * hook (md_control_frame).
 *
 * Unlike the PC Engine, the MD VDP renders a clean active display, so the frame
 * hook takes the DisplayRect straight through: at the RGM floor's H40 (320 px) +
 * V30 (240-line, PAL) mode the DisplayRect is 320x240 and maps 1:1 to the
 * /screenshot output. If a build ever hands us a wider line (borders), each line
 * is centre-cropped to its middle 320 — the PCE trick, rarely needed here. Run
 * the MD in a PAL region so V30 yields the full 240 lines (see
 * test_md_screenshot.py); an NTSC 224-line frame clamps its last rows.
 *
 * Enabled by setting MEDNAFEN_CONTROLPORT=<port> in the environment. */

#include "../extern/retro-remote-debug-controller/core/retro_control.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>

/* --- accessors implemented in md/genesis.cpp ----------------------------- */
extern "C" uint8_t *MD_GetRAM(uint32_t *size_out);   /* 64 KB 68000 work RAM */
extern "C" void     MD_GetRegs(uint32_t *out18);     /* [0]=pc [1]=sr [2..9]=d0-7 [10..17]=a0-7 */
extern "C" void     MD_ControlReset(void);           /* gen_reset(true) */

/* --- captured framebuffer (filled by the frontend hook each frame) ------- *
 * Fixed 320x240, pre-converted to packed RGB888 so /screenshot is correct
 * regardless of Mednafen's surface channel order. */
#define OUT_W 320
#define OUT_H 240
static uint8_t  g_fb[OUT_W * OUT_H * 3];
static int      g_fb_w = 0, g_fb_h = 0;
static uint64_t g_frames = 0;

/* --- captured audio ring (single-threaded, as in pce_control.cpp) -------- */
#define AUD_RING 65536
static int16_t  g_aud[AUD_RING];
static uint32_t g_aud_head = 0, g_aud_count = 0, g_aud_dropped = 0;
static int      g_aud_rate = 0, g_aud_chans = 2;

/* --- backend callbacks --------------------------------------------------- */
static uint32_t md_read_mem(uint32_t addr, int32_t bank, uint32_t len,
                            uint8_t *out, uint32_t cap)
{
    uint32_t size = 0, n = 0;
    uint8_t *ram = MD_GetRAM(&size);
    (void)bank;
    if (!ram || !size) return 0;
    for (; n < len && n < cap; n++)
        out[n] = ram[(addr + n) & (size - 1)];   /* wrap in the work-RAM space */
    return n;
}

static uint32_t md_write_mem(uint32_t addr, int32_t bank, uint32_t len,
                             const uint8_t *in)
{
    uint32_t size = 0, n = 0;
    uint8_t *ram = MD_GetRAM(&size);
    (void)bank;
    if (!ram || !size) return 0;
    for (; n < len; n++)
        ram[(addr + n) & (size - 1)] = in[n];
    return n;
}

static void md_get_regs_json(char *buf, size_t cap)
{
    uint32_t r[18];
    MD_GetRegs(r);
    snprintf(buf, cap,
             "{\"pc\":%u,\"sr\":%u,"
             "\"d0\":%u,\"d1\":%u,\"d2\":%u,\"d3\":%u,"
             "\"d4\":%u,\"d5\":%u,\"d6\":%u,\"d7\":%u,"
             "\"a0\":%u,\"a1\":%u,\"a2\":%u,\"a3\":%u,"
             "\"a4\":%u,\"a5\":%u,\"a6\":%u,\"a7\":%u}",
             (unsigned)r[0], (unsigned)r[1],
             (unsigned)r[2], (unsigned)r[3], (unsigned)r[4], (unsigned)r[5],
             (unsigned)r[6], (unsigned)r[7], (unsigned)r[8], (unsigned)r[9],
             (unsigned)r[10], (unsigned)r[11], (unsigned)r[12], (unsigned)r[13],
             (unsigned)r[14], (unsigned)r[15], (unsigned)r[16], (unsigned)r[17]);
}

static void md_get_framebuffer(retro_framebuffer_t *out)
{
    out->pixels = g_fb;                  /* already packed RGB888, 320x240 */
    out->width  = g_fb_w;
    out->height = g_fb_h;
    out->fmt    = RETRO_PIX_RGB888;
}

static uint64_t md_get_frame_count(void) { return g_frames; }

static void md_reset(void) { MD_ControlReset(); }

/* Pad/pointer/key injection are not wired for the MD yet (the screenshot test
 * needs none of them); honest stubs keep /status advertising the cumulative
 * contract while those verbs answer 400. */
static int md_inject_key(int is_text, uint32_t value, int action)
{ (void)is_text; (void)value; (void)action; return 0; }
static int md_set_pad(int index, int buttons, int connected)
{ (void)index; (void)buttons; (void)connected; return 0; }
static int md_get_pad(int index, int *buttons, int *connected)
{ (void)index; (void)buttons; (void)connected; return 0; }
static int md_set_pointer(int absolute, int32_t x, int32_t y, int buttons)
{ (void)absolute; (void)x; (void)y; (void)buttons; return 0; }
static int md_get_pointer(int32_t *x, int32_t *y, int *buttons)
{ (void)x; (void)y; (void)buttons; return 0; }

static uint32_t md_capture_audio(int16_t *out, uint32_t cap,
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

static const retro_control_backend_t md_backend = {
    /*platform*/       "megadrive",
    /*emulator*/       "mednafen",
    /*read_mem*/       md_read_mem,
    /*get_regs_json*/  md_get_regs_json,
    /*get_framebuffer*/md_get_framebuffer,
    /*get_frame_count*/md_get_frame_count,
    /*inject_key*/     md_inject_key,
    /*reset*/          md_reset,
    /*write_mem*/      md_write_mem,
    /*capture_audio*/  md_capture_audio,
    /*set_pointer*/    md_set_pointer,
    /*get_pointer*/    md_get_pointer,
    /*set_pad*/        md_set_pad,
    /*get_pad*/        md_get_pad,
};

/* --- frontend hooks (called from src/drivers/main.cpp) ------------------- */
extern "C" void md_control_init(void)
{
    static int started = 0;
    if (started) return;
    started = 1;
    const char *p = getenv("MEDNAFEN_CONTROLPORT");
    if (p && *p)
        retro_control_start(atoi(p), &md_backend);
}

extern "C" void md_control_service(void)  { retro_control_service(); }
extern "C" int  md_control_running(void)  { return retro_control_running(); }
extern "C" void md_control_on_frame(void) { retro_control_on_frame(); }

/* Copy this frame's displayed pixels to a fixed 320x240, decoding each 32-bit
 * surface pixel to packed RGB888 with the surface's channel shifts. The MD's
 * H40+V30 DisplayRect is already 320x240, so the sample is 1:1; a wider line is
 * centre-cropped, and a short (NTSC 224-line) frame clamps its final rows. */
extern "C" void md_control_frame(const uint32_t *pixels, int w, int h, int pitch,
                                 const int32_t *line_widths, int rsh, int gsh, int bsh)
{
    if (pixels && w > 0 && h > 0) {
        for (int oy = 0; oy < OUT_H; oy++) {
            int sy = oy;
            if (sy >= h) sy = h - 1;
            int lw = (line_widths && line_widths[sy] > 0) ? (int)line_widths[sy] : w;
            int x0 = (lw > OUT_W) ? (lw - OUT_W) / 2 : 0;   /* centre if bordered */
            const uint32_t *row = &pixels[(size_t)sy * pitch];
            uint8_t *o = &g_fb[(size_t)oy * OUT_W * 3];
            for (int ox = 0; ox < OUT_W; ox++) {
                int sx = x0 + ox;
                if (sx >= pitch) sx = pitch - 1;
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

extern "C" void md_control_audio(const int16_t *samples, int frames,
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
