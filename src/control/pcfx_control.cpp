/* PC-FX control backend for the Retro Remote Debug Controller.
 *
 * Maps the portable RRDC backend (control/retro_control.h) onto Mednafen's
 * PC-FX core. To stay free of Mednafen's C++ headers, the emulator-specific
 * bits are reached through tiny extern "C" accessors added to pcfx.cpp
 * (PCFX_GetRAM / PCFX_GetRegs / PCFX_ControlReset), and the framebuffer is
 * pushed in each frame from the frontend hook (pcfx_control_frame).
 *
 * Enabled by setting MEDNAFEN_CONTROLPORT=<port> in the environment. */

#include "retro_control.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>

/* --- accessors implemented in pcfx/pcfx.cpp ------------------------------ */
extern "C" uint8_t *PCFX_GetRAM(uint32_t *size_out);   /* 2 MB main RAM */
extern "C" void     PCFX_GetRegs(uint32_t *out33);     /* [0]=pc, [1..32]=r0..r31 */
extern "C" void     PCFX_ControlReset(void);           /* MDFNI_Reset() */

/* --- captured framebuffer (filled by the frontend hook each frame) ------- */
#define FB_MAX_W 512
#define FB_MAX_H 256
static uint32_t g_fb[FB_MAX_W * FB_MAX_H];
static int      g_fb_w = 0, g_fb_h = 0;
static uint64_t g_frames = 0;

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
    out->pixels = (const uint8_t *)g_fb;
    out->width  = g_fb_w;
    out->height = g_fb_h;
    /* Mednafen surfaces are 32-bit; the exact channel order is refined once we
     * eyeball a /screenshot. XRGB8888 is the common Mednafen layout. */
    out->fmt = RETRO_PIX_BGRA8888;
}

static uint64_t pcfx_get_frame_count(void) { return g_frames; }

static void pcfx_reset(void) { PCFX_ControlReset(); }

static const retro_control_backend_t pcfx_backend = {
    /*platform*/       "pcfx",
    /*emulator*/       "mednafen",
    /*read_mem*/       pcfx_read_mem,
    /*get_regs_json*/  pcfx_get_regs_json,
    /*get_framebuffer*/pcfx_get_framebuffer,
    /*get_frame_count*/pcfx_get_frame_count,
    /*inject_key*/     0,
    /*reset*/          pcfx_reset,
    /*write_mem*/      0,
    /*capture_audio*/  0,
};

/* --- frontend hooks (called from src/drivers/main.cpp) ------------------- */
extern "C" void pcfx_control_init(void)
{
    static int started = 0;
    if (started) return;
    started = 1;
    const char *p = getenv("MEDNAFEN_CONTROLPORT");
    if (p && *p)
        retro_control_start(atoi(p), &pcfx_backend);
}

/* Copy this frame's displayed pixels + service any pending control request.
 * Called once per emulated frame with the surface pixels + display size. */
extern "C" void pcfx_control_frame(const uint32_t *pixels, int w, int h, int pitch)
{
    int y;
    if (pixels && w > 0 && h > 0) {
        if (w > FB_MAX_W) w = FB_MAX_W;
        if (h > FB_MAX_H) h = FB_MAX_H;
        for (y = 0; y < h; y++)
            memcpy(&g_fb[y * w], &pixels[y * pitch], (size_t)w * 4);
        g_fb_w = w;
        g_fb_h = h;
    }
    g_frames++;
    retro_control_service();
}
