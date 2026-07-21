/* retro_control.h — portable emulator control server (see ../SPEC.md).
 *
 * A retro-platform emulator implements the small read-only backend below and
 * calls retro_control_start(). The shared server (retro_control.c) owns the
 * socket loop, HTTP parsing, PPM/JSON encoding, and control flow
 * (pause / step / free-run). It always binds 127.0.0.1. Contract 0.1.0.
 *
 * Integration is three calls in the emulator's main loop:
 *   retro_control_service();                 // once per iteration (cheap)
 *   if (!retro_control_running()) { idle; }  // gate advancing the machine
 *   ... run one instruction / frame ...
 *   if (frame_completed) retro_control_on_frame();
 *
 * Threading: the server runs on its own thread but never touches machine state
 * there. State-reading endpoints (/mem, /regs, /screenshot) are executed inside
 * retro_control_service() ON THE EMULATOR THREAD, so every read is consistent.
 * The backend callbacks are therefore called from the emulator thread (except
 * get_frame_count, which the server may also read for /status — make it a plain
 * atomic-ish counter read).
 */
#ifndef RETRO_CONTROL_H
#define RETRO_CONTROL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Highest contract the core implements. The server advertises the highest level
 * whose backend callbacks are all present: 0.3.0 needs inject_key + write_mem +
 * capture_audio; 0.2.0 needs inject_key; else 0.1.0. Keeps a partial backend
 * honest. */
#define RETRO_CONTROL_CONTRACT "0.3.0"

/* Native framebuffer pixel layout; the server swizzles to PPM RGB. */
typedef enum {
    RETRO_PIX_RGB888 = 0,   /* 3 bytes/px: R,G,B                       */
    RETRO_PIX_RGBA8888,     /* 4 bytes/px: R,G,B,A                     */
    RETRO_PIX_BGRA8888      /* 4 bytes/px: B,G,R,A (x16 framebuffer)   */
} retro_pix_fmt_t;

typedef struct {
    const uint8_t *pixels;  /* start of the current complete frame     */
    int width;
    int height;
    retro_pix_fmt_t fmt;
} retro_framebuffer_t;

/* Key action for inject_key (0.2). */
typedef enum {
    RETRO_KEY_TAP  = 0,   /* press then release */
    RETRO_KEY_DOWN = 1,   /* press / hold       */
    RETRO_KEY_UP   = 2    /* release            */
} retro_key_action_t;

/* The emulator implements these. Any may be NULL: the matching endpoint then
 * returns 501 Not Implemented. */
typedef struct retro_control_backend {
    const char *platform;   /* "x16" | "neo6502" | "agon" | ...        */
    const char *emulator;   /* human name, e.g. "x16emu"               */

    /* Debug-read up to min(len, out_cap) bytes at addr into out, side-effect
     * free. bank is the optional bank selector (<0 = current bank). Returns the
     * number of bytes written. */
    uint32_t (*read_mem)(uint32_t addr, int32_t bank, uint32_t len,
                         uint8_t *out, uint32_t out_cap);

    /* Write a JSON object of registers into buf (NUL-terminated). */
    void (*get_regs_json)(char *buf, size_t cap);

    /* Report the current complete-frame framebuffer. */
    void (*get_framebuffer)(retro_framebuffer_t *out);

    /* Monotonic completed-frame counter. */
    uint64_t (*get_frame_count)(void);

    /* 0.2: inject a key. is_text=1 => `value` is a character code point;
     * is_text=0 => `value` is a raw platform key code. `action` is a
     * retro_key_action_t. Return 1 on success, 0 if unmappable (server -> 400).
     * NULL => /key returns 501 and the server advertises contract 0.1.0. */
    int (*inject_key)(int is_text, uint32_t value, int action);

    /* 0.2: soft/cold reset the machine. NULL => /reset returns 501. */
    void (*reset)(void);

    /* 0.3: write len bytes from `in` at addr (debug/state poke; bank<0 =
     * current). Intended for RAM/state — I/O-register writes may trigger device
     * side effects. Return bytes written. NULL => POST /mem returns 501. */
    uint32_t (*write_mem)(uint32_t addr, int32_t bank, uint32_t len,
                          const uint8_t *in);

    /* 0.3: drain up to `cap` interleaved signed-16-bit audio samples the
     * emulator has synthesised since the last call into `out`; set *rate and
     * *channels; set *dropped to samples lost to overflow since the last drain.
     * Return the number of int16 samples written. NULL => /audio returns 501. */
    uint32_t (*capture_audio)(int16_t *out, uint32_t cap,
                              int *rate, int *channels, uint32_t *dropped);
} retro_control_backend_t;

/* Start the server thread bound to 127.0.0.1:port. Non-blocking; 0 on success,
 * negative on error. backend must outlive the server. */
int  retro_control_start(int port, const retro_control_backend_t *backend);

/* Drain a pending control request. Call once per main-loop iteration, at an
 * instruction/frame boundary. Cheap no-op when idle and when control is off. */
void retro_control_service(void);

/* Nonzero => the machine should advance this iteration. Returns 1 always when
 * control is off or the machine is free-running; 0 when paused with no steps
 * left. */
int  retro_control_running(void);

/* Call when a video frame completes. Drives /step (decrements the step budget,
 * halts and answers when it reaches zero). No-op when control is off. */
void retro_control_on_frame(void);

/* Stop the server (optional; process exit is fine too). */
void retro_control_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* RETRO_CONTROL_H */
