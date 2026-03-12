/* dma-proxy.h  — shared by kernel driver and userspace tests
 *
 * Keeps the original proxy layout and ioctls, but uses portable types so it
 * compiles cleanly in userspace (stdint.h) and in kernel (__KERNEL__).
 */

#ifndef DMA_PROXY_H
#define DMA_PROXY_H
#define SET_BUFFER_SIZE  _IOW('d', 0x10, unsigned long)  // userspace passes size in bytes

/* ---------- Types & IOCTL headers for both worlds ---------- */
#ifdef __KERNEL__
  #include <linux/types.h>
  #include <linux/ioctl.h>
  typedef __u32  dp_u32;
  typedef __u64  dp_u64;
  typedef __s32  dp_i32;
#else
  #include <stdint.h>
  #include <sys/ioctl.h>
  typedef uint32_t dp_u32;
  typedef uint64_t dp_u64;
  typedef int32_t  dp_i32;
#endif

/* ---------- Ring / buffer sizes (must match driver) ---------- */
#ifndef BUFFER_SIZE
# define BUFFER_SIZE (128 * 1024)   /* payload bytes per buffer (driver must match) */
#endif

#ifndef BUFFER_COUNT
# define BUFFER_COUNT 32            /* driver-allocated buffers in the ring */
#endif

/* App-side hints (<= driver values) */
#ifndef TX_BUFFER_COUNT
# define TX_BUFFER_COUNT 1
#endif
#ifndef RX_BUFFER_COUNT
# define RX_BUFFER_COUNT 32
#endif
#ifndef BUFFER_INCREMENT
# define BUFFER_INCREMENT 1
#endif

/* For apps that refer to this symbol */
#ifndef DMA_PROXY_BUFFER_DATA_BYTES
# define DMA_PROXY_BUFFER_DATA_BYTES BUFFER_SIZE
#endif

/* ---------- Classic proxy ioctls (single-buffer start/finish) ---------- */
#define FINISH_XFER _IOW('a','a', dp_i32*)
#define START_XFER  _IOW('a','b', dp_i32*)
#define XFER        _IOR('a','c', dp_i32*)

/* ---------- Streaming ioctls (optional; driver may implement) ---------- */
#ifndef PROXY_STREAMING_API
#define PROXY_STREAMING_API 1

struct proxy_stream_cfg {
    dp_u32 frame_bytes;        /* <= BUFFER_SIZE; 0 => BUFFER_SIZE */
    dp_u32 frames_per_round;   /* 0 => BUFFER_COUNT */
    dp_u64 frames_goal;        /* 0 => infinite */
};

struct proxy_stream_state {
    dp_u32 running;            /* 1 while active, 0 when stopped/completed */
    dp_u32 frame_bytes;
    dp_u32 frames_per_round;
    dp_u32 _pad;
    dp_u64 frames_done;        /* total frames produced into ring */
    dp_u64 frames_goal;        /* 0 if infinite */
};

#define PROXY_STREAM_START _IOW('d', 0x10, struct proxy_stream_cfg)
#define PROXY_STREAM_STOP  _IO('d',  0x11)
#define PROXY_STREAM_GET   _IOR('d', 0x12, struct proxy_stream_state)
#endif /* PROXY_STREAMING_API */

/* ---------- Channel buffer shared layout ---------- */
struct channel_buffer {
    /* Keep buffer first so it’s naturally aligned for DMA */
    unsigned int buffer[BUFFER_SIZE / sizeof(unsigned int)];

    enum proxy_status {
        PROXY_NO_ERROR = 0,
        PROXY_BUSY     = 1,
        PROXY_TIMEOUT  = 2,
        PROXY_ERROR    = 3
    } status;

    unsigned int length;   /* payload length for this transfer */
} __attribute__ ((aligned (1024)));  /* generous alignment (64B is minimum) */

#endif /* DMA_PROXY_H */


