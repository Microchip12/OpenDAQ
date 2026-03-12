// rx_dma_capture_parallel.c
// Optimized parallel capture from 3 DMA channels using START_ALL_CHANNELS
// Build: gcc -O2 -Wall -Wextra -std=gnu11 -pthread -o rx_test3 rx_test3.c


#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>
#include <pthread.h>


// ---- dma_proxy ioctls ----
#ifndef START_XFER
#define START_XFER      0x1001
#endif
#ifndef FINISH_XFER
#define FINISH_XFER     0x1002
#endif
#ifndef GET_PROD_IDX
#define GET_PROD_IDX    0x2001
#endif
#ifndef RESET_PROD_IDX
#define RESET_PROD_IDX  0x2002
#endif
#ifndef GET_STATS
#define GET_STATS       0x2004
#endif
#ifndef RESET_DMA
#define RESET_DMA       0x3001
#endif
#ifndef START_ALL_CHANNELS
#define START_ALL_CHANNELS 0x3002  // NEW: Synchronous start
#endif


struct dma_proxy_stats {
    uint32_t phys_lo;
    uint32_t last_da_lo;
    uint32_t last_dmasr;
    uint32_t prod_idx;
};


// ---- DMA channel configuration ----
#define MAX_DMA_CHANNELS 3
#define NUM_DMA_CHANNELS 3


static const char *DMA_PATHS[MAX_DMA_CHANNELS] = {
    "/dev/dma_proxy0",  // dma0
    "/dev/dma_proxy1",  // dma1
    "/dev/dma_proxy2"   // dma2
};


// User-configurable active channels
static int g_num_active_channels = 3;
static int g_active_channels[MAX_DMA_CHANNELS] = {0, 1, 2};  // Which DMAs to use


// ---- Per-channel state ----
typedef struct {
    int channel_id;
    const char *dev_path;
    int fd;
    void *ring;
    volatile uint32_t *ring_u32;
    size_t ring_bytes;
   
    // Capture buffers
    uint32_t *raw_data;
    size_t raw_written;
    size_t target_raw_words;
   
    // Processed data (after dedup, skip, zero removal)
    uint32_t *processed_data;
    size_t processed_count;
   
    // Thread control
    pthread_t thread;
    int thread_result;
   
    // Synchronization
    pthread_mutex_t *start_mutex;
    pthread_cond_t *start_cond;
    volatile int *start_flag;
   
    // Output files
    char bin_path[256];
} dma_channel_t;


// ---- Global config (shared across channels) ----
static size_t g_buffer_count = 1;
static size_t g_frame_bytes = 33554432;
static size_t g_batch_frames = 1;
static int g_verbose = 1;


static const char *COUNTER_FILE = "capture_counter.txt";
static int g_write_bin = 1;
static int g_write_csv = 1;
static int g_reset_dma_before_capture = 0;  // Reset DMAs before starting
static int g_no_confirm = 0;  // Skip confirmation prompt


// Transform knobs
static int g_msb_first = 0;
static int g_swap_bytes = 0;
static int g_rev_bits_in_byte = 0;


// Post-process controls
static int g_dedup_pairs = 0;
static int g_skip_first_word = 1;
static int g_drop_zeros = 1;
static size_t g_skip_bits = 1200000;
static size_t g_capture_multiplier = 1;
static size_t g_preview_words = 32;


// FPGA pipeline compensation offsets (in words)
// Adjust these based on your hardware
static size_t g_offset_dma1 = 0;  // Set to measured offset for DMA1
static size_t g_offset_dma2 = 0;  // Set to measured offset for DMA2


// ---- Helpers ----
static inline void cpu_barrier(void){
    __sync_synchronize();
    asm volatile("dsb sy" ::: "memory");
}


static inline uint32_t swap_bytes_u32(uint32_t v){
    return ((v & 0x000000FFu) << 24) |
           ((v & 0x0000FF00u) <<  8) |
           ((v & 0x00FF0000u) >>  8) |
           ((v & 0xFF000000u) >> 24);
}


static int get_next_capture_id(void) {
    int id = 0;
    FILE *f = fopen(COUNTER_FILE, "r+");
    if (f) {
        if (fscanf(f, "%d", &id) != 1) id = 0;
        rewind(f);
    } else {
        f = fopen(COUNTER_FILE, "w");
        if (!f) {
            perror("fopen(counter)");
            return 0;
        }
        id = 0;
    }
    fprintf(f, "%d\n", id + 1);
    fclose(f);
    return id;
}


static inline uint8_t reverse_bits8(uint8_t x){
    x = (uint8_t)(((x & 0x55u) << 1) | ((x & 0xAAu) >> 1));
    x = (uint8_t)(((x & 0x33u) << 2) | ((x & 0xCCu) >> 2));
    x = (uint8_t)(((x & 0x0Fu) << 4) | ((x & 0xF0u) >> 4));
    return x;
}


static inline uint32_t reverse_bits_each_byte_u32(uint32_t v){
    uint8_t b0 = reverse_bits8((uint8_t)(v));
    uint8_t b1 = reverse_bits8((uint8_t)(v >> 8));
    uint8_t b2 = reverse_bits8((uint8_t)(v >> 16));
    uint8_t b3 = reverse_bits8((uint8_t)(v >> 24));
    return ((uint32_t)b3 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b0;
}


static inline uint32_t apply_xforms(uint32_t w){
    if (g_swap_bytes) w = swap_bytes_u32(w);
    if (g_rev_bits_in_byte) w = reverse_bits_each_byte_u32(w);
    return w;
}


#ifdef DEBUG_FUNCTIONS
static void dump_preview(const char *prefix, const uint32_t *buf, size_t words){
    size_t show = (g_preview_words < words ? g_preview_words : words);
    if (!show) return;
    printf("%s: Preview first %zu words:\n", prefix, show);
    for (size_t i = 0; i < show; ++i){
        printf("  [%4zu] 0x%08x\n", i, buf[i]);
    }
}


static void dump_stats(int fd, const char *tag){
    struct dma_proxy_stats s;
    if (ioctl(fd, GET_STATS, (unsigned long)&s) == 0) {
        printf("[%s] stats: phys=0x%08x prod=%u lastDA=0x%08x DMASR=0x%08x\n",
               tag, s.phys_lo, s.prod_idx, s.last_da_lo, s.last_dmasr);
    } else {
        perror("GET_STATS");
    }
}
#endif


static size_t collapse_doublets(const uint32_t *in, size_t n, uint32_t *out){
    size_t k = 0, i = 0;
    while (i < n){
        uint32_t v0 = in[i];
        if (i + 1 < n && v0 == in[i + 1]) { out[k++] = v0; i += 2; }
        else { out[k++] = v0; i += 1; }
    }
    return k;
}


static int write_bin(const char *path, const uint32_t *buf, size_t words){
    if (!g_write_bin) return 0;
    FILE *f = fopen(path, "wb");
    if (!f){ return -1; }
    size_t wrote = fwrite(buf, sizeof(uint32_t), words, f);
    if (wrote != words){ fclose(f); return -1; }
    fclose(f);
    return 0;
}


// ---- DMA capture thread - waits for main thread to start DMA ----
static void* dma_capture_thread(void *arg) {
    dma_channel_t *ch = (dma_channel_t*)arg;
    const size_t frame_words = g_frame_bytes / 4;
   
    // NO PRINTS - must be fast!
   
    // Open device
    ch->fd = open(ch->dev_path, O_RDWR);
    if (ch->fd < 0) {
        perror("open dma device");
        ch->thread_result = -1;
        return NULL;
    }
   
    // Map ring buffer
    ch->ring_bytes = g_buffer_count * g_frame_bytes;
    ch->ring = mmap(NULL, ch->ring_bytes, PROT_READ|PROT_WRITE, MAP_SHARED, ch->fd, 0);
    if (ch->ring == MAP_FAILED) {
        perror("mmap ring");
        close(ch->fd);
        ch->thread_result = -1;
        return NULL;
    }
   
    ch->ring_u32 = (volatile uint32_t*)ch->ring;
   
    // Reset producer index
    (void)ioctl(ch->fd, RESET_PROD_IDX, 0);
   
    // Pre-allocate capture buffer
    ch->raw_data = (uint32_t*)malloc(ch->target_raw_words * sizeof(uint32_t));
    if (!ch->raw_data) {
        perror("malloc raw");
        munmap((void*)ch->ring, ch->ring_bytes);
        close(ch->fd);
        ch->thread_result = -1;
        return NULL;
    }
   
    // Signal ready and wait for main thread to issue START_ALL_CHANNELS
    pthread_mutex_lock(ch->start_mutex);
    while (!(*ch->start_flag)) {
        pthread_cond_wait(ch->start_cond, ch->start_mutex);
    }
    pthread_mutex_unlock(ch->start_mutex);
   
    // Main thread has issued START_ALL_CHANNELS for all channels
    // Now just wait for completion and read data
    ch->raw_written = 0;
    size_t batches_needed = (ch->target_raw_words + frame_words - 1) / frame_words;
   
    for (size_t batch = 0; batch < batches_needed && ch->raw_written < ch->target_raw_words; ++batch) {
        size_t frame_idx = batch % g_buffer_count;
        volatile uint32_t *frame = ch->ring_u32 + frame_idx * frame_words;
       
        // Wait for this transfer to complete
        if (ioctl(ch->fd, FINISH_XFER, (unsigned long)1) < 0) {
            perror("FINISH_XFER");
            break;
        }
       
        cpu_barrier();
       
        // Copy entire frame
        size_t words_to_copy = frame_words;
        size_t space_left = ch->target_raw_words - ch->raw_written;
        if (words_to_copy > space_left) words_to_copy = space_left;
       
        memcpy(ch->raw_data + ch->raw_written, (const void*)frame, words_to_copy * sizeof(uint32_t));
        ch->raw_written += words_to_copy;
       
        // Issue next START_XFER if we need more batches
        if (batch + 1 < batches_needed) {
            if (ioctl(ch->fd, START_XFER, (unsigned long)1) < 0) {
                perror("START_XFER");
                break;
            }
        }
    }
   
    // NO PRINTS - write bin file immediately without interruption
    if (write_bin(ch->bin_path, ch->raw_data, ch->raw_written) != 0) {
        ch->thread_result = -1;
        return NULL;
    }
   
    ch->thread_result = 0;
    return NULL;
}


// ---- Post-process channel data ----
static int process_channel_data(dma_channel_t *ch, size_t csv_words_goal) {
    if (g_verbose) {
        printf("[DMA%d] Processing %zu raw words\n", ch->channel_id, ch->raw_written);
       
        // Show RAW data preview (pre-processed, from bin file)
        if (ch->raw_written > 0) {
            printf("[DMA%d] RAW DATA (pre-processed from bin file):\n", ch->channel_id);
            size_t raw_preview = (g_preview_words < ch->raw_written) ? g_preview_words : ch->raw_written;
            for (size_t i = 0; i < raw_preview; ++i) {
                printf("  [%4zu] 0x%08x\n", i, ch->raw_data[i]);
            }
        }
    }
   
    // Post-process
    size_t n1 = ch->raw_written;
    uint32_t *stage = ch->raw_data;
   
    // IMPORTANT: For aligned multi-channel capture, dedup can break alignment
    // because different channels may have different duplicate patterns.
    // Consider disabling dedup for synchronized captures.
   
    // Dedup
    uint32_t *ded = NULL;
    if (g_dedup_pairs) {
        ded = (uint32_t*)malloc(n1 * sizeof(uint32_t));
        if (!ded) {
            perror("malloc dedup");
            return -1;
        }
        size_t n2 = collapse_doublets(stage, n1, ded);
       
        if (g_verbose && ch->channel_id == 0) {
            printf("WARNING: Deduplication can break channel alignment!\n");
            printf("         Consider using --no-dedup for synchronized captures.\n");
        }
       
        stage = ded;
        n1 = n2;
    }
   
    // Skip bits - apply raw word skip before bit calculation to maintain alignment
    size_t skip_words = (g_skip_bits + 31) / 32;
   
    // Apply channel-specific FPGA pipeline offset compensation
    if (ch->channel_id == 1) {
        skip_words += g_offset_dma1;
    } else if (ch->channel_id == 2) {
        skip_words += g_offset_dma2;
    }
   
    size_t start_idx = skip_words < n1 ? skip_words : n1;
   
    // Drop zeros and collect into processed buffer
    uint32_t *filt = (uint32_t*)malloc(n1 * sizeof(uint32_t));
    if (!filt) {
        perror("malloc filt");
        if (ded) free(ded);
        return -1;
    }
   
    size_t kept = 0;
    for (size_t i = start_idx; i < n1; ++i) {
        uint32_t w = stage[i];
        if (!g_drop_zeros || w != 0u) filt[kept++] = w;
    }
   
    // Trim to goal
    size_t final_words = kept < csv_words_goal ? kept : csv_words_goal;
   
    // Store processed data
    ch->processed_data = (uint32_t*)malloc(final_words * sizeof(uint32_t));
    if (!ch->processed_data) {
        perror("malloc processed");
        if (ded) free(ded);
        free(filt);
        return -1;
    }
   
    memcpy(ch->processed_data, filt, final_words * sizeof(uint32_t));
    ch->processed_count = final_words;
   
    if (g_verbose) {
        printf("[DMA%d] Post-process: raw=%zu, dedup=%zu, kept=%zu, final=%zu\n",
               ch->channel_id, ch->raw_written, n1, kept, final_words);
       
        // Show PROCESSED data that will go to CSV
        if (final_words) {
            printf("[DMA%d] PROCESSED DATA (post-processed, going to CSV):\n", ch->channel_id);
            size_t proc_preview = (g_preview_words < final_words) ? g_preview_words : final_words;
            for (size_t i = 0; i < proc_preview; ++i) {
                printf("  [%4zu] 0x%08x\n", i, ch->processed_data[i]);
            }
        }
    }
   
    // Cleanup temp buffers
    if (ded) free(ded);
    free(filt);
   
    return 0;
}


// ---- Write combined CSV with N columns ----
static int write_combined_csv(const char *path, dma_channel_t *channels, int num_channels) {
    if (!g_write_csv) return 0;
   
    // Find max length
    size_t max_words = 0;
    for (int i = 0; i < num_channels; ++i) {
        if (channels[i].processed_count > max_words) {
            max_words = channels[i].processed_count;
        }
    }
   
    if (max_words == 0) {
        printf("NOTE: No data to write to CSV\n");
        return 0;
    }
   
    FILE *f = fopen(path, "w");
    if (!f) {
        perror("fopen(csv)");
        return -1;
    }
   
    // Write header with actual DMA IDs
    for (int i = 0; i < num_channels; ++i) {
        fprintf(f, "DMA%d", channels[i].channel_id);
        if (i < num_channels - 1) fprintf(f, ",");
    }
    fprintf(f, "\n");
   
    // Write bit rows (32 bits per word)
    size_t total_rows = max_words * 32;
   
    for (size_t row = 0; row < total_rows; ++row) {
        for (int ch = 0; ch < num_channels; ++ch) {
            size_t word_idx = row / 32;
            int bit_idx = row % 32;
           
            char bit_char = '0';  // default if no data
           
            if (word_idx < channels[ch].processed_count) {
                uint32_t w = apply_xforms(channels[ch].processed_data[word_idx]);
               
                // Extract bit based on msb_first setting
                if (g_msb_first) {
                    bit_idx = 31 - bit_idx;  // reverse bit order within word
                }
               
                bit_char = ((w >> bit_idx) & 1u) ? '1' : '0';
            }
           
            fputc(bit_char, f);
           
            if (ch < num_channels - 1) {
                fputc(',', f);
            }
        }
        fputc('\n', f);
    }
   
    fclose(f);
   
    if (g_verbose) {
        printf("Combined CSV written: %s\n", path);
        printf("  %zu rows (bits), %d columns (DMAs)\n", total_rows, num_channels);
    }
   
    return 0;
}


static void usage(const char *prog){
    fprintf(stderr,
      "Usage: sudo %s <csv_words>\n"
      "  [--buffer-count N] [--frame-bytes N] [--batch N]\n"
      "  [--no-bin] [--no-csv]\n"
      "  [--msb-first | --lsb-first] [--swap-bytes] [--rev-bits]\n"
      "  [--no-dedup] [--no-skip-first] [--keep-zeros]\n"
      "  [--multiplier M] [--quiet]\n"
      "  [--channels N] [--dma-ids ID1,ID2,...]\n"
      "  [--offset-dma1 N] [--offset-dma2 N]\n"
      "  [--no-confirm]\n"
      "\n"
      "DMA Channel Selection:\n"
      "  --channels N         Number of DMA channels to use (1-3, default: 3)\n"
      "  --dma-ids ID1,ID2... Comma-separated DMA IDs to use (default: 0,1,2)\n"
      "                       Example: --dma-ids 0,2 (use only DMA0 and DMA2)\n"
      "  --no-confirm         Skip pinout confirmation prompt\n"
      "\n"
      "Examples:\n"
      "  sudo %s 8000000 --lsb-first\n"
      "  sudo %s 8000000 --channels 2 --dma-ids 0,1\n"
      "  sudo %s 8000000 --dma-ids 1 --no-confirm\n",
      prog, prog, prog, prog);
}


static void print_pinout_banner(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════╗\n");
    printf("║                     ZEDBOARD PMOD PINOUT                       ║\n");
    printf("╠════════════════════════════════════════════════════════════════╣\n");
    printf("║  JC1P (Pin 1) --> CLKP        (Clock Positive)                 ║\n");
    printf("║  JC1N (Pin 2) --> CLKN        (Clock Negative)                 ║\n");
    printf("║  JC2P (Pin 3) --> Data P      (DMA0 Data Positive)             ║\n");
    printf("║  JC2N (Pin 4) --> Data N      (DMA0 Data Negative)             ║\n");
    printf("║  JC3P (Pin 7) --> Data 2 P    (DMA1 Data Positive)             ║\n");
    printf("║  JC3N (Pin 8) --> Data 2 N    (DMA1 Data Negative)             ║\n");
    printf("║  JC4P (Pin 9) --> Data 3 P    (DMA2 Data Positive)             ║\n");
    printf("║  JC4N (Pin 10)--> Data 3 N    (DMA2 Data Negative)             ║\n");
    printf("╚════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
}


static int confirm_capture(void) {
    char response[10];
    printf("Press ENTER to start capture (or Ctrl+C to abort): ");
    fflush(stdout);
   
    if (fgets(response, sizeof(response), stdin) == NULL) {
        return 0;  // EOF or error
    }
   
    return 1;  // User pressed enter
}


static int parse_dma_ids(const char *str) {
    char *copy = strdup(str);
    if (!copy) return -1;
   
    char *token = strtok(copy, ",");
    g_num_active_channels = 0;
   
    while (token && g_num_active_channels < MAX_DMA_CHANNELS) {
        int id = atoi(token);
        if (id < 0 || id >= MAX_DMA_CHANNELS) {
            fprintf(stderr, "Invalid DMA ID: %d (must be 0-%d)\n", id, MAX_DMA_CHANNELS - 1);
            free(copy);
            return -1;
        }
        g_active_channels[g_num_active_channels++] = id;
        token = strtok(NULL, ",");
    }
   
    free(copy);
   
    if (g_num_active_channels == 0) {
        fprintf(stderr, "No valid DMA IDs specified\n");
        return -1;
    }
   
    return 0;
}


int main(int argc, char **argv){
    if (argc < 2) { usage(argv[0]); return 1; }
   
    // Parse arguments
    char *endp = NULL;
    long req_words_long = strtol(argv[1], &endp, 0);
    if (endp == argv[1] || req_words_long <= 0) {
        fprintf(stderr, "Invalid <csv_words>\n");
        return 1;
    }
    size_t csv_words_goal = (size_t)req_words_long;
   
    for (int i = 2; i < argc; ++i) {
        if (!strcmp(argv[i], "--buffer-count") && i+1 < argc)
            g_buffer_count = (size_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--frame-bytes") && i+1 < argc)
            g_frame_bytes = (size_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--batch") && i+1 < argc)
            g_batch_frames = (size_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--no-bin")) g_write_bin = 0;
        else if (!strcmp(argv[i], "--no-csv")) g_write_csv = 0;
        else if (!strcmp(argv[i], "--msb-first")) g_msb_first = 1;
        else if (!strcmp(argv[i], "--lsb-first")) g_msb_first = 0;
        else if (!strcmp(argv[i], "--swap-bytes")) g_swap_bytes = 1;
        else if (!strcmp(argv[i], "--rev-bits")) g_rev_bits_in_byte = 1;
        else if (!strcmp(argv[i], "--no-dedup")) g_dedup_pairs = 0;
        else if (!strcmp(argv[i], "--no-skip-first")) g_skip_first_word = 0;
        else if (!strcmp(argv[i], "--keep-zeros")) g_drop_zeros = 0;
        else if (!strcmp(argv[i], "--multiplier") && i+1 < argc)
            g_capture_multiplier = (size_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--offset-dma1") && i+1 < argc)
            g_offset_dma1 = (size_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--offset-dma2") && i+1 < argc)
            g_offset_dma2 = (size_t)strtoul(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--reset-dma")) g_reset_dma_before_capture = 1;
        else if (!strcmp(argv[i], "--channels") && i+1 < argc)
            g_num_active_channels = (int)strtol(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--dma-ids") && i+1 < argc) {
            if (parse_dma_ids(argv[++i]) != 0) return 1;
        }
        else if (!strcmp(argv[i], "--no-confirm")) g_no_confirm = 1;
        else if (!strcmp(argv[i], "--quiet")) g_verbose = 0;
        else { usage(argv[0]); return 1; }
    }
   
    // Validate config
    if (g_frame_bytes == 0 || (g_frame_bytes & 3)) {
        fprintf(stderr, "frame-bytes must be multiple of 4\n");
        return 1;
    }
    if (g_batch_frames == 0 || g_batch_frames > g_buffer_count) {
        fprintf(stderr, "batch must be in 1..buffer-count\n");
        return 1;
    }
   
    // Show pinout if requested
    if (!g_no_confirm) {
        print_pinout_banner();
        if (!confirm_capture()) {
            printf("Capture aborted.\n");
            return 0;
        }
    }
   
    // Get capture ID
    int cap_id = get_next_capture_id();
    if (g_verbose) {
        printf("Capture ID: %d\n", cap_id);
        printf("Starting parallel capture on %d DMA channels\n", NUM_DMA_CHANNELS);
    }
   
    // Synchronization primitives
    pthread_mutex_t start_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t start_cond = PTHREAD_COND_INITIALIZER;
    volatile int start_flag = 0;
   
    // Initialize channel structures
    dma_channel_t channels[NUM_DMA_CHANNELS];
    for (int i = 0; i < NUM_DMA_CHANNELS; ++i) {
        channels[i].channel_id = i;
        channels[i].dev_path = DMA_PATHS[i];
        channels[i].fd = -1;
        channels[i].ring = NULL;
        channels[i].ring_u32 = NULL;
        channels[i].raw_data = NULL;
        channels[i].raw_written = 0;
        channels[i].processed_data = NULL;
        channels[i].processed_count = 0;
        channels[i].thread_result = -1;
        channels[i].start_mutex = &start_mutex;
        channels[i].start_cond = &start_cond;
        channels[i].start_flag = &start_flag;
       
        size_t target = csv_words_goal * g_capture_multiplier;
        if (target == 0) target = csv_words_goal;
        channels[i].target_raw_words = target;
       
        // Generate bin filenames per channel
        snprintf(channels[i].bin_path, sizeof(channels[i].bin_path),
                 "cap_%04d_dma%d.bin", cap_id, i);
       
        if (g_verbose) {
            printf("  DMA%d: %s -> %s\n", i, channels[i].dev_path, channels[i].bin_path);
        }
    }
   
    // Launch threads - they will prepare but not start DMA
    for (int i = 0; i < NUM_DMA_CHANNELS; ++i) {
        if (pthread_create(&channels[i].thread, NULL, dma_capture_thread, &channels[i]) != 0) {
            perror("pthread_create");
        }
    }
   
    // Give threads time to open devices and allocate buffers
    usleep(200000);  // 200ms
   
    // Reset all DMAs if requested (clears any stale FIFO data)
    if (g_reset_dma_before_capture) {
        if (g_verbose) {
            printf("\n=== Resetting all DMAs ===\n");
        }
        for (int i = 0; i < NUM_DMA_CHANNELS; ++i) {
            if (channels[i].fd >= 0) {
                if (ioctl(channels[i].fd, RESET_DMA, 0) < 0) {
                    perror("RESET_DMA");
                }
            }
        }
        usleep(50000);  // Wait 50ms after reset
    }
   
    // ===== CRITICAL CHANGE: Use START_ALL_CHANNELS instead of individual START_XFER =====
    if (g_verbose) {
        printf("\n=== Issuing START_ALL_CHANNELS (synchronized start) ===\n");
    }
   
    // Call START_ALL_CHANNELS on ANY channel fd - it starts all registered channels atomically
    int any_fd = -1;
    for (int i = 0; i < NUM_DMA_CHANNELS; ++i) {
        if (channels[i].fd >= 0) {
            any_fd = channels[i].fd;
            break;
        }
    }
   
    if (any_fd < 0) {
        fprintf(stderr, "ERROR: No valid channel file descriptors!\n");
        return 1;
    }
   
    if (ioctl(any_fd, START_ALL_CHANNELS, 0) < 0) {
        perror("START_ALL_CHANNELS failed");
        fprintf(stderr, "NOTE: Make sure you're using the updated driver with START_ALL_CHANNELS support\n");
        return 1;
    }
   
    if (g_verbose) {
        printf("All channels started atomically!\n");
    }
   
    // Signal all threads that DMA has started
    pthread_mutex_lock(&start_mutex);
    start_flag = 1;
    pthread_cond_broadcast(&start_cond);
    pthread_mutex_unlock(&start_mutex);
   
    // Wait for all threads to complete
    for (int i = 0; i < NUM_DMA_CHANNELS; ++i) {
        pthread_join(channels[i].thread, NULL);
    }
   
    // NOW we can print - all bin files written
    printf("\n=== Capture Complete - Bin Files Written ===\n");
    for (int i = 0; i < NUM_DMA_CHANNELS; ++i) {
        printf("[DMA%d] Captured %zu words -> %s\n",
               i, channels[i].raw_written, channels[i].bin_path);
    }
   
    printf("\n=== PHASE 2: Post-Processing ===\n");
    // Process all channels
    int overall_result = 0;
    for (int i = 0; i < NUM_DMA_CHANNELS; ++i) {
        if (channels[i].thread_result != 0) {
            printf("[DMA%d] Thread failed\n", i);
            overall_result = -1;
            continue;
        }
       
        if (process_channel_data(&channels[i], csv_words_goal) != 0) {
            printf("[DMA%d] Processing failed\n", i);
            overall_result = -1;
        }
    }
   
    // Write combined CSV
    printf("\n=== PHASE 3: Writing Combined CSV ===\n");
    char csv_path[256];
    snprintf(csv_path, sizeof(csv_path), "output_%04d.csv", cap_id);
   
    // FIX: Pass correct number of arguments to write_combined_csv
    if (write_combined_csv(csv_path, channels, NUM_DMA_CHANNELS) != 0) {
        printf("Failed to write combined CSV\n");
        overall_result = -1;
    }
   
    // Summary
    printf("\n=== SUMMARY ===\n");
    for (int i = 0; i < NUM_DMA_CHANNELS; ++i) {
        printf("[DMA%d] raw=%zu words, processed=%zu words\n",
               i, channels[i].raw_written, channels[i].processed_count);
    }
   
    // Cleanup
    for (int i = 0; i < NUM_DMA_CHANNELS; ++i) {
        if (channels[i].processed_data) free(channels[i].processed_data);
        if (channels[i].raw_data) free(channels[i].raw_data);
        if (channels[i].ring) munmap((void*)channels[i].ring, channels[i].ring_bytes);
        if (channels[i].fd >= 0) close(channels[i].fd);
    }
   
    pthread_mutex_destroy(&start_mutex);
    pthread_cond_destroy(&start_cond);
   
    printf("\n=== ALL CHANNELS COMPLETE ===\n");
    return overall_result;
}
