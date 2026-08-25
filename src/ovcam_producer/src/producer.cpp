// ─────────────────────────────────────────────────────────────────
// ovcam_producer — libcamera → POSIX shm ring buffer
// ─────────────────────────────────────────────────────────────────

#include "ovcam_shm.hpp"
#include <libcamera/libcamera.h>

#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <ctime>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <unordered_map>

using namespace libcamera;

// ── signal handling ───────────────────────────────────────────────
static volatile sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

// ── shared memory context ─────────────────────────────────────────
struct Shm {
    void*      base  = MAP_FAILED;
    size_t     total = 0;
    uint32_t   slots = 0;
    uint64_t   ss    = 0;
    ShmGlobal* g     = nullptr;
};

// ── pre-mapped DMA plane ──────────────────────────────────────────
// We need TWO pointers per plane:
//   mmap_base / mmap_len — what mmap returned, passed to munmap
//   data      / data_len — where the actual pixels are (mmap_base + plane.offset)
struct Plane {
    void*  mmap_base;
    size_t mmap_len;
    void*  data;      // = mmap_base + plane.offset
    size_t data_len;
};

// ── globals shared with callback ──────────────────────────────────
static Shm        g_shm;
static sem_t*     g_sem    = SEM_FAILED;
static uint64_t   g_fn     = 0;
static Stream*    g_stream = nullptr;
static Camera*    g_camera = nullptr;
static std::unordered_map<const FrameBuffer*, std::vector<Plane>> g_planes;

// ── create shared memory ──────────────────────────────────────────
static Shm make_shm(const char* name, uint32_t w, uint32_t h, uint32_t n_slots) {
    const uint32_t stride      = (w + 31) & ~31u;
    const uint64_t frame_bytes = (uint64_t)stride * h * 3 / 2;
    const uint64_t ss    = ((sizeof(SlotHeader) + frame_bytes) + SHM_ALIGN - 1)
                           & ~(SHM_ALIGN - 1);
    const size_t   total = sizeof(ShmGlobal) + n_slots * ss;

    shm_unlink(name);
    int fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (fd < 0) throw std::runtime_error("shm_open failed");
    ftruncate(fd, (off_t)total);

    void* base = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (base == MAP_FAILED) throw std::runtime_error("mmap failed");
    memset(base, 0, total);

    auto* g        = static_cast<ShmGlobal*>(base);
    g->magic       = OVCAM_MAGIC;
    g->version     = 1;
    g->slot_count  = n_slots;
    g->slot_size   = ss;
    g->max_frame_bytes = frame_bytes;
    g->width       = w;
    g->height      = h;
    g->stride      = stride;
    g->fourcc      = FOURCC_NV12;
    g->write_seq   = 0;

    printf("[shm] created '%s': %u slots x %llu bytes, total=%zu\n",
           name, n_slots, (unsigned long long)ss, total);
    return {base, total, n_slots, ss, g};
}

// ── libcamera callback — called every frame ───────────────────────
static void on_request(Request* req) {
    if (req->status() == Request::RequestCancelled) return;

    const FrameBuffer* fb = req->buffers().at(g_stream);
    auto it = g_planes.find(fb);
    if (it == g_planes.end()) {
        req->reuse(Request::ReuseBuffers);
        g_camera->queueRequest(req);
        return;
    }

    // which ring buffer slot?
    uint64_t fn  = ++g_fn;
    uint32_t idx = (uint32_t)((fn - 1) % g_shm.slots);
    SlotHeader* slot = slot_at(g_shm.base, g_shm.ss, idx);
    uint8_t*    dst  = frame_data(slot);

    // seqlock open: odd = "being written"
    seq_store(slot, fn * 2 - 1, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_RELEASE);

    // timestamp
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);

    // THE ONE COPY: DMA buffer → shared memory
    size_t off = 0;
    for (auto& p : it->second) {
        size_t n = std::min(p.data_len, g_shm.ss - sizeof(SlotHeader) - off);
        memcpy(dst + off, p.data, n);
        off += n;
    }

    // fill metadata
    slot->t_ns       = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    slot->width      = g_shm.g->width;
    slot->height     = g_shm.g->height;
    slot->stride     = g_shm.g->stride;
    slot->fourcc     = FOURCC_NV12;
    slot->bytes_used = (uint32_t)off;

    // The real camera has no upstream timestamp to carry, so t_src_ns is always
    // "absent" here. Written explicitly rather than left alone so a reused ring
    // slot can never hand a consumer a stale value from a previous frame.
    slot->t_src_ns   = 0ULL;

    // seqlock close: even = "complete"
    seq_store(slot, fn * 2, __ATOMIC_RELEASE);

    // advance global cursor so consumer knows a new frame is ready
    ws_store(g_shm.g, fn);

    // wake consumer
    sem_post(g_sem);

    // give buffer back to ISP for next frame
    req->reuse(Request::ReuseBuffers);
    g_camera->queueRequest(req);
}

// ── usage ─────────────────────────────────────────────────────────
static void print_usage(const char* prog) {
    printf("Usage: %s [options]\n\n"
           "Stream options:\n"
           "  --width  <px>      Frame width  (default: 1280)\n"
           "  --height <px>      Frame height (default: 720)\n"
           "  --fps    <n>       Target frame rate (default: 30)\n"
           "  --slots  <n>       Ring-buffer slot count (default: 4)\n\n"
           "Exposure options:\n"
           "  --shutter <us>     Fixed shutter time in microseconds.\n"
           "                     Disables AE for exposure. Example: 10000 = 10 ms.\n"
           "                     Omit to let AE control exposure automatically.\n"
           "  --gain <factor>    Analogue gain multiplier (e.g. 1.0 = no gain, 8.0 = 8x).\n"
           "                     Disables AE for gain. Omit for automatic.\n\n"
           "Note: setting either --shutter or --gain locks AE entirely. Set both\n"
           "      for full manual control, or neither for full auto.\n",
           prog);
}

// ── main ──────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    uint32_t W = 1280, H = 720, FPS = 30, N = 4;
    const char* SHM = "/ovcam_frames";
    const char* SEM = "/ovcam_ready";

    // Shutter / gain: negative sentinel = "not set by user → use AE"
    int64_t shutter_us = -1;   // microseconds; -1 = auto
    float   gain       = -1.f; // analogue gain; -1 = auto

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            print_usage(argv[0]);
            return 0;
        }
        if (!strcmp(argv[i], "--width")   && i+1 < argc) W          = atoi(argv[++i]);
        if (!strcmp(argv[i], "--height")  && i+1 < argc) H          = atoi(argv[++i]);
        if (!strcmp(argv[i], "--fps")     && i+1 < argc) FPS        = atoi(argv[++i]);
        if (!strcmp(argv[i], "--slots")   && i+1 < argc) N          = atoi(argv[++i]);
        if (!strcmp(argv[i], "--shutter") && i+1 < argc) shutter_us = atoll(argv[++i]);
        if (!strcmp(argv[i], "--gain")    && i+1 < argc) gain       = atof(argv[++i]);
    }

    // Basic sanity checks for exposure args
    if (shutter_us == 0) {
        fprintf(stderr, "[error] --shutter must be > 0 microseconds\n");
        return 1;
    }
    if (gain == 0.f || gain < -1.f) {
        fprintf(stderr, "[error] --gain must be a positive multiplier (e.g. 1.0–16.0)\n");
        return 1;
    }

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    g_shm = make_shm(SHM, W, H, N);

    sem_unlink(SEM);
    g_sem = sem_open(SEM, O_CREAT, 0666, 0);
    if (g_sem == SEM_FAILED) throw std::runtime_error("sem_open failed");

    // open camera
    auto cm = std::make_unique<CameraManager>();
    cm->start();
    if (cm->cameras().empty()) throw std::runtime_error("no cameras found");

    auto cam = cm->get(cm->cameras()[0]->id());
    cam->acquire();
    g_camera = cam.get();

    // configure stream
    auto cfg = cam->generateConfiguration({StreamRole::VideoRecording});
    auto& sc = cfg->at(0);
    sc.size        = {W, H};
    sc.pixelFormat = formats::NV12;
    sc.bufferCount = N;
    cfg->validate();
    cam->configure(cfg.get());
    g_stream = sc.stream();
    printf("[cam] actual: %ux%u stride=%u\n",
           sc.size.width, sc.size.height, sc.stride);

    // allocate DMA buffers
    auto alloc = std::make_unique<FrameBufferAllocator>(cam);
    alloc->allocate(g_stream);

    // pre-map every DMA buffer ONCE at startup
    //
    // WHY NOT just use pl.offset in mmap():
    //   mmap() requires its offset argument to be page-aligned (multiple of 4096).
    //   libcamera plane offsets are NOT guaranteed page-aligned.
    //   Fix: always map from offset 0, then shift the pointer by pl.offset.
    //
    for (const auto& buf : alloc->buffers(g_stream)) {
        std::vector<Plane> planes;
        for (const auto& pl : buf->planes()) {
            size_t map_len = pl.offset + pl.length;  // map from 0 to cover the plane
            void*  mb = mmap(nullptr, map_len, PROT_READ,
                             MAP_SHARED, pl.fd.get(), 0);
            if (mb == MAP_FAILED) throw std::runtime_error("DMA mmap failed");

            planes.push_back({
                mb,                                          // mmap_base → for munmap
                map_len,                                     // mmap_len  → for munmap
                static_cast<uint8_t*>(mb) + pl.offset,      // data      → for memcpy
                pl.length                                    // data_len  → for memcpy
            });
        }
        g_planes[buf.get()] = std::move(planes);
    }

    // build requests (one per buffer)
    std::vector<std::unique_ptr<Request>> reqs;
    for (const auto& buf : alloc->buffers(g_stream)) {
        auto r = cam->createRequest();
        r->addBuffer(g_stream, buf.get());
        reqs.push_back(std::move(r));
    }

    // ── build ControlList ─────────────────────────────────────────
    //
    // Frame-duration limits lock the FPS ceiling/floor.
    // Shutter and gain are independent: set either/both for manual,
    // or omit both to stay in full-AE mode.
    //
    // AeEnable must be explicitly disabled whenever we override
    // exposure time or gain, otherwise the ISP will fight our values.
    //
    ControlList ctrls;

    // FPS via frame duration (always set)
    int64_t fd_us = 1000000LL / FPS;
    ctrls.set(controls::FrameDurationLimits,
              Span<const int64_t, 2>({fd_us, fd_us}));

    const bool manual_shutter = (shutter_us > 0);
    const bool manual_gain    = (gain > 0.f);

    if (manual_shutter || manual_gain) {
        // Disable AE so the ISP doesn't override our values
        ctrls.set(controls::AeEnable, false);

        if (manual_shutter) {
            ctrls.set(controls::ExposureTime, (int32_t)shutter_us);
            printf("[cam] shutter: %lld µs (manual)\n", (long long)shutter_us);
        } else {
            printf("[cam] shutter: auto (only gain is manual)\n");
        }

        if (manual_gain) {
            ctrls.set(controls::AnalogueGain, gain);
            printf("[cam] gain: %.2fx (manual)\n", gain);
        } else {
            printf("[cam] gain: auto (only shutter is manual)\n");
        }
    } else {
        // Full auto-exposure — AeEnable defaults to true, but be explicit
        ctrls.set(controls::AeEnable, true);
        printf("[cam] exposure: full AE (auto shutter + auto gain)\n");
    }

    // plain function pointer — libcamera Signal does not accept lambdas
    cam->requestCompleted.connect(on_request);
    cam->start(&ctrls);
    for (auto& r : reqs) cam->queueRequest(r.get());

    printf("[producer] streaming %ux%u NV12 @ %u fps — Ctrl-C to stop\n",
           W, H, FPS);

    while (!g_stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // cleanup
    cam->stop();
    for (auto& [fb, planes] : g_planes)
        for (auto& p : planes)
            munmap(p.mmap_base, p.mmap_len);
    reqs.clear();
    alloc->free(g_stream);
    cam->release();
    cm->stop();
    sem_close(g_sem);
    sem_unlink(SEM);
    munmap(g_shm.base, g_shm.total);
    shm_unlink(SHM);

    printf("[producer] done. %llu frames.\n", (unsigned long long)g_fn);
}
