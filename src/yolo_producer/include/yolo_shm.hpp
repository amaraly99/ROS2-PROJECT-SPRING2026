#pragma once
#include <cstdint>
#include <cstddef>

// ─────────────────────────────────────────────────────────────────
// Shared memory layout for YOLO detection results + annotated frame.
// Written by yolo_producer (host), read by yolo_bridge (Docker/ROS2).
// Same seqlock + semaphore pattern as ovcam_shm.hpp.
// ─────────────────────────────────────────────────────────────────

static constexpr uint32_t YOLO_MAGIC        = 0x594F4C4Fu;  // "YOLO"
static constexpr uint32_t YOLO_SHM_VERSION  = 1u;
static constexpr size_t   YOLO_SHM_ALIGN    = 64u;
static constexpr uint32_t YOLO_MAX_DETS     = 64u;

// ─────────────────────────────────────────────────────────────────
// Single detection (16 bytes)
// Bounding box in source-image pixel coordinates.
// ─────────────────────────────────────────────────────────────────
struct YoloDet {
    uint16_t class_id;
    uint16_t _pad;
    float    confidence;
    uint16_t x1, y1, x2, y2;   // bbox corners in source pixels
};
static_assert(sizeof(YoloDet) == 16, "YoloDet must be 16 bytes");

// ─────────────────────────────────────────────────────────────────
// Per-frame slot: detection array + annotated BGR frame
//
//   [ YoloSlotHeader (128 bytes)               ]
//   [ det_count × YoloDet (up to 1024 bytes)   ]  ← detections pad
//   [ BGR pixel data (width × height × 3)      ]  ← annotated frame
// ─────────────────────────────────────────────────────────────────
struct alignas(64) YoloSlotHeader {
    uint64_t seq;           // seqlock (odd = writing, even = done)
    uint64_t t_ns;          // source frame timestamp (CLOCK_MONOTONIC)
    uint32_t det_count;     // number of detections in this frame
    uint32_t img_width;     // annotated image width
    uint32_t img_height;    // annotated image height
    uint32_t img_stride;    // bytes per row (= width * 3 for BGR)
    uint32_t img_bytes;     // total bytes of BGR pixel data
    uint32_t _pad[5];       // fill to 64 bytes for first cache line

    // Second cache line: reserved
    uint64_t _reserved[8];
};
static_assert(sizeof(YoloSlotHeader) == 128, "YoloSlotHeader must be 128 bytes");

// ─────────────────────────────────────────────────────────────────
// Global header — sits at the start of /yolo_shm
// ─────────────────────────────────────────────────────────────────
struct alignas(64) YoloShmGlobal {
    uint32_t magic;         // YOLO_MAGIC
    uint32_t version;       // YOLO_SHM_VERSION
    uint32_t slot_count;    // ring buffer depth (e.g. 4)
    uint32_t _p0;

    uint64_t slot_size;     // bytes per slot (header + dets pad + image)
    uint32_t img_width;     // annotated image dimensions
    uint32_t img_height;

    alignas(64) uint64_t write_seq;
};

// ─────────────────────────────────────────────────────────────────
// Layout helpers
// ─────────────────────────────────────────────────────────────────

// Offset from slot start to the detection array
static constexpr size_t YOLO_DETS_OFFSET = sizeof(YoloSlotHeader);

// Offset from slot start to the annotated image data
// Detection array occupies YOLO_MAX_DETS * sizeof(YoloDet) = 1024 bytes
static constexpr size_t YOLO_DETS_PAD  = YOLO_MAX_DETS * sizeof(YoloDet);
static constexpr size_t YOLO_IMG_OFFSET = sizeof(YoloSlotHeader) + YOLO_DETS_PAD;

inline YoloSlotHeader* yolo_slot_at(void* base, uint64_t slot_size, uint32_t i) {
    uint8_t* b = static_cast<uint8_t*>(base);
    return reinterpret_cast<YoloSlotHeader*>(
        b + sizeof(YoloShmGlobal) + i * slot_size);
}

inline YoloDet* yolo_dets(YoloSlotHeader* h) {
    return reinterpret_cast<YoloDet*>(
        reinterpret_cast<uint8_t*>(h) + YOLO_DETS_OFFSET);
}

inline uint8_t* yolo_img_data(YoloSlotHeader* h) {
    return reinterpret_cast<uint8_t*>(h) + YOLO_IMG_OFFSET;
}

// ─────────────────────────────────────────────────────────────────
// Atomic helpers (same rationale as ovcam_shm.hpp — no std::atomic
// in mmap'd memory; use GCC builtins instead)
// ─────────────────────────────────────────────────────────────────
inline uint64_t yolo_seq_load(const YoloSlotHeader* h) {
    return __atomic_load_n(&h->seq, __ATOMIC_ACQUIRE);
}

inline void yolo_seq_store(YoloSlotHeader* h, uint64_t v, int order) {
    __atomic_store_n(&h->seq, v, order);
}

inline uint64_t yolo_ws_load(const YoloShmGlobal* g) {
    return __atomic_load_n(&g->write_seq, __ATOMIC_ACQUIRE);
}

inline void yolo_ws_store(YoloShmGlobal* g, uint64_t v) {
    __atomic_store_n(&g->write_seq, v, __ATOMIC_RELEASE);
}
