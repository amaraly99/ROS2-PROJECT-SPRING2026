#pragma once          // only include this file once per compilation
#include <cstdint>    // uint8_t, uint32_t, uint64_t
#include <cstddef>    // size_t

// ─────────────────────────────────────────────────────────────────
// WHAT IS THIS FILE?
// A header shared by BOTH the producer and the ROS2 bridge.
// It defines exactly what the shared memory looks like in RAM.
// Both programs must agree on this layout — like a contract.
// ─────────────────────────────────────────────────────────────────

// A stamp we write into shm so the consumer can verify it opened
// the right shared memory. Spells "OVCB" in ASCII.
static constexpr uint32_t OVCAM_MAGIC = 0x4F564342u;

// Pixel format identifier. Spells "NV12" in ASCII (little-endian).
// NV12 = Y plane (brightness) + interleaved UV plane (color).
// This is the format the Pi ISP outputs natively — no conversion needed.
static constexpr uint32_t FOURCC_NV12 = 0x3231564Eu;

// ARM Cortex-A cache line = 64 bytes.
// Aligning structs to this prevents two threads from accidentally
// sharing a cache line and causing slowdowns (false sharing).
static constexpr size_t SHM_ALIGN = 64u;

// ─────────────────────────────────────────────────────────────────
// SLOT HEADER — sits at the start of each ring buffer slot
// Followed immediately in memory by the raw pixel bytes.
//
// alignas(64) = start this struct on a 64-byte boundary
// ─────────────────────────────────────────────────────────────────
struct alignas(64) SlotHeader {

    // Seqlock counter. Rules:
    //   0        = slot was never written
    //   odd      = writer is currently filling this slot (don't read!)
    //   even N   = slot is complete, frame_number = N / 2
    uint64_t seq;

    uint64_t t_ns;      // capture timestamp (CLOCK_MONOTONIC, nanoseconds)
    uint32_t width;     // pixels
    uint32_t height;    // pixels
    uint32_t stride;    // bytes per row (may be wider than width due to alignment)
    uint32_t fourcc;    // pixel format (FOURCC_NV12 above)
    uint32_t bytes_used;// total bytes of pixel data written

    uint32_t _pad0;     // keeps t_src_ns 8-byte aligned

    // SOURCE timestamp, nanoseconds. 0 = "not provided", which is what every
    // writer predating this field effectively stores.
    //
    // WHY THIS EXISTS: t_ns above is stamped by the *consumer side* when the
    // frame is written into this ring — i.e. it encodes the Pi's arrival time,
    // including network jitter. For one camera that is harmless. For STEREO it
    // is fatal: two independent bridge instances would each stamp their own
    // arrival jitter, so a left/right pair would never share a timestamp, and
    // a stereo matcher would silently pair frames that do not correspond and
    // produce garbage depth with no error.
    //
    // t_src_ns carries the ORIGINAL capture/simulation timestamp through the
    // ring untouched, so both eyes can be republished with one shared stamp.
    //
    // ABI: this occupies bytes that were previously _pad[3], so
    // sizeof(SlotHeader) is unchanged at 64 and every existing consumer, which
    // reads only the fields above, is unaffected. Readers must treat 0 as
    // "absent" and fall back to t_ns.
    uint64_t t_src_ns;
};
// Compile-time check: if this fails your layout is wrong
static_assert(sizeof(SlotHeader) == 64, "SlotHeader must be 64 bytes");

// ─────────────────────────────────────────────────────────────────
// GLOBAL HEADER — sits at the very start of the shared memory
//
// Contains metadata about the ring buffer and one critical field:
//   write_seq = the frame number of the last fully written frame
//
// The consumer reads write_seq to know which frames are ready.
// ─────────────────────────────────────────────────────────────────
struct alignas(64) ShmGlobal {
    uint32_t magic;       // must equal OVCAM_MAGIC
    uint32_t version;     // for future compatibility checks
    uint32_t slot_count;  // how many slots in the ring (e.g. 4)
    uint32_t _p0;         // padding for alignment

    uint64_t slot_size;        // bytes per slot (SlotHeader + pixel data)
    uint64_t max_frame_bytes;  // max pixel bytes per frame

    uint32_t width;    // configured capture width
    uint32_t height;   // configured capture height
    uint32_t stride;   // bytes per row
    uint32_t fourcc;   // pixel format

    // alignas(64) puts write_seq on its own cache line.
    // The producer writes this frequently; isolating it prevents
    // the consumer's reads of other fields from interfering.
    alignas(64) uint64_t write_seq;
};

// ─────────────────────────────────────────────────────────────────
// LAYOUT IN MEMORY (example: 4 slots)
//
//  [ ShmGlobal : 128 bytes         ]
//  [ SlotHeader | pixel data...    ]  ← slot 0
//  [ SlotHeader | pixel data...    ]  ← slot 1
//  [ SlotHeader | pixel data...    ]  ← slot 2
//  [ SlotHeader | pixel data...    ]  ← slot 3
// ─────────────────────────────────────────────────────────────────

// Returns pointer to slot i inside the shared memory block
inline SlotHeader* slot_at(void* base, uint64_t slot_size, uint32_t i) {
    uint8_t* b = static_cast<uint8_t*>(base);
    // skip the global header, then skip i complete slots
    return reinterpret_cast<SlotHeader*>(b + sizeof(ShmGlobal) + i * slot_size);
}

// Returns pointer to the pixel data that follows a slot's header
inline uint8_t* frame_data(SlotHeader* h) {
    return reinterpret_cast<uint8_t*>(h) + sizeof(SlotHeader);
}

// ─────────────────────────────────────────────────────────────────
// ATOMIC HELPERS
//
// We use GCC builtins instead of std::atomic because std::atomic
// is NOT safe in mmap'd shared memory — it may use address-based
// locks internally, which break when two processes map the same
// physical memory at different virtual addresses.
//
// __ATOMIC_ACQUIRE = "I'm reading — ensure I see all writes that
//                    happened before the matching RELEASE store"
// __ATOMIC_RELEASE = "I'm writing — ensure all my prior writes are
//                    visible before this store is seen"
// ─────────────────────────────────────────────────────────────────

// Read slot's seq with acquire ordering
inline uint64_t seq_load(const SlotHeader* h) {
    return __atomic_load_n(&h->seq, __ATOMIC_ACQUIRE);
}

// Write slot's seq with specified ordering
inline void seq_store(SlotHeader* h, uint64_t v, int order) {
    __atomic_store_n(&h->seq, v, order);
}

// Read global write_seq (consumer calls this to find latest frame)
inline uint64_t ws_load(const ShmGlobal* g) {
    return __atomic_load_n(&g->write_seq, __ATOMIC_ACQUIRE);
}

// Write global write_seq (producer calls this after completing a frame)
inline void ws_store(ShmGlobal* g, uint64_t v) {
    __atomic_store_n(&g->write_seq, v, __ATOMIC_RELEASE);
}
