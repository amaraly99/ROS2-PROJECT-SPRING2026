// Quick compile test for yolo_shm.hpp — verifies layouts at build time.
#include "yolo_shm.hpp"
#include <cstdio>

int main() {
    printf("YoloShmGlobal : %zu bytes\n", sizeof(YoloShmGlobal));
    printf("YoloSlotHeader: %zu bytes\n", sizeof(YoloSlotHeader));
    printf("YoloDet       : %zu bytes\n", sizeof(YoloDet));
    printf("DETS_OFFSET   : %zu\n",       YOLO_DETS_OFFSET);
    printf("IMG_OFFSET    : %zu\n",        YOLO_IMG_OFFSET);
    printf("OK\n");
    return 0;
}
