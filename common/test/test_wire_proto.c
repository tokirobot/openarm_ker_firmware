#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* Mirrors PROTO_LOCK_STREAK in M5/include/RSNexus.h. */
#define PROTO_LOCK_STREAK_TEST 8
#include "KerWireProto.h"

/* Rebuild the 28-bit codeword the way it actually travels: header7 || payload21
 * (the CRC lives in the payload's bottom 6 bits). */
static uint32_t codeword(uint8_t header7, uint16_t angle15) {
    uint8_t  crc = ker_crc6(header7, angle15);
    uint32_t pay = ker_pack_payload21(angle15, crc);
    return ((uint32_t)(header7 & 0x7F) << 21) | pay;
}
static int codeword_ok(uint32_t cw) {
    uint8_t  header7 = (uint8_t)((cw >> 21) & 0x7F);
    uint32_t pay     = cw & KER_PAYLOAD_MASK;
    return ker_low6(pay) == ker_crc6(header7, ker_angle15(pay));
}

int main(void) {
    /* --- 1. round trip over every angle x every header --- */
    long checked = 0;
    for (uint32_t h = 0; h < 128; h++)
        for (uint32_t a = 0; a < 32768; a++) {
            uint32_t cw = codeword((uint8_t)h, (uint16_t)a);
            if (!codeword_ok(cw)) { printf("FAIL roundtrip h=%u a=%u\n", h, a); return 1; }
            if (ker_angle15(cw & KER_PAYLOAD_MASK) != a) { printf("FAIL angle recover\n"); return 1; }
            checked++;
        }
    printf("1. round trip + angle recovery : PASS (%ld codewords)\n", checked);

    /* --- 2. every single-bit error, over a wide sample of codewords --- */
    long miss1 = 0, tried1 = 0;
    for (uint32_t h = 0; h < 128; h += 1)
        for (uint32_t a = 0; a < 32768; a += 37) {
            uint32_t cw = codeword((uint8_t)h, (uint16_t)a);
            for (int b = 0; b < 28; b++) { tried1++; if (codeword_ok(cw ^ (1u << b))) miss1++; }
        }
    printf("2. single-bit errors           : %s (%ld tested, %ld undetected)\n",
           miss1 ? "FAIL" : "PASS", tried1, miss1);

    /* --- 3. every two-bit error --- */
    long miss2 = 0, tried2 = 0;
    for (uint32_t h = 0; h < 128; h += 8)
        for (uint32_t a = 0; a < 32768; a += 311) {
            uint32_t cw = codeword((uint8_t)h, (uint16_t)a);
            for (int i = 0; i < 28; i++)
                for (int j = i + 1; j < 28; j++) {
                    tried2++;
                    if (codeword_ok(cw ^ (1u << i) ^ (1u << j))) miss2++;
                }
        }
    printf("3. two-bit errors              : %s (%ld tested, %ld undetected)\n",
           miss2 ? "FAIL" : "PASS", tried2, miss2);

    /* --- 4. bursts up to 6 bits --- */
    long missb = 0, triedb = 0;
    for (uint32_t h = 0; h < 128; h += 8)
        for (uint32_t a = 0; a < 32768; a += 311) {
            uint32_t cw = codeword((uint8_t)h, (uint16_t)a);
            for (int start = 0; start <= 28 - 6; start++)
                for (uint32_t pat = 1; pat < 64; pat++) {
                    triedb++;
                    if (codeword_ok(cw ^ (pat << start))) missb++;
                }
        }
    printf("4. bursts <= 6 bits            : %s (%ld tested, %ld undetected)\n",
           missb ? "FAIL" : "PASS", triedb, missb);

    /* --- 5. random multi-bit corruption: expect ~1/64 = 1.56% --- */
    uint32_t rng = 12345; long undet = 0, n = 0;
    for (long k = 0; k < 4000000; k++) {
        rng = rng * 1664525u + 1013904223u;
        uint8_t  h = (uint8_t)(rng & 0x7F);
        uint16_t a = (uint16_t)((rng >> 7) & 0x7FFF);
        uint32_t cw = codeword(h, a);
        rng = rng * 1664525u + 1013904223u;
        uint32_t err = rng & 0x0FFFFFFFu;
        if (!err) continue;
        n++; if (codeword_ok(cw ^ err)) undet++;
    }
    printf("5. random corruption           : %.3f%% undetected (theory 1.563%%)\n",
           100.0 * (double)undet / (double)n);

    /* --- 6. does a legacy payload get mistaken for a CRC one? --- */
    long legacy_passes_crc = 0;
    for (uint32_t h = 0; h < 128; h++)
        for (uint32_t a = 0; a < 32768; a++) {
            if (ker_legacy_low6((uint16_t)a) == ker_crc6((uint8_t)h, (uint16_t)a))
                legacy_passes_crc++;
        }
    {
        double amb = (double)legacy_passes_crc / (128.0 * 32768.0);
        printf("6. legacy payload passes CRC   : %.3f%% ambiguous; %d in a row\n"
               "   misclassifies with p=%.2e\n",
               100.0 * amb, PROTO_LOCK_STREAK_TEST,
               pow(amb, PROTO_LOCK_STREAK_TEST));
    }

    /* --- 7. all-zero bus must not decode as a valid 0 deg reading --- */
    printf("7. silent bus (payload all 0)  : %s\n",
           codeword_ok(((uint32_t)0x04 << 21) | 0) ? "FAIL (accepted!)" : "PASS (rejected)");
    printf("   ker_crc6(header7=0x04, 0)   = 0x%02X (non-zero => zero payload is invalid)\n",
           ker_crc6(0x04, 0));
    return (miss1 || miss2 || missb) ? 1 : 0;
}
