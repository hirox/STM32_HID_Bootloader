
#include <cstdint>
#include <cstdio>
#include <bit>

#include "crypt.h"

static void dump_plain(const std::uint64_t* in, const std::uint64_t extended_key[2 * ROUNDS]) {
    printf("Key: ");
    for (auto x = 0; x < 2 * ROUNDS; x++) {
        printf("0x%08llx ", extended_key[x]);
    }
    printf("\nPlain: ");
    for (auto x = 0; x < 10; x++) {
        printf("0x%08llx ", in[x]);
    }
    printf("\n");
}

static void dump_crypt(std::uint64_t* out) {
    printf("Crypt: ");
    for (auto x = 0; x < 10; x++) {
        printf("0x%08llx ", out[x]);
    }
    printf("\n");
}

static void speck128_round(std::uint64_t* x, std::uint64_t* y, std::uint64_t k) {
    *x = std::rotr(*x, 8);
    *x += *y;
    *x ^= k;
    *y = std::rotl(*y, 3);
    *y ^= *x;
}

static void speck128_invround(std::uint64_t* x, std::uint64_t* y, std::uint64_t k) {
    *y ^= *x;
    *y = std::rotr(*y, 3);
    *x ^= k;
    *x -= *y;
    *x = std::rotl(*x, 8);
}

static void decrypt_internal(
        std::uint64_t pt[2],
        const std::uint64_t ct[2],
        const std::uint64_t K[ROUNDS * 2]) {
   std::uint64_t y = ct[0];
   std::uint64_t x = ct[1];

   for (std::int32_t i = ROUNDS - 1; i >= 0; i--) {
	  speck128_invround(&x, &y, K[i * 2 + 1]);
   }

   pt[0] = y;
   pt[1] = x;
}

static void encrypt_internal(
        std::uint64_t ct[2],        // crypted text
        const std::uint64_t pt[2],  // plain text
        const std::uint64_t K[2])   // extended key
{
   std::uint64_t y = pt[0];
   std::uint64_t x = pt[1];

   for (std::int32_t i = 0; i < ROUNDS; i++) {
	  speck128_round(&x, &y, K[i * 2 + 1]);
   }

   ct[0] = y;
   ct[1] = x;
}

void extend_key(std::uint64_t kb[2 * ROUNDS], 
	 	        const std::uint64_t K[2]) {
	std::uint64_t i, a, b;
	b = K[1], a = K[0];
	for (i = 0; i < ROUNDS; i++) {
		kb[i * 2] = b;
		kb[i * 2 + 1] = a;
		speck128_round(&b, &a, i);
   }
}

void encrypt(std::uint8_t* out, const std::uint8_t* in,
             const std::uint32_t bytes, const std::uint64_t extended_key[2 * ROUNDS]) {
#if 0
    dump_plain(reinterpret_cast<const std::uint64_t*>(in), extended_key);
#endif

    std::uint32_t count = bytes / 8;
	std::uint32_t i = 0;
	while (i < count) {
        encrypt_internal(reinterpret_cast<std::uint64_t*>(out) + i,
                         reinterpret_cast<const std::uint64_t*>(in) + i,
                         extended_key);
		i += 2;
	}

#if 0
    dump_crypt(reinterpret_cast<std::uint64_t*>(out));
#endif
}

void decrypt(std::uint8_t* out, const std::uint8_t* in,
             const std::uint32_t bytes, const std::uint64_t extended_key[2 * ROUNDS]) {
    std::uint32_t count = bytes / 8;
	std::uint32_t i = 0;
	while (i < count) {
        decrypt_internal(reinterpret_cast<std::uint64_t*>(out) + i,
                         reinterpret_cast<const std::uint64_t*>(in) + i,
                         extended_key);
		i += 2;
	}
}
