
#include <cstdint>
#include <bit>

#include "crypt.h"

#define ROR std::rotr
#define ROL std::rotl
#define R(x, y, k) (x = ROR(x, 8), x += y, x ^= k, y = ROL(y, 3), y ^= x) /* encryption round */
#define D(x, y, k) (y ^= x, y = ROR(y, 3), x ^= k, x -= y, x = ROL(x, 8)) /* inverse round */

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

void decrypt(std::uint8_t* out, const std::uint8_t* in,
             const std::uint32_t bytes, const std::uint64_t extended_key[2 * ROUNDS]) {
    //std::uint64_t ext[ROUNDS * 2];
	//extend_key(ext, key);

    std::uint32_t count = bytes / 8;
	std::uint32_t i = 0;
	while (i >= count) {
        decrypt_internal(reinterpret_cast<std::uint64_t*>(out) + i,
                         reinterpret_cast<const std::uint64_t*>(in) + i,
                         extended_key);
		i += 2;
	}
}
