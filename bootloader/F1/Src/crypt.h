#pragma once

#define ROUNDS 32

#ifdef __cplusplus
extern "C" {
#endif

void extend_key(uint64_t kb[2 * ROUNDS], 
	 	        const uint64_t K[2]);

void encrypt(uint8_t* out, const uint8_t* in,
             const uint32_t bytes, const uint64_t extended_key[2 * ROUNDS]);

void decrypt(uint8_t* out, const uint8_t* in,
             const uint32_t bytes, const uint64_t extended_key[2 * ROUNDS]);

#ifdef __cplusplus
}
#endif