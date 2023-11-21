/**
 * @file
 * @ingroup crypto
 *
 * @brief  ED25519 signature implementation using CryptoCell.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2023
 */

#include <stdint.h>
#include <string.h>

#include <nrf.h>
#include "ed25519.h"
#include "utils.h"

#include "nrf_cc310/include/crys_ec_edw_api.h"

size_t crypto_ed25519_sign(uint8_t *signature, const uint8_t *data, size_t data_len, const uint8_t *private_key, const uint8_t *public_key) {
    CRYS_ECEDW_TempBuff_t tmp;
    size_t                signature_len                                                           = 64;
    uint8_t               secret_key[CRYS_ECEDW_ORD_SIZE_IN_BYTES + CRYS_ECEDW_MOD_SIZE_IN_BYTES] = { 0 };
    uint8_t               secret_key_len                                                          = CRYS_ECEDW_ORD_SIZE_IN_BYTES + CRYS_ECEDW_MOD_SIZE_IN_BYTES;

    memcpy(secret_key, private_key, CRYS_ECEDW_ORD_SIZE_IN_BYTES);
    memcpy(&secret_key[CRYS_ECEDW_ORD_SIZE_IN_BYTES], public_key, CRYS_ECEDW_MOD_SIZE_IN_BYTES);
    crypto_enable_cryptocell();
    CRYSError_t result = CRYS_ECEDW_Sign(signature, &signature_len, data, data_len, secret_key, secret_key_len, &tmp);
    crypto_disable_cryptocell();
    if (result != CRYS_OK) {
        return 0;
    }
    return signature_len;
}

bool crypto_ed25519_verify(const uint8_t *signature, size_t signature_len, const uint8_t *data, size_t data_len, const uint8_t *public_key) {
    CRYS_ECEDW_TempBuff_t tmp;
    crypto_enable_cryptocell();
    CRYSError_t result = CRYS_ECEDW_Verify(signature, signature_len, public_key, CRYS_ECEDW_MOD_SIZE_IN_BYTES, (uint8_t *)data, data_len, &tmp);
    crypto_disable_cryptocell();

    return result == CRYS_OK;
}
