#pragma once

#include <stddef.h>
#include <stdint.h>

namespace OtaUrl {

static constexpr size_t ERROR_CODE_MAX = 48;

struct Error {
    char code[ERROR_CODE_MAX] = {};
    int http_status = 0;
    int socket_error = 0;
    int esp_error = 0;
    int tls_error = 0;
    int tls_flags = 0;
};

using WriteCallback =
    bool (*)(void *ctx, size_t offset, const uint8_t *data, size_t len);
using ContinueCallback = bool (*)(void *ctx);

bool supported(const char *url);
bool stream(const char *url, size_t expected_size,
            WriteCallback write_callback,
            ContinueCallback continue_callback,
            void *callback_ctx, Error &error);
// On success the caller owns buffer and releases it with heap_caps_free.
bool fetch(const char *url, uint8_t *&buffer, size_t capacity, size_t &length,
           Error &error, ContinueCallback continue_callback = nullptr,
           void *callback_ctx = nullptr, size_t internal_fallback_limit = SIZE_MAX);

}  // namespace OtaUrl
