#include "ota_url_client.h"
#include "debug_log.h"
#include "memory_manager.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include <esp_crt_bundle.h>
#include <esp_http_client.h>
#include <esp_heap_caps.h>
#include <esp_tls_errors.h>

namespace OtaUrl {
namespace {

static constexpr char USER_AGENT[] = "AirBridge OTA";
static constexpr int HTTP_TIMEOUT_MS = 15000;
static constexpr int HTTP_BUFFER_BYTES = 1024;
static constexpr int HTTP_TX_BUFFER_BYTES = 1024;
static constexpr int REDIRECT_LIMIT = 5;

struct StreamContext {
    size_t expected_size = 0;
    size_t offset = 0;
    bool size_checked = false;
    WriteCallback write_callback = nullptr;
    ContinueCallback continue_callback = nullptr;
    void *callback_ctx = nullptr;
    Error *error = nullptr;
};

struct FetchContext {
    uint8_t *buffer = nullptr;
    size_t capacity = 0;
    size_t offset = 0;
    size_t expected_size = 0;
    size_t internal_fallback_limit = SIZE_MAX;
    bool size_checked = false;
    ContinueCallback continue_callback = nullptr;
    void *callback_ctx = nullptr;
    Error *error = nullptr;
};

void set_error(Error &error, const char *code) {
    snprintf(error.code, sizeof(error.code), "%s", code ? code : "url_error");
}

bool allowed(ContinueCallback callback, void *ctx) {
    return !callback || callback(ctx);
}

esp_http_client_handle_t create_client(const char *url,
                                       esp_http_client_method_t method,
                                       http_event_handle_cb handler,
                                       void *user_data) {
    esp_http_client_config_t config = {};
    config.url = url;
    config.user_agent = USER_AGENT;
    config.method = method;
    config.timeout_ms = HTTP_TIMEOUT_MS;
    config.disable_auto_redirect = false;
    config.max_redirection_count = REDIRECT_LIMIT;
    config.max_authorization_retries = -1;
    config.event_handler = handler;
    config.buffer_size = HTTP_BUFFER_BYTES;
    config.buffer_size_tx = HTTP_TX_BUFFER_BYTES;
    config.user_data = user_data;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.keep_alive_enable = false;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client) {
        esp_http_client_set_header(client, "Accept-Encoding", "identity");
        esp_http_client_set_header(client, "Connection", "close");
    }
    return client;
}

void capture_transport_error(esp_http_client_handle_t client,
                             esp_err_t result, Error &error) {
    error.esp_error = (int)result;
    error.socket_error = esp_http_client_get_errno(client);
    int tls_error = 0;
    int tls_flags = 0;
    esp_err_t tls_result = esp_http_client_get_and_clear_last_tls_error(
        client, &tls_error, &tls_flags);
    error.tls_error = tls_error;
    error.tls_flags = tls_flags;
    Log::logf(CAT_OTA, LOG_DEBUG,
              "transport: esp=0x%x tls=0x%x mbed=%d flags=0x%x errno=%d\n",
              (unsigned)result, (unsigned)tls_result, tls_error,
              (unsigned)tls_flags, error.socket_error);
    // Failure-time availability, not the handshake's peak allocation.
    Log::logf(CAT_OTA, LOG_DEBUG,
              "transport context: internal=%u largest=%u epoch=%lld\n",
              (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
              (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
              (long long)time(nullptr));
    if (error.code[0]) return;

    if (result == ESP_ERR_HTTP_MAX_REDIRECT) set_error(error, "url_too_many_redirects");
    else if (tls_result == ESP_ERR_ESP_TLS_CANNOT_RESOLVE_HOSTNAME) set_error(error, "url_dns_failed");
    else if (tls_result != ESP_OK || tls_error || tls_flags) set_error(error, "url_tls_failed");
    else if (result == ESP_ERR_HTTP_READ_TIMEOUT || result == ESP_ERR_HTTP_EAGAIN ||
             error.socket_error == ETIMEDOUT) set_error(error, "url_timeout");
    else if (result == ESP_ERR_HTTP_CONNECT) set_error(error, "url_connect_failed");
    else if (result == ESP_ERR_HTTP_FETCH_HEADER) set_error(error, "url_header_failed");
    else if (result == ESP_ERR_HTTP_INCOMPLETE_DATA ||
             result == ESP_ERR_HTTP_CONNECTION_CLOSED) set_error(error, "url_incomplete_response");
    else set_error(error, "url_transport_failed");
}

esp_err_t stream_event(esp_http_client_event_t *event) {
    if (!event || !event->user_data || event->event_id != HTTP_EVENT_ON_DATA ||
        event->data_len <= 0) return ESP_OK;
    StreamContext &ctx = *(StreamContext *)event->user_data;
    if (esp_http_client_get_status_code(event->client) != 200) return ESP_OK;
    if (!allowed(ctx.continue_callback, ctx.callback_ctx)) {
        set_error(*ctx.error, "url_cancelled");
        return ESP_FAIL;
    }
    if (!ctx.size_checked) {
        int64_t length = esp_http_client_get_content_length(event->client);
        if (length > 0 && (uint64_t)length != ctx.expected_size) {
            set_error(*ctx.error, "url_size_changed");
            return ESP_FAIL;
        }
        ctx.size_checked = true;
    }

    size_t len = (size_t)event->data_len;
    if (ctx.offset > ctx.expected_size || len > ctx.expected_size - ctx.offset) {
        set_error(*ctx.error, "url_response_too_large");
        return ESP_FAIL;
    }
    if (!ctx.write_callback || !ctx.write_callback(
            ctx.callback_ctx, ctx.offset, (const uint8_t *)event->data, len)) {
        if (!ctx.error->code[0]) set_error(*ctx.error, "url_ota_write_failed");
        return ESP_FAIL;
    }
    ctx.offset += len;
    return ESP_OK;
}

esp_err_t fetch_event(esp_http_client_event_t *event) {
    if (!event || !event->user_data || event->event_id != HTTP_EVENT_ON_DATA ||
        event->data_len <= 0) return ESP_OK;
    FetchContext &ctx = *(FetchContext *)event->user_data;
    if (ctx.error->code[0]) return ESP_FAIL;
    if (esp_http_client_get_status_code(event->client) != 200) return ESP_OK;
    if (!allowed(ctx.continue_callback, ctx.callback_ctx)) {
        set_error(*ctx.error, "url_cancelled");
        return ESP_FAIL;
    }
    if (!ctx.size_checked) {
        int64_t length = esp_http_client_get_content_length(event->client);
        if (length <= 0) {
            set_error(*ctx.error, "url_content_length_missing");
            return ESP_FAIL;
        }
        if ((uint64_t)length > ctx.capacity) {
            set_error(*ctx.error, "url_response_too_large");
            return ESP_FAIL;
        }
        ctx.expected_size = (size_t)length;
        ctx.buffer = static_cast<uint8_t *>(aircannect::Memory::alloc_large(
            ctx.expected_size, ctx.expected_size <= ctx.internal_fallback_limit));
        if (!ctx.buffer) {
            set_error(*ctx.error, "manifest_alloc_failed");
            return ESP_FAIL;
        }
        ctx.size_checked = true;
    }
    size_t len = (size_t)event->data_len;
    if (ctx.offset > ctx.expected_size || len > ctx.expected_size - ctx.offset) {
        set_error(*ctx.error, "url_response_too_large");
        return ESP_FAIL;
    }
    memcpy(ctx.buffer + ctx.offset, event->data, len);
    ctx.offset += len;
    return ESP_OK;
}

}  // namespace

bool supported(const char *url) {
    if (!url || !*url) return false;
    bool http = strncasecmp(url, "http://", 7) == 0;
    bool https = strncasecmp(url, "https://", 8) == 0;
    if (!http && !https) return false;
    const char *host = url + (https ? 8 : 7);
    if (!*host) return false;
    for (const unsigned char *p = (const unsigned char *)url; *p; p++)
        if (*p <= 0x20 || *p == 0x7f) return false;
    return true;
}

bool stream(const char *url, size_t expected_size,
            WriteCallback write_callback, ContinueCallback continue_callback,
            void *callback_ctx, Error &error) {
    error = {};
    if (!supported(url) || expected_size == 0 || !write_callback) {
        set_error(error, "url_invalid_request");
        return false;
    }

    StreamContext context;
    context.expected_size = expected_size;
    context.write_callback = write_callback;
    context.continue_callback = continue_callback;
    context.callback_ctx = callback_ctx;
    context.error = &error;
    esp_http_client_handle_t client = create_client(
        url, HTTP_METHOD_GET, stream_event, &context);
    if (!client) {
        set_error(error, "url_client_alloc_failed");
        return false;
    }

    esp_err_t result = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    error.http_status = status;
    if (result != ESP_OK) capture_transport_error(client, result, error);
    else if (status != 200) snprintf(error.code, sizeof(error.code), "url_http_%d", status);
    else if (context.offset != expected_size) set_error(error, "url_incomplete_response");
    esp_http_client_cleanup(client);
    return result == ESP_OK && status == 200 && context.offset == expected_size;
}

bool fetch(const char *url, uint8_t *&buffer, size_t capacity, size_t &length,
           Error &error, ContinueCallback continue_callback,
           void *callback_ctx, size_t internal_fallback_limit) {
    length = 0;
    buffer = nullptr;
    error = {};
    if (!supported(url) || capacity == 0) {
        set_error(error, "url_invalid_request");
        return false;
    }

    FetchContext context;
    context.capacity = capacity;
    context.internal_fallback_limit = internal_fallback_limit;
    context.continue_callback = continue_callback;
    context.callback_ctx = callback_ctx;
    context.error = &error;
    esp_http_client_handle_t client = create_client(
        url, HTTP_METHOD_GET, fetch_event, &context);
    if (!client) {
        set_error(error, "url_client_alloc_failed");
        return false;
    }

    esp_err_t result = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    error.http_status = status;
    if (result != ESP_OK) capture_transport_error(client, result, error);
    else if (status != 200) snprintf(error.code, sizeof(error.code), "url_http_%d", status);
    else if (!context.size_checked || context.offset != context.expected_size)
        set_error(error, "url_incomplete_response");
    else if (!error.code[0]) length = context.offset;
    esp_http_client_cleanup(client);
    if (length) buffer = context.buffer;
    else heap_caps_free(context.buffer);
    return length > 0;
}

}  // namespace OtaUrl
