#include "live_stream.h"
#include "uart_arbiter.h"
#include "debug_log.h"
#include <string.h>

#define LS_DEVICE_CMD_TIMEOUT   1000
#define LS_DECODE_BUF           64

namespace LiveStream {

struct Stream {
    char        tag[LIVE_STREAM_TAG_LEN + 1];
    decode_fn_t decode_fn;
    prepare_fn_t prepare_fn;
    uint16_t    sample_size;
    uint8_t     ref_count;
    bool        subscribed;
    bool        sync_pending;
    bool        in_use;
};

struct Consumer {
    bool          in_use;
    uint16_t      inflight;
    int8_t        stream_idx;
    consumer_cb_t cb;
    void         *ctx;
};

static Stream    streams[LIVE_STREAMS_MAX];
static Consumer  consumers[LIVE_CONSUMERS_MAX];
static bool      initialized = false;
static SemaphoreHandle_t control_mutex = nullptr;
static portMUX_TYPE data_mux = portMUX_INITIALIZER_UNLOCKED;

class ControlGuard {
public:
    ControlGuard() { xSemaphoreTake(control_mutex, portMAX_DELAY); }
    ~ControlGuard() { xSemaphoreGive(control_mutex); }
};

static int find_stream_idx(const char *tag) {
    for (int i = 0; i < LIVE_STREAMS_MAX; i++) {
        if (streams[i].in_use &&
            memcmp(streams[i].tag, tag, LIVE_STREAM_TAG_LEN) == 0) {
            return i;
        }
    }
    return -1;
}

static int create_stream(const char *tag, decode_fn_t decode_fn,
                         uint16_t sample_size, prepare_fn_t prepare_fn = nullptr) {
    for (int i = 0; i < LIVE_STREAMS_MAX; i++) {
        if (streams[i].in_use) continue;
        memcpy(streams[i].tag, tag, LIVE_STREAM_TAG_LEN);
        streams[i].tag[LIVE_STREAM_TAG_LEN] = 0;
        streams[i].decode_fn = decode_fn;
        streams[i].prepare_fn = prepare_fn;
        streams[i].sample_size = sample_size;
        portENTER_CRITICAL(&data_mux);
        streams[i].in_use = true;
        portEXIT_CRITICAL(&data_mux);
        Log::logf(CAT_STREAM, LOG_DEBUG,
                  "stream '%s' registered%s\n", streams[i].tag,
                  decode_fn ? "" : " (raw)");
        return i;
    }
    Log::logf(CAT_STREAM, LOG_WARN,
              "stream table full, '%c%c%c' rejected\n",
              tag[0], tag[1], tag[2]);
    return -1;
}

static bool device_subscribe(int sidx, bool on, cmd_source_t src = CMD_SRC_INTERNAL) {
    if (on && streams[sidx].prepare_fn && !streams[sidx].prepare_fn()) return false;
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "P S &%s %d", streams[sidx].tag, on ? 1 : 0);
    char resp[32] = {};
    uint16_t resp_len = sizeof(resp);
    bool ok = Arbiter::send_cmd(cmd, src, CMD_PRIO_NORMAL,
                                resp, &resp_len, LS_DEVICE_CMD_TIMEOUT);
    // A lost ACK leaves the device state unknown, even if the command ran.
    streams[sidx].sync_pending = !ok;
    if (ok) streams[sidx].subscribed = on;
    return ok;
}

void init() {
    if (initialized) return;
    control_mutex = xSemaphoreCreateMutex();
    if (!control_mutex) {
        Log::logf(CAT_STREAM, LOG_ERROR, "control mutex init failed\n");
        return;
    }
    memset(streams, 0, sizeof(streams));
    memset(consumers, 0, sizeof(consumers));
    initialized = true;
    Log::logf(CAT_STREAM, LOG_DEBUG, "broker init\n");
}

bool register_stream(const char *tag, decode_fn_t decode_fn, uint16_t sample_size,
                     prepare_fn_t prepare_fn) {
    if (!initialized) init();
    if (!initialized) return false;
    ControlGuard guard;
    if (!tag || !decode_fn || sample_size == 0 || sample_size > LS_DECODE_BUF)
        return false;
    int sidx = find_stream_idx(tag);
    if (sidx >= 0) {
        if (streams[sidx].decode_fn) return false;
        portENTER_CRITICAL(&data_mux);
        streams[sidx].decode_fn = decode_fn;
        streams[sidx].prepare_fn = prepare_fn;
        streams[sidx].sample_size = sample_size;
        portEXIT_CRITICAL(&data_mux);
        Log::logf(CAT_STREAM, LOG_DEBUG,
                  "stream '%s' decoder attached\n", streams[sidx].tag);
        return true;
    }
    return create_stream(tag, decode_fn, sample_size, prepare_fn) >= 0;
}

consumer_handle_t subscribe(const char *tag, consumer_cb_t cb, void *ctx) {
    if (!initialized) init();
    if (!initialized) return -1;
    ControlGuard guard;
    if (!cb) return -1;
    int sidx = find_stream_idx(tag);
    if (sidx < 0) {
        Log::logf(CAT_STREAM, LOG_WARN,
                  "subscribe to unknown tag '%c%c%c'\n",
                  tag[0], tag[1], tag[2]);
        return -1;
    }

    int cidx = -1;
    for (int i = 0; i < LIVE_CONSUMERS_MAX; i++) {
        if (!consumers[i].in_use && consumers[i].inflight == 0) {
            cidx = i;
            break;
        }
    }
    if (cidx < 0) {
        Log::logf(CAT_STREAM, LOG_WARN, "consumer table full\n");
        return -1;
    }

    portENTER_CRITICAL(&data_mux);
    consumers[cidx].stream_idx = (int8_t)sidx;
    consumers[cidx].cb         = cb;
    consumers[cidx].ctx        = ctx;
    consumers[cidx].in_use     = true;

    streams[sidx].ref_count++;
    portEXIT_CRITICAL(&data_mux);
    if (streams[sidx].ref_count == 1 &&
        (!streams[sidx].subscribed || streams[sidx].sync_pending)) {
        if (device_subscribe(sidx, true)) {
            streams[sidx].subscribed = true;
            Log::logf(CAT_STREAM, LOG_DEBUG,
                      "%s subscribed (refs=1)\n", streams[sidx].tag);
        } else {
            Log::logf(CAT_STREAM, LOG_DEBUG,
                      "%s subscribe failed (will retry on resync)\n",
                      streams[sidx].tag);
        }
    }
    return (consumer_handle_t)cidx;
}

void unsubscribe(consumer_handle_t h) {
    if (!initialized) return;
    ControlGuard guard;
    if (h < 0 || h >= LIVE_CONSUMERS_MAX) return;
    if (!consumers[h].in_use) return;

    int sidx = consumers[h].stream_idx;
    portENTER_CRITICAL(&data_mux);
    consumers[h].in_use = false;
    portEXIT_CRITICAL(&data_mux);

    while (true) {
        portENTER_CRITICAL(&data_mux);
        bool idle = consumers[h].inflight == 0;
        if (idle) {
            consumers[h].cb = nullptr;
            consumers[h].ctx = nullptr;
        }
        portEXIT_CRITICAL(&data_mux);
        if (idle) break;
        vTaskDelay(1);
    }

    if (sidx >= 0 && sidx < LIVE_STREAMS_MAX && streams[sidx].in_use) {
        portENTER_CRITICAL(&data_mux);
        if (streams[sidx].ref_count > 0) streams[sidx].ref_count--;
        portEXIT_CRITICAL(&data_mux);
        if (streams[sidx].ref_count == 0 &&
            (streams[sidx].subscribed || streams[sidx].sync_pending)) {
            if (device_subscribe(sidx, false)) {
                streams[sidx].subscribed = false;
                Log::logf(CAT_STREAM, LOG_DEBUG,
                          "%s unsubscribed (refs=0)\n", streams[sidx].tag);
            } else {
                Log::logf(CAT_STREAM, LOG_WARN,
                          "%s unsubscribe failed (refs=0)\n", streams[sidx].tag);
            }
        }
    }
}

static int8_t acquire_raw(const char *tag, cmd_source_t source,
                          const char *owner) {
    if (!initialized) init();
    if (!initialized) return -1;
    ControlGuard guard;
    if (!tag || !tag[0] || !tag[1] || !tag[2]) return -1;

    int sidx = find_stream_idx(tag);
    if (sidx < 0) sidx = create_stream(tag, nullptr, 0);
    if (sidx < 0 || streams[sidx].ref_count == UINT8_MAX) return -1;

    if (!streams[sidx].subscribed || streams[sidx].sync_pending) {
        if (!device_subscribe(sidx, true, source)) {
            Log::logf(CAT_STREAM, LOG_WARN,
                      "%s subscribe to %s failed\n",
                      owner, streams[sidx].tag);
            // Keep the slot so resync can stop a stream whose ACK was lost.
            return -1;
        }
        streams[sidx].subscribed = true;
    }
    streams[sidx].ref_count++;
    Log::logf(CAT_STREAM, LOG_DEBUG,
              "%s %s lease acquired (refs=%u)\n",
              streams[sidx].tag, owner, streams[sidx].ref_count);
    return (int8_t)sidx;
}

static void release_raw(int8_t h, cmd_source_t source, const char *owner) {
    if (!initialized) return;
    ControlGuard guard;
    if (h < 0 || h >= LIVE_STREAMS_MAX || !streams[h].in_use ||
        streams[h].ref_count == 0) {
        return;
    }

    streams[h].ref_count--;
    if (streams[h].ref_count == 0 &&
        (streams[h].subscribed || streams[h].sync_pending)) {
        if (device_subscribe(h, false, source)) {
            streams[h].subscribed = false;
        } else {
            Log::logf(CAT_STREAM, LOG_WARN,
                      "%s unsubscribe from %s failed\n",
                      owner, streams[h].tag);
        }
    }
    Log::logf(CAT_STREAM, LOG_DEBUG,
              "%s %s lease released (refs=%u)\n",
              streams[h].tag, owner, streams[h].ref_count);
    if (streams[h].ref_count == 0 && !streams[h].subscribed &&
        !streams[h].sync_pending &&
        !streams[h].decode_fn) {
        portENTER_CRITICAL(&data_mux);
        memset(&streams[h], 0, sizeof(streams[h]));
        portEXIT_CRITICAL(&data_mux);
    }
}

external_handle_t acquire_external(const char *tag) {
    return acquire_raw(tag, CMD_SRC_TCP, "external");
}

void release_external(external_handle_t h) {
    release_raw(h, CMD_SRC_TCP, "external");
}

internal_handle_t acquire_internal(const char *tag) {
    return acquire_raw(tag, CMD_SRC_INTERNAL, "internal");
}

void release_internal(internal_handle_t h) {
    release_raw(h, CMD_SRC_INTERNAL, "internal");
}

static bool suspend_with_source(cmd_source_t src) {
    if (!initialized) return true;
    ControlGuard guard;
    bool ok = true;
    for (int i = 0; i < LIVE_STREAMS_MAX; i++) {
        if (!streams[i].in_use ||
            (!streams[i].subscribed && !streams[i].sync_pending)) continue;
        if (device_subscribe(i, false, src)) {
            streams[i].subscribed = false;
            Log::logf(CAT_STREAM, LOG_DEBUG, "%s suspended\n", streams[i].tag);
        } else {
            ok = false;
            Log::logf(CAT_STREAM, LOG_ERROR,
                      "%s suspend failed\n", streams[i].tag);
        }
    }
    return ok;
}

bool suspend() {
    return suspend_with_source(CMD_SRC_INTERNAL);
}

bool suspend_for_ota() {
    return suspend_with_source(CMD_SRC_OTA);
}

void resume() {
    if (!initialized) return;
    ControlGuard guard;
    for (int i = 0; i < LIVE_STREAMS_MAX; i++) {
        if (!streams[i].in_use) continue;
        if (streams[i].ref_count > 0 &&
            (!streams[i].subscribed || streams[i].sync_pending)) {
            if (device_subscribe(i, true)) {
                streams[i].subscribed = true;
                Log::logf(CAT_STREAM, LOG_DEBUG,
                          "%s resumed\n", streams[i].tag);
            }
        }
    }
}

void resync(const char *tag) {
    if (!initialized) return;
    ControlGuard guard;
    for (int i = 0; i < LIVE_STREAMS_MAX; i++) {
        if (!streams[i].in_use) continue;
        if (tag && strcmp(streams[i].tag, tag)) continue;
        if (streams[i].ref_count > 0 &&
            (!streams[i].subscribed || streams[i].sync_pending)) {
            if (device_subscribe(i, true)) {
                streams[i].subscribed = true;
                Log::logf(CAT_STREAM, LOG_DEBUG,
                          "%s re-subscribed\n", streams[i].tag);
            }
        } else if (streams[i].ref_count == 0 &&
                   (streams[i].subscribed || streams[i].sync_pending)) {
            if (device_subscribe(i, false)) {
                streams[i].subscribed = false;
                Log::logf(CAT_STREAM, LOG_DEBUG,
                          "%s unsubscribed (retry)\n", streams[i].tag);
            }
        }
        if (streams[i].ref_count == 0 && !streams[i].subscribed &&
            !streams[i].sync_pending && !streams[i].decode_fn) {
            portENTER_CRITICAL(&data_mux);
            memset(&streams[i], 0, sizeof(streams[i]));
            portEXIT_CRITICAL(&data_mux);
        }
    }
}

void reattach() {
    if (!initialized) return;
    ControlGuard guard;
    for (int i = 0; i < LIVE_STREAMS_MAX; i++) {
        if (!streams[i].in_use || streams[i].ref_count == 0) continue;
        streams[i].subscribed = false;
        if (device_subscribe(i, true)) {
            streams[i].subscribed = true;
            Log::logf(CAT_STREAM, LOG_DEBUG,
                      "%s reattached\n", streams[i].tag);
        }
    }
}

void on_l_frame(const uint8_t *payload, uint16_t len) {
    if (!initialized || len < LIVE_STREAM_TAG_LEN) return;

    int sidx = -1;
    decode_fn_t decode_fn = nullptr;
    uint16_t sample_size = 0;
    consumer_cb_t callbacks[LIVE_CONSUMERS_MAX] = {};
    void *contexts[LIVE_CONSUMERS_MAX] = {};
    uint8_t callback_slots[LIVE_CONSUMERS_MAX] = {};
    uint8_t callback_count = 0;

    portENTER_CRITICAL(&data_mux);
    for (int i = 0; i < LIVE_STREAMS_MAX; i++) {
        if (streams[i].in_use &&
            memcmp(streams[i].tag, payload, LIVE_STREAM_TAG_LEN) == 0) {
            sidx = i;
            break;
        }
    }
    if (sidx >= 0) {
        decode_fn = streams[sidx].decode_fn;
        sample_size = streams[sidx].sample_size;
        if (decode_fn && sample_size > 0) {
            for (uint8_t i = 0; i < LIVE_CONSUMERS_MAX; i++) {
                if (!consumers[i].in_use || consumers[i].stream_idx != sidx ||
                    !consumers[i].cb) {
                    continue;
                }
                callbacks[callback_count] = consumers[i].cb;
                contexts[callback_count] = consumers[i].ctx;
                callback_slots[callback_count] = i;
                consumers[i].inflight++;
                callback_count++;
            }
        }
    }
    portEXIT_CRITICAL(&data_mux);

    if (sidx < 0 || !decode_fn || sample_size == 0) return;

    // Decode once into a stack buffer, then fan out to consumers.
    uint8_t sample_buf[LS_DECODE_BUF];
    bool decoded = sample_size <= sizeof(sample_buf) &&
                   decode_fn(payload, len, sample_buf, sample_size);

    for (uint8_t i = 0; i < callback_count; i++) {
        if (decoded) {
            callbacks[i](sample_buf, sample_size, contexts[i]);
        }
        portENTER_CRITICAL(&data_mux);
        Consumer &consumer = consumers[callback_slots[i]];
        if (consumer.inflight > 0) consumer.inflight--;
        portEXIT_CRITICAL(&data_mux);
    }
}

bool is_stream_active(const char *tag) {
    if (!initialized) return false;
    ControlGuard guard;
    int sidx = find_stream_idx(tag);
    return sidx >= 0 && streams[sidx].subscribed;
}

bool is_active() {
    if (!initialized) return false;
    ControlGuard guard;
    for (int i = 0; i < LIVE_STREAMS_MAX; i++) {
        if (streams[i].in_use && streams[i].subscribed) return true;
    }
    return false;
}

}
