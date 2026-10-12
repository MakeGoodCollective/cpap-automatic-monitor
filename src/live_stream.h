#pragma once
#include <Arduino.h>

// AirSense L-frame stream broker. Pluggable parsers + pluggable consumers.
//
// Concepts:
//   - A "stream" is a class of L-frame identified by a 3-char ASCII tag
//     (e.g. "PMD"). Each stream has a registered decode function.
//   - A "consumer" subscribes to a stream and receives decoded samples
//     via a callback. Multiple consumers may share a stream.
//   - Device-level subscription (`P S &<TAG> 1/0`) is refcount-managed:
//     first consumer triggers the on, last unsubscribe triggers the off.
//
// Callback contract:
//   Consumer callbacks run on the UART rx_task. They MUST return quickly:
//   target <1 ms, budget 2 ms. Do not log timings on the RX path.
//   Heavy work belongs in the consumer's own task.

namespace LiveStream {

#define LIVE_STREAM_TAG_LEN     3
#define LIVE_STREAMS_MAX        16
#define LIVE_CONSUMERS_MAX      8

typedef int8_t consumer_handle_t;       // -1 on failure, >=0 on success
typedef int8_t external_handle_t;
typedef int8_t internal_handle_t;

// Decode raw L-frame payload (including the 3-char tag) into a typed sample.
// Return false to drop the frame.
typedef bool (*decode_fn_t)(const uint8_t *payload, uint16_t len,
                            void *out, uint16_t out_size);
// Optional schema preparation on the control task before enabling a stream.
typedef bool (*prepare_fn_t)();

// Consumer callback. `sample` is decode_fn output; cast to your stream's
// sample struct. `ctx` is whatever you passed to subscribe(). Decoders and
// callbacks run on UART RX and must not block or change subscriptions.
typedef void (*consumer_cb_t)(const void *sample, uint16_t sample_size,
                              void *ctx);

void init();

// Register a parser for a stream tag. Returns true on success, false if the
// tag is already registered or the table is full. `tag` must be exactly 3
// ASCII chars (no NUL); only the first 3 are used.
bool register_stream(const char *tag, decode_fn_t decode_fn,
                     uint16_t sample_size, prepare_fn_t prepare_fn = nullptr);

// Subscribe to a registered stream. Refcount-managed: first subscribe sends
// `P S &<TAG> 1`; last unsubscribe sends `P S &<TAG> 0`. Returns a handle
// (>=0) or -1 on failure (unknown tag, consumer table full).
consumer_handle_t subscribe(const char *tag, consumer_cb_t cb, void *ctx);

// Unsubscribe; safe to call with handle == -1 (no-op).
void unsubscribe(consumer_handle_t h);

// Hold a raw stream subscription for an external framed client. The stream
// need not have a registered decoder; raw L frames are routed by the UART
// listener. Releasing the final reference disables the device stream.
external_handle_t acquire_external(const char *tag);
void release_external(external_handle_t h);

// Hold a raw stream subscription for a local service such as the EDF recorder.
internal_handle_t acquire_internal(const char *tag);
void release_internal(internal_handle_t h);

bool suspend();
bool suspend_for_ota();
void resume();
// A schema-ready notification retries only that stream, outside UART RX.
void resync(const char *tag = nullptr);
void reattach();

// Fed by uart_arbiter rx_task on every QFRAME_TYPE_L frame.
void on_l_frame(const uint8_t *payload, uint16_t len);

// True if the named stream currently has a live device subscription.
bool is_stream_active(const char *tag);

// True if any stream currently has a live device subscription.
bool is_active();

}
