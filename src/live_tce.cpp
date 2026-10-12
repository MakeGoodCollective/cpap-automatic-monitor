#include "live_tce.h"
#include "air10_stream.h"
#include "live_stream.h"
#include <Arduino.h>

namespace LiveTce {

const char TAG[4] = "TCE";
const uint16_t SAMPLE_SIZE = sizeof(Sample);

static bool prepare() {
    Air10Stream::request(TAG);
    Air10Stream::Schema current;
    return Air10Stream::schema(TAG, current);
}

static bool decode(const uint8_t *payload, uint16_t len,
                   void *out, uint16_t out_size) {
    if (out_size < sizeof(Sample)) return false;
    Air10Stream::Schema current;
    if (!Air10Stream::schema(TAG, current)) return false;
    Air10Stream::Frame frame;
    if (!Air10Stream::decode_frame(payload, len, current, frame)) return false;
    auto *sample = static_cast<Sample *>(out);
    if (!Air10Stream::decoded_value(current, frame, "MKP", sample->mkp) ||
        !Air10Stream::decoded_value(current, frame, "RFL", sample->rfl) ||
        !Air10Stream::decoded_value(current, frame, "LYK", sample->lyk)) return false;
    sample->ts_ms = millis();
    return true;
}

void register_parser() {
    LiveStream::register_stream(TAG, decode, SAMPLE_SIZE, prepare);
}

}  // namespace LiveTce
