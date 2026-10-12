#include "air10_stored.h"
#include "uart_arbiter.h"
#include <stdio.h>
#include <string.h>

namespace Air10Stored {
namespace {
struct Capture {
    char tag[4];
    uint16_t day;
    Value value;
    bool received;
    bool unsupported;
};

bool capture_value(const qframe_t *frame, void *context) {
    auto &capture = *static_cast<Capture *>(context);
    if (frame->type == QFRAME_TYPE_E) {
        capture.unsupported = unsupported_response(frame->payload,
            frame->payload_len, capture.tag, capture.day);
        return true;
    }
    capture.received = frame->type == QFRAME_TYPE_R &&
        parse_response(frame->payload, frame->payload_len,
                       capture.tag, capture.day, capture.value);
    return capture.received;
}
}

ReadResult read(const char *tag, uint16_t day, Value &out,
                uint16_t timeout_ms, bool background, uint16_t hard_timeout_ms) {
    out = {};
    if (!tag || strlen(tag) != 3 || !timeout_ms) return ReadResult::Failed;
    char command[28];
    snprintf(command, sizeof(command), "G V #%s %04X 0", tag, day);
    uart_response_policy_t policy = {};
    policy.accepted_types = QFRAME_MASK_R | QFRAME_MASK_E;
    policy.terminal_types = policy.accepted_types;
    policy.success_types = QFRAME_MASK_R;
    policy.first_timeout_ms = timeout_ms;
    policy.overall_timeout_ms = hard_timeout_ms;
    Capture capture = {};
    memcpy(capture.tag, tag, 3);
    capture.day = day;
    uart_transaction_result_t result = {};
    bool completed = Arbiter::transact_cmd(command, CMD_SRC_INTERNAL,
        background ? CMD_PRIO_LOW : CMD_PRIO_NORMAL, policy,
        capture_value, &capture, &result, sizeof(capture));
    if (completed && result.protocol_error && capture.unsupported)
        return ReadResult::Unsupported;
    if (!completed || !result.success || !capture.received)
        return ReadResult::Failed;
    out = capture.value;
    return ReadResult::Ok;
}
}
