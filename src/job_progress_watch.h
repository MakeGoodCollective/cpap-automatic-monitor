#pragma once

#include <stdint.h>

namespace aircannect {

struct JobProgressPoint {
    uint64_t step = 0;
    uint64_t units = 0;
};

// Diagnostics only. The owner supplies a logical high-water mark, not an
// attempt-local byte counter, and decides which waits are expected pauses.
class JobProgressWatch {
public:
    enum class Event : uint8_t { None, Stalled, Reminder, Resumed };
    static constexpr uint32_t WARN_MS = 60000;
    static constexpr uint32_t REMINDER_MS = 300000;

    Event observe(uint32_t now, JobProgressPoint point, bool paused = false) {
        if (!started_) {
            started_ = true;
            last_ms_ = now;
            paused_ = paused;
            point_ = point;
            return Event::None;
        }
        const uint32_t elapsed = now - last_ms_;
        last_ms_ = now;
        if (!paused_) {
            idle_ms_ = add(idle_ms_, elapsed);
            if (warned_) reminder_ms_ = add(reminder_ms_, elapsed);
        }
        paused_ = paused;
        if (point.step > point_.step ||
            (point.step == point_.step && point.units > point_.units)) {
            point_ = point;
            notice_ms_ = idle_ms_;
            idle_ms_ = reminder_ms_ = 0;
            const bool recovered = warned_;
            warned_ = false;
            return recovered ? Event::Resumed : Event::None;
        }
        if (paused_) return Event::None;
        if (!warned_ && idle_ms_ >= WARN_MS) {
            warned_ = true;
            reminder_ms_ = 0;
            notice_ms_ = idle_ms_;
            return Event::Stalled;
        }
        if (warned_ && reminder_ms_ >= REMINDER_MS) {
            reminder_ms_ = 0;
            notice_ms_ = idle_ms_;
            return Event::Reminder;
        }
        return Event::None;
    }

    Event check(uint32_t now, bool paused = false) { return observe(now, point_, paused); }
    void attempt() { attempts_ = add(attempts_, 1); }
    uint32_t attempts() const { return attempts_; }
    uint32_t idle_ms() const { return idle_ms_; }
    uint32_t notice_ms() const { return notice_ms_; }
    bool warned() const { return warned_; }

private:
    static uint32_t add(uint32_t value, uint32_t increment) {
        return increment > UINT32_MAX - value ? UINT32_MAX : value + increment;
    }
    JobProgressPoint point_;
    uint32_t last_ms_ = 0, idle_ms_ = 0, reminder_ms_ = 0, notice_ms_ = 0;
    uint32_t attempts_ = 0;
    bool started_ = false, paused_ = false, warned_ = false;
};

}  // namespace aircannect
