#include "async_deferred_response.h"

#include <new>
#include <utility>

#include "async_tcp_dispatch.h"

namespace aircannect {

AsyncDeferredResponse::State::~State() { cancel(); }

bool AsyncDeferredResponse::State::publish(
    std::unique_ptr<AsyncWebServerResponse> response) {
    if (cancelled()) return false;
    // Allocation failure must terminate the pending HTTP response, not strand it.
    if (!response) cancel();

    AsyncWebServerResponse *expected = nullptr;
    AsyncWebServerResponse *raw = response.release();
    if (!ready_.compare_exchange_strong(expected, raw)) {
        delete raw;
        return false;
    }

    // Disconnect may have won immediately before publication.
    if (raw && cancelled()) {
        delete ready_.exchange(nullptr);
        return false;
    }

    auto self = weak_from_this().lock();
    if (self) {
        auto *queued = new (std::nothrow) std::shared_ptr<State>(std::move(self));
        if (queued && !ac_async_tcp_dispatch(resume, queued)) delete queued;
    }
    // Under allocation/queue pressure the existing TCP poll remains a fallback.
    return raw != nullptr;
}

void AsyncDeferredResponse::State::resume(void *context) {
    std::unique_ptr<std::shared_ptr<State>> queued(
        static_cast<std::shared_ptr<State> *>(context));
    State &state = **queued;
    if (state.owner_ && state.request_) {
        state.owner_->start_ready(state.request_);
    }
}

void AsyncDeferredResponse::State::cancel() {
    cancelled_.store(true);
    delete ready_.exchange(nullptr);
}

std::unique_ptr<AsyncWebServerResponse>
AsyncDeferredResponse::State::take() {
    return std::unique_ptr<AsyncWebServerResponse>(ready_.exchange(nullptr));
}

AsyncDeferredResponse::AsyncDeferredResponse(std::shared_ptr<State> state)
    : state_(std::move(state)) {}

AsyncDeferredResponse::~AsyncDeferredResponse() {
    if (state_) {
        state_->owner_ = nullptr;
        state_->request_ = nullptr;
        state_->cancel();
    }
}

bool AsyncDeferredResponse::_sourceValid() const { return state_ != nullptr; }

bool AsyncDeferredResponse::_started() const {
    return response_ && response_->_started();
}

bool AsyncDeferredResponse::_finished() const {
    return failed_ || (response_ && response_->_finished());
}

bool AsyncDeferredResponse::_failed() const {
    return failed_ || (response_ && response_->_failed());
}

void AsyncDeferredResponse::start_ready(AsyncWebServerRequest *request) {
    if (response_ || !state_ || failed_) return;

    if (state_->cancelled()) {
        failed_ = true;
        if (request && request->client()) request->client()->abort();
        return;
    }

    response_ = state_->take();
    if (!response_) return;

    if (!response_->_sourceValid()) {
        failed_ = true;
        if (request && request->client()) request->client()->abort();
        return;
    }

    _code = response_->code();
    response_->_respond(request);
}

void AsyncDeferredResponse::_respond(AsyncWebServerRequest *request) {
    if (state_) {
        state_->owner_ = this;
        state_->request_ = request;
    }
    start_ready(request);
}

size_t AsyncDeferredResponse::_ack(AsyncWebServerRequest *request,
                                   size_t len,
                                   uint32_t time) {
    if (!response_) {
        start_ready(request);
        return 0;
    }

    return failed_ ? 0 : response_->_ack(request, len, time);
}

}  // namespace aircannect
