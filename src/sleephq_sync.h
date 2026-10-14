#pragma once

#include <stddef.h>
#include <stdint.h>

#include "background_operation_control.h"
#include "edf_catalog.h"
#include "sleephq_client.h"
#include "sd_storage.h"

namespace SleepHqSync {

using Config = aircannect::SleepHqConfig;

struct Progress {
    uint32_t files_seen;
    uint32_t files_uploaded;
    uint32_t files_skipped;
    uint32_t files_attached;
    uint64_t bytes_uploaded;
    uint32_t import_id;
    char current_day[9];
    char import_status[aircannect::AC_SLEEPHQ_STATUS_MAX];
    const char *phase;
    char file[128];
};

using ProgressCallback = void (*)(void *context, const Progress &progress);

struct BacklogChanges {
    bool entry;
    bool other_entries;
};

bool configured(const Config &config);
bool check_account(const Config &config,
                   aircannect::BackgroundOperationControl &operation,
                   char *error, size_t error_size);
bool complete(SdStorage::Session &storage, const Config &config,
              const EdfCatalog::Entry &entry);
bool pending_files(SdStorage::Session &storage, const Config &config,
                   const EdfCatalog::Entry &entry, uint32_t &files,
                   bool &incomplete, char *error, size_t error_size);
// One run owns authentication and team resolution across catalog entries.
class Run {
public:
    explicit Run(const Config &config);
    ~Run();
    Run(const Run &) = delete;
    Run &operator=(const Run &) = delete;
    bool resume(SdStorage::Session &storage,
                aircannect::BackgroundOperationControl &operation,
                Progress &progress, ProgressCallback callback, void *context,
                char *error, size_t error_size, BacklogChanges *changes = nullptr);
    bool sync_day(SdStorage::Session &storage, const char *day, bool repair,
                      aircannect::BackgroundOperationControl &operation,
                      Progress &progress, ProgressCallback callback,
                      void *callback_context, char *error, size_t error_size,
                      BacklogChanges *changes = nullptr);
    const char *last_request() const { return client_.last_request(); }

private:
    struct Inventory;
    const Config config_;
    aircannect::SleepHqClient client_;
    uint32_t team_id_ = 0;
    Inventory *inventory_ = nullptr;
};

}  // namespace SleepHqSync
