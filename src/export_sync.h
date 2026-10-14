#pragma once

#include <stdint.h>

#include "edf_catalog.h"

namespace ExportSync {

enum class Engine : uint8_t { Smb, SleepHq, Count };
enum class Action : uint8_t { Sync, Check, Repair };

enum class State : uint8_t {
    Unsupported,
    Disabled,
    Idle,
    Pending,
    Working,
    Error,
};

enum class CheckState : uint8_t { NotRun, Pending, Working, Passed, Failed };

struct CheckStatus {
    CheckState state;
    uint32_t config_revision;
    char error[96];
};

struct Backlog {
    bool known;
    uint32_t files;
    uint32_t sessions;
    uint32_t waiting_sessions;
    uint32_t config_revision;
    uint32_t catalog_generation;
    char error[48];
};

struct Status {
    bool supported;
    State state;
    uint32_t files_seen;
    bool has_files;
    uint32_t files_uploaded;
    uint32_t files_skipped;
    uint64_t bytes_uploaded;
    uint32_t last_sync_epoch;
    char current_day[9];
    char last_error[64];
    CheckStatus check;
    Backlog backlog;
};

struct SleepHqStatus {
    bool supported;
    State state;
    uint32_t files_seen;
    bool has_files;
    uint32_t files_uploaded;
    uint32_t files_skipped;
    uint32_t files_attached;
    uint64_t bytes_uploaded;
    uint32_t import_id;
    uint32_t last_sync_epoch;
    char current_day[9];
    char import_status[32];
    char last_error[96];
    bool retry_pending;
    uint32_t retry_at_epoch;
    CheckStatus check;
    Backlog backlog;
};

struct PublicationStamp {
    uint32_t exports, smb_config, shq_config, catalog, network;
    int system, rop;
    bool mounted, backlog_pending, check_pending;

    bool operator==(const PublicationStamp &other) const {
        return exports == other.exports && smb_config == other.smb_config &&
            shq_config == other.shq_config && catalog == other.catalog &&
            network == other.network && system == other.system && rop == other.rop &&
            mounted == other.mounted && backlog_pending == other.backlog_pending &&
            check_pending == other.check_pending;
    }
};

void init();
void therapy_started();
bool request_post_therapy(const EdfCatalog::Entry &entry);
// A repair range is inclusive; one date selects a day, empty selects all local days.
// Engines reject operations they cannot implement, without enqueuing any work.
bool request_manual(Engine engine, Action action = Action::Sync,
                    const char *from = nullptr, const char *to = nullptr,
                    const char **error = nullptr);
bool parse_engine(const char *name, Engine &engine);
const char *engine_name(Engine engine);
void request_backlog_refresh(bool files_changed = false);
uint32_t revision();
PublicationStamp publication_stamp();
bool busy();
void get_status(Status &out);
void get_sleephq_status(SleepHqStatus &out);
const char *action_blocked(Engine engine, Action action);
const char *backlog_state(const Backlog &backlog, bool smb);
const char *state_name(State state);
const char *check_state_name(CheckState state);

}  // namespace ExportSync
