#include "app_config.h"
#include "ui_languages_generated.h"
#include "auth_utils.h"
#include "network_hints.h"
#include "debug_log.h"
#include <Preferences.h>
#include <nvs.h>
#include <initializer_list>
#include <lwip/sockets.h>
#include <lwip/inet.h>

#define CFG_DEFAULT_TCP_PORT    23

static Preferences prefs;
static AirBridgeConfig cfg;
static uint32_t config_revision = 0;
static uint32_t section_revisions[static_cast<size_t>(Config::Section::Logging) + 1] = {};
static bool onboarding_done = false;
static bool onboarding_stored = false;

// Also consumed by generate_web_ui.py; field membership lives in kv_table.
static const struct {
    Config::Section section;
    const char *name;
    const char *label;
} config_sections[] = {
    {Config::Section::Network, "network", "Network"},
    {Config::Section::Access, "access", "Web access"},
    {Config::Section::Time, "time", "Time"},
    {Config::Section::Oximetry, "oximetry", "Oximetry"},
    {Config::Section::Uart, "uart", "UART"},
    {Config::Section::Smb, "smb", "SMB"},
    {Config::Section::SleepHq, "sleephq", "SleepHQ"},
    {Config::Section::Updates, "updates", "Firmware updates"},
    {Config::Section::Logging, "logging", "Logging"},
};

struct KVEntry {
    const char *key;
    const char *nvs_key;
    enum { STR, U8, U16, BOOL } type;
    void *ptr;
    union Default {
        const char *text;
        uint32_t number;
        constexpr Default(const char *value) : text(value) {}
        constexpr Default(uint32_t value) : number(value) {}
    } initial;
    bool sensitive;
    Config::Section section;
};

#define KV_STR(k, n, f, d, s) {k, n, KVEntry::STR, &cfg.f, {d}, false, Config::Section::s}
#define KV_SECRET(k, n, f, d, s) {k, n, KVEntry::STR, &cfg.f, {d}, true, Config::Section::s}
#define KV_U8(k, n, f, d, s) {k, n, KVEntry::U8, &cfg.f, {uint32_t(d)}, false, Config::Section::s}
#define KV_U16(k, n, f, d, s) {k, n, KVEntry::U16, &cfg.f, {uint32_t(d)}, false, Config::Section::s}
// Choice labels are used at build time, not retained in firmware RAM.
#define KV_ENUM(k, n, f, d, s, ...) KV_U8(k, n, f, d, s)
#define KV_LANGUAGE(k, n, f, d, s) KV_STR(k, n, f, d, s)
#define KV_BOOL(k, n, f, d, s, ...) {k, n, KVEntry::BOOL, &cfg.f, {uint32_t(d)}, false, Config::Section::s}

static const KVEntry kv_table[] = {
    KV_STR("hostname", "hostname", hostname, DEFAULT_HOSTNAME, Network),
    KV_ENUM("wifi_mode", "wifi_mode", wifi_mode, WIFI_MODE_AP_ONLY, Network, "0=Auto (STA + AP fallback)", "1=AP only", "2=Off", "3=STA only", "4=STA+AP always"),
    KV_BOOL("wifi_roam", "wifi_roam", wifi_roam, true, Network),
    KV_STR("wifi_country", "wifi_country", wifi_country, "01", Network),
    KV_U16("tcp_port", "tcp_port", tcp_port, CFG_DEFAULT_TCP_PORT, Uart),
    KV_BOOL("oxi_enabled", "oxi_enabled", oxi_enabled, true, Oximetry),
    KV_BOOL("oxi_auto_start", "oxi_autostart", oxi_auto_start, true, Oximetry),
    KV_BOOL("oxi_feed_therapy_only", "oxi_thronly", oxi_feed_therapy_only, false, Oximetry),
    KV_U16("oxi_interval_ms", "oxi_interval", oxi_interval_ms, 500, Oximetry),
    KV_BOOL("oxi_lframe_continuous", "oxi_lframe_cont", oxi_lframe_continuous, true, Oximetry),
    KV_U16("uart_cmd_timeout_ms", "uart_timeout", uart_cmd_timeout_ms, 500, Uart),
    KV_BOOL("allow_transparent_during_therapy", "allow_transp", allow_transparent_during_therapy, false, Uart),
    KV_U16("debug_port", "debug_port", debug_port, 8023, Logging),
    KV_BOOL("syslog_en", "syslog_en", syslog_enabled, false, Logging),
    KV_STR("syslog_host", "syslog_host", syslog_host, "", Logging),
    KV_U16("syslog_port", "syslog_port", syslog_port, 514, Logging),
    KV_U16("http_port", "http_port", http_port, 80, Access),
    KV_STR("http_user", "http_user", http_user, "admin", Access),
    KV_SECRET("http_pass", "http_pass", http_pass, "airbridge", Access),
    KV_STR("auth_whitelist", "auth_wl", auth_whitelist, "", Access),
    KV_LANGUAGE("ui_language", "ui_language", ui_language, "auto", Access),
    KV_STR("update_url", "update_url", update_url, AB_DEFAULT_UPDATE_URL, Updates),
    KV_STR("ntp_server", "ntp_server", ntp_server, "", Time),
    KV_STR("tz", "tz", tz, "UTC0", Time),
    KV_BOOL("resmed_time", "resmed_time", resmed_time, true, Time),
    KV_U16("udp_oxi_port", "udp_oxi_port", udp_oxi_port, 8025, Oximetry),
    KV_BOOL("smb_enabled", "smb_enable", smb_enabled, false, Smb),
    KV_BOOL("smb_auto_after_therapy", "smb_auto", smb_auto_after_therapy, true, Smb, "0=Manual", "1=Automatic"),
    KV_STR("smb_endpoint", "smb_ep", smb_endpoint, "", Smb),
    KV_STR("smb_user", "smb_user", smb_user, "", Smb),
    KV_SECRET("smb_password", "smb_pass", smb_password, "", Smb),
    KV_BOOL("sleephq_enabled", "shq_enable", sleephq_enabled, false, SleepHq),
    KV_BOOL("sleephq_auto_after_therapy", "shq_auto", sleephq_auto_after_therapy, true, SleepHq, "0=Manual", "1=Automatic"),
    KV_STR("sleephq_client_id", "shq_id", sleephq_client_id, "", SleepHq),
    KV_SECRET("sleephq_client_secret", "shq_secret", sleephq_client_secret, "", SleepHq),
    KV_STR("sleephq_team_id", "shq_team", sleephq_team_id, "", SleepHq),
    KV_STR("sleephq_device_id", "shq_device", sleephq_device_id, "", SleepHq),
};


static void reset_value(const KVEntry &entry) {
    switch (entry.type) {
        case KVEntry::STR:  *(String *)entry.ptr = entry.initial.text; break;
        case KVEntry::U8:   *(uint8_t *)entry.ptr = entry.initial.number; break;
        case KVEntry::U16:  *(uint16_t *)entry.ptr = entry.initial.number; break;
        case KVEntry::BOOL: *(bool *)entry.ptr = entry.initial.number; break;
    }
}

static void apply_defaults() {
    for (const KVEntry &entry : kv_table) reset_value(entry);
    for (auto &revision : section_revisions) __atomic_add_fetch(&revision, 1, __ATOMIC_RELEASE);
    cfg.wifi_net_count = 0;
    for (int i = 0; i < WIFI_MAX_NETWORKS; i++) {
        cfg.wifi_nets[i].ssid = "";
        cfg.wifi_nets[i].pass = "";
        cfg.wifi_nets[i].enabled = false;
    }
}

void Config::init() {
    apply_defaults();
    prefs.begin("airbridge", false);
}

struct StoredNetworks {
    uint8_t version, count;
    struct Entry { uint8_t enabled; char ssid[33], pass[65]; } entries[WIFI_MAX_NETWORKS];
};
static_assert(sizeof(StoredNetworks) == 398, "WiFi profile layout");

static bool decode_wifi_nets(const StoredNetworks &stored) {
    if (stored.version != 1 || stored.count > WIFI_MAX_NETWORKS) return false;
    for (uint8_t i = 0; i < stored.count; i++) {
        const auto &entry = stored.entries[i];
        if (entry.enabled > 1 || !entry.ssid[0] ||
            !memchr(entry.ssid, 0, sizeof(entry.ssid)) ||
            !memchr(entry.pass, 0, sizeof(entry.pass))) return false;
    }
    cfg.wifi_net_count = stored.count;
    for (uint8_t i = 0; i < WIFI_MAX_NETWORKS; i++) {
        const bool present = i < stored.count;
        cfg.wifi_nets[i].ssid = present ? stored.entries[i].ssid : "";
        cfg.wifi_nets[i].pass = present ? stored.entries[i].pass : "";
        cfg.wifi_nets[i].enabled = present && stored.entries[i].enabled;
    }
    return true;
}

static void load_wifi_nets(bool migrate = true) {
    Preferences wp;
    // writable: we may need to scrub legacy bssid_<i>/chan_<i> keys after
    // migrating them to NetworkHints.
    wp.begin("wnet", false);
    if (wp.isKey("profiles")) {
        StoredNetworks stored = {};
        if (wp.getBytesLength("profiles") != sizeof(stored) ||
            wp.getBytes("profiles", &stored, sizeof(stored)) != sizeof(stored) ||
            !decode_wifi_nets(stored)) {
            cfg.wifi_net_count = 0;
            Log::logf(CAT_CONFIG, LOG_ERROR, "Invalid saved WiFi profiles\n");
        }
        wp.end();
        return;
    }
    cfg.wifi_net_count = wp.getUChar("count", 0);
    if (cfg.wifi_net_count > WIFI_MAX_NETWORKS) cfg.wifi_net_count = WIFI_MAX_NETWORKS;
    for (int i = 0; i < cfg.wifi_net_count; i++) {
        char key[14];
        snprintf(key, sizeof(key), "ssid_%d", i);
        cfg.wifi_nets[i].ssid = wp.getString(key, "");
        snprintf(key, sizeof(key), "pass_%d", i);
        cfg.wifi_nets[i].pass = wp.getString(key, "");
        snprintf(key, sizeof(key), "ena_%d", i);
        cfg.wifi_nets[i].enabled = wp.getBool(key, true) && cfg.wifi_nets[i].ssid.length() > 0;

        // Legacy hint keys: bssid_<i> + chan_<i> used to live here. Migrate
        // any populated values into NetworkHints, then nuke the old keys.
        // Preferences::remove is a no-op on missing keys.
        uint8_t bssid[6] = {0};
        snprintf(key, sizeof(key), "bssid_%d", i);
        wp.getBytes(key, bssid, 6);
        wp.remove(key);
        snprintf(key, sizeof(key), "chan_%d", i);
        uint8_t channel = wp.getUChar(key, 0);
        wp.remove(key);
        const uint8_t zero[6] = {0};
        if (channel > 0 && memcmp(bssid, zero, 6) != 0 &&
            cfg.wifi_nets[i].ssid.length() > 0) {
            NetworkHints::upsert(cfg.wifi_nets[i].ssid.c_str(),
                                 bssid, channel, false);
        }
    }
    wp.end();

    // migrate from old single wifi_ssid/wifi_pass
    if (cfg.wifi_net_count == 0) {
        String old_ssid = prefs.getString("wifi_ssid", "");
        if (old_ssid.length() > 0) {
            cfg.wifi_nets[0].ssid = old_ssid;
            cfg.wifi_nets[0].pass = prefs.getString("wifi_pass", "");
            cfg.wifi_nets[0].enabled = true;
            cfg.wifi_net_count = 1;
            if (migrate && Config::save_wifi_nets()) {
                prefs.remove("wifi_ssid");
                prefs.remove("wifi_pass");
            }
        }
    }
}

static void apply_syslog() {
    if (!Log::configure_syslog(cfg.syslog_enabled, cfg.syslog_host.c_str(),
                               cfg.syslog_port, cfg.hostname.c_str())) {
        Log::logf(CAT_GENERAL, LOG_WARN, "[LOG] Cannot enable syslog\n");
    }
}

static bool load_values() {
    bool legacy = prefs.isKey("wifi_ssid");
    for (const KVEntry &entry : kv_table) {
        reset_value(entry);
        if (!prefs.isKey(entry.nvs_key)) continue;
        legacy = true;
        switch (entry.type) {
            case KVEntry::STR: {
                auto &value = *(String *)entry.ptr;
                value = prefs.getString(entry.nvs_key, value);
                break;
            }
            case KVEntry::U8: {
                auto &value = *(uint8_t *)entry.ptr;
                value = prefs.getUChar(entry.nvs_key, value);
                break;
            }
            case KVEntry::U16: {
                auto &value = *(uint16_t *)entry.ptr;
                value = prefs.getUShort(entry.nvs_key, value);
                break;
            }
            case KVEntry::BOOL: {
                auto &value = *(bool *)entry.ptr;
                value = prefs.getBool(entry.nvs_key, value);
                break;
            }
        }
    }
    for (auto &revision : section_revisions) __atomic_add_fetch(&revision, 1, __ATOMIC_RELEASE);
    return legacy;
}

void Config::load() {
    bool legacy = load_values();
    // Classify old installations before migration or any new settings are saved.
    nvs_handle_t networks;
    if (nvs_open("wnet", NVS_READONLY, &networks) == ESP_OK) {
        uint8_t count = 0;
        if (nvs_get_u8(networks, "count", &count) == ESP_OK && count > 0)
            legacy = true;
        nvs_close(networks);
    }
    onboarding_stored = prefs.isKey("onboard");
    onboarding_done = onboarding_stored ? prefs.getBool("onboard", false) : legacy;
    if (!onboarding_stored)
        onboarding_stored = prefs.putBool("onboard", onboarding_done) == 1;
    load_wifi_nets();
    apply_syslog();
}

static esp_err_t store_value(nvs_handle_t handle, const KVEntry &entry) {
    switch (entry.type) {
        case KVEntry::STR: return nvs_set_str(handle, entry.nvs_key, ((String *)entry.ptr)->c_str());
        case KVEntry::U8: return nvs_set_u8(handle, entry.nvs_key, *(uint8_t *)entry.ptr);
        case KVEntry::U16: return nvs_set_u16(handle, entry.nvs_key, *(uint16_t *)entry.ptr);
        case KVEntry::BOOL: return nvs_set_u8(handle, entry.nvs_key, *(bool *)entry.ptr);
    }
    return ESP_ERR_INVALID_ARG;
}

static bool store_onboarding_marker() {
    if (!onboarding_stored)
        onboarding_stored = prefs.putBool("onboard", onboarding_done) == 1;
    return onboarding_stored;
}

bool Config::save() {
    if (!store_onboarding_marker()) {
        Log::logf(CAT_CONFIG, LOG_ERROR, "Save failed: onboarding marker\n");
        load_values();
        return false;
    }
    nvs_handle_t handle;
    esp_err_t error = nvs_open("airbridge", NVS_READWRITE, &handle);
    if (error != ESP_OK) {
        Log::logf(CAT_CONFIG, LOG_ERROR, "Save failed: NVS open: %s\n", esp_err_to_name(error));
        load_values();
        return false;
    }
    const char *stage = "commit";
    for (const KVEntry &entry : kv_table) {
        error = store_value(handle, entry);
        if (error != ESP_OK) { stage = entry.nvs_key; break; }
    }
    if (error == ESP_OK) error = nvs_commit(handle);
    nvs_close(handle);
    if (error != ESP_OK) {
        Log::logf(CAT_CONFIG, LOG_ERROR, "Save failed: %s: %s\n", stage, esp_err_to_name(error));
        load_values();
    }
    apply_syslog();
    __atomic_add_fetch(&config_revision, 1, __ATOMIC_RELEASE);
    if (error == ESP_OK) Log::logf(CAT_CONFIG, LOG_INFO, "Configuration saved\n");
    return error == ESP_OK;
}

bool Config::onboarding_complete() {
    return __atomic_load_n(&onboarding_done, __ATOMIC_ACQUIRE);
}

bool Config::complete_onboarding(const char *user, const char *password) {
    const String previous_user = cfg.http_user;
    const String previous_password = cfg.http_pass;
    if (user) cfg.http_user = user;
    if (password) cfg.http_pass = password;
    if (((user || password) && !save()) || prefs.putBool("onboard", true) != 1) {
        cfg.http_user = previous_user;
        cfg.http_pass = previous_password;
        return false;
    }
    __atomic_store_n(&onboarding_done, true, __ATOMIC_RELEASE);
    onboarding_stored = true;
    __atomic_add_fetch(&config_revision, 1, __ATOMIC_RELEASE);
    return true;
}

uint32_t Config::revision(Section section) {
    return __atomic_load_n(section == Section::All ? &config_revision :
        &section_revisions[static_cast<size_t>(section)], __ATOMIC_ACQUIRE);
}

void Config::reset_defaults() {
    prefs.clear();
    onboarding_done = false;
    onboarding_stored = false;
    apply_defaults();
    save();
}

AirBridgeConfig& Config::get() {
    return cfg;
}


static void format_value(const KVEntry &entry, String &out) {
    switch (entry.type) {
        case KVEntry::STR:  out = *(String *)entry.ptr; break;
        case KVEntry::U8:   out = String(*(uint8_t *)entry.ptr); break;
        case KVEntry::U16:  out = String(*(uint16_t *)entry.ptr); break;
        case KVEntry::BOOL: out = *(bool *)entry.ptr ? "1" : "0"; break;
    }
}

bool Config::get_value(const char *key, String &out) {
    for (const KVEntry &entry : kv_table) {
        if (strcasecmp(key, entry.key) == 0) {
            format_value(entry, out);
            return true;
        }
    }
    return false;
}

bool Config::is_sensitive(const char *key) {
    for (const KVEntry &entry : kv_table)
        if (strcasecmp(key, entry.key) == 0) return entry.sensitive;
    return false;
}

const char *Config::ui_language() {
    for (const char *tag : UI_LANGUAGE_TAGS)
        if (cfg.ui_language == tag) return tag;
    return "auto";
}

bool Config::set_value(const char *key, const char *value) {
    if (strcasecmp(key, "ui_language") == 0 && strcmp(value, "auto") != 0) {
        bool supported = false;
        for (const char *tag : UI_LANGUAGE_TAGS)
            if (strcmp(value, tag) == 0) supported = true;
        if (!supported) return false;
    }
    if (strcasecmp(key, "auth_whitelist") == 0) {
        if (!aircannect::valid_auth_whitelist(value)) return false;
    } else if (strcasecmp(key, "syslog_host") == 0 && *value) {
        in_addr address;
        if (inet_pton(AF_INET, value, &address) != 1) return false;
    } else if (strcasecmp(key, "syslog_port") == 0) {
        char *end;
        unsigned long port = strtoul(value, &end, 10);
        if (*value < '0' || *value > '9' || *end || port == 0 || port > 65535)
            return false;
    } else if (strcasecmp(key, "syslog_en") == 0) {
        if (strcmp(value, "0") != 0 && strcmp(value, "1") != 0) return false;
    }

    for (const KVEntry &entry : kv_table) {
        if (strcasecmp(key, entry.key) == 0) {
            bool changed = false;
            switch (entry.type) {
            case KVEntry::STR:
                changed = *(String*)entry.ptr != value;
                *(String*)entry.ptr = value;
                break;
            case KVEntry::U8:
                changed = *(uint8_t*)entry.ptr != static_cast<uint8_t>(atoi(value));
                *(uint8_t*)entry.ptr = atoi(value);
                break;
            case KVEntry::U16:
                changed = *(uint16_t*)entry.ptr != static_cast<uint16_t>(atoi(value));
                *(uint16_t*)entry.ptr = atoi(value);
                break;
            case KVEntry::BOOL:
                changed = *(bool*)entry.ptr != (atoi(value) != 0);
                *(bool*)entry.ptr = (atoi(value) != 0);
                break;
            }
            if (changed) __atomic_add_fetch(&section_revisions[static_cast<size_t>(entry.section)],
                                             1, __ATOMIC_RELEASE);
            return true;
        }
    }
    return false;
}

bool Config::parse_section(const char *name, Section &out) {
    for (const auto &entry : config_sections) {
        if (strcmp(name, entry.name) != 0) continue;
        out = entry.section;
        return true;
    }
    return false;
}

void Config::foreach_kv(kv_visitor_fn fn, void *ctx, Section section) {
    String val;
    for (const KVEntry &entry : kv_table) {
        if (section != Section::All && entry.section != section) continue;
        format_value(entry, val);
        fn(entry.key, val, entry.sensitive, ctx);
    }
}

String Config::dump() {
    String out;
    // wifi networks
    for (int i = 0; i < cfg.wifi_net_count; i++) {
        out += "wifi_net_" + String(i) + "=" + cfg.wifi_nets[i].ssid;
        if (!cfg.wifi_nets[i].enabled) out += " [disabled]";
        out += "\n";
    }
    // KV table entries
    foreach_kv([](const char *key, const String &val, bool sensitive, void *p) {
        String v = val;
        if (sensitive && v.length() > 0) v = "****";
        *(String*)p += String(key) + "=" + v + "\n";
    }, &out);
    return out;
}


bool Config::save_wifi_nets() {
    const auto failed = []() { load_wifi_nets(false); return false; };
    if (!store_onboarding_marker()) return failed();
    if (cfg.wifi_net_count > WIFI_MAX_NETWORKS) return failed();
    StoredNetworks stored = {};
    stored.version = 1;
    stored.count = cfg.wifi_net_count;
    for (uint8_t i = 0; i < stored.count; i++) {
        const auto &net = cfg.wifi_nets[i];
        if (!net.ssid.length() || net.ssid.length() >= sizeof(stored.entries[i].ssid) ||
            net.pass.length() >= sizeof(stored.entries[i].pass)) return failed();
        stored.entries[i].enabled = net.enabled;
        strcpy(stored.entries[i].ssid, net.ssid.c_str());
        strcpy(stored.entries[i].pass, net.pass.c_str());
    }
    nvs_handle_t handle;
    esp_err_t error = nvs_open("wnet", NVS_READWRITE, &handle);
    if (error != ESP_OK) {
        Log::logf(CAT_CONFIG, LOG_ERROR, "WiFi profiles: NVS open failed: %s\n", esp_err_to_name(error));
        return failed();
    }
    const char *stage = "write";
    error = nvs_set_blob(handle, "profiles", &stored, sizeof(stored));
    if (error == ESP_OK) { stage = "commit"; error = nvs_commit(handle); }
    nvs_close(handle);
    if (error != ESP_OK) {
        Log::logf(CAT_CONFIG, LOG_ERROR, "WiFi profiles: %s failed: %s\n", stage, esp_err_to_name(error));
        return failed();
    }
    Log::logf(CAT_CONFIG, LOG_INFO, "WiFi profiles saved count=%u\n", stored.count);
    return true;
}

bool Config::add_network(const char *ssid, const char *pass) {
    if (cfg.wifi_net_count >= WIFI_MAX_NETWORKS) return false;
    int idx = cfg.wifi_net_count;
    cfg.wifi_nets[idx].ssid = ssid;
    cfg.wifi_nets[idx].pass = pass ? pass : "";
    cfg.wifi_nets[idx].enabled = true;
    cfg.wifi_net_count++;
    return save_wifi_nets();
}

bool Config::remove_network(uint8_t idx) {
    if (idx >= cfg.wifi_net_count) return false;
    // drop any cached hints for this SSID before the slot goes away
    NetworkHints::clear_for(cfg.wifi_nets[idx].ssid.c_str());
    // shift remaining entries down
    for (int i = idx; i < cfg.wifi_net_count - 1; i++)
        cfg.wifi_nets[i] = cfg.wifi_nets[i + 1];
    cfg.wifi_net_count--;
    // clear the vacated slot
    cfg.wifi_nets[cfg.wifi_net_count].ssid = "";
    cfg.wifi_nets[cfg.wifi_net_count].pass = "";
    cfg.wifi_nets[cfg.wifi_net_count].enabled = false;
    return save_wifi_nets();
}
