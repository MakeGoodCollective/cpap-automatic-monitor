#include "auth_utils.h"
#include "string_util.h"

#include <IPAddress.h>
#include <string.h>
#include <utility>

namespace aircannect {
namespace {

uint32_t ip_to_u32(const IPAddress &ip) {
    return (uint32_t(ip[0]) << 24) | (uint32_t(ip[1]) << 16) |
        (uint32_t(ip[2]) << 8) | ip[3];
}

bool parse_ipv4(char *text, uint32_t &out) {
    trim_ascii(text);
    IPAddress ip;
    if (!ip.fromString(text) || ip.type() != IPv4) return false;
    out = ip_to_u32(ip);
    return true;
}

bool parse_entry(char *entry, uint32_t &first, uint32_t &last) {
    if (!strcmp(entry, "*")) {
        first = 0;
        last = UINT32_MAX;
        return true;
    }
    if (char *slash = strchr(entry, '/')) {
        *slash++ = 0;
        trim_ascii(slash);
        uint32_t prefix = 0;
        if (!parse_ipv4(entry, first) || !parse_uint32_decimal(slash, prefix) ||
            prefix > 32) return false;
        const uint32_t mask = prefix ? UINT32_MAX << (32 - prefix) : 0;
        first &= mask;
        last = first | ~mask;
        return true;
    }
    if (char *dash = strchr(entry, '-')) {
        *dash++ = 0;
        if (!parse_ipv4(entry, first) || !parse_ipv4(dash, last)) return false;
        if (first > last) std::swap(first, last);
        return true;
    }
    if (!parse_ipv4(entry, first)) return false;
    last = first;
    return true;
}

// AirCANnect's IPv4/CIDR/range syntax, with one parser for admission and
// validation. Finish the whole list before accepting even a wildcard match.
bool parse_whitelist(const char *whitelist, uint32_t remote, bool &matched) {
    matched = false;
    if (!whitelist) return false;
    const size_t length = strnlen(whitelist, AUTH_WHITELIST_MAX_LENGTH + 1);
    if (length > AUTH_WHITELIST_MAX_LENGTH) return false;
    for (size_t i = 0; i < length; ++i) {
        const char c = whitelist[i];
        if ((c < '0' || c > '9') && c != '.' && c != '/' && c != '-' &&
            c != ',' && c != '*' && c != ' ') return false;
    }
    char entries[AUTH_WHITELIST_MAX_LENGTH + 1];
    memcpy(entries, whitelist, length + 1);
    char *entry = entries;
    while (entry) {
        char *next = strchr(entry, ',');
        if (next) *next++ = 0;
        trim_ascii(entry);
        if (*entry) {
            uint32_t first = 0, last = 0;
            if (!parse_entry(entry, first, last)) return false;
            matched |= remote >= first && remote <= last;
        }
        entry = next;
    }
    return true;
}

}  // namespace

bool valid_auth_whitelist(const char *whitelist) {
    bool matched = false;
    return parse_whitelist(whitelist, 0, matched);
}

bool auth_whitelist_matches(const IPAddress &remote_ip, const char *whitelist) {
    if (remote_ip.type() != IPv4) return false;
    bool matched = false;
    return parse_whitelist(whitelist, ip_to_u32(remote_ip), matched) && matched;
}

}  // namespace aircannect
