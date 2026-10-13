#pragma once

#include <stddef.h>

class IPAddress;

namespace aircannect {

constexpr size_t AUTH_WHITELIST_MAX_LENGTH = 160;

bool valid_auth_whitelist(const char *whitelist);
// Invalid lists and non-IPv4 peers never bypass authentication.
bool auth_whitelist_matches(const IPAddress &remote_ip, const char *whitelist);

}  // namespace aircannect
