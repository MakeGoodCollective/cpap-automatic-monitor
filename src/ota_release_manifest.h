#pragma once

#include <stddef.h>

namespace OtaRelease {

static constexpr size_t VERSION_MAX = 48;
static constexpr size_t ERROR_MAX = 48;
static constexpr size_t URL_MAX = 512;

struct Artifact {
    char url[URL_MAX] = {};
    size_t size = 0;
    size_t image_size = 0;
    bool zlib = false;
};

struct Manifest {
    char version[VERSION_MAX] = {};
    char release_notes_url[URL_MAX] = {};
    Artifact artifact;
};

bool parse_manifest(char *json, size_t json_len, const char *target,
                    Manifest &manifest, char error[ERROR_MAX]);

bool is_newer(const char *current_version, const char *release_version,
              bool &newer);

bool resolve_artifact_url(const char *manifest_url, const char *artifact_url,
                          char *resolved_url, size_t resolved_capacity);

}  // namespace OtaRelease
