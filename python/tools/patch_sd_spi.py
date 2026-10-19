"""Build AirCANnect SPI SD patches and AirBridge follow-ups on generated copies."""

from library_patches import build_patched_source


def build_source(env, node):
    env.AppendUnique(CPPPATH=[env["PROJECT_INCLUDE_DIR"]])
    return build_patched_source(env, node, "sd-spi")


if "Import" in globals():
    Import("env")
    flags = env.ParseFlags(env.GetProjectOption("build_flags", []))
    definitions = dict(
        item for item in flags.get("CPPDEFINES", []) if isinstance(item, (tuple, list))
    )
    if str(definitions.get("AB_STORAGE_SPI_SD_ENABLED")) == "1":
        env.AddBuildMiddleware(build_source, "*/libraries/SD/src/sd_diskio.cpp")
