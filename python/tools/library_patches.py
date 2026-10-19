"""Apply ordered patches to generated library sources, never upstream files.

Use plain unified diffs: Git-format headers can be skipped when applying from
a subdirectory of another repository, even with --no-index.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile


def patch_files(directory: Path) -> list[Path]:
    return sorted(directory.glob("*.patch"))


def apply_patches(source_dir: Path, patches: list[Path]) -> None:
    for patch in patches:
        subprocess.run(
            ["git", "apply", "--no-index", str(patch.resolve())],
            cwd=source_dir,
            check=True,
        )


def build_patched_source(env, node, patch_directory):
    original = Path(node.srcnode().get_abspath())
    output = Path(env.subst("$BUILD_DIR")) / "generated" / original.name
    patches = patch_files(Path(env["PROJECT_DIR"]) / "patches" / patch_directory)
    if not patches:
        raise RuntimeError(f"{patch_directory} patches are missing")
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(dir=output.parent) as temporary:
        staging = Path(temporary)
        source = staging / original.name
        shutil.copyfile(original, source)
        apply_patches(staging, patches)
        content = source.read_bytes()

    if not output.exists() or output.read_bytes() != content:
        output.write_bytes(content)
    return env.File(str(output))
