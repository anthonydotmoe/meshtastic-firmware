import os
import shutil
import tarfile
from pathlib import Path

from SCons.Script import Import

Import("env")

MAX_BASENAME_LEN = 32


def safe_extract(tar, destination):
    dest_root = destination.resolve()
    for member in tar.getmembers():
        member_path = (destination / member.name).resolve()
        if os.path.commonpath([dest_root, member_path]) != str(dest_root):
            raise ValueError(f"Refusing to extract path outside staging dir: {member.name}")
    tar.extractall(destination)


def overlay_tree(source, destination):
    shutil.copytree(source, destination, dirs_exist_ok=True)


def prune_unsupported_filenames(root):
    removed = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and len(path.name) > MAX_BASENAME_LEN:
            removed.append(path.relative_to(root))
            path.unlink()
    return removed


project_dir = Path(env.subst("$PROJECT_DIR"))
data_dir = project_dir / "data"
webui_dir = project_dir / env.GetProjectOption("custom_webui_dir")
staging_dir = Path(env.subst("$BUILD_DIR")) / "webui-data"
fs_image = "$BUILD_DIR/${ESP32_FS_IMAGE_NAME}.bin"

env.Replace(PROJECT_DATA_DIR=str(staging_dir), PROJECTDATA_DIR=str(staging_dir))


def stage_webui(source, target, env):
    if staging_dir.exists():
        shutil.rmtree(staging_dir)

    if data_dir.is_dir():
        shutil.copytree(data_dir, staging_dir)
    else:
        staging_dir.mkdir(parents=True, exist_ok=True)

    tar_path = webui_dir / "build.tar"
    static_dir = webui_dir / "static"
    static_target = staging_dir / "static"
    static_target.mkdir(parents=True, exist_ok=True)

    if tar_path.is_file():
        with tarfile.open(tar_path) as archive:
            safe_extract(archive, static_target)
        print(f"Using variant-local WebUI tarball: {tar_path}")
    elif static_dir.is_dir():
        overlay_tree(static_dir, static_target)
        print(f"Using variant-local WebUI directory: {static_dir}")
    else:
        print(f"No variant-local WebUI payload found in {webui_dir}; using default data/")
        return

    removed = prune_unsupported_filenames(static_target)
    if removed:
        print("Skipping files with LittleFS-incompatible names:")
        for path in removed:
            print(f"  {path}")


env.AddPreAction(fs_image, stage_webui)
