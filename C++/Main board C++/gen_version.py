import subprocess

Import("env")

try:
    version = subprocess.check_output(
        ["git", "describe", "--tags", "--always", "--dirty"],
        cwd=env["PROJECT_DIR"],
        stderr=subprocess.DEVNULL,
    ).decode().strip()
except (subprocess.CalledProcessError, FileNotFoundError):
    version = "unknown"

with open(env["PROJECT_DIR"] + "/include/firmware_version.h", "w") as f:
    f.write("#pragma once\n")
    f.write('#define FIRMWARE_VERSION "%s"\n' % version)
