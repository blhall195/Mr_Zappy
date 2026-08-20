import datetime
import subprocess

Import("env")


def git(*args):
    return subprocess.check_output(
        ["git"] + list(args),
        cwd=env["PROJECT_DIR"],
        stderr=subprocess.DEVNULL,
    ).decode().strip()


try:
    # Reachable tag exists — use it as-is (e.g. "v1.2", "v1.2-3-gabcdef-dirty")
    version = git("describe", "--tags", "--dirty")
except (subprocess.CalledProcessError, FileNotFoundError):
    try:
        commit_hash = git("rev-parse", "--short", "HEAD")
        dirty = bool(git("status", "--porcelain"))
        if dirty:
            # Working tree has uncommitted changes — commit date is misleading,
            # so prefix with today's build date instead.
            prefix = datetime.date.today().strftime("%Y%m%d")
            version = "%s-%s-dirty" % (prefix, commit_hash)
        else:
            commit_date = git("show", "-s", "--format=%cd", "--date=format:%Y%m%d", "HEAD")
            version = "%s-%s" % (commit_date, commit_hash)
    except (subprocess.CalledProcessError, FileNotFoundError):
        version = "unknown"

print("Firmware version: %s" % version)

with open(env["PROJECT_DIR"] + "/include/firmware_version.h", "w") as f:
    f.write("#pragma once\n")
    f.write('#define FIRMWARE_VERSION "%s"\n' % version)
