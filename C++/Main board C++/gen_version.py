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
    dirty = bool(git("status", "--porcelain"))

    try:
        # HEAD is exactly a tagged commit — the tag alone identifies it,
        # so neither the date nor the hash add anything.
        tag = git("describe", "--tags", "--exact-match")
        exact = True
    except subprocess.CalledProcessError:
        try:
            tag = git("describe", "--tags", "--abbrev=0")
        except subprocess.CalledProcessError:
            tag = None
        exact = False

    if exact and not dirty:
        # Clean checkout of a tagged commit — the tag alone identifies it.
        parts = [tag]
    else:
        if dirty:
            # Uncommitted changes — commit date would be misleading, so use
            # today's build date instead.
            date = datetime.date.today().strftime("%Y%m%d")
        else:
            date = git("show", "-s", "--format=%cd", "--date=format:%Y%m%d", "HEAD")

        parts = [date]
        if tag:
            parts.append(tag)
        if not exact:
            # Hash is redundant once the tag pins down the exact commit.
            parts.append(git("rev-parse", "--short", "HEAD"))

    version = "-".join(parts)
    if dirty:
        version += "-dirty"
except (subprocess.CalledProcessError, FileNotFoundError):
    version = "unknown"

print("Firmware version: %s" % version)

with open(env["PROJECT_DIR"] + "/include/firmware_version.h", "w") as f:
    f.write("#pragma once\n")
    f.write('#define FIRMWARE_VERSION "%s"\n' % version)
