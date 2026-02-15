// nvm_manager.cpp — Flash-backed device-name storage
//
// Mirrors the CircuitPython NVM layout in code.py:17-32:
//   byte 0 : magic (0xBE)
//   byte 1 : name length
//   byte 2+ : UTF-8 name bytes

#include "nvm_manager.h"
#include "config.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

void nvmInit() {
    InternalFS.begin();
}

String nvmReadName() {
    File f = InternalFS.open(NVM_FILENAME, FILE_O_READ);
    if (!f) {
        return String(DEFAULT_NAME);
    }

    uint8_t header[2];
    if (f.read(header, 2) != 2 || header[0] != NVM_MAGIC) {
        f.close();
        return String(DEFAULT_NAME);
    }

    uint8_t len = header[1];
    if (len == 0 || len > MAX_NAME_LEN) {
        f.close();
        return String(DEFAULT_NAME);
    }

    char buf[MAX_NAME_LEN + 1];
    if (f.read(buf, len) != len) {
        f.close();
        return String(DEFAULT_NAME);
    }
    f.close();

    buf[len] = '\0';
    return String(buf);
}

bool nvmWriteName(const String& name) {
    if (name.length() < 1 || name.length() > MAX_NAME_LEN) {
        return false;
    }

    // Remove old file first (LittleFS doesn't truncate on open)
    InternalFS.remove(NVM_FILENAME);

    File f = InternalFS.open(NVM_FILENAME, FILE_O_WRITE);
    if (!f) {
        return false;
    }

    uint8_t header[2] = { NVM_MAGIC, (uint8_t)name.length() };
    f.write(header, 2);
    f.write(name.c_str(), name.length());
    f.close();
    return true;
}
