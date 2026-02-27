#include "config_manager.h"
#include "config.h"
#include "mag_cal/calibration.h"

#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include <ArduinoJson.h>

// ── Flash & filesystem (file-scope — transport must not be copied) ──
static Adafruit_FlashTransport_QSPI s_flashTransport;
static Adafruit_SPIFlash            s_flash(&s_flashTransport);
static FatVolume                    s_fatfs;

// ── Flash access (for USB MSC) ──────────────────────────────────────
Adafruit_SPIFlash* ConfigManager::getFlash() {
    return &s_flash;
}

// ── begin() ────────────────────────────────────────────────────────
bool ConfigManager::begin() {
    if (mounted_) return true;  // already initialized — idempotent

    if (!s_flash.begin()) {
        Serial.println(F("QSPI flash init FAILED"));
        return false;
    }

    Serial.print(F("QSPI flash: "));
    Serial.print(s_flash.size() / 1024);
    Serial.println(F(" KB"));

    if (!s_fatfs.begin(&s_flash)) {
        Serial.println(F("FAT mount FAILED — flash may need formatting"));
        return false;
    }

    // Update volume label from "CIRCUITPY" to "DISCOX" via raw flash sectors.
    // SdFat File32 doesn't support writing raw dir entries, so we go direct.
    {
        static const char NEW_LABEL[11] = {'D','I','S','C','O','X',' ',' ',' ',' ',' '};
        uint8_t sector[512];

        // 1. Update BPB label in boot sector (offset 43 for FAT12/16)
        if (s_flash.readBlocks(0, sector, 1)) {
            if (memcmp(sector + 43, NEW_LABEL, 11) != 0) {
                memcpy(sector + 43, NEW_LABEL, 11);
                s_flash.writeBlocks(0, sector, 1);
                s_flash.syncBlocks();
                Serial.println(F("  BPB volume label → DISCOX"));
            }

            // 2. Update root directory volume label entry.
            //    Root dir starts at: reserved + (num_fats * sectors_per_fat)
            uint16_t reserved   = sector[14] | (sector[15] << 8);
            uint8_t  numFats    = sector[16];
            uint16_t fatSectors = sector[22] | (sector[23] << 8);
            uint32_t rootStart  = reserved + ((uint32_t)numFats * fatSectors);
            uint16_t rootEntries = sector[17] | (sector[18] << 8);
            uint16_t rootSectors = ((rootEntries * 32) + 511) / 512;

            for (uint32_t rs = 0; rs < rootSectors; rs++) {
                if (!s_flash.readBlocks(rootStart + rs, sector, 1)) break;
                for (uint16_t off = 0; off < 512; off += 32) {
                    uint8_t* entry = sector + off;
                    if (entry[0] == 0x00) goto labelDone;   // end of dir
                    if (entry[0] == 0xE5) continue;          // deleted
                    if (entry[11] == 0x08) {                  // volume label attr
                        if (memcmp(entry, NEW_LABEL, 11) != 0) {
                            memcpy(entry, NEW_LABEL, 11);
                            s_flash.writeBlocks(rootStart + rs, sector, 1);
                            s_flash.syncBlocks();
                            Serial.println(F("  Root dir label → DISCOX"));
                        }
                        goto labelDone;
                    }
                }
            }
            labelDone:;
        }
    }

    // Ensure /flags directory exists
    if (!s_fatfs.exists("/flags")) {
        s_fatfs.mkdir("/flags");
    }

    mounted_ = true;
    Serial.println(F("Filesystem mounted OK"));
    return true;
}

// ── Settings persistence ───────────────────────────────────────────

bool ConfigManager::loadConfig(Config& cfg) {
    if (!mounted_) return false;

    File32 file = s_fatfs.open("/config.json", FILE_READ);
    if (!file) return false;

    char buf[512];
    size_t len = file.read(buf, sizeof(buf) - 1);
    file.close();
    if (len == 0) return false;
    buf[len] = '\0';

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf, len);
    if (err) {
        Serial.print(F("Config parse error: "));
        Serial.println(err.c_str());
        return false;
    }

    cfg.magTolerance          = doc["mag_tolerance"]          | Defaults::magTolerance;
    cfg.gravTolerance         = doc["grav_tolerance"]         | Defaults::gravTolerance;
    cfg.dipTolerance          = doc["dip_tolerance"]          | Defaults::dipTolerance;
    cfg.anomalyDetection      = doc["anomaly_detection"]      | Defaults::anomalyDetection;
    cfg.stabilityTolerance    = doc["stability_tolerance"]    | Defaults::stabilityTolerance;
    cfg.stabilityBufferLength = doc["stability_buffer_length"] | Defaults::stabilityBufferLength;
    cfg.emaAlpha              = doc["ema_alpha"]               | Defaults::emaAlpha;
    cfg.legAngleTolerance     = doc["leg_angle_tolerance"]     | Defaults::legAngleTolerance;
    cfg.legDistanceTolerance  = doc["leg_distance_tolerance"]  | Defaults::legDistanceTolerance;
    cfg.laserDistanceOffset   = doc["laser_distance_offset"]   | Defaults::laserDistanceOffset;
    cfg.autoShutdownTimeout   = doc["auto_shutdown_timeout"]   | Defaults::autoShutdownTimeout;
    cfg.laserTimeout          = doc["laser_timeout"]           | Defaults::laserTimeout;

    const char* name = doc["ble_name"] | Defaults::bleName;
    strncpy(cfg.bleName, name, Defaults::bleNameMaxLen);
    cfg.bleName[Defaults::bleNameMaxLen] = '\0';

    return true;
}

bool ConfigManager::saveConfig(const Config& cfg) {
    if (!mounted_) return false;

    // Remove first — SdFat FILE_WRITE doesn't truncate
    s_fatfs.remove("/config.json");

    File32 file = s_fatfs.open("/config.json", FILE_WRITE);
    if (!file) return false;

    JsonDocument doc;
    doc["mag_tolerance"]          = cfg.magTolerance;
    doc["grav_tolerance"]         = cfg.gravTolerance;
    doc["dip_tolerance"]          = cfg.dipTolerance;
    doc["anomaly_detection"]      = cfg.anomalyDetection;
    doc["stability_tolerance"]    = cfg.stabilityTolerance;
    doc["stability_buffer_length"]= cfg.stabilityBufferLength;
    doc["ema_alpha"]              = cfg.emaAlpha;
    doc["leg_angle_tolerance"]    = cfg.legAngleTolerance;
    doc["leg_distance_tolerance"] = cfg.legDistanceTolerance;
    doc["laser_distance_offset"]  = cfg.laserDistanceOffset;
    doc["auto_shutdown_timeout"]  = cfg.autoShutdownTimeout;
    doc["laser_timeout"]          = cfg.laserTimeout;
    doc["ble_name"]               = cfg.bleName;

    size_t written = serializeJsonPretty(doc, file);
    file.close();
    return written > 0;
}

// ── Calibration data ───────────────────────────────────────────────

bool ConfigManager::loadCalibrationJson(char* buf, size_t bufSize, size_t& bytesRead) {
    if (!mounted_) return false;

    File32 file = s_fatfs.open("/calibration.json", FILE_READ);
    if (!file) return false;

    size_t fileSize = file.size();
    if (fileSize >= bufSize) {
        file.close();
        return false;  // buffer too small
    }

    bytesRead = file.read(buf, fileSize);
    file.close();
    buf[bytesRead] = '\0';
    return bytesRead > 0;
}

bool ConfigManager::saveCalibrationJson(const char* json, size_t len) {
    if (!mounted_) return false;

    s_fatfs.remove("/calibration.json");

    File32 file = s_fatfs.open("/calibration.json", FILE_WRITE);
    if (!file) return false;

    size_t written = file.write(json, len);
    file.close();
    return written == len;
}

bool ConfigManager::loadCalibrationBinary(MagCal::CalibrationBinary& out) {
    if (!mounted_) return false;

    File32 file = s_fatfs.open("/calibration.bin", FILE_READ);
    if (!file) return false;

    size_t fileSize = file.size();
    if (fileSize != sizeof(MagCal::CalibrationBinary)) {
        file.close();
        return false;
    }

    size_t bytesRead = file.read(reinterpret_cast<uint8_t*>(&out), sizeof(out));
    file.close();
    return bytesRead == sizeof(out);
}

bool ConfigManager::saveCalibrationBinary(const MagCal::CalibrationBinary& data) {
    if (!mounted_) return false;

    s_fatfs.remove("/calibration.bin");

    File32 file = s_fatfs.open("/calibration.bin", FILE_WRITE);
    if (!file) return false;

    size_t written = file.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
    file.close();
    return written == sizeof(data);
}

// ── Pending readings ───────────────────────────────────────────────

bool ConfigManager::appendPendingReading(float az, float inc, float dist) {
    if (pendingBufCount_ >= MAX_PENDING_BUF) {
        Serial.println(F("  Pending RAM buffer full — forcing sync"));
        syncPendingToFlash();
    }
    if (pendingBufCount_ < MAX_PENDING_BUF) {
        pendingBuf_[pendingBufCount_++] = {az, inc, dist};
        return true;
    }
    return false;  // sync failed and buffer still full
}

bool ConfigManager::syncPendingToFlash() {
    if (pendingBufCount_ == 0) return true;
    if (!mounted_) return false;

    File32 file = s_fatfs.open("/pending.txt", FILE_WRITE);
    if (!file) {
        Serial.println(F("  syncPending: open FAILED"));
        return false;
    }
    file.seekEnd();

    char line[40];
    for (uint8_t i = 0; i < pendingBufCount_; i++) {
        int n = snprintf(line, sizeof(line), "%.1f,%.1f,%.2f\n",
                         (double)pendingBuf_[i].az,
                         (double)pendingBuf_[i].inc,
                         (double)pendingBuf_[i].dist);
        size_t written = file.write(line, n);
        if (written != (size_t)n) {
            Serial.println(F("  syncPending: write FAILED"));
            file.close();
            return false;
        }
    }

    file.close();
    Serial.print(F("  syncPending: wrote "));
    Serial.print(pendingBufCount_);
    Serial.println(F(" readings to flash"));
    pendingBufCount_ = 0;
    return true;
}

uint16_t ConfigManager::countPendingReadings() {
    uint16_t count = pendingBufCount_;

    if (!mounted_) return count;

    File32 file = s_fatfs.open("/pending.txt", FILE_READ);
    if (!file) return count;

    while (file.available()) {
        if (file.read() == '\n') count++;
    }
    file.close();
    return count;
}

bool ConfigManager::flushPendingReadings(void(*callback)(float, float, float)) {
    // First flush any file-based readings
    if (mounted_) {
        File32 file = s_fatfs.open("/pending.txt", FILE_READ);
        if (file) {
            char line[48];
            uint8_t pos = 0;

            while (file.available()) {
                int c = file.read();
                if (c == '\n' || c == '\r') {
                    if (pos > 0) {
                        line[pos] = '\0';
                        // Parse with strtof (sscanf %f broken on newlib-nano)
                        char* p = line;
                        char* end;
                        float az = strtof(p, &end);
                        if (end != p && *end == ',') {
                            p = end + 1;
                            float inc = strtof(p, &end);
                            if (end != p && *end == ',') {
                                p = end + 1;
                                float dist = strtof(p, &end);
                                if (end != p) {
                                    callback(az, inc, dist);
                                }
                            }
                        }
                        pos = 0;
                    }
                } else if (pos < sizeof(line) - 1) {
                    line[pos++] = (char)c;
                }
            }
            file.close();
        }
    }

    // Then flush RAM buffer entries
    for (uint8_t i = 0; i < pendingBufCount_; i++) {
        callback(pendingBuf_[i].az, pendingBuf_[i].inc, pendingBuf_[i].dist);
    }
    pendingBufCount_ = 0;

    return true;
}

bool ConfigManager::clearPendingReadings() {
    pendingBufCount_ = 0;
    if (!mounted_) return false;
    return s_fatfs.remove("/pending.txt");
}

// ── Flag files ─────────────────────────────────────────────────────

void ConfigManager::buildFlagPath(const char* name, char* path, size_t pathSize) {
    snprintf(path, pathSize, "/flags/%s", name);
}

bool ConfigManager::writeFlag(const char* name) {
    if (!mounted_) return false;

    char path[32];
    buildFlagPath(name, path, sizeof(path));

    File32 file = s_fatfs.open(path, FILE_WRITE);
    if (!file) return false;
    file.write('1');
    file.close();
    return true;
}

bool ConfigManager::hasFlag(const char* name) {
    if (!mounted_) return false;

    char path[32];
    buildFlagPath(name, path, sizeof(path));
    return s_fatfs.exists(path);
}

bool ConfigManager::clearFlag(const char* name) {
    if (!mounted_) return false;

    char path[32];
    buildFlagPath(name, path, sizeof(path));
    return s_fatfs.remove(path);
}
