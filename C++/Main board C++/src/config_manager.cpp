#include "config_manager.h"
#include "config.h"
#include "mag_cal/calibration.h"

#include "Adafruit_SPIFlash.h"
#include "FatLib/FatFormatter.h"
#include "SdFat.h"
#include <ArduinoJson.h>

// ── Flash & filesystem (file-scope — transport must not be copied) ──
static Adafruit_FlashTransport_QSPI s_flashTransport;
static Adafruit_SPIFlash s_flash(&s_flashTransport);
static FatVolume s_fatfs;

// ── Flash access (for USB MSC) ──────────────────────────────────────
Adafruit_SPIFlash *ConfigManager::getFlash() { return &s_flash; }

// ── Robust flash write ──────────────────────────────────────────────
// QSPI writes occasionally fail right after heavy I2C/SERCOM activity
// (laser UART, BLE, sensor polling) — see Session 13. These helpers add
// a settle delay, retry up to 3x, and remount the filesystem between
// attempts so a transient failure no longer surfaces as a hard "save
// fail". s_remount re-inits the QSPI transport + FAT volume in place.
static bool s_remount() {
    // Re-mount the FAT volume only. Do NOT re-run s_flash.begin() — that
    // re-reads the JEDEC ID over the live QSPI link and can mis-detect the
    // chip (size → 0), breaking every subsequent flash operation.
    return s_fatfs.begin(&s_flash);
}

// Atomic byte-blob write: data → tmpPath, sync, then rename over path so
// a failed/interrupted write never corrupts the existing file.
static bool writeFileAtomic(const char *path, const char *tmpPath, const uint8_t *data, size_t len) {
    // Let any in-flight SERCOM traffic drain and QSPI settle before writing.
    delay(20);

    for (int attempt = 0; attempt < 3; attempt++) {
        s_fatfs.remove(tmpPath);

        File32 file = s_fatfs.open(tmpPath, FILE_WRITE);
        if (file) {
            size_t written = file.write(data, len);
            file.sync();
            file.close();
            if (written == len) {
                s_fatfs.remove(path);
                if (s_fatfs.rename(tmpPath, path)) {
                    if (attempt > 0) {
                        Serial.print(F("  flash write OK on retry "));
                        Serial.println(attempt);
                    }
                    return true;
                }
            }
        }

        Serial.print(F("  flash write FAILED (attempt "));
        Serial.print(attempt + 1);
        Serial.println(F(") — remounting"));
        s_fatfs.remove(tmpPath);
        if (!s_remount()) {
            Serial.println(F("  remount FAILED"));
        }
        delay(50);
    }
    return false;
}

// ── begin() ────────────────────────────────────────────────────────
bool ConfigManager::begin() {
    if (mounted_) {
        return true; // already initialized — idempotent
    }

    if (!s_flash.begin()) {
        Serial.println(F("QSPI flash init FAILED"));
        return false;
    }

    Serial.print(F("QSPI flash: "));
    Serial.print(s_flash.size() / 1024);
    Serial.println(F(" KB"));

    if (!s_fatfs.begin(&s_flash)) {
        Serial.println(F("FAT mount FAILED — attempting auto-reformat..."));

        // Erase chip and create a fresh FAT filesystem
        if (!s_flash.eraseChip()) {
            Serial.println(F("  Flash erase FAILED"));
            return false;
        }

        FatFormatter formatter;
        uint8_t workBuf[512];
        if (!formatter.format(&s_flash, workBuf, &Serial)) {
            Serial.println(F("  FAT format FAILED"));
            return false;
        }
        s_flash.syncBlocks();

        // Try mounting the freshly formatted filesystem
        if (!s_fatfs.begin(&s_flash)) {
            Serial.println(F("  Mount after format FAILED"));
            return false;
        }
        Serial.println(F("  Auto-reformat successful — flash recovered"));
        reformatted_ = true;
    }

    // Update volume label from "CIRCUITPY" to "DISCOX" via raw flash sectors.
    // SdFat File32 doesn't support writing raw dir entries, so we go direct.
    {
        static const char NEW_LABEL[11] = {'D', 'I', 'S', 'C', 'O', 'X', ' ', ' ', ' ', ' ', ' '};
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
            uint16_t reserved = sector[14] | (sector[15] << 8);
            uint8_t numFats = sector[16];
            uint16_t fatSectors = sector[22] | (sector[23] << 8);
            uint32_t rootStart = reserved + ((uint32_t)numFats * fatSectors);
            uint16_t rootEntries = sector[17] | (sector[18] << 8);
            uint16_t rootSectors = ((rootEntries * 32) + 511) / 512;

            for (uint32_t rs = 0; rs < rootSectors; rs++) {
                if (!s_flash.readBlocks(rootStart + rs, sector, 1)) {
                    break;
                }
                for (uint16_t off = 0; off < 512; off += 32) {
                    uint8_t *entry = sector + off;
                    if (entry[0] == 0x00) {
                        goto labelDone; // end of dir
                    }
                    if (entry[0] == 0xE5) {
                        continue; // deleted
                    }
                    if (entry[11] == 0x08) { // volume label attr
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

bool ConfigManager::loadConfig(Config &cfg) {
    if (!mounted_) {
        return false;
    }

    File32 file = s_fatfs.open("/config.json", FILE_READ);
    if (!file) {
        return false;
    }

    char buf[1024];
    size_t len = file.read(buf, sizeof(buf) - 1);
    file.close();
    if (len == 0) {
        return false;
    }
    buf[len] = '\0';

    Serial.println(F("  Raw config.json:"));
    Serial.println(buf);

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf, len);
    if (err) {
        Serial.print(F("Config parse error: "));
        Serial.println(err.c_str());
        return false;
    }

    cfg.magTolerance = doc["mag_tolerance"] | Defaults::magTolerance;
    cfg.gravTolerance = doc["grav_tolerance"] | Defaults::gravTolerance;
    cfg.dipTolerance = doc["dip_tolerance"] | Defaults::dipTolerance;
    cfg.anomalyDetection = doc["anomaly_detection"] | Defaults::anomalyDetection;
    cfg.stabilityTolerance = doc["stability_tolerance"] | Defaults::stabilityTolerance;
    cfg.stabilityBufferLength = doc["stability_buffer_length"] | (int)Defaults::stabilityBufferLength;
    cfg.emaAlphaStable = doc["ema_alpha_stable"] | Defaults::emaAlphaStable;
    cfg.emaAlphaMoving = doc["ema_alpha_moving"] | Defaults::emaAlphaMoving;
    cfg.legAngleTolerance = doc["leg_angle_tolerance"] | Defaults::legAngleTolerance;
    cfg.cartesianTolerance = doc["cartesian_tolerance"] | Defaults::cartesianTolerance;
    cfg.laserDistanceOffset = doc["laser_distance_offset"] | Defaults::laserDistanceOffset;
    cfg.calMagConsistency = doc["cal_mag_consistency"] | Defaults::calMagConsistency;
    cfg.calGravConsistency = doc["cal_grav_consistency"] | Defaults::calGravConsistency;
    cfg.calBufferLength = doc["cal_buffer_length"] | (int)Defaults::calBufferLength;
    cfg.calSettleMs = doc["cal_settle_ms"] | (int)Defaults::calSettleMs;
    cfg.calEmaAlpha = doc["cal_ema_alpha"] | Defaults::calEmaAlpha;
    cfg.calTimeoutMs = doc["cal_timeout_ms"] | (int)Defaults::calTimeoutMs;
    cfg.autoShutdownTimeout = doc["auto_shutdown_timeout"] | Defaults::autoShutdownTimeout;
    cfg.laserTimeout = doc["laser_timeout"] | Defaults::laserTimeout;
    cfg.laserWibble = doc["laser_wibble"] | Defaults::laserWibble;
    cfg.measureFromFront = doc["measure_from_front"] | Defaults::measureFromFront;
    cfg.screenBrightness = doc["screen_brightness"] | (int)Defaults::screenBrightness;

    const char *rawName = doc["ble_name"] | Defaults::bleName;
    // Strip SAP6_ prefix if user included it — we always prepend it ourselves
    const char *userPart = (strncmp(rawName, "SAP6_", 5) == 0) ? rawName + 5 : rawName;
    snprintf(cfg.bleName, sizeof(cfg.bleName), "SAP6_%s", userPart);
    cfg.bleName[Defaults::bleNameMaxLen] = '\0';

    Serial.print(F("  Loaded ble_name: "));
    Serial.println(cfg.bleName);
    Serial.print(F("  Loaded brightness: "));
    Serial.println(cfg.screenBrightness);

    return true;
}

bool ConfigManager::saveConfig(const Config &cfg) {
    if (!mounted_) {
        return false;
    }

    JsonDocument doc;

    // ── Settings values ──
    doc["mag_tolerance"] = cfg.magTolerance;
    doc["grav_tolerance"] = cfg.gravTolerance;
    doc["dip_tolerance"] = cfg.dipTolerance;
    doc["anomaly_detection"] = cfg.anomalyDetection;
    doc["stability_tolerance"] = cfg.stabilityTolerance;
    doc["stability_buffer_length"] = (int)cfg.stabilityBufferLength;
    doc["ema_alpha_stable"] = cfg.emaAlphaStable;
    doc["ema_alpha_moving"] = cfg.emaAlphaMoving;
    doc["leg_angle_tolerance"] = cfg.legAngleTolerance;
    doc["cartesian_tolerance"] = cfg.cartesianTolerance;
    doc["laser_distance_offset"] = cfg.laserDistanceOffset;
    doc["cal_mag_consistency"] = cfg.calMagConsistency;
    doc["cal_grav_consistency"] = cfg.calGravConsistency;
    doc["cal_buffer_length"] = (int)cfg.calBufferLength;
    doc["cal_settle_ms"] = (int)cfg.calSettleMs;
    doc["cal_ema_alpha"] = cfg.calEmaAlpha;
    doc["cal_timeout_ms"] = (int)cfg.calTimeoutMs;
    doc["auto_shutdown_timeout"] = cfg.autoShutdownTimeout;
    doc["laser_timeout"] = cfg.laserTimeout;
    doc["laser_wibble"] = cfg.laserWibble;
    doc["measure_from_front"] = cfg.measureFromFront;
    doc["screen_brightness"] = (int)cfg.screenBrightness;
    // Save only the user portion — SAP6_ prefix is always auto-prepended on load
    const char *nameToSave = (strncmp(cfg.bleName, "SAP6_", 5) == 0) ? cfg.bleName + 5 : cfg.bleName;
    doc["ble_name"] = nameToSave;

    // Serialize to a RAM buffer first, then write atomically with retry.
    char buf[1280];
    if (measureJsonPretty(doc) >= sizeof(buf)) {
        Serial.println(F("Config JSON too large for buffer"));
        return false;
    }
    size_t len = serializeJsonPretty(doc, buf, sizeof(buf));
    if (len == 0) {
        return false;
    }

    return writeFileAtomic("/config.json", "/cfg_tmp.json", reinterpret_cast<const uint8_t *>(buf), len);
}

// ── Calibration data ───────────────────────────────────────────────

bool ConfigManager::loadCalibrationJson(char *buf, size_t bufSize, size_t &bytesRead) {
    if (!mounted_) {
        return false;
    }

    File32 file = s_fatfs.open("/calibration.json", FILE_READ);
    if (!file) {
        return false;
    }

    size_t fileSize = file.size();
    if (fileSize >= bufSize) {
        file.close();
        return false; // buffer too small
    }

    bytesRead = file.read(buf, fileSize);
    file.close();
    buf[bytesRead] = '\0';
    return bytesRead > 0;
}

bool ConfigManager::saveCalibrationJson(const char *json, size_t len) {
    if (!mounted_) {
        return false;
    }
    return writeFileAtomic("/calibration.json", "/cal_tmp.json", reinterpret_cast<const uint8_t *>(json),
                           len);
}

bool ConfigManager::loadCalibrationBinary(MagCal::CalibrationBinary &out) {
    if (!mounted_) {
        return false;
    }

    File32 file = s_fatfs.open("/calibration.bin", FILE_READ);
    if (!file) {
        return false;
    }

    size_t fileSize = file.size();
    if (fileSize != sizeof(MagCal::CalibrationBinary)) {
        file.close();
        return false;
    }

    size_t bytesRead = file.read(reinterpret_cast<uint8_t *>(&out), sizeof(out));
    file.close();
    return bytesRead == sizeof(out);
}

bool ConfigManager::saveCalibrationBinary(const MagCal::CalibrationBinary &data) {
    if (!mounted_) {
        return false;
    }
    return writeFileAtomic("/calibration.bin", "/cal_tmp.bin", reinterpret_cast<const uint8_t *>(&data),
                           sizeof(data));
}

// ── Calibration quality metrics ─────────────────────────────────────

bool ConfigManager::saveCalMetrics(const CalMetrics &m) {
    if (!mounted_) {
        return false;
    }
    return writeFileAtomic("/cal_metrics.bin", "/met_tmp.bin", reinterpret_cast<const uint8_t *>(&m),
                           sizeof(m));
}

bool ConfigManager::loadCalMetrics(CalMetrics &m) {
    if (!mounted_) {
        return false;
    }
    File32 file = s_fatfs.open("/cal_metrics.bin", FILE_READ);
    if (!file) {
        return false;
    }
    size_t bytesRead = file.read(reinterpret_cast<uint8_t *>(&m), sizeof(m));
    file.close();
    return bytesRead == sizeof(m);
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
    return false; // sync failed and buffer still full
}

bool ConfigManager::syncPendingToFlash() {
    if (pendingBufCount_ == 0) {
        return true;
    }
    if (!mounted_) {
        return false;
    }

    File32 file = s_fatfs.open("/pending.txt", FILE_WRITE);
    if (!file) {
        Serial.println(F("  syncPending: open FAILED"));
        return false;
    }
    file.seekEnd();

    char line[40];
    for (uint8_t i = 0; i < pendingBufCount_; i++) {
        int n = snprintf(line, sizeof(line), "%.1f,%.1f,%.2f\n", (double)pendingBuf_[i].az,
                         (double)pendingBuf_[i].inc, (double)pendingBuf_[i].dist);
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

    if (!mounted_) {
        return count;
    }

    File32 file = s_fatfs.open("/pending.txt", FILE_READ);
    if (!file) {
        return count;
    }

    while (file.available()) {
        if (file.read() == '\n') {
            count++;
        }
    }
    file.close();
    return count;
}

bool ConfigManager::flushPendingReadings(void (*callback)(float, float, float)) {
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
                        char *p = line;
                        char *end;
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
    if (!mounted_) {
        return false;
    }
    return s_fatfs.remove("/pending.txt");
}

// ── Flag files ─────────────────────────────────────────────────────

void ConfigManager::buildFlagPath(const char *name, char *path, size_t pathSize) {
    snprintf(path, pathSize, "/flags/%s", name);
}

bool ConfigManager::writeFlag(const char *name) {
    if (!mounted_) {
        return false;
    }

    char path[32];
    buildFlagPath(name, path, sizeof(path));

    delay(20); // let SERCOM/QSPI settle (same fragility as data writes)

    for (int attempt = 0; attempt < 3; attempt++) {
        // begin() creates /flags, but a remount below won't — ensure it exists.
        if (!s_fatfs.exists("/flags")) {
            s_fatfs.mkdir("/flags");
        }

        File32 file = s_fatfs.open(path, FILE_WRITE);
        if (file) {
            file.write('1');
            file.sync();
            file.close();
            if (s_fatfs.exists(path)) {
                return true;
            }
        }

        Serial.print(F("  flag write FAILED (attempt "));
        Serial.print(attempt + 1);
        Serial.println(F(") — remounting"));
        if (!s_remount()) {
            Serial.println(F("  remount FAILED"));
        }
        delay(50);
    }
    return false;
}

bool ConfigManager::hasFlag(const char *name) {
    if (!mounted_) {
        return false;
    }

    char path[32];
    buildFlagPath(name, path, sizeof(path));
    return s_fatfs.exists(path);
}

bool ConfigManager::clearFlag(const char *name) {
    if (!mounted_) {
        return false;
    }

    char path[32];
    buildFlagPath(name, path, sizeof(path));
    return s_fatfs.remove(path);
}
