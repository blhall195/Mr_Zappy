#pragma once

#include <Arduino.h>
#include "device_context.h"

class Adafruit_SPIFlash;  // forward declaration for USB MSC
namespace MagCal { struct CalibrationBinary; }

// Boot-mode flag names
namespace Flags {
    constexpr const char* CALIBRATION = "calibration";
    constexpr const char* MENU        = "menu";
    constexpr const char* SNAKE       = "snake";
}

class ConfigManager {
public:
    // Initialize QSPI flash and mount FAT filesystem.
    // Returns false if flash init or mount fails (device runs with defaults).
    bool begin();

    bool isReady() const { return mounted_; }

    // Expose the internal QSPI flash object (for USB MSC block callbacks)
    Adafruit_SPIFlash* getFlash();

    // ── Settings persistence ────────────────────────────────────────

    // Load config from /config.json. Returns false if missing/corrupt.
    // On failure, cfg is left unchanged (caller uses defaults).
    bool loadConfig(Config& cfg);

    // Save config to /config.json. Returns false on write failure.
    bool saveConfig(const Config& cfg);

    // ── Calibration data ────────────────────────────────────────────

    // Load calibration JSON into caller-provided buffer.
    // Buffer is null-terminated on success. bytesRead excludes the null.
    bool loadCalibrationJson(char* buf, size_t bufSize, size_t& bytesRead);

    // Save calibration JSON string to /calibration.json.
    bool saveCalibrationJson(const char* json, size_t len);

    // Load calibration binary from /calibration.bin.
    bool loadCalibrationBinary(MagCal::CalibrationBinary& out);

    // Save calibration binary to /calibration.bin.
    bool saveCalibrationBinary(const MagCal::CalibrationBinary& data);

    // ── Pending readings (offline queue) ────────────────────────────

    // Buffer a reading in RAM (fast, no flash I/O).
    bool appendPendingReading(float az, float inc, float dist);

    // Write any RAM-buffered readings to flash. Call from main loop
    // during idle — QSPI writes crash if done mid-measurement.
    // Also call from doShutdown() before pulling power pin.
    bool syncPendingToFlash();

    // True if RAM buffer has unsaved readings.
    bool hasPendingToSync() const { return pendingBufCount_ > 0; }

    // Count lines in /pending.txt + RAM buffer.
    uint16_t countPendingReadings();

    // Read each line from flash, parse, call callback. Returns false on IO error.
    bool flushPendingReadings(void(*callback)(float az, float inc, float dist));

    // Delete /pending.txt and clear RAM buffer.
    bool clearPendingReadings();

    // ── Flag files (boot mode triggers) ─────────────────────────────

    // Create /flags/<name> (empty file).
    bool writeFlag(const char* name);

    // Check if /flags/<name> exists.
    bool hasFlag(const char* name);

    // Delete /flags/<name>.
    bool clearFlag(const char* name);

private:
    bool mounted_ = false;

    // RAM buffer for pending readings (avoids QSPI writes mid-measurement)
    static const uint8_t MAX_PENDING_BUF = 20;
    struct PendingEntry { float az, inc, dist; };
    PendingEntry pendingBuf_[MAX_PENDING_BUF];
    uint8_t pendingBufCount_ = 0;

    void buildFlagPath(const char* name, char* path, size_t pathSize);
};
