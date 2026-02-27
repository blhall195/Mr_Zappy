#include <Arduino.h>
#include <bluefruit.h>
#include "config.h"
#include "sap6_protocol.h"
#include "nvm_manager.h"
#include "uart_handler.h"

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void connectCallback(uint16_t conn_handle);
void disconnectCallback(uint16_t conn_handle, uint8_t reason);
void restartAdvertisingWithName(const String& newName);
static void startAdvertising();

// ---------------------------------------------------------------------------
// Connection-monitoring state
// ---------------------------------------------------------------------------
static bool     lastConnected = false;
static uint32_t lastConnCheck = 0;
static uint32_t phyRequestTime = 0;   // PHY update retry timer
static bool     phyRequestPending = false;
static uint32_t phyCheckTime = 0;     // Delayed PHY report after connection
static bool     phyCheckPending = false;

// ---------------------------------------------------------------------------
// Advertising helper — builds adv + scan-response data and starts
// ---------------------------------------------------------------------------
static void startAdvertising() {
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(sap6.service());
    Bluefruit.Advertising.addTxPower();
    Bluefruit.ScanResponse.addName();   // complete name in scan response

    Bluefruit.Advertising.restartOnDisconnect(false);  // we restart manually after bond-clear
    Bluefruit.Advertising.setInterval(32, 244);        // fast then slow advertising
    Bluefruit.Advertising.setFastTimeout(30);           // 30 s of fast interval
    Bluefruit.Advertising.start(0);                     // 0 = advertise forever
}

// ---------------------------------------------------------------------------
// Restart advertising with a new device name  (matches code.py:79-93)
// ---------------------------------------------------------------------------
void restartAdvertisingWithName(const String& newName) {
    Bluefruit.Advertising.stop();
    Bluefruit.setName(newName.c_str());

    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(sap6.service());
    Bluefruit.Advertising.addTxPower();
    Bluefruit.ScanResponse.addName();

    if (!Bluefruit.connected()) {
        Bluefruit.Advertising.start(0);
    }
}

// ---------------------------------------------------------------------------
// BLE connection callbacks
// ---------------------------------------------------------------------------
void connectCallback(uint16_t conn_handle) {
    (void)conn_handle;
    // PHY negotiation handled by patched BLEConnection library (Coded PHY preferred)
}

void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
    (void)conn_handle;
    (void)reason;
    // Main handling is in loop() for consistent state tracking
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    // USB debug serial
    Serial.begin(115200);

    // UART to/from master survey instrument
    Serial1.begin(UART_BAUD);

    // GPIO  (matches code.py:62-77)
    pinMode(PIN_DRDY, INPUT_PULLDOWN);

    pinMode(PIN_BLE_CONNECTED, OUTPUT);
    digitalWrite(PIN_BLE_CONNECTED, LOW);   // start disconnected

    pinMode(PIN_LZR_POWER, OUTPUT);
    digitalWrite(PIN_LZR_POWER, HIGH);      // laser power on

    // NVM — mount flash filesystem, read stored device name
    nvmInit();
    String deviceName = nvmReadName();
    Serial.print("BLE Name: ");
    Serial.println(deviceName);

    // BLE stack — increase event length for Coded PHY (default 3 is too short)
    Bluefruit.configPrphConn(BLE_GATT_ATT_MTU_DEFAULT, 24, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    Bluefruit.begin();
    Bluefruit.setTxPower(BLE_TX_POWER);     // +8 dBm for advertising
    Bluefruit.setName(deviceName.c_str());

    // Clear bonds on startup — critical for reliable re-pairing (code.py:43)
    Bluefruit.Periph.clearBonds();
    Serial.println("Cleared BLE bonds");

    // Register connection callbacks
    Bluefruit.Periph.setConnectCallback(connectCallback);
    Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

    // SAP6 GATT service (must be after Bluefruit.begin())
    sap6.begin();

    // Start BLE advertising
    startAdvertising();
    Serial.println("BLE advertising started");

    // UART handler state
    uartInit();

    // Signal main board that UART/BLE stack is ready to receive commands.
    Serial1.write("READY\n");

    Serial.println("DiscoX BLE bridge ready");
}

// ---------------------------------------------------------------------------
// loop()  — 10 ms cycle, mirrors the three Python async tasks
// ---------------------------------------------------------------------------
void loop() {
    // ── Task 1: UART reading (non-blocking state machine) ──────────────
    uartPoll();

    // ── Task 2: BLE protocol poll ──────────────────────────────────────
    // Drives ACK/retry AND returns any inbound command for UART forwarding.
    int cmd = sap6.poll();
    if (cmd >= 0) {
        Serial.printf("Received from BLE: %d\n", cmd);
        if (cmd == CMD_ACK0 || cmd == CMD_ACK1) {
            // Matches code.py: uart.write(b"ACK_RECEIVED\n")
            Serial1.write("ACK_RECEIVED\n");
        } else {
            // Forward non-ACK commands as decimal string
            char buf[8];
            snprintf(buf, sizeof(buf), "%d\n", cmd);
            Serial1.write(buf);
        }
    }

    // ── Task 3: Connection monitoring (every 100 ms) ───────────────────
    if (millis() - lastConnCheck >= CONN_CHECK_MS) {
        lastConnCheck = millis();
        bool connected = Bluefruit.connected();

        if (connected != lastConnected) {
            if (connected) {
                Serial.println("BLE Connected");
                digitalWrite(PIN_BLE_CONNECTED, HIGH);
                // Schedule PHY update request
                phyRequestTime = millis();
                phyRequestPending = true;
            } else {
                Serial.println("BLE Disconnected");
                digitalWrite(PIN_BLE_CONNECTED, LOW);
                phyRequestPending = false;
                phyCheckPending = false;
                Bluefruit.Periph.clearBonds();
                if (!Bluefruit.Advertising.isRunning()) {
                    Serial.println("Restarting BLE advertising at +8 dBm");
                    Bluefruit.Advertising.start(0);
                }
            }
            lastConnected = connected;
        }
    }

    // ── PHY request with retry (every 500 ms until accepted) ───────────────
    if (phyRequestPending && millis() - phyRequestTime >= 500) {
        phyRequestTime = millis();
        uint16_t conn_handle = Bluefruit.connHandle();
        ble_gap_phys_t phys = { BLE_GAP_PHY_CODED, BLE_GAP_PHY_CODED };
        uint32_t err = sd_ble_gap_phy_update(conn_handle, &phys);
        if (err == 0) {
            Serial.println("PHY update request: OK");
            phyRequestPending = false;
            // Set connection TX power to +8 dBm (Bluefruit.setTxPower only sets advertising)
            BLEConnection* conn = Bluefruit.Connection(conn_handle);
            if (conn) {
                conn->setTxPower(BLE_TX_POWER);
                Serial.println("Connection TX power set to +8 dBm");
            }
            phyCheckTime = millis();
            phyCheckPending = true;
        } else {
            Serial.printf("PHY update retry: 0x%04lX\n", err);
        }
    }

    // ── Delayed PHY check (2 s after PHY request accepted) ──────────────
    if (phyCheckPending && millis() - phyCheckTime >= 2000) {
        phyCheckPending = false;
        BLEConnection* conn = Bluefruit.Connection(0);
        if (conn) {
            uint8_t phy = conn->getPHY();
            if (phy == BLE_GAP_PHY_CODED) {
                Serial.println("Coded PHY: YES - Long range active");
            } else {
                const char* phyNames[] = {"Auto", "1 Mbps", "2 Mbps", "unknown", "Coded"};
                Serial.printf("Coded PHY: NO - Fell back to %s\n",
                              (phy <= 4) ? phyNames[phy] : "unknown");
            }
        }
    }

    delay(LOOP_INTERVAL_MS);  // 10 ms — matches asyncio.sleep(0.01)
}
