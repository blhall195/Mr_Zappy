// uart_handler.cpp — Non-blocking DRDY-gated UART reader + line parser
//
// Faithful port of the read_uart_loop() async task in F:\code.py (lines 111-148).
//
// The Python code uses three cooperative async tasks; here we use a state
// machine so that loop() never blocks and BLE polling continues.

#include "uart_handler.h"
#include "config.h"
#include "sap6_protocol.h"
#include "nvm_manager.h"

// Forward — defined in main.cpp
extern void restartAdvertisingWithName(const String& newName);

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
enum UARTState {
    UART_IDLE,           // waiting for DRDY HIGH
    UART_READING,        // DRDY went HIGH, draining serial buffer
    UART_WAIT_DRDY_LOW   // data consumed, waiting for DRDY to deassert
};

static UARTState  state        = UART_IDLE;
static String     lineBuffer;
static uint32_t   drdyRiseTime = 0;

// ---------------------------------------------------------------------------
// Line parser — mirrors code.py:96-108 (parse_uart_line) and 122-142
// ---------------------------------------------------------------------------
static void processLine(const String& line) {
    // Keep-alive from master — acknowledge but don't forward
    if (line == "ALIVE") {
        Serial.println("Keep-alive received via UART");
        return;
    }

    // Remote name-change command via UART
    if (line.startsWith("NAME:")) {
        String newName = line.substring(5);
        newName.trim();
        if (newName.length() >= 1 && (int)newName.length() <= MAX_NAME_LEN) {
            nvmWriteName(newName);
            restartAdvertisingWithName(newName);
            Serial.print("BLE name changed to: ");
            Serial.println(newName);
        } else {
            Serial.print("Invalid name length: ");
            Serial.println(newName.length());
        }
        return;
    }

    // Survey data: "Compass:XXX,Clino:YYY,Distance:ZZZ"
    int c1 = line.indexOf(',');
    int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
    if (c1 < 0 || c2 < 0) {
        Serial.print("Parse error: bad format: ");
        Serial.println(line);
        return;
    }

    int cp = line.indexOf(':');
    int ci = line.indexOf(':', c1 + 1);
    int di = line.indexOf(':', c2 + 1);
    if (cp < 0 || ci < 0 || di < 0) {
        Serial.print("Parse error: missing colon: ");
        Serial.println(line);
        return;
    }

    float compass  = line.substring(cp + 1, c1).toFloat();
    float clino    = line.substring(ci + 1, c2).toFloat();
    float distance = line.substring(di + 1).toFloat();

    Serial.printf("Sending over BLE: %.2f, %.2f, %.2f\n", compass, clino, distance);
    sap6.sendData(compass, clino, distance);
}

// ---------------------------------------------------------------------------
// Drain any available serial bytes into lineBuffer, processing complete lines
// ---------------------------------------------------------------------------
static void drainSerial() {
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == '\n') {
            lineBuffer.trim();
            if (lineBuffer.length() > 0) {
                processLine(lineBuffer);
            }
            lineBuffer = "";
        } else if (c != '\r') {
            // Overflow protection
            if (lineBuffer.length() < 128) {
                lineBuffer += c;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void uartInit() {
    lineBuffer.reserve(64);
}

void uartPoll() {
    switch (state) {

    case UART_IDLE:
        if (digitalRead(PIN_DRDY) == HIGH) {
            // Debounce: record first rising edge, wait DRDY_DEBOUNCE_MS
            if (drdyRiseTime == 0) {
                drdyRiseTime = millis();
                return;
            }
            if (millis() - drdyRiseTime < DRDY_DEBOUNCE_MS) {
                return;  // still debouncing
            }
            drdyRiseTime = 0;
            state = UART_READING;
            // fall through to READING
        } else {
            drdyRiseTime = 0;  // DRDY bounced back low
            return;
        }
        /* FALLTHROUGH */

    case UART_READING:
        drainSerial();
        state = UART_WAIT_DRDY_LOW;
        break;

    case UART_WAIT_DRDY_LOW:
        // Keep draining late-arriving bytes while DRDY is still asserted
        drainSerial();
        if (digitalRead(PIN_DRDY) == LOW) {
            state = UART_IDLE;
        }
        break;
    }
}
