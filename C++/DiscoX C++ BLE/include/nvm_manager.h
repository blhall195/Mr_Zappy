#pragma once
#include <Arduino.h>

// Initialise internal flash filesystem — call once in setup()
void nvmInit();

// Read the stored device name, or return DEFAULT_NAME if none/corrupt
String nvmReadName();

// Write a new device name (1..MAX_NAME_LEN chars). Returns true on success.
bool nvmWriteName(const String& name);
