#pragma once
#include <Arduino.h>
#include <EEPROM.h>

void EepromInit();
void Read(uint32_t _addr, uint8_t *_data, uint32_t _len);
bool Write(uint32_t _addr, uint8_t *_data, uint32_t _len);
