#pragma once

#include "main.h" 


void init_LED();  // LED struktura in funkcije
void syncTimestamp();
uint32_t getTime();
void printLocalTime();
uint32_t getCurrentSeconds();
String formatTime(uint32_t totalSeconds);
void Blink_led(uint8_t status);
void formatSecondsToTime(char *buffer, size_t bufferSize, int seconds);
void set_Interval(uint8_t minutes);
unsigned long get_Interval();

extern unsigned long Interval_mS;