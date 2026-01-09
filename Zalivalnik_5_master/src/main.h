#pragma once


//#include <FirebaseClient.h>
#include <Arduino.h>
#include "credentials.h"  // Insert your network credentials
#include "utilities.h"    // Include the utility functions
#include "message_protocol.h" // Include the message protocol definitions

// GLOBAL DEFINITIONS
#define DEFAULT_SENSOR_READ_INTERVAL_MINUTES 5  // Default interval for sensor reading

// GLOBAL VARIABLES
// Struktura za shranjevanje podatkov o relejih
struct Kanal
{
  bool state;    // Stanje releja (vklopljen/izklopljen)
  char start[6]; // Začetni čas (HH:MM)
  int start_sec; // Začetni čas v sekundah od polnoči
  char end[6];   // Končni čas (HH:MM)
  int end_sec;   // Končni čas v sekundah od polnoči
};

// Struktura za prenos posodobljenih podatkov o kanalu iz Firebase streaminga
struct ChannelUpdateData {
  int kanalIndex = -1;
  int start_sec = -1;
  int end_sec = -1;
};

extern Kanal firebase_kanal[8]; // Polje struktur za kanale