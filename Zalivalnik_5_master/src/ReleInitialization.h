#pragma once


//#include <FirebaseClient.h>
#include <Arduino.h>
#include "credentials.h"  // Insert your network credentials
#include "utilities.h"    // Include the utility functions
#include "message_protocol.h" // Include the message protocol definitions

// --- SPREMENLJIVKE ZA SEKVENČNI AVTOMAT (STATE MACHINE) ---

// Definicija stanj za inicializacijo Rele modula
enum class InitState
{
  IDLE,                       // Ne dela ničesar
  READ_FIREBASE_INTERVAL,     // Pošlji zahtevo za branje intervala iz Firebase
  WAIT_FOR_INTERVAL_RESPONSE, // Čakaj na odgovor za interval
  START,                      // Začetek inicializacije
  SEND_TIME,                  // Pošlji ukaz za čas
  WAIT_FOR_TIME_RESPONSE,     // Čakaj na odgovor za čas
  SEND_STATUS_REQUEST,        // Pošlji zahtevo za status
  WAIT_FOR_STATUS_RESPONSE,   // Čakaj na odgovor za status
  SEND_SENSORS_REQUEST,       // Pošlji zahtevo za senzorje
  WAIT_FOR_SENSORS_RESPONSE,  // Čakaj na odgovor za senzorje
  SEND_URNIK_REQUEST,         // Pošlji zahtevo za urnik
  WAIT_FOR_URNIK_RESPONSE,    // Čakaj na odgovor za urnik
  READ_FROM_FIREBASE,         // Pošlji zahtevo na Firebase za en kanal
  WAIT_FOR_FIREBASE_RESPONSE, // Čakaj na odgovor od Firebase
  COMPARE_AND_SEND_LORA,      // Primerjaj urnike in po potrebi pošlji LoRa ukaz
  WAIT_FOR_UPDATE_URNIK,      // Čakaj na potrditev RESPONSE_UPDATE_URNIK od Rele modula
  FIREBASE_UPDATE_STATUS,     // Pošlji zahtevo za posodobitev stanja kanala v Firebase
  WAIT_FOR_FIREBASE_UPDATE_STATUS, // Čakaj na odgovor od Firebase za posodobitev stanja kanala
  INIT_DONE,                  // Inicializacija končana
  WAIT_FOR_INIT_COMPLETE,     // Čakaj na dokončanje inicializacije
  DONE,                       // Inicializacija končana
  STOP,                       // Zaustavi inicializacijo
  ERROR                       // Napaka med inicializacijo
};

extern void Lora_prepare_and_send_packet(CommandType cmd, const void *payload_data, size_t payload_size);
extern void Firebase_readInterval();
extern void Firebase_readKanalUrnik(uint8_t kanalIndex);
extern void Firebase_Update_Relay_State(uint8_t kanalIndex, bool state);
extern void Rele_sendUTC();
extern void Rele_readSensors();
extern void Rele_readRelayUrnik(uint8_t kanalIndex);
extern void Rele_updateRelayUrnik(uint8_t index, uint32_t startSec, uint32_t endSec);
extern void Rele_readRelaysStatus();

extern Kanal kanal[8];
extern bool init_done_flag;

// zastavice za spremljanje prejetih odgovorov
extern bool lora_response_received;
extern bool firebase_response_received;
extern uint8_t currentChannelInProcess;

void triggerReleReinitialization();
void manageReleInitialization();
void StartReleInitialization();