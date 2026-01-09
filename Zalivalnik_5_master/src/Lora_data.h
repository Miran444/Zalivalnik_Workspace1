#pragma once

#include <Arduino.h>
#include "Lora_handler.h"
#include "main.h"
#include "Firebase_manager.h" 

extern uint32_t getTime();
extern uint32_t getCurrentSeconds();
extern void Rele_sendUTC();

// Forward deklaracija za Sensor_queue funkcije
extern void Sensor_OnLoRaResponse(bool success);

// Eksterne globalne spremenljivke
extern uint16_t messageCounter;
extern uint32_t secFromMidnight;
extern uint32_t timestamp;
extern uint8_t currentChannelInProcess;
extern bool lora_response_received;
extern bool reset_occured;

extern Kanal kanal[8];
extern Kanal firebase_kanal[8];

// Globalne spremenljivke za senzorje
extern float Temperature;
extern float Humidity;
extern float SoilMoisture;
extern float BatteryVoltage;
extern float CurrentConsumption;
extern float WaterConsumption;

// Funkcije za pošiljanje LoRa paketov
void Lora_prepare_and_send_packet(CommandType cmd, const void *payload_data, size_t payload_size);
void Lora_prepare_and_send_response(uint16_t request_id, CommandType cmd, const void *payload_data, size_t payload_size);

// Funkcija za obdelavo prejetih paketov
void Lora_handle_received_packet(const LoRaPacket &packet);

