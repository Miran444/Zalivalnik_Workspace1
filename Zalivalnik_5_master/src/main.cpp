
/*
------------------------------------------------------------------------
  Glavna datoteka za ESP32 Master enoto za zalivalni sistem

------------------------------------------------------------------------
*/ 

#include "main.h"  
#include <WiFi.h>
#include "Firebase_manager.h" 

//GLOBAL VARIABLES

// --- SPREMENLJIVKE ZA ČASOVNIK IN URNIK ---
uint32_t timestamp = 0;           // Trenutni timestamp (vsaj enkrat na sekundo posodobljen)
uint32_t secFromMidnight = 0; // Čas od polnoči v sekundah

unsigned long Interval_mS = 0;
bool firebaseUpdatePending = false;  // Ali imamo čakajočo posodobitev iz Firebase?
ChannelUpdateData pendingUpdateData; // Podatki, ki čakajo na pošiljanje
uint8_t currentChannelInProcess = 0; // Kateri kanal (0-7) trenutno obdelujemo

uint8_t Led_status = 0; // Začetni status LED


//-----------------------------------------------------------------------------------------------------
// Funkcija za povezavo na WiFi 
void connectToWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}


//---------------------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);

  // Connect to Wi-Fi
  connectToWiFi();

  // Initialize onboard LED
  init_LED();

  syncTimestamp(); // Sinhroniziraj čas z NTP strežnikom

  Firebase_setup();

  Serial.println("Setup končan.");

}

//---------------------------------------------------------------------------------------------------------------------
void loop()
{
  //-------------------------------------------------------------------------------------
  // 1. Preverimo WiFi povezavo
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi povezava izgubljena! Poskušam ponovno...");
    connectToWiFi();
  }

  // 2. FIREBASE PROCESSING (najprej!)
  Firebase_loop();

      // // Ko je Lora prosta, pošlji posodobitev prek LoRa
      // if (firebaseUpdatePending && !lora_is_busy())
      // {
      //   Firebase.printf("[STREAM] LoRa prosta. Pošiljam posodobitev.\n");
      //   // Pošljemo podatke, ki so bili shranjeni v nabiralniku
      //   Rele_updateRelayUrnik(
      //       pendingUpdateData.kanalIndex,
      //       pendingUpdateData.start_sec,
      //       pendingUpdateData.end_sec);
      //   firebaseUpdatePending = false;
      // }

  Blink_led(Led_status);

//----------------------------------------------------------------------------------
}
