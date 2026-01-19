
/*
------------------------------------------------------------------------
  Glavna datoteka za ESP32 Master enoto za zalivalni sistem

------------------------------------------------------------------------
*/ 

#include "main.h"  
#include <WiFi.h>
#include "Firebase_manager.h" 
#include "Lora_handler.h"
#include "Lora_data.h"
#include "sensor_queue.h"
#include "ReleInitialization.h"

//GLOBAL VARIABLES

// --- SPREMENLJIVKE ZA ČASOVNIK IN URNIK ---
uint32_t timestamp = 0;           // Trenutni timestamp (vsaj enkrat na sekundo posodobljen)
uint32_t secFromMidnight = 0; // Čas od polnoči v sekundah

unsigned long Interval_mS = 0;
bool firebaseUpdatePending = false;  // Ali imamo čakajočo posodobitev iz Firebase?
ChannelUpdateData pendingUpdateData; // Podatki, ki čakajo na pošiljanje
uint8_t currentChannelInProcess = 0; // Kateri kanal (0-7) trenutno obdelujemo

uint8_t Led_status = 0; // Začetni status LED

uint16_t messageCounter = 0; // Števec poslanih sporočil
bool reset_occured = false; // zastavica, da je prišlo do ponovnega zagona na Rele modulu
bool lora_response_received = false;     // Ali je bil prejet Lora odgovor
bool rele_init_in_progress = false;      // Ali je inicializacija Rele modula v teku

// --- SPREMENLJIVKE ZA TIMEOUT PRI INICIALIZACIJI ---
// unsigned long initState_timeout_start = 0;           // Časovnik za timeout pri čakanju na odgovor
// const unsigned long INIT_RESPONSE_TIMEOUT_MS = 5000; // 5 sekund za odgovor
// const unsigned long FIREBASE_RESPONSE_TIMEOUT_MS = 15000; // 15 sekund za odgovor iz Firebase

// // --- SPREMENLJIVKE ZA PONOVNE POSKUSE ---
// const uint8_t MAX_INIT_RETRIES = 3; // Največje število ponovnih poskusov
// // uint8_t init_retry_count = 0;       // Trenutni števec ponovnih poskusov

bool init_done_flag = false; // zastavica za zaključek inicializacije

Kanal kanal[8] = {
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0}}; // Končamo z definicijo kanalov

// globalne spremenljivke za podatke senzorjev
float Temperature = 0.0;        // trenutna temperatura
float Humidity = 0.0;           // trenutna vlažnost
float SoilMoisture = 0.0;       // trenutna vlažnost tal
float BatteryVoltage = 0.0;     // trenutna napetost baterije
float CurrentConsumption = 0.0; // trenutna poraba toka
float WaterConsumption = 0.0;   // skupna poraba vode


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
// Funkcija za pripravo in pošiljanje LoRa paketa


// ---------------------------------------------------------------------------------------------------------------------
// Funkcija za pripravo in pošiljanje ODGOVORA na prejet paket


//----------------------------------------------------------------------------------------------------------------------
// Funkcija za pošiljanje UTC timestamp za sinhronizacijo ure na LoRa rele modul
// Pošlje paket z ukazom CMD_SET_TIME in trenutnim Unix časovnim žigom v payloadu.

void Rele_sendUTC()
{
  Serial.println("Pripravljam paket za sinhronizacijo casa (CMD_SET_TIME)...");

  // 1. Pripravimo payload s trenutnim časom.
  TimePayload time_payload;
  // shranimo tudi čas od polnoči za poznejše preverjanje
  secFromMidnight = getCurrentSeconds(); // Pridobimo trenutni čas v sekundah od polnoči
  Serial.printf("  Čas od polnoči: %u sekund\n", secFromMidnight);
  time_payload.unix_time = getTime(); // Pridobimo trenutni Unix časovni žig.

  Serial.printf("  Posiljam timestamp: %u\n", time_payload.unix_time);

  // 2. Pošljemo paket z uporabo obstoječe funkcije.
  Lora_prepare_and_send_packet(CommandType::CMD_SET_TIME, &time_payload, sizeof(time_payload));
  // Odgovor v Lora_handle_received_packet, - RESPONSE_TIME.
}

//--------------------------------------------------------------------------------------
// Funkcija za pošiljanje ukaza za branje senzorjev
void Rele_readSensors()
{
  Serial.println("Pripravljam paket za branje senzorjev (CMD_GET_SENSORS)...");

  // Pošljemo paket z ukazom CMD_GET_SENSORS.
  // Ta ukaz ne potrebuje nobenih podatkov v payloadu.
  Lora_prepare_and_send_packet(CommandType::CMD_GET_SENSORS, nullptr, 0);
  // Odgovor v Lora_handle_received_packet, - RESPONSE_SENSORS.
}

//----------------------------------------------------------------------------------------------------------------------
// Funkcija za pošiljanje ukaza za branje INA podatkov
void Read_INA()
{
  Serial.println("Pripravljam paket za branje INA podatkov (CMD_GET_INA_DATA)...");

  // Pošljemo paket z ukazom CMD_GET_INA_DATA.
  // Ta ukaz ne potrebuje nobenih podatkov v payloadu.
  Lora_prepare_and_send_packet(CommandType::CMD_GET_INA_DATA, nullptr, 0);
  // Odgovor v Lora_handle_received_packet, - RESPONSE_INA_DATA.
}

//----------------------------------------------------------------------------------------------------------------------
// Funkcija za preverjanje stanja relejev
void Rele_readRelaysStatus()
{
  Serial.println("Preverjanje stanja relejev (CMD_GET_STATUS)...");
  // Pošljemo paket z ukazom CMD_GET_STATUS.
  // Ta ukaz ne potrebuje nobenih podatkov v payloadu.
  Lora_prepare_and_send_packet(CommandType::CMD_GET_STATUS, nullptr, 0);
  // Odgovor v Lora_handle_received_packet, - RESPONSE_STATUS.
}

//---------------------------------------------------------------------------------------------------------------------
// Funkcija za branje urnikov relejev iz Rele modula
void Rele_readRelayUrnik(uint8_t kanalIndex)
{
  Serial.printf("Branje urnika za kanal %d (CMD_GET_URNIK)...\n", kanalIndex + 1);

  // Pripravimo payload z indeksom kanala
  UrnikPayload urnik_request;
  urnik_request.releIndex = kanalIndex; // Kanali so indeksirani od 0 do 7

  // Pošljemo paket z uporabo obstoječe funkcije.
  Lora_prepare_and_send_packet(CommandType::CMD_GET_URNIK, &urnik_request, sizeof(urnik_request));
  // Odgovor v Lora_handle_received_packet, - RESPONSE_URNIK.
}

//-----------------------------------------------------
// Funkcija za update urnikov relejev v Rele modulu
void Rele_updateRelayUrnik(uint8_t index, uint32_t startSec, uint32_t endSec)
{
  Serial.printf("Posiljanje posodobitve urnika za kanal %d (CMD_UPDATE_URNIK)...\n", index + 1);
  currentChannelInProcess = index; // shranimo kateri kanal posodabljamo
  // Pripravimo payload z novim urnikom za izbrani kanal
  UrnikPayload urnik_payload;
  urnik_payload.releIndex = index; // Kanali so indeksirani od 0 do 7
  urnik_payload.startTimeSec = startSec;
  urnik_payload.endTimeSec = endSec;

  Serial.printf("  Nov urnik - Start sec: %d, End sec: %d\n",
                urnik_payload.startTimeSec,
                urnik_payload.endTimeSec);

  // Pošljemo paket z uporabo obstoječe funkcije.
  Lora_prepare_and_send_packet(CommandType::CMD_UPDATE_URNIK, &urnik_payload, sizeof(urnik_payload));
  // Odgovor v Lora_handle_received_packet, - RESPONSE_UPDATE_URNIK.
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

  // Registriraj callback funkcijo za obdelavo LoRa paketov
  // Ta funkcija bo poklicana iz lora_process_received_packets()
  lora_initialize(Lora_handle_received_packet);

  Sensor_InitQueue(); // Inicializacija senzorske čakalne vrste

  Serial.println("Setup končan.");
  printLocalTime();                                     // Izpiši lokalni čas
  timestamp = getTime();                                // Get current timestamp
  Serial.println("Trenutni čas: " + String(timestamp)); // izpiši trenutni čas
  secFromMidnight = getCurrentSeconds();                // Pridobimo trenutni čas v sekundah od polnoči
  Firebase.printf("[F_HEAP] Free Heap po SETUP: %d\n", ESP.getFreeHeap());

}

//---------------------------------------------------------------------------------------------------------------------
void loop()
{
  // 1. Preverimo WiFi povezavo
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi povezava izgubljena! Poskušam ponovno...");
    connectToWiFi();
  }

  // 2. FIREBASE PROCESSING (najprej!)
  Firebase_loop();

  // 2.a. Zagon Rele inicijacije
  if (firebase_connected)
  {
    if (!init_done_flag)
    {
      if (!rele_init_in_progress)
      {
        // StartReleInitialization();
        rele_init_in_progress = true;
      }

      // 2.b. Upravljanje inicializacije Rele modula
      // manageReleInitialization();
    }
    else
    {
      // 2.c. Upravljanje ponovnega zagona Rele modula
      if (reset_occured && !lora_is_busy())
      {
        triggerReleReinitialization();
        reset_occured = false; // Ponastavi zastavico
      }
    }
  
    // 3.Ko je Lora prosta, pošlji posodobitev prek LoRa
    if (firebaseUpdatePending && !lora_is_busy())
    {
      Firebase.printf("[STREAM] LoRa prosta. Pošiljam posodobitev.\n");
      // Pošljemo podatke, ki so bili shranjeni v nabiralniku
      Rele_updateRelayUrnik(
          pendingUpdateData.kanalIndex,
          pendingUpdateData.start_sec,
          pendingUpdateData.end_sec);
      firebaseUpdatePending = false;
    }

    // 4. Periodično branje senzorjev
    if (init_done_flag)
    {
      // if (Sensor_IntervalRead(Interval_mS)) // Če je čas za branje, pokliči funkcijo
      // {
      //   Serial.print("------------------------------------------\n");
      //   Serial.println("[SENSOR] Čas za branje senzorjev. Dodajam v čakalno vrsto...");
      //   Serial.print("------------------------------------------\n");

      //   Sensor_QueueOperation(SensorTaskType::READ_SENSORS);
      //   Sensor_QueueOperation(SensorTaskType::READ_INA);
      //   // read_sensor = false; // Ponastavimo zastavico
      // }
    }

    // Procesiranje senzorske čakalne vrste 10x na sekundo
    Sensor_ProcessQueue();
  }

  // 5. LoRa obdelava prejetih paketov
  lora_process_received_packets(); // Preverimo če so prisotni novi paketi

  Blink_led(Led_status);

  //----------------------------------------------------------------------------------
}
