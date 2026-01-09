
/*
------------------------------------------------------------------------
  Glavna datoteka za ESP32 Master enoto za zalivalni sistem

------------------------------------------------------------------------
*/ 

#include "main.h"  
#include <WiFi.h>
#include "Firebase_manager.h" 
#include "Lora_handler.h"
#include "sensor_queue.h"


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

// --- SPREMENLJIVKE ZA TIMEOUT PRI INICIALIZACIJI ---
unsigned long initState_timeout_start = 0;           // Časovnik za timeout pri čakanju na odgovor
const unsigned long INIT_RESPONSE_TIMEOUT_MS = 5000; // 5 sekund za odgovor
const unsigned long FIREBASE_RESPONSE_TIMEOUT_MS = 15000; // 15 sekund za odgovor iz Firebase

// --- SPREMENLJIVKE ZA PONOVNE POSKUSE ---
const uint8_t MAX_INIT_RETRIES = 3; // Največje število ponovnih poskusov
uint8_t init_retry_count = 0;       // Trenutni števec ponovnih poskusov

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
void Lora_prepare_and_send_packet(CommandType cmd, const void *payload_data, size_t payload_size)
{
  // NAMESTO malloc, uporabi stack:
  LoRaPacket packet;  // Stack allocation!
  packet.syncWord = LORA_SYNC_WORD;
  packet.messageId = messageCounter++;
  packet.command = cmd;
  memset(packet.payload, 0, sizeof(packet.payload));
  
  if (payload_data != nullptr && payload_size > 0) {
    memcpy(packet.payload, payload_data, payload_size);
  }

  UBaseType_t stackLeft = uxTaskGetStackHighWaterMark(NULL);
  Serial.printf("[Lora] Stack left: %d\n", stackLeft);

  // POMEMBNO: Preveri če je stack kritičen
  if (stackLeft < 2048) {
    Serial.println("[LORA] ⚠️ STACK CRITICAL! Prekinjam pošiljanje.");
    lora_set_context(LoRaContext::IDLE);
    return;
  }

  char logBuffer[32];
  snprintf(logBuffer, sizeof(logBuffer), "OUT ID: %d, CMD: %d", packet.messageId, (uint8_t)packet.command);
  Serial.println(logBuffer);

  packet.crc = calculate_crc((const uint8_t *)&packet, offsetof(LoRaPacket, crc));

  // Nastavi kontekst...
  if (lora_get_context() != LoRaContext::INITIALIZATION && 
      lora_get_context() != LoRaContext::SENSOR_QUEUE) {
    lora_set_context(LoRaContext::WAITING_FOR_RESPONSE);
  }

  // Pošlji
  if (!lora_send_packet(packet)) {  // Zdaj pošiljamo referenco na stack
    Serial.println("[MAIN] Napaka pri pošiljanju!");
    lora_set_context(LoRaContext::IDLE);
  }
  
  // packet se avtomatsko sprosti ko funkcija returna
}

// ---------------------------------------------------------------------------------------------------------------------
// Funkcija za pripravo in pošiljanje ODGOVORA na prejet paket
void Lora_prepare_and_send_response(uint16_t request_id, CommandType cmd, const void *payload_data, size_t payload_size)
{
  LoRaPacket packet;
  packet.syncWord = LORA_SYNC_WORD;
  packet.messageId = request_id; // Uporabi ID iz zahteve!
  packet.command = cmd;

  memset(packet.payload, 0, sizeof(packet.payload));
  if (payload_data != nullptr && payload_size > 0)
  {
    memcpy(packet.payload, payload_data, payload_size);
  }

  packet.crc = calculate_crc((const uint8_t *)&packet, offsetof(LoRaPacket, crc));

  // Odgovor ni kritičen, ne čakamo na potrditev
  // lora_set_waiting_for_response(false);
  lora_set_context(LoRaContext::JUST_ACK);

    if (!lora_send_packet(packet))
  {
    Serial.println("[MAIN] Napaka pri pošiljanju paketa!");
    lora_set_context(LoRaContext::IDLE); // Reset konteksta
  }
}

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

void Lora_handle_received_packet(const LoRaPacket &packet)
//================================================================================================================
{
  // Prikažemo ID in ukaz prejetega paketa na zaslonu
  char logBuffer[32];
  snprintf(logBuffer, sizeof(logBuffer), "In ID: %d, CMD: %d", packet.messageId, (uint8_t)packet.command);
  Serial.println(logBuffer);
  // displayLogOnLine(4, logBuffer);

  // Preverimo CRC
  uint16_t received_crc = packet.crc;
  uint16_t calculated_crc = calculate_crc((const uint8_t *)&packet, offsetof(LoRaPacket, crc));
  if (received_crc != calculated_crc)
  {
    Serial.println("NAPAKA: CRC se ne ujema!");
    return;
  }
  Serial.println("CRC OK.");

  // Logika za obdelavo glede na tip ukaza
  switch (packet.command)
  {
  //=================================================================================================
  case CommandType::RESPONSE_ACK:
    //=================================================================================================
    {
      AckStatus status = (AckStatus)packet.payload[0];
      Serial.print("Prejet ACK za sporocilo ID ");
      Serial.print(packet.messageId);
      Serial.print(" s statusom: ");
      Serial.println((int)status);

      if (status == AckStatus::ACK_OK)
      {
      }

      lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat
      break;
    }

  //=================================================================================================
  case CommandType::RESPONSE_TIME_SET:
    //=================================================================================================
    {
      // Preberemo odgovor na CMD_SET_TIME
      TimeResponsePayload sec_from_midnight_payload;
      memcpy(&sec_from_midnight_payload, packet.payload, sizeof(TimeResponsePayload));

      Serial.print("Prejet odgovor: ");
      Serial.println(sec_from_midnight_payload.seconds_since_midnight);

      // Preverimo ali se je čas pravilno nastavil
      if (sec_from_midnight_payload.seconds_since_midnight == secFromMidnight)
      {
        // Serial.println("Čas na Rele enoti je bil uspešno nastavljen.");
        lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat
      }
      else
      {
        Serial.println("NAPAKA: Čas na Rele enoti ni bil pravilno nastavljen.");
        // Tukaj lahko dodate dodatno obdelavo napake, npr. ponovni poskus nastavitve časa
        // Rele_sendUTC(); // Ponovno pošljemo čas
      }
      break;
    }

  //=================================================================================================
  case CommandType::RESPONSE_STATUS:
    //=================================================================================================
    { // Preberemo stanje relejev na Rele modulu
      RelayStatusPayload status;
      memcpy(&status, packet.payload, sizeof(RelayStatusPayload));
      Serial.println("Prejeto stanje relejev:");
      // Izpiši stanje relejev iz bitmaske
      for (int i = 0; i < 8; i++)
      {
        bool is_on = bitRead(status.relayStates, i);
        Serial.printf("  Rele %d: %s\n", i + 1, is_on ? "ON" : "OFF");
#ifdef IS_MASTER
        // Shranimo v strukturo kanalov
        kanal[i].state = is_on;
        //Firebase_Update_Relay_State(i, kanal[i].state); // Posodobimo stanje v Firebase
#endif
      }
      lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat
      // Po prejemu statusa lahko prikažemo posodobljeno stanje
      // PrikaziStanjeRelejevNaSerial();

      break;
    }
  //=================================================================================================
  case CommandType::RESPONSE_SENSORS:
    //=================================================================================================
    { // Preberemo podatke senzorjev iz Rele modula
      SensorDataPayload sensors;
      memcpy(&sensors, packet.payload, sizeof(SensorDataPayload));
      Serial.println("Prejeti podatki senzorjev:");
      Temperature = sensors.temperature;
      Humidity = sensors.humidity;
      SoilMoisture = sensors.soil_moisture;
      BatteryVoltage = sensors.battery_voltage;
      // CurrentConsumption = sensors.current_consumption;
      // WaterConsumption = sensors.water_consumption;

      // DODAJ: Signaliziraj čakalni vrsti, da je LoRa odgovor uspešen
      Sensor_OnLoRaResponse(true);

      // Tukaj obdelajte podatke senzorjev, npr. posodobite Firebase
      timestamp = getTime(); // Pridobimo trenutni Unix časovni žig.
      // FirebaseOperation op;
      // op.type = FirebaseTaskType::UPDATE_SENSORS;
      // op.timestamp = timestamp;
      // op.data.sensors = sensors;
      // op.pending = true;

      // if (!Firebase_QueueOperation(op)) { // Dodamo operacijo v čakalno vrsto
      //     Serial.println("[LORA] OPOZORILO: Firebase queue full, sensor data dropped!");
      // }

      // firebase_response_received = false;
      Firebase_Update_Sensor_Data(timestamp, sensors);
      // povratna informacija za update v: Firebase_processResponse
      // PrikaziStanjeSenzorjevNaSerial();
      lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat

      break;
    }

  // =================================================================================================
  case CommandType::RESPONSE_URNIK:
    // =================================================================================================
    {
      // Rele modul je poslal urnik za en kanal
      UrnikPayload received_urnik;
      memcpy(&received_urnik, packet.payload, sizeof(UrnikPayload));
      uint8_t status = received_urnik.status; // vrne se status - 0 OK, >1 error
      uint8_t index = received_urnik.releIndex;

      if (status == (uint8_t)AckStatus::ACK_OK) // status OK
      {

        if (index < 8 && index == currentChannelInProcess)
        {
          // Posodobi lokalno stanje za prejeti kanal
          kanal[index].start_sec = received_urnik.startTimeSec;
          kanal[index].end_sec = received_urnik.endTimeSec;
          formatSecondsToTime(kanal[index].start, sizeof(kanal[index].start), kanal[index].start_sec);
          formatSecondsToTime(kanal[index].end, sizeof(kanal[index].end), kanal[index].end_sec);

          Serial.printf("[INIT] Prejet urnik za kanal %d: %s - %s (Start sec=%d, End sec=%d)\n",
                        index + 1,
                        kanal[index].start,
                        kanal[index].end,
                        kanal[index].start_sec,
                        kanal[index].end_sec);
        }
        else
        {
          Serial.printf("NAPAKA: Neujemanje indeksa pri prejetem urniku! Pričakovan: %d, Prejet: %d\n", currentChannelInProcess, index);
        }
      }
      else
      {
        Serial.printf("NAPAKA: Rele modul vrnil napako pri branju urnika kanala %d, status: %d\n", index + 1, status);
      }

      lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat

      break;
    }
  //=================================================================================================
  case CommandType::RESPONSE_UPDATE_URNIK:
    //=================================================================================================
    {
      // Tukaj obdelajte odgovor po posodobitvi urnika v Rele modulu
      // vrne se status - 0 OK, >1 error in indeks posodobljenega kanala
      UrnikPayload update_response;
      memcpy(&update_response, packet.payload, sizeof(UrnikPayload));

      uint8_t status = update_response.status;   // AckStatus poslan z Rele modula
      uint8_t index = update_response.releIndex; // Indeks kanala (1-8) ki smo ga posodobili

      if (status == (uint8_t)AckStatus::ACK_OK)
      {

        if (index == currentChannelInProcess)
        {
          Serial.printf("ACK OK za posodobitev urnika kanala %d. Posodabljam lokalno stanje.\n", index + 1);

          // Posodobimo našo lokalno kopijo stanja, da se ujema s tem, kar smo poslali
          kanal[index].start_sec = firebase_kanal[index].start_sec;
          kanal[index].end_sec = firebase_kanal[index].end_sec;

          // Pridobimo čas v obliki "HH:MM"
          formatSecondsToTime(kanal[index].start, sizeof(kanal[index].start), kanal[index].start_sec);
          formatSecondsToTime(kanal[index].end, sizeof(kanal[index].end), kanal[index].end_sec);

          // Izpišemo nov urnik za debug
          Serial.printf("  Nov urnik kanala %d: %s - %s\n",
                        index + 1,
                        kanal[index].start,
                        kanal[index].end);
        }
        else
        {
          Serial.printf("NAPAKA: Neujemanje indeksa pri posodobitvi urnika! Pričakovan: %d, Prejet: %d\n", currentChannelInProcess, index);
        }
      }
      else
      {
        Serial.printf("NAPAKA: Rele modul vrnil napako pri posodobitvi urnika kanala %d, status: %d\n", index + 1, status);
      }

      lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat

      break;
    }

  //=================================================================================================
  case CommandType::RESPONSE_INIT_DONE:
    //=================================================================================================
    {
      Serial.println("Rele modul je potrdil, da je inicializacija končana.");
      lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat
      break;
    }

  //=================================================================================================
  case CommandType::NOTIFY_LOW_BATT:
    //=================================================================================================
    {
      Serial.println("!!! OPOZORILO: Nizka napetost baterije na Rele enoti !!!");
      // Pošljemo potrditev (ACK) nazaj na Rele
      // pridobimo ID sporočila iz prejete notifikacije
      uint8_t message_id = packet.messageId;
      Lora_prepare_and_send_response(message_id, CommandType::ACK_NOTIFICATION, &message_id, sizeof(message_id));
      break;
    }

  //=================================================================================================
  case CommandType::NOTIFY_RELAY_STATE_CHANGED:
    //=================================================================================================
    {
      Serial.println("!!! OPOZORILO: Sprememba stanja releja na Rele enoti !!!");
      // Pridobimo kateri rele se je spremenil in njegovo novo stanje
      RelayStatusPayload status;
      memcpy(&status, packet.payload, sizeof(RelayStatusPayload));

      for (int i = 0; i < 8; i++)
      {
        bool is_on = bitRead(status.relayStates, i);
        
        // Preverimo če je prišlo do spremembe stanja
        if (kanal[i].state != is_on)
        {
          Serial.printf("  Rele %d novo stanje: %s\n", i + 1, is_on ? "ON" : "OFF");

          // POSODOBI LOKALNO STANJE PRED POŠILJANJEM V FIREBASE!
          kanal[i].state = is_on;          

          // Pošlji v Firebase
          Firebase_Update_Relay_State(i + 1, is_on);

        }
      }

      // Pošljemo potrditev (ACK) nazaj na Rele
      // pridobimo ID sporočila iz prejete notifikacije
      uint8_t message_id = packet.messageId;
      Lora_prepare_and_send_response(message_id, CommandType::ACK_NOTIFICATION, &message_id, sizeof(message_id));

      break;
    }

  // ... obdelava ostalih tipov odgovorov in notifikacij ...

  //=================================================================================================
  // NOVO: Obdelava podatkov iz INA3221 senzorja
  //=================================================================================================
  case CommandType::RESPONSE_INA_DATA:
  {
      Serial.println("Prejeti podatki iz INA3221 senzorja.");

      // 1. Preberi podatke iz payloada
      // spremenljivka za shranjevanje podatkov iz INA3221
      INA3221_DataPayload ina_data;
      memcpy(&ina_data, packet.payload, sizeof(INA3221_DataPayload));

      // 2. Izpiši prejete podatke za preverjanje
      // const char* kanal_str[3] = {"Battery", "Solar", "Load"};
      // for (int i = 0; i < 3; i++) {
      //     Serial.printf("  %s: Napetost: %.2f V, Tok: %.2f mA, Shunt napetost: %.2f mV, Moč: %.2f mW\n",
      //                   kanal_str[i],
      //                   ina_data.channels[i].bus_voltage,
      //                   ina_data.channels[i].current_mA,
      //                   ina_data.channels[i].shunt_voltage_mV,
      //                   ina_data.channels[i].power_mW);
      // }
      // Serial.printf("  Alert zastavice: 0x%04X\n", ina_data.alert_flags);
      // Serial.printf("  Shunt voltage Sum: %.3f \n", ina_data.shunt_voltage_sum_mV);

      // Signaliziraj čakalni vrsti, da so podatki uspešno prejeti
      Sensor_OnLoRaResponse(true);

      // 3. Posodobi podatke v Firebase
      timestamp = getTime(); // Pridobimo trenutni Unix časovni žig.

      // Pošlji v Firebase
      Firebase_Update_INA_Data(timestamp, ina_data);
      break;
  }


    //=================================================================================================
  case CommandType::NOTIFY_TIME_REQUEST:
    //=================================================================================================
    {
      Serial.println("!!! OPOZORILO: Zahteva za čas na Rele enoti !!!");
      // Pripravimo in pošljemo paket nazaj na Rele
      Rele_sendUTC();                     // Pošljemo ukaz za sinhronizacijo časa
      initState_timeout_start = millis(); // Zaženemo timer za timeout
      break;
    }

    //=================================================================================================
  case CommandType::NOTIFY_RESET_OCCURED:
    //=================================================================================================
    {
      Serial.println("!!! OPOZORILO: Ponastavitev se je zgodila na Rele enoti !!!");
      // Začnemo inicializacijski avtomat znova
      // Najprej pošljemo nazaj ACK
      uint8_t message_id = packet.messageId;
      Lora_prepare_and_send_response(message_id, CommandType::ACK_NOTIFICATION, &message_id, sizeof(message_id));
      reset_occured = true; // nastavimo zastavico za inicializacijski avtomat
      // Ko bo sporočilo poslano , se bo inicializacijski avtomat zagnal znova v loop funkciji
      break;
    }

  default:
    Serial.println("Neznan ukaz v prejetem paketu.");
    break;
  }
  Serial.println("--------------------");
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
  // Ta funkcija bo poklicana iz lora_process_received_packets() v loop()
  lora_initialize(Lora_handle_received_packet);

  Sensor_InitQueue(); // Inicializacija senzorske čakalne vrste

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

  // 5. LoRa obdelava prejetih paketov
  lora_process_received_packets(); // Namesto vsega v LoRaTask

  // 6. Preveri, ali je prišlo do ponovnega zagona Rele modula
  if (reset_occured && !lora_is_busy())
  {
    // Zaženemo inicializacijski avtomat znova
    // init_done_flag = false; // Ponastavi zastavico, da inicializacija ni končana
    // Serial.println("[INIT] Ponovna inicializacija zaradi ponastavitve Rele modula...");
    // initState_timeout_start = millis(); // Zaženemo timer za timeout
    // init_retry_count = 0;               // Ponastavimo števec poskusov
    // currentInitState = InitState::START;
    // reset_occured = false; // Ponastavi zastavico
  }

  Blink_led(Led_status);

  //----------------------------------------------------------------------------------
}
