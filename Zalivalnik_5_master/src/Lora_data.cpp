#include "Lora_data.h"
#include "Lora_handler.h"

// ----------------------------------------------------------------------------
// --- Pomožne funkcije ---
uint16_t calculate_crc(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++)
    {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
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

  // UBaseType_t stackLeft = uxTaskGetStackHighWaterMark(NULL);
  // Serial.printf("[Lora] Stack left: %d\n", stackLeft);

  // // POMEMBNO: Preveri če je stack kritičen
  // if (stackLeft < 2048) {
  //   Serial.println("[LORA] ⚠️ STACK CRITICAL! Prekinjam pošiljanje.");
  //   lora_set_context(LoRaContext::IDLE);
  //   return;
  // }

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

//---------------------------------------------------------------------------------------------------------------------
// Funkcija za obdelavo prejetih LoRa paketov
void Lora_handle_received_packet(const LoRaPacket &packet)
{
  // Prikažemo ID in ukaz prejetega paketa na zaslonu
  char logBuffer[32];
  snprintf(logBuffer, sizeof(logBuffer), "In ID: %d, CMD: %d", packet.messageId, (uint8_t)packet.command);
  Serial.println(logBuffer);

  // Preverimo CRC
  uint16_t received_crc = packet.crc;
  uint16_t calculated_crc = calculate_crc((const uint8_t *)&packet, offsetof(LoRaPacket, crc));
  if (received_crc != calculated_crc)
  {
    Serial.println("NAPAKA: CRC se ne ujema!");
    return;
  }
  Serial.println("CRC OK.");

  CommandType cmd = packet.command;
  uint8_t payload = 0;
  // Preverimo ali je paket odgovor na našo zahtevo ali notifikacija
  if (cmd > CommandType::NOTIFY_BLOCK_START && cmd < CommandType::NOTIFY_BLOCK_END)
  {
    Serial.println("Prejeta notifikacija.");
  }
  else
  {
    Serial.println("Prejet odgovor na zahtevo.");
  }

  // Logika za obdelavo glede na tip ukaza
  switch (cmd)
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
      // Prejet odgovor na CMD_SET_TIME
      TimeResponsePayload sec_from_midnight_payload;
      memcpy(&sec_from_midnight_payload, packet.payload, sizeof(TimeResponsePayload));

      Serial.print("Prejet odgovor: ");
      Serial.println(sec_from_midnight_payload.seconds_since_midnight);

      // Preverimo ali se je čas pravilno nastavil
      if (sec_from_midnight_payload.seconds_since_midnight == secFromMidnight)
      {
        lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat
      }
      else
      {
        Serial.println("NAPAKA: Čas na Rele enoti ni bil pravilno nastavljen.");
      }
      break;
    }

    //=================================================================================================
    case CommandType::RESPONSE_STATUS:
    //=================================================================================================
    { // Prejeto stanje relejev na Rele modulu
      RelayStatusPayload status;
      memcpy(&status, packet.payload, sizeof(RelayStatusPayload));
      Serial.println("Prejeto stanje relejev:");
      // Izpiši stanje relejev iz bitmaske
      for (int i = 0; i < 8; i++)
      {
        bool is_on = bitRead(status.relayStates, i);
        Serial.printf("  Rele %d: %s\n", i + 1, is_on ? "ON" : "OFF");

        // Shranimo v strukturo kanalov
        kanal[i].state = is_on;

      }
      lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat

      break;
    }


    // =================================================================================================
    case CommandType::RESPONSE_URNIK:
    // =================================================================================================
    {
      // Prejet urnik za en kanal
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
      // Prejet odgovor po posodobitvi urnika v Rele modulu
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
    // Prejet odgovor po inicializaciji
    {
      Serial.println("Rele modul je potrdil, da je inicializacija končana.");
      lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat
      break;
    }

    //=================================================================================================
    case CommandType::RESPONSE_SENSORS:
    //=================================================================================================
    { // Prejeti podatki senzorjev iz Rele modula
      SensorDataPayload sensors;
      memcpy(&sensors, packet.payload, sizeof(SensorDataPayload));
      Serial.println("Prejeti podatki senzorjev:");
      Temperature = sensors.temperature;
      Humidity = sensors.humidity;
      SoilMoisture = sensors.soil_moisture;
      BatteryVoltage = sensors.battery_voltage;

      // DODAJ: Signaliziraj čakalni vrsti, da je LoRa odgovor uspešen
      Sensor_OnLoRaResponse(true);

      // Tukaj obdelajte podatke senzorjev, npr. posodobite Firebase
      timestamp = getTime(); // Pridobimo trenutni Unix časovni žig.

      Firebase_Update_Sensor_Data(timestamp, sensors);
      lora_response_received = true; // nastavimo zastavico za inicializacijski avtomat

      break;
    }    

    //=================================================================================================
    case CommandType::RESPONSE_INA_DATA:
    //=================================================================================================
    // Prejeti podatki iz INA3221 senzorja
    {
      Serial.println("Prejeti podatki iz INA3221 senzorja.");

      // Preberi podatke iz payloada
      INA3221_DataPayload ina_data;
      memcpy(&ina_data, packet.payload, sizeof(INA3221_DataPayload));

      // Signaliziraj čakalni vrsti, da so podatki uspešno prejeti
      Sensor_OnLoRaResponse(true);

      // Posodobi podatke v Firebase
      timestamp = getTime(); // Pridobimo trenutni Unix časovni žig.

      // Pošlji v Firebase
      Firebase_Update_INA_Data(timestamp, ina_data);
      break;
    }



  // Obdelava notifikacij
 
    //=================================================================================================
    case CommandType::NOTIFY_LOW_BATT:
    //=================================================================================================
    // Obvestilo o nizki napetosti baterije
    {
      Serial.println("!!! OPOZORILO: Nizka napetost baterije na Rele enoti !!!");
      // Pošljemo potrditev (ACK) nazaj na Rele
      uint8_t message_id = packet.messageId;
      Lora_prepare_and_send_response(message_id, CommandType::ACK_NOTIFICATION, &message_id, sizeof(message_id));
      break;
    }

    //=================================================================================================
    case CommandType::NOTIFY_RELAY_STATE_CHANGED:
    //=================================================================================================
    // Obvestilo o spremembi stanja releja
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
      uint8_t message_id = packet.messageId;
      Lora_prepare_and_send_response(message_id, CommandType::ACK_NOTIFICATION, &message_id, sizeof(message_id));

      break;
    }


    //=================================================================================================
    case CommandType::NOTIFY_TIME_REQUEST:
    //=================================================================================================
    // Obvestilo o zahtevi za čas
    {
      Serial.println("!!! OPOZORILO: Zahteva za čas na Rele enoti !!!");
      // Pripravimo in pošljemo paket nazaj na Rele
      Rele_sendUTC(); // Pošljemo ukaz za sinhronizacijo časa
      break;
    }

    //=================================================================================================
    case CommandType::NOTIFY_RESET_OCCURED:
    //=================================================================================================
    // Obvestilo o ponastavitvi
    {
      Serial.println("!!! OPOZORILO: Ponastavitev se je zgodila na Rele enoti !!!");
      // Pošljemo nazaj ACK
      uint8_t message_id = packet.messageId;
      Lora_prepare_and_send_response(message_id, CommandType::ACK_NOTIFICATION, &message_id, sizeof(message_id));
      reset_occured = true; // nastavimo zastavico za inicializacijski avtomat
      break;
    }

    default:
    Serial.println("Neznan ukaz v prejetem paketu.");
    break;
  }
  Serial.println("--------------------");
}