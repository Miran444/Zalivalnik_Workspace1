
#include "ReleInitialization.h"
#include "Lora_handler.h"


InitState currentInitState = InitState::IDLE; // Trenutno stanje avtomata
// uint8_t currentChannelInProcess = 0;          // Kateri kanal (0-7) trenutno obdelujemo

// --- SPREMENLJIVKE ZA TIMEOUT PRI INICIALIZACIJI ---
unsigned long initState_timeout_start = 0;           // Časovnik za timeout pri čakanju na odgovor
const unsigned long INIT_RESPONSE_TIMEOUT_MS = 5000; // 5 sekund za odgovor
const unsigned long FIREBASE_RESPONSE_TIMEOUT_MS = 15000; // 15 sekund za odgovor iz Firebase

// --- SPREMENLJIVKE ZA PONOVNE POSKUSE ---
const uint8_t MAX_INIT_RETRIES = 3; // Največje število ponovnih poskusov
uint8_t init_retry_count = 0;       // Trenutni števec ponovnih poskusov


//---------------------------------------------------------------------------------------------------------------------
// --- NOVA POMOŽNA FUNKCIJA ZA PREVERJANJE STANJA ČAKANJA V AVTOMATU ---
// Ta funkcija preveri, ali je bil prejet odgovor ali je potekel čas.
// Vrne `true`, če je avtomat prešel v novo stanje (uspeh, napaka ali ponovni poskus).
// Vrne `false`, če še vedno čakamo in ni prišlo do spremembe stanja.
bool check_init_wait_state(bool response_flag, InitState success_state, InitState retry_state, const char *wait_description)
{
  // 1. PRIMER: USPEH - Prejeli smo odgovor
  if (response_flag)
  {
    init_retry_count = 0; // Uspeh! Ponastavimo števec poskusov.
    Serial.printf("[INIT] Uspeh pri čakanju na '%s'.\n", wait_description);
    currentInitState = success_state; // Premaknemo se na naslednje stanje
    return true;                      // Stanje se je spremenilo
  }

  // 2. PRIMER: TIMEOUT - Odgovora nismo prejeli v določenem času
  if (millis() - initState_timeout_start > INIT_RESPONSE_TIMEOUT_MS)
  {
    init_retry_count++; // Povečamo števec poskusov
    Serial.printf("[INIT] Timeout pri čakanju na '%s'. Poskus %d/%d...\n", wait_description, init_retry_count, MAX_INIT_RETRIES);

    // Preverimo, ali smo presegli število poskusov
    if (init_retry_count >= MAX_INIT_RETRIES)
    {
      Serial.println("[INIT] NAPAKA: Preseženo število poskusov. Prekinjam inicializacijo.");
      // displayLogOnLine(LINE_ERROR_WARNING, "[INIT] Fatal Timeout!");
      currentInitState = InitState::ERROR;
      lora_set_context(LoRaContext::IDLE);   // VRNI na normalno delovanje
      initState_timeout_start = millis(); // Ponastavimo časovnik za čakanje v ERROR stanju
    }
    else
    {
      // Nismo presegli poskusov, vrnemo se na prejšnje stanje za ponovni poskus.
      currentInitState = retry_state;
    }
    return true; // Stanje se je spremenilo
  }

  // 3. PRIMER: ČAKANJE - Ni ne odgovora ne timeouta.
  return false; // Še vedno čakamo, stanje se ni spremenilo.

}

//---------------------------------------------------------------------------------------------------------------------
// Zaženemo inicializacijski avtomat
void StartReleInitialization()
{
  Serial.println("[INIT] Začenjam inicializacijo Rele modula...");
  initState_timeout_start = millis(); // Zaženemo timer za timeout
  init_retry_count = 0;               // Ponastavimo števec poskusov
  currentInitState = InitState::READ_FIREBASE_INTERVAL;
}

//---------------------------------------------------------------------------------------------------------------------
// Zaženemo inicializacijski avtomat znova
void triggerReleReinitialization()
{
    init_done_flag = false; // Ponastavi zastavico, da inicializacija ni končana
    Serial.println("[INIT] Ponovna inicializacija zaradi ponastavitve Rele modula...");
    initState_timeout_start = millis(); // Zaženemo timer za timeout
    init_retry_count = 0;               // Ponastavimo števec poskusov
    currentInitState = InitState::START;
    

}    
//---------------------------------------------------------------------------------------------------------------------
// Funkcija za upravljanje inicializacijo releja
void manageReleInitialization()
{

  // Avtomat se ne izvaja, če je končan, v mirovanju ali v stanju napake.
  if (currentInitState == InitState::IDLE || currentInitState == InitState::STOP)
  {
    return;
  }

  // Glavna logika avtomata
  switch (currentInitState)
  {
  // --- NOVO STANJE: Pošlji zahtevo za branje intervala ---
  //============================================================================
  case InitState::READ_FIREBASE_INTERVAL:
  //============================================================================
    // displayLogOnLine(LINE_ERROR_WARNING, "[INIT] Reading Interval...");
    
     // SPREMEMBA: Kličemo funkcijo direktno, ne uporabljamo več čakalne vrste.
    Firebase_readInterval();

    firebase_response_received = false; // Ponastavimo zastavico za odgovor
    initState_timeout_start = millis();      // Ponastavimo timer za timeout
    currentInitState = InitState::WAIT_FOR_INTERVAL_RESPONSE;
    break;


  // --- NOVO STANJE: Čakaj na odgovor za interval ---
  //============================================================================
  case InitState::WAIT_FOR_INTERVAL_RESPONSE:
  //============================================================================  
    if (firebase_response_received)
    {
      Serial.println("[INIT] Uspeh pri branju intervala.");
      currentInitState = InitState::START; // Nadaljujemo z naslednjim korakom (START)
      initState_timeout_start = millis(); // Ponastavimo timer za naslednji korak
    }
    else if (millis() - initState_timeout_start > FIREBASE_RESPONSE_TIMEOUT_MS)
    {
      Serial.println("[INIT] Timeout pri čakanju na 'odgovor za interval'. Uporabljam privzeto vrednost.");
      set_Interval(DEFAULT_SENSOR_READ_INTERVAL_MINUTES); // Nastavimo privzeto vrednost
      currentInitState = InitState::START; // Vseeno nadaljujemo z inicializacijo
    }
    break;

  //============================================================================
  case InitState::START:
    //============================================================================
    Serial.println("--- ZACENJAM INICIALIZACIJO RELE MODULA ---");
    lora_set_context(LoRaContext::INITIALIZATION); // NASTAVI kontekst inicializacije
    // displayLogOnLine(LINE_ERROR_WARNING, "[INIT] Starting...");
    currentInitState = InitState::SEND_TIME;
    break;

  //============================================================================
  case InitState::SEND_TIME:
    //============================================================================
    Serial.println("[INIT] Korak 1: Posiljam cas...");
    // displayLogOnLine(LINE_ERROR_WARNING, "[INIT] UTC time");
    Rele_sendUTC();                     // Pošljemo ukaz za sinhronizacijo časa
    initState_timeout_start = millis(); // Zaženemo timer za timeout
    currentInitState = InitState::WAIT_FOR_TIME_RESPONSE;
    break;

  //============================================================================
  case InitState::WAIT_FOR_TIME_RESPONSE:
    //============================================================================
    if (check_init_wait_state(lora_response_received, InitState::SEND_SENSORS_REQUEST, InitState::SEND_TIME, "odgovor za čas"))
    {
      lora_response_received = false; // Vedno počistimo zastavico po obdelavi
      // if (init_retry_count == 2) // Če je bil to zadnji poskus
      // {
      //   init_retry_count = 0; // Ponastavimo števec poskusov zato da pošiljamo neprekinjeno

      // }
    }
    break;

  //============================================================================
  case InitState::SEND_SENSORS_REQUEST:
    //============================================================================
    Serial.println("[INIT] Korak 3: Posiljam zahtevo za senzorje...");
    // displayLogOnLine(LINE_ERROR_WARNING, "[INIT] Sensors");
    Rele_readSensors(); // Pošljemo zahtevo za branje senzorjev
    initState_timeout_start = millis();
    currentInitState = InitState::WAIT_FOR_SENSORS_RESPONSE;
    lora_response_received = false; // Resetiramo zastavico
    break;

  //============================================================================
  case InitState::WAIT_FOR_SENSORS_RESPONSE:
    //============================================================================
    if (check_init_wait_state(lora_response_received, InitState::SEND_URNIK_REQUEST, InitState::SEND_SENSORS_REQUEST, "odgovor za senzorje"))
    {
      lora_response_received = false;
      if (currentInitState == InitState::SEND_URNIK_REQUEST)
      {
        currentChannelInProcess = 0; // Resetiramo indeks samo ob uspehu
      }
    }
    break;

  //============================================================================
  case InitState::SEND_URNIK_REQUEST:
    //============================================================================
    Serial.printf("[INIT] Posiljam zahtevo za urnik kanala %d...\n", currentChannelInProcess + 1);

    char logBuffer[20];
    snprintf(logBuffer, sizeof(logBuffer), "[INIT] Urnik K%d", currentChannelInProcess + 1);
    // displayLogOnLine(LINE_ERROR_WARNING, logBuffer);

    // Pošljemo zahtevo za EN sam urnik
    Rele_readRelayUrnik(currentChannelInProcess);

    initState_timeout_start = millis(); // Ponastavimo timeout za čakanje na odgovor
    currentInitState = InitState::WAIT_FOR_URNIK_RESPONSE;
    lora_response_received = false; // Resetiramo zastavico
    break;

  //============================================================================
  case InitState::WAIT_FOR_URNIK_RESPONSE:
    //============================================================================
    if (check_init_wait_state(lora_response_received, InitState::SEND_URNIK_REQUEST, InitState::SEND_URNIK_REQUEST, "odgovor za urnik"))
    {
      lora_response_received = false;
      // Posebna logika za to stanje
      if (currentInitState == InitState::SEND_URNIK_REQUEST)
      { // Če je bil klic uspešen
        currentChannelInProcess++;
        if (currentChannelInProcess >= 8)
        {
          Serial.println("[INIT] Vsi urniki uspesno prebrani.");
          currentInitState = InitState::READ_FROM_FIREBASE;
          currentChannelInProcess = 0;
        }
      }
    }
    break;

  //============================================================================
  case InitState::READ_FROM_FIREBASE:
    //============================================================================
    Serial.printf("[INIT] Branje urnika iz Firebase za kanal %d...\n", currentChannelInProcess + 1);
    
    char logBufferFb[20];
    snprintf(logBufferFb, sizeof(logBufferFb), "[INIT] Branje K%d", currentChannelInProcess + 1);
    // displayLogOnLine(LINE_ERROR_WARNING, logBufferFb);

    Firebase_readKanalUrnik(currentChannelInProcess); // Pošljemo zahtevo za branje urnika iz Firebase za trenutni kanal

    currentInitState = InitState::WAIT_FOR_FIREBASE_RESPONSE;
    initState_timeout_start = millis(); // za timeout
    firebase_response_received = false; // Resetiramo zastavico
    break;

  //============================================================================
  case InitState::WAIT_FOR_FIREBASE_RESPONSE:
    //============================================================================
    // Za Firebase uporabimo `firebase_response_received` zastavico
    if (check_init_wait_state(firebase_response_received, InitState::READ_FROM_FIREBASE, InitState::READ_FROM_FIREBASE, "odgovor iz Firebase"))
    {
      firebase_response_received = false;
      // Posebna logika za to stanje
      if (init_retry_count == 0)  // Če je bil klic uspešen
      { 
        if (currentInitState == InitState::READ_FROM_FIREBASE)  // nam pove da smo v inicializaciji
        {        
          currentChannelInProcess++;
          if (currentChannelInProcess >= 8)
          {
            Serial.println("[INIT] Vsi urniki iz Firebase uspesno prebrani.");
            currentInitState = InitState::COMPARE_AND_SEND_LORA;
            currentChannelInProcess = 0;
          }
        }
      }
      else
      { // Če je bil klic neuspešen (ponovni poskus)
        // currentChannelInProcess ostane enak, da ponovimo branje za isti kanal
      }
    }
    break;

  //============================================================================
  case InitState::COMPARE_AND_SEND_LORA:
    //============================================================================
    // Primerjamo podatke iz Firebase (firebase_kanal) z lokalnimi (kanal)

    if (currentChannelInProcess >= 8)
    {
      // Končali smo z vsemi kanali, obvestimo Rele, da je inicializacija končana
      currentInitState = InitState::SEND_STATUS_REQUEST;
      currentChannelInProcess = 0; // Resetiramo za morebitno kasnejšo uporabo
    }
    else
    {
      if (firebase_kanal[currentChannelInProcess].start_sec != kanal[currentChannelInProcess].start_sec ||
          firebase_kanal[currentChannelInProcess].end_sec != kanal[currentChannelInProcess].end_sec)
      {
        Serial.printf("[INIT] Razlika za kanal %d. Posiljam posodobitev na Rele...\n", currentChannelInProcess + 1);

        Rele_updateRelayUrnik(
            currentChannelInProcess,
            firebase_kanal[currentChannelInProcess].start_sec,
            firebase_kanal[currentChannelInProcess].end_sec);

        currentInitState = InitState::WAIT_FOR_UPDATE_URNIK;
        initState_timeout_start = millis();
        lora_response_received = false; // Resetiramo zastavico
        // Odgovor v Lora_handle_received_packet - RESPONSE_UPDATE_URNIK.
      }
      else // Ni razlike, gremo direktno na naslednji kanal
      {
        Serial.printf("[INIT] Urnik za kanal %d je ze usklajen.\n", currentChannelInProcess + 1);
        currentChannelInProcess++;
        currentInitState = InitState::COMPARE_AND_SEND_LORA; // Naslednji krog
      }
    }

    break;
  //============================================================================
  case InitState::WAIT_FOR_UPDATE_URNIK:
    //============================================================================
    // Čakamo na odgovor iz `Lora_handle_received_packet` - RESPONSE_UPDATE_URNIK.
    if (check_init_wait_state(lora_response_received, InitState::WAIT_FOR_UPDATE_URNIK, InitState::WAIT_FOR_UPDATE_URNIK, "odgovor update urnik"))
    {
      lora_response_received = false;
      currentChannelInProcess++;                           // Povečamo indeks za naslednji kanal
      currentInitState = InitState::COMPARE_AND_SEND_LORA; // Naslednji krog
    }

    break;


  //============================================================================
  case InitState::SEND_STATUS_REQUEST:
    //============================================================================
    Serial.println("[INIT] Korak 2: Posiljam zahtevo za status relejev...");
    // displayLogOnLine(LINE_ERROR_WARNING, "[INIT] Status");
    Rele_readRelaysStatus();
    initState_timeout_start = millis();
    currentInitState = InitState::WAIT_FOR_STATUS_RESPONSE;
    break;

  //============================================================================
  case InitState::WAIT_FOR_STATUS_RESPONSE:
    //============================================================================
    if (check_init_wait_state(lora_response_received, InitState::FIREBASE_UPDATE_STATUS, InitState::SEND_STATUS_REQUEST, "odgovor za status"))
    { 
      lora_response_received = false;
    }
    // Odgovor v Lora_handle_received_packet, - RESPONSE_STATUS.

    break;

  //============================================================================
  case InitState::FIREBASE_UPDATE_STATUS:
    //============================================================================
    Serial.printf("[INIT] Posodabljanje stanja v Firebase za kanal %d...\n", currentChannelInProcess + 1);
    
    char logBufferUpdate[24];
    snprintf(logBufferUpdate, sizeof(logBufferUpdate), "[INIT] Posodabljanje K%d", currentChannelInProcess + 1);
    // displayLogOnLine(LINE_ERROR_WARNING, logBufferUpdate);

    Firebase_Update_Relay_State(currentChannelInProcess + 1, kanal[currentChannelInProcess].state); // Pošljemo zahtevo za posodobitev stanja v Firebase za trenutni kanal

    currentInitState = InitState::WAIT_FOR_FIREBASE_UPDATE_STATUS;
    initState_timeout_start = millis(); // za timeout
    firebase_response_received = false; // Resetiramo zastavico
    break;

  //============================================================================
  case InitState::WAIT_FOR_FIREBASE_UPDATE_STATUS:
    //============================================================================
    // Za Firebase uporabimo `firebase_response_received` zastavico
    if (check_init_wait_state(firebase_response_received, InitState::FIREBASE_UPDATE_STATUS, InitState::FIREBASE_UPDATE_STATUS, "odgovor iz Firebase"))
    {
      firebase_response_received = false;
      // Posebna logika za to stanje
      if (currentInitState == InitState::FIREBASE_UPDATE_STATUS)
      { // Če je bil klic uspešen
        currentChannelInProcess++;
        if (currentChannelInProcess >= 8)
        {
          Serial.println("[INIT] Vsi statusi kanalov iz Firebase uspesno posodobljeni.");
          currentInitState = InitState::INIT_DONE;
          currentChannelInProcess = 0;
        }
      }
    }
    break;

  //============================================================================
  case InitState::INIT_DONE:
    //============================================================================
    // Pošljemo signal Rele modulu, da je inicializacija končana

    Serial.println("[INIT] Obvestilo Rele modulu, da je inicializacija končana...");
    // displayLogOnLine(LINE_ERROR_WARNING, "[INIT] Finalizing...");
    // Pošljemo paket z ukazom CMD_INIT_DONE.
    Lora_prepare_and_send_packet(CommandType::CMD_INIT_DONE, nullptr, 0);
    currentInitState = InitState::WAIT_FOR_INIT_COMPLETE;
    initState_timeout_start = millis();
    lora_response_received = false; // Resetiramo zastavico

    break;

  //============================================================================
  case InitState::WAIT_FOR_INIT_COMPLETE:
    //============================================================================
    // Čakamo na potrditev, da je inicializacija končana
    if (check_init_wait_state(lora_response_received, InitState::DONE, InitState::INIT_DONE, "odgovor za init done"))
    {
      lora_response_received = false;
      // currentInitState = InitState::DONE;
    }

    break;

  //============================================================================
  case InitState::DONE:
    //============================================================================
    Serial.println("[INIT] Inicializacija uspesno zakljucena.");
    // displayLogOnLine(LINE_ERROR_WARNING, "[INIT] Done");
    lora_set_context(LoRaContext::IDLE); // Vrni na IDLE stanje
    init_done_flag = true;              // Nastavi zastavico, da je inicializacija zaključena
    currentInitState = InitState::STOP; // Postavi avtomat v stanje mirovanja
    // lastSync = millis();                // Zapomni si čas zadnje sinhronizacije
    break;

  //============================================================================
  case InitState::ERROR:
    //============================================================================
    // Tukaj moramo počakati 5 minut in znova poskusiti inicializacijo
    if (millis() - initState_timeout_start > 300000)
    { // 5 minut (300000 ms)
      Serial.println("[INIT] Ponovni poskus inicializacije po napaki...");

      currentInitState = InitState::START;
      init_retry_count = 0; // Ponastavimo števec poskusov
    }

    break;

    // ... ostala stanja (STOP) ...
  }

  // ... preostanek loop() funkcije ...
}