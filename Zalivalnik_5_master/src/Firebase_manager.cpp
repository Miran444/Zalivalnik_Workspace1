/**
 * The beare minimum code example for using Realtime Database service.
 *
 * The steps which are generally required are explained below.
 *
 * Step 1. Include the network, SSL client and Firebase libraries.
 * ===============================================================
 *
 * Step 2. Define the user functions that requred for the library usage.
 * =====================================================================
 *
 * Step 3. Define the authentication config (identifier) class.
 * ============================================================
 * In the Firebase/Google Cloud services REST APIs, the auth tokens are used for authentication/authorization.
 *
 * The auth token is a short-lived token that will be expired in 60 minutes and need to be refreshed or re-created when it expired.
 *
 * There can be some special use case that some services provided the non-authentication usages e.g. using database secret
 * in Realtime Database, setting the security rules in Realtime Database, Firestore and Firebase Storage to allow public read/write access.
 *
 * The UserAuth (user authentication with email/password) is the basic authentication for Realtime Database,
 * Firebase Storage and Firestore services except for some Firestore services that involved with the Google Cloud services.
 *
 * It stores the email, password and API keys for authentication process.
 *
 * In Google Cloud services e.g. Cloud Storage and Cloud Functions, the higest authentication level is required and
 * the ServiceAuth class (OAuth2.0 authen) and AccessToken class will be use for this case.
 *
 * While the CustomAuth provides the same authentication level as user authentication unless it allows the custom UID and claims.
 *
 * Step 4. Define the authentication handler class.
 * ================================================
 * The FirebaseApp actually works as authentication handler.
 * It also maintains the authentication or re-authentication when you place the FirebaseApp::loop() inside the main loop.
 *
 * Step 5. Define the SSL client.
 * ==============================
 * It handles server connection and data transfer works.
 *
 * In this beare minimum example we use only one SSL client for all processes.
 * In some use cases e.g. Realtime Database Stream connection, you may have to define the SSL client for it separately.
 *
 * Step 6. Define the Async Client.
 * ================================
 * This is the class that is used with the functions where the server data transfer is involved.
 * It stores all sync/async taks in its queue.
 *
 * It requires the SSL client and network config (identifier) data for its class constructor for its network re-connection
 * (e.g. WiFi and GSM), network connection status checking, server connection, and data transfer processes.
 *
 * This makes this library reliable and operates precisely under various server and network conditions.
 *
 * Step 7. Define the class that provides the Firebase/Google Cloud services.
 * ==========================================================================
 * The Firebase/Google Cloud services classes provide the member functions that works with AsyncClient.
 *
 * Step 8. Start the authenticate process.
 * ========================================
 * At this step, the authentication credential will be used to generate the auth tokens for authentication by
 * calling initializeApp.
 *
 * This allows us to use different authentications for each Firebase/Google Cloud services with different
 * FirebaseApps (authentication handler)s.
 *
 * When calling initializeApp with timeout, the authenication process will begin immediately and wait at this process
 * until it finished or timed out. It works in sync mode.
 *
 * If no timeout was assigned, it will work in async mode. The authentication task will be added to async client queue
 * to process later e.g. in the loop by calling FirebaseApp::loop.
 *
 * The workflow of authentication process.
 *
 * -----------------------------------------------------------------------------------------------------------------
 *  Setup   |    FirebaseApp [account credentials/tokens] ───> InitializeApp (w/wo timeout) ───> FirebaseApp::getApp
 * -----------------------------------------------------------------------------------------------------------------
 *  Loop    |    FirebaseApp::loop  ───> FirebaseApp::ready ───> Firebase Service API [auth token]
 * ---------------------------------------------------------------------------------------------------
 *
 * Step 9. Bind the FirebaseApp (authentication handler) with your Firebase/Google Cloud services classes.
 * ========================================================================================================
 * This allows us to use different authentications for each Firebase/Google Cloud services.
 *
 * It is easy to bind/unbind/change the authentication method for different Firebase/Google Cloud services APIs.
 *
 * Step 10. Set the Realtime Database URL (for Realtime Database only)
 * ===================================================================
 *
 * Step 11. Maintain the authentication and async tasks in the loop.
 * ==============================================================
 * This is required for authentication/re-authentication process and keeping the async task running.
 *
 * Step 12. Checking the authentication status before use.
 * =======================================================
 * Before calling the Firebase/Google Cloud services functions, the FirebaseApp::ready() of authentication handler that bined to it
 * should return true.
 *
 * Step 13. Process the results of async tasks the end of the loop.
 * ============================================================================
 * This requires only when async result was assigned to the Firebase/Google Cloud services functions.
 */

/**
 * The example to stream changes to multiple locations in Realtime Database.
 *
 * This example uses the UserAuth class for authentication.
 * See examples/App/AppInitialization for more authentication examples.
 *
 * For the complete usage guidelines, please read README.md or visit https://github.com/mobizt/FirebaseClient
 */

#include "Firebase_manager.h" // Insert your network credentials
//#include <FirebaseClient.h>
#include "ExampleFunctions.h"

WiFiClient basic_client1, basic_client2;

// The ESP_SSLClient uses PSRAM by default (if it is available), for PSRAM usage, see https://github.com/mobizt/FirebaseClient#memory-options
// For ESP_SSLClient documentation, see https://github.com/mobizt/ESP_SSLClient
ESP_SSLClient ssl_client, stream_ssl_client1;

using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client), streamClient1(stream_ssl_client1);

UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);
FirebaseApp app;
RealtimeDatabase Database;
// AsyncResult streamResult1, streamResult2;

char databasePath[48];
char sensorPath[64];
char inaPath[64];
char kanaliPath[64];
char chartIntervalPath[72];
char examplesPath[64];
char examplePath1[64];
char examplePath2[64];
char uid[32];
unsigned long ms = 0;
bool extract_uid = false;

// Dodajte globalno spremenljivko za sledenje minimalnega heap-a
uint32_t minHeapDuringAuth = 0xFFFFFFFF;
uint32_t heapBeforeAuth = 0;

Kanal firebase_kanal[8] = {
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0},
    {false, "00:00", 0, "00:00", 0}}; // Končamo z definicijo Firebase kanalov

volatile bool newChannelDataAvailable;
ChannelUpdateData channelUpdate;
// Kanal firebase_kanal[8]; // Polje struktur za kanale

bool ssl_avtentikacija = false; // Spremenljivka za shranjevanje stanja SSL avtentikacije aClient

bool firebase_response_received = false; // Ali smo prejeli odgovor iz Firebase?
bool dummyOperationSent = false;  // Spremenljivka za sledenje dummy operaciji

// DODAJ retry mehanizem:
static unsigned long lastFirebaseOperationTime = 0;
static uint8_t firebaseRetryCount = 0;
const uint8_t MAX_FIREBASE_RETRIES = 3;
const unsigned long FIREBASE_RESPONSE_TIMEOUT = 5000; // 5 sekund

// Struktura za shranjevanje zadnje operacije (za retry)
struct LastFirebaseOperation {
    enum class Type { NONE, UPDATE_SENSOR, UPDATE_INA, UPDATE_RELAY, GET_URNIK, GET_INTERVAL } type;
    union {
        struct {
            unsigned long timestamp;                 
            // SensorDataPayload data;  // POPRAVIT KO BO LORA INSTALIRANA
        } sensor;
        struct {
            unsigned long timestamp;
            // INA3221_DataPayload data;  // POPRAVIT KO BO LORA INSTALIRANA
        } ina;
        struct {
            int kanal;
            bool state;
        } relay;
        struct {
            uint8_t kanalIndex;
        } urnik;
    } data;
    bool waiting_for_response;
} lastOperation = {LastFirebaseOperation::Type::NONE, {}, false};


//------------------------------------------------------------------------------------------------
// Funkcija za inicializacijo Firebase
void Firebase_setup()
{
  heapBeforeAuth = ESP.getFreeHeap();
  Firebase.printf("Free Heap BEFORE Firebase: %d\n", heapBeforeAuth);
  Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);

  ssl_client.setClient(&basic_client1);
  stream_ssl_client1.setClient(&basic_client2);
  // stream_ssl_client2.setClient(&basic_client3);

  ssl_client.setInsecure();
  stream_ssl_client1.setInsecure();
  // stream_ssl_client2.setInsecure();

  ssl_client.setBufferSizes(2048, 1024);
  stream_ssl_client1.setBufferSizes(2048, 1024);
  // stream_ssl_client2.setBufferSizes(2048, 1024);

  // In case using ESP8266 without PSRAM and you want to reduce the memory usage,
  // you can use WiFiClientSecure instead of ESP_SSLClient with minimum receive and transmit buffer size setting as following.
  // ssl_client1.setBufferSizes(1024, 512);
  // ssl_client2.setBufferSizes(1024, 512);
  // ssl_client3.setBufferSizes(1024, 512);
  // Note that, because the receive buffer size was set to minimum safe value, 1024, the large server response may not be able to handle.
  // The WiFiClientSecure uses 1k less memory than ESP_SSLClient.

  ssl_client.setDebugLevel(1);
  stream_ssl_client1.setDebugLevel(1);
  // stream_ssl_client2.setDebugLevel(1);

  // In ESP32, when using WiFiClient with ESP_SSLClient, the WiFiClient was unable to detect
  // the server disconnection in case server session timed out and the TCP session was kept alive for reusage.
  // The TCP session timeout in seconds (>= 60 seconds) can be set via `ESP_SSLClient::setSessionTimeout`.
  ssl_client.setSessionTimeout(150);
  stream_ssl_client1.setSessionTimeout(150);
  // stream_ssl_client2.setSessionTimeout(150);

  Serial.println("Initializing app...");
  minHeapDuringAuth = ESP.getFreeHeap(); // Reset
  Firebase.printf("Free Heap: %d\n", minHeapDuringAuth);

  initializeApp(aClient, app, getAuth(user_auth), Firebase_processResponse, "🔐 authTask");

  // Or intialize the app and wait.
  // initializeApp(aClient, app, getAuth(user_auth), 120 * 1000, auth_debug_print);

  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  // streamClient2.setSSEFilters("get,put,patch,keep-alive,cancel,auth_revoked");

  // The "unauthenticate" error can be occurred in this case because we don't wait
  // the app to be authenticated before connecting the stream.
  // This is ok as stream task will be reconnected automatically when the app is authenticated.
  // Async call with AsyncResult for returning result.
  // Database.get(streamClient1, examplePath1, processData, true /* SSE mode (HTTP Streaming) */, "streamTask1");
  // Database.get(streamClient2, examplePath2, processData, true /* SSE mode (HTTP Streaming) */, "streamTask2");
}

//------------------------------------------------------------------------------------------------
// Stream callback function
void streamCallback(AsyncResult &streamResult)
{
  // Exits when no result is available when calling from the loop.
  if (!streamResult.isResult()) return;

  if (streamResult.isEvent()) Firebase.printf("[F_STREAM] Event task: %s, msg: %s, code: %d\n", streamResult.uid().c_str(), streamResult.eventLog().message().c_str(), streamResult.eventLog().code());
  if (streamResult.isDebug()) Firebase.printf("[F_STREAM] Debug task: %s, msg: %s\n", streamResult.uid().c_str(), streamResult.debug().c_str());

  if (streamResult.isError())
  {
    Firebase.printf("[STREAM] ⚠️ Error task: %s, msg: %s, code: %d\n",
                    streamResult.uid().c_str(),
                    streamResult.error().message().c_str(),
                    streamResult.error().code());

    // // ✅ Označi, da stream ni več aktiven
    // if (streamResult.error().code() == -118 || // operation cancelled
    //     streamResult.error().code() == -1 ||   // TCP failed
    //     streamResult.error().code() == 401)    // unauthorized
    // {
    //   stream_is_active = false;
    //   FirebaseNeedsReconnect = true;
    // }

    return;
  }

  // Check if the stream result is available
  if (streamResult.available())
  {
    RealtimeDatabaseResult &stream = streamResult.to<RealtimeDatabaseResult>();
    if (stream.isStream())
    {
      //Serial.println("----------------------------");
      Firebase.printf("[STREAM] event: %s\n", stream.event().c_str());
   
      // if (stream.event() == "keep-alive")   // če je event "keep-alive"
      // {
      //   lastFirebaseActivityTime = millis();
      // }

      // Najprej shrani v String objekt
      String path_str = stream.dataPath();
      const char* path = path_str.c_str();  // Zdaj je kazalec veljaven
      // Firebase.printf("[STREAM] String path: %s\n", path);

      //-----------------------------------
      // Ali je sprememba znotraj /Kanali?
      if (strncmp(path, "/Kanali", 7) == 0)
      {
        // PRAV TAKO shranimo payload v String
        String payload_str = stream.data();
        const char* payload = payload_str.c_str();  // Kazalec na veljavno pomnilniško lokacijo
        Firebase.printf("[STREAM] Payload: %s\n", payload);

        int kanalIndex = -1;
        const char* kanalStr = strstr(payload, "kanal");
        if (kanalStr != NULL) {
            // Preskočimo "kanal" in pretvorimo številko v int
            kanalIndex = atoi(kanalStr + 5);
        }

        if (kanalIndex != -1)
        {
          // Preveri, ali je sprememba v 'state' polju
          const char* stateKey = "\"state\":";
          const char* statePtr = strstr(payload, stateKey);
          if (statePtr != NULL) {
            Serial.printf("[STREAM] Zaznana sprememba stanja, ne urnika. Ignoriram.\n");
            return; // NE pošiljamo nazaj na Rele!
          }

          // Samo urnik (start_sec/end_sec) procesiramo naprej
          int startSec = firebase_kanal[kanalIndex - 1].start_sec;
          int endSec = firebase_kanal[kanalIndex - 1].end_sec;
          bool dataChanged = false; // Zastavica, ki pove, ali je prišlo do spremembe

          // Preverimo, ali se je spremenil 'start_sec'
          const char* startSecKey = "start_sec\":";
          const char* startSecPtr = strstr(payload, startSecKey);
          if (startSecPtr != NULL)
          {
            // Premaknemo kazalec za dolžino ključa, da pridemo do vrednosti
            startSec = atoi(startSecPtr + strlen(startSecKey));
            Firebase.printf("[STREAM] Kanal %d je spremenjen, nov začetni čas: %d\n", kanalIndex, startSec);
            firebase_kanal[kanalIndex - 1].start_sec = startSec;
            dataChanged = true;
          }

          // Preverimo, ali se je spremenil 'end_sec'
          const char* endSecKey = "end_sec\":";
          const char* endSecPtr = strstr(payload, endSecKey);
          if (endSecPtr != NULL)
          {
            // Premaknemo kazalec za dolžino ključa, da pridemo do vrednosti
            endSec = atoi(endSecPtr + strlen(endSecKey));
            Firebase.printf("[STREAM] Kanal %d je spremenjen, nov končni čas: %d\n", kanalIndex, endSec);
            firebase_kanal[kanalIndex - 1].end_sec = endSec;
            dataChanged = true;
          }
          // Pošljemo posodobitev samo, če je dejansko prišlo do spremembe
          if (dataChanged)
          {
            // Nastavimo zastavico, da je na voljo nova posodobitev za pošiljanje na rele
            channelUpdate.kanalIndex = kanalIndex;
            channelUpdate.start_sec = startSec;
            channelUpdate.end_sec = endSec;
            newChannelDataAvailable = true;
          }
        }
      }
      //-----------------------------------------
      // Ali je sprememba poti /charts/Interval?
      else if (strcmp(path, "/charts/Interval") == 0)
      {
        uint8_t chartInterval = stream.to<uint8_t>();
        set_Interval(chartInterval);
        Firebase.printf("[STREAM] Nastavljen interval branja senzorjev: %d minut\n", chartInterval);
      }
    }
    else
    {
      Serial.println("----------------------------");
      Firebase.printf("[STREAM] task: %s, payload: %s\n", streamResult.uid().c_str(), streamResult.c_str());
    }
  }
}

//----------------------------------------------------------------------------------------------------------
// Funkcija za obdelavo posodobitev iz Firebase
bool Firebase_handleStreamUpdate(int kanalIndex, int start_sec, int end_sec)
{
  int index = kanalIndex - 1;
  bool updated = false;

  if (index >= 0 && index < 8)
  {
    if (start_sec != -1)
    {
      firebase_kanal[index].start_sec = start_sec;
      formatSecondsToTime(firebase_kanal[index].start, sizeof(firebase_kanal[index].start), start_sec);
    }
    if (end_sec != -1)
    {
      firebase_kanal[index].end_sec = end_sec;
      formatSecondsToTime(firebase_kanal[index].end, sizeof(firebase_kanal[index].end), end_sec);
    }

    // Pripravi podatke za posodobitev
    pendingUpdateData.kanalIndex = index;
    pendingUpdateData.start_sec = firebase_kanal[index].start_sec;
    pendingUpdateData.end_sec = firebase_kanal[index].end_sec;
    updated = true;

  }
  else
  {
    Firebase.printf("[STREAM] Ni sprememb urnika, samo stanje. Ne pošiljam posodobitve.\n");
  }

  return updated;
}

//-------------------------------------------------------------------------------------------------------
// Pomožna funkcija za ekstrakcijo številke za ključem
int extractIntValue(const char* json, const char* key) {
  if (!json || !key) return -1;  // Preveri NULL kazalce
  
  const char* keyPos = strstr(json, key);
  if (!keyPos) return -1;
  
  keyPos += strlen(key);
  
  // Preskočimo ': ' ali '":"' in whitespace
  while (*keyPos && (*keyPos == ':' || *keyPos == ' ' || 
                     *keyPos == '"' || *keyPos == '\t')) {
    keyPos++;
  }
  
  // Preveri, ali je naslednji znak številka
  if (!isdigit(*keyPos) && *keyPos != '-') {
    return -1;
  }
  
  return atoi(keyPos);
}

//------------------------------------------------------------------------------------------------------------------------
// Funkcija za preverjanje stanja SSL povezave
void Firebase_Check_Active_State(bool wait) {

  // če je wait true, preverimo stanje vsakih 15 sekund, če je false, preverimo takoj
  static unsigned long lastCheckTime = 0;
  bool checkNow = !wait || (millis() - lastCheckTime > 15000);

  if (checkNow) {
    lastCheckTime = millis();
    Firebase.printf("[F_DEBUG] Stream Active tasks: %d, Stream SSL connected: %d\n",
                  streamClient1.taskCount(),
                  stream_ssl_client1.connected());


    Firebase.printf("[F_DEBUG] aClient Active tasks: %d, aClient SSL connected: %d, Auth ready: %d\n",
                  aClient.taskCount(),
                  ssl_client.connected(),
                  ssl_avtentikacija);
  }
}


//------------------------------------------------------------------------------------------------------------------------
// Funkcija za preverjanje če je Firebase pripravljen
bool Firebase_IsReady()
{
  // 1. Preveri heap
  size_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < 60000) {
    Firebase.printf("[F_HEAP] ⚠️ Premalo heap-a: %d B\n", freeHeap);
    return false;
  }

  // 2. Preveri app status
  if (!app.ready())
  {
    Firebase.printf("[F_READY] Firebase app ni pripravljen.\n");
    return false;
  }

  // 3. Počakaj na avtentikacijo
  if (!ssl_avtentikacija) {
    return false;
  }

  return true;
}


//------------------------------------------------------------------------------------------------------------------------
// Funkcija za posodobitev podatkov iz Firebase (chart interval)
void Firebase_readInterval()
{
  Firebase.printf("[F_GET_INTERVAL] Reading interval (poskus %d/%d)...\n",
                  firebaseRetryCount + 1, MAX_FIREBASE_RETRIES);

  lastOperation.type = LastFirebaseOperation::Type::GET_INTERVAL;
  lastOperation.waiting_for_response = true;
  lastFirebaseOperationTime = millis();
  firebase_response_received = false;

  if (!Firebase_IsReady())
  {
    Firebase.printf("[F_GET_INTERVAL] Firebase ni pripravljen.\n");
    return;
  }

  Database.get(streamClient1, chartIntervalPath, Firebase_processResponse, false, "getChartIntervalTask");
}

//----------------------------------------------------------------------------------------------------------------------
// Funkcija ki prebere urnik iz Firebase in ga shrani v globalno spremenljivko firebase_kanal
void Firebase_readKanalUrnik(uint8_t kanalIndex)
{
  char path_buffer[96];
  snprintf(path_buffer, sizeof(path_buffer), "%s%d", kanaliPath, kanalIndex + 1);

  Firebase.printf("[F_GET_URNIK] Reading schedule (poskus %d/%d)...\n",
                  firebaseRetryCount + 1, MAX_FIREBASE_RETRIES);

  // NOVO: Shrani operacijo
  lastOperation.type = LastFirebaseOperation::Type::GET_URNIK;
  lastOperation.data.urnik.kanalIndex = kanalIndex;
  lastOperation.waiting_for_response = true;
  lastFirebaseOperationTime = millis();
  firebase_response_received = false;

  if (!Firebase_IsReady())
  {
    Firebase.printf("[F_GET_URNIK] Firebase ni pripravljen.\n");
    return;
  }

  Database.get(streamClient1, path_buffer, Firebase_processResponse, false, "getUrnikTask");
}

//------------------------------------------------------------------------------------------------------------------------
// Funkcija za posodobitev podatkov v Firebase (eno polje kanala)
void Firebase_Update_Relay_State(int kanal, bool state)
{
  char path_buffer[100];
  snprintf(path_buffer, sizeof(path_buffer), "%s%d/state", kanaliPath, kanal);
  const char *state_payload = state ? "ON" : "OFF";

  Firebase.printf("[F_UPDATE_RELAY] Sending relay state (poskus %d/%d)...\n",
                  firebaseRetryCount + 1, MAX_FIREBASE_RETRIES);

  // NOVO: Shrani operacijo
  lastOperation.type = LastFirebaseOperation::Type::UPDATE_RELAY;
  lastOperation.data.relay.kanal = kanal;
  lastOperation.data.relay.state = state;
  lastOperation.waiting_for_response = true;
  lastFirebaseOperationTime = millis();
  firebase_response_received = false;

  if (!Firebase_IsReady())
  {
    Firebase.printf("[F_UPDATE_RELAY] Firebase ni pripravljen.\n");
    return;
  }

  Database.set(streamClient1, path_buffer, state_payload, Firebase_processResponse, "updateStateTask");
}

//------------------------------------------------------------------------------------------------------------------------
// Funkcija za posodobitev podatkov senzorjev v Firebase
void Firebase_Update_Sensor_Data(unsigned long timestamp, const SensorDataPayload &sensors)
{
  // Pripravimo bufferje za pretvorbo float vrednosti v nize
  char temp_str[8];
  char hum_str[8];
  char soil_str[8];
  char time_str[12];

  // Varno pretvorimo vrednosti v nize
  dtostrf(sensors.temperature, 4, 2, temp_str);
  dtostrf(sensors.humidity, 4, 2, hum_str);
  dtostrf(sensors.soil_moisture, 4, 2, soil_str);
  snprintf(time_str, sizeof(time_str), "%lu", timestamp);

  // Uporabimo vgrajeni JsonWriter knjižnice Firebase
  object_t json, obj1, obj2, obj3, obj4;
  JsonWriter writer;

  writer.create(obj1, "temperature", string_t(temp_str));
  writer.create(obj2, "humidity", string_t(hum_str));
  writer.create(obj3, "soil_moisture", string_t(soil_str));
  writer.create(obj4, "timestamp", string_t(time_str));
  writer.join(json, 4, obj1, obj2, obj3, obj4);

  // Sestavimo pot brez uporabe String objekta
  char path_buffer[96];
  snprintf(path_buffer, sizeof(path_buffer), "%s/%lu", sensorPath, timestamp);

  Firebase.printf("[F_UPDATE_SENSOR] Sending sensor data (poskus %d/%d)...\n",
                  firebaseRetryCount + 1, MAX_FIREBASE_RETRIES);

  // NOVO: Shrani operacijo za morebitni retry
  lastOperation.type = LastFirebaseOperation::Type::UPDATE_SENSOR;
  lastOperation.data.sensor.timestamp = timestamp;
  // lastOperation.data.sensor.data = sensors;      // POPRAVIT KO BO LORA INSTALIRANA
  lastOperation.waiting_for_response = true;
  lastFirebaseOperationTime = millis();
  firebase_response_received = false;

  if (!Firebase_IsReady())
  {
    Firebase.printf("[F_UPDATE_SENSOR] Firebase ni pripravljen za posodobitev podatkov senzorjev!\n");
    return;
  }
  // Pošljemo podatke
  Database.set<object_t>(streamClient1, path_buffer, json, Firebase_processResponse, "updateSensorTask");

}

//----------------------------------------------------------------------------------------------------------------------
// Funkcija za pošiljanje INA podatkov v Firebase (PREDELANO z JsonWriter)
void Firebase_Update_INA_Data(unsigned long timestamp, const INA3221_DataPayload &data)
{
  JsonWriter writer;
  object_t final_json;

  // Pripravimo bufferje za pretvorbo float vrednosti v nize
  char value_str[12];

  // Ustvarimo objekte za vsak kanal posebej
  object_t ch0_obj, ch1_obj, ch2_obj;
  object_t ch0_items[4], ch1_items[4], ch2_items[4];

  // Kanal 0
  dtostrf(data.channels[0].bus_voltage, 4, 2, value_str);
  writer.create(ch0_items[0], "bus_voltage_V", string_t(value_str));
  dtostrf(data.channels[0].shunt_voltage_mV, 4, 2, value_str);
  writer.create(ch0_items[1], "shunt_voltage_mV", string_t(value_str));
  dtostrf(data.channels[0].current_mA, 4, 2, value_str);
  writer.create(ch0_items[2], "current_mA", string_t(value_str));
  dtostrf(data.channels[0].power_mW, 4, 2, value_str);
  writer.create(ch0_items[3], "power_mW", string_t(value_str));
  writer.join(ch0_obj, 4, ch0_items[0], ch0_items[1], ch0_items[2], ch0_items[3]);

  // Kanal 1
  dtostrf(data.channels[1].bus_voltage, 4, 2, value_str);
  writer.create(ch1_items[0], "bus_voltage_V", string_t(value_str));
  dtostrf(data.channels[1].shunt_voltage_mV, 4, 2, value_str);
  writer.create(ch1_items[1], "shunt_voltage_mV", string_t(value_str));
  dtostrf(data.channels[1].current_mA, 4, 2, value_str);
  writer.create(ch1_items[2], "current_mA", string_t(value_str));
  dtostrf(data.channels[1].power_mW, 4, 2, value_str);
  writer.create(ch1_items[3], "power_mW", string_t(value_str));
  writer.join(ch1_obj, 4, ch1_items[0], ch1_items[1], ch1_items[2], ch1_items[3]);

  // Kanal 2
  dtostrf(data.channels[2].bus_voltage, 4, 2, value_str);
  writer.create(ch2_items[0], "bus_voltage_V", string_t(value_str));
  dtostrf(data.channels[2].shunt_voltage_mV, 4, 2, value_str);
  writer.create(ch2_items[1], "shunt_voltage_mV", string_t(value_str));
  dtostrf(data.channels[2].current_mA, 4, 2, value_str);
  writer.create(ch2_items[2], "current_mA", string_t(value_str));
  dtostrf(data.channels[2].power_mW, 4, 2, value_str);
  writer.create(ch2_items[3], "power_mW", string_t(value_str));
  writer.join(ch2_obj, 4, ch2_items[0], ch2_items[1], ch2_items[2], ch2_items[3]);

  // Pripravimo še ostale objekte na najvišjem nivoju
  object_t top_level_items[6];
  writer.create(top_level_items[0], "ch0", ch0_obj);
  writer.create(top_level_items[1], "ch1", ch1_obj);
  writer.create(top_level_items[2], "ch2", ch2_obj);
  
  snprintf(value_str, sizeof(value_str), "%u", data.alert_flags);
  writer.create(top_level_items[3], "alert_flags", string_t(value_str));
  
  dtostrf(data.shunt_voltage_sum_mV, 4, 2, value_str);
  writer.create(top_level_items[4], "shunt_voltage_sum_mV", string_t(value_str));
  
  snprintf(value_str, sizeof(value_str), "%lu", timestamp);
  writer.create(top_level_items[5], "timestamp", string_t(value_str));

  // Združimo vse v končni JSON
  writer.join(final_json, 6, top_level_items[0], top_level_items[1], top_level_items[2], top_level_items[3], top_level_items[4], top_level_items[5]);

  // Sestavimo pot
  char path_buffer[96];
  snprintf(path_buffer, sizeof(path_buffer), "%s/%lu", inaPath, timestamp);

  Firebase.printf("[F_UPDATE_INA] Sending INA data (poskus %d/%d)...\n",
                  firebaseRetryCount + 1, MAX_FIREBASE_RETRIES);

  // NOVO: Shrani operacijo za retry
  lastOperation.type = LastFirebaseOperation::Type::UPDATE_INA;
  lastOperation.data.ina.timestamp = timestamp;
  // lastOperation.data.ina.data = data;        // POPRAVIT KO BO LORA INSTALIRANA
  lastOperation.waiting_for_response = true;
  lastFirebaseOperationTime = millis();
  firebase_response_received = false;

  if (!Firebase_IsReady())
  {
    Firebase.printf("[F_UPDATE_INA] Firebase ni pripravljen za posodobitev INA podatkov!\n");
    return;
  }

  // Pošljemo podatke
  Database.set<object_t>(streamClient1, path_buffer, final_json, Firebase_processResponse, "updateINA3221Task");

}


//------------------------------------------------------------------------------------------------------------------------
// Funkcija za preverjanje in ponovni poskus Firebase operacij
void Firebase_CheckAndRetry()
{
  // 1. Če ni aktivne operacije, nič ne počnemo
  if (!lastOperation.waiting_for_response)
  {
    return;
  }

  // 2. Če smo prejeli odgovor, resetiramo
  if (firebase_response_received)
  {
    lastOperation.waiting_for_response = false;
    firebaseRetryCount = 0;
    Firebase.printf("[FB_RETRY] ✅ Odgovor prejet, reset retry števca.\n");
    return;
  }

  // 3. Preveri timeout
  if (millis() - lastFirebaseOperationTime < FIREBASE_RESPONSE_TIMEOUT)
  {
    return; // Še čakamo
  }

  // 4. Timeout!
  firebaseRetryCount++;

  Firebase.printf("[FB_RETRY] ⚠️ TIMEOUT! Poskus %d/%d\n",
                  firebaseRetryCount, MAX_FIREBASE_RETRIES);


  // 5. Preveri, ali je Firebase še vedno pripravljen
  if (!Firebase_IsReady())
  {
    Firebase.printf("[FB_RETRY] Firebase ni pripravljen. Preskakujem retry.\n");
    
    // Signaliziraj neuspeh
    if (lastOperation.type == LastFirebaseOperation::Type::UPDATE_SENSOR ||
        lastOperation.type == LastFirebaseOperation::Type::UPDATE_INA)
    {
      // Sensor_OnFirebaseResponse(false);
    }
    
    lastOperation.waiting_for_response = false;
    firebaseRetryCount = 0;
    return;
  }                

  // 6.Če smo dosegli max retry, signaliziraj neuspeh
  if (firebaseRetryCount >= MAX_FIREBASE_RETRIES)
  {
    Firebase.printf("[FB_RETRY] ❌ Maksimalno število poskusov doseženo. NEUSPEH!\n");

    // Signaliziraj neuspeh glede na tip operacije
    if (lastOperation.type == LastFirebaseOperation::Type::UPDATE_SENSOR ||
        lastOperation.type == LastFirebaseOperation::Type::UPDATE_INA)
    {
      // Sensor_OnFirebaseResponse(false);
    }

    // Reset
    lastOperation.waiting_for_response = false;
    firebaseRetryCount = 0;
    return;
  }

  // 7. Ponovni poskus - pokliči ustrezno funkcijo
  Firebase.printf("[FB_RETRY] 🔄 Ponovni poskus...\n");

  switch (lastOperation.type)
  {
  // case LastFirebaseOperation::Type::UPDATE_SENSOR:                     // POPRAVIT KO BO LORA INSTALIRANA
  //   Firebase_Update_Sensor_Data(lastOperation.data.sensor.timestamp,
  //                               lastOperation.data.sensor.data);
  //   break;

  // case LastFirebaseOperation::Type::UPDATE_INA:
  //   Firebase_Update_INA_Data(lastOperation.data.ina.timestamp,
  //                            lastOperation.data.ina.data);
  //   break;

  case LastFirebaseOperation::Type::UPDATE_RELAY:
    Firebase_Update_Relay_State(lastOperation.data.relay.kanal,
                                lastOperation.data.relay.state);
    break;

  case LastFirebaseOperation::Type::GET_URNIK:
    Firebase_readKanalUrnik(lastOperation.data.urnik.kanalIndex);
    break;

  case LastFirebaseOperation::Type::GET_INTERVAL:
    Firebase_readInterval();
    break;

  default:
    Firebase.printf("[FB_RETRY] ⚠️ Neznani tip operacije!\n");
    lastOperation.waiting_for_response = false;
    firebaseRetryCount = 0;
    break;
  }
}

//----------------------------------------------------------------------------------------------------------
// Firebase loop function
void Firebase_loop()
{
  app.loop();

  if (app.ready())
  {
    // Testirajte re-avtentikacijo po 2 minutah
    static unsigned long forceReauthAt = millis() + 120000;
    if (millis() > forceReauthAt)
    {
      Firebase.printf("🔄 FORCING RE-AUTH [Heap before: %d]\n", ESP.getFreeHeap());
      app.authenticate();                // Force library to re-authenticate (refresh the auth token).
      forceReauthAt = millis() + 120000; // Naslednji test čez 2 min
    }

    if (!extract_uid) // samo enkrat ob prvem ready
    {
      // Pridobimo UID
      strncpy(uid, app.getUid().c_str(), sizeof(uid) - 1);
      uid[sizeof(uid) - 1] = '\0'; // Zagotovimo null-terminacijo
      Firebase.printf("User UID: %s\n", uid);
      snprintf(databasePath, sizeof(databasePath), "/UserData/%s", uid);
      snprintf(examplesPath, sizeof(examplesPath), "%s/examples", databasePath);

      snprintf(examplePath1, sizeof(examplePath1), "%s/1", examplesPath);
      snprintf(examplePath2, sizeof(examplePath2), "%s/2", examplesPath);

      // In SSE mode (HTTP Streaming) task, you can filter the Stream events by using AsyncClientClass::setSSEFilters(<keywords>),
      // which the <keywords> is the comma separated events.
      // The event keywords supported are:
      // get - To allow the http get response (first put event since stream connected).
      // put - To allow the put event.
      // patch - To allow the patch event.
      // keep-alive - To allow the keep-alive event.
      // cancel - To allow the cancel event.
      // auth_revoked - To allow the auth_revoked event.
      // To clear all prevousely set filter to allow all Stream events, use AsyncClientClass::setSSEFilters().
      streamClient1.setSSEFilters("get,put,patch,keep-alive,cancel,auth_revoked");

      // VZPOSTAVITE STREAM ŠELE TUKAJ, ko imate pravilne poti
      Database.get(streamClient1, examplesPath, streamCallback, true, "streamTask");
      // Database.get(streamClient2, examplePath2, processData, true, "streamTask2");
      Firebase.printf("Free Heap: %d\n", ESP.getFreeHeap());
      extract_uid = true;
    }

    //---- Periodične naloge----
    // Tukaj lahko izvajamo periodične naloge
    static unsigned long lastFirebaseCheck = 0;
    if (millis() - lastFirebaseCheck > 200) // 5x na sekundo
    {
      lastFirebaseCheck = millis();
      // 1. Preveri stanje Firebase povezave
      Firebase_Check_Active_State(true);

      // 2. Preveri timeouts in retry
      Firebase_CheckAndRetry();

      // 3. Preveri, ali so na voljo novi podatki iz Firebase streama
      if (newChannelDataAvailable)
      {
        // Tukaj pokličite funkcijo za pošiljanje podatkov Rele modulu
        if (Firebase_handleStreamUpdate(channelUpdate.kanalIndex, channelUpdate.start_sec, channelUpdate.end_sec))
        {
          firebaseUpdatePending = true; // čakamo na prosto LoRa
        }
        // Počisti zastavico, da ne obdelamo istih podatkov večkrat
        newChannelDataAvailable = false;
      }
    }
    // --- konec periodičnih nalog ---


    if (millis() - ms > 20000)
    {
      ms = millis();

      JsonWriter writer;

      object_t json, obj1, obj2;

      writer.create(obj1, "ms", ms);
      writer.create(obj2, "rand", random(10000, 30000));
      writer.join(json, 2, obj1, obj2);

      Database.set<object_t>(aClient, examplePath1, json, Firebase_processResponse, "setTask1");

      Database.set<int>(aClient, examplePath2, random(100000, 200000), Firebase_processResponse, "setTask2");
    }


  }
}

// Procesiranje podatkov iz Firebase (callback)
/* void processData(AsyncResult &aResult)
{
  uint32_t heap = ESP.getFreeHeap();

  if (heap < minHeapDuringAuth)
  {
    Firebase.printf("Minimum Free Heap: %d\n", heap);
    minHeapDuringAuth = heap;
  }

  if (!aResult.isResult())
    return;

  if (aResult.isEvent())
  {
    Firebase.printf("Event task: %s, msg: %s, code: %d [Heap: %d]\n",
                    aResult.uid().c_str(),
                    aResult.eventLog().message().c_str(),
                    aResult.eventLog().code(),
                    heap);
  }

  if (aResult.isDebug())
  {
    Firebase.printf("Debug task: %s, msg: %s [Heap: %d]\n",
                    aResult.uid().c_str(),
                    aResult.debug().c_str(),
                    heap);
  }

  if (aResult.isError())
  {
    Firebase.printf("❌ Error task: %s, msg: %s, code: %d [Heap: %d, Min: %d]\n",
                    aResult.uid().c_str(),
                    aResult.error().message().c_str(),
                    aResult.error().code(),
                    heap, minHeapDuringAuth);
  }

  if (aResult.available())
  {
    RealtimeDatabaseResult &stream = aResult.to<RealtimeDatabaseResult>();
    if (stream.isStream())
    {
      Serial.println("----------------------------");
      Firebase.printf("task: %s\n", aResult.uid().c_str());
      Firebase.printf("event: %s\n", stream.event().c_str());
      Firebase.printf("path: %s\n", stream.dataPath().c_str());
      Firebase.printf("data: %s\n", stream.to<const char *>());
      Firebase.printf("type: %d\n", stream.type());

      // The stream event from RealtimeDatabaseResult can be converted to the values as following.
      // bool v1 = stream.to<bool>();
      // int v2 = stream.to<int>();
      // float v3 = stream.to<float>();
      // double v4 = stream.to<double>();
      // String v5 = stream.to<String>();
    }
    else
    {
      Serial.println("----------------------------");
      Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
    }
#if defined(ESP32) || defined(ESP8266)
    Firebase.printf("Free Heap: %d\n", ESP.getFreeHeap());
#elif defined(ARDUINO_RASPBERRY_PI_PICO_W)
    Firebase.printf("Free Heap: %d\n", rp2040.getFreeHeap());
#endif
  }
} */

void Firebase_processResponse(AsyncResult &aResult)
{
  Serial.println("************************************");
  Firebase.printf("[F_HEAP] Free Heap pred RESPONSE: %d\n", ESP.getFreeHeap());
  // Preveri in izpiši stanje SSL povezave
  
  Firebase_Check_Active_State(false);

  // SAMO EN KLIC available() NA ZAČETKU!
  bool hasResult = aResult.isResult();
  bool hasError = aResult.isError();
  bool isEvent = aResult.isEvent();
  bool isDebug = aResult.isDebug();
  bool hasData = aResult.available();

  static uint32_t call_count = 0;
  Serial.printf("[F_RESPONSE] KLIC #%u, isResult=%d, isError=%d, isEvent=%d, isDebug=%d, available=%d\n",
                ++call_count, hasResult, hasError, isEvent, isDebug, hasData);

  if (!hasResult)
  {
    return;
  }

  // ----------------------------------------------------------------------------------------
  // 1. NAJPREJ napake
  if (hasError)
  {
    Firebase.printf("[F_RESPONSE] Error task: %s, msg: %s, code: %d\n",
                    aResult.uid().c_str(),
                    aResult.error().message().c_str(),
                    aResult.error().code());

    // // NOVO: TCP connection failed -> označimo za reconnect
    // if (aResult.error().code() == -1 || aResult.error().code() == 401)
    // {
    //   Firebase.printf("[F_RESPONSE] ⚠️ TCP povezava prekinjena! Načrtujem reconnect...\n");
    //   FirebaseNeedsReconnect = true;
    // }

    firebase_response_received = false;
    return; // error sporočila nimajo payload-a
  }

  // ----------------------------------------------------------------------------------------
  // 2. DOGODKI
  if (isEvent)
  {
    Firebase.printf("[F_RESPONSE] Event task: %s, msg: %s, code: %d\n",
                    aResult.uid().c_str(),
                    aResult.appEvent().message().c_str(),
                    aResult.appEvent().code());

    // Preverimo za avtentikacijo
    if (aResult.uid().equals("🔐 authTask"))
    {
      if (aResult.appEvent().code() == 7) // začetek avtentikacije
      {
        ssl_avtentikacija = false; // Avtentikacija v teku (traja cca 3 - 5 sekund)
        Firebase.printf("[F_READY] Avtentikacija v teku...\n");

      }
      if (aResult.appEvent().code() == 10) // začetek avtentikacije
      {
        // Firebase.printf("[F_AUTH] ✅ Avtentikacija uspešna!\n");
        ssl_avtentikacija = true;
        Firebase_Check_Active_State(false);
      }
    }
    //return;  // Eventi imajo tudi debug
  }

  // ----------------------------------------------------------------------------------------
  // 3. DEBUG
  if (isDebug)
  {
    // pripišemo trenutni čas
    // printLocalTime();
    Firebase.printf("[F_RESPONSE] Debug task: %s, msg: %s\n",
                    aResult.uid().c_str(),
                    aResult.debug().c_str());

    //return; // Debug sporočila imajo tudi payload-a

  }

  // ----------------------------------------------------------------------------------------
  // 4. CHECK

  if (!hasData) {
    // Serial.println("[F_RESPONSE] ⚠️ No available result!");
    return;
  }

  // DIREKTNE PRIMERJAVE - brez vmesnih spremenljivk:
  if (aResult.path().length() == 0) {
    Serial.println("[F_RESPONSE] ⚠️ Path is empty!");
    return;
  }

  // Debugging - uporabi String objekte
  Serial.printf("[F_RESPONSE] Task: %s\n", aResult.uid().c_str());
  // Serial.printf("[F_RESPONSE] Path: %s\n", aResult.path().c_str());

  // Payload shranimo v static buffer (kot prej)
  static char payloadBuf[512];
  strncpy(payloadBuf, aResult.c_str(), sizeof(payloadBuf) - 1);
  payloadBuf[sizeof(payloadBuf) - 1] = '\0';

  // Serial.printf("[F_RESPONSE] RAW Payload: [%s]\n", payloadBuf);
  Serial.printf("[F_RESPONSE] Payload length: %d\n", strlen(payloadBuf));

  // lastFirebaseActivityTime = millis();

  // PRIMERJAVE: Uporabi .equals() namesto strcmp():
  
  //-----------------------------------------------------------------------------------------
  // Preberi urnik kanala
  if (aResult.uid().equals("getUrnikTask"))  // ✅ SAFE primerjava
  {
    const char* path = aResult.path().c_str();  // Uporabi takoj, ne shrani
    
    int kanalIndex = -1;
    const char *kanalStr = strstr(path, "kanal");
    
    if (kanalStr != NULL) {
      kanalIndex = atoi(kanalStr + 5) - 1;
    }

    Firebase.printf("[F_RESPONSE] Prejet urnik za kanal: %d (pričakovan: %d)\n", 
                    kanalIndex, currentChannelInProcess);

    if (kanalIndex != -1 && kanalIndex == currentChannelInProcess)
    {
      int start_sec = extractIntValue(payloadBuf, "\"start_sec\"");
      int end_sec = extractIntValue(payloadBuf, "\"end_sec\"");

      if (start_sec != -1 && end_sec != -1)
      {
        firebase_kanal[kanalIndex].start_sec = start_sec;
        firebase_kanal[kanalIndex].end_sec = end_sec;
        formatSecondsToTime(firebase_kanal[kanalIndex].start, 
                          sizeof(firebase_kanal[kanalIndex].start), start_sec);
        formatSecondsToTime(firebase_kanal[kanalIndex].end, 
                          sizeof(firebase_kanal[kanalIndex].end), end_sec);

        Firebase.printf("[F_RESPONSE] ✅ Urnik prebran: %d - %d\n", 
                        start_sec, end_sec);
        
        firebase_response_received = true;
        firebaseRetryCount = 0;
        lastOperation.waiting_for_response = false;
      }
      else
      {
        Firebase.printf("[F_RESPONSE] ⚠️ Parsing failed: start=%d, end=%d\n", 
                        start_sec, end_sec);
      }
    }
  }

  //-----------------------------------------------------------------------------------------
  // Preberi interval
  else if (aResult.uid().equals("getChartIntervalTask"))  // ✅
  {
    uint8_t interval = atoi(payloadBuf);
    if (interval > 0) {
      set_Interval(interval);
      Firebase.printf("[F_RESPONSE] Interval nastavljen: %d minut ✅\n", interval);
      firebase_response_received = true;
      firebaseRetryCount = 0;
      lastOperation.waiting_for_response = false;
    }
  }

  //-----------------------------------------------------------------------------------------
  // Senzorji
  else if (aResult.uid().equals("updateSensorTask"))  // ✅
  {
    Firebase.printf("[F_RESPONSE] Sensor data uploaded ✅\n");
    firebase_response_received = true;
    firebaseRetryCount = 0;
    lastOperation.waiting_for_response = false;
    // Sensor_OnFirebaseResponse(true);

  }

  //-----------------------------------------------------------------------------------------
  // INA3221
  else if (aResult.uid().equals("updateINA3221Task"))  // ✅
  {
    Firebase.printf("[F_RESPONSE] INA3221 data uploaded ✅\n");
    firebase_response_received = true;
    firebaseRetryCount = 0;
    lastOperation.waiting_for_response = false;
    // Sensor_OnFirebaseResponse(true);
    // Firebase_CloseSSL();

  }

  //-----------------------------------------------------------------------------------------
  // Relay state
  else if (aResult.uid().equals("updateStateTask"))  // ✅
  {
    Firebase.printf("[F_RESPONSE] State data uploaded ✅\n");
    firebase_response_received = true;
    firebaseRetryCount = 0;
    lastOperation.waiting_for_response = false;

  //-------------------------------------------------------------------------------------
  // Dummy task
  }
  else if (aResult.uid().equals("dummyTask"))
  {
    Firebase.printf("[F_RESPONSE] Dummy task executed ✅\n");
    dummyOperationSent = false;
  }

  //------------------------------------------------------------------------------------
  else
  {
    Serial.println("-------------Neznan task---------------");
    Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
  }

  Firebase.printf("[F_RESPONSE] Free Heap po RESPONSE: %d\n", ESP.getFreeHeap());
}