
#include "utilities.h"
#include "time.h"

const char *ntpServer = "pool.ntp.org";

// ----------------------------------------------------------------------------
// Definition of the LED component
// ----------------------------------------------------------------------------
struct Led
{
  // state variables
  uint8_t pin; // pin number for the LED
  bool on;     // logical state of the LED

  // methods
  void update() // method for updating the physical state of the LED
  {
    digitalWrite(pin, on ? HIGH : LOW);
  }

  void toggle() // method for toggling the logical state of the LED
  {
    on = !on;
    update();
  }
  // method for blinking the LED for a given duration on and off
  void blink(unsigned long onInterval, unsigned long offInterval)
  {
    on = millis() % (onInterval + offInterval) < onInterval;
    update();
  }
};

// Led    led = {LED_PIN, false}; // LED na protoboardu
Led onboard_led = {BOARD_LED, false}; // onboard LED

//-----------------------------------------------------------------------------------------------------
// Funkcija za inicializacijo LED
void init_LED()
{
  pinMode(onboard_led.pin, OUTPUT); // onboard LED is output
  onboard_led.on = false;           // turn off the LED
  onboard_led.update();             // update the LED state
}

//-----------------------------------------------------------------------------------------------------
// Funkcija za sinhronizacijo časa z NTP strežnikom 
void syncTimestamp()
{
  // displayLogOnLine(3, "Sync cas...");
  
  // Definiramo TZ niz za Srednjeevropski čas
  const char* tz_info = "CET-1CEST,M3.5.0,M10.5.0/3";

  // Uporabimo configTzTime, ki hkrati nastavi časovni pas in zažene NTP klienta.
  // To je pravilna metoda za ESP32.
  configTzTime(tz_info, ntpServer);

  // Počakamo na dejansko sinhronizacijo časa.
  Serial.print("Cakam na NTP sinhronizacijo... ");
  struct tm timeinfo;
  int retry = 0;
  const int retry_max = 15; // Počakamo največ 15 sekund

  // Poskušamo dobiti čas. Če leto ni večje od 2022, čas še ni nastavljen.
  while (!getLocalTime(&timeinfo) || timeinfo.tm_year < (2022 - 1900)) {
    if(retry++ > retry_max) {
      Serial.println("\nNTP sinhronizacija neuspesna!");
      // displayLogOnLine(3, "NTP fail");
      return;
    }
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nNTP sinhronizacija uspesna!");
  // displayLogOnLine(3, "NTP OK");
  
  // Sedaj, ko je čas zagotovo nastavljen, ga lahko izpišemo.
  printLocalTime();
  uint32_t timestamp = getTime(); // Pridobi trenutni čas v UTC
  Serial.println("Trenutni čas: " + String(timestamp)); // izpiši trenutni čas
}

//---------------------------------------------------------------------------------------------------------------------
// Funkcija za pridobivanje trenutnega časa v sekundah od 1.1.1970 (prej v main.cpp)
uint32_t getTime()
{
  time_t now;
  time(&now); // Preprosto preberi sistemsko uro (ki je v UTC)
  
  // Preverimo, ali je ura sploh že bila nastavljena (čas po letu 2023)
  if (now < 1672531200) { 
    return 0;
  }
  return now;
}

//---------------------------------------------------------------------------------------------------------------------
// Funkcija za izpis lokalnega časa (prej v main.cpp)
void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

//---------------------------------------------------------------------------------------------------------------------
// Funkcija za pridobivanje sekund od polnoči (prej v main.cpp)
uint32_t getCurrentSeconds()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return 0;
  }
  return timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
}

//-----------------------------------------------------------------------------------------------------
// Funkcija za formatiranje časa iz sekund v HH:MM
String formatTime(uint32_t totalSeconds)
{
  uint8_t hours = (totalSeconds / 3600) % 24;   // Izračunaj ure
  uint8_t minutes = (totalSeconds % 3600) / 60; // Izračunaj minute

  char timeString[6];                                                    // HH:MM + null terminator
  snprintf(timeString, sizeof(timeString), "%02d:%02d", hours, minutes); // Oblikuj čas v HH:MM

  return String(timeString); // Vrni oblikovan čas kot String
}

//-----------------------------------------------------------------------------------------------------
// Funkcija za utripanje LED
void Blink_led(uint8_t status)
{
  if (status == 3)
  {
    onboard_led.blink(100, 100); // LED blinka vsakih 100 ms
  }
  else if (status == 1 || status == 2)
  {
    onboard_led.blink(100, 900); // LED blinka vsakih 1000 ms
  }
  else
  {
    onboard_led.blink(500, 500); // LED blinka vsakih 500 ms
  }
}

// ----------------------------------------------------------------------------
// Funkcija za pretvorbo časa v int obliki sekunde od začetka dneva v obliko "HH:MM"
// --- SPREMENJENA, BOLJ UČINKOVITA VERZIJA ---
void formatSecondsToTime(char *buffer, size_t bufferSize, int seconds)
{
  int hours = seconds / 3600;
  int minutes = (seconds % 3600) / 60;
  snprintf(buffer, bufferSize, "%02d:%02d", hours, minutes);
}

// ----------------------------------------------------------------------------
// Funkcija za pretvorbo minut intervala v milisekunde
void set_Interval(uint8_t minutes)
{
  Interval_mS = minutes * 60UL * 1000UL; // Pretvorba minut v milisekunde
}

// ----------------------------------------------------------------------------
// Funkcija za pretvorbo minut intervala v milisekunde
unsigned long get_Interval()
{
  if (Interval_mS == 0)
  {
    return DEFAULT_SENSOR_READ_INTERVAL_MINUTES * 60UL * 1000UL;
  }
  return Interval_mS;
}