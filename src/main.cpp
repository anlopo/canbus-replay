#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>

#define SHOW_PPS

// Odkomentuj nasledujúci riadok, ak chceš povoliť debug správy (funkcia wait_for() nebude fungovať správne!)
// #define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

#define SD_CS_PIN 9   // Pin pre SD kartu
#define CAN_CS_PIN 10 // Pin pre CAN modul

void canInit();
bool read_line(char *, size_t);
void send_line(const char *);
void wait_for(double);
unsigned long start_time, oldSendTime = 0;
#ifdef SHOW_PPS
unsigned long pps_last;
unsigned int pps = 0;
#endif

File logFile;
MCP_CAN CAN0(CAN_CS_PIN);

double lastTimestamp = 0.0;
bool canInitialized = false;

void setup()
{
  Serial.begin(115200);

  SD.begin(SD_CS_PIN);
  delay(10);
  logFile = SD.open("logfile.asc");
  if (!logFile)
  {
    Serial.println(F("Error opening file"));
    while (1)
      ;
  }

  canInit();

  Serial.println(F("Starting log replay..."));

  start_time = millis();
}

void loop()
{
  if (!canInitialized)
  {
    delay(2000);
    Serial.println(F("Retrying initialize CAN-BUS"));
    canInit();
    return;
  }

  if (!logFile.available())
  {
    Serial.print(F("Log replay finished after "));
    Serial.print((millis() - start_time) / 1000);
    Serial.println("s");
    logFile.close();
    while (1)
      ;
  }

  char lineBuffer[128]; // Maximálna dĺžka riadku
  if (read_line(lineBuffer, sizeof(lineBuffer)))
  {
    send_line(lineBuffer);
  }

#ifdef SHOW_PPS
  pps++;
  if ((pps_last + 1000) < millis())
  {
    pps_last = millis();
    Serial.print(pps);
    Serial.println(F("pps"));
    pps = 0;
  }
#endif
}

void canInit()
{
  if (CAN_OK == CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ))
  {
    canInitialized = true;
    Serial.println(F("CAN BUS Shield init ok!"));
    CAN0.setMode(MCP_NORMAL);
    CAN0.enOneShotTX();
  }
  else
  {
    Serial.println(F("ERROR!!!! CAN-BUS Shield init fail"));
    Serial.println(F("ERROR!!!! Will try to init CAN-BUS shield again"));
  }
}

bool read_line(char *buffer, size_t maxLen)
{
  size_t index = 0;
  bool hasStarted = false;

  while (logFile.available())
  {
    char c = logFile.read();

    if (c == '\r')
      continue; // Ignorovanie CR
    if (c == '\n')
      break; // Koniec riadku

    if (!hasStarted && (c == ' ' || c == '\t'))
      continue;

    hasStarted = true;
    if (index < maxLen - 1)
    {
      buffer[index++] = c;
    }
  }

  buffer[index] = '\0'; // Null-terminate string

  if (index == 0)
    return false; // Prázdny riadok

  DEBUG_PRINT(F("read_line():"));
  DEBUG_PRINTLN(buffer);
  return true;
}

// Pošle CAN správu
void send_line(const char *line)
{
  if (strlen(line) < 5 || line[0] == '/')
    return; // Preskočí komentáre

  char str_timestamp[13];
  double timestamp;
  unsigned int canID;
  int dlc;
  uint8_t data[8] = {0};

  int matched = sscanf(line, "%s %*s %x %*s %*s %d %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                       str_timestamp, &canID, &dlc,
                       &data[0], &data[1], &data[2], &data[3],
                       &data[4], &data[5], &data[6], &data[7]);

  if (matched < 3 || dlc < 1 || dlc > 8)
  {
    Serial.print(F("Chyba pri parsovaní riadku! matched="));
    Serial.println(matched);
    return;
  }

  timestamp = strtod(str_timestamp, NULL);

  unsigned long elapsed = micros() - oldSendTime;
  double remainingTime = (timestamp - lastTimestamp) - (elapsed * 1e-6);
  if (remainingTime > 0)
  {
    wait_for(remainingTime);
  }
  lastTimestamp = timestamp;
  oldSendTime = micros();

  CAN0.sendMsgBuf(canID, 0, dlc, data);

  DEBUG_PRINT(F("Odoslané: "));
  DEBUG_PRINT(canID, HEX);
  DEBUG_PRINT(F("\t0\t"));
  DEBUG_PRINT(dlc);
  DEBUG_PRINT(F("\t"));
#ifdef DEBUF
  for (int i = 0; i < dlc; i++)
  {
    Serial.print(data[i], HEX);
    Serial.print(F(" "));
  }
#endif
  DEBUG_PRINTLN();
}

// Čakanie na daný čas (v sekundách)
void wait_for(double delayTime)
{
  DEBUG_PRINT(F("wait_for() -> "));
  DEBUG_PRINTLN(delayTime, 5);
  if (delayTime > 0)
  {
    unsigned long us = static_cast<unsigned long>(delayTime * 1e6);
    unsigned long start = micros();
    while (micros() - start < us)
      ;
  }
}