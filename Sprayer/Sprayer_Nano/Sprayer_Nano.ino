/* Sprayer_Nano.ino - 5-section sprayer control for AgOpenWeb / AgOpenGPS
   Version 0.2 (2026-10-22), not yet tested on the sprayer.

   Hardware:
     Arduino Nano + ENC28J60 Ethernet shield (CS on D10)
     Blue 8-relay board with 5 V relays (SRD-05VDC), switches on LOW
       (for a DEWIN 12 V board with its jumpers on "High", set RELAY_ON to HIGH)
     5 normally closed solenoid valves, 0-10 bar pressure sensor (0.5-4.5 V), YF-S201 flow meter

   Network: the Nano is the standard "machine module", IP 192.168.5.123.
     It listens on UDP port 8888 for messages from the program (PGN 239 = section states).
     It sends to port 9999 (broadcast): the reply to "hello" and "scan", and the
     sprayer readings (pressure, flow, litres) in a PGN of its own.

   Pins:
     D2       flow meter (yellow wire)
     D3..D7   relay board IN1..IN5 (sections 1..5)
     A0       pressure sensor (green wire)
     A6       master switch (optional, 10k / 4k7 divider from the 12 V after the switch)
     D10..D13 Ethernet shield

   Library: EtherCard (Arduino IDE Library Manager: search for "EtherCard").
*/

#include <EtherCard.h>

// ---------------- Settings ----------------
#define NUM_SECTIONS         5
const uint8_t relayPin[NUM_SECTIONS] = { 3, 4, 5, 6, 7 };
#define RELAY_ON             LOW     // blue 5 V board: switches on LOW. DEWIN board on "High": HIGH
#define RELAY_OFF            (!RELAY_ON)

#define FLOW_PIN             2
#define PRESSURE_PIN         A0
#define MASTER_PIN           A6
#define ETHERNET_CS_PIN      10

#define PULSES_PER_LITRE     450.0   // YF-S201: about 450. Calibrate with a bucket (see the plan)
#define PRESSURE_MAX_BAR     10.0    // 0-10 bar sensor
#define USE_MASTER_SWITCH    0       // 1 = A6 reads the master switch (needs the divider). 0 = not read
#define MASTER_THRESHOLD     300     // A6 reading above this = master switch on (about 1.5 V)
#define NO_DATA_TIMEOUT_MS   2000    // no messages from the program for 2 s: close all sections

// Network (the same as AgIO / AgOpenWeb: 192.168.5.x)
uint8_t myIP[]      = { 192, 168, 5, 123 };
uint8_t gatewayIP[] = { 192, 168, 5, 1 };
uint8_t dnsIP[]     = { 8, 8, 8, 8 };
uint8_t netmask[]   = { 255, 255, 255, 0 };
uint8_t destIP[]    = { 192, 168, 5, 255 };   // broadcast
uint8_t myMAC[]     = { 0x00, 0x00, 0x56, 0x00, 0x00, 0x7B };
#define LISTEN_PORT          8888
#define SEND_PORT            9999
#define SOURCE_PORT          5123

// Our own PGN for pressure / flow. AgOpenGPS and AgOpenWeb ignore PGNs they don't know.
#define SPRAYER_PGN          0xA0

uint8_t Ethernet::buffer[200];

// ---------------- State ----------------
uint8_t sectionsRequested = 0;       // bits from the program (bit 0 = section 1)
uint8_t sectionsOn = 0;              // what the relays are actually doing
uint32_t lastMessageMs = 0;
bool masterOn = false;

volatile uint16_t pulses = 0;
float flowLpm = 0;
float totalLitres = 0;
float pressureBar = 0;
uint32_t lastSecondMs = 0;

void countPulse() { pulses++; }

// AgOpenGPS checksum: sum of the bytes from the 3rd up to the second to last
uint8_t checksum(const uint8_t *msg, uint8_t length)
{
    uint8_t sum = 0;
    for (uint8_t i = 2; i < length - 1; i++) sum += msg[i];
    return sum;
}

void send(uint8_t *msg, uint8_t length)
{
    msg[length - 1] = checksum(msg, length);
    ether.sendUdp((char *)msg, length, SOURCE_PORT, destIP, SEND_PORT);
}

// Called every time a message arrives on port 8888
void udpReceived(uint16_t destPort, uint8_t srcIP[IP_LEN], uint16_t srcPort, const char *data, uint16_t length)
{
    const uint8_t *d = (const uint8_t *)data;
    if (length < 6 || d[0] != 0x80 || d[1] != 0x81) return;   // not from AgOpenGPS / AgOpenWeb
    uint8_t pgn = d[3];

    if (pgn == 239 && length >= 14) {           // Machine Data: sections 1..8 in byte 11
        sectionsRequested = d[11];
        lastMessageMs = millis();
    }
    else if (pgn == 200) {                      // "hello" from AgIO: answer as a machine module
        uint8_t hello[] = { 0x80, 0x81, 123, 123, 5, sectionsOn, 0, 0, 0, 0, 0 };
        send(hello, sizeof(hello));
    }
    else if (pgn == 202) {                      // network "scan": tell them our IP
        uint8_t scan[] = { 0x80, 0x81, 123, 203, 7,
                           myIP[0], myIP[1], myIP[2], myIP[3],
                           srcIP[0], srcIP[1], srcIP[2], 0 };
        send(scan, sizeof(scan));
    }
}

void updateRelays()
{
    // No messages for too long (loose cable, frozen tablet) or master switch off: close everything
    bool linkOK = (millis() - lastMessageMs) < NO_DATA_TIMEOUT_MS;
    sectionsOn = (linkOK && masterOn) ? sectionsRequested : 0;

    for (uint8_t i = 0; i < NUM_SECTIONS; i++)
        digitalWrite(relayPin[i], bitRead(sectionsOn, i) ? RELAY_ON : RELAY_OFF);
}

void readSensors()
{
    // Pressure: smoothed so it doesn't jump around. 0.5 V = 0 bar, 4.5 V = full scale
    float volts = analogRead(PRESSURE_PIN) * 5.0 / 1023.0;
    float bar = (volts - 0.5) / 4.0 * PRESSURE_MAX_BAR;
    if (bar < 0) bar = 0;
    pressureBar = pressureBar * 0.9 + bar * 0.1;

#if USE_MASTER_SWITCH
    masterOn = analogRead(MASTER_PIN) > MASTER_THRESHOLD;
#else
    masterOn = true;    // the master switch cuts the valves' 12 V in the wiring
#endif
}

void updateFlow()
{
    noInterrupts();
    uint16_t n = pulses;
    pulses = 0;
    interrupts();

    float litres = n / PULSES_PER_LITRE;      // litres in this second
    flowLpm = litres * 60.0;
    totalLitres += litres;
}

void sendReadings()
{
    uint16_t p = pressureBar * 100;           // hundredths of a bar
    uint16_t f = flowLpm * 100;               // hundredths of a L/min
    uint32_t t = totalLitres * 10;            // tenths of a litre
    uint8_t msg[] = { 0x80, 0x81, 123, SPRAYER_PGN, 10,
                      lowByte(p), highByte(p),
                      lowByte(f), highByte(f),
                      (uint8_t)t, (uint8_t)(t >> 8), (uint8_t)(t >> 16), (uint8_t)(t >> 24),
                      masterOn, sectionsOn,
                      0 };
    send(msg, sizeof(msg));
}

void setup()
{
    // Relays off straight away at power-up (valves closed)
    for (uint8_t i = 0; i < NUM_SECTIONS; i++) {
        digitalWrite(relayPin[i], RELAY_OFF);
        pinMode(relayPin[i], OUTPUT);
    }

    Serial.begin(38400);
    Serial.println(F("Sprayer_Nano v0.2"));

    pinMode(FLOW_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), countPulse, FALLING);

    if (ether.begin(sizeof Ethernet::buffer, myMAC, ETHERNET_CS_PIN) == 0)
        Serial.println(F("ERROR: Ethernet shield not responding. Check the wiring / CS pin."));
    ether.staticSetup(myIP, gatewayIP, dnsIP, netmask);
    ether.udpServerListenOnPort(&udpReceived, LISTEN_PORT);
    Serial.println(F("Network ready: 192.168.5.123, listening on port 8888"));
}

void loop()
{
    ether.packetLoop(ether.packetReceive());   // handle any messages that arrived

    static uint32_t last50ms = 0;
    if (millis() - last50ms >= 50) {            // 20 times a second
        last50ms = millis();
        readSensors();
        updateRelays();
    }

    if (millis() - lastSecondMs >= 1000) {      // once a second
        lastSecondMs += 1000;
        updateFlow();
        sendReadings();

        Serial.print(F("Master ")); Serial.print(masterOn ? F("ON ") : F("OFF"));
        Serial.print(F(" | Sections ")); Serial.print(sectionsOn, BIN);
        Serial.print(F(" | ")); Serial.print(pressureBar, 1); Serial.print(F(" bar | "));
        Serial.print(flowLpm, 1); Serial.print(F(" L/min | "));
        Serial.print(totalLitres, 1); Serial.println(F(" L"));
    }
}
