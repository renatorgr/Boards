// v7_XIAO_ESP32C3_bridge_test.ino
// Test program for the Seeed XIAO ESP32C3 on the GPS2 socket of the AiO v4.5 Micro (adapter board
// or 3D-printed holder), with the Teensy running v7_AIO_UM982_X20P_F9P_TM171_WEB_XIAO (the official
// Teensy firmware v26 uses the same protocol).
//
// What it does:
//   - says "ESP32-hello" to the Teensy every 2 s (the Teensy then passes it the AgOpenGPS packets)
//   - reads those packets (AgOpenGPS -> UDP 8888 -> Teensy -> here), checks their checksum and
//     counts them by PGN; from PGN 254 it keeps the speed and the steer angle asked for
//   - every 5 s sends a test PGN to AgOpenGPS (PGN 140, source 0x50, 8 bytes: a counter). The Teensy
//     sends it to UDP 9999; AgOpenGPS ignores this PGN number. On the Teensy web page (Estado, card
//     XIAO) "PGN do XIAO" goes up and "Ultimo PGN do XIAO" shows 140.
//   - at start and then every 60 s writes one text line to the Teensy, shown on the Teensy web page
//     (Mensagens) as "[xiao] > test: ...".
//
// Arduino IDE: install the board package "esp32" by Espressif Systems (3.x). Board "XIAO_ESP32C3",
// Tools > USB CDC On Boot: Enabled. Upload over the XIAO's USB-C, OFF the Micro.
// NEVER plug the USB-C while the XIAO sits on the Micro: it is powered by the board's 3.3 V.
//
// Bench test without the Micro: XIAO on USB-C (power + serial monitor at any baud), and only three
// wires to the Teensy: D6 -> Teensy pin 7 (RX2), D7 -> Teensy pin 8 (TX2), GND -> GND. Do NOT wire
// 3V3 then. The serial monitor shows the same report every 10 s.

#if !ARDUINO_USB_CDC_ON_BOOT
#error "Arduino IDE: Tools > USB CDC On Boot > Enabled (D6/D7 are used for the Teensy)"
#endif

#define TEENSY_BAUD   460800
#define PIN_FROM_TEENSY D7      // GPIO20 <- Teensy TX2 (pin 8), GPS2 socket pin 3
#define PIN_TO_TEENSY   D6      // GPIO21 -> Teensy RX2 (pin 7), GPS2 socket pin 2
#define TEST_SOURCE   0x50
#define TEST_PGN      140       // not used by AgOpenGPS
#define HELLO_MS      2000
#define TEST_PGN_MS   5000
#define TEXT_MS       60000
#define USB_REPORT_MS 10000

HardwareSerial& Teensy = Serial1;    // UART1 on D7/D6 (UART0 stays with the boot messages)

// ---- AgOpenGPS frames from the Teensy: [0x80][0x81][src][pgn][len][data...][crc]
// crc = low 8 bits of the sum of the bytes from src to the last data byte
static uint8_t  fr[262];
static uint16_t frLen = 0;
static uint32_t frLastMs = 0;

static uint32_t rxGood = 0, rxBad = 0, rxBytes = 0;
static uint32_t perPgn[256];
static float    speedKmh = 0, steerSetDeg = 0;
static uint8_t  guidance = 0;
static bool     have254 = false;
static uint32_t testCount = 0;

void sendPgn(uint8_t src, uint8_t pgn, const uint8_t* data, uint8_t len)
{
    uint8_t b[262];
    b[0] = 0x80; b[1] = 0x81; b[2] = src; b[3] = pgn; b[4] = len;
    memcpy(b + 5, data, len);
    uint8_t sum = 0;
    for (int i = 2; i < 5 + len; i++) sum += b[i];
    b[5 + len] = sum;
    Teensy.write(b, 6 + len);
}

static void onPgn(const uint8_t* p)
{
    rxGood++;
    perPgn[p[3]]++;
    if (p[3] == 254 && p[4] >= 8)                   // from AgOpenGPS / AgIO: speed, guidance, steer angle
    {
        speedKmh    = (uint16_t)(p[5] | p[6] << 8) * 0.1f;
        guidance    = p[7];
        steerSetDeg = (int16_t)(p[8] | p[9] << 8) * 0.01f;
        have254     = true;
    }
}

// drop the first n bytes of fr, and anything up to the next 0x80
static void frShift(uint16_t n)
{
    while (n < frLen && fr[n] != 0x80) n++;
    if (n >= frLen) { frLen = 0; return; }
    memmove(fr, fr + n, frLen - n);
    frLen -= n;
}

static void frScan()
{
    while (frLen > 0)
    {
        if (fr[0] != 0x80 || (frLen >= 2 && fr[1] != 0x81)) { frShift(1); continue; }
        if (frLen < 5) return;
        const uint16_t need = 6 + fr[4];
        if (frLen < need) return;
        uint8_t sum = 0;
        for (uint16_t i = 2; i < need - 1; i++) sum += fr[i];
        if (sum == fr[need - 1]) { onPgn(fr); frShift(need); }
        else                     { rxBad++;   frShift(1); }
    }
}

static void readTeensy()
{
    while (Teensy.available() > 0)
    {
        const uint8_t b = (uint8_t)Teensy.read();
        rxBytes++;
        if (frLen == 0 && b != 0x80) continue;      // the Teensy only sends frames
        fr[frLen++] = b;
        frLastMs = millis();
        frScan();
    }
    if (frLen > 0 && millis() - frLastMs > 50) { frShift(1); frScan(); frLastMs = millis(); }
}

static void report(char* line, size_t size)
{
    snprintf(line, size, "test: %lu PGN in (254: %lu, 239: %lu, 200: %lu), %lu bad, %lu test PGN out",
             (unsigned long)rxGood, (unsigned long)perPgn[254], (unsigned long)perPgn[239], (unsigned long)perPgn[200],
             (unsigned long)rxBad, (unsigned long)testCount);
    if (have254)
    {
        const size_t n = strlen(line);
        snprintf(line + n, size - n, ", %.1f km/h, steer %.2f, guidance %u", speedKmh, steerSetDeg, guidance);
    }
}

void setup()
{
    Serial.begin(115200);                           // USB-C serial monitor (bench only)
    Teensy.setRxBufferSize(1024);
    Teensy.begin(TEENSY_BAUD, SERIAL_8N1, PIN_FROM_TEENSY, PIN_TO_TEENSY);
    delay(50);
    Teensy.print("ESP32-hello");
    Teensy.print("\nXIAO ESP32C3 bridge test v1 started\n");
}

void loop()
{
    static uint32_t lastHello = 0, lastTest = 0, lastText = 0, lastUsb = 0;
    const uint32_t now = millis();

    if (now - lastHello >= HELLO_MS)
    {
        lastHello = now;
        Teensy.print("ESP32-hello");                // plain text, no line ending
    }

    readTeensy();

    if (now - lastTest >= TEST_PGN_MS)
    {
        lastTest = now;
        testCount++;
        const uint8_t d[8] = {(uint8_t)testCount, (uint8_t)(testCount >> 8), (uint8_t)(testCount >> 16), (uint8_t)(testCount >> 24), 'X', 'I', 'A', 'O'};
        sendPgn(TEST_SOURCE, TEST_PGN, d, sizeof(d));
    }

    char line[200];
    if (now - lastText >= TEXT_MS || lastText == 0)
    {
        lastText = now;
        report(line, sizeof(line));
        Teensy.print(line);
        Teensy.print("\n");                         // a text line for the Teensy log ends with a line feed
    }

    if (now - lastUsb >= USB_REPORT_MS)
    {
        lastUsb = now;
        if (Serial)                                 // only when a PC is listening on the USB-C
        {
            report(line, sizeof(line));
            Serial.println(line);
        }
    }
}
