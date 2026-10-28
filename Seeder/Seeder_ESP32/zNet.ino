/* zNet.ino - the network cable, Wi-Fi (own network + the tractor's), the web page and its API, program update.
   Part of the seed drill section controller (folder Seeder of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr. Made from Sprayer_ESP32 v0.8.
   Same pattern as the XIAO program of the AiO v4.5 boards (Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32).
   2026-10-28 (Claude): v0.2: the network chip of the board chosen in Seeder_ESP32.ino (RTL8201F on the
   Relay24_ESP32, LAN8720 on the KC868-A32), held in reset before a restart.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Own network "Seeder-xxxx" at 192.168.4.1, always on (a phone that joins it gets the page by itself).
// Tractor network (the router AgOpenWeb's tablet uses): through the network cable, with a fixed IP
// <subnet>.123 (the last number can be changed on the page, for a second module), which is where the program
// looks for the machine module. The page can choose the tractor's Wi-Fi instead of the cable, with the same
// fixed IP. Changes on the page need the own network's password.
//
// GET  /              the page                       GET  /api/status    everything the page shows
// GET  /api/settings  network + relays settings      POST /api/wifi      network settings + subnet, then restart
// POST /api/relays    the first section relay 1 follows (0 = the program's pin config)
// POST /api/restart                                  POST /api/update    a new program (.bin), then restart
// GET  /aow/status    name and alarms, as the "Implements bring their UI" proposal describes

static bool updateAuth = false;

// The module's fixed IP on the tractor network
IPAddress moduleIp() { return IPAddress(cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], cfg.ipLast); }

// ---------------------------------------------------------------- network cable (the board's network chip)
// Relay24_ESP32: RTL8201F, its clock into GPIO 0, its reset on GPIO 33 (ETH.begin lets it out of reset
// before it starts the ESP32's network block, which needs that clock). KC868-A32: LAN8720, clock out on
// GPIO 17. The pins are in Seeder_ESP32.ino.
static bool ethStarted = false, ethBegun = false;

static void ethStart()
{
    ethBegun = true;
    // The arguments of ETH.begin changed order in version 3 of the esp32 boards package
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ethStarted = ETH.begin(LAN_PHY, LAN_PHY_ADDR, LAN_MDC, LAN_MDIO, LAN_POWER, LAN_CLOCK);
#else
    ethStarted = ETH.begin(LAN_PHY_ADDR, LAN_POWER, LAN_MDC, LAN_MDIO, LAN_PHY, LAN_CLOCK);
#endif
    if (!ethStarted) { logEvent("network cable: the network chip did not start"); return; }
    const IPAddress gw(cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], 1);
    ETH.config(moduleIp(), gw, IPAddress(255, 255, 255, 0), gw);
}

bool ethUp() { return ethStarted && ETH.linkUp(); }

// ETH.begin has been called (it takes GPIO 0 on the Relay24_ESP32, even when it fails)
bool ethTried() { return ethBegun; }

// Before a restart: the network chip back in reset, so its clock on GPIO 0 stops (Relay24_ESP32)
void ethHoldReset()
{
    if (LAN_POWER >= 0 && ethBegun) gpio_set_level((gpio_num_t)LAN_POWER, 0);
}

// ---------------------------------------------------------------- Wi-Fi
void wifiStart()
{
    WiFi.persistent(false);
    WiFi.setHostname(cfg.host.c_str());
    const bool sta = cfg.staOn && cfg.staSsid.length() > 0;
    WiFi.mode(sta ? WIFI_AP_STA : WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(cfg.apSsid.c_str(), cfg.apPass.c_str());
    if (sta)
    {
        const IPAddress gw(cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], 1);
        WiFi.config(moduleIp(), gw, IPAddress(255, 255, 255, 0), gw);
        WiFi.setAutoReconnect(true);
        WiFi.begin(cfg.staSsid.c_str(), cfg.staPass.c_str());
    }
    if (!sta) ethStart();                                   // the cable, unless the tractor's Wi-Fi is chosen
    dns.setErrorReplyCode(DNSReplyCode::NoError);
    dns.start(53, "*", WiFi.softAPIP());                    // every name -> this page (captive page)
    if (MDNS.begin(cfg.host.c_str()))
    {
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("aow-ui", "tcp", 80);               // how an AgOpenWeb could find this page one day
        MDNS.addServiceTxt("aow-ui", "tcp", "v", "1");
        MDNS.addServiceTxt("aow-ui", "tcp", "name", moduleName());
        MDNS.addServiceTxt("aow-ui", "tcp", "path", "/");
        MDNS.addServiceTxt("aow-ui", "tcp", "status", "/aow/status");
    }
    logEvent("own network '%s' at 192.168.4.1%s", cfg.apSsid.c_str(),
             sta ? ", joining the tractor's Wi-Fi" : ", tractor network on the cable");
}

// Once a second: say when the tractor network comes and goes
void wifiWatch()
{
    const bool cable = ethUp();
    if (cable != ethWasUp)
    {
        ethWasUp = cable;
        if (cable) logEvent("network cable connected, IP %s, %u Mbit/s", moduleIp().toString().c_str(), (unsigned)ETH.linkSpeed());
        else logEvent("network cable: no link (unplugged, or the switch is off)");
    }
    const bool up = staUp();
    if (up == staWasUp) return;
    staWasUp = up;
    if (up) logEvent("joined '%s', IP %s, %d dBm", cfg.staSsid.c_str(), WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
    else if (cfg.staOn) logEvent("lost the tractor network '%s'", cfg.staSsid.c_str());
}

// ---------------------------------------------------------------- web helpers
static void sendJson(const String& body, int code = 200)
{
    server.sendHeader("Cache-Control", "no-store");
    server.send(code, "application/json", body);
}

static void sendError(int code, const char* msg)
{
    sendJson(String("{\"ok\":false,\"error\":\"") + msg + "\"}", code);
}

static bool passwordOk()
{
    if (server.arg("pw") == cfg.apPass) return true;
    sendError(403, "wrong password");
    return false;
}

static String jsonStr(const String& s)
{
    String o = "\"";
    for (size_t i = 0; i < s.length(); i++)
    {
        const char c = s[i];
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if ((uint8_t)c < 0x20) o += ' ';
        else o += c;
    }
    return o + "\"";
}

// Sections 1-64 as hex digits for the page, section 1 = the lowest bit ("7ffff" = sections 1-19)
static String jsonHex(uint64_t v)
{
    char b[20];
    if (v >> 32) snprintf(b, sizeof(b), "\"%lx%08lx\"", (unsigned long)(v >> 32), (unsigned long)(v & 0xFFFFFFFF));
    else snprintf(b, sizeof(b), "\"%lx\"", (unsigned long)v);
    return String(b);
}

// A whole number lo..hi in the request (only digits), else false
static bool argNumber(const char* name, long lo, long hi, long& out)
{
    const String a = server.arg(name);
    if (a.length() < 1 || a.length() > 5) return false;
    for (size_t i = 0; i < a.length(); i++) if (a[i] < '0' || a[i] > '9') return false;
    out = a.toInt();
    return out >= lo && out <= hi;
}

static bool printable(const String& s, size_t lo, size_t hi)
{
    if (s.length() < lo || s.length() > hi) return false;
    for (size_t i = 0; i < s.length(); i++) if ((uint8_t)s[i] < 0x20 || (uint8_t)s[i] > 0x7E) return false;
    return true;
}

// Requests on the own network for any other host name (phones checking for internet) go to the page
static bool captiveRedirect()
{
    if (!(server.client().localIP() == WiFi.softAPIP())) return false;
    String h = server.hostHeader();
    h.toLowerCase();
    if (h.length() == 0 || h == "192.168.4.1" || h == cfg.host + ".local") return false;
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
    return true;
}

// ---------------------------------------------------------------- web answers
static void apiStatus()
{
    const uint32_t now = millis();
    String j;
    j.reserve(2000);
    j += "{\"fw\":\"" FW_NAME "\",\"ver\":\"" FW_VERSION "\",\"built\":\"" __DATE__ " " __TIME__ "\"";
    j += ",\"kind\":\"Seeder\",\"board\":\"" BOARD_NAME "\"";
    j += ",\"up\":" + String(now / 1000) + ",\"heap\":" + String(ESP.getFreeHeap());
    j += ",\"name\":" + jsonStr(moduleName());

    j += ",\"wifi\":{\"ap\":" + jsonStr(cfg.apSsid) + ",\"apCl\":" + String(WiFi.softAPgetStationNum());
    j += ",\"staOn\":" + String(cfg.staOn ? "true" : "false") + ",\"sta\":" + jsonStr(cfg.staSsid);
    j += ",\"up\":" + String(netUp() ? "true" : "false") + ",\"staUp\":" + String(staUp() ? "true" : "false");
    j += ",\"eth\":true,\"ethUp\":" + String(ethUp() ? "true" : "false");
    j += ",\"ip\":" + jsonStr(netUp() ? moduleIp().toString() : String("")) + ",\"ipLast\":" + String(cfg.ipLast);
    j += ",\"rssi\":" + String(staUp() ? WiFi.RSSI() : 0) + ",\"host\":" + jsonStr(cfg.host);
    j += ",\"subnet\":\"" + String(cfg.subnet[0]) + "." + String(cfg.subnet[1]) + "." + String(cfg.subnet[2]) + "\"}";

    j += ",\"aog\":{\"link\":" + String(linkOk ? "true" : "false");
    j += ",\"ago239\":" + String(have239 ? (long)((now - last239Ms) / 1000) : -1);
    j += ",\"agoHello\":" + String(haveHello ? (long)((now - lastHelloMs) / 1000) : -1);
    j += ",\"kmh\":" + String(gpsKmh, 1) + ",\"hyd\":" + String(hydLift) + ",\"tram\":" + String(tramline);
    j += ",\"geo\":" + String(geoStop) + ",\"uturn\":" + String(uTurn) + ",\"req\":" + jsonHex(sectionsReq);
    j += ",\"secTop\":" + String(secTop) + ",\"ago229\":" + String(have229 ? (long)((now - last229Ms) / 1000) : -1);
    j += ",\"n239\":" + String(count239) + ",\"n229\":" + String(count229);
    j += ",\"nHello\":" + String(countHello) + ",\"crcErr\":" + String(countCrcErr);
    j += ",\"invert\":" + String(cfg.invertOutputs) + ",\"hydEn\":" + String(cfg.hydEnable);
    j += ",\"raise\":" + String(cfg.raiseTime) + ",\"lower\":" + String(cfg.lowerTime) + "}";

    j += ",\"out\":{\"bits\":" + String(relayBits) + ",\"on\":" + jsonHex(sectionsOn) + ",\"first\":" + String(cfg.firstSection);
    j += ",\"onHigh\":" + String(RELAY_ON_HIGH ? "true" : "false") + ",\"fn\":[";
    for (uint8_t i = 0; i < NUM_RELAYS; i++) { if (i) j += ","; j += String(relayFn(i)); }
    j += "]}";

    j += ",\"log\":[";
    bool first = true;
    for (uint8_t k = 0; k < 12; k++)
    {
        const char* line = eventLog[(eventNext + k) % 12];
        if (!line[0]) continue;
        if (!first) j += ",";
        first = false;
        j += jsonStr(line);
    }
    j += "]}";
    sendJson(j);
}

static void apiSettingsGet()
{
    String j = "{\"apSsid\":" + jsonStr(cfg.apSsid) + ",\"staOn\":" + String(cfg.staOn ? "true" : "false");
    j += ",\"staSsid\":" + jsonStr(cfg.staSsid) + ",\"staPassSet\":" + String(cfg.staPass.length() ? "true" : "false");
    j += ",\"host\":" + jsonStr(cfg.host) + ",\"eth\":true,\"ipLast\":" + String(cfg.ipLast) + ",\"first\":" + String(cfg.firstSection);
    j += ",\"subnet\":\"" + String(cfg.subnet[0]) + "." + String(cfg.subnet[1]) + "." + String(cfg.subnet[2]) + "\"}";
    sendJson(j);
}

// "192.168.1" -> the three numbers; false when it is not three numbers 0-255
static bool parseSubnet(const String& text, uint8_t out[3])
{
    int part = 0, value = -1;
    for (size_t i = 0; i <= text.length(); i++)
    {
        const char c = i < text.length() ? text[i] : '.';
        if (c >= '0' && c <= '9') { value = (value < 0 ? 0 : value * 10) + (c - '0'); if (value > 255) return false; }
        else if (c == '.') { if (value < 0 || part == 3) return false; out[part++] = value; value = -1; }
        else return false;
    }
    return part == 3;
}

static void apiWifiPost()
{
    if (!passwordOk()) return;
    Settings s = cfg;
    s.apSsid = server.arg("apSsid");
    if (!printable(s.apSsid, 1, 32)) { sendError(400, "own network name: 1 to 32 characters"); return; }
    const String apPass = server.arg("apPass");
    if (apPass.length() && !printable(apPass, 8, 63)) { sendError(400, "own network password: 8 to 63 characters"); return; }
    if (apPass.length()) s.apPass = apPass;
    s.staOn = server.arg("staOn") == "1";
    s.staSsid = server.arg("staSsid");
    if (s.staOn && !printable(s.staSsid, 1, 32)) { sendError(400, "tractor network name: 1 to 32 characters"); return; }
    const String staPass = server.arg("staPass");
    if (staPass.length() && !printable(staPass, 8, 63)) { sendError(400, "tractor network password: 8 to 63 characters"); return; }
    if (staPass.length() || s.staSsid != cfg.staSsid) s.staPass = staPass;
    s.host = server.arg("host");
    if (!printable(s.host, 1, 24)) { sendError(400, "name: 1 to 24 characters"); return; }
    if (!parseSubnet(server.arg("subnet"), s.subnet)) { sendError(400, "subnet: three numbers, like 192.168.1"); return; }
    long ipLast = cfg.ipLast;
    if (server.hasArg("ipLast") && !argNumber("ipLast", 2, 254, ipLast)) { sendError(400, "module IP last number: 2 to 254"); return; }
    s.ipLast = ipLast;
    cfg = s;
    settingsSave();
    logEvent("network settings saved from the page, restarting");
    sendJson("{\"ok\":true,\"restart\":true}");
    restartAtMs = millis() + 500;
}

// Relay 1 follows section "first", the next relays the next sections (0 = the program's pin config)
static void apiRelaysPost()
{
    if (!passwordOk()) return;
    long first = 0;
    if (!argNumber("first", 0, MAX_SECTIONS, first)) { sendError(400, "first section: 0 to 64"); return; }
    cfg.firstSection = first;
    settingsSave();
    if (first) logEvent("relays 1-%u follow sections %ld-%ld", (unsigned)NUM_RELAYS, first, min((long)MAX_SECTIONS, first + NUM_RELAYS - 1));
    else logEvent("relays follow the program's pin config");
    applyOutputs();
    sendJson("{\"ok\":true}");
}

static void apiRestartPost()
{
    if (!passwordOk()) return;
    logEvent("restart from the page");
    sendJson("{\"ok\":true,\"restart\":true}");
    restartAtMs = millis() + 500;
}

// The name and the alarms, as the "Implements bring their UI" proposal describes (GET /aow/status)
static void apiAowStatus()
{
    String j = "{\"v\":1,\"name\":" + jsonStr(moduleName()) + ",\"alarms\":[";
    if (!linkOk) j += "{\"id\":\"no-program\",\"priority\":\"medium\",\"text\":\"No messages from AgOpenWeb\"}";
    j += "]}";
    sendJson(j);
}

// A new program (.bin from the Arduino IDE: Sketch > Export compiled binary), password in the URL
static void apiUpdateDone()
{
    if (!updateAuth) { sendError(403, "wrong password"); return; }
    if (Update.hasError()) { sendError(500, "update failed, the old program stays"); return; }
    sendJson("{\"ok\":true,\"restart\":true}");
    restartAtMs = millis() + 500;
}

static void apiUpdateUpload()
{
    HTTPUpload& up = server.upload();
    if (up.status == UPLOAD_FILE_START)
    {
        updateAuth = server.arg("pw") == cfg.apPass;
        if (!updateAuth) return;
        outputsAllOff();
        logEvent("program update: %s", up.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) logEvent("update: no room");
    }
    else if (up.status == UPLOAD_FILE_WRITE)
    {
        if (updateAuth && Update.write(up.buf, up.currentSize) != up.currentSize) logEvent("update: write failed");
    }
    else if (up.status == UPLOAD_FILE_END)
    {
        if (updateAuth && Update.end(true)) logEvent("update: %u bytes written, restarting", (unsigned)up.totalSize);
        else if (updateAuth) logEvent("update: failed");
    }
}

static void notFound()
{
    if (captiveRedirect()) return;
    server.send(404, "text/plain", "not found");
}

void webStart()
{
    server.on("/", HTTP_GET, []() {
        if (captiveRedirect()) return;
        server.sendHeader("Cache-Control", "no-store");
        server.send_P(200, "text/html", SEEDER_PAGE);
    });
    server.on("/api/status", HTTP_GET, apiStatus);
    server.on("/api/settings", HTTP_GET, apiSettingsGet);
    server.on("/api/wifi", HTTP_POST, apiWifiPost);
    server.on("/api/relays", HTTP_POST, apiRelaysPost);
    server.on("/api/restart", HTTP_POST, apiRestartPost);
    server.on("/api/update", HTTP_POST, apiUpdateDone, apiUpdateUpload);
    server.on("/aow/status", HTTP_GET, apiAowStatus);
    server.onNotFound(notFound);
    server.begin();
}
