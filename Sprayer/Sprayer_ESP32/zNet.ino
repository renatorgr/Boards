/* zNet.ino - Wi-Fi (own network + the tractor's network), the web page and its API, program update.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Same pattern as the XIAO program of the AiO v4.5 boards (Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32).
   2026-10-27 (Claude): the sensors fitted are saved with the calibration.
   2026-10-27 (Claude): the module IP's last number.
   2026-11-02 (Claude): v0.10.0: the spraying settings (POST /api/spray), the rate, pressure for the target and the
   alarms in /api/status, the sprayer alarms in /aow/status. The start of /api/status is read from the .bin by the
   page's update check and by Firmware/simulator/build.py (firmware.json): keep it as it is.
   2026-11-02 (Claude): v0.10.1: GET /api/scan, the Wi-Fi networks the module hears, for "Search for networks" on the
   page (as on the XIAO page).
   2026-11-07 (Claude): v0.10.2: the page is announced as an AgOpenWeb add-on card (mDNS "_aowcard._tcp", TXT v=1 and path,
   the official Docs/ADD_ON_CARDS.md) under the own network's name; the home-made "_aow-ui._tcp" and GET /aow/status are gone.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Own network "Sprayer-xxxx" at 192.168.4.1, always on (a phone that joins it gets the page by itself).
// Tractor network (the router AgOpenWeb's tablet uses): joined with a fixed IP <subnet>.123 (the last number
// can be changed on the page, for a second module), which is where the program looks for the machine module.
// Changes on the page need the own network's password.
//
// GET  /              the page                       GET  /api/status    everything the page shows
// GET  /api/settings  Wi-Fi + sensors + calibration  POST /api/wifi      Wi-Fi settings + subnet, then restart
// POST /api/calib     sensors fitted + calibration   POST /api/total     total litres back to 0
// POST /api/spray     target rate, nozzles, pressure limits, flow tolerance
// GET  /api/scan      the Wi-Fi networks around      POST /api/restart
// POST /api/update    a new program (.bin), then restart

static bool updateAuth = false;

// The module's fixed IP on the tractor network
IPAddress moduleIp() { return IPAddress(cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], cfg.ipLast); }

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
    dns.setErrorReplyCode(DNSReplyCode::NoError);
    dns.start(53, "*", WiFi.softAPIP());                    // every name -> this page (captive page)
    if (MDNS.begin(cfg.host.c_str()))
    {
        MDNS.setInstanceName(cfg.apSsid);                   // the name in AgOpenWeb's list: the own network's, unique to this module
        MDNS.addService("http", "tcp", 80);                 // the page, for the AgOpenWeb fork's "Search the network"
        MDNS.addService("aowcard", "tcp", 80);              // the official AgOpenWeb add-on card announcement (Docs/ADD_ON_CARDS.md)
        MDNS.addServiceTxt("aowcard", "tcp", "v", "1");     //   TXT v=1 is required, path says where the page is
        MDNS.addServiceTxt("aowcard", "tcp", "path", "/");
    }
    logEvent("own network '%s' at 192.168.4.1%s", cfg.apSsid.c_str(), sta ? ", joining the tractor network" : ", tractor network not set");
}

// Once a second: say when the tractor network comes and goes
void wifiWatch()
{
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

// A number for the JSON with d decimals; -1 for "none" (a NaN would break the JSON)
static String jsonNum(float v, uint8_t d)
{
    return isfinite(v) && v >= 0 ? String(v, (unsigned int)d) : String("-1");
}

// A number lo..hi in the request (digits and one dot), else false
static bool argFloat(const char* name, float lo, float hi, float& out)
{
    const String a = server.arg(name);
    if (a.length() < 1 || a.length() > 8) return false;
    uint8_t dots = 0;
    for (size_t i = 0; i < a.length(); i++)
    {
        if (a[i] == '.') { if (++dots > 1) return false; }
        else if (a[i] < '0' || a[i] > '9') return false;
    }
    out = a.toFloat();
    return out >= lo && out <= hi;
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
    // one text in the program image: the page (before it installs a .bin) and build.py (firmware.json) find in it
    // which program, version and build a .bin is, and the board in the line below; keep both as they are
    j += "{\"fw\":\"" FW_NAME "\",\"ver\":\"" FW_VERSION "\",\"built\":\"" __DATE__ " " __TIME__ "\"";
    j += ",\"board\":\"" BOARD_NAME "\"";
    j += ",\"up\":" + String(now / 1000) + ",\"heap\":" + String(ESP.getFreeHeap());
    j += ",\"name\":" + jsonStr(moduleName());

    j += ",\"wifi\":{\"ap\":" + jsonStr(cfg.apSsid) + ",\"apCl\":" + String(WiFi.softAPgetStationNum());
    j += ",\"staOn\":" + String(cfg.staOn ? "true" : "false") + ",\"sta\":" + jsonStr(cfg.staSsid);
    j += ",\"up\":" + String(staUp() ? "true" : "false");
    j += ",\"ip\":" + jsonStr(staUp() ? moduleIp().toString() : String("")) + ",\"ipLast\":" + String(cfg.ipLast);
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

    j += ",\"out\":{\"bits\":" + String(relayBits) + ",\"on\":" + jsonHex(sectionsOn);
    j += ",\"onHigh\":" + String(RELAY_ON_HIGH ? "true" : "false") + ",\"fn\":[";
    for (uint8_t i = 0; i < NUM_RELAYS; i++) { if (i) j += ","; j += String(relayFn(i)); }
    j += "]}";

    j += ",\"spr\":{\"bar\":" + String(pressureBar, 2) + ",\"lpm\":" + String(flowLpm, 2) + ",\"total\":" + String(totalLitres, 1);
    j += ",\"master\":" + String(masterOn ? "true" : "false") + ",\"volts\":" + String(sensorVolts, 2) + ",\"pulses\":" + String(totalPulses) + "}";

    j += ",\"cal\":{\"sensors\":" + String(cfg.sensors) + ",\"ppl\":" + String(cfg.pulsesPerLitre, 1) + ",\"pmax\":" + String(cfg.pressureMaxBar, 1) + ",\"pdiv\":" + String(cfg.pressureDivider, 3) + "}";

    // the rate and the alarms (zRate.ino; -1 = none), the spraying settings, the tool's sections (PGN 235)
    j += ",\"rate\":{\"lha\":" + jsonNum(rateLha, 1) + ",\"target\":" + jsonNum(cfg.targetRate, 0);
    j += ",\"needBar\":" + jsonNum(needBar, 2) + ",\"hintKmh\":" + jsonNum(hintKmh, 1);
    j += ",\"width\":" + String(openWidthM, 2) + ",\"nozzles\":" + String(openNozzles) + ",\"expected\":" + jsonNum(expectedLpm, 2);
    j += ",\"alarms\":" + String(sprayAlarms) + ",\"minBar\":" + String(cfg.minBar, 1) + ",\"maxBar\":" + String(cfg.maxBar, 1) + "}";
    j += ",\"tool\":{\"n\":" + String(cfg.toolSections) + ",\"n235\":" + String(count235) + ",\"cm\":[";
    for (uint8_t i = 0; i < MAX_TOOL_SECTIONS && i < cfg.toolSections; i++) { if (i) j += ","; j += String(cfg.sectionCm[i]); }
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
    j += ",\"host\":" + jsonStr(cfg.host) + ",\"sensors\":" + String(cfg.sensors) + ",\"ppl\":" + String(cfg.pulsesPerLitre, 1);
    j += ",\"ipLast\":" + String(cfg.ipLast);
    j += ",\"subnet\":\"" + String(cfg.subnet[0]) + "." + String(cfg.subnet[1]) + "." + String(cfg.subnet[2]) + "\"";
    j += ",\"pmax\":" + String(cfg.pressureMaxBar, 1) + ",\"pdiv\":" + String(cfg.pressureDivider, 3);
    j += ",\"rate\":" + String(cfg.targetRate, 0) + ",\"nzSp\":" + String(cfg.nozzleSpacing, 2) + ",\"nzFl\":" + String(cfg.nozzleFlow, 2);
    j += ",\"nzBar\":" + String(cfg.nozzleBar, 1) + ",\"pLow\":" + String(cfg.minBar, 1) + ",\"pHigh\":" + String(cfg.maxBar, 1);
    j += ",\"fTol\":" + String(cfg.flowTolerance) + "}";
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
    logEvent("Wi-Fi settings saved from the page, restarting");
    sendJson("{\"ok\":true,\"restart\":true}");
    restartAtMs = millis() + 500;
}

// GET /api/scan: the Wi-Fi networks the module hears, as on the XIAO page. The first call starts a search in the
// background and answers {"busy":true}; the page asks again until the list comes: {"busy":false,"list":[{"s":name,
// "r":dBm,"c":channel,"open":true|false}]}, strongest first, each name once. No password: it changes nothing.
static int8_t scanState = -2;             // -2 not started, -1 running

static void apiScan()
{
    const int16_t n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) { sendJson("{\"busy\":true}"); return; }
    if (n >= 0 && scanState == -1)
    {
        String j = "{\"busy\":false,\"list\":[";
        bool first = true;
        for (int i = 0; i < n; i++)
        {
            const String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) continue;
            bool dup = false;
            for (int k = 0; k < i; k++) if (WiFi.SSID(k) == ssid) dup = true;   // strongest first (sorted)
            if (dup) continue;
            if (!first) j += ',';
            first = false;
            j += "{\"s\":" + jsonStr(ssid) + ",\"r\":" + String((int)WiFi.RSSI(i)) + ",\"c\":" + String((int)WiFi.channel(i)) +
                 ",\"open\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "true" : "false") + "}";
        }
        j += "]}";
        WiFi.scanDelete();
        scanState = -2;
        sendJson(j);
        return;
    }
    WiFi.scanNetworks(true);              // in the background; the page asks again
    scanState = -1;
    sendJson("{\"busy\":true}");
}

static void apiCalibPost()
{
    if (!passwordOk()) return;
    const float ppl = server.arg("ppl").toFloat();
    const float pmax = server.arg("pmax").toFloat();
    const float pdiv = server.arg("pdiv").toFloat();
    const long sensors = server.hasArg("sensors") ? server.arg("sensors").toInt() : cfg.sensors;   // an older page sends none
    if (!(ppl > 1 && ppl < 100000)) { sendError(400, "pulses per litre: 1 to 100000"); return; }
    if (!(pmax > 0.5f && pmax < 100)) { sendError(400, "sensor range: 0.5 to 100 bar"); return; }
    if (!(pdiv > 0.1f && pdiv <= 1)) { sendError(400, "divider: 0.1 to 1"); return; }
    if (sensors < 0 || sensors > (SENSOR_PRESSURE | SENSOR_FLOW)) { sendError(400, "sensors: 0 to 3"); return; }
    cfg.sensors = sensors;
    cfg.pulsesPerLitre = ppl;
    cfg.pressureMaxBar = pmax;
    cfg.pressureDivider = pdiv;
    settingsSave();
    static const char* const FITTED[] = { "no sensors", "pressure sensor", "flow meter", "pressure sensor + flow meter" };
    logEvent("saved: %s, %.1f pulses/L, %.1f bar sensor, divider %.3f", FITTED[cfg.sensors], ppl, pmax, pdiv);
    sendJson("{\"ok\":true}");
}

// The spraying settings (Settings tab): target rate, nozzles, pressure limits, flow tolerance
static void apiSprayPost()
{
    if (!passwordOk()) return;
    float rate, nzSp, nzFl, nzBar, pLow, pHigh, fTol;
    if (!argFloat("rate", 0, 2000, rate))     { sendError(400, "target rate: 0 to 2000 L/ha"); return; }
    if (!argFloat("nzSp", 0, 5, nzSp))        { sendError(400, "nozzle spacing: 0 to 5 m"); return; }
    if (!argFloat("nzFl", 0, 50, nzFl))       { sendError(400, "nozzle flow: 0 to 50 L/min"); return; }
    if (!argFloat("nzBar", 0.1f, 50, nzBar))  { sendError(400, "chart pressure: 0.1 to 50 bar"); return; }
    if (!argFloat("pLow", 0, 50, pLow))       { sendError(400, "minimum pressure: 0 to 50 bar"); return; }
    if (!argFloat("pHigh", 0, 50, pHigh))     { sendError(400, "maximum pressure: 0 to 50 bar"); return; }
    if (pHigh > 0 && pLow >= pHigh)           { sendError(400, "the minimum pressure must be below the maximum"); return; }
    if (!argFloat("fTol", 1, 100, fTol))      { sendError(400, "flow tolerance: 1 to 100 %"); return; }
    cfg.targetRate = rate;
    cfg.nozzleSpacing = nzSp;
    cfg.nozzleFlow = nzFl;
    cfg.nozzleBar = nzBar;
    cfg.minBar = pLow;
    cfg.maxBar = pHigh;
    cfg.flowTolerance = (uint8_t)lroundf(fTol);
    settingsSave();
    logEvent("saved: target %.0f L/ha, nozzles every %.2f m, %.2f L/min at %.1f bar, %.1f-%.1f bar, flow +-%u %%",
             rate, nzSp, nzFl, nzBar, pLow, pHigh, cfg.flowTolerance);
    sendJson("{\"ok\":true}");
}

static void apiTotalPost()
{
    if (!passwordOk()) return;
    totalLitres = 0;
    totalPulses = 0;
    logEvent("total set to 0 from the page");
    sendJson("{\"ok\":true}");
}

static void apiRestartPost()
{
    if (!passwordOk()) return;
    logEvent("restart from the page");
    sendJson("{\"ok\":true,\"restart\":true}");
    restartAtMs = millis() + 500;
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
        server.send_P(200, "text/html", SPRAYER_PAGE);
    });
    server.on("/api/status", HTTP_GET, apiStatus);
    server.on("/api/settings", HTTP_GET, apiSettingsGet);
    server.on("/api/wifi", HTTP_POST, apiWifiPost);
    server.on("/api/scan", HTTP_GET, apiScan);
    server.on("/api/calib", HTTP_POST, apiCalibPost);
    server.on("/api/spray", HTTP_POST, apiSprayPost);
    server.on("/api/total", HTTP_POST, apiTotalPost);
    server.on("/api/restart", HTTP_POST, apiRestartPost);
    server.on("/api/update", HTTP_POST, apiUpdateDone, apiUpdateUpload);
    server.onNotFound(notFound);
    server.begin();
}
