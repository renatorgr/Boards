/* zNet.ino - Wi-Fi (own network + the tractor's network), the web page and its API, program update.
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   Same pattern as the XIAO program of the AiO v4.5 boards (Firmware/v4.5_AiO_WEB/v4.5_XIAO_ESP32).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Own network "Sprayer-xxxx" at 192.168.4.1, always on (a phone that joins it gets the page by itself).
// Tractor network (the router AgOpenWeb's tablet uses): joined with a fixed IP <subnet>.123, which is
// where the program looks for the machine module. Changes on the page need the own network's password.
//
// GET  /              the page                       GET  /api/status    everything the page shows
// GET  /api/settings  Wi-Fi + calibration            POST /api/wifi      Wi-Fi settings + subnet, then restart
// POST /api/calib     calibration                    POST /api/total     total litres back to 0
// POST /api/restart                                  POST /api/update    a new program (.bin), then restart
// GET  /aow/status    name and alarms, as the "Implements bring their UI" proposal describes

static bool updateAuth = false;

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
        const IPAddress ip(cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], 123);
        const IPAddress gw(cfg.subnet[0], cfg.subnet[1], cfg.subnet[2], 1);
        WiFi.config(ip, gw, IPAddress(255, 255, 255, 0), gw);
        WiFi.setAutoReconnect(true);
        WiFi.begin(cfg.staSsid.c_str(), cfg.staPass.c_str());
    }
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
    j.reserve(1500);
    j += "{\"fw\":\"" FW_NAME "\",\"ver\":\"" FW_VERSION "\",\"built\":\"" __DATE__ " " __TIME__ "\"";
    j += ",\"up\":" + String(now / 1000) + ",\"heap\":" + String(ESP.getFreeHeap());
    j += ",\"name\":" + jsonStr(moduleName());

    j += ",\"wifi\":{\"ap\":" + jsonStr(cfg.apSsid) + ",\"apCl\":" + String(WiFi.softAPgetStationNum());
    j += ",\"staOn\":" + String(cfg.staOn ? "true" : "false") + ",\"sta\":" + jsonStr(cfg.staSsid);
    j += ",\"up\":" + String(staUp() ? "true" : "false") + ",\"ip\":" + jsonStr(staUp() ? WiFi.localIP().toString() : String(""));
    j += ",\"rssi\":" + String(staUp() ? WiFi.RSSI() : 0) + ",\"host\":" + jsonStr(cfg.host);
    j += ",\"subnet\":\"" + String(cfg.subnet[0]) + "." + String(cfg.subnet[1]) + "." + String(cfg.subnet[2]) + "\"}";

    j += ",\"aog\":{\"link\":" + String(linkOk ? "true" : "false");
    j += ",\"ago239\":" + String(have239 ? (long)((now - last239Ms) / 1000) : -1);
    j += ",\"agoHello\":" + String(haveHello ? (long)((now - lastHelloMs) / 1000) : -1);
    j += ",\"kmh\":" + String(gpsKmh, 1) + ",\"hyd\":" + String(hydLift) + ",\"tram\":" + String(tramline);
    j += ",\"geo\":" + String(geoStop) + ",\"uturn\":" + String(uTurn) + ",\"req\":" + String(sectionsReq);
    j += ",\"n239\":" + String(count239) + ",\"nHello\":" + String(countHello) + ",\"crcErr\":" + String(countCrcErr);
    j += ",\"invert\":" + String(cfg.invertOutputs) + ",\"hydEn\":" + String(cfg.hydEnable);
    j += ",\"raise\":" + String(cfg.raiseTime) + ",\"lower\":" + String(cfg.lowerTime) + "}";

    j += ",\"out\":{\"bits\":" + String(relayBits) + ",\"on\":" + String(sectionsOn) + ",\"fn\":[";
    for (uint8_t i = 0; i < NUM_RELAYS; i++) { if (i) j += ","; j += String(cfg.pin[i]); }
    j += "]}";

    j += ",\"spr\":{\"bar\":" + String(pressureBar, 2) + ",\"lpm\":" + String(flowLpm, 2) + ",\"total\":" + String(totalLitres, 1);
    j += ",\"master\":" + String(masterOn ? "true" : "false") + ",\"volts\":" + String(sensorVolts, 2) + ",\"pulses\":" + String(totalPulses) + "}";

    j += ",\"cal\":{\"ppl\":" + String(cfg.pulsesPerLitre, 1) + ",\"pmax\":" + String(cfg.pressureMaxBar, 1) + ",\"pdiv\":" + String(cfg.pressureDivider, 3) + "}";

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
    j += ",\"host\":" + jsonStr(cfg.host) + ",\"ppl\":" + String(cfg.pulsesPerLitre, 1);
    j += ",\"subnet\":\"" + String(cfg.subnet[0]) + "." + String(cfg.subnet[1]) + "." + String(cfg.subnet[2]) + "\"";
    j += ",\"pmax\":" + String(cfg.pressureMaxBar, 1) + ",\"pdiv\":" + String(cfg.pressureDivider, 3) + "}";
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
    cfg = s;
    settingsSave();
    logEvent("Wi-Fi settings saved from the page, restarting");
    sendJson("{\"ok\":true,\"restart\":true}");
    restartAtMs = millis() + 500;
}

static void apiCalibPost()
{
    if (!passwordOk()) return;
    const float ppl = server.arg("ppl").toFloat();
    const float pmax = server.arg("pmax").toFloat();
    const float pdiv = server.arg("pdiv").toFloat();
    if (!(ppl > 1 && ppl < 100000)) { sendError(400, "pulses per litre: 1 to 100000"); return; }
    if (!(pmax > 0.5f && pmax < 100)) { sendError(400, "sensor range: 0.5 to 100 bar"); return; }
    if (!(pdiv > 0.1f && pdiv <= 1)) { sendError(400, "divider: 0.1 to 1"); return; }
    cfg.pulsesPerLitre = ppl;
    cfg.pressureMaxBar = pmax;
    cfg.pressureDivider = pdiv;
    settingsSave();
    logEvent("calibration saved: %.1f pulses/L, %.1f bar sensor, divider %.3f", ppl, pmax, pdiv);
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
        server.send_P(200, "text/html", SPRAYER_PAGE);
    });
    server.on("/api/status", HTTP_GET, apiStatus);
    server.on("/api/settings", HTTP_GET, apiSettingsGet);
    server.on("/api/wifi", HTTP_POST, apiWifiPost);
    server.on("/api/calib", HTTP_POST, apiCalibPost);
    server.on("/api/total", HTTP_POST, apiTotalPost);
    server.on("/api/restart", HTTP_POST, apiRestartPost);
    server.on("/api/update", HTTP_POST, apiUpdateDone, apiUpdateUpload);
    server.on("/aow/status", HTTP_GET, apiAowStatus);
    server.onNotFound(notFound);
    server.begin();
}
