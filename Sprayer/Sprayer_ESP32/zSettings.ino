/* zSettings.ino - the settings kept in the ESP32's flash (Preferences).
   Part of the sprayer section controller (folder Sprayer of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// What is kept: the two Wi-Fi networks, the host name, the program's subnet (PGN 201) and the module IP's
// last number, the machine config (PGN 238), the output functions (PGN 236), the sensors fitted and their
// calibration, the spraying settings and the tool's section widths (PGN 235). Anything missing in flash keeps the
// default from the Settings struct in Sprayer_ESP32.ino.

String defaultApSsid()
{
    const uint64_t mac = ESP.getEfuseMac();
    char b[20];
    snprintf(b, sizeof(b), "Sprayer-%02X%02X", (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
    return String(b);
}

// The module's name on the network: its own Wi-Fi network's name ("Sprayer-xxxx" unless changed on the page), the mDNS
// instance name AgOpenWeb's add-on card list shows, so one module can be told from another (zNet.ino)
String moduleName() { return cfg.apSsid; }

void settingsLoad()
{
    Settings d;                                             // the defaults
    prefs.begin("sprayer", true);
    cfg.apSsid  = prefs.getString("apSsid", defaultApSsid());
    cfg.apPass  = prefs.getString("apPass", DEFAULT_AP_PASS);
    cfg.staOn   = prefs.getBool("staOn", false);
    cfg.staSsid = prefs.getString("staSsid", "");
    cfg.staPass = prefs.getString("staPass", "");
    cfg.host    = prefs.getString("host", DEFAULT_HOST);
    if (prefs.getBytesLength("subnet") == 3) prefs.getBytes("subnet", cfg.subnet, 3);
    cfg.ipLast       = prefs.getUChar("ipLast", d.ipLast);
    cfg.raiseTime     = prefs.getUChar("raise", d.raiseTime);
    cfg.lowerTime     = prefs.getUChar("lower", d.lowerTime);
    cfg.hydEnable     = prefs.getUChar("hyd", d.hydEnable);
    cfg.invertOutputs = prefs.getUChar("invert", d.invertOutputs);
    cfg.user1 = prefs.getUChar("user1", 0);
    cfg.user2 = prefs.getUChar("user2", 0);
    cfg.user3 = prefs.getUChar("user3", 0);
    cfg.user4 = prefs.getUChar("user4", 0);
    if (prefs.getBytesLength("pins") == 24) prefs.getBytes("pins", cfg.pin, 24);
    cfg.sensors         = prefs.getUChar("sensors", d.sensors);
    cfg.pulsesPerLitre  = prefs.getFloat("ppl", d.pulsesPerLitre);
    cfg.pressureMaxBar  = prefs.getFloat("pmax", d.pressureMaxBar);
    cfg.pressureDivider = prefs.getFloat("pdiv", d.pressureDivider);
    cfg.targetRate      = prefs.getFloat("rate", d.targetRate);
    cfg.nozzleSpacing   = prefs.getFloat("nzSp", d.nozzleSpacing);
    cfg.nozzleFlow      = prefs.getFloat("nzFl", d.nozzleFlow);
    cfg.nozzleBar       = prefs.getFloat("nzBar", d.nozzleBar);
    cfg.minBar          = prefs.getFloat("pLow", d.minBar);
    cfg.maxBar          = prefs.getFloat("pHigh", d.maxBar);
    cfg.flowTolerance   = prefs.getUChar("fTol", d.flowTolerance);
    cfg.toolSections    = prefs.getUChar("secN", 0);
    if (prefs.getBytesLength("secCm") == sizeof(cfg.sectionCm)) prefs.getBytes("secCm", cfg.sectionCm, sizeof(cfg.sectionCm));
    prefs.end();

    // never let a bad value through
    if (cfg.apSsid.length() < 1 || cfg.apSsid.length() > 32) cfg.apSsid = defaultApSsid();
    if (cfg.apPass.length() < 8 || cfg.apPass.length() > 63) cfg.apPass = DEFAULT_AP_PASS;
    if (cfg.host.length() < 1 || cfg.host.length() > 24)    cfg.host = DEFAULT_HOST;
    if (!(cfg.pulsesPerLitre > 1 && cfg.pulsesPerLitre < 100000)) cfg.pulsesPerLitre = d.pulsesPerLitre;
    if (!(cfg.pressureMaxBar > 0.5f && cfg.pressureMaxBar < 100))  cfg.pressureMaxBar = d.pressureMaxBar;
    if (!(cfg.pressureDivider > 0.1f && cfg.pressureDivider <= 1)) cfg.pressureDivider = d.pressureDivider;
    if (cfg.ipLast < 2 || cfg.ipLast > 254) cfg.ipLast = d.ipLast;
    if (!(cfg.targetRate >= 0 && cfg.targetRate <= 2000))  cfg.targetRate = d.targetRate;
    if (!(cfg.nozzleSpacing >= 0 && cfg.nozzleSpacing <= 5)) cfg.nozzleSpacing = d.nozzleSpacing;
    if (!(cfg.nozzleFlow >= 0 && cfg.nozzleFlow <= 50))    cfg.nozzleFlow = d.nozzleFlow;
    if (!(cfg.nozzleBar > 0 && cfg.nozzleBar <= 50))       cfg.nozzleBar = d.nozzleBar;
    if (!(cfg.minBar >= 0 && cfg.minBar <= 50))            cfg.minBar = d.minBar;
    if (!(cfg.maxBar >= 0 && cfg.maxBar <= 50))            cfg.maxBar = d.maxBar;
    if (cfg.flowTolerance < 1 || cfg.flowTolerance > 100)  cfg.flowTolerance = d.flowTolerance;
    cfg.sensors &= SENSOR_PRESSURE | SENSOR_FLOW;
    for (uint8_t i = 0; i < 24; i++) if (cfg.pin[i] > PIN_GEO_STOP) cfg.pin[i] = 0;
}

void settingsSave()
{
    prefs.begin("sprayer", false);
    prefs.putString("apSsid", cfg.apSsid);
    prefs.putString("apPass", cfg.apPass);
    prefs.putBool("staOn", cfg.staOn);
    prefs.putString("staSsid", cfg.staSsid);
    prefs.putString("staPass", cfg.staPass);
    prefs.putString("host", cfg.host);
    prefs.putBytes("subnet", cfg.subnet, 3);
    prefs.putUChar("ipLast", cfg.ipLast);
    prefs.putUChar("raise", cfg.raiseTime);
    prefs.putUChar("lower", cfg.lowerTime);
    prefs.putUChar("hyd", cfg.hydEnable);
    prefs.putUChar("invert", cfg.invertOutputs);
    prefs.putUChar("user1", cfg.user1);
    prefs.putUChar("user2", cfg.user2);
    prefs.putUChar("user3", cfg.user3);
    prefs.putUChar("user4", cfg.user4);
    prefs.putBytes("pins", cfg.pin, 24);
    prefs.putUChar("sensors", cfg.sensors);
    prefs.putFloat("ppl", cfg.pulsesPerLitre);
    prefs.putFloat("pmax", cfg.pressureMaxBar);
    prefs.putFloat("pdiv", cfg.pressureDivider);
    prefs.putFloat("rate", cfg.targetRate);
    prefs.putFloat("nzSp", cfg.nozzleSpacing);
    prefs.putFloat("nzFl", cfg.nozzleFlow);
    prefs.putFloat("nzBar", cfg.nozzleBar);
    prefs.putFloat("pLow", cfg.minBar);
    prefs.putFloat("pHigh", cfg.maxBar);
    prefs.putUChar("fTol", cfg.flowTolerance);
    prefs.putUChar("secN", cfg.toolSections);
    prefs.putBytes("secCm", cfg.sectionCm, sizeof(cfg.sectionCm));
    prefs.end();
}

// Only the Wi-Fi settings and the host name go back to the defaults (a forgotten password); the
// program's config, the output functions and the calibration stay
void settingsResetWifi()
{
    cfg.apSsid  = defaultApSsid();
    cfg.apPass  = DEFAULT_AP_PASS;
    cfg.staOn   = false;
    cfg.staSsid = "";
    cfg.staPass = "";
    cfg.host    = DEFAULT_HOST;
    settingsSave();
}
