/* zSettings.ino - the settings kept in the ESP32's flash (Preferences).
   Part of the seed drill section controller (folder Seeder of the Boards repository).
   Written 2026 by Claude (Anthropic) for renatorgr. Made from Sprayer_ESP32 v0.8.
   Licence: GNU General Public License v3 or later (Firmware/v4.5_AiO_WEB/COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// What is kept: the two Wi-Fi networks (the own one and the tractor's), the host name, the program's subnet
// (PGN 201) and the module IP's last number, the first section the relays follow, the machine config
// (PGN 238) and the output functions (PGN 236). Anything missing in flash keeps the default from the
// Settings struct in Seeder_ESP32.ino.

String defaultApSsid()
{
    const uint64_t mac = ESP.getEfuseMac();
    char b[20];
    snprintf(b, sizeof(b), "Seeder-%02X%02X", (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
    return String(b);
}

// A 16-digit name for this module (the ESP32's factory MAC), as the "Implements bring their UI" paper asks
String moduleName()
{
    const uint64_t mac = ESP.getEfuseMac();
    char b[20];
    snprintf(b, sizeof(b), "%08lX%08lX", (unsigned long)(mac >> 32), (unsigned long)(mac & 0xFFFFFFFF));
    return String(b);
}

void settingsLoad()
{
    Settings d;                                             // the defaults
    prefs.begin("seeder", true);
    cfg.apSsid  = prefs.getString("apSsid", defaultApSsid());
    cfg.apPass  = prefs.getString("apPass", DEFAULT_AP_PASS);
    cfg.staOn   = prefs.getBool("staOn", false);
    cfg.staSsid = prefs.getString("staSsid", "");
    cfg.staPass = prefs.getString("staPass", "");
    cfg.host    = prefs.getString("host", DEFAULT_HOST);
    if (prefs.getBytesLength("subnet") == 3) prefs.getBytes("subnet", cfg.subnet, 3);
    cfg.ipLast       = prefs.getUChar("ipLast", d.ipLast);
    cfg.firstSection = prefs.getUChar("first", d.firstSection);
    cfg.raiseTime     = prefs.getUChar("raise", d.raiseTime);
    cfg.lowerTime     = prefs.getUChar("lower", d.lowerTime);
    cfg.hydEnable     = prefs.getUChar("hyd", d.hydEnable);
    cfg.invertOutputs = prefs.getUChar("invert", d.invertOutputs);
    cfg.user1 = prefs.getUChar("user1", 0);
    cfg.user2 = prefs.getUChar("user2", 0);
    cfg.user3 = prefs.getUChar("user3", 0);
    cfg.user4 = prefs.getUChar("user4", 0);
    if (prefs.getBytesLength("pins") == 24) prefs.getBytes("pins", cfg.pin, 24);
    prefs.end();

    // never let a bad value through
    if (cfg.apSsid.length() < 1 || cfg.apSsid.length() > 32) cfg.apSsid = defaultApSsid();
    if (cfg.apPass.length() < 8 || cfg.apPass.length() > 63) cfg.apPass = DEFAULT_AP_PASS;
    if (cfg.host.length() < 1 || cfg.host.length() > 24)    cfg.host = DEFAULT_HOST;
    if (cfg.ipLast < 2 || cfg.ipLast > 254) cfg.ipLast = d.ipLast;
    if (cfg.firstSection > MAX_SECTIONS) cfg.firstSection = d.firstSection;
    for (uint8_t i = 0; i < 24; i++) if (cfg.pin[i] > PIN_GEO_STOP) cfg.pin[i] = 0;
}

void settingsSave()
{
    prefs.begin("seeder", false);
    prefs.putString("apSsid", cfg.apSsid);
    prefs.putString("apPass", cfg.apPass);
    prefs.putBool("staOn", cfg.staOn);
    prefs.putString("staSsid", cfg.staSsid);
    prefs.putString("staPass", cfg.staPass);
    prefs.putString("host", cfg.host);
    prefs.putBytes("subnet", cfg.subnet, 3);
    prefs.putUChar("ipLast", cfg.ipLast);
    prefs.putUChar("first", cfg.firstSection);
    prefs.putUChar("raise", cfg.raiseTime);
    prefs.putUChar("lower", cfg.lowerTime);
    prefs.putUChar("hyd", cfg.hydEnable);
    prefs.putUChar("invert", cfg.invertOutputs);
    prefs.putUChar("user1", cfg.user1);
    prefs.putUChar("user2", cfg.user2);
    prefs.putUChar("user3", cfg.user3);
    prefs.putUChar("user4", cfg.user4);
    prefs.putBytes("pins", cfg.pin, 24);
    prefs.end();
}

// Only the Wi-Fi settings and the host name go back to the defaults (a forgotten password); the
// program's config, the output functions, the module IP and the relays setting stay
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
