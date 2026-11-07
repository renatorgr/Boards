/* zOTA.ino - firmware update from the web page.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Written 2026 by Claude (Anthropic) for renatorgr.
   Uses FlasherX by Joe Pasquariello (FlashTxx.c / FlashTxx.h, public domain) for the flash.
   The history of changes is in the README changelog and in the git log.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// zOTA.ino
// Firmware update from the web page (zWeb.ino), without opening the box: the browser sends the
// .hex file made by the Arduino IDE ("Sketch > Export Compiled Binary"), the Teensy checks it,
// keeps it in a free part of its flash (the "buffer", 4-6 MB) and only then copies it over the
// running firmware (FlasherX flash_move) and restarts. The same .hex can come over the XIAO link
// (zXiao.ino, PGN 145) when the module's own page cannot be reached: the XIAO page feeds the same functions.
//
// The previous firmware (2 Nov 2026): just before the new image is copied over the running one, the running
// one is copied to another free part of the flash (2-4 MB: one header sector, then the image), with a
// checksum. "Go back to the previous firmware" on the Firmware tab or the XIAO page puts it back the same way,
// without a file, internet or USB. flash_move() leaves that part alone when it erases the rest of the flash
// (flash_keep_start / flash_keep_end, FlashTxx.c); going back erases it (the copy is consumed).
//
// Flash layout (Teensy 4.1, 8 MB): [0-2 MB firmware][2-4 MB previous firmware][4-6 MB update buffer][6 MB-...
// free][top 256 KB: EEPROM emulation, FLASH_RESERVE].
//
// Safety:
//   - nothing is written over the running firmware until the whole file has arrived and passed
//     every check: Intel HEX checksums, end-of-file record, starts at the flash base, fits the
//     buffer, contains "fw_teensy41" (built for a Teensy 4.1) and OTA_MARKER (it is this firmware family,
//     with this web page - so the next update can also be done from the web page)
//   - each flash sector written to the buffer is read back and compared
//   - steering is switched off before receiving (and an update is refused while autosteer is
//     engaged or the tractor is moving - zWeb.ino)
//   - the EEPROM (AgOpenGPS steer settings, web settings) is kept: FLASH_RESERVE in FlashTxx.h
//   - the only risky moment is the final copy (a few seconds): a power cut THEN leaves the Teensy
//     without firmware, and it must be reprogrammed over USB (the Teensy itself is not damaged).

extern "C" {
#include "FlashTxx.h"
}

#define OTA_BUFFER_ADDR   (FLASH_BASE_ADDR + 0x400000)   // 4 MB into the flash
#define OTA_BUFFER_SIZE   (0x200000)                     // 2 MB (the firmware is ~0.4 MB)
// The firmware family marker. The RUNNING firmware refuses a .hex that does not contain the marker it
// knows, so the name must stay the same (a new name needs a transition release that carries both: the name
// before 22 Oct 2026, "AOG-UM982-TM171-WEB-OTA-1", was carried until 2 Nov 2026 and dropped on 3 Nov 2026;
// a module still on a firmware from before 22 Oct 2026 takes a build of 22 Oct to 2 Nov first, or USB).
#define OTA_MARKER        "AiO-WEB-OTA-1"                   // must be in every version that updates another from the web page

// Kept in the image (printed in /api/status) so a new firmware of this family can be recognised
const char otaMarker[]  = OTA_MARKER;
const char otaFlashId[] = FLASH_ID;

extern unsigned long _flashimagelen;                     // size of the running firmware (linker)
#ifndef OTA_IMAGE_LEN
#define OTA_IMAGE_LEN     ((uint32_t)&_flashimagelen)
#endif

struct OtaState
{
    uint32_t    base;           // extended linear address (record 04)
    uint32_t    minAddr, maxAddr;
    int32_t     sector;         // buffer sector held in "buf" (-1 none)
    int32_t     lastWritten;    // last buffer sector written to flash (-1 none)
    bool        eof;
    uint32_t    lines, dataBytes;
    const char* err;
    uint16_t    lineLen;
    char        line[600];
    uint8_t     buf[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));
};
static OtaState ota;

static int hexNib(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int hexByte(const char* p)
{
    const int h = hexNib(p[0]), l = hexNib(p[1]);
    return (h < 0 || l < 0) ? -1 : (h << 4) | l;
}

// Erase (if needed) one 4 KB sector of the buffer
static bool otaEraseSector(int32_t sector)
{
    const uint32_t addr = OTA_BUFFER_ADDR + (uint32_t)sector * FLASH_SECTOR_SIZE;
    if (flash_sector_not_erased(addr)) eepromemu_flash_erase_sector((void*)addr);
    return !flash_sector_not_erased(addr);
}

// Write the sector held in ota.buf to the buffer and read it back
static bool otaFlushSector()
{
    if (ota.sector < 0) return true;
    const uint32_t addr = OTA_BUFFER_ADDR + (uint32_t)ota.sector * FLASH_SECTOR_SIZE;
    if (!otaEraseSector(ota.sector)) { ota.err = "flash erase failed"; return false; }
    for (uint32_t off = 0; off < FLASH_SECTOR_SIZE; off += 256)      // flash page = 256 bytes
    {
        bool blank = true;
        for (uint32_t i = 0; i < 256; i++) if (ota.buf[off + i] != 0xFF) { blank = false; break; }
        if (!blank) eepromemu_flash_write((void*)(addr + off), ota.buf + off, 256);
    }
    if (memcmp((const void*)addr, ota.buf, FLASH_SECTOR_SIZE) != 0) { ota.err = "flash verify failed"; return false; }
    ota.lastWritten = ota.sector;
    ota.sector = -1;
    return true;
}

// Put one data byte at absolute flash address "a" into the image
static bool otaPutByte(uint32_t a, uint8_t v)
{
    if (a < FLASH_BASE_ADDR || a >= FLASH_BASE_ADDR + OTA_BUFFER_SIZE) { ota.err = "address outside the Teensy 4.1 program flash"; return false; }
    const uint32_t off = a - FLASH_BASE_ADDR;
    const int32_t  sec = (int32_t)(off / FLASH_SECTOR_SIZE);
    if (sec != ota.sector)
    {
        if (sec < ota.sector || sec <= ota.lastWritten) { ota.err = "hex records out of order"; return false; }
        if (!otaFlushSector()) return false;
        // sectors with no data between the last one and this one must be blank in the buffer
        for (int32_t s = ota.lastWritten + 1; s < sec; s++)
            if (!otaEraseSector(s)) { ota.err = "flash erase failed"; return false; }
        ota.sector = sec;
        memset(ota.buf, 0xFF, sizeof(ota.buf));
    }
    ota.buf[off % FLASH_SECTOR_SIZE] = v;
    if (a < ota.minAddr) ota.minAddr = a;
    if (a + 1 > ota.maxAddr) ota.maxAddr = a + 1;
    return true;
}

// One Intel HEX line ":LLAAAATT<data>CC"
static bool otaLine(const char* l, uint16_t n)
{
    if (n == 0) return true;
    if (l[0] != ':' || n < 11) { ota.err = "not an Intel HEX file"; return false; }
    if (ota.eof) return true;                                  // ignore anything after the end record
    const int len = hexByte(l + 1);
    if (len < 0 || n < (uint16_t)(11 + 2 * len)) { ota.err = "bad hex line"; return false; }
    // count, address (2), type, data (len) and the checksum byte must add up to 0
    uint8_t sum = 0;
    for (int i = 0; i < 5 + len; i++)
    {
        const int b = hexByte(l + 1 + 2 * i);
        if (b < 0) { ota.err = "bad hex digit"; return false; }
        sum += (uint8_t)b;
    }
    if (sum != 0) { ota.err = "hex checksum error"; return false; }
    const uint32_t addr = ((uint32_t)hexByte(l + 3) << 8) | (uint32_t)hexByte(l + 5);
    const int      type = hexByte(l + 7);
    ota.lines++;
    switch (type)
    {
        case 0:                                                 // data
            for (int i = 0; i < len; i++)
                if (!otaPutByte(ota.base + addr + i, (uint8_t)hexByte(l + 9 + 2 * i))) return false;
            ota.dataBytes += len;
            return true;
        case 1: ota.eof = true; return true;                    // end of file
        case 4:                                                 // extended linear address
            if (len != 2) { ota.err = "bad hex address record"; return false; }
            ota.base = ((uint32_t)hexByte(l + 9) << 24) | ((uint32_t)hexByte(l + 11) << 16);
            return true;
        case 3: case 5: return true;                            // start address: not needed
        default: ota.err = "unsupported hex record"; return false;
    }
}

// Search the buffer for any string (like FlasherX check_flash_id)
static bool check_flash_id_str(uint32_t buffer, uint32_t size, const char* id)
{
    const size_t n = strlen(id);
    for (uint32_t i = buffer; i + n <= buffer + size; ++i)
        if (memcmp((const void*)i, id, n) == 0) return true;
    return false;
}

// Quiet the peripherals whose interrupts could touch flash constants during a copy
static void otaQuietIrqs()
{
    NVIC_DISABLE_IRQ(IRQ_ENET);
    NVIC_DISABLE_IRQ(IRQ_USB1);
    NVIC_DISABLE_IRQ(IRQ_LPUART2);    // Serial3 (XBee)
    NVIC_DISABLE_IRQ(IRQ_LPUART8);    // Serial5 (TM171)
    NVIC_DISABLE_IRQ(IRQ_LPUART7);    // Serial7 (GNSS)
    NVIC_DISABLE_IRQ(IRQ_LPUART4);    // Serial2 (GPS2 port: XIAO ESP32 or second F9P)
    NVIC_DISABLE_IRQ(IRQ_CAN3);
    NVIC_DISABLE_IRQ(IRQ_PIT);
}

// ---------------------------------------------------------------- the previous firmware
#define OTA_PREV_ADDR     (FLASH_BASE_ADDR + 0x200000)   // 2 MB into the flash: the header sector
#define OTA_PREV_SIZE     (0x200000)                     // 2 MB with the header, up to the update buffer
#define OTA_PREV_IMAGE    (OTA_PREV_ADDR + FLASH_SECTOR_SIZE)
#define OTA_PREV_MAGIC    0x50564F41u                    // "AOVP"

struct OtaPrevHead                 // the first bytes of the header sector; the rest of the sector is 0xFF
{
    uint32_t magic;                // OTA_PREV_MAGIC
    uint32_t size;                 // bytes of the image, whole sectors
    uint32_t crc;                  // CRC-32 of the image (crc32Flash)
    char     fw[40];               // its name and version, as /api/status "fw"
    char     built[24];            // when it was compiled, as /api/status "built"
};
static OtaPrevHead prevHead;       // read at start-up (otaSetup): magic 0 = no previous firmware kept
static bool        prevOk = false;

// CRC-32 (IEEE, as zip) of a range of flash, bit by bit: 0.7 MB takes about 20 ms
static uint32_t crc32Flash(uint32_t addr, uint32_t len)
{
    uint32_t c = 0xFFFFFFFFu;
    const uint8_t* p = (const uint8_t*)addr;
    for (uint32_t i = 0; i < len; i++)
    {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

// Erase whole sectors from addr (sector aligned) for len bytes; false when a sector would not erase
static bool flashEraseRange(uint32_t addr, uint32_t len)
{
    for (uint32_t a = addr; a < addr + len; a += FLASH_SECTOR_SIZE)
    {
        if (flash_sector_not_erased(a)) eepromemu_flash_erase_sector((void*)a);
        if (flash_sector_not_erased(a)) return false;
    }
    return true;
}

// Copy len bytes (whole pages) of flash from src to dst (erased), page by page through RAM, then compare
static bool flashCopy(uint32_t dst, uint32_t src, uint32_t len)
{
    static uint8_t page[256] __attribute__((aligned(4)));
    for (uint32_t off = 0; off < len; off += sizeof(page))
    {
        memcpy(page, (const void*)(src + off), sizeof(page));
        bool blank = true;
        for (uint32_t i = 0; i < sizeof(page); i++) if (page[i] != 0xFF) { blank = false; break; }
        if (!blank) eepromemu_flash_write((void*)(dst + off), page, sizeof(page));
    }
    return memcmp((const void*)dst, (const void*)src, len) == 0;
}

static bool otaTextOk(const char* s, size_t cap)      // a short, printable, 0-ended text in the header
{
    for (size_t i = 0; i < cap; i++)
    {
        if (s[i] == '\0') return i > 0;
        if ((uint8_t)s[i] < 0x20 || (uint8_t)s[i] > 0x7E) return false;
    }
    return false;
}

// The running firmware to the previous-firmware area, header last (a copy that stopped half way is never
// taken: the header sector is erased first). Called by otaInstall() once the new image has passed every
// check, just before it is copied: the loop is stopped and the watchdog is on its long timeout. About 10 s.
static void otaPrevSave()
{
    const uint32_t len = (OTA_IMAGE_LEN + FLASH_SECTOR_SIZE - 1) & ~(uint32_t)(FLASH_SECTOR_SIZE - 1);
    if (len < 16384 || len + FLASH_SECTOR_SIZE > OTA_PREV_SIZE || FLASH_BASE_ADDR + len > OTA_PREV_ADDR) return;
    if (!flashEraseRange(OTA_PREV_ADDR, FLASH_SECTOR_SIZE)) return;
    if (!flashEraseRange(OTA_PREV_IMAGE, len)) return;
    if (!flashCopy(OTA_PREV_IMAGE, FLASH_BASE_ADDR, len)) return;
    static uint8_t page[256] __attribute__((aligned(4)));
    memset(page, 0xFF, sizeof(page));
    OtaPrevHead h;
    memset(&h, 0, sizeof(h));
    h.magic = OTA_PREV_MAGIC;
    h.size  = len;
    h.crc   = crc32Flash(OTA_PREV_IMAGE, len);
    strncpy(h.fw, FW_NAME " " FW_VERSION, sizeof(h.fw) - 1);
    strncpy(h.built, __DATE__ " " __TIME__, sizeof(h.built) - 1);
    memcpy(page, &h, sizeof(h));
    eepromemu_flash_write((void*)OTA_PREV_ADDR, page, sizeof(page));
}

// At start-up (setup, after the web page): is a previous firmware kept? One [boot] line when it is
void otaSetup()
{
    memcpy(&prevHead, (const void*)OTA_PREV_ADDR, sizeof(prevHead));
    if (prevHead.magic != OTA_PREV_MAGIC) { prevHead.magic = 0; return; }
    prevOk = prevHead.size >= 16384 && prevHead.size + FLASH_SECTOR_SIZE <= OTA_PREV_SIZE &&
             (prevHead.size % FLASH_SECTOR_SIZE) == 0 &&
             otaTextOk(prevHead.fw, sizeof(prevHead.fw)) && otaTextOk(prevHead.built, sizeof(prevHead.built)) &&
             crc32Flash(OTA_PREV_IMAGE, prevHead.size) == prevHead.crc;
    if (prevOk) StatusLog.line("[boot] Previous firmware kept in the flash: %s, built %s (Firmware tab or the XIAO page: go back to it)", prevHead.fw, prevHead.built);
    else      { StatusLog.line("[boot] Previous firmware kept in the flash: damaged (checksum), not offered"); prevHead.magic = 0; }
}

// For /api/status (zWeb.ino): the previous firmware kept, or false when there is none
bool otaPrevGet(const char** fw, const char** built, uint32_t* size)
{
    if (!prevOk) return false;
    *fw = prevHead.fw;
    *built = prevHead.built;
    *size = prevHead.size;
    return true;
}

// Can the previous firmware be put back now? nullptr = yes (the callers check autosteer and the speed first)
const char* otaRevertCheck()
{
    if (!prevOk) return "no previous firmware kept (it is kept from the next update on)";
    if (crc32Flash(OTA_PREV_IMAGE, prevHead.size) != prevHead.crc) { prevOk = false; return "the kept firmware is damaged (checksum) - not installed"; }
    if (!check_flash_id(OTA_PREV_IMAGE, prevHead.size))            return "the kept firmware is not a Teensy 4.1 firmware";
    return nullptr;
}

// Copy the previous firmware over the running one and restart; the rest of the flash is erased (no previous
// firmware after this). Does not return.
void otaRevertInstall()
{
    Serial.flush();
    wdtLongTimeout();
    otaQuietIrqs();
    flash_keep_start = flash_keep_end = 0;
    flashEraseRange(OTA_PREV_ADDR, FLASH_SECTOR_SIZE);               // the header first: no half state
    flash_move(FLASH_BASE_ADDR, OTA_PREV_IMAGE, prevHead.size);        // erases the rest afterwards, reboots
    for (;;) {}
}

// ---------------------------------------------------------------- interface for zWeb.ino
// Can an update be stored now? (the running firmware must end below the buffer)
const char* otaBegin()
{
    memset(&ota, 0, offsetof(OtaState, buf));
    ota.minAddr = 0xFFFFFFFF;
    ota.sector = ota.lastWritten = -1;
    if (FLASH_BASE_ADDR + OTA_IMAGE_LEN + FLASH_SECTOR_SIZE > OTA_BUFFER_ADDR) ota.err = "running firmware too large for the update buffer";
    return ota.err;
}

// Feed the bytes of the .hex file as they arrive
const char* otaFeed(const uint8_t* d, size_t n)
{
    for (size_t i = 0; i < n && !ota.err; i++)
    {
        const char c = (char)d[i];
        if (c == '\n' || c == '\r')
        {
            if (!otaLine(ota.line, ota.lineLen)) break;
            ota.lineLen = 0;
        }
        else if (ota.lineLen < sizeof(ota.line) - 1) ota.line[ota.lineLen++] = c;
        else ota.err = "hex line too long";
    }
    return ota.err;
}

// After the last byte: every check. Returns nullptr when the image may be installed;
// *size = image size in bytes.
const char* otaFinish(bool force, uint32_t* size)
{
    if (!ota.err && ota.lineLen)                                  // last line without a line end
    {
        if (!otaLine(ota.line, ota.lineLen)) ota.err = "file incomplete (cut off in the middle of a line)";
        ota.lineLen = 0;
    }
    if (!ota.err) otaFlushSector();
    if (ota.err) return ota.err;
    if (!ota.eof)                                    return ota.err = "file incomplete (no end record)";
    if (ota.dataBytes < 16384)                       return ota.err = "file too small to be a Teensy firmware";
    if (ota.minAddr != FLASH_BASE_ADDR)              return ota.err = "image does not start at the Teensy 4.1 flash base";
    const uint32_t len = ota.maxAddr - FLASH_BASE_ADDR;
    if (!check_flash_id(OTA_BUFFER_ADDR, len))       return ota.err = "not built for a Teensy 4.1 (fw_teensy41 missing)";
    if (!force && !check_flash_id_str(OTA_BUFFER_ADDR, len, OTA_MARKER))
        return ota.err = "not this firmware (no web update in it) - tick 'other firmware' to install it anyway";
    *size = len;
    return nullptr;
}

// Copy the checked image over the running firmware and restart. Does not return.
// First the running firmware goes to the previous-firmware area (otaPrevSave, about 10 s), which flash_move
// then leaves alone while it erases the rest of the flash.
void otaInstall(uint32_t size)
{
    Serial.flush();
    wdtLongTimeout();                 // the copies take seconds - watchdog timeout 128 s (zSafety.ino)
    otaQuietIrqs();
    otaPrevSave();
    flash_keep_start = OTA_PREV_ADDR;
    flash_keep_end   = OTA_PREV_ADDR + OTA_PREV_SIZE;
    flash_move(FLASH_BASE_ADDR, OTA_BUFFER_ADDR, size);   // erases the buffer afterwards, reboots
    for (;;) {}
}
