#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <Adafruit_PN532.h>
#include <RF24.h>

// ─── OLED SPI ────────────────────────────────────────────────────────────────
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, /*SCK*/12, /*MOSI*/11, /*CS*/8, /*DC*/9, /*RES*/10);

// ─── Gombok ──────────────────────────────────────────────────────────────────
#define BTN_UP    4
#define BTN_DOWN  5
#define BTN_ENTER 38

// ─── Buzzer ──────────────────────────────────────────────────────────────────
#define BUZZER_PIN 21

// ─── IR ──────────────────────────────────────────────────────────────────────
#define IR_TX_PIN 47
bool irTxActive = false;

// ─── PN532 I2C ───────────────────────────────────────────────────────────────
#define PN532_SDA 18
#define PN532_SCL 17
Adafruit_PN532 nfc(2, 3);

// ─── NRF24 SPI ───────────────────────────────────────────────────────────────
#define NRF_CE   7
#define NRF_CSN  6
#define NRF_MOSI 11
#define NRF_MISO 13
#define NRF_SCK  12
SPIClass* hspi = nullptr;
RF24 radio(NRF_CE, NRF_CSN);

// ─── Kontraszt ───────────────────────────────────────────────────────────────
int oledContrast = 200;

// ─── NFC napló ───────────────────────────────────────────────────────────────
#define MAX_NFC_LOGS 20
String nfcLogs[MAX_NFC_LOGS];
int nfcLogCount = 0, nfcLogHead = 0;

void addNfcLog(const String& uid) {
  nfcLogs[nfcLogHead] = uid;
  nfcLogHead = (nfcLogHead + 1) % MAX_NFC_LOGS;
  if (nfcLogCount < MAX_NFC_LOGS) nfcLogCount++;
}
String getNfcLog(int i) {
  int start = (nfcLogHead - nfcLogCount + MAX_NFC_LOGS) % MAX_NFC_LOGS;
  return nfcLogs[(start + i) % MAX_NFC_LOGS];
}

// ─── Scanner ─────────────────────────────────────────────────────────────────
#define NUM_CH  126
#define SAMPLES  20
uint8_t  spectrum[NUM_CH], peak[NUM_CH];
uint32_t peakTimer[NUM_CH];
bool     scannerReady = false;  // NEM static, reset-elhető kívülről

enum ScanMode { MODE_BOTH, MODE_LIVE, MODE_PEAK };
ScanMode scanMode = MODE_BOTH;
const char* modeNames[] = { "Live+Peak", "Live", "Peak" };

// ─── Navigáció ───────────────────────────────────────────────────────────────
enum Screen { SCR_MAIN, SCR_NFC, SCR_SCANNER, SCR_IR, SCR_SETTINGS };
Screen currentScreen = SCR_MAIN;

const char* mainMenu[]     = { "NFC Menu", "2.4GHz Scanner", "IR Tools", "Settings", "About" };
const char* nfcMenu[]      = { "Read Card", "View Logs", "Clear Logs", "Back" };
const char* irMenu[]       = { "IR Jammer", "38kHz Toggle", "Short Pulse", "Back" };
const char* settingsMenu[] = { "Contrast Adjust", "Back" };

int curMain = 0, curNfc = 0, curIr = 0, curSettings = 0;

// ─── Elődeklarációk ──────────────────────────────────────────────────────────
void beep(uint16_t freq, uint16_t dur);
void showMessage(const char* l1, const char* l2, int ms);
void drawMenu(const char** items, int count, int sel, const char* title);
void drawMainMenu(); void drawNfcMenu(); void drawIrMenu(); void drawSettingsMenu();
void handleUp(); void handleDown(); void handleEnter();
void bootAnimation();
void readNFC(); void viewNfcLogs(); void clearNfcLogs();
void runScanner(); void scanChannels(); void drawSpectrum();
void irJammer(); void irToggle(); void irShortPulse();
void adjustContrast(); void drawContrastBar();
void waitAllReleased();

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BTN_UP,     INPUT_PULLUP);
  pinMode(BTN_DOWN,   INPUT_PULLUP);
  pinMode(BTN_ENTER,  INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(IR_TX_PIN,  OUTPUT); digitalWrite(IR_TX_PIN,  LOW);

  u8g2.begin();
  u8g2.setContrast(oledContrast);
  bootAnimation();

  Wire.begin(PN532_SDA, PN532_SCL);
  Wire.setClock(400000);
  nfc.begin();
  if (!nfc.getFirmwareVersion()) {
    showMessage("PN532 ERROR!", "Check wiring", 2000);
    beep(400, 150);
  } else {
    nfc.SAMConfig();
  }

  hspi = new SPIClass(HSPI);
  hspi->begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);
  if (radio.begin(hspi)) {
    radio.setAutoAck(false);
    radio.disableCRC();
    radio.setPayloadSize(1);
    radio.setAddressWidth(2);
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();
    memset(spectrum,  0, sizeof(spectrum));
    memset(peak,      0, sizeof(peak));
    memset(peakTimer, 0, sizeof(peakTimer));
  } else {
    showMessage("NRF24 ERROR!", "Check wiring", 2000);
  }

  // OLED újrainicializálás HSPI után
  u8g2.begin();
  u8g2.setContrast(oledContrast);

  showMessage("Ready!", nullptr, 800);
  beep(1100, 80); delay(40); beep(1400, 60); delay(30); beep(1800, 40);
  drawMainMenu();
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  if (currentScreen == SCR_SCANNER) { runScanner(); return; }

  // Folyamatos 38kHz IR ha be van kapcsolva
  if (irTxActive) irPulse(13, 5000);  // 5ms burst, majd gombokat ellenőriz
  else digitalWrite(IR_TX_PIN, LOW);

  static bool lastUp = HIGH, lastDown = HIGH, lastEnter = HIGH;
  bool up    = digitalRead(BTN_UP);
  bool down  = digitalRead(BTN_DOWN);
  bool enter = digitalRead(BTN_ENTER);

  if (up    == LOW && lastUp    == HIGH) { handleUp();    delay(170); }
  if (down  == LOW && lastDown  == HIGH) { handleDown();  delay(170); }
  if (enter == LOW && lastEnter == HIGH) { handleEnter(); delay(170); }

  lastUp = up; lastDown = down; lastEnter = enter;
  delay(8);
}

// ─── Segédfüggvények ─────────────────────────────────────────────────────────
void beep(uint16_t freq, uint16_t dur) {
  tone(BUZZER_PIN, freq, dur);
  delay(dur + 10);
  noTone(BUZZER_PIN);
}

void waitAllReleased() {
  while (digitalRead(BTN_UP) == LOW || digitalRead(BTN_DOWN) == LOW || digitalRead(BTN_ENTER) == LOW)
    delay(10);
  delay(80);
}

// ─── Boot animáció ───────────────────────────────────────────────────────────
void bootAnimation() {
  for (int p = 0; p <= 100; p += 5) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr); u8g2.drawStr(8, 18, "ESP32-S3");
    u8g2.setFont(u8g2_font_6x10_tr);   u8g2.drawStr(8, 30, "Grabber v1.2");
    u8g2.drawRFrame(8, 38, 112, 14, 3);
    int fill = (p * 108) / 100;
    if (fill > 0) u8g2.drawRBox(10, 40, fill, 10, 2);
    char buf[8]; snprintf(buf, sizeof(buf), "%d%%", p);
    u8g2.drawStr(56, 62, buf);
    u8g2.sendBuffer(); delay(40);
  }
  delay(200);
  u8g2.clearBuffer();
  u8g2.drawBox(0, 0, 128, 12); u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(28, 10, "System Info");
  u8g2.setDrawColor(1);
  char buf[32];
  snprintf(buf, sizeof(buf), "CPU : %u MHz",   ESP.getCpuFreqMHz());               u8g2.drawStr(4, 24, buf);
  snprintf(buf, sizeof(buf), "Heap: %u KB",    ESP.getFreeHeap() / 1024);           u8g2.drawStr(4, 36, buf);
  snprintf(buf, sizeof(buf), "Flash: %u MB",   ESP.getFlashChipSize()/(1024*1024)); u8g2.drawStr(4, 48, buf);
  u8g2.drawHLine(0, 52, 128);
  u8g2.setFont(u8g2_font_ncenB08_tr); u8g2.drawStr(10, 63, "NFC + 2.4GHz + IR");
  u8g2.sendBuffer(); delay(1600);
}

// ─── Menürajzoló ─────────────────────────────────────────────────────────────
void drawMenu(const char** items, int count, int sel, const char* title) {
  u8g2.clearBuffer();
  int yStart = 2;
  if (title) {
    u8g2.drawBox(0, 0, 128, 13); u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(4, 10, title);
    u8g2.setDrawColor(1); yStart = 15;
  }
  u8g2.setFont(u8g2_font_ncenB08_tr);
  const int ROW = 13, VIS = 4;
  int scroll = max(0, min(sel - VIS + 1, count - VIS));
  for (int i = 0; i < VIS && (scroll + i) < count; i++) {
    int idx = scroll + i, y = yStart + i * ROW;
    if (idx == sel) { u8g2.drawRBox(0, y, 128, ROW, 2); u8g2.setDrawColor(0); }
    else u8g2.setDrawColor(1);
    u8g2.drawStr(6, y + ROW - 3, items[idx]);
  }
  u8g2.setDrawColor(1);
  if (scroll > 0)          u8g2.drawTriangle(122, 15, 127, 15, 124, 11);
  if (scroll+VIS < count)  u8g2.drawTriangle(122, 60, 127, 60, 124, 64);
  u8g2.sendBuffer();
}

void drawMainMenu()     { drawMenu(mainMenu,     5, curMain,     nullptr);          }
void drawNfcMenu()      { drawMenu(nfcMenu,      4, curNfc,      "NFC");            }
void drawIrMenu()       { drawMenu(irMenu,       4, curIr,       "IR Tools");       }
void drawSettingsMenu() { drawMenu(settingsMenu, 2, curSettings, "Settings");       }

// ─── Gombkezelők ─────────────────────────────────────────────────────────────
void handleUp() {
  switch (currentScreen) {
    case SCR_MAIN:     curMain     = (curMain     - 1 + 5) % 5; drawMainMenu();     break;
    case SCR_NFC:      curNfc      = (curNfc      - 1 + 4) % 4; drawNfcMenu();      break;
    case SCR_IR:       curIr       = (curIr       - 1 + 4) % 4; drawIrMenu();       break;
    case SCR_SETTINGS: curSettings = (curSettings - 1 + 2) % 2; drawSettingsMenu(); break;
    default: break;
  }
}
void handleDown() {
  switch (currentScreen) {
    case SCR_MAIN:     curMain     = (curMain     + 1) % 5; drawMainMenu();     break;
    case SCR_NFC:      curNfc      = (curNfc      + 1) % 4; drawNfcMenu();      break;
    case SCR_IR:       curIr       = (curIr       + 1) % 4; drawIrMenu();       break;
    case SCR_SETTINGS: curSettings = (curSettings + 1) % 2; drawSettingsMenu(); break;
    default: break;
  }
}
void handleEnter() {
  switch (currentScreen) {
    case SCR_MAIN:
      switch (curMain) {
        case 0: currentScreen = SCR_NFC;      curNfc = 0;      drawNfcMenu();      break;
        case 1:
          scannerReady = false;  // reset belépés előtt!
          currentScreen = SCR_SCANNER;
          break;
        case 2: currentScreen = SCR_IR;       curIr = 0;       drawIrMenu();       break;
        case 3: currentScreen = SCR_SETTINGS; curSettings = 0; drawSettingsMenu(); break;
        case 4: showMessage("ESP32-S3 Grabber", "v1.2 NFC+RF+IR", 2500); drawMainMenu(); break;
      } break;
    case SCR_NFC:
      switch (curNfc) {
        case 0: readNFC();    break;
        case 1: viewNfcLogs(); break;
        case 2: clearNfcLogs(); showMessage("Logs cleared!", nullptr, 1200); drawNfcMenu(); break;
        case 3: currentScreen = SCR_MAIN; drawMainMenu(); break;
      } break;
    case SCR_IR:
      switch (curIr) {
        case 0: irJammer();    break;
        case 1: irToggle();    break;
        case 2: irShortPulse(); break;
        case 3: currentScreen = SCR_MAIN; drawMainMenu(); break;
      } break;
    case SCR_SETTINGS:
      switch (curSettings) {
        case 0: adjustContrast(); break;
        case 1: currentScreen = SCR_MAIN; drawMainMenu(); break;
      } break;
    default: break;
  }
}

// ─── NFC ─────────────────────────────────────────────────────────────────────
void readNFC() {
  showMessage("Hold card near", "reader...", 0);
  uint8_t uid[7], uidLen;
  bool ok = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 1500);
  u8g2.clearBuffer();
  if (ok) {
    String s = "";
    for (uint8_t i = 0; i < uidLen; i++) {
      if (i) s += ':';
      if (uid[i] < 0x10) s += '0';
      s += String(uid[i], HEX);
    }
    s.toUpperCase();
    addNfcLog(s);
    beep(1800, 60); delay(30); beep(2200, 50);
    u8g2.drawBox(0, 0, 128, 13); u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(4, 10, " Card detected!");
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_6x10_tr);    u8g2.drawStr(4, 26, "UID:");
    u8g2.setFont(u8g2_font_ncenB08_tr); u8g2.drawStr(4, 40, s.c_str());
    char info[28]; snprintf(info, sizeof(info), "Type:%ubyte  #%d saved", uidLen, nfcLogCount);
    u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(4, 56, info);
  } else {
    beep(400, 150);
    u8g2.setFont(u8g2_font_ncenB10_tr); u8g2.drawStr(14, 28, "No card found");
    u8g2.setFont(u8g2_font_6x10_tr);    u8g2.drawStr(24, 46, "Try again...");
  }
  u8g2.sendBuffer();
  delay(3000);
  drawNfcMenu();
}

void viewNfcLogs() {
  if (nfcLogCount == 0) { showMessage("No logs yet!", nullptr, 1800); drawNfcMenu(); return; }
  int scroll = max(0, nfcLogCount - 4);
  bool active = true;
  while (active) {
    u8g2.clearBuffer();
    u8g2.drawBox(0, 0, 128, 13); u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_6x10_tr);
    char hdr[22]; snprintf(hdr, sizeof(hdr), "Logs %d/%d  ENT=exit", nfcLogCount, MAX_NFC_LOGS);
    u8g2.drawStr(2, 10, hdr); u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_5x8_tr);
    for (int i = 0; i < 4 && (scroll + i) < nfcLogCount; i++) {
      char buf[28]; snprintf(buf, sizeof(buf), "#%02d %s", scroll+i+1, getNfcLog(scroll+i).c_str());
      u8g2.drawStr(2, 23 + i * 11, buf);
    }
    if (scroll > 0)             u8g2.drawTriangle(122, 15, 127, 15, 124, 11);
    if (scroll+4 < nfcLogCount) u8g2.drawTriangle(122, 60, 127, 60, 124, 64);
    u8g2.sendBuffer();
    unsigned long t = millis();
    while (millis() - t < 10000) {
      if (digitalRead(BTN_UP)    == LOW) { scroll = max(0, scroll-1);                    delay(160); break; }
      if (digitalRead(BTN_DOWN)  == LOW) { scroll = min(max(0,nfcLogCount-4), scroll+1); delay(160); break; }
      if (digitalRead(BTN_ENTER) == LOW) { active = false;                                delay(180); break; }
      delay(8);
    }
    if (millis() - t >= 10000) active = false;
  }
  drawNfcMenu();
}

void clearNfcLogs() {
  nfcLogCount = 0; nfcLogHead = 0;
  for (int i = 0; i < MAX_NFC_LOGS; i++) nfcLogs[i] = "";
}

// ─── IR Tools ────────────────────────────────────────────────────────────────

// IR JAMMER: közvetlen GPIO toggle – tone() nem bír 40kHz felett ESP32-S3-on
// Különböző periódusidők = különböző frekvenciák, megzavarja az IR vevőket
void irPulse(uint32_t halfPeriodUs, uint32_t durationUs) {
  uint32_t end = micros() + durationUs;
  while (micros() < end) {
    digitalWrite(IR_TX_PIN, HIGH);
    delayMicroseconds(halfPeriodUs);
    digitalWrite(IR_TX_PIN, LOW);
    delayMicroseconds(halfPeriodUs);
  }
}

void irJammer() {
  waitAllReleased();
  u8g2.clearBuffer();
  u8g2.drawBox(0, 0, 128, 13); u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(4, 10, "IR JAMMER");
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_ncenB10_tr); u8g2.drawStr(18, 36, "JAMMING...");
  u8g2.setFont(u8g2_font_6x10_tr);   u8g2.drawStr(14, 54, "ENTER = stop");
  u8g2.sendBuffer();
  beep(1600, 60); delay(20); beep(1900, 40);

  // halfPeriod értékek us-ban: 1000000/(freq*2)
  // 30kHz=17us, 33kHz=15us, 36kHz=14us, 38kHz=13us,
  // 40kHz=13us, 44kHz=11us, 47kHz=11us, 50kHz=10us, 56kHz=9us
  uint8_t periods[] = { 17, 15, 14, 13, 13, 11, 11, 10, 9 };
  int pCount = 9, pi = 0;

  while (digitalRead(BTN_ENTER) == HIGH) {
    // 3ms burst egy frekvencián
    irPulse(periods[pi], 3000);
    pi = (pi + 1) % pCount;
    // 1ms szünet – ez is zavar mert a vevő elveszíti a szinkront
    digitalWrite(IR_TX_PIN, LOW);
    delayMicroseconds(1000);
  }

  digitalWrite(IR_TX_PIN, LOW);
  waitAllReleased();
  beep(800, 100);
  showMessage("Jammer OFF", nullptr, 800);
  drawIrMenu();
}

void irToggle() {
  irTxActive = !irTxActive;
  if (irTxActive) {
    beep(1400, 60);
    showMessage("IR ON", "38kHz continuous", 800);
    // 38kHz folyamatos – a loop()-ban fut amíg ki nem kapcsolják
    // Jelzés hogy a loop kezelje
  } else {
    digitalWrite(IR_TX_PIN, LOW);
    beep(800, 80);
    showMessage("IR OFF", nullptr, 800);
  }
  drawIrMenu();
}

void irShortPulse() {
  showMessage("Short Pulse", "Sending...", 0);
  // 5x 38kHz burst, közvetlen GPIO toggle-lal (13us halfperiod = ~38kHz)
  for (int i = 0; i < 5; i++) {
    irPulse(13, 180000);  // 180ms burst
    digitalWrite(IR_TX_PIN, LOW);
    delay(120);
  }
  beep(1200, 50);
  delay(500);
  drawIrMenu();
}

// ─── 2.4GHz Scanner ──────────────────────────────────────────────────────────
void runScanner() {
  // Belépéskor várakozás amíg minden gomb felenged
  if (!scannerReady) {
    waitAllReleased();
    scannerReady = true;
    // Első scan + kirajzolás
    scanChannels();
    drawSpectrum();
    return;
  }

  // Gomb ellenőrzés
  if (digitalRead(BTN_ENTER) == LOW) {
    waitAllReleased();
    beep(900, 40); delay(30); beep(600, 60);
    scannerReady  = false;
    currentScreen = SCR_MAIN;
    radio.stopListening();
    drawMainMenu();
    return;
  }
  if (digitalRead(BTN_UP) == LOW) {
    scanMode = (ScanMode)((scanMode + 1) % 3);
    while (digitalRead(BTN_UP) == LOW) delay(10);
    beep(1200, 30);
  }
  if (digitalRead(BTN_DOWN) == LOW) {
    scanMode = (ScanMode)((scanMode + 2) % 3);
    while (digitalRead(BTN_DOWN) == LOW) delay(10);
    beep(1200, 30);
  }

  scanChannels();
  if (currentScreen == SCR_SCANNER) drawSpectrum();
}

void scanChannels() {
  radio.stopListening();
  for (int ch = 0; ch < NUM_CH; ch++) {
    // Kilépés ellenőrzés scan közben minden 15. csatornánál
    if (ch % 15 == 0 && digitalRead(BTN_ENTER) == LOW) {
      scannerReady  = false;
      currentScreen = SCR_MAIN;
      radio.stopListening();
      drawMainMenu();
      return;
    }
    radio.setChannel(ch);
    uint8_t hits = 0;
    for (int s = 0; s < SAMPLES; s++) {
      radio.startListening();
      delayMicroseconds(100);
      radio.stopListening();
      if (radio.testCarrier()) hits++;
    }
    hits = (hits * 50) / SAMPLES;
    spectrum[ch] = hits;
    if (hits >= peak[ch]) { peak[ch] = hits; peakTimer[ch] = millis(); }
    else if (millis() - peakTimer[ch] > 3000 && peak[ch] > 0) peak[ch]--;
  }
}

void drawSpectrum() {
  u8g2.clearBuffer();
  u8g2.drawBox(0, 0, 128, 12); u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_5x8_tr);
  char hdr[32]; snprintf(hdr, sizeof(hdr), "2.4GHz [%s] ENT=back", modeNames[scanMode]);
  u8g2.drawStr(2, 9, hdr); u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(0, 63, "2400"); u8g2.drawStr(48, 63, "2450"); u8g2.drawStr(96, 63, "2500");
  const int Y_BASE = 54, Y_TOP = 13, H = Y_BASE - Y_TOP;
  for (int ch = 0; ch < 126; ch++) {
    int x = ch + 1;
    if (scanMode == MODE_LIVE || scanMode == MODE_BOTH) {
      int h = (spectrum[ch] * H) / 50;
      if (h > 0) u8g2.drawVLine(x, Y_BASE - h, h);
    }
    if (scanMode == MODE_PEAK || scanMode == MODE_BOTH) {
      if (peak[ch] > 0) {
        int py = Y_BASE - (peak[ch] * H) / 50;
        u8g2.drawPixel(x, py);
        if (py > Y_TOP) u8g2.drawPixel(x, py - 1);
      }
    }
  }
  const int wCh[]    = { 12, 37, 62, 72 };
  const char* wLbl[] = { "1", "6", "11", "13" };
  u8g2.setFont(u8g2_font_4x6_tr);
  for (int i = 0; i < 4; i++) {
    for (int y = Y_TOP; y < Y_BASE; y += 2) u8g2.drawPixel(wCh[i]+1, y);
    u8g2.drawStr(wCh[i]-1, Y_TOP+8, wLbl[i]);
  }
  u8g2.sendBuffer();
}

// ─── Kontraszt ───────────────────────────────────────────────────────────────
void adjustContrast() {
  drawContrastBar();
  bool active = true;
  bool lastU = HIGH, lastD = HIGH, lastE = HIGH;
  while (active) {
    bool u = digitalRead(BTN_UP), d = digitalRead(BTN_DOWN), e = digitalRead(BTN_ENTER);
    bool changed = false;
    if (u == LOW && lastU == HIGH) { oledContrast = min(255, oledContrast+10); changed = true; delay(120); }
    if (d == LOW && lastD == HIGH) { oledContrast = max(0,   oledContrast-10); changed = true; delay(120); }
    if (e == LOW && lastE == HIGH) { active = false; delay(200); }
    if (changed) { u8g2.setContrast(oledContrast); drawContrastBar(); }
    lastU = u; lastD = d; lastE = e; delay(8);
  }
  drawSettingsMenu();
}

void drawContrastBar() {
  u8g2.clearBuffer();
  u8g2.drawBox(0, 0, 128, 13); u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(4, 10, "Contrast");
  u8g2.setDrawColor(1);
  char buf[8]; snprintf(buf, sizeof(buf), "%d", oledContrast);
  u8g2.setFont(u8g2_font_ncenB10_tr); u8g2.drawStr(50, 34, buf);
  u8g2.drawRFrame(4, 40, 120, 12, 3);
  int fill = (oledContrast * 116) / 255;
  if (fill > 0) u8g2.drawRBox(6, 42, fill, 8, 2);
  u8g2.setFont(u8g2_font_6x10_tr); u8g2.drawStr(4, 60, "UP/DN  ENTER=ok");
  u8g2.sendBuffer();
}

// ─── Üzenet ──────────────────────────────────────────────────────────────────
void showMessage(const char* l1, const char* l2, int ms) {
  u8g2.clearBuffer();
  u8g2.drawRFrame(2, 2, 124, 60, 4);
  if (l2) {
    u8g2.setFont(u8g2_font_ncenB10_tr); u8g2.drawStr(8, 26, l1);
    u8g2.drawHLine(8, 32, 112);
    u8g2.setFont(u8g2_font_6x10_tr);    u8g2.drawStr(8, 48, l2);
  } else {
    u8g2.setFont(u8g2_font_ncenB10_tr);
    int w = u8g2.getStrWidth(l1);
    u8g2.drawStr((128-w)/2, 36, l1);
  }
  u8g2.sendBuffer();
  if (ms > 0) delay(ms);
}
