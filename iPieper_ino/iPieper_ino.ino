/*
  iPieper software v1.0

  Te bouwen met:
    - Arduino IDE : open iPieper_ino.ino
    - VS Code     : open de map iPieper_ino (PlatformIO, zie platformio.ini)

  Compileert op zowel arduino-esp32 core 2.x als 3.x: de ledc-API in panic
  mode switcht automatisch via #if ESP_ARDUINO_VERSION_MAJOR >= 3.

  Instellingen voor een zo laag mogelijk stroomverbruik. In de Arduino IDE
  stel je deze in via het Tools-menu; bij PlatformIO staan ze al in
  platformio.ini. Bovendien zet setup() de klok zelf op 80 MHz, zodat de
  lage kloksnelheid in beide omgevingen gegarandeerd is:

  CPU Frequency                80MHz (laagste klok waarbij Bluetooth blijft werken)
  Flash frequency              40MHz
*/

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <SPI.h>
#include <JQ8400_Serial.h>
#include <EEPROM.h>
#include <BluetoothSerial.h>
#include <Update.h>
#include "mbedtls/base64.h"
#include "esp_bt.h"          // esp_bredr_tx_power_set()
#include "nvs_flash.h"       // nvs_flash_erase() voor #FACTORYRESET

BluetoothSerial SerialBT;
JQ8400_Serial mp3(Serial2);

bool                  bootEnd = false;
bool                  config = false;
bool                  ledState = false;
bool                  terrormode = false;
bool                  testmode = false;
bool                  txonoffstate = true;
byte                  mp3track = 0;
byte                  previousTrack = 255;
byte                  txoff = 0;
byte                  txon = 0;
char                  piepername[11];
byte                  audioMode = 0;        // 0 = MP3 via JQ8400, 1 = morse, 2 = Nokia ringtone
byte                  morseWpm = 6;         // 3..12 WPM (PARIS-norm)
char                  morseText[31];        // max 30 chars + null
byte                  morseAmp = 5;         // 1..10 stap; geldt voor morse EN ringtone
byte                  ringTrack = 1;        // 1..10, geselecteerde ringtone slot
char                  ringBuf[251];         // tijdelijke buffer voor slot-load; max == EE_RINGSLOT_SIZE
const unsigned long   bootTime = 120000;     // config-venster bij boot (2 min)
const unsigned long   longPressMs = 2000;     // drempel voor lang-indrukken (TX-kill)
String                inputBuffer = "";
unsigned int          frequency = 145000;
unsigned int          numberoffiles;
unsigned int          panictime = 0;
unsigned long         ledMillis = 0;
unsigned long         txonoffMillis = 0;

// OTA-state (BT-OTA via het #-protocol). otaActive=true zolang er een
// transfer loopt; tijdens die periode wordt de TX-cyclus uitgezet om
// flash-writes niet te storen.
bool                  otaActive = false;
unsigned long         otaBytesReceived = 0;
unsigned long         otaExpectedSize = 0;

typedef struct {
  unsigned int  index;
  char          name[120];
  unsigned int  length;
} FileLibrary;

FileLibrary file[50];

#define RF_TX                       4
#define pin_LE                      5
#define pin_Audio                   2
#define pin_Button                  0
#define pin_LED                     13

#define EE_TOTAL_CNT                2567   // 57 + 10*251 (RINGSLOTS). 10 slots is
                                            // de stabiele max binnen de standaard 20 KB
                                            // NVS-partitie van min_spiffs.csv. Meer
                                            // vereist een custom partition table met
                                            // grotere NVS - alleen via USB-flash.
#define EE_CHECKNUMBER              12

#define EE_UINT16_FREQUENCY         0
#define EE_UINT16_PANICTIME         4
#define EE_BYTE_TXOFF               8
#define EE_BYTE_TXON                9
#define EE_BYTE_MP3TRACK            10
#define EE_BYTE_TERRORMODE          11
#define EE_CHECK                    12
#define EE_BYTE_PIEPERNAME          13    // 10 posities
#define EE_BYTE_AUDIOMODE           23    // 0 = MP3 (JQ8400), 1 = morse, 2 = Nokia ringtone
#define EE_BYTE_MORSEWPM            24    // 3-12 WPM
#define EE_BYTE_MORSETEXT           25    // 30 posities (+ null)
#define EE_BYTE_MORSEAMP            55    // 1-10 stap (geldt voor morse EN ringtone)
#define EE_BYTE_RINGTRACK           56    // 1..10, geselecteerde ringtone slot
#define EE_BYTE_RINGSLOTS           57    // 10 slots * 251 bytes = 2510 bytes
                                          // elk slot: "tempo,notes" string (max 250 + null)
#define EE_RINGSLOT_SIZE            251

// GUI-protocol (over Bluetooth, prefix '#'): firmware- en protocolversie
// teruggegeven door #V?. Pre-release: we bumpen nog NIET per kleine wijziging
// omdat er nog geen Windows-tool in het wild zit die op een specifiek nummer
// vertrouwt. Bump pas bij echte release, of als een breaking change in de
// wire-API een client moet kunnen detecteren.
//
// Huidig command-surface (proto=2):
//   V?          -> #V: fw=...;proto=...;build=...
//   GET         -> #CFG: name=...;freq=...;...;audiomode=0/1/2;morsewpm=...;
//                       morseamp=...;ringtrack=1..10;morsetext=...
//   SET key=val -> #OK: <key>=<val>  of  #ERR: <reason>
//                  keys: name, freq, txon, txoff, mp3, terror, test, panic,
//                        audiomode, morsewpm, morsetext, morseamp, ringtrack,
//                        ringslot<N>  (N=1..10, strict RTTTL "name:defaults:notes",
//                        naam max 20 chars)
//   LIST        -> #TRK: index=...;name=...  (× N MP3-tracks van JQ8400) + #END
//   RTLIST      -> #RTSLOT: n=<N>;data=<rtttl>  (× 50, lege als data=) + #END
//   GETDEFAULT <n> -> #RTDEFAULT: n=<n>;data=<rtttl>  (slot 1-10 klassiekers,
//                     1-10) of #ERR: slot 1..10
//   OTA?, OTA START/D/END/ABORT -> #OTA: ... / #OK / #ERR
//   RST         -> herstart pieper
#define FW_VERSION                  "1.10"
#define GUI_PROTO_VERSION           2

void dbg(const String& s) {
  Serial.print('[');
  Serial.print(millis());
  Serial.print("] ");
  Serial.println(s);
}

void setup() {
  setCpuFrequencyMhz(80);                 // Laagste klok waarbij Bluetooth blijft werken (stroombesparing)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial2.begin(9600);

  pinMode(RF_TX, OUTPUT);
  pinMode(pin_LE, OUTPUT);
  pinMode(pin_LED, OUTPUT);
  pinMode(pin_Button, INPUT);
  digitalWrite(pin_LED, HIGH);

  doTXEnable(0);

  EEPROM.begin(EE_TOTAL_CNT);
  if (EEPROM.readByte(EE_CHECK) != EE_CHECKNUMBER) Defaultsettings();
  frequency = EEPROM.readUInt(EE_UINT16_FREQUENCY);
  panictime = EEPROM.readUInt(EE_UINT16_PANICTIME);
  txoff = EEPROM.readByte(EE_BYTE_TXOFF);
  txon = EEPROM.readByte(EE_BYTE_TXON);
  mp3track = EEPROM.readByte(EE_BYTE_MP3TRACK);
  terrormode = EEPROM.readByte(EE_BYTE_TERRORMODE);

  for (byte x = 0; x < 10; x++) piepername[x] = EEPROM.read(EE_BYTE_PIEPERNAME + x);

  // Nieuwere settings: clamp/validate omdat oude EEPROM 0xFF of mesh-data
  // op deze offsets kan bevatten (zie git-historie).
  audioMode = EEPROM.readByte(EE_BYTE_AUDIOMODE);
  if (audioMode > 2) audioMode = 0;        // clamp; 0=MP3, 1=morse, 2=ringtone

  morseWpm = EEPROM.readByte(EE_BYTE_MORSEWPM);
  if (morseWpm < 3 || morseWpm > 12) morseWpm = 6;
  for (byte x = 0; x < 30; x++) morseText[x] = EEPROM.read(EE_BYTE_MORSETEXT + x);
  morseText[30] = '\0';
  // Trim op eerste 0xFF (= niet-geinitialiseerde flash); default als leeg.
  for (byte x = 0; x < 30; x++) {
    if ((byte)morseText[x] == 0xFF) { morseText[x] = '\0'; break; }
  }
  if (morseText[0] == '\0') strcpy(morseText, "CQ");

  morseAmp = EEPROM.readByte(EE_BYTE_MORSEAMP);
  if (morseAmp < 1 || morseAmp > 10) morseAmp = 5;   // clamp; oude waarden (50/100/0xFF) -> default

  ringTrack = EEPROM.readByte(EE_BYTE_RINGTRACK);
  if (ringTrack < 1 || ringTrack > 10) ringTrack = 1;

  // Migratie: vul alle 10 slots met de default RTTTL-klassiekers (getRtDefault)
  // als ze leeg, uninitialized, Nokia-format, of v1.8-test zijn. User RTTTL-
  // customizations blijven respecteerd (alleen lege/foute slots krijgen default).
  bool needCommit = false;
  char migBuf[EE_RINGSLOT_SIZE];
  for (byte s = 1; s <= 10; s++) {
    loadRingSlot(s, migBuf, sizeof(migBuf));
    bool isEmpty   = (migBuf[0] == '\0');                              // ook 0xFF -> '\0'
    bool isNokia   = (!isEmpty && strchr(migBuf, ':') == nullptr);
    bool isV18Test = (strncmp(migBuf, "007 Tune:", 9) == 0);            // oude v1.8 default
    const char* def = getRtDefault(s);
    if (isEmpty && def[0] == '\0') continue;                            // beide leeg: niets doen
    if (isEmpty || isNokia || isV18Test) {
      saveRingSlot(s, def);
      needCommit = true;
      dbg("Migratie: slot " + String(s) + " -> default");
    }
  }
  if (needCommit) EEPROM.commit();

  Serial.println();
  Serial.println(String(piepername) + " opstarten..........");

  SPI.begin();
  mp3.reset();
  mp3.setVolume(30);
  numberoffiles = mp3.countFiles();
  if (numberoffiles > 49) numberoffiles = 49;   // file[] heeft 50 plekken; we gebruiken 1..49

  for (int x = 1; x < numberoffiles + 1; x++) {
    mp3.playFileByIndexNumber(x);
    file[x].index = x;
    mp3.currentFileName(file[x].name, sizeof(file[x].name));
    file[x].length = mp3.currentFileLengthInSeconds();
  }

  mp3.sleep();
  txonoffMillis = millis();
  digitalWrite(pin_LED, LOW);
  dbg("Opstarten voltooid (txon=" + String(txon) + " txoff=" + String(txoff) + " freq=" + String(frequency) + ")");
}

void loop() {
  if (millis() < bootTime && !config) {
    blinkLED(250);                          // signaal: config-venster open
    if (digitalRead(pin_Button) == LOW) {
      // Knop ingedrukt: meten hoe lang. Twee mogelijke uitkomsten:
      //   - Loslaten < longPressMs  -> CONFIG-mode (BT activeren)
      //   - Vasthouden >= longPressMs -> TX UIT zolang vastgehouden,
      //                                  bij loslaten direct TX AAN + cyclus
      //                                  vanaf het begin van een TX-ON fase
      unsigned long pressStart = millis();
      bool longPress = false;
      while (digitalRead(pin_Button) == LOW) {
        if (!longPress && (millis() - pressStart >= longPressMs)) {
          longPress = true;
          doTXEnable(0);
          digitalWrite(pin_LED, HIGH);      // continu brand = "TX kill actief"
          dbg("Knop >= 2s ingedrukt - TX UIT (kill switch)");
        }
        delay(10);
      }

      if (longPress) {
        // Loslaten na lange druk: direct in de lucht, cyclus herstart vanaf
        // de TX-ON fase met audio. txonoffstate=true zorgt dat de cyclus-state
        // machine in zijn ON-helft staat; txonoffMillis=now reset de timer.
        dbg("Knop losgelaten - TX AAN, cyclus herstart");
        doTXEnable(1);
        txonoffstate = true;
        txonoffMillis = millis();
      } else {
        // Korte druk: config-mode (huidige gedrag).
        config = true;
        testmode = false;
        doTXEnable(0);
        dbg("Configuratie gestart");
        SerialBT.begin(piepername);
        // Verlaag BT TX-power van default +9 dBm naar 0 dBm (1 mW). Kleinere
        // stroompieken op de 3.3 V rail = minder kans op PLL/PA-storing. Moet
        // NA SerialBT.begin() omdat de controller dan pas geinitialiseerd is.
        esp_bredr_tx_power_set(ESP_PWR_LVL_N0, ESP_PWR_LVL_N0);
        dbg("BT TX-power gezet op 0 dBm (1 mW)");
        digitalWrite(pin_LED, HIGH);
      }
    }
  } else if (!bootEnd) {
    bootEnd = true;
    digitalWrite(pin_LED, HIGH);
    dbg("Boot venster afgelopen");
  }

  if (millis() >= (unsigned long)panictime * 60UL * 1000UL) {
    if (config) SerialBT.println("Panic mode aktief!");
    dbg("Panic mode aktief!");
    frequency = 145000;
    mp3.sleep();
    doTXEnable(1);
    pinMode(pin_Audio, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    // arduino-esp32 3.x: pin-based ledc API
    ledcAttach(pin_Audio, 1000, 8);
    ledcWriteTone(pin_Audio, 1000);
    ledcWrite(pin_Audio, 50);
#else
    // arduino-esp32 2.x: channel-based ledc API
    ledcAttachPin(pin_Audio, 0);
    ledcWriteTone(0, 1000);
    ledcWrite(0, 50);
#endif
    while (true) blinkLED(500);
  }

  // Cyclus (toggle + TX/MP3) draait alleen wanneer we niet in pure config-
  // mode zitten en geen OTA-transfer bezig is. Test mode tijdens config
  // laat hem wel doorgaan zodat je de configuratie kunt verifieren. Zonder
  // deze guard zou de bookkeeping in config-mode blijven flippen en zou
  // een TX-aktie midden in een OTA-flashwrite vallen.
  if ((!config || testmode) && !otaActive) {
    // Bepaal de actieve audio-duur in ms (i.p.v. afgeronde seconden). Met
    // seconden-precisie was audioLenSec bij morse tot ~1 sec hoger dan wat
    // playMorse() werkelijk blokkeerde, waardoor de toggle niet vuurde toen
    // morse terugkwam -> de restart-if pakte morse opnieuw en het bericht
    // werd 2x per ON-fase geseind. Met ms-precisie vuurt de toggle exact
    // wanneer playMorse retour komt.
    unsigned long audioLenMs;
    if (audioMode == 1) {
      // Morse audio + 1 sec pre-roll + 1 sec post-roll (zie playMorse).
      audioLenMs = morseDurationMs(morseText, morseWpm) + 2000UL;
    } else if (audioMode == 2) {
      // Laad geselecteerd slot, dispatch op format (Nokia tempo,notes of RTTTL).
      loadRingSlot(ringTrack, ringBuf, sizeof(ringBuf));
      if (ringBuf[0] == '\0') {
        audioLenMs = 2000UL;
      } else {
        audioLenMs = slotDurationMs(ringBuf) + 2000UL;
      }
    } else {
      unsigned int mp3LenSec = (txon == 0) ? mp3.currentFileLengthInSeconds() : 0;
      if (mp3LenSec == 0 && txon == 0 && numberoffiles >= 1) {
        byte t = previousTrack;
        if (t < 1 || t > numberoffiles || t >= 50) {
          t = (mp3track >= 1 && mp3track <= numberoffiles) ? mp3track : (byte)1;
        }
        mp3LenSec = file[t].length;
      }
      audioLenMs = (unsigned long)mp3LenSec * 1000UL;
    }
    unsigned long needed = txonoffstate
                            ? (txon == 0 ? audioLenMs : (unsigned long)txon * 1000UL)
                            : (unsigned long)txoff * 1000UL;
    if (millis() >= txonoffMillis + needed) {
      unsigned long phaseMs = millis() - txonoffMillis;
      txonoffMillis = millis();
      txonoffstate = !txonoffstate;
      dbg(String("Toggle -> ") + (txonoffstate ? "ON" : "OFF") + " (vorige fase " + String(phaseMs) + " ms, audio=" + (audioMode == 1 ? "morse" : (audioMode == 2 ? "melody" : "mp3")) + " len=" + String(audioLenMs) + "ms, txon=" + String(txon) + " txoff=" + String(txoff) + ")");
      if (!txonoffstate && txoff != 0) {
        doTXEnable(0);
        if (config) SerialBT.println("TX UIT");
      }
    }

    if (txonoffstate) {
      if (audioMode == 1) {
        // Morse: playMorse blokkeert tot tekst af is. De volgende loop-iter
        // doet de toggle naar OFF (txon=0) of speelt opnieuw af (txon > duur).
        dbg("Morse start: \"" + String(morseText) + "\" (" + String(morseWpm) + " WPM)");
        if (config) SerialBT.println("TX AAN (morse)");
        doTXEnable(1);
        // SerialBT.flush() forceert dat de lokale BT-TX-buffer (waarin print/
        // println eerst hun bytes verzamelen tot MTU-grootte) nu daadwerkelijk
        // de send-queue in gaat - anders blijven kleine responses zoals
        // "#OK: test=1" achter en ziet de Windows-app pas na de morse iets.
        if (config) SerialBT.flush();
        playMorse(morseText, morseWpm);
      } else if (audioMode == 2) {
        loadRingSlot(ringTrack, ringBuf, sizeof(ringBuf));
        if (ringBuf[0] == '\0') {
          dbg("Slot " + String(ringTrack) + " is leeg, geen audio deze cyclus");
          // TX uit laten staan, cyclus loopt door op audioLenMs=2000.
        } else {
          dbg("RTTTL start: slot=" + String(ringTrack));
          if (config) SerialBT.println("TX AAN (RTTTL slot " + String(ringTrack) + ")");
          doTXEnable(1);
          if (config) SerialBT.flush();
          playSlot(ringBuf);
        }
      } else {
        byte mp3st = mp3.getStatus();
        if (mp3st != MP3_STATUS_PLAYING) {
          dbg("MP3 niet aan het spelen (status=" + String(mp3st) + "), start TX + playMP3");
          if (config) SerialBT.println("TX AAN");
          doTXEnable(1);
          playMP3(mp3track);
        }
      }
    }
  }

  if (config) handleMenu();
}

void doRandomFreq() {
  int randomFrequency = random(144000, 146000);
  randomFrequency = round(randomFrequency / 25.0) * 25;
  if (config) SerialBT.println("Willekeurige frequentie: " + String(randomFrequency / 1000) + "." + (randomFrequency % 1000 < 10 ? "0" : "") + (randomFrequency % 1000 < 100 ? "0" : "") + String(randomFrequency % 1000) + "MHz");
  setFreq(randomFrequency);
  dbg("Random freq: " + String(randomFrequency) + " kHz");
}

void doTXEnable(bool status) {
  if (status) {
    dbg("Zender INGESCHAKELD");
    digitalWrite(pin_LE, LOW);
    SPI.transfer(0x90);
    SPI.transfer(0x80);
    SPI.transfer(0x82);
    digitalWrite(pin_LE, HIGH);
    delay(50);
    digitalWrite(pin_LE, LOW);
    SPI.transfer(0x06);
    SPI.transfer(0x40);
    digitalWrite(pin_LE, HIGH);
    delay(50);
    if (terrormode) doRandomFreq(); else setFreq(frequency);
    digitalWrite(RF_TX, LOW);
  } else {
    dbg("Zender UITGESCHAKELD");
    digitalWrite(RF_TX, HIGH);
    mp3.stop();
    mp3.sleep();
  }
}

void setFreq(unsigned int frequency_kHz) {
  frequency_kHz /= 25;
  digitalWrite(pin_LE, LOW);
  SPI.transfer((frequency_kHz >> 8) & 0xFF) + 0x20;
  SPI.transfer(frequency_kHz & 0xFF);
  SPI.transfer(0x01);
  digitalWrite(pin_LE, HIGH);
  dbg("PLL ingesteld op " + String(frequency_kHz * 25) + " kHz");
}

void playMP3(unsigned int track) {
  if (track == 0) {
    if (numberoffiles == 1) {
      track = 1;
    } else {
      do {
        track = random(1, numberoffiles + 1);
      } while (track == previousTrack);
    }
  }

  mp3.playFileByIndexNumber(track);
  dbg("MP3 track " + String(track) + " gestart");
  mp3.play();
  previousTrack = track;
}

// === Morse code generator ====================================================
// 800 Hz toon op pin_Audio via LEDC. Tempo via WPM volgens de PARIS-norm
// (1 dot = 1200/wpm ms). Werkt onder arduino-esp32 2.x EN 3.x (ledc-API
// verschilt, vandaar de ifdefs - zelfde aanpak als panic-mode).

struct MorseEntry { char c; const char *code; };
static const MorseEntry morseTable[] = {
  {'A',".-"},   {'B',"-..."}, {'C',"-.-."}, {'D',"-.."},  {'E',"."},
  {'F',"..-."}, {'G',"--."},  {'H',"...."}, {'I',".."},   {'J',".---"},
  {'K',"-.-"},  {'L',".-.."}, {'M',"--"},   {'N',"-."},   {'O',"---"},
  {'P',".--."}, {'Q',"--.-"}, {'R',".-."},  {'S',"..."},  {'T',"-"},
  {'U',"..-"},  {'V',"...-"}, {'W',".--"},  {'X',"-..-"}, {'Y',"-.--"}, {'Z',"--.."},
  {'0',"-----"},{'1',".----"},{'2',"..---"},{'3',"...--"},{'4',"....-"},
  {'5',"....."},{'6',"-...."},{'7',"--..."},{'8',"---.."},{'9',"----."},
  {'/',"-..-."},{'?',"..--.."},{'.',".-.-.-"},{',',"--..--"},
  {'=',"-...-"},{'+',".-.-."},{'-',"-....-"}
};

const char* morseCode(char c) {
  if (c >= 'a' && c <= 'z') c -= 32;          // naar uppercase
  for (size_t i = 0; i < sizeof(morseTable)/sizeof(morseTable[0]); i++) {
    if (morseTable[i].c == c) return morseTable[i].code;
  }
  return nullptr;                              // onbekend teken -> overslaan
}

// Bereken totale duur in ms voor de gegeven tekst+snelheid (zonder echt te
// klinken). Gebruikt door de loop om de TX-on duur te bepalen wanneer txon=0.
unsigned long morseDurationMs(const char *text, byte wpm) {
  if (wpm < 3) wpm = 3; else if (wpm > 12) wpm = 12;
  unsigned long dotMs = 1200UL / wpm;
  unsigned long total = 0;
  bool firstChar = true;
  for (size_t i = 0; text[i]; i++) {
    char c = text[i];
    if (c == ' ') { total += dotMs * 7; firstChar = true; continue; }
    const char *code = morseCode(c);
    if (!code) continue;
    if (!firstChar) total += dotMs * 3;       // inter-letter gap
    firstChar = false;
    for (size_t j = 0; code[j]; j++) {
      if (j > 0) total += dotMs;              // inter-element gap
      total += (code[j] == '-') ? dotMs * 3 : dotMs;
    }
  }
  return total;
}

// Helpers: tone aan/uit en interrupteerbare delay (returns true = afbreken
// gevraagd door inkomende BT-input).
// Gemeenschappelijke duty-berekening voor morse én ringtone. morseAmp is een
// stap 1..10; we mappen op een smalle high-duty band die empirisch het beste
// klinkt op deze pieper-hardware: step N -> duty (100 - N)%, dus step 1 =
// 99% duty (smalste LOW-puls), step 10 = 90% duty. Bewust hoog-duty omdat
// de modulator-keten daar de luidste tonen geeft.
static uint32_t toneDuty() {
  uint32_t pct = (morseAmp >= 1 && morseAmp <= 10) ? (100UL - morseAmp) : 95UL;
  return pct * 256UL / 100UL;
}

static inline void morseToneOn() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin_Audio, toneDuty());
#else
  ledcWrite(0, toneDuty());
#endif
}
static inline void morseToneOff() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin_Audio, 0);
#else
  ledcWrite(0, 0);
#endif
}
// Speel een toon voor totaal `ms` milliseconden met een attack/release-envelope.
// In plaats van een harde duty-step (= klik) wordt de duty in ~3ms opgeramped
// (attack) en in ~10ms weer afgebouwd (release). Sustain houdt target-duty
// vast. De totale duur is exact `ms` zodat de cyclus-timing niet schuift.
// Returnt true bij abort (BT-input).
static bool playToneFor(unsigned long ms) {
  uint32_t target = toneDuty();

  // Te kort voor envelope (< 20ms): gewoon harde blip
  if (ms < 20UL) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin_Audio, target);
#else
    ledcWrite(0, target);
#endif
    bool abort = delayOrAbort(ms);
    morseToneOff();
    return abort;
  }

  // Attack: 3 ms in 12 stappen (250 µs/stap) - linear ramp 0 -> target
  for (int i = 1; i <= 12; i++) {
    uint32_t duty = (target * (uint32_t)i) / 12UL;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin_Audio, duty);
#else
    ledcWrite(0, duty);
#endif
    delayMicroseconds(250);
    if (SerialBT.available()) { morseToneOff(); return true; }
  }

  // Sustain op target voor (ms - 13ms attack+release)
  if (delayOrAbort(ms - 13UL)) { morseToneOff(); return true; }

  // Release: 10 ms in 20 stappen (500 µs/stap) - linear ramp target -> 0
  for (int i = 1; i <= 20; i++) {
    uint32_t duty = target - (target * (uint32_t)i) / 20UL;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin_Audio, duty);
#else
    ledcWrite(0, duty);
#endif
    delayMicroseconds(500);
    if (SerialBT.available()) { morseToneOff(); return true; }
  }
  morseToneOff();   // zeker weten duty=0 aan einde
  return false;
}

static bool delayOrAbort(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (SerialBT.available()) return true;                     // BT-input -> direct af
    // De config-knop alleen meetellen zolang we ook werkelijk config-mode in
    // kunnen (= dezelfde voorwaarde als de boot-window check in loop()).
    // Buiten dat venster zou afbreken alleen maar morse herstarten zonder
    // dat de knop iets functioneels doet.
    if ((millis() < bootTime) && !config && digitalRead(pin_Button) == LOW) return true;
    delay(1);                                                  // yield aan FreeRTOS
  }
  return false;
}

// Genereer de morse-toon. Blokkeert tot de hele tekst klaar is, maar checkt
// tussen elementen op BT-input zodat config-commando's tijdens lange teksten
// snel verwerkt worden. Bij afbreek wordt de toon uitgezet en de loop pakt op
// het volgende rondje opnieuw op (of niet, als de gebruiker test-mode/cyclus
// uitzette).
void playMorse(const char *text, byte wpm) {
  if (wpm < 3) wpm = 3; else if (wpm > 12) wpm = 12;
  unsigned long dotMs = 1200UL / wpm;

  pinMode(pin_Audio, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin_Audio, 800, 8);
#else
  ledcSetup(0, 800, 8);
  ledcAttachPin(pin_Audio, 0);
#endif

  bool firstChar = true;

  // 1 sec stilte op de carrier vóór de eerste dit/dah, zodat de squelch
  // van de ontvanger tijd heeft om open te komen en de eerste karakters
  // niet worden afgekapt. PTT (RF_TX) staat hier al aan via doTXEnable(1).
  if (delayOrAbort(1000)) goto morse_abort;

  for (size_t i = 0; text[i]; i++) {
    char c = text[i];
    if (c == ' ') {
      if (delayOrAbort(dotMs * 7)) goto morse_abort;
      firstChar = true;
      continue;
    }
    const char *code = morseCode(c);
    if (!code) continue;
    if (!firstChar && delayOrAbort(dotMs * 3)) goto morse_abort;
    firstChar = false;
    for (size_t j = 0; code[j]; j++) {
      if (j > 0 && delayOrAbort(dotMs)) goto morse_abort;
      // playToneFor levert attack+sustain+release binnen dezelfde totaal-tijd
      // (verzacht de aanzet/uitfade-klik), en doet ook de abort-check.
      if (playToneFor(code[j] == '-' ? dotMs * 3 : dotMs)) goto morse_abort;
    }
  }
  // Houd de PTT nog ~1 sec aan na het laatste morse-element zodat de TX
  // niet abrupt wordt uitgeschakeld direct op de zijslag (zachter eind voor
  // de luisteraar). Interrupteerbaar - retour-waarde negeren want we gaan
  // sowieso terug naar de loop.
  (void) delayOrAbort(1000);
  return;

morse_abort:
  morseToneOff();
  dbg("Morse afgebroken (BT input)");
}

// === Nokia ringtone player ===================================================
// Speelt eenvoudige Nokia-Composer melodieën. Tokens met spatie gescheiden:
//   [dur][.][#]<note><octave>
// dur=1/2/4/8/16/32 (default 4), .=dotted (1.5x), #=kruis, note=a..g of '-'
// (rust), octave=1/2/3 (default 2 ≈ middel-C-octaaf, MIDI 5). Voorbeeld:
// "8e2 8d2 4#f1" = achtste E, achtste D, kwart Fis (octaaf 1).
// Gebruikt dezelfde toneDuty() / pin_Audio als morse - morseAmp regelt ook
// het ringtone-volume.

// === RTTTL audio toon-generatie ==============================================
// Frequenties voor RTTTL-octave 5 (middle C, per RTTTL spec: C5 = 261.63 Hz).
// Octave 4 = freq/2, octave 6 = freq*2, octave 7 = freq*4.
static const uint16_t noteFreq[12] = {
//  C    C#   D    D#   E    F    F#   G    G#   A    A#   B
  262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494
};

// === Ringtone slot management (10 slots, 251 bytes elk in EEPROM) ============
// Slot-format: strict RTTTL "name:defaults:notes". Slots worden niet permanent
// in RAM gehouden - bij gebruik geladen in ringBuf (single shared buffer).

// Lees slot (1..10) in buf. Returnt false bij leeg slot of ongeldige index.
bool loadRingSlot(byte slot, char* buf, size_t bufSize) {
  if (slot < 1 || slot > 10 || bufSize < 1) { if (bufSize) buf[0] = '\0'; return false; }
  unsigned int offset = EE_BYTE_RINGSLOTS + (slot - 1) * EE_RINGSLOT_SIZE;
  size_t i = 0;
  for (; i < EE_RINGSLOT_SIZE - 1 && i < bufSize - 1; i++) {
    byte b = EEPROM.read(offset + i);
    if (b == 0 || b == 0xFF) break;
    buf[i] = (char)b;
  }
  buf[i] = '\0';
  return i > 0;
}

// Snel checken of een slot data heeft, zonder de hele string te laden.
bool slotHasData(byte slot) {
  if (slot < 1 || slot > 10) return false;
  byte first = EEPROM.read(EE_BYTE_RINGSLOTS + (slot - 1) * EE_RINGSLOT_SIZE);
  return first != 0 && first != 0xFF;
}

// Schrijf een slot. Geeft EEPROM.commit() vrij aan de caller (efficiënter
// als meerdere slots tegelijk worden geschreven).
void saveRingSlot(byte slot, const char* text) {
  if (slot < 1 || slot > 10) return;
  unsigned int offset = EE_BYTE_RINGSLOTS + (slot - 1) * EE_RINGSLOT_SIZE;
  size_t len = strlen(text);
  if (len > EE_RINGSLOT_SIZE - 1) len = EE_RINGSLOT_SIZE - 1;
  for (size_t i = 0; i < len; i++) EEPROM.write(offset + i, (byte)text[i]);
  EEPROM.write(offset + len, 0);                  // null terminator
}

// Frequentie-helper voor RTTTL-octave 4..7. Octave 5 = base van noteFreq-tabel.
uint16_t computeFreq(byte noteIdx, byte octave) {
  if (noteIdx >= 12) return 0;
  uint16_t f = noteFreq[noteIdx];
  if (octave == 4)      f >>= 1;
  else if (octave == 6) f <<= 1;
  else if (octave == 7) f <<= 2;
  // octave == 5 = base (middle C range)
  return f;
}

// === RTTTL parser ============================================================
// Per Wikipedia RTTTL spec: https://en.wikipedia.org/wiki/Ring_Tone_Text_Transfer_Language
// Format: "<name>:<defaults>:<notes>"
//   defaults: comma-separated 'd=N,o=N,b=N' pairs (allen optioneel).
//             d = default note duration: 1, 2, 4, 8, 16, 32 (default 4)
//             o = default octave: 4, 5, 6, 7 (default 6 per spec; 5 = middle C range)
//             b = BPM: spec-range 25..900 (wij clampen 40..900 voor onze noot-timing)
//   notes: comma-separated tone commands. Per spec: "[dur]<note>[#][octave][.]"
//          note = c, c#, d, d#, e, f, f#, g, g#, a, a#, b (case-insensitive)
//          P = pause/rest
//          dur, octave optioneel; ontbrekend = default uit defaults-sectie
//          dotted-note: '.' aan einde (sommige implementaties accepteren ook na duration)
// Voorbeeld: "007 Tune:o=5,d=4,b=320:c,8d,8d,d,2d,c,c,c,c,8d#,..."

// Skip name + parse defaults. Returnt pointer naar notes-sectie.
const char* parseRttlDefaults(const char* p, byte& defOctave, byte& defDur, uint16_t& bpm) {
  defOctave = 6;
  defDur = 4;
  bpm = 63;                                       // RTTTL spec-default
  // Skip name section
  while (*p && *p != ':') p++;
  if (*p == ':') p++;
  // Parse "key=val" pairs gescheiden door komma's, tot volgende ':'
  while (*p && *p != ':') {
    while (*p == ' ' || *p == ',') p++;
    if (!*p || *p == ':') break;
    char key = (*p++) | 0x20;                    // lowercase
    if (*p == '=') p++;
    int val = 0;
    while (*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
    if (key == 'o') {
      if (val >= 4 && val <= 7) defOctave = (byte)val;
    } else if (key == 'd') {
      if (val == 1 || val == 2 || val == 4 || val == 8 || val == 16 || val == 32) defDur = (byte)val;
    } else if (key == 'b') {
      if (val < 40) val = 40; else if (val > 900) val = 900;
      bpm = (uint16_t)val;
    }
  }
  if (*p == ':') p++;
  return p;
}

// Parse één RTTTL-noot vanaf p. Output: dur, dotted, noteIdx (12=rust), octave.
// Returnt nullptr aan einde tekst of parse-fout.
const char* parseRttlNote(const char* p, byte defDur, byte defOctave, byte& dur, bool& dotted, byte& noteIdx, byte& octave) {
  while (*p == ' ' || *p == ',' || *p == '\r' || *p == '\n') p++;
  if (!*p) return nullptr;
  dotted = false;
  // Optionele duration prefix
  dur = 0;
  while (*p >= '0' && *p <= '9') { dur = dur * 10 + (*p - '0'); p++; }
  if (dur == 0) dur = defDur;
  // Sommige RTTTL-varianten zetten de dot direct NA duration ipv aan het eind.
  if (*p == '.') { dotted = true; p++; }
  // Note letter
  char c = *p | 0x20;
  if (c == 'p') {                                // RTTTL rust
    noteIdx = 12; octave = defOctave; p++;
  } else {
    byte base;
    switch (c) {
      case 'c': base = 0;  break;
      case 'd': base = 2;  break;
      case 'e': base = 4;  break;
      case 'f': base = 5;  break;
      case 'g': base = 7;  break;
      case 'a': base = 9;  break;
      case 'b': base = 11; break;
      default: return nullptr;
    }
    p++;
    // Optionele sharp (in RTTTL: NA de note-letter)
    byte sharp = 0;
    if (*p == '#') { sharp = 1; p++; }
    noteIdx = (base + sharp) % 12;
    // Optionele octave (4..7)
    if (*p >= '4' && *p <= '7') { octave = (byte)(*p - '0'); p++; }
    else { octave = defOctave; }
  }
  // Standard RTTTL dot-positie: aan einde van het token.
  if (*p == '.') { dotted = true; p++; }
  return p;
}

unsigned long rttlDurationMs(const char* rtttl) {
  byte defOctave, defDur;
  uint16_t bpm;
  const char* p = parseRttlDefaults(rtttl, defOctave, defDur, bpm);
  unsigned long total = 0;
  byte dur, noteIdx, octave;
  bool dotted;
  while ((p = parseRttlNote(p, defDur, defOctave, dur, dotted, noteIdx, octave)) != nullptr) {
    unsigned long noteMs = 240000UL / ((unsigned long)bpm * dur);
    if (dotted) noteMs = (noteMs * 3UL) / 2UL;
    total += noteMs;
  }
  return total;
}

void playRttl(const char* rtttl) {
  byte defOctave, defDur;
  uint16_t bpm;
  const char* p = parseRttlDefaults(rtttl, defOctave, defDur, bpm);

  pinMode(pin_Audio, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin_Audio, 1000, 8);
#else
  ledcSetup(0, 1000, 8);
  ledcAttachPin(pin_Audio, 0);
#endif

  if (delayOrAbort(1000)) goto rttl_abort;

  {
    byte dur, noteIdx, octave;
    bool dotted;
    while ((p = parseRttlNote(p, defDur, defOctave, dur, dotted, noteIdx, octave)) != nullptr) {
      unsigned long noteMs = 240000UL / ((unsigned long)bpm * dur);
      if (dotted) noteMs = (noteMs * 3UL) / 2UL;
      if (noteIdx == 12) {
        morseToneOff();
        if (delayOrAbort(noteMs)) goto rttl_abort;
      } else {
        uint16_t freq = computeFreq(noteIdx, octave);
        // BELANGRIJK: NIET ledcWriteTone() gebruiken - die forceert intern
        // 10-bit resolutie, waardoor toneDuty() (gemaakt voor 8-bit) een
        // verkeerde duty oplevert en morseAmp dus geen hoorbaar volume-effect
        // meer heeft. ledcChangeFrequency / ledcSetup behoudt onze 8-bit
        // resolutie zodat morseAmp wel werkt voor RTTTL.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcChangeFrequency(pin_Audio, freq, 8);
#else
        ledcSetup(0, freq, 8);
#endif
        unsigned long playMs = (noteMs > 30UL) ? (noteMs - 30UL) : noteMs;
        if (playToneFor(playMs)) goto rttl_abort;
        if (noteMs > 30UL && delayOrAbort(30)) goto rttl_abort;
      }
    }
  }

  (void) delayOrAbort(1000);
  return;

rttl_abort:
  morseToneOff();
  dbg("RTTTL afgebroken (BT input)");
}

// Top-level slot-dispatchers (RTTTL-only sinds v1.8).
unsigned long slotDurationMs(const char* slot) {
  if (!slot || !*slot) return 0;
  return rttlDurationMs(slot);
}

void playSlot(const char* slot) {
  if (!slot || !*slot) return;
  playRttl(slot);
}

// Zet de TX-cyclus terug naar een schone startstand: TX uit, ON-fase, timer
// op nu. Aanroepen wanneer test mode wordt aan- of uitgezet, zodat de
// volgende fase netjes vanaf nul begint i.p.v. midden in een oude periode.
void resetCycle() {
  doTXEnable(0);
  txonoffstate = true;
  txonoffMillis = millis();
  dbg("Cyclus gereset");
}

// === GUI-protocol (afgescheiden van het terminal-menu) =====================
// Alle commando's beginnen met '#'. Responses ook met '#', gevolgd door een
// type-prefix (V, CFG, TRK, OK, ERR, BYE, END). KV-paren scheiden met ';'
// zodat waarden spaties mogen bevatten (bv. name=Pieper X). Het terminal-
// menu (F/A/B/N/etc.) blijft hierdoor volledig onaangetast.
void handleGuiCommand(const String& input) {
  String cmd = input.substring(1);    // strip de '#'
  cmd.replace("\r", "");
  cmd.replace("\n", "");
  cmd.trim();

  String head = cmd;
  String args = "";
  int sp = head.indexOf(' ');
  if (sp >= 0) {
    head = cmd.substring(0, sp);
    args = cmd.substring(sp + 1);
  }
  head.toUpperCase();

  if (head == "V?" || head == "VERSION") {
    SerialBT.print("#V: fw=" FW_VERSION ";proto=");
    SerialBT.print(GUI_PROTO_VERSION);
    SerialBT.print(";build=");
    SerialBT.println(__DATE__);
    return;
  }

  if (head == "GET") {
    SerialBT.print("#CFG: name=");      SerialBT.print(piepername);
    SerialBT.print(";freq=");           SerialBT.print(frequency);
    SerialBT.print(";txon=");           SerialBT.print(txon);
    SerialBT.print(";txoff=");          SerialBT.print(txoff);
    SerialBT.print(";mp3=");            SerialBT.print(mp3track);
    SerialBT.print(";nfiles=");         SerialBT.print(numberoffiles);
    SerialBT.print(";terror=");         SerialBT.print(terrormode ? 1 : 0);
    SerialBT.print(";test=");           SerialBT.print(testmode ? 1 : 0);
    SerialBT.print(";panic=");          SerialBT.print(panictime);
    SerialBT.print(";audiomode=");      SerialBT.print(audioMode);
    SerialBT.print(";morsewpm=");       SerialBT.print(morseWpm);
    SerialBT.print(";morseamp=");       SerialBT.print(morseAmp);
    SerialBT.print(";ringtrack=");      SerialBT.print(ringTrack);
    SerialBT.print(";morsetext=");      SerialBT.println(morseText);   // als laatste vrije-tekst veld;
    // (ringtone slot-data hoort niet in #GET wegens lengte - gebruik #RTLIST)
    return;
  }

  if (head == "RTLIST") {
    // Geef alle 10 ringtone-slots terug, lege slots als data= (leeg).
    char buf[EE_RINGSLOT_SIZE];
    for (byte s = 1; s <= 10; s++) {
      SerialBT.print("#RTSLOT: n="); SerialBT.print(s); SerialBT.print(";data=");
      if (loadRingSlot(s, buf, sizeof(buf))) SerialBT.println(buf);
      else SerialBT.println();
    }
    SerialBT.println("#END");
    return;
  }

  if (head == "FACTORYRESET") {
    // Wist de complete NVS-partitie (EEPROM-blob + BT-pairing + alle andere
    // NVS-state) en herstart. Gebruik dit als de NVS vol/corrupt geraakt is
    // (bv. ESP_ERR_NVS_NOT_ENOUGH_SPACE bij elke EEPROM.commit()), of als
    // de gebruiker een echte fabrieksreset wil. Na reboot draait
    // Defaultsettings() vanzelf omdat EE_CHECK weg is.
    SerialBT.println("#OK: wiping NVS en herstart over 1 sec");
    SerialBT.flush();
    delay(1000);
    nvs_flash_erase();
    nvs_flash_init();
    esp_restart();
    return;
  }

  if (head == "GETDEFAULT") {
    // Stuur de firmware-default RTTTL voor een slot. Gebruikt door de Windows-
    // tool als "herstel default"-knop (één bron van waarheid: getRtDefault).
    int slot = args.toInt();
    if (slot < 1 || slot > 10) { SerialBT.println("#ERR: slot 1..10"); return; }
    SerialBT.print("#RTDEFAULT: n="); SerialBT.print(slot);
    SerialBT.print(";data=");        SerialBT.println(getRtDefault((byte)slot));
    return;
  }

  if (head == "LIST") {
    for (unsigned int x = 1; x <= numberoffiles && x < 50; x++) {
      SerialBT.print("#TRK: index=");   SerialBT.print(x);
      SerialBT.print(";name=");         SerialBT.print(file[x].name);
      SerialBT.print(";length=");       SerialBT.println(file[x].length);
    }
    SerialBT.println("#END");
    return;
  }

  if (head == "RST") {
    SerialBT.println("#BYE");
    delay(200);                          // geef BT-stack tijd om te flushen
    esp_restart();
    return;
  }

  if (head == "SET") {
    int eq = args.indexOf('=');
    if (eq < 1) { SerialBT.println("#ERR: bad SET format (verwacht #SET key=value)"); return; }
    String key = args.substring(0, eq);
    String val = args.substring(eq + 1);
    key.toLowerCase();

    if (key == "freq") {
      unsigned int f = val.toInt();
      if (f >= 144000 && f <= 146000) {
        frequency = (unsigned int)(round(f / 25.0) * 25);
        EEPROM.writeUInt(EE_UINT16_FREQUENCY, frequency); EEPROM.commit();
        SerialBT.print("#OK: freq="); SerialBT.println(frequency);
      } else SerialBT.println("#ERR: freq out of range (144000-146000)");
    } else if (key == "txon") {
      int v = val.toInt();
      if (v >= 0 && v <= 99) {
        txon = (byte)v;
        EEPROM.writeByte(EE_BYTE_TXON, txon); EEPROM.commit();
        SerialBT.print("#OK: txon="); SerialBT.println(txon);
      } else SerialBT.println("#ERR: txon out of range (0-99)");
    } else if (key == "txoff") {
      int v = val.toInt();
      if (v >= 0 && v <= 99) {
        txoff = (byte)v;
        EEPROM.writeByte(EE_BYTE_TXOFF, txoff); EEPROM.commit();
        SerialBT.print("#OK: txoff="); SerialBT.println(txoff);
      } else SerialBT.println("#ERR: txoff out of range (0-99)");
    } else if (key == "mp3") {
      int v = val.toInt();
      if (v >= 0 && v <= (int)numberoffiles) {
        mp3track = (byte)v;
        EEPROM.writeByte(EE_BYTE_MP3TRACK, mp3track); EEPROM.commit();
        SerialBT.print("#OK: mp3="); SerialBT.println(mp3track);
      } else SerialBT.println("#ERR: mp3 out of range (0..nfiles)");
    } else if (key == "panic") {
      int v = val.toInt();
      if (v > 0) {
        panictime = (unsigned int)v;
        EEPROM.writeUInt(EE_UINT16_PANICTIME, panictime); EEPROM.commit();
        SerialBT.print("#OK: panic="); SerialBT.println(panictime);
      } else SerialBT.println("#ERR: panic must be > 0");
    } else if (key == "name") {
      val.toCharArray(piepername, 11);
      EEPROM.writeString(EE_BYTE_PIEPERNAME, String(piepername)); EEPROM.commit();
      SerialBT.print("#OK: name="); SerialBT.println(piepername);
    } else if (key == "terror") {
      terrormode = (val.toInt() != 0);
      EEPROM.writeByte(EE_BYTE_TERRORMODE, terrormode); EEPROM.commit();
      SerialBT.print("#OK: terror="); SerialBT.println(terrormode ? 1 : 0);
    } else if (key == "test") {
      testmode = (val.toInt() != 0);     // niet in EEPROM (transient)
      resetCycle();
      SerialBT.print("#OK: test="); SerialBT.println(testmode ? 1 : 0);
    } else if (key == "audiomode") {
      int v = val.toInt();
      if (v >= 0 && v <= 2) {
        audioMode = (byte)v;
        EEPROM.writeByte(EE_BYTE_AUDIOMODE, audioMode); EEPROM.commit();
        SerialBT.print("#OK: audiomode="); SerialBT.println(audioMode);
      } else SerialBT.println("#ERR: audiomode 0..2 (0=mp3, 1=morse, 2=ringtone)");
    } else if (key == "morsewpm") {
      int v = val.toInt();
      if (v >= 3 && v <= 12) {
        morseWpm = (byte)v;
        EEPROM.writeByte(EE_BYTE_MORSEWPM, morseWpm); EEPROM.commit();
        SerialBT.print("#OK: morsewpm="); SerialBT.println(morseWpm);
      } else SerialBT.println("#ERR: morsewpm 3..12");
    } else if (key == "morseamp") {
      int v = val.toInt();
      if (v >= 1 && v <= 10) {
        morseAmp = (byte)v;
        EEPROM.writeByte(EE_BYTE_MORSEAMP, morseAmp); EEPROM.commit();
        SerialBT.print("#OK: morseamp="); SerialBT.println(morseAmp);
      } else SerialBT.println("#ERR: morseamp 1..10 (stap)");
    } else if (key == "morsetext") {
      if (val.length() > 30) val = val.substring(0, 30);
      val.toCharArray(morseText, 31);
      EEPROM.writeString(EE_BYTE_MORSETEXT, String(morseText)); EEPROM.commit();
      SerialBT.print("#OK: morsetext="); SerialBT.println(morseText);
    } else if (key == "ringtrack") {
      int v = val.toInt();
      if (v >= 1 && v <= 10) {
        ringTrack = (byte)v;
        EEPROM.writeByte(EE_BYTE_RINGTRACK, ringTrack); EEPROM.commit();
        SerialBT.print("#OK: ringtrack="); SerialBT.println(ringTrack);
      } else SerialBT.println("#ERR: ringtrack 1..10");
    } else if (key.startsWith("ringslot")) {
      int slot = key.substring(8).toInt();              // "ringslot3" -> 3
      int colonPos = val.indexOf(':');
      if (slot < 1 || slot > 10) {
        SerialBT.println("#ERR: ringslot index 1..10");
      } else if (val.length() > 0 && colonPos < 0) {
        SerialBT.println("#ERR: RTTTL-only (string moet ':' bevatten)");
      } else if (colonPos > 20) {
        SerialBT.println("#ERR: naam max 20 chars (was " + String(colonPos) + ")");
      } else {
        if (val.length() > EE_RINGSLOT_SIZE - 1) val = val.substring(0, EE_RINGSLOT_SIZE - 1);
        char buf[EE_RINGSLOT_SIZE];
        val.toCharArray(buf, EE_RINGSLOT_SIZE);
        saveRingSlot((byte)slot, buf);
        EEPROM.commit();
        SerialBT.print("#OK: ringslot"); SerialBT.print(slot); SerialBT.print("=");
        SerialBT.println(buf);
      }
    } else {
      SerialBT.println("#ERR: unknown key");
    }
    return;
  }

  // === OTA (firmware-flash via BT) ============================================
  if (head == "OTA?") {
    SerialBT.print("#OTA: ready=");      SerialBT.print(otaActive ? 0 : 1);
    SerialBT.print(";max=");             SerialBT.print(ESP.getFreeSketchSpace());
    SerialBT.print(";chunk=1024");
    SerialBT.print(";received=");        SerialBT.println(otaBytesReceived);
    return;
  }

  if (head == "OTA") {
    int sp2 = args.indexOf(' ');
    String subHead = args, subArgs = "";
    if (sp2 >= 0) { subHead = args.substring(0, sp2); subArgs = args.substring(sp2 + 1); }
    subHead.toUpperCase();

    if (subHead == "START") {
      if (otaActive) { Update.abort(); otaActive = false; }
      unsigned long declaredSize = 0;
      String md5hex = "";
      int p = 0;
      while (p < (int)subArgs.length()) {
        int semi = subArgs.indexOf(';', p);
        if (semi < 0) semi = subArgs.length();
        String kv = subArgs.substring(p, semi);
        int eq2 = kv.indexOf('=');
        if (eq2 > 0) {
          String k = kv.substring(0, eq2); k.toLowerCase();
          String v = kv.substring(eq2 + 1);
          if (k == "size") declaredSize = (unsigned long)v.toInt();
          else if (k == "md5") md5hex = v;
        }
        p = semi + 1;
      }
      if (declaredSize == 0 || md5hex.length() != 32) {
        SerialBT.println("#OTA: ERR bad START args (verwacht size=N;md5=32hex)");
        return;
      }
      if (!Update.begin(declaredSize)) {
        SerialBT.print("#OTA: ERR begin (");
        SerialBT.print(Update.errorString());
        SerialBT.println(")");
        return;
      }
      Update.setMD5(md5hex.c_str());
      otaActive = true;
      otaExpectedSize = declaredSize;
      otaBytesReceived = 0;
      doTXEnable(0);                     // veiligheidshalve TX uit tijdens OTA
      dbg("OTA START " + String(declaredSize) + " bytes, md5=" + md5hex);
      SerialBT.println("#OTA: OK");
      return;
    }

    if (subHead == "D") {
      if (!otaActive) {
        SerialBT.println("#OTA: ERR not active (stuur eerst #OTA START)");
        return;
      }
      unsigned char binBuf[2048];
      size_t binLen = 0;
      int rc = mbedtls_base64_decode(binBuf, sizeof(binBuf), &binLen,
                                     (const unsigned char *)subArgs.c_str(),
                                     subArgs.length());
      if (rc != 0) {
        SerialBT.print("#OTA: ERR base64 (rc="); SerialBT.print(rc); SerialBT.println(")");
        Update.abort(); otaActive = false;
        return;
      }
      size_t w = Update.write(binBuf, binLen);
      if (w != binLen) {
        SerialBT.print("#OTA: ERR write ("); SerialBT.print(w);
        SerialBT.print("/"); SerialBT.print(binLen); SerialBT.println(")");
        Update.abort(); otaActive = false;
        return;
      }
      otaBytesReceived += w;
      SerialBT.print("#OTA: ACK ");
      SerialBT.println(otaBytesReceived);
      return;
    }

    if (subHead == "END") {
      if (!otaActive) { SerialBT.println("#OTA: ERR not active"); return; }
      if (Update.end(true)) {
        dbg("OTA END ok, " + String(otaBytesReceived) + " bytes; reboot");
        SerialBT.println("#OTA: OK reboot");
        SerialBT.println("#BYE");
        delay(500);
        ESP.restart();
      } else {
        SerialBT.print("#OTA: ERR end (");
        SerialBT.print(Update.errorString());
        SerialBT.println(")");
        otaActive = false;
      }
      return;
    }

    if (subHead == "ABORT") {
      if (otaActive) { Update.abort(); otaActive = false; dbg("OTA ABORT"); }
      SerialBT.println("#OTA: OK");
      return;
    }

    SerialBT.println("#OTA: ERR unknown subcommand");
    return;
  }

  SerialBT.println("#ERR: unknown command");
}

void handleMenu() {
  if (!testmode) blinkLED(1000);

  // Leeg de hele BT-RX-buffer per loop()-iteratie i.p.v. één byte per keer.
  // Zonder deze while loopt de OTA-throughput tegen 1 byte per loop-iter aan,
  // en omdat loop() (line ~145) elke iter een mp3.getStatus() doet naar de
  // slapende JQ8400, kost dat ~150 ms per byte - praktisch onbruikbaar voor
  // chunks van 1 kB.
  while (SerialBT.available()) {
    char input = SerialBT.read();
    inputBuffer += input;

    if (input == '\n' || input == '\r') {
      if (!inputBuffer.isEmpty()) {
        switch (inputBuffer[0]) {
          case '#':
            handleGuiCommand(inputBuffer);
            break;

          case 'h':
          case 'H':
          case '?':
            SerialBT.println("Pieper menu");
            SerialBT.println("-----------");
            SerialBT.println();
            SerialBT.println("?\t\t\tToon menu.");
            SerialBT.println("F<freq in kHz>\tStel de frequentie in kHz in.");
            SerialBT.println("A<sec>\t\tTijd in sec., dat de zender uit de lucht is.");
            SerialBT.println("B<sec>\t\tTijd in sec., dat de zender in de lucht is. 0 voor MP3 lengte.");
            SerialBT.println("P<min>\t\tPaniektijd in min. (Hierna Permanente TX op 145.0MHz met toon.");
            SerialBT.println("N<naam>\t\tPiepernaam (max 10 characters).");
            SerialBT.println("T\t\t\tTestmode met huidige configuratie.");
            SerialBT.println("R\t\t\tActiveer terror mode, elke TX een andere frequentie xD.");
            SerialBT.println("M<track>\t\tAf te spelen MP3 track. (0 voor willekeurig).");
            SerialBT.println("K<0/1/2>\tAudiomodus: 0=MP3, 1=Morse, 2=Nokia ringtone.");
            SerialBT.println("C<tekst>\tMorse tekst (max 30 karakters).");
            SerialBT.println("W<wpm>\t\tMorse snelheid in WPM (3-12, standaard 6).");
            SerialBT.println("V<1-10>\tToon amplitude-stap (1=luidst, 10=zachtst, std 5; voor morse+ringtone).");
            SerialBT.println("X<n>\t\tRingtone slot selecteren (n=1..10).");
            SerialBT.println("Z<n>=<text>\tRingtone slot zetten (tempo,notes format, max 250 chars).");
            SerialBT.println("Z<n>\t\tInhoud van slot n tonen.");
            SerialBT.println("L\t\t\tLijst met beschikbare MP3 bestanden.");
            SerialBT.println("O\t\t\tOverzicht van huidige configuratie.");
            SerialBT.println("S\t\t\tHerstart de pieper.");
            SerialBT.println("Q\t\t\tFabrieksreset (NVS wipen, alle config + BT-pairing weg).");
            SerialBT.println();
            break;

          case 'F':
          case 'f': {
              unsigned int freqValue = inputBuffer.substring(1).toInt();
              if (freqValue >= 144000 && freqValue <= 146000) {
                frequency = round(freqValue / 25.0) * 25;
                SerialBT.println("Frequentie ingesteld op " + String(frequency / 1000) + "." + (frequency % 1000 < 10 ? "0" : "") + (frequency % 1000 < 100 ? "0" : "") + String(frequency % 1000) + "MHz");
                EEPROM.writeUInt(EE_UINT16_FREQUENCY, frequency);
                EEPROM.commit();
              } else {
                SerialBT.println("Ongeldige frequentie. Frequentie moet liggen tussen 144000-146000!");
              }
              SerialBT.println();
              break;
            }

          case 'A':
          case 'a': {
              byte txoffValue = inputBuffer.substring(1).toInt();
              if (txoffValue >= 0 && txoffValue <= 99) {
                txoff = txoffValue;
                SerialBT.println("TX uit periode ingesteld op " + String(txoff) + " Sec.");
                EEPROM.writeByte(EE_BYTE_TXOFF, txoff);
                EEPROM.commit();
              } else {
                SerialBT.println("Ongeldige periode. Waarde moet tussen 0 en 99 liggen!");
                SerialBT.println("Kies 0 om pieper permanent uit te laten zenden.");
              }
              SerialBT.println();
              break;
            }

          case 'B':
          case 'b': {
              byte txonValue = inputBuffer.substring(1).toInt();
              if (txonValue >= 0 && txonValue <= 99) {
                txon = txonValue;
                if (txon == 0) {
                  SerialBT.println("TX aan tot einde van MP3 track of permament indien TX uit op 0 staat.");
                } else {
                  SerialBT.println("TX aan periode ingesteld op " + String(txon) + " Sec.");
                }
                EEPROM.writeByte(EE_BYTE_TXON, txon);
                EEPROM.commit();
              } else {
                SerialBT.println("Ongeldige periode. Waarde moet tussen 0 en 99 liggen!");
                SerialBT.println("Kies 0 om pieper uit te schakelen na het einde van de MP3 track.");
              }
              SerialBT.println();
              break;
            }

          case 'L':
          case 'l':
            SerialBT.println("MP3 lijst");
            SerialBT.println("---------");
            SerialBT.println();
            SerialBT.println("Track:\tNaam:\t\tLengte: (sec)");
            for (int x = 1; x < numberoffiles + 1; x++) {
              SerialBT.print(file[x].index);
              SerialBT.print("\t\t");
              SerialBT.print(file[x].name);
              SerialBT.print("\t");
              SerialBT.println(file[x].length);
            }
            break;

          case 'P':
          case 'p': {
              unsigned int panicValue = inputBuffer.substring(1).toInt();
              if (panicValue > 0) {
                panictime = panicValue;
                SerialBT.println("Paniek timer ingesteld op " + String(panictime) + " minuten");
                if (panictime < 60) SerialBT.println("LET OP: Dit is minder dan één uur!");
                SerialBT.println("Na deze periode zal de zender permanent worden ingeschakeld op 145.000MHz met een continu toon.");
                EEPROM.writeUInt(EE_UINT16_PANICTIME, panictime);
                EEPROM.commit();
              } else {
                SerialBT.println("Ongeldige waarde. Waarde moet groter zijn dan 0!");
              }
              SerialBT.println();
              break;
            }

          case 'N':
          case 'n': {
              String name = inputBuffer.substring(1);
              name.replace("\n", "");
              name.replace("\r", "");
              name.toCharArray(piepername, 11);
              SerialBT.println("Piepernaam ingesteld op: " + String(piepername));
              SerialBT.println();
              EEPROM.writeString(EE_BYTE_PIEPERNAME, String(piepername));
              EEPROM.commit();
              break;
            }

          case 'T':
          case 't': {
              testmode = !testmode;
              resetCycle();
              SerialBT.println("Test mode is " + String(testmode ? "ingeschakeld." : "uitgeschakeld."));
              SerialBT.println();
              break;
            }

          case 'R':
          case 'r': {
              terrormode = !terrormode;
              SerialBT.println("Terror mode is " + String(terrormode ? "ingeschakeld." : "uitgeschakeld."));
              SerialBT.println();
              EEPROM.writeByte(EE_BYTE_TERRORMODE, terrormode);
              EEPROM.commit();
              break;
            }

          case 'M':
          case 'm': {
              byte trackNumber = inputBuffer.substring(1).toInt();
              mp3track = trackNumber;
              if (mp3track == 0) {
                SerialBT.println("Willekeurige MP3's worden afgespeeld.");
              } else {
                SerialBT.println("MP3 track nummer ingesteld op: " + String(mp3track));
              }
              SerialBT.println();
              EEPROM.writeByte(EE_BYTE_MP3TRACK, mp3track);
              EEPROM.commit();
              break;
            }

          case 'K':
          case 'k': {
              byte v = inputBuffer.substring(1).toInt();
              if (v <= 2) {
                audioMode = v;
                const char* modeName = (audioMode == 0 ? "MP3 (JQ8400)" : (audioMode == 1 ? "Morse code" : "Nokia ringtone"));
                SerialBT.println("Audiomodus ingesteld op: " + String(modeName));
                EEPROM.writeByte(EE_BYTE_AUDIOMODE, audioMode);
                EEPROM.commit();
              } else {
                SerialBT.println("Ongeldige waarde. 0=MP3, 1=Morse, 2=Ringtone.");
              }
              SerialBT.println();
              break;
            }

          case 'X':
          case 'x': {
              int slot = inputBuffer.substring(1).toInt();
              if (slot >= 1 && slot <= 10) {
                ringTrack = (byte)slot;
                EEPROM.writeByte(EE_BYTE_RINGTRACK, ringTrack);
                EEPROM.commit();
                SerialBT.println("Ringtone slot geselecteerd: " + String(ringTrack));
              } else {
                SerialBT.println("Ongeldige waarde. Slot moet tussen 1 en 10 liggen.");
              }
              SerialBT.println();
              break;
            }

          case 'Z':
          case 'z': {
              String input = inputBuffer.substring(1);
              input.replace("\n", "");
              input.replace("\r", "");
              int eq = input.indexOf('=');
              int slot = input.substring(0, eq >= 0 ? (unsigned)eq : input.length()).toInt();
              if (slot < 1 || slot > 10) {
                SerialBT.println("Gebruik: Z<n>=<RTTTL-string>  (n=1..10), of Z<n> om slot te tonen.");
                SerialBT.println("Format: <name>:<defaults>:<notes>");
                SerialBT.println("Voorbeeld: Z1=007 Tune:o=5,d=4,b=320:c,8d,8d,d,2d,c,c,c,c,8d#,8d#,...");
              } else if (eq < 0) {
                // Lees-modus
                char buf[EE_RINGSLOT_SIZE];
                if (loadRingSlot((byte)slot, buf, sizeof(buf))) {
                  SerialBT.println("Slot " + String(slot) + ": " + String(buf));
                  SerialBT.println("Duur: ~" + String(slotDurationMs(buf) / 1000UL) + " sec");
                } else {
                  SerialBT.println("Slot " + String(slot) + " is leeg");
                }
              } else {
                // Schrijf-modus. Strikt RTTTL: moet ':' bevatten, naam max 20 chars.
                String text = input.substring(eq + 1);
                if (text.length() > EE_RINGSLOT_SIZE - 1) text = text.substring(0, EE_RINGSLOT_SIZE - 1);
                int colonPos = text.indexOf(':');
                if (text.length() > 0 && colonPos < 0) {
                  SerialBT.println("Geweigerd: alleen RTTTL-format toegestaan (moet ':' bevatten).");
                  SerialBT.println("Format: <name>:<defaults>:<notes>");
                } else if (colonPos > 20) {
                  SerialBT.println("Geweigerd: naam max 20 chars (was " + String(colonPos) + ").");
                } else {
                  char buf[EE_RINGSLOT_SIZE];
                  text.toCharArray(buf, EE_RINGSLOT_SIZE);
                  saveRingSlot((byte)slot, buf);
                  EEPROM.commit();
                  SerialBT.println("Slot " + String(slot) + " ingesteld (" + String(text.length()) + " chars RTTTL)");
                  SerialBT.println("Geschatte duur: ~" + String(slotDurationMs(buf) / 1000UL) + " sec");
                }
              }
              SerialBT.println();
              break;
            }

          case 'W':
          case 'w': {
              byte v = inputBuffer.substring(1).toInt();
              if (v >= 3 && v <= 12) {
                morseWpm = v;
                SerialBT.println("Morse snelheid ingesteld op: " + String(morseWpm) + " WPM");
                EEPROM.writeByte(EE_BYTE_MORSEWPM, morseWpm);
                EEPROM.commit();
              } else {
                SerialBT.println("Ongeldige waarde. Snelheid moet tussen 3 en 12 WPM liggen.");
              }
              SerialBT.println();
              break;
            }

          case 'V':
          case 'v': {
              int v = inputBuffer.substring(1).toInt();
              if (v >= 1 && v <= 10) {
                morseAmp = (byte)v;
                SerialBT.println("Morse stap ingesteld op: " + String(morseAmp) + "/10 (= " + String(100 - morseAmp) + "% duty)");
                EEPROM.writeByte(EE_BYTE_MORSEAMP, morseAmp);
                EEPROM.commit();
              } else {
                SerialBT.println("Ongeldige waarde. Stap moet tussen 1 en 10 liggen.");
              }
              SerialBT.println();
              break;
            }

          case 'C':
          case 'c': {
              String text = inputBuffer.substring(1);
              text.replace("\n", "");
              text.replace("\r", "");
              if (text.length() > 30) text = text.substring(0, 30);
              text.toCharArray(morseText, 31);
              EEPROM.writeString(EE_BYTE_MORSETEXT, String(morseText));
              EEPROM.commit();
              SerialBT.println("Morse tekst ingesteld op: \"" + String(morseText) + "\"");
              SerialBT.println("Geschatte duur: " + String(morseDurationMs(morseText, morseWpm) / 1000UL) + " sec");
              SerialBT.println();
              break;
            }

          case 'S':
          case 's': {
              SerialBT.print("Herstarten in...");
              for (int i = 3; i > 0; i--) {
                SerialBT.print(String(i) + "...");
                delay(1000);
              }
              SerialBT.print("Go!");
              esp_restart();
              break;
            }

          case 'Q':
          case 'q': {
              // Fabrieksreset: complete NVS-wipe (EEPROM-blob + BT-pairing).
              // Voor herstel uit NVS-vol situaties (ESP_ERR_NVS_NOT_ENOUGH_SPACE
              // bij EEPROM.commit) of als de gebruiker echt schoon wil beginnen.
              SerialBT.println("FABRIEKSRESET: NVS wissen + herstart in 3 sec...");
              SerialBT.println("(alle config + BT-pairing weg, Defaultsettings draait bij boot)");
              SerialBT.flush();
              delay(3000);
              nvs_flash_erase();
              nvs_flash_init();
              esp_restart();
              break;
            }

          case 'o':
          case 'O':
            SerialBT.println("Overzicht");
            SerialBT.println("---------");
            SerialBT.println();
            SerialBT.println("Pieper naam:\t" + String(piepername));
            SerialBT.print("TX frequentie:\t");
            if (terrormode) {
              SerialBT.println("Willekeurig");
            } else {
              SerialBT.println(String(frequency / 1000) + "." + (frequency % 1000 < 10 ? "0" : "") + (frequency % 1000 < 100 ? "0" : "") + String(frequency % 1000) + "MHz");
            }
            SerialBT.print("MP3 track\t\t");
            if (mp3track == 0) {
              SerialBT.println("Willekeurig");
            } else {
              SerialBT.println(String(mp3track) + "\t" + String(file[mp3track].name));
            }
            SerialBT.println("TX AAN periode:\t" + String(txon) + " seconden.");
            SerialBT.println("TX UIT periode:\t" + String(txoff) + " seconden.");
            SerialBT.println("Panic mode:\t\tNa " + String(panictime) + " minuten.");
            {
              const char* modeName = (audioMode == 0 ? "MP3 (JQ8400)" : (audioMode == 1 ? "Morse code" : "RTTTL ringtone"));
              SerialBT.println("Audiomodus:\t" + String(modeName));
            }
            if (audioMode == 1) {
              SerialBT.println("Morse tekst:\t\"" + String(morseText) + "\"");
              SerialBT.println("Morse WPM:\t" + String(morseWpm));
              SerialBT.println("Toon stap:\t" + String(morseAmp) + "/10 (= " + String(100 - morseAmp) + "% duty)");
            } else if (audioMode == 2) {
              SerialBT.println("Slot actief:\t" + String(ringTrack));
              char buf[EE_RINGSLOT_SIZE];
              if (loadRingSlot(ringTrack, buf, sizeof(buf))) {
                char nameBuf[64];
                size_t i = 0;
                for (; buf[i] && buf[i] != ':' && i < sizeof(nameBuf) - 1; i++) nameBuf[i] = buf[i];
                nameBuf[i] = '\0';
                SerialBT.println("Naam:\t\t" + String(nameBuf));
                SerialBT.println("Duur:\t\t" + String(slotDurationMs(buf) / 1000UL) + " sec");
              } else {
                SerialBT.println("Slot " + String(ringTrack) + ":\t(leeg)");
              }
              SerialBT.println("Toon stap:\t" + String(morseAmp) + "/10");
              SerialBT.print("Slots gevuld:\t");
              bool any = false;
              for (byte s = 1; s <= 10; s++) {
                if (slotHasData(s)) {
                  if (any) SerialBT.print(", ");
                  SerialBT.print(s);
                  any = true;
                }
              }
              if (!any) SerialBT.print("(geen)");
              SerialBT.println();
            }
            SerialBT.println();
            break;

          default:
            if (inputBuffer.length() > 1) {
              SerialBT.println("Ongeldig commando. Gebruik H voor help.");
            }
            break;
        }
      }
      SerialBT.print(piepername);
      SerialBT.println("> ");
      // Forceer dat de hele response (commando-output + prompt) de TX-queue
      // uitgaat vóór handleMenu retourneert. Daarna kan de loop zonder zorg
      // direct in een blokkerende playMorse() duiken zonder dat de Windows-
      // app op de bevestiging hoeft te wachten tot de morse-sequentie klaar
      // is.
      SerialBT.flush();
      inputBuffer = "";
    }
  }
}

void Defaultsettings() {
  EEPROM.writeByte(EE_CHECK, EE_CHECKNUMBER);
  EEPROM.writeUInt(EE_UINT16_FREQUENCY, 145000);
  EEPROM.writeUInt(EE_UINT16_PANICTIME, 180);
  EEPROM.writeByte(EE_BYTE_TXOFF, 0);
  EEPROM.writeByte(EE_BYTE_TXON, 0);
  EEPROM.writeByte(EE_BYTE_MP3TRACK, 0);
  EEPROM.writeByte(EE_BYTE_TERRORMODE, 0);
  EEPROM.writeString(EE_BYTE_PIEPERNAME, "Pieper X");
  EEPROM.writeByte(EE_BYTE_AUDIOMODE, 0);
  EEPROM.writeByte(EE_BYTE_MORSEWPM, 6);
  EEPROM.writeString(EE_BYTE_MORSETEXT, "CQ");
  EEPROM.writeByte(EE_BYTE_MORSEAMP, 5);
  EEPROM.writeByte(EE_BYTE_RINGTRACK, 1);
  // 10 slots: alle gevuld met RTTTL-klassiekers via getRtDefault().
  for (byte s = 1; s <= 10; s++) {
    EEPROM.writeString(EE_BYTE_RINGSLOTS + (s - 1) * EE_RINGSLOT_SIZE, getRtDefault(s));
  }
  EEPROM.commit();
}

// Default RTTTL-string voor slot 1..10. Centraal punt zodat zowel Defaultsettings()
// (verse EEPROM) als de boot-migratie en #GETDEFAULT hetzelfde gebruiken.
const char* getRtDefault(byte slot) {
  switch (slot) {
    case 1:  return "007:o=5,d=4,b=320,b=320:c,8d,8d,d,2d,c,c,c,c,8d#,8d#,2d#,d,d,d,c,8d,8d,d,2d,c,c,c,c,8d#,8d#,d#,2d#,d,c#,c,c6,1b.,g,f,1g.";
    case 2:  return "Axel F:o=5,d=8,b=125,b=125:16g,16g,a#.,16g,16p,16g,c6,g,f,4g,d6.,16g,16p,16g,d#6,d6,a#,g,d6,g6,16g,16f,16p,16f,d,a#,2g,4p,16f6,d6,c6,a#,4g,a#.,16g,16p,16g,c6,g,f,4g,d6.,16g,16p,16g,d#6,d6,a#,g,d6,g6,16g,16f,16p,16f,d,a#,2g";
    case 3:  return "Birdy Song:o=5,d=16,b=100,b=100:g,g,a,a,e,e,8g,g,g,a,a,e,e,8g,g,g,a,a,c6,c6,8b,8b,8a,8g,8f,f,f,g,g,d,d,8f,f,f,g,g,d,d,8f,f,f,g,g,a,b,8c6,8a,8g,8e,4c";
    case 4:  return "Bolero:o=5,d=16,b=80,b=80:c6,8c6,b,c6,d6,c6,b,a,8c6,c6,a,4c6,8c6,b,c6,a,g,e,f,2g,g,f,e,d,e,f,g,a,4g,4g,g,a,b,a,g,f,e,d,e,d,8c,8c,c,d,8e,8f,4d,2g";
    case 5:  return "Colonel Bogey:o=5,d=8,b=140,b=140:g,e,4p,p,e,f,g,e6,p,e6,p,2c6,g,e,4p,p,e,f,e,g,p,g,p,2f,f,d,4p,p,d,e,f,g,e,4p,p,e,f#,e,d,g,p,e,f#,d,p,a,g.,16f#,g,a,g,f,e,d";
    case 6:  return "Entertainer:o=5,d=8,b=140,b=140:d,d#,e,4c6,e,4c6,e,2c6.,c6,d6,d#6,e6,c6,d6,4e6,b,4d6,2c6,4p,d,d#,e,4c6,e,4c6,e,2c6.,p,a,g,f#,a,c6,4e6,d6,c6,a,2d6";
    case 7:  return "Knight Rider:o=5,d=32,b=63,b=63:16e,f,e,8b,16e6,f6,e6,8b,16e,f,e,16b,16e6,4d6,8p,4p,16e,f,e,8b,16e6,f6,e6,8b,16e,f,e,16b,16e6,4f6";
    case 8:  return "Mission Impossible:o=5,d=16,b=100,b=100:32d,32d#,32d,32d#,32d,32d#,32d,32d#,32d,32d,32d#,32e,32f,32f#,32g,g,8p,g,8p,a#,p,c6,p,g,8p,g,8p,f,p,f#,p,g,8p,g,8p,a#,p,c6,p,g,8p,g,8p,f,p,f#,p,a#,g,2d,32p,a#,g,2c#,32p,a#,g,2c,p,a#4,c";
    case 9:  return "Muppet:o=5,d=8,b=250,b=250:c6,4c6,4a,4b,a,4b,4g,4p,4c6,4c6,4a,b,a,p,4g.,4p,4e,4e,4g,4f,e,4f,c6,c,d,4e,e,e,p,e,4g,2p,4c6,4c6,4a,4b,a,4b,4g,4p,4c6,4c6,4a,b,4a,4g.,4p,4e,4e,4g,4f,e,4f,c6,c,d,4e,e,4d,d,4c";
    case 10: return "Smurfs:o=5,d=4,b=200,b=200:2c6,f6.,8c6,d6,a#,2g,c6.,8a,f,a,2g,p,16g,16a,16a#,16b,2c6,f6.,8c6,d6,a#,2g,c6.,8a,a#,e,2f,p,16g,16a,16a#,16b,2c6,f6.,8c6,d6,a#,2g,c6.,8a,f,a,2g,p,16g,16a,16a#,16b,2c6,f6.,8c6,d6,a#,2g,c6.,8a,a#,e,2f.,1p";
    default: return "";
  }
}

void blinkLED(int interval) {
  if (millis() - ledMillis >= interval) {
    ledMillis = millis();
    ledState = !ledState;
    digitalWrite(pin_LED, ledState);
  }
}
