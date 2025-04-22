#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <SPI.h>
#include <JQ8400_Serial.h>
#include <EEPROM.h>
#include <BluetoothSerial.h>

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
const unsigned long   bootTime = 20000;
String                inputBuffer = "";
unsigned int          frequency = 145000;
unsigned int          numberoffiles;
unsigned int          panictime = 0;
unsigned long         ledMillis = 0;
unsigned long         txonoffMillis = 0;

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

#define EE_TOTAL_CNT                24
#define EE_CHECKNUMBER              12

#define EE_UINT16_FREQUENCY         0
#define EE_UINT16_PANICTIME         4
#define EE_BYTE_TXOFF               8
#define EE_BYTE_TXON                9
#define EE_BYTE_MP3TRACK            10
#define EE_BYTE_TERRORMODE          11
#define EE_CHECK                    12
#define EE_BYTE_PIEPERNAME          13    // 10 positions

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
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

  SerialBT.println();
  SerialBT.println(String(piepername) + " opstarten..........");

  SPI.begin();
  mp3.reset();
  mp3.setVolume(30);
  numberoffiles = mp3.countFiles();

  for (int x = 1; x < numberoffiles + 1; x++) {
    mp3.playFileByIndexNumber(x);
    file[x].index = x;
    mp3.currentFileName(file[x].name, sizeof(file[x].name));
    file[x].length = mp3.currentFileLengthInSeconds();
  }

  mp3.sleep();
  txonoffMillis = millis();
  digitalWrite(pin_LED, LOW);
}

void loop() {
  if (millis() < bootTime && !config) {
    if (digitalRead(pin_Button) == LOW) {
      config = true;
      testmode = false;
      doTXEnable(0);
      SerialBT.begin(piepername);
      digitalWrite(pin_LED, HIGH);
      while (digitalRead(pin_Button) == LOW);
    }
  } else if (!bootEnd) {
    bootEnd = true;
    digitalWrite(pin_LED, HIGH);
  }

  if (config && !testmode && mp3.getStatus() == MP3_STATUS_PLAYING) doTXEnable(0);

  if ((float)millis() / (60.0 * 1000.0) >= panictime) {
    if (config) SerialBT.println("Panic mode aktief!");
    frequency = 145000;
    mp3.sleep();
    doTXEnable(1);
    pinMode(pin_Audio, OUTPUT);
    ledcAttachPin(pin_Audio, 0);
    ledcWriteTone(0, 1000);
    ledcWrite(0, 50);
    while (true) blinkLED(500);
  }

  if (millis() >= txonoffMillis + (txonoffstate ? (txon == 0 ? mp3.currentFileLengthInSeconds() : txon) : txoff) * 1000) {
    txonoffMillis = millis();
    txonoffstate = !txonoffstate;
    if (!txonoffstate && txoff != 0 && (!config || testmode)) {
      doTXEnable(0);
      if (config) SerialBT.println("TX UIT");
    }
  }

  if (txonoffstate && (!config || testmode)) {
    if (mp3.getStatus() != MP3_STATUS_PLAYING) {
      if (config) SerialBT.println("TX AAN");
      doTXEnable(1);
      playMP3(mp3track);
    }
  }

  if (config) handleMenu();
}

void doRandomFreq() {
  int randomFrequency = random(144000, 146000);
  randomFrequency = round(randomFrequency / 25.0) * 25;
  if (config) SerialBT.println("Willekeurige frequentie: " + String(randomFrequency / 1000) + "." + (randomFrequency % 1000 < 10 ? "0" : "") + (randomFrequency % 1000 < 100 ? "0" : "") + String(randomFrequency % 1000) + "MHz");
  setFreq(randomFrequency);
}

void doTXEnable(bool status) {
  if (status) {
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
  mp3.play();
  previousTrack = track;
}

void handleMenu() {
  if (!testmode) blinkLED(1000);

  if (SerialBT.available()) {
    char input = SerialBT.read();
    inputBuffer += input;

    if (input == '\n' || input == '\r') {
      if (!inputBuffer.isEmpty()) {
        switch (inputBuffer[0]) {
          case 'h':
          case 'H':
          case '?':
            SerialBT.println("Pieper menu");
            SerialBT.println("-----------");
            SerialBT.println();
            SerialBT.println("?\t\tToon menu.");
            SerialBT.println("F<freq in kHz>\tStel de frequentie in kHz in.");
            SerialBT.println("A<sec>\t\tTijd in sec., dat de zender uit de lucht is.");
            SerialBT.println("B<sec>\t\tTijd in sec., dat de zender in de lucht is. 0 voor MP3 lengte.");
            SerialBT.println("P<min>\t\tPaniektijd in min. (Hierna Permanente TX op 145.0MHz met toon.");
            SerialBT.println("N<naam>\t\tPiepernaam (max 10 characters).");
            SerialBT.println("T\t\tTestmode met huidige configuratie.");
            SerialBT.println("R\t\tActiveer terror mode, elke TX een andere frequentie xD.");
            SerialBT.println("M<track>\tAf te spelen MP3 track. (0 voor willekeurig).");
            SerialBT.println("L\t\tLijst met beschikbare MP3 bestanden.");
            SerialBT.println("O\t\tOverzicht van huidige configuratie.");
            SerialBT.println("S\t\tHerstart de pieper.");
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
              SerialBT.print("\t");
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
            SerialBT.print("MP3 track\t");
            if (mp3track == 0) {
              SerialBT.println("Willekeurig");
            } else {
              SerialBT.println(String(mp3track) + "\t" + String(file[mp3track].name));
            }
            SerialBT.println("TX AAN periode:\t" + String(txon) + " seconden.");
            SerialBT.println("TX UIT periode:\t" + String(txoff) + " seconden.");
            SerialBT.println("Panic mode:\tNa " + String(panictime) + " minuten.");
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
  EEPROM.commit();
}

void blinkLED(int interval) {
  if (millis() - ledMillis >= interval) {
    ledMillis = millis();
    ledState = !ledState;
    digitalWrite(pin_LED, ledState);
  }
}
