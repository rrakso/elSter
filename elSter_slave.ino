/* _____ Sterowanie elWire modul niewolnika. _____ v2.0
 *
 * Modul nasluchuje na transmiterze komend
 * i nastepnie je odbiera, i interpretuje.
 *
 * Podlaczenie NRF24L01+
 *   1 - GND
 *   2 - VCC 3.3V !!! NOT 5V
 *   3 - CE to Arduino pin 8
 *   4 - CSN to Arduino pin 7
 *   5 - SCK to Arduino pin 13
 *   6 - MOSI to Arduino pin 11
 *   7 - MISO to Arduino pin 12
 *   8 - UNUSED
 *
 *  Nazwa m modulu: 0xE8E8F0F0E2
 *odulu: 2
 *  Adres
 * By: Oskar & Patryk 2016
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h> //printf_begin();
#include <RF24.h>
#include <RF24_config.h>
#include <EEPROM.h>
#include <Wire.h>


#include <stdio.h>  //do generowania adresow HEX
#include <stdint.h> //do generowania adresow HEX
#include <ctype.h>  //do generowania adresow HEX

/////////////USTAWIENIA///////////////
const int CE_PIN = 6;
const int CSN_PIN = 5;
//const int CE_PIN = 9;
//const int CSN_PIN = 8;

const int ledInfo = 4;

int elWire[2] = {A4, A5};
int bateria[2] = {A2, A1};

int miganieS = 500; //domyslnie przerwa w swieceniu w ms
int miganieN = 100; //domyslnie czas swiecenia w ms
//////////////////////////////////////

int master = 1;
uint64_t pipe;
bool PCF;
int ID;
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

bool usrBttnZ = false;

const int adresID = 1 + sizeof(pipe);
const int adresPCF = 1 + sizeof(pipe) + sizeof(adresID);

bool PCFaktywny[8] = {0, 0, 0, 0, 0, 0, 0, 0};
const int PCFadres[8] = {0x38, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
byte PCF_DDR[8];
byte PCF_PORT[8];

void setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), usrBttnZmienna, RISING);
  attachInterrupt(digitalPinToInterrupt(3), ostatnieSlowo, FALLING);
  pinMode(ledInfo, OUTPUT);
  for (int i = 0; i < 2; i++) {
    pinMode(elWire[i], OUTPUT);
  }
  for (int i = 0; i < 2; i++) {
    pinMode(bateria[i], INPUT);
  }

  EEPROM.get(0, pipe);
  if (pipe == 0 || pipe == -1) {
    master = 1;
    uint64_t adresPoczatkowyNRF = 0xE8E8F0F0E2LL; //Adres 0xE8E8F0F0E2LL czyli 2
    ID = 2;
    PCF = false;
    EEPROM.put(0, adresPoczatkowyNRF);  // zapisz fabryczny adres
    EEPROM.put(adresID, ID);            // zapisz fabryczne ID
    EEPROM.put(adresPCF, PCF);          // zapisz fabryczny stan funkcji PCF
  }
  EEPROM.get(0, pipe);
  EEPROM.get(adresID, ID);
  EEPROM.get(adresPCF, PCF);

  digitalWrite(ledInfo, HIGH);
  delay(200);
  digitalWrite(ledInfo, LOW);
  delay(100);

  //Wire.begin();
  radio.begin();

  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(0, 0);
  radio.disableCRC();
  radio.setAutoAck(false);

  radio.openReadingPipe(1, pipe);
  radio.openReadingPipe(2, 0xE8E8F0F000); //broadcast
  radio.startListening();
  //printf_begin();
  //radio.printDetails();

  wyslijKomende(1, 3, ID, 97, PCF);
  wyslijKomende(1, 3, ID, 98, pipe);
}

void loop() {
  if (usrBttnZ) {
    detachInterrupt(digitalPinToInterrupt(2));
    usrBttn();
    attachInterrupt(digitalPinToInterrupt(2), usrBttnZmienna, RISING);
  }
  if (radio.available()) {
    int daneOdebrane[3];
    radio.read(daneOdebrane, sizeof(daneOdebrane));

    if ((daneOdebrane[0] == 1 && !PCF) || (daneOdebrane[0] == 0 && !PCF)) {
      digitalWrite(elWire[daneOdebrane[1] - 1], daneOdebrane[0]); //Ustaw stan HIGH/LOW na pinie elWire[x] np. 1,3 / 0,3; 1,x / 0,x

    } else if ((daneOdebrane[0] == 1 && PCF) || (daneOdebrane[0] == 0 && PCF)) {
      if (PCFaktywny[daneOdebrane[1] - 1]) {
        jebanyPCF(daneOdebrane[1], daneOdebrane[2], daneOdebrane[0]);
      } else {
        for (int o = 0; o < 8; o++)
          jebanyPCFjako(daneOdebrane[1], o, OUTPUT);
        PCFaktywny[daneOdebrane[1] - 1] = true;
        jebanyPCF(daneOdebrane[1], daneOdebrane[2], daneOdebrane[0]);
      }

    } else if (daneOdebrane[0] == 2) {
      if (daneOdebrane[1] == 5 && daneOdebrane[1] != 4) {
        miganie(daneOdebrane[2]); //miganie, moze migac kilka paskow
      } else {
        miganieS = daneOdebrane[1]; //ustaw czas przerwy w swieceniu
        miganieN = daneOdebrane[2]; //ustaw czas swiecenia
        //wyslijKomende(0, 3, slaveNAME, 1); //OK
      }

    } else if (daneOdebrane[0] == 3) {
      int pozNalBat1, pozNalBat2;
      if (daneOdebrane[2] == 0) {
        pozNalBat1 = analogRead(bateria[daneOdebrane[1] - 1]); //odczytaj poziom naladowania baterii bateria[x]
        wyslijKomende(1, 1, ID, 1, pozNalBat1);
      } else {
        pozNalBat1 = analogRead(bateria[daneOdebrane[1] - 1]); //odcztaj poziomy naladowania dwoch baterii bateria[x] bateria[y]
        pozNalBat2 = analogRead(bateria[daneOdebrane[2] - 1]);
        wyslijKomende(1, 1, ID, 1, pozNalBat1);
        wyslijKomende(1, 1, ID, 2, pozNalBat2);
      }

    } else if (daneOdebrane[0] == 4) { // zmiana adresu KM: 4,<nowy adres>[1-6]
      EEPROM.put(0, zrobAdres(daneOdebrane[1] + 224));
      EEPROM.get(0, pipe);
      EEPROM.put(adresID, daneOdebrane[1]);
      EEPROM.get(adresID, ID);
      if ((pipe == zrobAdres(daneOdebrane[1] + 224)) && (ID == daneOdebrane[1])) {
        wyslijKomende(master, 3, ID, 4, 1); //powiadom o prawidlowym wykoananiu komendy przesylajac swoje ID
      } else {
        wyslijKomende(master, 2, ID, 2, -1); // -1 powiadom o bledzie
      }

    } else if (daneOdebrane[0] == 5) { //wlaczenie/wylaczenie obslugi PCF8574 KM: 5,1 / 5,0
      bool pcfTMP;
      if (daneOdebrane[1] > 0) {//ogranicza wyobraznie Patyka
        PCF = true;
      } else {
        PCF = false;
      }
      EEPROM.put(adresPCF, PCF);
      EEPROM.get(adresPCF, pcfTMP);
      if (pcfTMP == PCF) {
        wyslijKomende(master, 3, ID, 5, PCF); //powiadom o prawidlowym wykoananiu komendy przesylajac stan funkcji PCF
        wyslijKomende(master, 3, ID, 8, 1); //powiadom o resecie
        powerReset();
      } else {
        wyslijKomende(master, 2, ID, 3, -1); //-1 powiadom o bledzie
      }

    } else if (daneOdebrane[0] == 6) { //wymazywanie danych z EEPROMu / ustawienia fabryczne KM: 6
      przywrocUstawieniaFabryczne();

    } else if (daneOdebrane[0] == 7) { //ping
      wyslijKomende(master, 3, ID, 98, pipe);
      digitalWrite(ledInfo, HIGH);
      delay(500);
      digitalWrite(ledInfo, LOW);

    } else if (daneOdebrane[0] == 8) { //PowerReset
      wyslijKomende(master, 3, ID, 8, 1); //powiadom o resecie
      powerReset();

    } else if (daneOdebrane[0] == 9) {
      wyslijKomende(master, 3, ID, 9, 1);
      digitalWrite(ledInfo, HIGH);
      delay(2000);
      digitalWrite(ledInfo, LOW);
      delay(2000);
      digitalWrite(ledInfo, HIGH);
      delay(2000);
      digitalWrite(ledInfo, LOW);
    }
  }
}

bool wyslijKomende(int remote, int k1, int k2, int k3, int k4) {
  bool stan = true;
  int dane[4];
  radio.stopListening();
  radio.openWritingPipe(zrobAdres(remote + 224));
  dane[0] = k1;
  dane[1] = k2;
  dane[2] = k3;
  dane[3] = k4;
  if (!radio.write(dane, sizeof(dane))) {
    stan = false;
  }
  radio.openReadingPipe(1, pipe);
  radio.openReadingPipe(2, 0xE8E8F0F000); //broadcast
  radio.startListening();
  return stan;
}

uint64_t zrobAdres(long int koncowka) {
  uint64_t accumulator = 0;
  String koncowkaObliczona = String(koncowka, HEX);
  String adres = "e8e8f0f0" + koncowkaObliczona;
  //Dlugosc (+ jeden char dla null)
  int str_len = adres.length() + 1;
  //przygotuj char array (bufor)
  //char adresSpoko[30]; //nie wiem jak sprawdzic rozmiar maksymalnego adresu, a 30 dziala xD
  char adresSpoko[str_len];
  adres.toCharArray(adresSpoko, str_len);

  for (size_t i = 0 ; isxdigit((unsigned char)adresSpoko[i]) ; ++i) {
    char c = adresSpoko[i];
    accumulator *= 16;
    if (isdigit(c)) /* '0' .. '9'*/
      accumulator += c - '0';
    else if (isupper(c)) /* 'A' .. 'F'*/
      accumulator += c - 'A' + 10;
    else /* 'a' .. 'f'*/
      accumulator += c - 'a' + 10;

  }
  return accumulator;
}

void powerReset() {
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);
}

void miganie(int ile) {
  if (radio.available()) {
    int daneOdebrane[3];
    radio.read(daneOdebrane, sizeof(daneOdebrane));
    if (daneOdebrane[0] == 2) {
      if (daneOdebrane[1] != 5 && daneOdebrane[1] == 4) {
        return; //przerwi miganie
      } else {
        miganieS = daneOdebrane[1]; //ustaw czas swiecenia
        miganieN = daneOdebrane[2]; //ustaw czas przerwy w swieceniu
      }
    }
  }
  for (int i = 0; i < ile - 1; i++) {
    digitalWrite(elWire[i], HIGH);
  }
  delay(miganieS);
  for (int i = 0; i < ile - 1; i++) {
    digitalWrite(elWire[i], LOW);
  }
  delay(miganieN);
}

void jebanyPCF(uint8_t ID, uint8_t pin, uint8_t stan) {
  byte _PIN;
  --ID;
  uint8_t adres = PCFadres[ID];

  /* Set PORT bit value */
  if (stan)
    PCF_PORT[ID] |= (1 << pin);
  else
    PCF_PORT[ID] &= ~(1 << pin);

  /* Start request, wait for data and receive GPIO values as byte */
  Wire.requestFrom(adres, (uint8_t) 0x01);
  while (Wire.available() < 1)
    ;
  _PIN = Wire.read();

  /* Update GPIO values */
  /* Compute new GPIO states */
  //uint8_t value = ((_PIN & ~PCF_DDR[addres]) & ~(~PCF_DDR[addres] & PCF_PORT[addres])) | PCF_PORT[addres]; // Experimental
  uint8_t value = (_PIN & ~PCF_DDR[ID]) | PCF_PORT[ID];
  /*Serial.println("jebany");
  Serial.println(value, BIN);
  Serial.println("pin");
  Serial.println(_PIN, BIN);*/

  /* Start communication and send GPIO values as byte */
  Wire.beginTransmission(adres);
  Wire.write(value);
  Wire.endTransmission();
}

void jebanyPCFjako(uint8_t ID, uint8_t pin, uint8_t mode) {
  byte _PIN;
  --ID;
  uint8_t adres = PCFadres[ID];

  /* Switch according mode */
  switch (mode) {
    case INPUT:
      PCF_DDR[ID] &= ~(1 << pin);
      PCF_PORT[ID] &= ~(1 << pin);
      break;

    case INPUT_PULLUP:
      PCF_DDR[ID] &= ~(1 << pin);
      PCF_PORT[ID] |= (1 << pin);
      break;

    case OUTPUT:
      PCF_DDR[ID] |= (1 << pin);
      PCF_PORT[ID] &= ~(1 << pin);
      break;

    default:
      break;
  }

  /* Start request, wait for data and receive GPIO values as byte */
  Wire.requestFrom(adres, (uint8_t) 0x01);
  while (Wire.available() < 1)
    ;
  _PIN = Wire.read();

  /* Update GPIO values */
  /* Compute new GPIO states */
  //uint8_t value = ((_PIN & ~PCF_DDR[addres]) & ~(~PCF_DDR[addres] & PCF_PORT[addres])) | PCF_PORT[addres]; // Experimental
  uint8_t value = (_PIN & ~PCF_DDR[ID]) | PCF_PORT[ID];
  /*Serial.println("JAKO");
  Serial.println(value, BIN);
  Serial.println("pin");
  Serial.println(_PIN, BIN);*/

  /* Start communication and send GPIO values as byte */
  Wire.beginTransmission(adres);
  Wire.write(value);
  Wire.endTransmission();
}

void ostatnieSlowo() {
  wyslijKomende(master, 3, ID, 99, 1);
}

void usrBttnZmienna() {
  usrBttnZ = true;
}

void usrBttn() {
  int blink = 0;
  //delay(200);
  while (digitalRead(2)) { //migaj, zlicz ilosc migniec
    digitalWrite(ledInfo, HIGH);
    delay(250);
    digitalWrite(ledInfo, LOW);
    delay(250);
    blink++;
  }
  
  switch (blink) {
    case 2:
      wyslijKomende(master, 3, ID, 98, pipe);
      digitalWrite(ledInfo, HIGH);
      delay(500);
      digitalWrite(ledInfo, LOW);
      break;
    case 5:
      delay(250);
      while (!digitalRead(2)) { //migaj szybko aby potwierdzic wymazanie danych
        digitalWrite(ledInfo, HIGH);
        delay(250);
        digitalWrite(ledInfo, LOW);
        delay(150);
      }
      float t0, czas;
      t0 = millis();
      while (digitalRead(2)) { //przycisniecie przycisku potwierdzi wymazanie, przytrzymanie przycisku powyzej 3s anuluje wymazanie
        digitalWrite(ledInfo, HIGH);
        czas = millis() - t0;
        if (czas > 3000) {
          digitalWrite(ledInfo, LOW);
          delay(850);
          exit;
        }
      }
      if (czas < 3000) {
        digitalWrite(ledInfo, LOW);
        delay(600);
        digitalWrite(ledInfo, HIGH);
        delay(100);
        digitalWrite(ledInfo, LOW);
        delay(600);
        digitalWrite(ledInfo, HIGH);
        delay(100);
        digitalWrite(ledInfo, LOW);
        przywrocUstawieniaFabryczne();
      }
      break;
  }
  usrBttnZ = false;
}

void przywrocUstawieniaFabryczne() {
  wyslijKomende(master, 3, ID, 6, 0); //powiadom o rozpoczęciu kasowania pamięci
  for ( int i = 0 ; i < EEPROM.length() ; i++ )
    EEPROM.write(i, 0);
  wyslijKomende(master, 3, ID, 6, 1); //powiadom o zakonczeniu kasowania pamięci
  powerReset();
}

