/* _____ Sterowanie elWire modul glowny. _____ v2.1

   Modul nasluchuje na porcie szeregowym komend
   i nastepnie je wysyla, po otrzymaniu odpowiedzi/bledu zwraca ja.

   Podlaczenie NRF24L01+
     1 - GND
     2 - VCC 3.3V !!! NOT 5V
     3 - CE to Arduino pin 9
     4 - CSN to Arduino pin 10
     5 - SCK to Arduino pin 13
     6 - MOSI to Arduino pin 11
     7 - MISO to Arduino pin 12
     8 - UNUSED

    Nazwa modulu: 1
    Adres modulu: 0xE8E8F0F0E1

   komendy:
   ---------
   99 - ustawienia modulu
   -- 1 zmiana adresu lokalnego 99,1,<ID>
   -- 2 restart do ustawień fabrycznych 99,2
   -- 3 restart 99,3
   ---------
   wyslijKomende(A, B, C, D);
   A - do kogo [0-4]
   B - komenda:
   - 1 - wlacz el wire numer C (domyslnie C = 1)
   - 0 - wylacz elwire numer C (domyslnie C = 1)
   - 2 - miganie, wymaga parametrow: C (czas swiecenia w ms lub wlacz/wylacz miganie(T/N) i D (czas przerwy w ms) (jeżeli C=T, wlacza zapisane* miganie; C=N wyłącza zapisane* miganie)
   - 3 - sprawdz poziom naladowania baterii, wymaga parametrow C i D lub C (C=X: BAT_X; [zwr. poziom nal. bat. X],   C=X i D=Y: BAT_X BAT_Y; [zwr. poziomy nal. bat. X i Y])
   - 4 - jasnosc PWM, C - numer elwie D - jasnosc (dokonczyc)

   miganie* - domyslne czasy to 100ms wlaczone 500ms wylaczone

   By: Oskar & Patryk 2016
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h> //printf_begin();
#include <RF24.h>
#include <RF24_config.h>
#include <EEPROM.h>

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>

uint64_t pipe;
const int CE_PIN = 9;
const int CSN_PIN = 10;
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
int ID;
const int adresID = 1 + sizeof(pipe);


//#define numUsers (sizeof(users)/sizeof(char *)) //array size
#define MAX_ID_ONLINE 40
unsigned int onLineID[MAX_ID_ONLINE];

void setup() {
  //attachInterrupt(digitalPinToInterrupt(2), sprawdzAlarm, FALLING); //przerywanie na pinie 2
  Serial.setTimeout(10);
  Serial.begin(9600);

  for (unsigned int ok = 0; ok < MAX_ID_ONLINE; ok++) {
    onLineID[ok] = 0;
  }

  EEPROM.get(0, pipe);
  EEPROM.get(adresID, ID);
  if (pipe == 0 || pipe == -1) {
    uint64_t adresPoczatkowyNRF = 0xE8E8F0F0E1LL;
    ID = 1;
    EEPROM.put(0, adresPoczatkowyNRF);  // zapisz fabryczny adres
    EEPROM.put(adresID, ID);            // zapisz fabryczne ID
    EEPROM.get(0, pipe);
  }

  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(0, 0);
  radio.disableCRC();
  radio.setAutoAck(false);

  radio.openReadingPipe(1, pipe);
  radio.startListening();
  printf_begin();
  radio.printDetails();
  //radio.get_status(); //nie wiem jak to obsluzyc
  //Serial.println("Kto zyje?");
  //wyslijPING(); //PING
}

void loop() {
  if (radio.available()) {
    Serial.println("OKOKOK!");
    int daneOdebrane[4];
    radio.read(daneOdebrane, sizeof(daneOdebrane));

    if (daneOdebrane[0] == 1) { //Informacja (0-1024) o poziomie baterii w 4 (BAT_4_232)
      Serial.print("BAT_"); Serial.print(daneOdebrane[1]); Serial.print("_"); Serial.print(daneOdebrane[2]); Serial.print("_"); Serial.print(daneOdebrane[3]); Serial.println(";");
    } else if (daneOdebrane[0] == 2) {
      Serial.print("ER_");
      switch (daneOdebrane[2]) {
        case 1: //Alarm niski poziom baterii w 2 (ER_BAT_NP_2)
          Serial.print("BAT_NP_"); Serial.print(daneOdebrane[1]); Serial.println(";");
          break;
        case 2: //Alarm blad zapisu do eepromu ADRESU
          Serial.print(daneOdebrane[1]); Serial.print("_EEPROM_ADDRESS_"); Serial.print(daneOdebrane[3]); Serial.println(";");
        //Miejsce na kolejne bledy
        case 3: //Alarm blad zapisu do eepromu PCF
          Serial.print(daneOdebrane[1]); Serial.print("_EEPROM_PCF_"); Serial.print(daneOdebrane[3]); Serial.println(";");
          //Miejsce na kolejne bledy
      }
    } else if (daneOdebrane[0] == 3) { //Wiadomosc od modulu
      if (daneOdebrane[2] == 98) { //meldunek slave o zakonczeniu inicjalizacji
        manageActiveID(true, daneOdebrane[1]); //zapisz ID
      } else if (daneOdebrane[2] == 99) {
        manageActiveID(false, daneOdebrane[1]); //usun ID
      }
      Serial.print("MSG_"); Serial.print(daneOdebrane[1]); Serial.print("_"); Serial.print(daneOdebrane[2]); Serial.print("_"); Serial.print(daneOdebrane[3]); Serial.println(";"); //Wiadomosc od 3 o tresci wiadomosc (MSG_3_WIADOMOSC) jezeli wiadomosc = 1: OK
    }
  }

  if (Serial.available()) {
    int rem, t0, t1, t2;
    rem = Serial.parseInt();
    if (rem == 99) {
      ustawienia();
    } else {
      t0 = Serial.parseInt();
      t1 = Serial.parseInt();
      t2 = Serial.parseInt();
      bool wyslano = wyslijKomende(rem, t0, t1, t2);
      //Serial.print("SEND_"); Serial.println(wyslano); //SEND_TRUE / SEND_FALSE
    }
  }
}


void wyslijPING() {
  radio.stopListening();
  radio.openWritingPipe(0xE8E8F0F000);
  int komenda[3] = {7, 0, 0};
  radio.write(komenda, sizeof(komenda));
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  int timeout = 0;
  while (!radio.available()) {
    delay(500);
    timeout += 500;
    //Serial.print("WAIT: "); Serial.print(timeout); Serial.println("ms.");
    if (timeout == 2000) {
      Serial.println("TIMEOUT!!!");
      break;
    }
  }
  //  while (radio.available()) {
  //    int daneOdebrane[4];
  //    radio.read(daneOdebrane, sizeof(daneOdebrane));
  //    Serial.println("Zapis do bufora");
  //    zameldowane += String(daneOdebrane[1], DEC) + ";";
  //    Serial.println(zameldowane);
  //    Serial.print("PONG: "); Serial.println(daneOdebrane[1]);
  //  }
}

bool wyslijKomende(int remote, int k1, int k2, int k3) {
  bool stan = true;
  int dane[3];

  radio.stopListening();
  radio.openWritingPipe(zrobAdres(remote + 224));
  dane[0] = k1;
  dane[1] = k2;
  dane[2] = k3;
  //Serial.println(remote - 1);
  if (!radio.write(dane, sizeof(dane))) {
    stan = false;
  }
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  return stan;
}

uint64_t zrobAdres(long int koncowka) {
  uint64_t accumulator = 0;

  String koncowkaObliczona = String(koncowka, HEX);
  String adres = "e8e8f0f0" + koncowkaObliczona;
  // Length (with one extra character for the null terminator)
  int str_len = adres.length() + 1;
  // Prepare the character array (the buffer)
  //char adresSpoko[30]; //nie wiem jak sprawdzic rozmiar maksymalnego adresu, a 30 dziala xD
  char adresSpoko[str_len];
  adres.toCharArray(adresSpoko, str_len);

  for (size_t i = 0 ; isxdigit((unsigned char)adresSpoko[i]) ; ++i)
  {
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

void ustawienia() {
  //Serial.println("USTAWIENIA:");
  //Serial.println("1. zmiana adresu (np. 1,<adres> 1,4)");
  //Serial.println("2. Ustawienia fabryczne;)");
  int opcja = Serial.parseInt();
  long int argument;
  switch (opcja) {
    case 1:
      //Serial.println();
      argument = Serial.parseInt();
      //Serial.print("Zmiana adresu z "); Serial.print(ID); Serial.print(" na "); Serial.print(argument);
      EEPROM.put(0, zrobAdres(argument + 224));
      EEPROM.get(0, pipe);
      EEPROM.put(adresID, argument);
      EEPROM.get(adresID, ID);
      if ((pipe == zrobAdres(argument + 224)) && (ID == argument)) {
        //pipe = zrobAdres(argument + 224); // to nie dzialalo
        Serial.println(": OK!");
        radio.stopListening();
        radio.openReadingPipe(1, pipe);
        radio.startListening();
      } else {
        Serial.println(": NIEPOWODZENIE!");
        //Serial.println("Blad zapisu do EEPROMU - ER01E");
      }
      break;
    case 2:
      //Serial.print("Przywracanie ustawien fabrycznych, czekaj...");
      for ( int i = 0 ; i < EEPROM.length() ; i++ )
        EEPROM.write(i, 0);
      Serial.println("... OK!");
      //Serial.println("RESET ZA 2s!");
      //Serial.println("");
      //delay(2000);
      //Serial.println("RESET SZMATY!");
      arduinoReset();
      //Automatyczne restart potrzebny, wykonać pogramowo (sprzętowo)
      break;
    case 3:
      Serial.println();
      Serial.println("RESET!!!");
      Serial.println();
      arduinoReset();
      break;
    case 4:
      wyslijPING();
      //Serial.println("PING WYSLANY!");
      break;
    case 5:
      Serial.println(zwrocAktywneID());
      break;
    case 6:
      argument = Serial.parseInt();
      manageActiveID(true, argument);
      break;
    case 7:
      argument = Serial.parseInt();
      manageActiveID(false, argument);
      break;
  }
}

void arduinoReset() {
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW); //Arduino umiera :c
}

String zwrocAktywneID() {
  bool zameldowaneJuz = false;
  String zameldowane = "ACT:";
  for (unsigned int husky = 0; husky < MAX_ID_ONLINE; husky++) {
    if (onLineID[husky] != 0) {
      if (zameldowaneJuz) {
        zameldowane += ",";
      }
      zameldowane += String(onLineID[husky], DEC);
      zameldowaneJuz = true;
    }
  }
  zameldowane += ";";
  return zameldowane;
}

void manageActiveID(bool funkcja, int ID) {
  if (!czyIstnieje(ID) && funkcja) {
    unsigned int k = 0;
    while (1) {
      if (k == MAX_ID_ONLINE) {
        Serial.print("ER_ZAP_ID_"); Serial.println(ID);
        return;
      } else if (onLineID[k] == 0) {
        onLineID[k] = ID;
        return;
      } else {
        k++;
      }
    }
  } else if (czyIstnieje(ID) && !funkcja) {
    unsigned int k = 0;
    while (1) {
      if (k == MAX_ID_ONLINE) {
        Serial.print("ER_USN_ID_"); Serial.println(ID);
        return;
      } else if (onLineID[k] == ID) {
        onLineID[k] = 0;
        return;
      } else {
        k++;
      }
    }
  }
}

bool czyIstnieje(unsigned int ID) {
  unsigned int k = 0;
  while (1) {
    if (k == MAX_ID_ONLINE) {
      return false;
    } else if (onLineID[k] == ID) {
      return true;
    } else {
      k++;
    }
  }
}
