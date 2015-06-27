// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// created by Adam, chickenhouse@louisenhof2.de, 2015
// https://github.com/quantenschaum/chickenhouse

// configuration
// (*) = comment out to disable the feature
#define WEBTIME "nas" // hostname of http server to pull the time from (*)
#define WEB_PASSWD "YWRtaW46YWRtaW4=" // base64 of admin:admin
#define TIME_ADJUST 3600000ul // timeout in ms after which the time is pulled from the net
#define WATCHDOG WDTO_8S // watchdog timeout (*)
#define USEDHT DHT22 // type of DHT sensor (*)
#define FREEMEM // show free memory in (*)
#define BUFLEN 32 // buffer size for http server
#define NAME_VALUE_LEN 16 // buffer size for http server query parser
#define MAC_ADDRESS {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}
#define IP_ADDRESS ip(192, 168, 222, 201)
//#define USE_SERIAL // enable serial communication (*)

// disable reset on open tty
// stty -F /dev/ttyUSB0 115200 cs8 cread clocal -hupcl

// IO pins
// 11, 12, 13 are used by SPI
// 10 is used as CS for the ethernet shield
// usable digital pins: 0 1 2 3 4 5 6 7 8 9 (0 1 if no Serial is used)
// usable  analog pins: 0 1 2 3 4 5 (6 7 on Nano)

// outputs to operate the relais
#define UP           A0
#define DOWN         A1
#define LIGHT        A2
#define HEATER       A3
#define EXTRA1       A4
#define EXTRA2       A5

// analog inputs
#define BRIGHTNESS   A6
#define EXTRA3       A7

// digital inputs
#define UPPER        8
#define LOWER        7
#define TOGGLE       6
#define DOOR         5
#define DHTPIN       3


// constants
#define ON LOW
#define OFF HIGH
#define UNDEFINED 0
#define DAY 1
#define NIGHT -1
#define OPEN 1
#define STOP 0
#define CLOSE -1
#define SEC 1000ul


// includes
#include <SPI.h>
#include <Ethernet.h>
//#include <EthernetDHCP.h>
//#define WEBDUINO_SERIAL_DEBUGGING 1
#include <WebServer.h>
#include <Streaming.h>
#include <EEPROM.h>
#include "eeprom.h"
#include "state.h"

#if defined(WEBTIME)
#include "webtime.h"
#endif

#if defined(WATCHDOG)
#include <avr/wdt.h>
#endif

#if defined(FREEMEM)
#include <MemoryFree.h>
#endif

#if defined(USEDHT)
#include <DHT.h>
#endif


// global variables
byte mac[] = MAC_ADDRESS;
IPAddress IP_ADDRESS;

WebServer webserver("", 80);

unsigned long timer = 0;

#if defined(WEBTIME)
float day_hh, night_hh;
#endif

#if defined(USEDHT)
DHT dht(DHTPIN, USEDHT);
float temp = NAN, humi = NAN;
float cold_thres;
int temp_delay;
State cold(0);
#endif

float bright = 0;
int day_thres, night_thres;
int day_delay, up_timeout, down_timeout;
boolean locked = 1;

State toggle(0);
State day(0);
State hatch_state(0), hatch_moving(0), hatch_sensed(0);
State door(0);


void(* softReset) (void) = 0; //declare reset function at address 0

void turn(uint8_t pin, uint8_t i) {
  digitalWrite(pin, i ? ON : OFF);
}

uint8_t isOn(uint8_t pin) {
  return (digitalRead(pin) == ON) ? 1 : 0;
}

void hatch(int x) {
  turn(UP, 0);
  turn(DOWN, 0);
  if (x != STOP) {
    turn(x > STOP ? UP : DOWN, 1);
  }
  hatch_moving.set(x);
}


int hatch_sense() {
  boolean lower = isOn(LOWER);
  boolean upper = isOn(UPPER);
  if (upper && !lower)
    return 1;
  else if (lower && !upper)
    return -1;
  else
    return 0;
}

// from 0 to 100
float brightness() {
  return analogRead(BRIGHTNESS) * 0.097751711f;
}


void printdata(Print &s) {
  unsigned long t = millis();
  s << F("# chickenhouse ")    << F(__DATE__) << endl;
  s << F("uptime=")     << (t / SEC) << endl;
#if defined(FREEMEM)
  s << F("freemem=") << freeMemory() << endl;
#endif
#if defined(WEBTIME)
  s << F("time=")    << unixTime(t)
    << F(" # ")    << hours(t) << F(":")    << minutes(t)
    << F(" (")    << fhours(t) << F(")") << endl;
#endif
  s << F("brightness=")    << bright << endl;
#if defined(USEDHT)
  s << F("temp=")    << temp << endl;
  s << F("humi=")    << humi << endl;
#endif
  s << F("day=")    << day.get() << endl;
  s << F("hatch_sense=")    << hatch_sensed.get() << endl;
  s << F("hatch_state=")    << hatch_state.get() << endl;
  s << F("hatch_moving=")    << hatch_moving.get() << endl;
#if defined(USEDHT)
  s << F("cold=")    << cold.get() << endl;
#endif
  s << F("heater=")    << isOn(HEATER) << endl;
  s << F("door=")    << door.get() << endl;
  s << F("light=")    << isOn(LIGHT) << endl;
  s << F("extra1=")      << isOn(EXTRA1)  << endl;
  s << F("extra2=")      << isOn(EXTRA2)  << endl;
  s << F("extra3=")      << analogRead(EXTRA3) << endl;
#if defined(WEBTIME)
  s << F("day_hh=")    << day_hh << endl;
  s << F("night_hh=")    << night_hh << endl;
#endif
  s << F("day_thres=")    << day_thres << endl;
  s << F("night_thres=")    << night_thres << endl;
  s << F("locked=")    << locked << endl;
  s << F("day_delay=")    << day_delay << endl;
  s << F("up_timeout=")    << up_timeout << endl;
  s << F("down_timeout=")    << down_timeout << endl;
#if defined(USEDHT)
  s << F("temp_delay=")    << temp_delay << endl;
  s << F("cold_thres=")    << cold_thres << endl;
#endif
}


#if defined(USE_SERIAL)
void processinput() {
  char buf[BUFLEN + 1];
  int n = Serial.readBytesUntil('\n', buf, BUFLEN);
  buf[n] = 0;

  if (process(webserver, buf))
    printdata(Serial);
  else
    Serial << F("ERROR") << endl;

  while (Serial.available())
    Serial.read();
}
#endif

void read_settings() {
  day_thres = read_int(0);
  night_thres = read_int(2);
  day_delay = read_int(4);
  up_timeout = read_int(6);
  down_timeout = read_int(8);
#if defined(USEDHT)
  temp_delay = read_int(12);
  cold_thres = read_float(14);
#endif
#if defined(WEBTIME)
  day_hh = read_float(18);
  night_hh = read_float(22);
#endif
}


boolean equals(char* a, char* b) {
  return strcmp(a, b) == 0;
}

boolean process(WebServer &server, char* query) {
  URLPARAM_RESULT rc;
  char name[NAME_VALUE_LEN + 1];
  char value[NAME_VALUE_LEN + 1];
  while (strlen(query)) {
    rc = server.nextURLparam(&query, name, NAME_VALUE_LEN, value, NAME_VALUE_LEN);
    if (rc == URLPARAM_OK) {
      int i = atoi(value);
#if defined(WEBTIME)
      if (equals(name, "day_hh")) {
        float f = atof(value);
        write_float(18, f);
      } else if (equals(name, "night_hh")) {
        float f = atof(value);
        write_float(22, f);
      } else
#endif
        if (equals(name, "day_thres")) {
          write_int(0, i);
        } else if (equals(name, "night_thres")) {
          write_int(2, i);
        } else if (equals(name, "day_delay")) {
          write_int(4, i);
        } else if (equals(name, "up_timeout")) {
          write_int(6, i);
        } else if (equals(name, "down_timeout")) {
          write_int(8, i);
        } else
#if defined(USEDHT)
          if (equals(name, "temp_delay")) {
            write_int(12, i);
          } else if (equals(name, "cold_thres")) {
            float f = atof(value);
            write_float(14, f);
          } else
#endif
            if (equals(name, "light")) {
              turn(LIGHT, i);
            } else if (equals(name, "heater")) {
              turn(HEATER, i);
            } else if (equals(name, "extra1")) {
              turn(EXTRA1, i);
            } else if (equals(name, "extra2")) {
              turn(EXTRA2, i);
            } else if (equals(name, "hatch")) {
              if (!hatch_moving.get()) {
                if (i == OPEN && !hatch_state.is(OPEN)
                    && (!locked || (day.is(DAY) && day.age() > day_delay)))
                  hatch(i);
                if (i == CLOSE && hatch_state.is(OPEN))
                  hatch(i);
              } else if (i == STOP) {
                hatch(i);
              }
            } else if (equals(name, "locked")) {
              locked = i;
            } else if (equals(name, "reset")) {
              if (i == 1234) {
                delay(3000);
                softReset();
              }
            } else {
              return false;
            }
      read_settings();
    } else if (rc != URLPARAM_EOS) {
      return false;
    }
  }
  return true;
}

void rest(WebServer &server, WebServer::ConnectionType type, char* query, bool complete) {
  if (!complete) {
    server.httpFail();
    server << F("query overflow");
    return;
  }

  if (type == WebServer::HEAD)
    return;

  boolean ok = 1;

  if (strlen(query)) {
    if (server.checkCredentials(WEB_PASSWD))
      ok = process(webserver, query);
    else {
      server.httpUnauthorized();
      return;
    }
  }

  if (!ok) {
    server.httpFail();
    return;
  }

  server.httpSuccess("text/plain", "Cache-Control: no-cache, no-store, must-revalidate\r\nPragma: no-cache\r\nExpires: 0\r\n");
  printdata(server);
}


void setup() {
#if defined(WATCHDOG)
  wdt_disable();
  wdt_enable(WATCHDOG);
#endif
#if defined(USE_SERIAL)
  Serial.begin(115200);
  Serial.setTimeout(500);
#endif
  pinMode(UP, OUTPUT);
  pinMode(DOWN, OUTPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(LIGHT, OUTPUT);
  pinMode(EXTRA1, OUTPUT);
  pinMode(EXTRA2, OUTPUT);
  turn(UP, 0);
  turn(DOWN, 0);
  turn(HEATER, 0);
  turn(LIGHT, 0);
  turn(EXTRA1, 0);
  turn(EXTRA2, 0);
  pinMode(UPPER, INPUT_PULLUP);
  pinMode(LOWER, INPUT_PULLUP);
  pinMode(TOGGLE, INPUT_PULLUP);
  pinMode(DOOR, INPUT_PULLUP);

  read_settings();

  hatch_state.set(hatch_sense());
  float b = brightness();
  day.set(b < night_thres ? NIGHT : b > day_thres ? DAY : UNDEFINED);
  day.changed();

#if defined(DHT)
  dht.begin();
#endif

#if defined(WEBTIME)
  tset = TIME_ADJUST;
#endif

  Ethernet.begin(mac, ip);
  webserver.setDefaultCommand(&rest);
  webserver.begin();
}



void loop() {
  hatch_sensed.set(hatch_sense());
  toggle.set(isOn(TOGGLE));

  door.set(isOn(DOOR));
  if (door.age() > SEC && door.changed()) {
    turn(LIGHT, door.get());
  }

#if defined(USE_SERIAL)
  if (Serial.available())
    processinput();
#endif

  // Ethernet.maintain();

  int len = BUFLEN;
  char buf[BUFLEN + 1];
  webserver.processConnection(buf, &len);

  unsigned long t = millis();

  int moving = hatch_moving.get();
  if (moving != STOP) {
    if (moving == hatch_sensed.get() && hatch_sensed.age() > 100) {
      hatch(STOP);
      hatch_state.set(hatch_sensed.get());
    } else {
      unsigned long age = hatch_moving.age();
      if (moving > STOP && age > up_timeout * SEC) {
        hatch(STOP);
        hatch_state.set(UNDEFINED);
      }
      if (moving < STOP && age > down_timeout * SEC) {
        hatch(STOP);
        hatch_state.set(CLOSE);
      }
    }
  } else {

#if defined(WEBTIME)
    if (t - tset > TIME_ADJUST) {
      adjustTime();
    }
#endif

    // read sensors
    if (t - timer > 5 * SEC) {
      timer = t;
#if defined(USEDHT)
      temp = dht.readTemperature();
      humi = dht.readHumidity();
#endif
      bright = 0.9 * bright + 0.1 * brightness();
      locked = 1;
#if defined(WEBTIME)
      float h = fhours(t);
      boolean daytime = toffset == 0 || (h > day_hh && h < night_hh);
#else
      boolean daytime = true;
#endif
      day.set((bright < night_thres || !daytime) ? NIGHT : (bright > day_thres && daytime) ? DAY : UNDEFINED);
#if defined(USEDHT)
      cold.set(temp < cold_thres);
      if (cold.age() > temp_delay * SEC && cold.changed()) {
        turn(HEATER, cold.get());
      }
#endif
    }

    if (toggle.get() && toggle.age() > SEC && toggle.changed()) {
      if (!locked || (day.is(DAY) && day.age() > day_delay) || hatch_state.is(OPEN)) {
        hatch(hatch_state.is(OPEN) ? CLOSE : OPEN);
      }
    } else {
      if (day.age() > day_delay && day.changed()) {
        int hs = hatch_state.get();
        if (day.is(DAY)) { // DAY
          if (hs <= UNDEFINED) // if closed/undefined
            hatch(OPEN); // open
        } else if (day.is(NIGHT)) { // NIGHT
          if (hs == UNDEFINED) { // if undefined
            hatch(OPEN); // open first
            day.latch = 1; // relatch to close
          } else { // then
            if (hs == OPEN) // if open
              hatch(CLOSE); // close
          }
        }
      }
    }
  }

#if defined(WATCHDOG)
  wdt_reset();
#endif
}
