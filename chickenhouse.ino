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

// configuration
//#define WEBTIME "nas"
#define WATCHDOG WDTO_8S
//#define ONEWIRE
#define USEDHT DHT22
#define TIME_ADJUST 3600000ul
#define FREEMEM
#define BUFLEN 32
#define NAME_VALUE_LEN 16
#define MAC_ADDRESS { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }

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

// busses
#define ONE_WIRE_BUS 4
#define DHTPIN       3


// constants
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

#if defined(WEBTIME)
#include "webtime.h"
#endif

#if defined(WATCHDOG)
#include <avr/wdt.h>
#endif

#if defined(FREEMEM)
#include <MemoryFree.h>
#endif

#if defined(ONEWIRE)
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

#if defined(USEDHT)
#include <DHT.h>
#endif


byte mac[] = MAC_ADDRESS;

IPAddress ip(192, 168, 222, 201);

WebServer webserver("", 80);

unsigned long timer = 0;

#if defined(WEBTIME)
float day_hh, night_hh;
#endif

#if defined(ONEWIRE)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temp2 = NAN;
#endif

#if defined(USEDHT)
DHT dht(DHTPIN, USEDHT);
float temp = NAN, humi = NAN;
#endif

float bright = 0;
int day_thres, night_thres;
int day_delay, up_timeout, down_timeout, temp_delay;
float cold_thres;
boolean locked = 1;

void(* softReset) (void) = 0; //declare reset function at address 0




class State {
  public:
    int state;
    boolean latch;
    unsigned long flank;
    State(int s) {
      state = s;
      latch = true;
      flank = millis();
    }
    int get() {
      return state;
    }
    boolean is(int v) {
      return state == v;
    }
    void set(int s) {
      if (s != state) {
        state = s;
        latch = true;
        flank = millis();
      }
    }
    boolean changed() {
      boolean l = latch;
      latch = false;
      return l;
    }
    unsigned long age() {
      return millis() - flank;
    }
};

State toggle(0);
State day(0);
State hatch_state(0), hatch_moving(0), hatch_sensed(0);
State cold(0), heater(0);
State door(0), light(0);

//  0 = stop, 1 = up, -1 = down
void hatch(int x) {
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  if (x != 0) {
    digitalWrite(x > 0 ? UP : DOWN, LOW);
  }
  hatch_moving.set(x);
}


//  0 = unknown, 1 = opened, -1 = closed
int hatch_sense() {
  boolean lower = digitalRead(LOWER) == LOW;
  boolean upper = digitalRead(UPPER) == LOW;
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
  s << F("uptime=")     << (t / SEC) << endl;
#if defined(WEBTIME)
  s << F("time=")    << unixTime(t) << endl;
  s << F("# ")    << hours(t) << F(":")    << minutes(t) << F(" (")    << fhours(t) << F(")") << endl;
#endif
#if defined(FREEMEM)
  s << F("freemem=") << freeMemory() << endl;
#endif
  s << F("brightness=")    << bright << endl;
#if defined(ONEWIRE)
  s << F("temp2=")    << temp2 << endl;
#endif
#if defined(USEDHT)
  s << F("temp=")    << temp << endl;
  s << F("humi=")    << humi << endl;
#endif
  s << F("day=")    << day.get() << endl;
  s << F("hatch_sense=")    << hatch_sensed.get() << endl;
  s << F("hatch_state=")    << hatch_state.get() << endl;
  s << F("hatch_moving=")    << hatch_moving.get() << endl;
  s << F("cold=")    << cold.get() << endl;
  s << F("heater=")    << heater.get() << endl;
  s << F("door=")    << door.get() << endl;
  s << F("light=")    << light.get() << endl;
  s << F("extra1=")      << (digitalRead(EXTRA1) == LOW ? 1 : 0) << endl;
  s << F("extra2=")      << (digitalRead(EXTRA2) == LOW ? 1 : 0) << endl;
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
  s << F("temp_delay=")    << temp_delay << endl;
  s << F("cold_thres=")    << cold_thres << endl;
  s << F("build_date=")    << F(__DATE__) << endl;
  //  s << F("ip=") << Ethernet.localIP() << endl;
  //  s << F("_up=")      << (digitalRead(UP) == LOW ? 1 : 0) << endl;
  //  s << F("_down=")      << (digitalRead(DOWN) == LOW ? 1 : 0) << endl;
  //  s << F("_heater=")      << (digitalRead(HEATER) == LOW ? 1 : 0) << endl;
  //  s << F("_light=")      << (digitalRead(LIGHT) == LOW ? 1 : 0) << endl;
}



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

void read_settings() {
  day_thres = read_int(0);
  night_thres = read_int(2);
  day_delay = read_int(4);
  up_timeout = read_int(6);
  down_timeout = read_int(8);
  temp_delay = read_int(12);
  cold_thres = read_float(14);
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
    //    Serial << "#" << name << "=" << value << endl;
    if (rc == URLPARAM_OK) {
      int i = atoi(value);
      if (equals(name, "day_hh")) {
        float f = atof(value);
        write_float(18, f);
      } else if (equals(name, "night_hh")) {
        float f = atof(value);
        write_float(22, f);
      } else if (equals(name, "day_thres")) {
        write_int(0, i);
      } else if (equals(name, "night_thres")) {
        write_int(2, i);
      } else if (equals(name, "day_delay")) {
        write_int(4, i);
      } else if (equals(name, "up_timeout")) {
        write_int(6, i);
      } else if (equals(name, "down_timeout")) {
        write_int(8, i);
      } else if (equals(name, "temp_delay")) {
        write_int(12, i);
      } else if (equals(name, "cold_thres")) {
        float f = atof(value);
        write_float(14, f);
      } else if (equals(name, "light")) {
        light.set(i);
        digitalWrite(LIGHT, i ? LOW : HIGH);
      } else if (equals(name, "heater")) {
        heater.set(i);
        digitalWrite(HEATER, i ? LOW : HIGH);
      } else if (equals(name, "extra1")) {
        digitalWrite(EXTRA1, i ? LOW : HIGH);
      } else if (equals(name, "extra2")) {
        digitalWrite(EXTRA2, i ? LOW : HIGH);
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
    if (server.checkCredentials("YWRtaW46YWRtaW4="))
      ok = process(webserver, query);
    else {
      server.httpUnauthorized();
      return;
    }
  }

  if (!ok) {
    server.httpFail();
    server << F("query invalid");
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

  Serial.begin(115200);
  Serial.setTimeout(500);

  pinMode(UP, OUTPUT);
  pinMode(DOWN, OUTPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(LIGHT, OUTPUT);
  pinMode(EXTRA1, OUTPUT);
  pinMode(EXTRA2, OUTPUT);
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(HEATER, HIGH);
  digitalWrite(LIGHT, HIGH);
  digitalWrite(EXTRA1, HIGH);
  digitalWrite(EXTRA2, HIGH);
  pinMode(UPPER, INPUT_PULLUP);
  pinMode(LOWER, INPUT_PULLUP);
  pinMode(TOGGLE, INPUT_PULLUP);
  pinMode(DOOR, INPUT_PULLUP);

  read_settings();

  hatch_state.set(hatch_sense());
  float b = brightness();
  day.set(b < night_thres ? NIGHT : b > day_thres ? DAY : UNDEFINED);
  day.changed();

#if defined(ONEWIRE)
  sensors.begin();
  sensors.setWaitForConversion(true);
  sensors.setResolution(10);
#endif

#if defined(DHT)
  dht.begin();
#endif

#if defined(WEBTIME)
  tset = TIME_ADJUST;
#endif

  Ethernet.begin(mac, ip);
  webserver.setDefaultCommand(&rest);
  webserver.begin();
  //  Serial.println("UP");
}



void loop() {
  hatch_sensed.set(hatch_sense());
  toggle.set(digitalRead(TOGGLE) == LOW);

  door.set(digitalRead(DOOR) == LOW);
  if (door.age() > SEC && door.changed()) {
    light.set(door.get());
    digitalWrite(LIGHT, door.get() ? LOW : HIGH);
  }

  // serial communication
  if (Serial.available())
    processinput();

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
#if defined(ONEWIRE)
      sensors.requestTemperatures();
      temp2 = sensors.getTempCByIndex(0);
      temp2 = temp2 == -127 ? NAN : temp2;
#endif
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
        heater.set(cold.get());
        digitalWrite(HEATER, cold.get() ? LOW : HIGH);
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















































