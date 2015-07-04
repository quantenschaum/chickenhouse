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
#define WEBTIME // use time (*)
#define MIN_TIME 1436012692 // minimum valid time (UTC, secs sincs epoch) 
#define TIMEZONE 0 // offset in hours
//#define NTP_SERVER ntpIp(192, 168, 222, 201) // use gateway IP if undefined (*)
#define TIME_RESYNC 3600ul // timeout in s after which the time is pulled from the net
#define WEB_PASSWD "YWRtaW46YWRtaW4=" // base64 of admin:admin
#define WATCHDOG WDTO_8S // watchdog timeout (*)
#define USEDHT DHT22 // type of DHT sensor (*)
#define FREEMEM // show free memory in (*)
#define BUFLEN 32 // buffer size for http server
#define NAME_VALUE_LEN 16 // buffer size for http server query parser
#define MAC_ADDRESS {0xDE, 0xAD, 0xBE, 0xEF, 0x1C, 0xEA}
//#define IP_ADDRESS ip(192, 168, 222, 201) // use DHCP if disabled (*)
#define USE_SERIAL 115200 // enable serial communication (bauds) (*)
//#define WEBDUINO_SERIAL_DEBUGGING 1

// disable reset on open tty
// stty -F /dev/ttyUSB0 115200 cs8 cread clocal -hupcl

// IO pins
// 11, 12, 13 are used by SPI
// 10 is used as CS for the ethernet shield
// usable digital pins: 0 1 2 3 4 5 6 7 8 9 (0 1 only if no Serial is used)
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
#include <WebServer.h>
#include <Streaming.h>
#include <EEPROM.h>
#include "eeprom.h"
#include "state.h"

#if defined(WEBTIME)
#include <Time.h>
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
#if defined(IP_ADDRESS)
IPAddress IP_ADDRESS;
#endif

WebServer webserver("", 80);

unsigned long timer = 0;

#if defined(WEBTIME)
float day_hh, night_hh;
#endif

#if defined(USEDHT)
DHT dht(DHTPIN, USEDHT);
float temp = 20, humi = 50;
float cold_thres;
int temp_delay;
State cold(0);
#endif

float bright = 0;
int day_thres, night_thres;
int day_delay, up_timeout, down_timeout;
boolean locked = 1;

State toggle(0);
State daytime(0);
State hatch_state(0), hatch_moving(0), hatch_sensed(0);
State door(0);


void(* softReset) (void) = 0; //declare reset function at address 0

#if defined(WEBTIME)
time_t msToTime(unsigned long &ms) {
  return now() - (millis() - ms) / 1000ul;
}

void printTime(Print &s, time_t &t) {
  if (timeStatus() != timeNotSet) {
    s << F(" # ") << year(t) << F("-") << month(t) << F("-") << day(t) <<
      F(" ") << hour(t) << F(":") << minute(t) << F(":") << second(t) << endl;
  } else {
    s << endl;
  }
}

void printTimeMs(Print &s, unsigned long ms) {
  time_t t = msToTime(ms);
  printTime(s, t);
}
#else
void printTime(Print &s, unsigned long &t) {
  s << endl;
}

void printTimeMs(Print &s, unsigned long ms) {
  s << endl;
}
#endif

#if defined(WEBTIME)
#define NTP_PACKET_SIZE 48 // NTP time is in the first 48 bytes of message

time_t getNtpTime() {
  EthernetUDP Udp;
  Udp.begin(8888);
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets
#if defined(NTP_SERVER)
  IPAddress NTP_SERVER;
#else
  IPAddress ntpIp = Ethernet.gatewayIP();
#endif
  sendNTPpacket(Udp, packetBuffer, ntpIp);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      time_t t = secsSince1900 - 2208988800UL;
      return t > MIN_TIME ? t + TIMEZONE * SECS_PER_HOUR : 0;
    }
  }
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(EthernetUDP &Udp, byte packetBuffer[], IPAddress &address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
#endif


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
  s << F("# ") << F(__FILE__) << F(" ") << F(__DATE__) << F(" v???") << endl;
#if defined(WEBTIME)
  time_t tt = now();
  s << F("time=") << tt;
  printTime(s, tt);
#endif
  s << F("uptime=")     << (millis() / SEC);
  printTimeMs(s, 0);
#if defined(FREEMEM)
  s << F("freemem=") << freeMemory() << endl;
#endif
  s << F("brightness=")    << bright << endl;
#if defined(USEDHT)
  s << F("temp=")    << temp << endl;
  s << F("humi=")    << humi << endl;
#endif
  s << F("day=")    << daytime.get();
  printTimeMs(s, daytime.time());
  s << F("hatch_sense=")    << hatch_sensed.get();
  printTimeMs(s, hatch_sensed.time());
  s << F("hatch_state=")    << hatch_state.get();
  printTimeMs(s, hatch_state.time());
  s << F("hatch_moving=")    << hatch_moving.get();
  printTimeMs(s, hatch_moving.time());
  s << F("locked=")    << locked << endl;
#if defined(USEDHT)
  s << F("cold=")    << cold.get();
  printTimeMs(s, cold.time());
#endif
  s << F("heater=")    << isOn(HEATER) << endl;
  s << F("door=")    << door.get();
  printTimeMs(s, door.time());
  s << F("light=")    << isOn(LIGHT) << endl;
  s << F("extra1=")      << isOn(EXTRA1)  << endl;
  s << F("extra2=")      << isOn(EXTRA2)  << endl;
  s << F("extra3=")      << analogRead(EXTRA3) << endl;
#if defined(WEBTIME)
  s  << F("# settings") << endl;
  s << F("day_hh=")    << day_hh << endl;
  s << F("night_hh=")    << night_hh << endl;
#endif
  s << F("day_thres=")    << day_thres << endl;
  s << F("night_thres=")    << night_thres << endl;
  s << F("day_delay=")    << day_delay << endl;
  s << F("up_timeout=")    << up_timeout << endl;
  s << F("down_timeout=")    << down_timeout << endl;
#if defined(USEDHT)
  s << F("temp_delay=")    << temp_delay << endl;
  s << F("cold_thres=")    << cold_thres << endl;
#endif
  s << endl;
}


#if defined(USE_SERIAL)
void processinput() {
  char buf[BUFLEN + 1];
  int n = Serial.readBytesUntil('\n', buf, BUFLEN);
  buf[n] = 0;
  process(webserver, buf);
  printdata(Serial);
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
                    && (!locked || (daytime.is(DAY) && daytime.age() > day_delay * SEC))) {
                  Serial << "UP" << endl;
                  hatch(i);
                }
                if (i == CLOSE && hatch_state.is(OPEN)) {
                  Serial << "DOWN" << endl;
                  hatch(i);
                }
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

void readSensors() {
  locked = 1;
#if defined(USEDHT)
  float x = dht.readTemperature();
  if (x > -50 && x < 80)
    temp = 0.8 * temp + 0.2 * x;
  x = dht.readHumidity();
  if (x >= 0 && x <= 100)
    humi = 0.8 * humi + 0.2 * x;
#endif
  bright = 0.9 * bright + 0.1 * brightness();
#if defined(WEBTIME)
  time_t t = now();
  float h = hour(t) + minute(t) / 60.;
  boolean isday = (timeStatus() == timeNotSet) || (h > day_hh && h < night_hh);
#else
  boolean isday = true;
#endif
  daytime.set((bright < night_thres || !isday) ? NIGHT : (bright > day_thres && isday) ? DAY : UNDEFINED);
#if defined(USEDHT)
  cold.set(temp < cold_thres);
  if (cold.age() > temp_delay * SEC && cold.changed()) {
    turn(HEATER, cold.get());
  }
#endif
#if !defined(IP_ADDRESS)
  Ethernet.maintain();
#endif
}


void setup() {
#if defined(WATCHDOG)
  wdt_disable();
  wdt_enable(WATCHDOG);
#endif
#if defined(USE_SERIAL)
  Serial.begin(USE_SERIAL);
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
  daytime.set(b < night_thres ? NIGHT : b > day_thres ? DAY : UNDEFINED);
  daytime.changed();

#if defined(DHT)
  dht.begin();
#endif
#if defined(IP_ADDRESS)
  Ethernet.begin(mac, ip);
#else
  Ethernet.begin(mac);
#endif
  webserver.setDefaultCommand(&rest);
  webserver.begin();

#if defined(WEBTIME)
  setSyncProvider(getNtpTime);
  setSyncInterval(TIME_RESYNC);
#endif
#if defined(USE_SERIAL)
  Serial << F("UP");
#if defined(WEBTIME)
  time_t t = now();
  printTime(Serial, t);
#else
  Serial << endl;
#endif
#endif
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

  int len = BUFLEN;
  char buf[BUFLEN + 1];
  webserver.processConnection(buf, &len);

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

    // read sensors
    if (millis() - timer > 5 * SEC) {
      timer = millis();
      readSensors();
    }

    if (toggle.get() && toggle.age() > SEC && toggle.changed()) {
      if (!locked || (daytime.is(DAY) && daytime.age() > day_delay * SEC) || hatch_state.is(OPEN)) {
        hatch(hatch_state.is(OPEN) ? CLOSE : OPEN);
      }
    } else {
      if (daytime.age() > day_delay * SEC && daytime.changed()) {
        int hs = hatch_state.get();
        if (daytime.is(DAY)) { // DAY
          if (hs <= UNDEFINED) // if closed/undefined
            hatch(OPEN); // open
        } else if (daytime.is(NIGHT)) { // NIGHT
          if (hs == UNDEFINED) { // if undefined
            hatch(OPEN); // open first
            daytime.latch = 1; // relatch to close
          } else if (hs == OPEN) { // if open
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
