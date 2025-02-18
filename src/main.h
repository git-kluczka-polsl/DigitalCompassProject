#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#define PIN 8

Adafruit_NeoPixel RING = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

#include <QMC5883LCompass.h>

QMC5883LCompass compass;
char magnetometerData[150];
char gpsData[150];


/////////GPS//////////////

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>


static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


double lat, lon; 
uint8_t godz, minuta, sekunda;
uint16_t rok;
uint8_t miesiac, dzien;

struct JulianDate {
  long integerPart;  // Część całkowita
  float fractionalPart; // Część ułamkowa (zawsze w zakresie [0.0, 1.0))
};

JulianDate jd;

bool validGpsData = false;
void setupGPS();

void loopGPS();

void gpsHandler();

double magnetometerHandler();




