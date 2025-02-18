#include "main.h"
#include <math.h>


// Konwersja stopni na radiany
double toRadians(double degrees) {
  return degrees * (PI / 180.0);
}
double radToDeg(double rad) {
  return rad * (180.0 / PI);
}

int getDayOfYear(int day, int month, int year) {
  // Tablica liczby dni w poszczególnych miesiącach dla roku nieprzestępnego
  int monthDays[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

  // Sprawdzenie, czy rok jest przestępny
  // Rok przestępny: podzielny przez 4, ale nie przez 100, chyba że podzielny przez 400.
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
      monthDays[1] = 29;
  }

  int dayOfYear = 0;
  // Sumujemy dni dla wszystkich miesięcy poprzedzających podany miesiąc
  for (int i = 0; i < month - 1; ++i) {
      dayOfYear += monthDays[i];
  }
  
  // Dodajemy dni w bieżącym miesiącu
  dayOfYear += day;
  
  return dayOfYear;
}

// Obliczanie deklinacji Słońca
double calculateSolarDeclination(int dayOfYear) {
    Serial.print("Day of the year: ");
    Serial.println(dayOfYear);
    //return 23.45 * sin(toRadians((360.0 / 365.0) * (dayOfYear + 284)));
        // Obliczenie kąta frakcyjnego roku (w radianach)
        double gamma = 2.0 * PI / 365.0 * (dayOfYear - 1);
    
        // Wzór z szeregiem harmonicznym (Spencera, 1971)
        double decl = 0.006918 
                    - 0.399912 * cos(gamma) 
                    + 0.070257 * sin(gamma)
                    - 0.006758 * cos(2 * gamma) 
                    + 0.000907 * sin(2 * gamma)
                    - 0.002697 * cos(3 * gamma) 
                    + 0.00148  * sin(3 * gamma);
        
        // Konwersja wyniku z radianów na stopnie
        return decl * (180.0 / PI);
}

// Funkcja obliczająca równanie czasu (w minutach) dla danego dnia roku.
// Wzór przybliżony, gdzie dayOfYear = 1 dla 1 stycznia.
double calculateEquationOfTime(int dayOfYear) {
  // Obliczamy kąt B (w radianach) – przybliżenie
  double B = toRadians((360.0 / 365.0) * (dayOfYear - 81));
  // Przybliżone równanie czasu (w minutach)
  return 9.87 * sin(2 * B) - 7.53 * cos(B) - 1.5 * sin(B);
}

// Funkcja obliczająca czas słoneczny (w godzinach).
// Uwzględnia korekcję wynikającą z długości geograficznej i równania czasu.
double calculateSolarTime(double standardTimeUTC, double longitude, int dayOfYear) {
  // Równanie czasu w minutach
  double EoT = calculateEquationOfTime(dayOfYear);
  
  // Korekta związana z różnicą między lokalną długością a południkiem strefy czasowej.
  // Dla strefy UTC południk standardowy wynosi 0°, a korekta wynosi 4 minuty na stopień.
  double longitudeCorrection = longitude * 4.0;  // w minutach
  
  // Łączna korekta (w godzinach)
  double correction = (longitudeCorrection + EoT) / 60.0;
  
  // Czas słoneczny
  return standardTimeUTC + correction;
}

// Funkcja obliczająca kąt godzinowy Słońca (w stopniach).
// Przyjmuje czas standardowy (UTC), długość geograficzną oraz dzień roku.
double calculateSolarHourAngle(int hour, int minute, double longitude, int dayOfYear) {
  // Przeliczamy czas standardowy na godziny dziesiętne
  double standardTimeUTC = hour + (minute / 60.0);
  Serial.print("Czas UTC: ");
  Serial.println(standardTimeUTC);
  
  // Obliczamy czas słoneczny
  double solarTime = calculateSolarTime(standardTimeUTC, longitude, dayOfYear);
  Serial.print("Czas słoneczny: ");
  Serial.println(solarTime);
  
  // Kąt godzinowy = 15° * (czas słoneczny - 12)
  double hourAngle = 15.0 * (solarTime - 12.0);
  return hourAngle;
}

// Obliczanie azymutu Słońca
double calculateSolarAzimuth(double latitude, double solarDeclination, double solarHourAngle) {
    // Konwersja stopni na radiany
    latitude = toRadians(latitude);
    solarDeclination = toRadians(solarDeclination);
    solarHourAngle = toRadians(solarHourAngle);

    // Używamy atan2, aby poprawnie wyznaczyć kąt
    double azimuth = atan2(sin(solarHourAngle), 
                           cos(solarHourAngle) * sin(latitude) - tan(solarDeclination) * cos(latitude)) * RAD_TO_DEG -180;
    
    // Przekształcamy wynik do zakresu 0 - 360 stopni
    if (azimuth < 0) {
        azimuth += 360.0;
    }
    
    return azimuth;
}
//------------------------------------------------------
// Oblicza datę juliańską na podstawie UTC
//------------------------------------------------------
JulianDate getJulianDate() {
  //JulianDate jd;
  jd.integerPart = 0L;
  jd.fractionalPart = 0.0f;
  int localMonth = miesiac, localYear = rok;
  if (localMonth <= 2) {
    localYear -= 1;
    localMonth += 12;
}

int A = localYear / 100; // część całkowita z wieku
int B = int(2 - A + (A / 4)); // formuła dla gregoriańskiego

// Obliczanie części całkowitej
jd.integerPart += static_cast<long>(floor(365.25 * (localYear + 4716)));
jd.integerPart += static_cast<long>(floor(30.6001 * (localMonth + 1)));

// Składnik z dniem i poprawką B
float dayComponent = dzien + B - 1524.5;
long dayInt = static_cast<long>(floor(dayComponent));
jd.integerPart += dayInt;
jd.fractionalPart += dayComponent - dayInt;

// Składnik z godzinami
jd.fractionalPart += godz / 24.0f;
  jd.fractionalPart += minuta / 1440.0f; // 1440 = 24 * 60
  jd.fractionalPart += sekunda / 86400.0f; // 86400 = 24 * 60 * 60

// Korekta przeniesień w części ułamkowej
if (jd.fractionalPart >= 1.0f) {
    jd.integerPart += 1L;
    jd.fractionalPart -= 1.0f;
} else if (jd.fractionalPart < 0.0f) {
    jd.integerPart -= 1L;
    jd.fractionalPart += 1.0f;
}

Serial.print("JD całkowite: ");
Serial.println(jd.integerPart);
Serial.print("JD ułamkowe:  ");
Serial.println(jd.fractionalPart, 6);
Serial.print("JD pełne:     ");
Serial.println(jd.integerPart + jd.fractionalPart, 6);
return jd;
}


void computeMoonAngles(double T, double &L, double &l, double &F, double &D) {
  L = fmod(218.316 + 481267.881 * T, 360.0);  // Średnia długość
  l = fmod(134.963 + 477198.868 * T, 360.0);   // Anomalia średnia
  F = fmod(93.272 + 483202.018 * T, 360.0);    // Argument szerokości
  D = fmod(297.850 + 445267.112 * T, 360.0);   // Elongacja średnia
}

void getMoonEcliptic(double T, double &lambda, double &beta) {
  double L, l, F, D;
  computeMoonAngles(T, L, l, F, D);

  // Główne perturbacje (stopnie)
  lambda = L + 6.289 * sin(radians(l));          // Główny człon
  lambda += 1.274 * sin(radians(2*D - l));       // Evection
  lambda = fmod(lambda, 360.0);

  beta = 5.128 * sin(radians(F));                // Szerokość
  beta += 0.280 * sin(radians(l + F));           // Mała poprawka
}

double getLocalSiderealTime(double lon) {
  // Obliczanie parametru T z uwzględnieniem precyzji
  const long jd_ref = 2451545L; // JD dla J2000
  double delta_jd_int = static_cast<double>(jd.integerPart - jd_ref);
  double delta_jd_total = delta_jd_int + static_cast<double>(jd.fractionalPart);
  double T = delta_jd_total / 36525.0;

  // Obliczenie GMST z zachowaniem precyzji dla dużych wartości JD
  double GMST = 280.46061837 
              + 360.98564736629 * delta_jd_total
              + 0.000387933 * T * T 
              - (T * T * T) / 38710000.0;

  // Normalizacja do zakresu 0-360 stopni
  GMST = fmod(GMST, 360.0);
  if (GMST < 0) GMST += 360.0;

  // Obliczenie lokalnego czasu gwiazdowego
  double LST = fmod(GMST + lon, 360.0);
  if (LST < 0) LST += 360.0;

  // Debugowanie (opcjonalne)
  Serial.print("GMST: ");
  Serial.println(GMST, 6);
  Serial.print("LST:  ");
  Serial.println(LST, 6);

  return LST;
}

void eclipticToEquatorial(double lambda, double beta, double &ra, double &dec) {
  double eps = 23.4393; // Nachylenie ekliptyki

  double sin_lambda = sin(radians(lambda));
  double cos_lambda = cos(radians(lambda));
  double sin_beta = sin(radians(beta));

  ra = degrees(atan2(sin_lambda * cos(radians(eps)) - sin_beta * sin(radians(eps)), cos_lambda));
  dec = degrees(asin(sin_beta * cos(radians(eps)) + cos(radians(beta)) * sin(radians(eps)) * sin_lambda));
  Serial.print("Dec: ");
  Serial.println(dec, 6);
}

double computeMoonAzimuth(double lat, double lon) {
  const long jd_ref = 2451545L; // JD dla J2000
  double delta_jd_int = static_cast<double>(jd.integerPart - jd_ref);
  double delta_jd_total = delta_jd_int + static_cast<double>(jd.fractionalPart);
  double T = delta_jd_total / 36525.0;
  double lambda, beta;
  getMoonEcliptic(T, lambda, beta); // Pozycja w ekliptyce

  double ra, dec;
  eclipticToEquatorial(lambda, beta, ra, dec); // Na układ równikowy

  double lst = getLocalSiderealTime(lon);
  double H = lst -ra;
  while (H < -180) H += 360;
  while (H > 180) H -= 360;
  double Hrad = DEG_TO_RAD*H;
  double DecRad = DEG_TO_RAD*dec;
  double latRad = DEG_TO_RAD*lat;
  
  // Oblicz wysokość (altitude) a:
  double sinAlt = sin(DecRad) * sin(latRad) + cos(DecRad) * cos(latRad) * cos(Hrad);
  double alt = asin(sinAlt);
  
  // Oblicz azymut przy użyciu wzorów:
  double sinAz = - sin(Hrad) * cos(DecRad) / cos(alt);
  double cosAz = (sin(DecRad) - sin(latRad) * sin(alt)) / (cos(latRad) * cos(alt));
  
  double az_rad = atan2(sinAz, cosAz);
  double az_deg = RAD_TO_DEG*az_rad;
  if (az_deg < 0) az_deg += 360;
  return az_deg;
}

int getLEDfromAngle(float angle) {
  // Normalizacja kąta w zakresie [0, 360)
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;

  // Mamy 24 diody, każda obejmuje 15 stopni (360 / 24), LED 0 = 360°, LED 23 = 15°
  int ledIndex = round((360 - angle) / 15.0);

  // Zabezpieczenie przed przekroczeniem indeksu
  if (ledIndex == 24) ledIndex = 0;

  return ledIndex;
}

void lightLEDS(uint32_t northPin, uint16_t moonPin, uint16_t sunPin) {
  for(uint16_t i=0; i< RING.numPixels(); i++){
    RING.setPixelColor(i, 0);
  }
  RING.setPixelColor(northPin, RING.Color(255, 0, 0));
  RING.setPixelColor(moonPin, RING.Color(0, 0, 255));
  RING.setPixelColor(sunPin, RING.Color(255, 255, 0));
  RING.show();
}

void setup()
{
  Serial.begin(9600);          // Rozpocznij komunikację szeregową na 9600
  ss.begin(GPSBaud);           // Rozpocznij komunikację GPS na ustalonym baudrate
  compass.init();              // Inicjalizacja magnetometru
  compass.setCalibration(-1700, 1173, -1313, 448, -1418, 336); // Kalibracja
  setupGPS(); 
  RING.begin();
  RING.setBrightness(50);
  RING.show(); // Initialize all pixels to 'off'                 // Ustawienia GPS
  
}

void loop()
{
  Serial.println("################");
  gpsHandler();
  Serial.println(gps.satellites.value());
  Serial.println(gps.satellites.age());
  Serial.println(gpsData);
  //delay(100);
  double northAzimuth = magnetometerHandler();
  Serial.println(magnetometerData);
  int northPin = getLEDfromAngle(northAzimuth);
  int moonPin;
  int sunPin;

  // Obliczanie azymutu Słońca, jeśli GPS działa
  if (validGpsData) {
    getJulianDate();
    static double dayOfYear = getDayOfYear(dzien, miesiac, rok);
    double solarDeclination = calculateSolarDeclination(dayOfYear);
    double solarHourAngle = calculateSolarHourAngle(godz, minuta, lon, dayOfYear);
    double solarAzimuth = calculateSolarAzimuth(lat, solarDeclination, solarHourAngle);
    double moonAzimuth = computeMoonAzimuth(lat, lon);
    Serial.print("Moon Az: ");
    Serial.println(moonAzimuth, 6); 
    Serial.print("Solar Declination: ");
    Serial.println(solarDeclination, 6);
    Serial.print("Solar Hour Angle: ");
    Serial.println(solarHourAngle, 6);
    Serial.print("Solar Azimuth: ");
    Serial.println(solarAzimuth, 6);
    
    moonPin = getLEDfromAngle(northAzimuth - moonAzimuth);
    sunPin = getLEDfromAngle(northAzimuth - solarAzimuth);
    
    // Serial.print("Pin: ");
    // Serial.println(pin, 6);   
   
  }
  else {
    rok   = 2025;
    miesiac  = 2;
    dzien    = 19;
    godz   = 23;
    minuta = 11;
    sekunda = 0;
    lat = 50.294071;
    lon = 18.676486;
    getJulianDate();
    //int pin = getMoonLedIndex();
    // Serial.print("Pin: ");
    // Serial.println(pin, 6);

    double dayOfYear = getDayOfYear(dzien, miesiac, rok); // Przybliżona wartość
    double solarDeclination = calculateSolarDeclination(dayOfYear);
    double solarHourAngle = calculateSolarHourAngle(godz, minuta, lon,dayOfYear);
    double solarAzimuth = calculateSolarAzimuth(lat, solarDeclination, solarHourAngle);
    double moonAzimuth = computeMoonAzimuth(lat, lon);
    Serial.print("Moon Az: ");
    Serial.println(moonAzimuth, 6);
    Serial.print("Solar Declination: ");
    Serial.println(solarDeclination, 6);
    Serial.print("Solar Hour Angle: ");
    Serial.println(solarHourAngle, 6);
    Serial.print("Solar Azimuth: ");
    Serial.println(solarAzimuth, 6);
    //delay(10000);
  }

  // Jeśli nie ma danych GPS, ostrzeżenie co 5 sekund
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true); // Zatrzymaj działanie w przypadku braku GPS
  }
  lightLEDS(northPin, moonPin, sunPin);

  //delay(100); // Opóźnienie, aby uniknąć przeciążenia serial
}

void setupGPS()
{
  Serial.println(F("GPS + Magnetometer Data"));
}

void gpsHandler()
{
  //strcpy(gpsData, "");
  while (ss.available() > 0)
    if (gps.encode(ss.read())) {
      if (gps.location.isValid())
      {
        lat = gps.location.lat();
        lon = gps.location.lng();
        char latBuffer[10], lngBuffer[10];
        dtostrf(lat, 8, 6, latBuffer);
        dtostrf(lon, 8, 6, lngBuffer);
        sprintf(gpsData, "GPS: Lat=%s, Lng=%s", latBuffer, lngBuffer);
      }
      else
      {
        sprintf(gpsData, "GPS: Location INVALID");
      }

      if (gps.date.isValid() && gps.time.isValid())
      {
        rok   = gps.date.year();
        miesiac  = gps.date.month();
        dzien    = gps.date.day();
        godz   = gps.time.hour();
        minuta = gps.time.minute();
        sekunda = gps.time.second();
        char dateTime[50];
        sprintf(dateTime, "Date=%02d/%02d/%04d Time=%02d:%02d:%02d", 
                miesiac, dzien, rok,
                godz, minuta, sekunda);
        strcat(gpsData, " ");
        strcat(gpsData, dateTime);
      }
      else
      {
        strcat(gpsData, " Date/Time INVALID");
      }
      if (gps.date.isValid() && gps.time.isValid() && gps.location.isValid())
        validGpsData = true;
      else validGpsData = false;
    }
}

double magnetometerHandler() {
  strcpy(magnetometerData, "");
  int x_value, y_value, z_value, azimuth;
  byte bearing;
  char direction[4];
  direction[3] = '\0';
  compass.read();
  x_value = compass.getX();
  y_value = compass.getY();
  z_value = compass.getZ();
  azimuth = compass.getAzimuth();
  bearing = compass.getBearing(azimuth);
  compass.getDirection(direction, azimuth);

  sprintf(magnetometerData, 
          "Magnetometer: X=%6d | Y=%6d | Z=%6d | Azimuth=%3d° | Bearing=%02hu | Dir=%s", 
          x_value, y_value, z_value, azimuth, bearing, direction);
  return azimuth;        
}
