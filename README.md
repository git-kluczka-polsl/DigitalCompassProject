A project created for Politechnika Śląska's 'Systemy Mikroprocesorowe i Wbudowane' classes. A digital compass directing towards magnetic North as well as Sun and Moon azimuths in real time.

The following code works for Arduino Uno Rev3 connected to GPS Neo6M, QMC magnetometer and NeoPixel 24-led Ring. The algorithm uses GPS NMEA (current location, day, time) to calculate astromonical values and compares them with the current pointed direction from the magnetometer. The azimuths are represented as lighted LEDs on the ring.

Coded by Krzysztof Kluczka, 2024/25.
