#include <Arduino.h>
#include <QTRSensors.h>
#define sensorCount 19

const int left = 19;
const int right = 16;
const int minDiff = 100;

// Opprett et QTR-sensorobjekt og arrays for sensordata
QTRSensors qtr;
uint16_t sensorValues[sensorCount];
uint16_t startValues[sensorCount];

// Motorhastigheter for hver sensor
const int motorSpeeds[][2] = {
    {0, 255},    // Sensor 1, motorjustering for venstre og høyre
    {55, 255},   // Sensor 4
    {100, 255},  // Sensor 5
    {110, 255},  // Sensor 6
    {120, 255},  // Sensor 7
    {130, 255},  // Sensor 8
    {140, 255},  // Sensor 9
    {150, 255},  // Sensor 10
    {160, 255},  // Sensor 11
    {170, 255},  // Sensor 12
    {255, 255},  // Sensor 13
    {255, 170},  // Sensor 14
    {255, 160},  // Sensor 15
    {255, 150},  // Sensor 16
    {255, 140},  // Sensor 17
    {255, 130},  // Sensor 18
    {255, 120},  // Sensor 19
    {255, 110},  // Sensor 20
    {255, 100},  // Sensor 21
    {255, 55},   // Sensor 22
    {255, 0}     // Sensor 25
};

// Definer sensorpinnene
const uint8_t sensorPins[sensorCount] = {13, 12, 14, 27, 26, 25, 33, 32, 17, 3, 4, 16, 18, 20, 21, 22, 5, 15, 23};
//                    Sensor-pinner:      1   5   6   7   8   9  10  11  12  13 14 15  16  17  18  19  20 21  25
void setup() {
    Serial.begin(9600);
   pinMode(left, OUTPUT);
   pinMode(right, OUTPUT);

    for (uint8_t i = 0; i < sensorCount; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, sensorCount);
    qtr.read(startValues);  // Les og lagre startverdiene ved oppstart

    Serial.println("Sensor setup complete. Startverdier lagret.");
}

// Generisk funksjon for motorstyring basert på sensoravlesninger
void kontrollMotor(uint8_t sensorIndex, int leftSpeed, int rightSpeed) {
    if ((sensorValues[sensorIndex] - startValues[sensorIndex] > minDiff)) {
        analogWrite(left, leftSpeed);
        analogWrite(right, rightSpeed);
    }
}

void linje() {
    qtr.read(sensorValues);  // Les alle sensorverdiene

    // Juster motorene basert på sensorverdier ved å bruke motorSpeeds array
    for (uint8_t i = 0; i < sensorCount; i++) {
        kontrollMotor(i, motorSpeeds[i][0], motorSpeeds[i][1]);
    }
}

void loop() {
    linje();  // Kjør linjefølger-logikken

    // Valgfri: Skriv ut sensorverdier for feilsøking
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print("S");
        Serial.print(sensorPins[i]);  // Skriv ut sensornummeret basert på sensorPins[]-arrayet
        Serial.print(":");
        Serial.print(sensorValues[i]);
        Serial.print(" St:");
        Serial.print(startValues[i]);
        Serial.print(" D:");
        Serial.print(sensorValues[i] - startValues[i]);
        Serial.print(" ");
    }

    delay(1000);  // Vent ett sekund mellom hver syklus
    Serial.println();
    Serial.println();
}
