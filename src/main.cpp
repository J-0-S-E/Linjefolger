#include <Arduino.h>
#include <QTRSensors.h>

#define sensorCount 15

const int leftF = 2;
const int leftB = 4;
const int rightF = 15;
const int rightB = 16;
const int minDiff = 40;
float lastPositionError = 0; // For å huske siste kjente posisjonsfeil
float integral = 0;          // For integralleddet
unsigned long lastTime = 0;  // For tidsberegning

// Opprett et QTR-sensorobjekt og arrays for sensordata
QTRSensors qtr;
uint16_t sensorValues[sensorCount];
uint16_t startValues[sensorCount];

// Definer sensorpinnene
const uint8_t sensorPins[sensorCount] = {
    32, // Sensor 0 (venstre ytterste)
    5,  // Sensor 1
    33, // Sensor 2
    25, // Sensor 3
    26, // Sensor 4
    27, // Sensor 5
    14, // Sensor 6
    12, // Sensor 7 (midten)
    17, // Sensor 8
    13, // Sensor 9
    23, // Sensor 10
    22, // Sensor 11
    21, // Sensor 12
    19, // Sensor 13
    18  // Sensor 14 (høyre ytterste)
};

// Definer sensorvekter for beregning av posisjonsfeil
int sensorWeights[sensorCount] = {
    -7,  // Sensor 0
    -10,  // Sensor 1
    -5,  // Sensor 2
    -4,  // Sensor 3
    -3,  // Sensor 4
    -2,  // Sensor 5
    -1,  // Sensor 6
     0,  // Sensor 7 (midten)
     1,  // Sensor 8
     2,  // Sensor 9
     3,  // Sensor 10
     4,  // Sensor 11
     5,  // Sensor 12
     10,  // Sensor 13
     7   // Sensor 14
};

void setup() {
    Serial.begin(9600);

    pinMode(leftF, OUTPUT);
    pinMode(rightF, OUTPUT);
    pinMode(leftB, OUTPUT);
    pinMode(rightB, OUTPUT);

    for (uint8_t i = 0; i < sensorCount; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, sensorCount);
    delay(500); // Vent litt før lesing av startverdier
    qtr.read(startValues);  // Les og lagre startverdiene ved oppstart

    lastTime = millis(); // Initialiser tid

    Serial.println("Sensoroppsett fullført. Startverdier lagret.");
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    // Begrens hastighetene til -255 til 255
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Venstre motor
    if (leftSpeed >= 0) {
        analogWrite(leftF, leftSpeed);
        analogWrite(leftB, 0);
    } else {
        analogWrite(leftF, 0);
        analogWrite(leftB, -leftSpeed);
    }

    // Høyre motor
    if (rightSpeed >= 0) {
        analogWrite(rightF, rightSpeed);
        analogWrite(rightB, 0);
    } else {
        analogWrite(rightF, 0);
        analogWrite(rightB, -rightSpeed);
    }
}

void linjeOgMotor() {
    const int maxSpeed = 150; // Maksimal motorhastighet
    const int baseSpeed = 150; // Grunnhastighet

    qtr.read(sensorValues);   // Les alle sensorverdiene

    int weightedSum = 0;
    int totalWeight = 0;

    bool outerLeftActive = false;
    bool outerRightActive = false;

    // Iterer over alle sensorene
    for (int i = 0; i < sensorCount; i++) {
        int diff = abs(sensorValues[i] - startValues[i]);
        if (diff > minDiff) {
            // Sensor er aktiv
            int weight = sensorWeights[i];
            weightedSum += weight * diff;
            totalWeight += diff;

            // Sjekk om ytterste sensorer er aktive
            if (i == 0) {
                outerLeftActive = true;
            }
            if (i == sensorCount - 1) {
                outerRightActive = true;
            }
        }
    }

    float positionError = 0;

    if (totalWeight > 0) {
        // Beregn posisjonsfeil
        positionError = (float)weightedSum / totalWeight;
    } else {
        // Ingen sensorer er aktive, stopp roboten
        setMotorSpeed(0, 0);
        Serial.println("Ingen sensorer aktive, stopper roboten.");
        return;
    }

    // Normaliser posisjonsfeilen til et intervall mellom -1 og 1
    float normalizedError = positionError / 7.0;

    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;

    // Sjekk om ytterste sensorer er aktive
    if (outerLeftActive) {
        // Maks sving til venstre
        leftMotorSpeed = -maxSpeed;  // Venstre motor bakover
        rightMotorSpeed = maxSpeed;  // Høyre motor fremover
        Serial.println("Ytterste venstre sensor aktiv - maksimal sving til venstre.");
    } else if (outerRightActive) {
        // Maks sving til høyre
        leftMotorSpeed = maxSpeed;   // Venstre motor fremover
        rightMotorSpeed = -maxSpeed; // Høyre motor bakover
        Serial.println("Ytterste høyre sensor aktiv - maksimal sving til høyre.");
    } else {
        // Vanlig beregning av motorhastigheter
        // Beregn svingfaktor basert på posisjonsfeil
        float turnFactor = normalizedError;

        // Beregn hastighetsendring
        int speedAdjustment = (int)(maxSpeed * turnFactor);

        // Juster motorhastighetene
        leftMotorSpeed = baseSpeed + speedAdjustment;
        rightMotorSpeed = baseSpeed - speedAdjustment;

        // Begrens motorhastighetene til -maxSpeed til maxSpeed
        leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
        rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);
    }

    // Sett motorhastighetene
    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

    // Debugging-utskrifter
    Serial.print("Posisjonsfeil: ");
    Serial.println(normalizedError);
    Serial.print("Venstre motorhastighet: ");
    Serial.println(leftMotorSpeed);
    Serial.print("Høyre motorhastighet: ");
    Serial.println(rightMotorSpeed);
}


void loop() {
    linjeOgMotor();  // Kjør funksjonen for å sjekke sensorverdier og motorstyring
    // Ingen delay for raskere respons
}
