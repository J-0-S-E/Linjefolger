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
    32, // Sensor 25 (venstre ytterste)
    5,  // Sensor 19
    33, // Sensor 18
    25, // Sensor 17
    26, // Sensor 16
    27, // Sensor 15
    14, // Sensor 14
    12, // Sensor 13 (midten)
    17, // Sensor 12
    13, // Sensor 11
    23, // Sensor 10
    22, // Sensor 9
    21, // Sensor 8
    19, // Sensor 7
    18  // Sensor 1 (høyre ytterste)
};

// Definer sensorvekter for beregning av posisjonsfeil
int sensorWeights[sensorCount] = {
    -10,  // Sensor 0 / 25
    -6,  // Sensor 1 / 19
    -5,  // Sensor 2 / 18
    -4,  // Sensor 3 / 17
    -3,  // Sensor 4 / 16
    -2,  // Sensor 5 / 15
    -1,  // Sensor 6 / 14
     0,  // Sensor 7 (midten) / 13
     1,  // Sensor 8 / 12
     2,  // Sensor 9 / 11
     3,  // Sensor 10 / 10
     4,  // Sensor 11 / 9
     5,  // Sensor 12 / 8
     6,  // Sensor 13 / 7
     10   // Sensor 14 / 1
};

// Tilstander for om roboten skal svinge
bool turningLeft = false;
bool turningRight = false;

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
    delay(500);
    qtr.read(startValues);

    lastTime = millis();
    Serial.println("Sensoroppsett fullført. Startverdier lagret.");
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed >= 0) {
        analogWrite(leftF, leftSpeed);
        analogWrite(leftB, 0);
    } else {
        analogWrite(leftF, 0);
        analogWrite(leftB, -leftSpeed);
    }

    if (rightSpeed >= 0) {
        analogWrite(rightF, rightSpeed);
        analogWrite(rightB, 0);
    } else {
        analogWrite(rightF, 0);
        analogWrite(rightB, -rightSpeed);
    }
}

void linjeOgMotor() {
    const int maxSpeed = 150;
    const int baseSpeed = 150;

    qtr.read(sensorValues);

    int weightedSum = 0;
    int totalWeight = 0;
    bool sensorActive = false;

    for (int i = 0; i < sensorCount; i++) {
        int diff = abs(sensorValues[i] - startValues[i]);

        if (diff > minDiff) {
            int weight = sensorWeights[i];
            weightedSum += weight * diff;
            totalWeight += diff;
            sensorActive = true;

            // Hvis en sensor innover er aktiv, stopp svingingen
            if (turningLeft && i > 0) {
                turningLeft = false;
            }
            if (turningRight && i < sensorCount - 1) {
                turningRight = false;
            }
        }
    }

    if (!sensorActive) {
        setMotorSpeed(0, 0);
        Serial.println("Ingen sensorer aktive, stopper roboten.");
        return;
    }

    if (turningLeft) {
        setMotorSpeed(-maxSpeed, maxSpeed);
        delay(100);
        Serial.println("Svinger til venstre for å finne linjen igjen.");
        return;
    }

    if (turningRight) {
        setMotorSpeed(maxSpeed, -maxSpeed);
        Serial.println("Svinger til høyre for å finne linjen igjen.");
        delay(100);
        return;
    }

    // Sjekk om ytterste venstre sensor (fysisk sensor 25, på pinne 32) er aktiv
    int diff0 = abs(sensorValues[0] - startValues[0]);
    if (diff0 > minDiff) {
        turningLeft = true;
        setMotorSpeed(-maxSpeed, maxSpeed);
        Serial.println("Ytterste venstre sensor (25) aktiv - start sving til venstre.");
        return;
    }

    // Sjekk om ytterste høyre sensor (fysisk sensor 1, på pinne 18) er aktiv
    int diff14 = abs(sensorValues[sensorCount - 1] - startValues[sensorCount - 1]);
    if (diff14 > minDiff) {
        turningRight = true;
        setMotorSpeed(maxSpeed, -maxSpeed);
        Serial.println("Ytterste høyre sensor (1) aktiv - start sving til høyre.");
        return;
    }

    // Beregn posisjonsfeil og motorhastigheter som normalt
    float positionError = (totalWeight > 0) ? (float)weightedSum / totalWeight : 0;
    float normalizedError = positionError / 7.0;
    int speedAdjustment = (int)(maxSpeed * normalizedError);

    int leftMotorSpeed = baseSpeed + speedAdjustment;
    int rightMotorSpeed = baseSpeed - speedAdjustment;

    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

    Serial.print("Posisjonsfeil: ");
    Serial.println(normalizedError);
    Serial.print("Venstre motorhastighet: ");
    Serial.println(leftMotorSpeed);
    Serial.print("Høyre motorhastighet: ");
    Serial.println(rightMotorSpeed);
}

void loop() {
    linjeOgMotor();
}