#include <Stepper.h> // Library for drive for Motor 2

// Pins for motor 1
#define Motor1_pulse_pin 5 
#define Motor1_dir_pin 2
#define Motor1_enable_pin 4

// Pins for motor 2
#define N1 8 
#define N2 9
#define N3 10
#define N4 11

// Pins for Limit Switches
#define LS1_pin 13 // For X axis
#define LS2_pin 12 // For Y axis

// Motor characteristics
const int motor1_pulses_rev = 400;  // 400 pulses per revolution for motor 1
const int motor2_stepsPerRevolution = 2038; // Steps per revolution for motor 2

// Motors' diameters (Assumption)
float d1 = 2; // Diameter of motor 1 (cm)
float d2 = 2; // Diameter of motor 2 (cm)

// Limit Switch states
bool LS1 = false;
bool LS2 = false;

// Arrays to store points
#define MAX_POINTS 10
int points[MAX_POINTS][3]; // [P, X, Y]
int pointCount = 0;

// Track current position
float currentX = 0.0;
float currentY = 0.0;

// Class for Motor 1 - X axis ----------------------------------------------------------------------------------------
class Motor1 {
private:
    int pulsePin, dirPin, enablePin;
    float diameter;
    int pulsesPerRev;

    float circumference() {
        return diameter * 3.1416;
    }

public:
    Motor1(int pulse, int dir, int enable, float dia, int pulsesRev)
        : pulsePin(pulse), dirPin(dir), enablePin(enable), diameter(dia), pulsesPerRev(pulsesRev) {}

    void setup() {
        pinMode(pulsePin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, 0); // Enable motor
    }

      void move(int cm) {
        bool dir = cm >= 0;
        if (cm < 0) cm = -cm;

        float pulsesPerCm = pulsesPerRev / circumference();
        int pulses = pulsesPerCm * cm;

        digitalWrite(dirPin, dir); // Set direction
        for (int i = 0; i < pulses; i++) {
            stepOnce(1);
            delayMicroseconds(500);
        }
    }

    void stepOnce(int foward) {
        if (foward){
        digitalWrite(dirPin, 1); // Set direction
        digitalWrite(pulsePin, HIGH);
        delayMicroseconds(10);
        digitalWrite(pulsePin, LOW);
        delayMicroseconds(10);
        }
        else {
        digitalWrite(dirPin, 0); // Set direction
        digitalWrite(pulsePin, HIGH);
        delayMicroseconds(10);
        digitalWrite(pulsePin, LOW);
        delayMicroseconds(10);
        }

    }

    int calculateSteps(float cm) {
        return abs(cm) * pulsesPerRev / circumference();
    }
};
//End of the classs for Motor1----------------------------------------------------------------------------------------------------------------------------

// Class for Motor 2 - Y axis-------------------------------------------------------------------------------------------------
class Motor2 {
private:
    Stepper stepper;
    float diameter;
    int stepsPerRev;

    float circumference() {
        return diameter * 3.1416;
    }

public:
    Motor2(int stepsRev, int pin1, int pin2, int pin3, int pin4, float dia)
        : stepper(stepsRev, pin1, pin3, pin2, pin4), diameter(dia), stepsPerRev(stepsRev) {}

    void setup() {
        // Any specific setup for Motor 2 can be added here if needed
    }

    void move(int cm) {
        int steps = calculateSteps(cm);
        stepper.setSpeed(10);
        stepper.step(steps);
    }

    void stepOnce(bool forward) {
        if (forward) {
            stepper.step(1); // Forward step
        } else {
            stepper.step(-1); // Backward step
        }
    }

    int calculateSteps(float cm) {
        return abs(cm) * stepsPerRev / circumference();
    }
};
//End of the classs for Motor2----------------------------------------------------------------------------------------------------------------------------


// Class for 2D Cutting Machine -------------------------------------------------------------------------------------------------------------------------
class Cutter2D {
private: //Both motors need to be instatiated for it to work properly
    Motor1 motor1;
    Motor2 motor2;

public:
    Cutter2D(Motor1 m1, Motor2 m2) : motor1(m1), motor2(m2) {}

    void setup() {
        Serial.begin(9600);
        motor1.setup();
        motor2.setup();
        pinMode(LS1_pin, INPUT_PULLUP); //The Limit switches pins are declared as input pull ups
        pinMode(LS2_pin, INPUT_PULLUP); //Using this method implies 
        Serial.println("2D Cutter initialized.");
    }

    void home() {
        Serial.println("Homing...");
        while (digitalRead(LS1_pin) == HIGH) {
            motor1.move(-1);
        }
        Serial.println("Motor1 At Home.");
        while (digitalRead(LS2_pin) == HIGH) {
            motor2.move(-1);
        }
        Serial.println("Motor2 At Home.");
        currentX = 0.0;
        currentY = 0.0;
        Serial.println("Homing complete. Motors are at origin.");
    }

    void processSerialInput() {
        while (true) {
            if (Serial.available()) {
                String input = Serial.readStringUntil('\n');
                input.trim();

                if (input.equalsIgnoreCase("start")) {
                    Serial.println("Starting movement...");
                    for (int i = 0; i < pointCount; i++) {
                        Serial.print("Point ");
                        Serial.print(points[i][0]);
                        Serial.print(": X=");
                        Serial.print(points[i][1]);
                        Serial.print(" Y=");
                        Serial.println(points[i][2]);
                    }
                    break;
                }

                if (input.startsWith("P")) { // Change from G to P
                    int p = input.substring(1, input.indexOf('X')).toInt();
                    int x = input.substring(input.indexOf('X') + 1, input.indexOf('Y')).toInt();
                    int y = input.substring(input.indexOf('Y') + 1).toInt();

                    if (pointCount < MAX_POINTS) {
                        points[pointCount][0] = p;
                        points[pointCount][1] = x;
                        points[pointCount][2] = y;
                        pointCount++;
                    } else {
                        Serial.println("Max points reached.");
                    }
                }
            }
        }
    }

    void executePoints() {
        for (int i = 0; i < pointCount; i++) {
            float targetX = points[i][1];
            float targetY = points[i][2];

            // Calculate displacements from the current position
            float xDisplacement = targetX - currentX;
            float yDisplacement = targetY - currentY;

            Serial.print("Moving to Point ");
            Serial.print(points[i][0]);
            Serial.print(": X=");
            Serial.print(targetX);
            Serial.print(" Y=");
            Serial.println(targetY);

            // Move to the target position
            diagonalMove(xDisplacement, yDisplacement);

            // Update the current position
            currentX = targetX;
            currentY = targetY;

            Serial.print("Reached Point ");
            Serial.println(points[i][0]);
        }
    }

    void diagonalMove(float xDisplacement, float yDisplacement) {
        int xSteps = motor1.calculateSteps(xDisplacement);
        int ySteps = motor2.calculateSteps(yDisplacement);

        bool xDirection = xDisplacement > 0;
        bool yDirection = yDisplacement > 0;

        int totalSteps = max(xSteps, ySteps);
        float xStepInterval = (float)xSteps / totalSteps;
        float yStepInterval = (float)ySteps / totalSteps;

        float xAccumulator = 0;
        float yAccumulator = 0;

        for (int i = 0; i < totalSteps; i++) {
            xAccumulator += xStepInterval;
            yAccumulator += yStepInterval;

            if (xAccumulator >= 1) {
                motor1.stepOnce(xDirection);
                xAccumulator -= 1;
            }

            if (yAccumulator >= 1) {
                motor2.stepOnce(yDirection);
                yAccumulator -= 1;
            }

            delayMicroseconds(500); // Adjust for smooth motion
        }

        Serial.println("Diagonal move completed.");
    }
};
//End of 2D printer class----------------------------------------------------------------------------------------------------------------------

// Instantiate motor objects
Motor1 motor1(Motor1_pulse_pin, Motor1_dir_pin, Motor1_enable_pin, d1, motor1_pulses_rev);
Motor2 motor2(motor2_stepsPerRevolution, N1, N2, N3, N4, d2);

// Instantiate the Cutter2D object
Cutter2D cutter(motor1, motor2);

void setup() {
    cutter.setup();
    cutter.home();
    cutter.processSerialInput();
    cutter.executePoints();
}

void loop() {
    // The loop is empty because all tasks are executed in setup for now.
}
