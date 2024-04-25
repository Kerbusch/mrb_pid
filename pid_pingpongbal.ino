#include <Servo.h>
#include <HCSR04.h>

// pinnen
const int triggerPin = 3;
const int echoPin = 2;
const int servoPin = 10;
const int ledPin = 22;

// servo object
Servo servoMotor;

// PID parameters
double kp = 1.5;
double ki = 0.001;
double kd = 12;
double setpoint = 40; // Distance to maintain (in cm)

// PID variabelen
double integral = 0;
double lastError = 0;

// Median filter
class median_filter{
private:
    double values[3] = {0.0, 0.0, 0.0};

public:
    double calc(double x){
        // verplaats waarden en "drop" oude.
        values[2] = values[1];
        values[1] = values[0];

        // x toevoegen
        values[0] = x;

        // temp maken
        double temp[3];
        memcpy(temp, values, sizeof(values[0])*3);

        // sorteer
        if(temp[1] < temp[0]){
            double temp_value = temp[0];
            temp[0] = temp[1];
            temp[1] = temp_value;
        }
        if(temp[2] < temp[1]){
            double temp_value = temp[1];
            temp[1] = temp[2];
            temp[2] = temp_value;
        }
        if(temp[1] < temp[0]){
            double temp_value = temp[0];
            temp[0] = temp[1];
            temp[1] = temp_value;
        }

        // return median
        return temp[1];
    }
};

// median filter object
median_filter medianFilter;

bool led_state = false;

void setup() {
    // Start serial
    Serial.begin(9600);

    // Start servo
    servoMotor.attach(servoPin);

    // Start HC-SR04
    HCSR04.begin(triggerPin, echoPin);

    // Start LED
    pinMode(ledPin, OUTPUT);
}

void loop() {
    // Check if serial data is available
    if (Serial.available() > 0) {
        // Read the incoming byte
        char incomingByte = Serial.read();

        // If the incoming byte is a digit, update setpoint
        if (isdigit(incomingByte)) {
            setpoint = (incomingByte - '0') * 10; // Convert ASCII to integer and multiply by 10
        }
    }

    double distance = *HCSR04.measureDistanceCm();
    double rawDistance = distance;

    // Als distance meer is dan het einde van de balk.
    if (distance > 80.0){
        distance = 80.0;
    }
    // Else if distance is -1, probeer opnieuw.
    else if(distance <= -1.0){
        while(distance <= -1.0 || distance > 80.0){
            delay(10);
            distance = *HCSR04.measureDistanceCm();
        }
    }

    distance = medianFilter.calc(distance); // Median filter over distance

    // PID berekentingen
    double error = setpoint - distance;
    integral += error;
    double derivative = error - lastError;
    double output = kp * error + ki * integral + kd * derivative;

    // output omkeren (ander gaat de bal naar de verkeerde kant)
    output *= -1.0;

    // output limiteren
    if (output > 80.0) {
        output = 80.0;
    } else if (output < -80.0) {
        output = -80.0;
    }

    // servo aansturen (80 graden is het middenpunt, wanneer de bal stopt.
    servoMotor.write(80 + output); // Map output to servo position

    // Update last error
    lastError = error;

    // Print
    // Serial.print("Distance: ");
    // Serial.print(distance);
    // Serial.print(" cm, Output: ");
    // Serial.print(output);
    // Serial.print(", rawDistance: ");
    // Serial.print(rawDistance);
    // Serial.print(", ki * integral: ");
    // Serial.print(ki * integral);
    // Serial.print(", kd * derivative: ");
    // Serial.print(kd * derivative);
    // Serial.print(", distance from setpoint: ");
    // Serial.println(setpoint-distance);
    // Serial.print(", servo setting: ");
    // Serial.println(90 + output);

    // Serial.print("derivative: ");
    // Serial.println(derivative);
    
    Serial.print("distance from setpoint: ");
    Serial.println(setpoint-distance);

    //led, om te checken waarom alles stil loopt. Het bleek dat de servo te veel stroom gebruikt.
    if (led_state){
        digitalWrite(ledPin, HIGH);
    }else{
        digitalWrite(ledPin, LOW);
    }
    led_state = !led_state;

    delay(50);
}