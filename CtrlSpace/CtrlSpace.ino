#include <Servo.h>
#include <ArduinoJson.h>

#define SERVO_L_PIN 2
#define SERVO_C_PIN 3
#define SERVO_R_PIN 4
#define BUZZER_PIN 5

bool buzz = false;
bool isBuzzing = false;
unsigned long time;
unsigned long pastTime;
int angle_c;
int angle_l;
int angle_r;
Servo servoC; // Center
Servo servoR; // Right
Servo servoL; // Left

void setup() {
  Serial.begin(9600);
  
  // Set up servos
  servoL.attach(SERVO_L_PIN);
  servoC.attach(SERVO_C_PIN);
  servoR.attach(SERVO_R_PIN);
  // Set up buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);

  // initial variable setup
  pastTime = millis();
}

void loop() {
  // Read Serial port and get the servo angle values
  if (Serial.available()) {
    StaticJsonDocument<200> message;

    DeserializationError error = deserializeJson(message, Serial);

    if (!error) {
      angle_l = message["servoL"];
      angle_c = message["servoC"];
      angle_r = message["servoR"];
      buzz = message["buzz"];
    } else {
      angle_l = 90;
      angle_c = 90;
      angle_r = 90;
      buzz = false;
    }
  }

  // Write values to the servo
  servoL.write(angle_l);
  servoC.write(angle_c);
  servoR.write(angle_r);

  // get current time
  time = millis();

  if (buzz && !isBuzzing) {
    pastTime = millis();
    isBuzzing = true;
    // make the buzzer buzz
    tone(BUZZER_PIN, 600);
  }
  // stop the buzzer if has been buzzing for 1 sec
  if (isBuzzing && time - pastTime > 1000) {
    noTone(BUZZER_PIN);
    isBuzzing = false;
  }
}