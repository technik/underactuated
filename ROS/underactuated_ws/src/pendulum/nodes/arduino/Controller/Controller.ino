#include <Wire.h>
#include <AS5600.h>

AMS_5600 ams5600;

// Motor connections
int in1 = 5;
int in2 = 6;

const float CALIB_0_OFFSET = 295.05;
const float Hz = 20;
const float dTime = 1/Hz;
float angle = 0;
float prevAngle = 0;
float vel = 0;

// Pendulum Params
float m = 0.070;
float l = 0.280;
float g = 9.81;


void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // Set up L298N
  // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  
  // Setup AS5600
  while(ams5600.detectMagnet() != 1 ){  
    Serial.println("Can not detect magnet");
    delay(1000);
  }
  Serial.print("Current Magnitude: ");
  Serial.println(ams5600.getMagnitude());

}

// This function lets you control spinning direction of motors
void setMotorVoltage(bool dir, uint8_t speed) {
	// Set motors to maximum speed
	// For PWM maximum possible values are 0 to 255

	// Turn on motor A
	analogWrite(in1, dir ? speed : 0);
	analogWrite(in2, dir ? 0 : speed);
}

float convertRawAngleToDegrees(word newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087890625 of a degree */
  return newAngle * 0.087890625 - CALIB_0_OFFSET;
}

float deg2rad(float angle)
{
  return PI*angle/180;
}

float potentialEnergy(float angle)
{
  return m*g*l*(1-cos(angle));
}

float kineticEnergy(float vel)
{
  return 0.5*m*vel*vel*l*l;
}

float energy(float angle, float vel)
{
  return potentialEnergy(angle) + kineticEnergy(vel);
}

void loop()
{
    long int t0 = millis();

    setMotorVoltage(false,100);
    prevAngle = angle;
    angle = deg2rad(convertRawAngleToDegrees(ams5600.getRawAngle()));
    vel = (angle-prevAngle);
    if (vel>PI)
      vel -= 2*PI;
    else if (vel < -PI)
      vel += 2*PI;
    vel *= Hz;
    // Serial.print(energy(angle, vel));
    Serial.print(potentialEnergy(angle));
    Serial.print(",");
    Serial.print(kineticEnergy(vel));
    Serial.print(",");
    Serial.println(energy(angle,vel));

    while(millis()-t0 < 50)
    {}
}