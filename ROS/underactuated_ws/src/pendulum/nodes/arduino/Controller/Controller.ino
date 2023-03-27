#include <Wire.h>
#include <AS5600.h>

template<int PinA, int PinB>
class PWMMotor
{
  public:
    PWMMotor()
    {
      // Configure pins
      pinMode(PinA, OUTPUT);
      pinMode(PinB, OUTPUT);

      // Set initial state
      digitalWrite(PinA, LOW);
      digitalWrite(PinB, LOW);
    }

    static void SetPWM(bool dir, uint8_t duty)
    {
      analogWrite(PinA, dir ? duty : 0);
      analogWrite(PinB, dir ? 0 : duty);
    }

    void Disable()
    {
      digitalWrite(PinA, LOW);
      digitalWrite(PinB, LOW);
    }
};

// Motor connections
constexpr int motorPin1 = 5;
constexpr int motorPin2 = 6;
using PendulumMotor = PWMMotor<motorPin1, motorPin2>;
PendulumMotor g_Motor;

class AngleSensor
{
public:
  bool Ready()
  {
    return m_ams5600.detectMagnet() == 1;
  }

  void Calibrate()
  {
    m_offset = ReadRaw();
  }

  float GetOffsetRadians() const
  {
    return stepToRad * m_offset;
  }

  uint16_t ReadRaw()
  {
    return m_ams5600.getRawAngle();
  }

  float ReadRadians()
  {
    auto steps = int(ReadRaw() - m_offset);
    return stepToRad * steps;
  }

  AMS_5600 m_ams5600;
private:
  static inline constexpr float stepToRad = 2*PI / 4095.f;
  uint16_t m_offset;
} g_Sensor;

const float Hz = 20;
const float dTime = 1/Hz;
float angle = 0;
float prevAngle = 0;
float vel = 0;

// Pendulum Params
constexpr float m = 0.070;
constexpr float l = 0.280;
constexpr float g = 9.81;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // Setup AS5600
  //while(g_Sensor.m_ams5600.detectMagnet() != 1)//.Ready())
  while(!g_Sensor.Ready())
  {  
    Serial.println("Can not detect magnet");
    delay(1000);
  }
  
  g_Sensor.Calibrate();
  Serial.println("Ready");
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

bool g_ControlEnabled = false;

void loop()
{
    long int t0 = millis();

  // Poll control messages
  if(Serial.available())
  {
    auto c = Serial.read();
    switch(c)
    {
      case '1': // Turn on
        g_ControlEnabled = true;
      break;
      case '0': // Turn off control
        g_ControlEnabled = false;
        g_Motor.Disable();
      break;
      case 'c': // Calibrate 0
        g_Sensor.Calibrate();
        Serial.println(g_Sensor.GetOffsetRadians());
      break;
    }
  }

  prevAngle = angle;
  angle = g_Sensor.ReadRadians();
  vel = (angle-prevAngle);
  if (vel>PI)
    vel -= 2*PI;
  else if (vel < -PI)
    vel += 2*PI;
  vel *= Hz;

  if(g_ControlEnabled)
  {
    constexpr float EnergyGoal = 2*m*g*l;
    auto curEnergy = energy(angle, vel);

    bool sign = 0;
    if(curEnergy < EnergyGoal)
    {
      sign = vel > 0;
      PendulumMotor::SetPWM(sign, 100);
    }
    else
    {
      g_Motor.Disable();
    }
  }

  Serial.print(angle);
  Serial.print(",");
  Serial.print(potentialEnergy(angle));
  Serial.print(",");
  Serial.print(kineticEnergy(vel));
  Serial.print(",");
  Serial.println(energy(angle,vel));

  while(millis()-t0 < 50)
  {}
}
