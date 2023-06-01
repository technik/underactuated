#include <Arduino.h>

#include <pin.h>
#include <L298.h>
#include <vector.h>
#include <staticVector.h>

#include "AS5600.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Hardware defintions
AS5600 g_encoder;   //  use default Wire
Adafruit_MPU6050 g_imu;

void setup() {
    // Config serial port
    Serial.begin(115200);
    //g_Motor.Init();
    pinMode(13, OUTPUT);
    // Config IMU
    while(!g_imu.begin())
    {
        Serial.println("Can't init IMU");
        delay(1000);
    }

    Serial.println("MPU6050 Found!");
    g_imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    g_imu.setGyroRange(MPU6050_RANGE_500_DEG);
}

struct MessageParser
{
    static constexpr uint32_t kMaxMessageSize = 63;
    StaticVector<uint8_t,kMaxMessageSize+1> m_Message; // +1 to hold a terminator

    bool processMessage()
    {
        auto c = (uint8_t)Serial.read();
        if(c == ' ')
        {
            return false;
        }
        else if(c == '\n')
        {
            m_Message.push_back('\0'); // Finish the string
            return true;
        }
        else if(m_Message.size() < kMaxMessageSize) // leave room for a terminator
        {
            m_Message.push_back(c);
            return false;
        }
        return false;
    }
};

MessageParser g_pcConnection;

enum class State
{
    standby,
    balancing,
    torqueTest
} g_State = State::standby;

math::Vec3f g_gravityA, g_gravityB;

math::Vec3f readGravity()
{
    sensors_event_t a, g, temp;
    g_imu.getEvent(&a, &g, &temp);

    return math::Vec3f(a.acceleration.x, a.acceleration.y, a.acceleration.z);
}

int g_speed = 0;

void digestMessage()
{
    auto& msg = g_pcConnection.m_Message;
    if(msg.empty())
    {
        return; // Ignore blank lines
    }

    switch(msg[0])
    {
        case '0':
        {
            //g_Motor.Disable();
            g_State = State::standby;
            g_speed = 0;
            break;
        }
        case 't':
        case 'T':
        {
            g_speed = atoi((const char*)&msg[1]);
            Serial.print("Torque: ");
            Serial.println(g_speed);
            break;
        }
        case 'p':
        {
            int pos = atoi((const char*)&msg[1]);
            Serial.print("angle: ");
            Serial.println(pos);
            float angle = float(pos)/100.f;
            angle = max(0.f, angle);
            angle = min(angle, 1.f);
            //g_Motor.SetAngle(angle, g_speed);
        }
    }

    msg.clear();
}

int g_posCount = 0;

bool ledOn = false;

void loop()
{
    // Process host messages
    if(Serial.available())
    {
        if(g_pcConnection.processMessage())
        {
            digestMessage();
        }
    }

    if(g_State == State::balancing)
    {
        const int kTau = 50;
        g_posCount += (g_speed > 0 ? 1 : (kTau-1));
        g_posCount %= kTau;
        //Serial.print("Angle: ");
        //Serial.println(g_posCount);
        //g_Motor.SetAngle(g_posCount / (float(kTau-1)), g_speed);
    }

    digitalWrite(13, ledOn);
    ledOn = !ledOn;
}