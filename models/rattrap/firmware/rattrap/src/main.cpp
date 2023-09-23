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
bool g_ImuOk = false;

L298<10,9,6,5> g_MotorController;

template<int PulsePin>
struct SRF05
{
public:
    void init()
    {
        //
    }

    int measure() const
    {
        // Trigger pulse
        pinMode(PulsePin, OUTPUT);
        digitalWrite(PulsePin, HIGH);
        delayMicroseconds(10);
        digitalWrite(PulsePin, LOW);

        // Wait for response
        pinMode(PulsePin, INPUT);
        while(digitalRead(PulsePin) == LOW)
        {} // Wait for rise
        long t0 = micros();
        while(digitalRead(PulsePin) == HIGH)
        {
            //if((micros() - t0) > 100000)
            //{
            //    return -1;
            //}
        } // Wait for low
        long dt = micros() - t0;
        pinMode(PulsePin, OUTPUT);
        return (dt * 17) / 50; // mm
    }
};

enum class State
{
    standby,
    relay,
    calibrate,
    track,
    rotate
} g_State = State::standby;

void setup() {
    // Config serial port
    Serial.begin(115200);
    g_MotorController.Disable();
    pinMode(13, OUTPUT);
    Serial.println("Init");
    // Config IMU
    for(int i = 0; i < 4; ++i)
    {
        if(g_imu.begin())
        {
            g_ImuOk = true;
            break;
        }
        else
        {
            delay(100);
        }
    }
    if(g_ImuOk)
    {
        Serial.println("IMU ok");

        g_imu.setAccelerometerRange(MPU6050_RANGE_8_G);
        g_imu.setGyroRange(MPU6050_RANGE_500_DEG);
    }
    else
    {
        Serial.println("Can't init IMU");
    }

    g_State = State::calibrate;

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
            g_MotorController.Disable();
            g_State = State::standby;
            g_speed = 0;
            break;
        }
        case '1':
        {
            g_MotorController.Disable();
            g_State = State::relay;
            break;
        }
        case '2':
        {
            g_State = State::track;
            break;
        }
        case 'c':
        case 'C': // Calibrate
        {
            g_MotorController.Disable();
            g_State = State::calibrate;
            break;
        }
        case 'r':
        case 'R': // Rotate for calibration
        {
            g_MotorController.Disable();
            g_State = State::rotate;
            break;
        }
    }

    msg.clear();
}

int g_target = 200;
SRF05<8> g_frontSensor;

bool ledOn = false;

float readGyro()
{
    sensors_event_t a, g, temp;
    g_imu.getEvent(&a, &g, &temp);
    return g.gyro.z;
}

void loop()
{
    // Process host messages
    if(Serial.available())
    {
        if(g_pcConnection.processMessage())
        {
            Serial.println("copy");
            digestMessage();
        }
    }

    if(g_State == State::calibrate)
    {
        // Average a few reads
        long target = 0;
        const int N = 4;
        for(int i = 0; i < N; ++i)
        {
            target += g_frontSensor.measure();
            delay(25);
        }
        target /= N;
        Serial.print("target: ");
        Serial.println(target);
        g_target = target;

        g_State = State::track;
    }

    if(g_State == State::relay)
    {
        int y = g_frontSensor.measure();

        Serial.print("y:");
        Serial.println(y);

        delay(250);
    }

    if(g_State == State::track)
    {
        const float kGyro = -100/3.5;
        int y = g_frontSensor.measure();

        // read the gyro
        float errGyro = -kGyro * readGyro();

        int speed = 80;
        int tolerance = 40;
        if(y > g_target + tolerance)
        {
            g_MotorController.channelA.Write(speed + errGyro);
            g_MotorController.channelB.Write(speed - errGyro);
        }
        else if(y < g_target - tolerance)
        {
            g_MotorController.channelA.Write(-speed + errGyro);
            g_MotorController.channelB.Write(-speed - errGyro);
        }
        else
        {
            g_MotorController.Disable();
        }
    }
    
    if(g_State == State::rotate)
    {
        int speed = 100;
        if((millis() / 4000) & 1) // Alternate rotation directions
        {
            //speed = -speed;
        }

        g_MotorController.channelA.Write(speed);
        g_MotorController.channelB.Write(-speed);

        // Read the gyro
        Serial.println(readGyro());
    }

    digitalWrite(13, ledOn);
    ledOn = !ledOn;
}