#include <Arduino.h>

#include <pin.h>
#include <L298.h>
#include <vector.h>

#include <BLDCDrivers.h>
#include "AS5600.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Hardware defintions
AS5600 g_encoder;   //  use default Wire

#if 0 // Choose controller implementation
TrapezoidalController<
    BLDCAnalogChannel<Pin2, 9>,
    BLDCAnalogChannel<Pin3, 10>, 
    BLDCAnalogChannel<Pin4, 11> 
> g_Motor;
#else
SinglePortController<GPIOPortC> g_Motor;
#endif

Adafruit_MPU6050 g_imu;

void demoIMU()
{
    // Read IMU
    sensors_event_t a, g, temp;
    g_imu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    Serial.println("");
    delay(500);
}

void CalibrateMotor()
{
    Serial.print("Calibrating motor");

    g_Motor.SetAngle(0, 50); // Move to the reference position
    delay(500); // Give it time to settle
    g_Motor.Disable();

    Serial.println("...Done");
}

void setup() {
    // Config serial port
    Serial.begin(115200);
    g_Motor.Init();
    pinMode(13, OUTPUT);
    // Config IMU
    //while(!g_imu.begin())
    //{
    //    Serial.println("Can't init IMU");
    //    delay(1000);
    //}
//
    //Serial.println("MPU6050 Found!");
    CalibrateMotor();

    g_imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    g_imu.setGyroRange(MPU6050_RANGE_500_DEG);
}

template<class T, uint32_t Capacity>
class StaticVector
{
public:
    StaticVector()
        : m_End(m_Data)
    {}

    static uint32_t capacity() { return Capacity; }
    uint32_t size() const { return uint32_t(m_End - m_Data); }
    bool empty() const { return m_End == m_Data; }
    bool full() const { return size() == Capacity; }

    T* begin() { return m_Data; }
    T* end() { return m_End; }
    const T* begin() const { return m_Data; }
    const T* end() const { return m_End; }

    void push_back(T x)
    {
        *m_End = x;
        ++m_End;
    }

    void pop_back()
    {
        --m_End;
    }

    T& front() { return m_Data[0]; }
    const T& front() const { return m_Data[0]; }
    T& back() { return *(m_End-1); }
    const T& back() const { return *(m_End-1); }

    T* data() { return m_Data; }
    const T* data() const { return m_Data; }

    T& operator[](uint32_t i) { return m_Data[i]; }
    const T& operator[](uint32_t i) const { return m_Data[i]; }

    void clear() { m_End = m_Data; }

private:
    T* m_End;
    T m_Data[Capacity];
};

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
    calibrateA, // Calibrate start face
    calibrateB, // Calibrate end face
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

void calibrateFace(math::Vec3f& gravity)
{
    g_State = State::standby;
    constexpr int NumMeasures = 20;

    for(int  i = 0; i < NumMeasures; ++i)
    {
        // Read IMU and accumulate
        gravity += readGravity();

        delay(10); // wait a few milliseconds between measurements
    }

    // Normalize the average
    gravity /= float(NumMeasures);

    Serial.print("Average Gravity vector: (");
    Serial.print(gravity.x());
    Serial.print(", ");
    Serial.print(gravity.y());
    Serial.print(", ");
    Serial.print(gravity.z());
    Serial.println(")");

    Serial.print("Magnitude: ");
    Serial.println(gravity.norm());
}

math::Vec3f g_balanceDown;
math::Vec3f g_offBalance;

void recomputeFrame()
{
    math::Vec3f rotationAxis = normalize(cross(g_gravityA, g_gravityB));
    g_balanceDown = normalize(g_gravityA + g_gravityB);
    g_offBalance = cross(g_balanceDown, rotationAxis);
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
            g_Motor.Disable();
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
        case 'A':
        case 'a':
        {
            calibrateFace(g_gravityA);
            break;
        }
        case 'B':
        case 'b':
        {
            calibrateFace(g_gravityB);
            break;
        }
        case 'c':
        {
            recomputeFrame();
            Serial.println("Frame ready");
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
            g_Motor.SetAngle(angle, g_speed);
        }
        case '1':
        {
            g_State = State::balancing;
            break;
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
        g_posCount += (g_speed > 0 ? 1 : 99);
        g_posCount %= 100;
        //Serial.print("Angle: ");
        //Serial.println(g_posCount);
        g_Motor.SetAngle(g_posCount / 99.f, g_speed);
    }

    digitalWrite(13, ledOn);
    ledOn = !ledOn;
}