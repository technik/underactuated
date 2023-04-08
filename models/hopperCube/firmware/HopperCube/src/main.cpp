#include <Arduino.h>

#include <pin.h>
#include <L298.h>
#include <vector.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct BLDCController
{
    HBridge<2,9> m_ChannelA;
    HBridge<3,10> m_ChannelB;
    HBridge<4,11> m_ChannelC;

    //
    void Init()
    {}

    void Write(int16_t speed)
    {
        // Step once
        auto s = steps[m_nextStep];
        if(speed > 0)
        {
            m_nextStep = (m_nextStep+1)%6;
        }
        else{
            m_nextStep = (m_nextStep+5)%6;
        }

        digitalWrite(2,  s&1 ? HIGH : LOW);
        analogWrite(9,  s&2 ? abs(speed) : 0);
        digitalWrite(3,  s&4 ? HIGH : LOW);
        analogWrite(10, s&8 ? abs(speed) : 0);
        digitalWrite(4,  s&16 ? HIGH : LOW);
        analogWrite(11, s&32 ? abs(speed) : 0);
        //m_ChannelA.Write(s&1 ? (s&2 ? speed : -speed) : 0);
        //m_ChannelB.Write(s&4 ? (s&8 ? speed : -speed) : 0);
        //m_ChannelC.Write(s&16 ? (s&32 ? speed : -speed) : 0);
    }

    void Disable()
    {
        m_ChannelA.Disable();
        m_ChannelB.Disable();
        m_ChannelC.Disable();
    }

    uint8_t steps[6] = 
    { // For each channel (enable, dir)
        0b100011,
        0b101100,
        0b001110,
        0b110010,
        0b111000,
        0b001011
    };

    uint8_t m_nextStep;
} g_Motor;

// Hardware defintions

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

void setup() {
    // Config serial port
    Serial.begin(115200);

    // Config IMU
    //while(!g_imu.begin())
    //{
    //    Serial.println("Can't init IMU");
    //    delay(1000);
    //}
//
    //Serial.println("MPU6050 Found!");
    Serial.println("Ready");

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
            g_Motor.Write(g_speed);
            g_State = State::torqueTest;
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
        case '1':
        {
            g_State = State::balancing;
            break;
        }
    }

    msg.clear();
}

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
        auto gravity = readGravity();
        float invNorm = 1.f / gravity.norm();
        auto down = dot(gravity, g_balanceDown) * invNorm;
        auto side = dot(gravity, g_offBalance) * invNorm;

        Serial.print("Down: ");
        Serial.print(down);
        Serial.print(", Side: ");
        Serial.println(side);
        delay(200);
    }
    else if(g_State == State::torqueTest)
    {
        g_Motor.Write(g_speed);
        delay(5);
    }
}