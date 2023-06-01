#pragma once

#include <cstdint>

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