#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <Arduino.h>

class CircularBuffer {
public:
    CircularBuffer();

    void saveData(const String& mqttMessage);
    String getSavedData();
    int amountOfSavedData() const;

private:
    static const int MAX_DATA = 1440;
    String dataQueue[MAX_DATA];
    int writeIndex;
    int readIndex;
};

#endif // CIRCULAR_BUFFER_HPP
