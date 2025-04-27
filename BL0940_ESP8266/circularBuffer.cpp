#include "circularBuffer.hpp"

CircularBuffer::CircularBuffer() : writeIndex(0), readIndex(0) {}

void CircularBuffer::saveData(const String& mqttMessage) {
    dataQueue[writeIndex] = mqttMessage;
    writeIndex = (writeIndex + 1) % MAX_DATA;

    if (writeIndex == readIndex) {
        readIndex = (readIndex + 1) % MAX_DATA; // Overwrite oldest data
    }

    Serial.println("Dato guardado en índice " + String(writeIndex));
}

String CircularBuffer::getSavedData() {
    if (readIndex == writeIndex) {
        return ""; // No new data
    }

    String data = dataQueue[readIndex];
    readIndex = (readIndex + 1) % MAX_DATA;

    return data;
}

int CircularBuffer::amountOfSavedData() const {
    int savedDataQty = (writeIndex >= readIndex)
                       ? writeIndex - readIndex
                       : MAX_DATA - (readIndex - writeIndex);

    Serial.print("Saved data: ");
    Serial.println(savedDataQty);

    return savedDataQty;
}
