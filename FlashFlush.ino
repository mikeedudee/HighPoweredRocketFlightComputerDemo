/// Append one DataPoint to "/data.txt" on SPIFFS.  
void saveDataPoint(const DataPoint &pt) {
  File file = SPIFFS.open(SPIFFS_FILENAME, FILE_APPEND);

  if (!file) {
    Serial.println("Failed to open data.txt for appending");
    errorBlink();
    return;
  }

  // Format: temperature (float), pressure (long), altitudeFiltered (float), timestamp (ulong)
  file.printf(
    "%.2f %ld %.2f %lu\n",
    pt.temperature,
    pt.pressure,
    pt.altitudeFiltered,
    pt.timestamp
  );
  file.close();
}

/// Initialize SPIFFS. If mount fails, print and halt.
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    while (true) {
      errorBlink();       // never returns, red LED will blink
      freeKalmanFilter(); // Free the Kalman filter since we won’t need it any more
    }
  }
  Serial.println("SPIFFS Initialized");
}


/// Write all buffered samples to SPIFFS, then reset the buffer.
void flushBufferToSPIFFS() {
  int count = bufferIsFull ? BUFFER_SIZE : bufferIndex;
  Serial.println("Flushing buffer:");
  for (int i = 0; i < count; i++) {
    saveDataPoint(bufferArray[i]);
  }
  // Reset buffer pointers:
  bufferIndex  = 0; bufferIsFull = false;
}

/// Insert one DataPoint into the circular buffer.
///   - If buffer not yet full, store at bufferIndex++.
///   - Once bufferIndex == BUFFER_SIZE, set bufferIsFull = true and
///     shift all contents one place left on each subsequent insertion.
void addToBuffer(const DataPoint &pt) {
  if (!bufferIsFull) {
      bufferArray[bufferIndex++] = pt;
      if (bufferIndex >= BUFFER_SIZE) bufferIsFull = true;
  } else {
    // Shift everything left by one, then append new at the end
    // We use memmove since its more efficient and this single call is both clearer and faster on a microcontrollers.
    memmove(bufferArray, bufferArray + 1, sizeof(DataPoint)*(BUFFER_SIZE-1));
    bufferArray[BUFFER_SIZE-1] = pt;
  }
}
