#include <SoftwareSerial.h>
// works fine as LIN sniffer with 2.xx check sum and classic 1.3 checksum and direction
const int linRX = 2; 
SoftwareSerial linBus(2, 3);
long detectedBaud = 0;
uint8_t frameBuffer[20];
int bufIdx = 0;
unsigned long lastByteTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("=== LIN Bus Monitor - General Purpose Sniffer ===");
  Serial.println("Showing all frames with both Classic and Enhanced checksums");
  Serial.println();
}

void loop() {       
  if (detectedBaud == 0) {
    detectedBaud = autoDetectBaud(linRX);
    if (detectedBaud > 0) {
      Serial.print("Detected Baud: "); 
      Serial.println(detectedBaud);
      Serial.println();
      linBus.begin(detectedBaud);
    }
    return;
  }

  while (linBus.available()) {
    uint8_t b = linBus.read();
    lastByteTime = millis();
    
    if (b == 0x55) {
      if (bufIdx > 0) {
        processLastFrame();
      }
      bufIdx = 0;
      frameBuffer[bufIdx++] = b;
    } 
    else if (bufIdx > 0) {
      if (bufIdx < 20) {
        frameBuffer[bufIdx++] = b;
      }
      
      if (bufIdx >= 3 && linBus.available() == 0) {
        delay(2);
        if (linBus.available() == 0) {
          processLastFrame();
          bufIdx = 0;
        }
      }
    }
    
    if (bufIdx > 0 && (millis() - lastByteTime) > 15) {
      if (bufIdx >= 3) {
        processLastFrame();
      }
      bufIdx = 0;
    }
  }
}

long autoDetectBaud(int pin) {
  unsigned long timeout = millis() + 5000;
  long bitWidth = 0;
  
  while (millis() < timeout) {
    long breakWidth = pulseIn(pin, LOW, 500000);
    if (breakWidth > 500) {
      bitWidth = pulseIn(pin, LOW, 10000);
      if (bitWidth > 0) {
        long raw = 1000000 / bitWidth;
        if (raw > 18000 && raw < 21000) return 19200;
        if (raw > 9500 && raw < 11500)  return 10417;
        return raw;
      }
    }
    delay(1);
  }
  return 0;
}

void processLastFrame() {
  if (bufIdx < 3) return;
  
  uint8_t syncByte = frameBuffer[0];
  uint8_t fullPID = frameBuffer[1];
  uint8_t rawID = fullPID & 0x3F;
  uint8_t receivedChecksum = frameBuffer[bufIdx - 1];
  uint8_t dataBytesCount = bufIdx - 3;
  
  // Try both checksum types
  uint8_t finalDetectedLen = 0;
  uint8_t enhancedCalc = 0;
  uint8_t classicCalc = 0;
  bool enhancedValid = false;
  bool classicValid = false;
  uint8_t validLengthsEnhanced[10];
  uint8_t validLengthsClassic[10];
  int enhancedCount = 0;
  int classicCount = 0;
  
  // Try all possible data lengths for both checksum types
  for (uint8_t len = 0; len <= dataBytesCount && len <= 8; len++) {
    // Enhanced checksum (includes PID)
    uint16_t sumEnhanced = fullPID;
    for (uint8_t j = 0; j < len; j++) {
      sumEnhanced += frameBuffer[2 + j];
      if (sumEnhanced >= 256) sumEnhanced -= 255;
    }
    uint8_t calcEnhanced = (~sumEnhanced) & 0xFF;
    if (calcEnhanced == receivedChecksum) {
      validLengthsEnhanced[enhancedCount++] = len;
      if (!enhancedValid) {
        enhancedCalc = calcEnhanced;
        enhancedValid = true;
      }
    }
    
    // Classic checksum (excludes PID)
    uint16_t sumClassic = 0;
    for (uint8_t j = 0; j < len; j++) {
      sumClassic += frameBuffer[2 + j];
      if (sumClassic >= 256) sumClassic -= 255;
    }
    uint8_t calcClassic = (~sumClassic) & 0xFF;
    if (calcClassic == receivedChecksum) {
      validLengthsClassic[classicCount++] = len;
      if (!classicValid) {
        classicCalc = calcClassic;
        classicValid = true;
      }
    }
  }
  
  // Determine which checksum type to use
  bool isEnhanced = false;
  bool isClassic = false;
  uint8_t calculated = 0;
  
  // Prefer enhanced checksum for slave responses, classic for master requests
  if (enhancedValid && (rawID >= 0x10 || enhancedCount > 0)) {
    // Use longest valid length for enhanced
    for (int i = 0; i < enhancedCount; i++) {
      if (validLengthsEnhanced[i] > finalDetectedLen) {
        finalDetectedLen = validLengthsEnhanced[i];
      }
    }
    if (finalDetectedLen > 0) {
      isEnhanced = true;
      // Recalculate with selected length
      uint16_t sum = fullPID;
      for (uint8_t j = 0; j < finalDetectedLen; j++) {
        sum += frameBuffer[2 + j];
        if (sum >= 256) sum -= 255;
      }
      calculated = (~sum) & 0xFF;
    }
  }
  
  if (!isEnhanced && classicValid) {
    // Use longest valid length for classic
    for (int i = 0; i < classicCount; i++) {
      if (validLengthsClassic[i] > finalDetectedLen) {
        finalDetectedLen = validLengthsClassic[i];
      }
    }
    if (finalDetectedLen > 0) {
      isClassic = true;
      // Recalculate with selected length
      uint16_t sum = 0;
      for (uint8_t j = 0; j < finalDetectedLen; j++) {
        sum += frameBuffer[2 + j];
        if (sum >= 256) sum -= 255;
      }
      calculated = (~sum) & 0xFF;
    }
  }
  
  // If nothing valid, use max length
  if (!isEnhanced && !isClassic) {
    finalDetectedLen = dataBytesCount;
    uint16_t sum = fullPID;
    for (uint8_t j = 0; j < finalDetectedLen; j++) {
      sum += frameBuffer[2 + j];
      if (sum >= 256) sum -= 255;
    }
    calculated = (~sum) & 0xFF;
  }
  
  // Determine direction based on checksum type and data
  bool isSlaveResponse = (dataBytesCount > 0) && (isEnhanced || (!isClassic && dataBytesCount > 0));
  
  // Output - Clean format without decoding
  Serial.print("[");
  Serial.print(millis());
  Serial.print("ms] ");
  
  if (dataBytesCount == 0) {
    Serial.print("→ Master ");
  } else if (isSlaveResponse && isEnhanced) {
    Serial.print("← Slave  ");
  } else if (dataBytesCount > 0 && isClassic) {
    Serial.print("→ Master ");
  } else {
    Serial.print("? Unknown ");
  }
  
  Serial.print("ID:0x");
  if (rawID < 0x10) Serial.print("0");
  Serial.print(rawID, HEX);
  
  Serial.print(" PID:0x");
  if (fullPID < 0x10) Serial.print("0");
  Serial.print(fullPID, HEX);
  
  // Show raw data
  if (dataBytesCount > 0) {
    Serial.print(" [");
    Serial.print(dataBytesCount);
    Serial.print("] ");
    
    for (int i = 2; i < bufIdx - 1; i++) {
      if (frameBuffer[i] < 0x10) Serial.print("0");
      Serial.print(frameBuffer[i], HEX);
      Serial.print(" ");
    }
  } else {
    Serial.print(" (Header)");
  }
  
  // Show checksum info
  Serial.print(" CS:0x");
  if (receivedChecksum < 0x10) Serial.print("0");
  Serial.print(receivedChecksum, HEX);
  
  if (isEnhanced) {
    Serial.print(" ✓ Enhanced");
    if (enhancedCount > 1) {
      Serial.print(" (");
      Serial.print(enhancedCount);
      Serial.print(" lengths)");
    }
  } else if (isClassic) {
    Serial.print(" ✓ Classic");
    if (classicCount > 1) {
      Serial.print(" (");
      Serial.print(classicCount);
      Serial.print(" lengths)");
    }
  } else {
    Serial.print(" ✗");
    Serial.print(" (E:0x");
    if (enhancedCalc < 0x10) Serial.print("0");
    Serial.print(enhancedCalc, HEX);
    Serial.print(" C:0x");
    if (classicCalc < 0x10) Serial.print("0");
    Serial.print(classicCalc, HEX);
    Serial.print(")");
  }
  
  Serial.println();
}
