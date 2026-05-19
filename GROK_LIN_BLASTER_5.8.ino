#include <SoftwareSerial.h>
// works fine in sniff,auto blast & listen,target shoot & listen 
const int LIN_RX   = 2;
const int LIN_TX   = 3;
const int LIN_NSLP = 4;

SoftwareSerial linBus(LIN_RX, LIN_TX);

unsigned long currentBaud = 10417; 

void wakeTJA1020() {
  digitalWrite(LIN_TX, HIGH);
  digitalWrite(LIN_NSLP, LOW);
  delayMicroseconds(50);
  digitalWrite(LIN_NSLP, HIGH);
  delayMicroseconds(80);
}

uint8_t calculatePID(uint8_t id) {
  id &= 0x3F;
  uint8_t p0 = ((id >> 0) & 1) ^ ((id >> 1) & 1) ^ ((id >> 2) & 1) ^ ((id >> 4) & 1);
  uint8_t temp = ((id >> 1) & 1) ^ ((id >> 3) & 1) ^ ((id >> 4) & 1) ^ ((id >> 5) & 1);
  uint8_t p1 = (~temp) & 1;
  return id | (p0 << 6) | (p1 << 7);
}
//___________________sniffer mode_____________
void snifferMode() {
  currentBaud = detectLinBaud();
  if (currentBaud == 0) currentBaud = DEFAULT_BAUD;

  Serial.println(F("\n=== FAST-SYNC LIN SNIFFER (15s) ==="));
  Serial.print(F("Baud: ")); Serial.println(currentBaud);
  Serial.println(F("Break   Sync   PID (ID)      Data                    CS      Status"));
  Serial.println(F("----------------------------------------------------------------------"));

  unsigned long endTime = millis() + 15000;

  while (millis() < endTime) {
    pinMode(LIN_RX, INPUT);
    while (digitalRead(LIN_RX) == HIGH) {
      if (millis() > endTime) break;
    }

    unsigned long breakStarted = micros();
    bool isBreak = true;
    while (micros() - breakStarted < 500) {
      if (digitalRead(LIN_RX) == HIGH) { isBreak = false; break; }
    }

    if (isBreak) {
      linBus.begin(currentBaud);
      uint8_t buffer[12];
      int idx = 0;
      unsigned long frameTimeout = millis() + 18; // Your optimized 18ms

      while (millis() < frameTimeout && idx < 12) {
        if (linBus.available()) { buffer[idx++] = linBus.read(); }
      }
      linBus.end();

      if (idx >= 3) {
        uint8_t pid = buffer[1];
        uint8_t rawId = pid & 0x3F;
        uint8_t receivedCS = buffer[idx - 1];
        
        Serial.print(F("Break   0x55   0x"));
        if (pid < 0x10) Serial.print('0');
        Serial.print(pid, HEX);
        Serial.print(F(" (0x"));
        if (rawId < 0x10) Serial.print('0');
        Serial.print(rawId, HEX);
        Serial.print(F(")   "));

        uint16_t sumClassic = 0;
        uint16_t sumEnhanced = pid;
        int dataCount = 0;

        for (int i = 2; i < idx - 1; i++) {
          uint8_t d = buffer[i];
          if (d < 0x10) Serial.print("0");
          Serial.print(d, HEX);
          Serial.print(" ");
          
          sumClassic += d;
          if (sumClassic >= 256) sumClassic -= 255;
          sumEnhanced += d;
          if (sumEnhanced >= 256) sumEnhanced -= 255;
          dataCount++;
        }

        uint8_t calcClassic  = (~sumClassic) & 0xFF;
        uint8_t calcEnhanced = (~sumEnhanced) & 0xFF;

        for (int i = 0; i < (8 - dataCount); i++) Serial.print("   ");

        Serial.print("CS 0x");
        if (receivedCS < 0x10) Serial.print("0");
        Serial.print(receivedCS, HEX);

        if (receivedCS == calcClassic) Serial.println(F("   [OK 1.3]"));
        else if (receivedCS == calcEnhanced) Serial.println(F("   [OK 2.x]"));
        else {
          Serial.print(F("   [ERR! C:0x"));
          Serial.print(calcClassic, HEX);
          Serial.print(F(" E:0x"));
          Serial.print(calcEnhanced, HEX);
          Serial.println(F("]"));
        }
      }
    }
  }
  Serial.println(F("=== Sniffer Timeout ===\n"));
}

// ==================== IMPROVED SHOOT AND LISTEN ====================

void shootAndListen(uint8_t id) {
  uint8_t pid = calculatePID(id);
  wakeTJA1020();
  
  linBus.end();
  pinMode(LIN_TX, OUTPUT);
  
  // Send Master Header
  digitalWrite(LIN_TX, LOW);
  delayMicroseconds(1250); 
  digitalWrite(LIN_TX, HIGH);
  delayMicroseconds(100);

  linBus.begin(currentBaud);
  linBus.write(0x55);
  linBus.write(pid);
  linBus.flush(); 

  // Wait a tiny bit for the physical line to settle
  delayMicroseconds(200);

  uint8_t rxBuf[12];
  int rIdx = 0;
  unsigned long waitLimit = millis() + 30; 
  
  while (millis() < waitLimit && rIdx < 12) {
    if (linBus.available()) {
      uint8_t b = linBus.read();
      // Ignore the echo of the PID we just sent
      if (rIdx == 0 && (b == pid || b == 0xFF || b == 0x55)) continue; 
      rxBuf[rIdx++] = b;
    }
  }
  
  // Print even if silent so user knows TX happened
  Serial.print(F("TX ID 0x")); if(id < 0x10) Serial.print('0'); Serial.print(id, HEX);
  
  if (rIdx > 0) {
    Serial.print(F(" -> RX: "));
    for (int i = 0; i < rIdx; i++) {
      if (rxBuf[i] < 0x10) Serial.print('0');
      Serial.print(rxBuf[i], HEX); Serial.print(' ');
    }
  } else {
    Serial.print(F(" -> [NO RESPONSE]"));
  }
  Serial.println();
  
  linBus.end();
}
//---------------------detect baud----------------
long detectLinBaud() {
  const int samplesToTake = 5;
  long baudSum = 0;
  int successfulSamples = 0;
  Serial.println(F("Detecting Baud Rate..."));
  pinMode(LIN_RX, INPUT);
  unsigned long timeout = millis() + 4000;
  while (successfulSamples < samplesToTake && millis() < timeout) {
    unsigned long breakWidth = pulseIn(LIN_RX, LOW, 100000);
    if (breakWidth > 800) {
      long frameTotalTime = 0;
      int pulsesFound = 0;
      for (int i = 0; i < 4; i++) {
        long pulse = pulseIn(LIN_RX, LOW, 2000);
        if (pulse > 0) { frameTotalTime += pulse; pulsesFound++; }
      }
      if (pulsesFound == 4) {
        long estBaud = 8000000UL / (frameTotalTime * 2);
        baudSum += estBaud;
        successfulSamples++;
      }
    }
  }
  if (successfulSamples == 0) return 0;
  long avg = baudSum / successfulSamples;
  if (avg > 9000 && avg < 12000) return 10417;
  if (avg > 18000 && avg < 21000) return 19200;
  return avg;
}

// ==================== INTERFACE LOGIC ====================

void setup() {
  Serial.begin(115200);
  pinMode(LIN_TX, OUTPUT);
  pinMode(LIN_NSLP, OUTPUT);
  wakeTJA1020();
  
  Serial.println(F("\n=== LIN TOOL LOADED ==="));
  Serial.println(F("Commands: L (Sniff), A (Sweep), T (Target)"));
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.equalsIgnoreCase("L")) {
      Serial.println(F("Passive Sniffing..."));
      snifferMode();
    } 
    else if (input.equalsIgnoreCase("A")) {
      Serial.println(F("Starting Sweep..."));
      for (int i = 0; i <= 0x3F; i++) {
        shootAndListen(i);
        delay(20);
      }
    } 
    else if (input.equalsIgnoreCase("T")) {
      Serial.println(F("Enter ID (Hex) to Target:"));
      while (Serial.available() == 0); // Wait for input
      String targetStr = Serial.readStringUntil('\n');
      targetStr.trim();
      uint8_t targetId = (uint8_t)strtol(targetStr.c_str(), NULL, 16);
      
      Serial.print(F("Targeting 0x")); Serial.println(targetId, HEX);
      for(int j=0; j<10; j++) {
        shootAndListen(targetId);
        delay(200);
      }
    }
  }
}
