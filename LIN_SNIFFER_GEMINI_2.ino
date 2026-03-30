#include <SoftwareSerial.h>

const int linRX = 2; 
//SoftwareSerial linBus(linRX, -1); // RX only for sniffing
SoftwareSerial linBus(2, 3);
long detectedBaud = 0;
uint8_t frameBuffer[12];
int bufIdx = 0;

void setup() {
  Serial.begin(115200);
 // pinMode(linRX, INPUT);
  Serial.println("--- LIN 2.x Auto-Baud Sniffer Starting ---");
}

void loop() {
       
  // 1. If baud is not yet detected, find it from the bus traffic
  if (detectedBaud == 0) {
    detectedBaud = autoDetectBaud(linRX);
    if (detectedBaud > 0) {
      Serial.print("Detected Baud: "); Serial.println(detectedBaud);
      linBus.begin(detectedBaud);
      //linBus.begin(10417);
    }
    return;
  }

  // 2. Read incoming traffic
  if (linBus.available()) {
    uint8_t b = linBus.read();
    
    // LIN Sync Field is always 0x55
    if (b == 0x55) {
      processLastFrame(); // Process the previous frame before starting new one
      bufIdx = 0;
    }
    
    if (bufIdx < 12) frameBuffer[bufIdx++] = b;
  }
}

// Measures the bit-width of the 0x55 Sync byte to find the baud rate
long autoDetectBaud(int pin) {
  // Wait for the 'Break' (Low signal > 13 bits)
  while(pulseIn(pin, LOW, 500000) < 500); 

  // Measure the first LOW bit of the 0x55 Sync byte
  long bitWidth = pulseIn(pin, LOW, 10000); 
  if (bitWidth <= 0) return 0;

  long raw = 1000000 / bitWidth;
  
  // Snap to standard automotive speeds
  if (raw > 18000 && raw < 21000) return 19200;
  if (raw > 9500 && raw < 11500)  return 10417; // Common GM Speed
  return raw;
}

void processLastFrame() {
  if (bufIdx < 3) return; // Need at least Sync, PID, and Checksum

  uint8_t pid = frameBuffer[1];
  uint8_t receivedChecksum = frameBuffer[bufIdx - 1];
  uint8_t dataLen = bufIdx - 3; // Total bytes minus Sync, PID, and Checksum

  // Verify using LIN 2.x Enhanced Checksum (Includes PID)
  uint16_t sum = pid; 
  for (int i = 2; i < bufIdx - 1; i++) {
    sum += frameBuffer[i];
    if (sum > 255) sum -= 255;
  }
  uint8_t calculated = (uint8_t)(~sum);

  // Output results
  Serial.print("ID: 0x"); Serial.print(pid & 0x3F, HEX);
  Serial.print(" Data: ");
  for (int i = 2; i < bufIdx - 1; i++) {
    if(frameBuffer[i] < 0x10) Serial.print("0");
    Serial.print(frameBuffer[i], HEX); Serial.print(" ");
  }
  
  if (calculated == receivedChecksum) {
    Serial.println("[OK]");
  } else {
    Serial.print("[CHECKSUM ERROR: Calc 0x");
    Serial.print(calculated, HEX); Serial.println("]");
  }
}
