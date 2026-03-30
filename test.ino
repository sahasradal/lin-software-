#include <SoftwareSerial.h>   // Connecting the softserialport library
SoftwareSerial LINBusSerial(2, 3);   // Assign pins (we don't use others)

int k;  // Variable for working with PC line count
byte b, y;  // Variables
volatile uint32_t frame;  //variable for frame size working with microcontroller interrupts
void setup() {
  attachInterrupt(0, LinFrame, FALLING); // FALLING triggered when the signal on the pin changes from HIGH to LOW (interrupts)
  delay(3000);  // 3 second delay
  Serial.begin(19200);   // Set the speed on the serial port to work with the PC. You need to set the same speed in the serial port monitor.
  Serial.println(">>>> WAIT SIGNAL >>>>");
  LINBusSerial.begin(10417);  // We set the speed for working with the soft serial for the lin bus; you can change it: 2400, 9600, 19200 are the main speeds
}
void loop() {
  if (LINBusSerial.available()) {  // wait for signal
    b = LINBusSerial.read();    // We write the value into a variable
    Serial.print(b, HEX);    // Display on PC
    Serial.print(" ");   // Space for PC
  }
  }
void LinFrame() {   // working with microcontroller interrupts
  if (millis() - frame >= 150 && digitalRead(2)) {   // When there is a signal on pin 2, we start counting 150 (it can be changed from approximately 50 to 250, depending on the bus line speed) milliseconds...
    frame = millis();    //           HHHHHHHHHHHHHHHHH                 HHHHHHHHHHHHHHHHHHHH               HHHHHHHHHHHHHHHHHHHH          
    Serial.println();   //translate the line in PC
    y++;  // We increase the variable y by one to work with display lines.
    k++;  // We increase the variable k by one to work with the PC line count
    Serial.print(k);  // display the value of the variable k on the PC
    Serial.print(" "); // display space on PC
    Serial.print(y);  //  display the value of y on the PC to understand the display lines
    Serial.print(" ");  // display space on PC
  }}
