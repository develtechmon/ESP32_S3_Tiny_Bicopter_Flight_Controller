#include <ESP32Servo.h>      // 1) ESP32-compatible Servo library
// Pin where the PPM signal is connected
#define PPM_PIN 13           // GPIO pin for PPM input

// Servo output pin
#define SERVO_PIN1 1          // Digital pin 1 
#define SERVO_PIN2 2          // Digital pin 2 

// Number of channels
#define NUM_CHANNELS 8

// Timing constants
#define PPM_SYNC_THRESHOLD 3000 // Sync pulse threshold (µs)
#define CHANNEL_MIN 1000        // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000        // Maximum valid pulse width (µs)

volatile int ReceiverValue[NUM_CHANNELS]; // Store channel values
volatile int channelIndex = 0;            // Current channel index
volatile unsigned long lastTime = 0;      // Last pulse time

Servo pitchServoleft;  // 2) Create servo object
Servo pitchServoright;  // 2) Create servo object

void ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    // Detected sync pulse, reset channel index
    channelIndex = 0;
  } 
  else if (channelIndex < NUM_CHANNELS) {
    // Store the pulse width as channel value
    ReceiverValue[channelIndex] = constrain(pulseWidth, CHANNEL_MIN, CHANNEL_MAX);
    channelIndex++;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("PPM Reader + Servo Initialized");

  // Set up PPM input
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

  // 3) Attach servo to its pin
  pitchServoleft.attach(SERVO_PIN1);
  pitchServoright.attach(SERVO_PIN2);

}

void read_receiver(int* channelValues) {
  noInterrupts(); // Temporarily disable interrupts
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts(); // Re-enable interrupts
}

void loop() {
  int channelValues[NUM_CHANNELS];
  read_receiver(channelValues);

  // ################## Pitch #####################
  // Map 1000–2000 µs to 0–180 degrees
  // int pitchAngle = map(channelValues[1], CHANNEL_MIN, CHANNEL_MAX, 0, 180);
  
  // Map 1000–2000 µs to move between -40 to 40  degrees
  int pitchAngle = map(channelValues[1], CHANNEL_MIN, CHANNEL_MAX, 90-40, 90+40);

  // Constrain just in case beween 0-180 degress
  // pitchAngle = constrain(pitchAngle, 0, 180);

  // Constrain just in case beween 50-130 degress
  pitchAngle = constrain(pitchAngle, 50, 130);
  
  pitchServoleft.write(pitchAngle);
  pitchServoright.write(180 - pitchAngle);

  // Debug output
  Serial.print("Roll [µs]: ");
  Serial.print(channelValues[0]);
  Serial.print(" -> Angle: ");
  Serial.print(pitchAngle);
  Serial.print(" | Pitch [µs]: ");
  Serial.print(channelValues[1]);
  Serial.print(" | Throttle [µs]: ");
  Serial.print(channelValues[2]);
  Serial.print(" | Yaw [µs]: ");
  Serial.print(channelValues[3]);
  Serial.print(" | SWA [µs]: ");
  Serial.print(channelValues[4]);
  Serial.print(" | SWD [µs]: ");
  Serial.print(channelValues[5]);
  Serial.print(" | SWC [µs]: ");
  Serial.print(channelValues[6]);
  Serial.print(" | SWB [µs]: ");
  Serial.println(channelValues[7]);

  delay(50);
}
