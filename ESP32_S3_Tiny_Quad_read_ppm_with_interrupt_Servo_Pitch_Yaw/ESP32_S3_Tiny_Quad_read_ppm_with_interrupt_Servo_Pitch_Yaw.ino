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

// Tilt limits
const int maxPitchDeg = 40;   // ±40° pitch
const int maxYawDeg   = 40;   // ±40° yaw
const int midAngle    = 90;   // vertical position

volatile int ReceiverValue[NUM_CHANNELS]; // Store channel values
volatile int channelIndex = 0;            // Current channel index
volatile unsigned long lastTime = 0;      // Last pulse time

Servo pitchServoleft;  // 1) Create servo object
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
  Serial.println("ESP32 PPM + Pitch/Yaw Mix");

// ########################################################
  // ----- Initialize Servo Library for ESP32 -----
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  
  // ----- Setup Servos -----
  pitchServoleft.setPeriodHertz(50);    // Standard 50Hz servo frequency
  pitchServoright.setPeriodHertz(50);   // Standard 50Hz servo frequency
  
  // Set up PPM input
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

  // 3) Attach servo to its pin
  pitchServoleft.attach(SERVO_PIN1, 1000, 2000);
  pitchServoright.attach(SERVO_PIN2, 1000, 2000);

  pitchServoleft.write(midAngle);
  pitchServoright.write(midAngle);
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

  // 1) Compute pitch offset: –40…+40°
  int pitchOffset  = map(channelValues[1], CHANNEL_MIN, CHANNEL_MAX, -maxPitchDeg, maxPitchDeg);

  // 2) Compute yaw offset: +40…–40° (so stick-left = +yawOffset)
  int yawOffset  = map(channelValues[3], CHANNEL_MIN, CHANNEL_MAX, -maxYawDeg, maxYawDeg);

  // 3) Mix:
  int leftAngle  = midAngle + pitchOffset + yawOffset;
  int rightAngle = midAngle + pitchOffset - yawOffset;

  // 4) Clamp to safe window [50°, 130°]
  int minA = midAngle - 40; //90 - 40
  int maxA = midAngle + 40; //90 + 40 
  
  leftAngle  = constrain(leftAngle,  minA, maxA);
  rightAngle = constrain(rightAngle, minA, maxA);
  
  // leftAngle  = constrain(leftAngle,  50, 130);
  // rightAngle = constrain(rightAngle, 50, 130);

  // Write to servos
  pitchServoleft.write(leftAngle);
  pitchServoright.write(180 - rightAngle); // because our servo is mirror to each other
  
  Serial.print(" -> Left Angle: ");
  Serial.print(leftAngle);
  Serial.print(" -> Right Angle: ");
  Serial.print(rightAngle);
  Serial.print(" ");

  Serial.print("Roll [µs]: ");
  Serial.print(channelValues[0]);
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
