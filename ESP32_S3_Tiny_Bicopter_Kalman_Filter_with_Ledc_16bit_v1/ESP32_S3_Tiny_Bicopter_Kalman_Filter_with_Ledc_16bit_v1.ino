#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

// ==== Define the pin for the built-in LED. Change this if your board uses a different pin ====
#define LED_PIN 48  
#define NUM_LEDS 1

// ===== PPM Definitions =====
#define PPM_PIN 13             // Pin where the PPM signal is connected
#define NUM_CHANNELS 8         // Total number of channels in the PPM stream
#define PPM_SYNC_THRESHOLD 3000 // Pulse width (µs) above which a sync pulse is assumed
#define CHANNEL_MIN 1000       // Minimum valid pulse width (µs)
#define CHANNEL_MAX 2000       // Maximum valid pulse width (µs)

// Define your custom I2C pins (change these as needed)
#define CUSTOM_SDA_PIN 3   // Example: GPIO3
#define CUSTOM_SCL_PIN 4   // Example: GPIO4

// ===== Motor and Servo Pins for Bicopter =====
#define MOTOR_LEFT_PIN 5   // Left motor pin (adjust as needed)
#define MOTOR_RIGHT_PIN 6  // Right motor pin (adjust as needed)
#define SERVO_LEFT_PIN 7   // Left servo pin (adjust as needed)
#define SERVO_RIGHT_PIN 8  // Right servo pin (adjust as needed)

// ===== Flight Controller / PID Declarations =====
uint32_t LoopTimer;
volatile float MotorInputLeft, MotorInputRight;  // Only 2 motors for bicopter
volatile float ServoLeftPosition, ServoRightPosition;  // Servo positions
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;

// Global variables for PPM
volatile int ReceiverValue[NUM_CHANNELS] = {1500,1500,1000,1500,1500,1500,1500,1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;
int channelValues[NUM_CHANNELS];

// PID values (adjusted for bicopter dynamics)
float PAngleRoll = 2.5, PAnglePitch = 2.5;
float IAngleRoll = 0.6, IAnglePitch = 0.6;
float DAngleRoll = 0.009, DAnglePitch = 0.009;

float PRateRoll = 0.7, PRatePitch = 0.7;
float IRateRoll = 2.3, IRatePitch = 2.3;
float DRateRoll = 0.01, DRatePitch = DRateRoll;

float PRateYaw = 4.5;
float IRateYaw = 3.5;
float DRateYaw = 0.02;

// Throttle limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;
int led_time = 500;

// Servo limits
const int SERVO_MIN = 1000;    // 1ms pulse
const int SERVO_MAX = 2000;    // 2ms pulse
const int SERVO_MID = 1500;    // 1.5ms pulse (center position)

// LEDC (PWM) configuration
const int motorPwmFreq = 20000;  // 20 kHz PWM frequency for motors
const int servoPwmFreq = 50;     // 50 Hz (standard servo frequency)
const int motorResolution = 8;   // 8-bit resolution: values from 0 to 255
const int servoResolution = 16;  // 16-bit resolution for more precise servo control

// LEDC channel assignments
const int motorLeftChannel = 0;
const int motorRightChannel = 1;
const int servoLeftChannel = 2;
const int servoRightChannel = 3;

// PID variables
volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

// Kalman filters for angle estimation
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// Battery Parameters
float Voltage;

// Time step (seconds)
const float t = 0.004; 

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ===== Arming/Disarming Variables =====
bool armed = false;             // Drone armed state
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000; // milliseconds required to hold stick

// ===== Helper Function to Convert Microseconds to Servo Duty Cycle =====
int microsToDuty(int microseconds) {
  return map(microseconds, SERVO_MIN, SERVO_MAX, 3277, 6554); // For 50Hz signal with 16-bit resolution
}

// ===== PPM Interrupt Handler =====
// Use IRAM_ATTR for faster interrupt handling on the ESP32.
void IRAM_ATTR ppmInterruptHandler() {
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseWidth > PPM_SYNC_THRESHOLD) {
    // A long pulse is assumed to be the sync pulse – reset channel index.
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    // Store a valid channel pulse (constrained to valid range)
    if (pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
    else if (pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
    ReceiverValue[channelIndex] = pulseWidth;
    channelIndex++;
  }
}

// ==== Read Battery Voltage ====
void battery_voltage(void) {
  Voltage = (float)analogRead(1) / 237; // GPIO1
}

// ===== Helper Function to Safely Copy Receiver Values =====
void read_receiver(int *channelValues) {
  noInterrupts();
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = ReceiverValue[i];
  }
  interrupts();
}

// ===== Simple 1D Kalman Filter =====
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4); // IMU variance (4 deg/s)
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3); // error variance (3 deg)
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState; 
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  strip.begin();       // Initialize the NeoPixel strip
  strip.show();        // Turn all pixels off as an initial state

  // Flash built-in LED a few times for startup indication
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(led_time);
    digitalWrite(LED_BUILTIN, LOW);
    delay(led_time);
  }
  
  // ----- Setup PPM Receiver -----
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
  delay(100);

  // ----- Setup MPU6050 (I2C) -----
  Wire.setClock(400000);
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ----- Setup PWM for Motors using LEDC -----
  ledcSetup(motorLeftChannel, motorPwmFreq, motorResolution);
  ledcSetup(motorRightChannel, motorPwmFreq, motorResolution);
  ledcAttachPin(MOTOR_LEFT_PIN, motorLeftChannel);
  ledcAttachPin(MOTOR_RIGHT_PIN, motorRightChannel);

  // Initialize motor outputs to minimum throttle
  ledcWrite(motorLeftChannel, 0);
  ledcWrite(motorRightChannel, 0);

  // ----- Setup PWM for Servos using LEDC -----
  ledcSetup(servoLeftChannel, servoPwmFreq, servoResolution);
  ledcSetup(servoRightChannel, servoPwmFreq, servoResolution);
  ledcAttachPin(SERVO_LEFT_PIN, servoLeftChannel);
  ledcAttachPin(SERVO_RIGHT_PIN, servoRightChannel);
  
  // Center the servos at startup
  ledcWrite(servoLeftChannel, microsToDuty(SERVO_MID));
  ledcWrite(servoRightChannel, microsToDuty(SERVO_MID));

  // ----- Calibration Values -----
  // ----- 8520 Motor -----
  RateCalibrationRoll=-1.50;
  RateCalibrationPitch=-1.14;
  RateCalibrationYaw=0.48;
  AccXCalibration=0.06;
  AccYCalibration=-0.04;
  AccZCalibration=-0.00;

  // Green LED to indicate normal startup
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(1000);
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();

  LoopTimer = micros();
}

void loop() {
  read_receiver(channelValues);

  // ----- Arming/Disarming Logic -----
  // Check if throttle (channel 2) is low enough to allow arming/disarming
  if (channelValues[2] < 1050) {
    // To arm: yaw (channel 3) high
    if (!armed && channelValues[3] > 1900) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = true;
        // Yellow LED (Red + Green) to indicate arming
        strip.setPixelColor(0, strip.Color(255, 255, 0));
        strip.show();
        delay(1000);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        armDisarmTimer = 0;
      }
    }
    // To disarm: yaw (channel 3) low
    else if (armed && channelValues[3] < 1100) {
      if (armDisarmTimer == 0) {
        armDisarmTimer = millis();
      } else if (millis() - armDisarmTimer > armHoldTime) {
        armed = false;
        // White LED (all colors) to indicate disarming
        strip.setPixelColor(0, strip.Color(255, 255, 255));
        strip.show();
        delay(1000);
        strip.setPixelColor(0, strip.Color(0, 0, 0));
        strip.show();
        armDisarmTimer = 0;
      }
    } else {
      armDisarmTimer = 0;
    }
  } else {
    armDisarmTimer = 0;
  }
  
  // If not armed, immediately cut off motor output and reset PID integrals
  if (!armed) {
    MotorInputLeft = MotorInputRight = ThrottleCutOff;
    ServoLeftPosition = ServoRightPosition = SERVO_MID;  // Center servos when disarmed
    
    // Reset all PID integrals
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
    
    // Apply motor outputs (motors off)
    int pwmLeft = map(MotorInputLeft, 1000, 2000, 0, 255);
    int pwmRight = map(MotorInputRight, 1000, 2000, 0, 255);
    pwmLeft = constrain(pwmLeft, 0, 255);
    pwmRight = constrain(pwmRight, 0, 255);
    ledcWrite(motorLeftChannel, pwmLeft);
    ledcWrite(motorRightChannel, pwmRight);
    
    // Apply servo positions
    ledcWrite(servoLeftChannel, microsToDuty(ServoLeftPosition));
    ledcWrite(servoRightChannel, microsToDuty(ServoRightPosition));
    
    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
    return; // Skip the rest of the control loop if disarmed
  }

  // ----- Read MPU6050 Data -----
  // Request accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  // Request gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x08);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;
  AccX      = (float)AccXLSB / 4096;
  AccY      = (float)AccYLSB / 4096;
  AccZ      = (float)AccZLSB / 4096;

  // Apply calibration offsets
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;
  AccX      -= AccXCalibration;
  AccY      -= AccYCalibration;
  AccZ      -= AccZCalibration;

  // ----- Calculate Angles from Accelerometer -----
  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
  
  // Run simple Kalman filters for both angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Clamp the filtered angles to ±20 degrees
  KalmanAngleRoll = (KalmanAngleRoll > 20) ? 20 : ((KalmanAngleRoll < -20) ? -20 : KalmanAngleRoll);
  KalmanAnglePitch = (KalmanAnglePitch > 20) ? 20 : ((KalmanAnglePitch < -20) ? -20 : KalmanAnglePitch);

  // ----- Set Desired Angles and Throttle from Receiver Inputs -----
  DesiredAngleRoll  = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle     = ReceiverValue[2];
  DesiredRateYaw    = 0.15 * (ReceiverValue[3] - 1500);

  // --- Angle PID for Roll ---
  ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // --- Angle PID for Pitch ---
  ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // ----- Rate PID Calculations -----
  ErrorRateRoll  = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw   = DesiredRateYaw - RateYaw;

  // Roll Rate PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Rate PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Rate PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = constrain(ItermYaw, -400, 400);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = constrain(PtermYaw + ItermYaw + DtermYaw, -400, 400);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  if (InputThrottle > 2000) { 
    InputThrottle = 2000;
  }

  // ----- Bicopter Control Mixing -----
  // For bicopter: 
  // - Both motors provide throttle
  // - Differential thrust controls roll
  // - Servos control pitch and yaw
  
  // 1. Motor mixing for throttle and roll
  MotorInputLeft = InputThrottle - InputRoll;
  MotorInputRight = InputThrottle + InputRoll;

  // Clamp motor outputs to safe range
  MotorInputLeft = constrain(MotorInputLeft, ThrottleIdle, 2000);
  MotorInputRight = constrain(MotorInputRight, ThrottleIdle, 2000);

  // 2. Servo mixing for pitch and yaw
  // Servos work in mirrored configuration:
  // - For pitch: both servos move in the same direction
  // - For yaw: servos move in opposite directions
  ServoLeftPosition = SERVO_MID + InputPitch - InputYaw;
  ServoRightPosition = SERVO_MID + InputPitch + InputYaw;
  
  // Constrain servo positions
  ServoLeftPosition = constrain(ServoLeftPosition, SERVO_MIN, SERVO_MAX);
  ServoRightPosition = constrain(ServoRightPosition, SERVO_MIN, SERVO_MAX);

  // If throttle is too low, reset PID integrals and cut motors
  if (ReceiverValue[2] < 1030) {
    MotorInputLeft = MotorInputRight = ThrottleCutOff;
    ServoLeftPosition = ServoRightPosition = SERVO_MID;  // Center servos
    
    // Reset all PID integrals
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
  }

  // --- Convert Motor Input (µs) to PWM value (0-255) and update outputs ---
  int pwmLeft = map(MotorInputLeft, 1000, 2000, 0, 255);
  int pwmRight = map(MotorInputRight, 1000, 2000, 0, 255);
  
  pwmLeft = constrain(pwmLeft, 0, 255);
  pwmRight = constrain(pwmRight, 0, 255);

  // Apply motor outputs
  ledcWrite(motorLeftChannel, pwmLeft);
  ledcWrite(motorRightChannel, pwmRight);
  
  // Apply servo positions using LEDC
  ledcWrite(servoLeftChannel, microsToDuty(ServoLeftPosition));
  ledcWrite(servoRightChannel, microsToDuty(ServoRightPosition));

  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
