#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

// Define the pin for the built-in LED
#define LED_PIN 48
#define NUM_LEDS 1

// PPM Definitions
#define PPM_PIN 13
#define NUM_CHANNELS 8
#define PPM_SYNC_THRESHOLD 3000
#define CHANNEL_MIN 1000
#define CHANNEL_MAX 2000

// Custom I2C pins
#define CUSTOM_SDA_PIN 3
#define CUSTOM_SCL_PIN 4

// Flight Controller Declarations
uint32_t LoopTimer;
volatile float MotorInput1, MotorInput2; // Only 2 motors for bicopter
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;

// PPM Variables
volatile int ReceiverValue[NUM_CHANNELS] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500};
volatile int channelIndex = 0;
volatile unsigned long lastTime = 0;
int channelValues[NUM_CHANNELS];

// PID Gains
float PAngleRoll = 2, PAnglePitch = 2;
float IAngleRoll = 0.5, IAnglePitch = 0.5;
float DAngleRoll = 0.007, DAnglePitch = 0.007;
float PRateRoll = 0.625, PRatePitch = 0.625;
float IRateRoll = 2.1, IRatePitch = 2.1;
float DRateRoll = 0.0088, DRatePitch = DRateRoll;
float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

// Throttle Limits
int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;
int led_time = 500;

// PID Terms
volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// Kalman Filters
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = {0, 0};

// Battery Parameters
float Voltage;

// PWM Configuration
const int pwmFrequency = 50;    // 50Hz for ESCs and servos
const int pwmResolution = 8;    // 8-bit resolution

// Motor and Servo Pins and Channels
const int motor1Pin = 25;       // Left motor
const int motor2Pin = 26;       // Right motor
const int servo1Pin = 27;       // Servo 1
const int servo2Pin = 14;       // Servo 2
const int motor1Channel = 11;
const int motor2Channel = 10;
const int servo1Channel = 12;
const int servo2Channel = 13;

// Time Step (seconds)
const float t = 0.004;

// NeoPixel
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Arming/Disarming Variables
bool armed = false;
unsigned long armDisarmTimer = 0;
const unsigned long armHoldTime = 1000;

// Servo Scaling Factors
float Kp = 1.0; // Pitch scaling
float Ky = 1.0; // Yaw scaling

// PPM Interrupt Handler
void IRAM_ATTR ppmInterruptHandler() {
    unsigned long currentTime = micros();
    unsigned long pulseWidth = currentTime - lastTime;
    lastTime = currentTime;
    if (pulseWidth > PPM_SYNC_THRESHOLD) {
        channelIndex = 0;
    } else if (channelIndex < NUM_CHANNELS) {
        if (pulseWidth < CHANNEL_MIN) pulseWidth = CHANNEL_MIN;
        else if (pulseWidth > CHANNEL_MAX) pulseWidth = CHANNEL_MAX;
        ReceiverValue[channelIndex] = pulseWidth;
        channelIndex++;
    }
}

// Read Battery Voltage
void battery_voltage(void) {
    Voltage = (float)analogRead(1) / 237; // GPIO1
}

// Safely Copy Receiver Values
void read_receiver(int *channelValues) {
    noInterrupts();
    for (int i = 0; i < NUM_CHANNELS; i++) {
        channelValues[i] = ReceiverValue[i];
    }
    interrupts();
}

// 1D Kalman Filter
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState = KalmanState + (t * KalmanInput);
    KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4);
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

// Convert Pulse Width to Duty Cycle for 8-bit resolution
uint8_t pulseToDuty(float pulse) {
    // Map 1000-2000us to 0-255
    return (uint8_t)map(pulse, 1000, 2000, 0, 255);
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    strip.begin();
    strip.show();

    // Startup LED Flash
    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(led_time);
        digitalWrite(LED_BUILTIN, LOW);
        delay(led_time);
    }

    // Setup PPM Receiver
    pinMode(PPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);
    delay(100);

    // Setup MPU6050 (I2C)
    Wire.setClock(400000);
    Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Setup PWM for Motors and Servos
    ledcSetup(motor1Channel, pwmFrequency, pwmResolution);
    ledcAttachPin(motor1Pin, motor1Channel);
    ledcSetup(motor2Channel, pwmFrequency, pwmResolution);
    ledcAttachPin(motor2Pin, motor2Channel);
    ledcSetup(servo1Channel, pwmFrequency, pwmResolution);
    ledcAttachPin(servo1Pin, servo1Channel);
    ledcSetup(servo2Channel, pwmFrequency, pwmResolution);
    ledcAttachPin(servo2Pin, servo2Channel);

    // Initialize Outputs
    ledcWrite(motor1Channel, pulseToDuty(1000));
    ledcWrite(motor2Channel, pulseToDuty(1000));
    ledcWrite(servo1Channel, pulseToDuty(1500));
    ledcWrite(servo2Channel, pulseToDuty(1500));

    // Calibration Values for 8520 Motor
    RateCalibrationRoll = 2.80;
    RateCalibrationPitch = -1.86;
    RateCalibrationYaw = -1.47;
    AccXCalibration = 0.04;
    AccYCalibration = 0.01;
    AccZCalibration = 0.09;

    // Startup Indication
    strip.setPixelColor(0, strip.Color(0, 255, 0));
    strip.show();
    delay(1000);
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.show();

    LoopTimer = micros();
}

void loop() {
    read_receiver(channelValues);

    // Arming/Disarming Logic
    if (channelValues[2] < 1050) {
        if (!armed && channelValues[3] > 1900) {
            if (armDisarmTimer == 0) {
                armDisarmTimer = millis();
            } else if (millis() - armDisarmTimer > armHoldTime) {
                armed = true;
                strip.setPixelColor(0, strip.Color(255, 255, 0));
                strip.show();
                delay(1000);
                strip.setPixelColor(0, strip.Color(0, 0, 0));
                strip.show();
                armDisarmTimer = 0;
            }
        } else if (armed && channelValues[3] < 1100) {
            if (armDisarmTimer == 0) {
                armDisarmTimer = millis();
            } else if (millis() - armDisarmTimer > armHoldTime) {
                armed = false;
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

    if (!armed) {
        MotorInput1 = MotorInput2 = ThrottleCutOff;
        float servo1 = 1500;
        float servo2 = 1500;
        PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
        PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
        PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
        PrevItermAngleRoll = PrevItermAnglePitch = 0;

        // Write Outputs
        ledcWrite(motor1Channel, pulseToDuty(MotorInput1));
        ledcWrite(motor2Channel, pulseToDuty(MotorInput2));
        ledcWrite(servo1Channel, pulseToDuty(servo1));
        ledcWrite(servo2Channel, pulseToDuty(servo2));

        while (micros() - LoopTimer < (t * 1000000));
        LoopTimer = micros();
        return;
    }

    // Read MPU6050 Data
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

    RateRoll = (float)GyroX / 65.5 - RateCalibrationRoll;
    RatePitch = (float)GyroY / 65.5 - RateCalibrationPitch;
    RateYaw = (float)GyroZ / 65.5 - RateCalibrationYaw;
    AccX = (float)AccXLSB / 4096 - AccXCalibration;
    AccY = (float)AccYLSB / 4096 - AccYCalibration;
    AccZ = (float)AccZLSB / 4096 - AccZCalibration;

    // Calculate Angles
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

    // Kalman Filters
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    KalmanAngleRoll = constrain(KalmanAngleRoll, -20, 20);
    KalmanAnglePitch = constrain(KalmanAnglePitch, -20, 20);

    // Set Desired Inputs
    DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
    DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
    InputThrottle = ReceiverValue[2];
    DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

    // Angle PID for Roll
    ErrorAngleRoll = DesiredAngleRoll - KalmanAngleRoll;
    PtermRoll = PAngleRoll * ErrorAngleRoll;
    ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
    ItermRoll = constrain(ItermRoll, -400, 400);
    DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
    PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
    DesiredRateRoll = PIDOutputRoll;
    PrevErrorAngleRoll = ErrorAngleRoll;
    PrevItermAngleRoll = ItermRoll;

    // Angle PID for Pitch
    ErrorAnglePitch = DesiredAnglePitch - KalmanAnglePitch;
    PtermPitch = PAnglePitch * ErrorAnglePitch;
    ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
    ItermPitch = constrain(ItermPitch, -400, 400);
    DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
    PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
    DesiredRatePitch = PIDOutputPitch;
    PrevErrorAnglePitch = ErrorAnglePitch;
    PrevItermAnglePitch = ItermPitch;

    // Rate PID Calculations
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

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

    if (InputThrottle > 2000) InputThrottle = 2000;

    // Motor Mixing for Bicopter
    MotorInput1 = InputThrottle + InputRoll;  // Left motor
    MotorInput2 = InputThrottle - InputRoll;  // Right motor
    MotorInput1 = constrain(MotorInput1, ThrottleIdle, 2000);
    MotorInput2 = constrain(MotorInput2, ThrottleIdle, 2000);

    // Servo Mixing
    float servo1 = 1500 + Kp * InputPitch + Ky * InputYaw;  // Servo 1
    float servo2 = 1500 + Kp * InputPitch - Ky * InputYaw;  // Servo 2 (mirrored for yaw)
    servo1 = constrain(servo1, 1000, 2000);
    servo2 = constrain(servo2, 1000, 2000);

    // Throttle Safety Check
    if (ReceiverValue[2] < 1030) {
        MotorInput1 = MotorInput2 = ThrottleCutOff;
        servo1 = 1500;
        servo2 = 1500;
        PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
        PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
        PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
        PrevItermAngleRoll = PrevItermAnglePitch = 0;
    }

    // Write Outputs
    ledcWrite(motor1Channel, pulseToDuty(MotorInput1));
    ledcWrite(motor2Channel, pulseToDuty(MotorInput2));
    ledcWrite(servo1Channel, pulseToDuty(servo1));
    ledcWrite(servo2Channel, pulseToDuty(servo2));

    while (micros() - LoopTimer < (t * 1000000));
    LoopTimer = micros();
}
