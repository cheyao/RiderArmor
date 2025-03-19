#include "Grove_Motor_Driver_TB6612FNG.h"
#include "Ultrasonic.h"
#include <SoftwareSerial.h> //Software Serial Port
#include <Wire.h>

// Common ports
constexpr const static inline unsigned int RxD 8;
constexpr const static inline unsigned int TxD 9;
constexpr const static inline unsigned int ULTRASONIC_PIN 5;
constexpr const static inline unsigned int PIN 6;
constexpr const static inline unsigned int NUMPIXELS 20;

// Initialize bit-banging serial device
SoftwareSerial blueToothSerial(RxD, TxD);

// The main instance of the ultrasonic sensor
static Ultrasonic ultrasonic(ULTRASONIC_PIN);

static ICM20600 icm20600(true);
static Adafruit_NeoPixel pixels =
    Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// And the instance for the motor controller
static MotorDriver motor;

constexpr const uint16_t rpm = 120;

// Function to setup bluetooth
void setupBlueToothConnection() {
  // Start up our serial device
  blueToothSerial.begin(9600);
  blueToothSerial.print("AT");
  delay(400);

  // Restore all setup value to factory setup
  blueToothSerial.print("AT+DEFAULT");
  delay(2000);

  // Configure bluetooth name and password
  blueToothSerial.print("AT+NAMERiderArmor");
  delay(400);
  blueToothSerial.print("AT+PIN1234");
  delay(400);
  blueToothSerial.print("AT+AUTH1");
  delay(400);
  blueToothSerial.flush();
}

void setup() {
  // Initalize all libs
  Wire.begin();
  Serial.begin(9600); // Initialize the serial baud rate to 9600
  motor.init();

  Serial.begin(9600);
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);

  icm20600.initialize();
  icm20600.setAccAverageSample(ACC_AVERAGE_8);
  icm20600.setAccScaleRange(RANGE_2G);

  pixels.setBrightness(0);
  pixels.begin();
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(
        i, pixels.Color(255, 0, 0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
  }

  setupBlueToothConnection();
}

enum class { NONE, RIGHT, LEFT } dirState;

bool on = false;
void loop() {
  // Get the distace from the sensor
  const auto distance = ultrasonic.MeasureInCentimeters();
  const auto accel = icm20600.getAccelerationX();

  // Output the distance to the serial log
  Serial.print(distance);
  Serial.println(" cm");

  Serial.println(accel);

  if (accel > -700) {
    if (on) {
      pixels.setBrightness(0);
      pixels.show();
      on = false;
    }
  } else if (!on) {
    pixels.setBrightness(255);
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(
          i, pixels.Color(255, 0, 0)); // Moderately bright green color.
      pixels.show(); // This sends the updated pixel color to the hardware.
    }
    delay(1000);
    on = true;
  }

  if (blueToothSerial.available()) {
    // Make sure we clear the bt buffer
    char recu;
    while (blueToothSerial.available()) {
      recu = blueToothSerial.read();
    }
    Serial.println(recu);

    // We get the motor to run a whole tour
    // 1024 Steps is 2pi radd
    if (recu == 'g') {
      motor.stepperRun(FULL_STEP, 1024 / 4, rpm);
      dirState = LEFT;

      delay(1500);
    } else if (recu == 'd') {
      motor.stepperRun(FULL_STEP, -1024 / 4, rpm);
      dirState = RIGHT;

      delay(1500);
    }
  }

  blueToothSerial.print(distance);
  blueToothSerial.print(";");

  blueToothSerial.flush();
  delay(200);
}
