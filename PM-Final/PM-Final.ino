#include <LiquidCrystal_I2C.h>
#include <AFMotor.h>

static LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns and 2 rows
static AF_DCMotor leftMotor(2, MOTOR12_8KHZ); // The left motor of the robot, connected on pin 1
static AF_DCMotor rightMotor(1, MOTOR12_8KHZ); // The right motor of the robot, connected on pin 4

#define MOTOR1_PWM 11
#define MOTOR2_PWM 3

static long timeStampBattery = 0; // The timer value for the battery refresh
static long timeStampSensors = 0; // The timer value for the sensors refresh

/*
  - sensorValues[0] -> left sensor
  - sensorValues[1] -> middle sensor
  - sensorValues[2] -> right sensor
*/
static uint16_t sensorValues[3];

/* The function that sets the ADC on a specific channel given by the parameters */
static void setupADC(int channel0, int channel1, int channel2, int channel3)
{
  ADMUX = 0;

  /* AVCC with external capacitor at AREF pin - Vref = 5V*/
  ADMUX |= (1 << REFS0);

  // Setting ADC channel
  if (channel0 != 0) {
    ADMUX |= (1 << MUX0);
  }

  if (channel1 != 0) {
    ADMUX |= (1 << MUX1);
  }
  
  if (channel2 != 0) {
    ADMUX |= (1 << MUX2);
  }

  if (channel3 != 0) {
    ADMUX |= (1 << MUX3);
  }

  ADCSRA = 0;

  // Enable ADC
  ADCSRA |= (1 << ADEN);

  // Set prescaler to 128
  ADCSRA |= (7 << ADPS0);
}

/* The function that starts a conversion of the ADC that was configured by the function above */
static uint16_t readADCValue()
{
  // start conversion
  ADCSRA |= (1 << ADSC);

  // Wait until the conversion is complete
  while(! (ADCSRA & (1 << ADIF)));

  return ADC;
}

/* The function that prints the text on the LCD */
static void printBatteryLCD()
{
  lcd.clear();

  lcd.home();
  lcd.print(" Groot's Battery");

  // ADC = Vin * 1023 / Vref => Vin = ADC * Vref / 1023 si Vref = 5V deci Vin = ADC * 5 / 1023
  // (dar, Vin vine din divizorul rezistiv facut cu 2 rezistente de 10k)
  // iar Vout = Vin * R / 2R = Vin / 2 deci V-baterie este de fapt 2 * ADC * 5 / 1023
  double result = readADCValue() * 5.0 / 1023.0 * 2.0;
  lcd.setCursor(0, 1);

  lcd.print("     ");
  lcd.print(result);
  lcd.print(" V");
}

static void initializeLCD()
{
  lcd.init(); // initialize the lcd
  lcd.backlight(); // turn on the backlight of the LCD
}

/* The function that reads the values from the sensors. It does the ADC setup for each of the sensors, reads the
value that the sensor transmits and then puts LOW/HIGH at the sensor's specific position in the vector. At the end
of the function, it calculates the current state of the robot and returns it */
static double readSensors()
{
  setupADC(0, 0, 0, 0);
  uint16_t leftSensor = readADCValue();

  if (leftSensor <= 512) {
    sensorValues[0] = HIGH;
  } else {
    sensorValues[0] = LOW;
  }
  
  setupADC(1, 0, 0, 0);
  uint16_t middleSensor = readADCValue();

  if (middleSensor <= 512) {
    sensorValues[1] = HIGH;
  } else {
    sensorValues[1] = LOW;
  }

  setupADC(0, 1, 0, 0);
  uint16_t rightSensor = readADCValue();

  if (rightSensor <= 512) {
    sensorValues[2] = HIGH;
  } else {
    sensorValues[2] = LOW;
  }

  if ((sensorValues[2] + sensorValues[1] + sensorValues[0]) == 0) {
    return -1;
  }

  return (0 * sensorValues[2] + 1000 * sensorValues[1] + 2000 * sensorValues[0]) / (sensorValues[2] + sensorValues[1] + sensorValues[0]);
}

/* The function that sets the left motors and the right motors of the robot to run at the
given speed using run and setSpeed library methods */
int setMotors(int leftSpeed, int rightSpeed) { // Motor setup
  if (leftSpeed > 0 && rightSpeed > 0)  {
    leftMotor.run(FORWARD);
    rightMotor.run(FORWARD);

    analogWrite(MOTOR1_PWM, leftSpeed);
    analogWrite(MOTOR2_PWM, rightSpeed);
  } else if (leftSpeed < 0 && rightSpeed > 0) {
    leftMotor.run(BACKWARD);
    rightMotor.run(FORWARD);

    analogWrite(MOTOR1_PWM, -leftSpeed);
    analogWrite(MOTOR2_PWM, rightSpeed);
  } else if (leftSpeed > 0 && rightSpeed < 0) {
    leftMotor.run(FORWARD);
    rightMotor.run(BACKWARD);

    analogWrite(MOTOR1_PWM, leftSpeed);
    analogWrite(MOTOR2_PWM, -rightSpeed);

  } else if (leftSpeed == 0 && rightSpeed == 0) {
    leftMotor.run(RELEASE);
    rightMotor.run(RELEASE);

    analogWrite(MOTOR1_PWM, 0);
    analogWrite(MOTOR2_PWM, 0);
  }
}

/* The function that calculates the PID algorithm. Kp, Ki and Kd were determined in
a experimental way, and set to some values with which the robot runs ok. There might
be other values better for this robot */
void PIDAlgorithm() {
  int straightValue = 1000;
  double error, lastError = 0;

  int power_difference = 0;
  float Kp = 0.1, Ki = pow(50, -8), Kd = 5;

  double initialSpeed = 150;
  double maxExtraSpeed = 100;
  double power;

  int derivative = 0, proportional = 0, integral = 0;

  long loopTime = millis();
  while(1) {
    double currentValue = readSensors();
    if (currentValue == -1) {
      setMotors(0, 0);
      integral = 0;
      break;
    }

    error = straightValue - currentValue;

    proportional = Kp * error;
    derivative = Kd * (error - lastError) / loopTime;
    integral = integral + (loopTime * error * Ki);

    power = proportional + derivative + integral;

    if(power > maxExtraSpeed) {
      power = maxExtraSpeed;
    } else if (power < -maxExtraSpeed) {
      power = -maxExtraSpeed;
    }

    if (power > 0) {
      setMotors(initialSpeed - power, initialSpeed + power);
    } else if (power < 0) {
      setMotors(initialSpeed - power, initialSpeed + power);
    } else {
      setMotors(initialSpeed, initialSpeed);
    }
    

    loopTime = millis() - loopTime;
    lastError = error;
  }
}

void setup()
{
  initializeLCD();

  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
}

void loop()
{
  if ((millis() - timeStampBattery) >= 10000) {
    timeStampBattery = millis();
    setupADC(1, 1, 0, 0);
    printBatteryLCD();
  }

  if ((millis() - timeStampSensors) >= 1000) {
    timeStampSensors = millis();
    
    PIDAlgorithm();
  }
}
