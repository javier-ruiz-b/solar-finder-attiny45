#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SoftwareServo.h>

#define PIN_SOLAR_VOLTAGE   A1
#define PIN_SERVO_X         4
#define PIN_SERVO_Y         3
#define PIN_POWER_SERVOS    0

SoftwareServo servo;  
uint8_t servoXposition;
uint8_t servoYposition;

volatile boolean f_wdt = 1;
int sleepCounter = 0;

void sleepMins(int);
void sleep8s();
void setup_watchdog_timer() {
  cli();
  wdt_reset(); // Reset Watchdog Timer
  MCUSR &= ~(1 << WDRF); //Ruecksetzen des Watchdog System Reset Flag
  WDTCR = (1 << WDCE); //Watchdog Change Enable setzen
  WDTCR = (1<<WDP3 )|(0<<WDP2 )|(0<<WDP1)|(1<<WDP0); //8s
  WDTCR |= (1 << WDIE); //Watchdog Timeout Interrupt Enable setzen
  sei();
}

void setup() { 
  setup_watchdog_timer();
  
  pinMode(PIN_POWER_SERVOS, OUTPUT);
  servoXposition = 0;
  servoYposition = 0;
} 
//
//void rangeMove(SoftwareServo &servo, uint8_t minVal, uint8_t maxVal) {
//  for(uint8_t pos = minVal; pos < maxVal; pos++) {
//      servo.write(pos);
//      delay(50);
//      SoftwareServo::refresh();
//  }
//  for(uint8_t pos = maxVal; pos > minVal; pos--) {
//      servo.write(pos);
//      delay(50);
//      SoftwareServo::refresh();
//  }  
//}

int solarVoltage() {
  return analogRead(PIN_SOLAR_VOLTAGE);
}

void moveServo(SoftwareServo &servo, uint8_t position) {
  for (uint8_t i = 0; i < 10; i++) {
    servo.write(position);
    delay(20);
    SoftwareServo::refresh();
  }
}

void moveToBestPosition(uint8_t servoPin, uint8_t currentPosition, uint8_t minVal, uint8_t maxVal, uint8_t stepVal) {
  wdt_reset();
  servo.attach(servoPin);
  moveServo(servo, minVal);
  
  for(uint8_t pos = currentPosition; pos >= minVal; pos -= stepVal) {
    moveServo(servo, pos);
  }
  
  int bestVoltage = 0;
  uint8_t bestPosition = 0;
  for(uint8_t pos = minVal; pos < maxVal; pos += stepVal) {
    moveServo(servo, pos);
    int voltage = solarVoltage();
    if (voltage > bestVoltage) {
      bestVoltage = voltage;
      bestPosition = pos;
    }
  }
  moveServo(servo, bestPosition);
}
uint8_t readServoPositionAndDetach() {
  uint8_t result = servo.read();
  servo.detach();
  return result;
}

void loop() {
  if (f_wdt==0)    // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    return;       // reset flag
  f_wdt = 0;
  
  const uint8_t minX = 0;
  const uint8_t maxX = 180;
  const uint8_t minY = 0;
  const uint8_t maxY = 90;

  const uint8_t stepValue = 2;
  digitalWrite(PIN_POWER_SERVOS, HIGH);
  delay(500);
  
  moveToBestPosition(PIN_SERVO_X, servoXposition, minX, maxX, stepValue);
  servoXposition = readServoPositionAndDetach();

  moveToBestPosition(PIN_SERVO_Y, servoYposition, minY, maxY, stepValue);
  servoYposition = readServoPositionAndDetach();

  moveToBestPosition(PIN_SERVO_X, servoXposition, max(servoXposition - 3*stepValue, 0), servoXposition + 3*stepValue, 1);
  servoXposition = readServoPositionAndDetach();
  moveToBestPosition(PIN_SERVO_Y, servoYposition, max(servoYposition - 3*stepValue, 0), servoYposition + 3*stepValue, 1);
  servoYposition = readServoPositionAndDetach();
  
  delay(1000);
  digitalWrite(PIN_POWER_SERVOS, LOW);

  sleepMins(1);
}

void sleepMins(int minutes) {
  for (int i = 0; i < (minutes*60)/8; i++) {
    sleep8s();
  }  
}

void sleep8s() {
  byte adcsra;

  adcsra = ADCSRA; //ADC Control and Status Register A sichern
  ADCSRA &= ~(1 << ADEN); //ADC ausschalten

  MCUCR |= (1 << SM1) & ~(1 << SM0); //Sleep-Modus = Power Down
  MCUCR |= (1 << SE); //Sleep Enable setzen
  sleep_cpu(); //Schlafe ....
  MCUCR &= ~(1 << SE); //Sleep Disable setzen

  ADCSRA = adcsra; //ADCSRA-Register rueckspeichern
}

ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}

//  value = map(value, 0, 800, 0, 179);
