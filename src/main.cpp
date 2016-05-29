#include <Arduino.h>

#define MIN_OFFSET 0.2
#define MAX_OFFSET 4.7
#define ADC_RESOLUTION 1023
#define ADC_VOLTAGE 5
#define MAX_PRESSURE 100      // 100 kpa = max; min = 0 so ignore

// http://anver.com/company/reference-guides/vacuum/
/* note - 25" mercury in vaccum is about the max the venturi will pull
*  therefore set this as the max.  This is equal to 84.5 kpa
*/

/* sensor:  pin 1 - signal; 2 = gnd; 3 = 5v */

int ANALOG_PIN = A0;  //A0
int PUMP_PIN = 2;     //D2
int START_PIN = 6;
int STOP_PIN = 7;

int RUNNING = 1;
float AV_PRESSURE = 0;
float LAST_PRESSURE = 0;
float PRESSURE_TOLERANCE = 0.02;
//float TARGET_PRESSURE = 84.5;
float TARGET_PRESSURE = 70;

unsigned long MAX_RUN_TIME = 3600;  // max run for an hour

float calc_pressure(int voltage) {
  float temp = (float)voltage *  ((float)ADC_VOLTAGE / (float)ADC_RESOLUTION);
  temp = (temp - MIN_OFFSET) / (MAX_OFFSET - MIN_OFFSET);
  temp = temp * MAX_PRESSURE;
  return temp;
}

// determine if the vacuum pump should be on
bool vacuum_on(float pressure) {
  float n = 10;           // number of samples to average
  bool ret = 0;           // pump off by default

  float last_average = AV_PRESSURE;
  AV_PRESSURE = ((AV_PRESSURE * n) - LAST_PRESSURE + pressure) / n; //update av

  // check if we are filling - i.e. av < current
  if (last_average < pressure) {
    Serial.print(" >>Filling<< ");
    if (pressure < TARGET_PRESSURE) {   // we need to carry on sucking
      ret = 1;
    }
  }
  else {  // we aren't filling so allow a tollerance
    Serial.print(" >>Emptying<< ");
    if (pressure < (TARGET_PRESSURE * (1-PRESSURE_TOLERANCE))){
      ret = 1;
    }
  }

  Serial.print("Target:" + String(TARGET_PRESSURE, 1) + " Actual:" + String(pressure, 1));
  return ret;
}

void check_status(){
  int stop = digitalRead(STOP_PIN);   // remember these both have PULLUP res.
  int start = digitalRead(START_PIN);

  if (RUNNING == true && stop == 0) {
    RUNNING = false;
    //Serial.println(">>Stopping pump");
  }
  if (RUNNING == false && start == 0){
    RUNNING = true;
    //Serial.println(">>Starting pump");
  }
}

int read_voltage_av(int pin){
  int s = 0;
  for (int i=0; i<10; i++){
    s += analogRead(pin);
  }
  return s / 10;
}

void setup() {
  Serial.begin(9600);
  pinMode(PUMP_PIN, OUTPUT);
  //pinMode(START_PIN, INPUT_PULLUP);
  //pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  delay(1000);

  // if we haven't exceeded the run time
  if (millis() < (MAX_RUN_TIME * 1000)){
      //check_status();                                     // see if we are running
      int pressure_voltage = read_voltage_av(ANALOG_PIN); // read the presure av

      float pressure = calc_pressure(pressure_voltage);
      //Serial.println(" - voltage: " + String(pressure_voltage) + " | pressure: " + String(pressure));

      bool pump_needed = vacuum_on(pressure);             // determine if pump needed

      if (pump_needed == true && RUNNING == true) {
        Serial.println(" >>Pump ON<< ");
        digitalWrite(PUMP_PIN, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else {
        Serial.println(" >>Pump OFF<< ");
        digitalWrite(PUMP_PIN, LOW);
        digitalWrite(LED_BUILTIN, LOW);
      }
  } else {
    Serial.println("Timer complete...");
    digitalWrite(PUMP_PIN, LOW);
  }

}
