/* PlainReflow
 *
 * @file PlainReflow.ino
 * @author Jonas Deitmerg
 * @date 2017
 * @copyright MIT License
 * 
 * Reflow oven controller running on Arduino Nano.
 * See https://github.com/jdeitmerg/PlainReflow
 */

// Digital outputs
//     relays -> heating elements
const int HEATINGROD1 = 3;
const int HEATINGROD2 = 4;
//     MOSFET -> thermistor
const int THERMISTOR_MOSFET = 13;

// Analog input for temperature measurement: 
const int TEMP_INP = A0;

static void oven_on(void) {
  // turn both heating elements on
  // active low
  digitalWrite(HEATINGROD1, LOW);
  digitalWrite(HEATINGROD2, LOW);
}

static void oven_off(void) {
  // turn both heating elements on
  // active low
  digitalWrite(HEATINGROD1, HIGH);
  digitalWrite(HEATINGROD2, HIGH);
}

static double measure_temperature(void) {
  // Measure voltage, calculate resistance and temperature
  // from that.

  // First off: Enable thermistor
  digitalWrite(THERMISTOR_MOSFET, LOW);
  // Wait for filter cap to charge up (time constant is 100µs,
  // we'll wait 50x that)
  delay(5);
  // Take several measurements and calculate average
  double avg = 0;
  for(int i=0; i<5; i++) {
    avg += analogRead(TEMP_INP);
  }
  avg /= 5;

  double inp_voltage = avg * 1.1 / 1023; // Reference voltage = 1.1V,
                                         // maximum value = 1023 (10 bits)
  // voltage divider 1k over PT100 thermistor connected to 5V
  double R_PT100 = 1000./(5./inp_voltage-1);

  // PT100 formula: R = 100*(1+A*T+B*T²)
  // with A = 3.9083e-3
  //      B = -5.775e-7
  const double PTA = 3.9083e-3;
  const double PTB = -5.775e-7;
  double temperature = 
    -(PTA/PTB/2)-sqrt((PTA/PTB/2)*(PTA/PTB/2)-(1-R_PT100/100)/PTB);
  return(temperature);
}

void setup() {
  // Setup relay outputs (make sure they're low by default)
  oven_off();
  pinMode(HEATINGROD1, OUTPUT);
  pinMode(HEATINGROD2, OUTPUT);

  // MOSFET for disabling thermistor:
  pinMode(THERMISTOR_MOSFET, OUTPUT);
  // disable by default
  digitalWrite(THERMISTOR_MOSFET, HIGH);

  // Set analog reference to 1.1V:
  analogReference(INTERNAL);
}

void loop() {

}
