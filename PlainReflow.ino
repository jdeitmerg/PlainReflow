/* PlainReflow
 *
 * @file PlainReflow.ino
 * @author Jonas Deitmerg
 * @date 2017
 * @copyright MIT License
 *
 * Reflow oven controller running on Arduino Nano.
 * See https://github.com/jdeitmerg/PlainReflow
 *
 * Used libraries (please install via library manager):
 *  * "PID" by Brett Beauregard
 *  * "Plotter" by Devin Conley
 *  * "ArduinoThread" by Ivan Seidel
 *
 */

#include <PID_v1.h>
#include <Plotter.h>
#include <Thread.h>
#include <ThreadController.h>

Plotter plotter; // for plotting over serial

ThreadController thread_controller = ThreadController();
Thread plot_thread = Thread();
Thread control_thread = Thread();

// variables for PID controller
double PID_setpoint, PID_in, PID_out, PID_out_by_10;

// for slow PWM:
const int pwm_window = 5000; // 5 seconds
volatile unsigned long pwm_start_time;

// only for plotting:
char oven_is_on = 0;

//Specify PID pointers and initial tuning parameters
PID control_PID(&PID_in, &PID_out, &PID_setpoint,2,5,1, DIRECT);

// Digital outputs
//     relays -> heating elements
const int HEATINGROD1 = 3;
const int HEATINGROD2 = 4;
//     MOSFET -> thermistor
const int THERMISTOR_MOSFET = 13;

// Analog input for temperature measurement: 
const int TEMP_INP = A0;

void plot_callback(void) {
  // plotting thread
  plotter.Plot();
}

void control_callback(void) {
  // control thread
  PID_setpoint = 250; // constant for testing
  PID_in = measure_temperature();

  control_PID.Compute();
  PID_out_by_10 = PID_out/10;

  unsigned long now = millis();
  if (now - pwm_start_time > pwm_window) {
    // Overflow occured, switch to next window
    pwm_start_time += pwm_window;
  }
  if (PID_out > now - pwm_start_time) {
    oven_on();
  }
  else {
    oven_off();
  }
}

static void oven_on(void) {
  // turn both heating elements on
  // active low
  digitalWrite(HEATINGROD1, LOW);
  digitalWrite(HEATINGROD2, LOW);
  oven_is_on = 30; // 30 is well visible in the serial plot
}

static void oven_off(void) {
  // turn both heating elements on
  // active low
  digitalWrite(HEATINGROD1, HIGH);
  digitalWrite(HEATINGROD2, HIGH);
  oven_is_on = 0;
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

  // Set up threads and thread controller
  plot_thread.onRun(plot_callback);
  plot_thread.setInterval(10);

  control_thread.onRun(control_callback);
  control_thread.setInterval(100);

  thread_controller.add(&plot_thread);
  thread_controller.add(&control_thread);

  // Set up serial plotter
  plotter.Begin(); // start plotter
  plotter.AddTimeGraph( "Reflow oven variables", 18000, // about 3 minutes
                          "PID Setpoint (°C)", PID_setpoint,
                          "PID Input (°C)", PID_in,
                          "PID Output (10 ms)", PID_out_by_10,
                          "Oven state (on/off)", oven_is_on);

  // Configure PID controller
  control_PID.SetOutputLimits(0, pwm_window);
  // turn PID controller on
  control_PID.SetMode(AUTOMATIC);
}

void loop() {
  thread_controller.run();
}
