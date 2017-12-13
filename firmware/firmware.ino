/* PlainReflow firmware file
 *
 * @file firmware.ino
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
 *  * "FIR Filter" by LeemanGeophysicalLLC - not very efficient but
 *                                           get's the job done.
 *
 */

#include <PID_v1.h>
#include <Plotter.h>
#include <Thread.h>
#include <ThreadController.h>
#include <FIR.h>
#include "reflow_profile.h"

Plotter plotter; // for plotting over serial

ThreadController thread_controller = ThreadController();
Thread plot_thread = Thread();
Thread control_thread = Thread();

// variables for PID controller
double PID_setpoint, PID_in, PID_out, PID_out_percent;

// for slow PWM:
const int pwm_window = 5000; // 5 seconds
volatile unsigned long pwm_start_time;

// only for plotting:
char oven_is_on = 0;

//Specify PID pointers and initial tuning parameters
PID control_PID(&PID_in, &PID_out, &PID_setpoint,2,5,1, DIRECT);

// Digital outputs
//     relay -> fan
const int FAN = 3;
//     relays -> heating elements
const int HEATINGROD1 = 4;
const int HEATINGROD2 = 5;
//     MOSFET -> thermistor
const int THERMISTOR_MOSFET = 13;

// Analog input for temperature measurement: 
const int TEMP_INP = A7;

/* The temperature signal is lowpass filtered using a
 * moving average filter during the acquisition.
 * The cutoff of that filter is still pretty high for
 * a temperature signal, we need another FIR
 * filter with the cutoff in the .5Hz range.
 */
FIR<double, 15> filt_2nd_stage;

/* The following values define the temperature profile. It consits of
 * timestamps and temperatures at those timestamps. Between the
 * timestamps, the temperature is interpolated linearly.
 */
const unsigned int tprofile_n = 5;
const unsigned long tprofile_times[] = {30, 180, 220, 260, 300};
const float tprofile_temps[] =         {80, 200, 235, 235,   0};
reflow_profile tprofile(tprofile_n, tprofile_times, tprofile_temps);


void plot_callback(void) {
  // plotting thread
  plotter.Plot();
}

void control_callback(void) {
  // control thread
  PID_setpoint = tprofile.getval();
  PID_in = measure_temperature();

  control_PID.Compute();
  PID_out_percent = PID_out/pwm_window*100;

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
  // we'll wait 10x that)
  delayMicroseconds(1000);
  /* Take several measurements and apply low pass filter
   * As it turns out, the FIR lowpass with the lowest possible cutoff
   * frequency is the moving average filter. We should only sample for
   * ~10ms as this function is blocking. By reading one sample after the
   * other, we should get into the 8kHz sampling rate range. Just enough
   * to avoid most aliases with the analog lowpass cutoff at ~4.5kHz.
   *
   * By sampling for only 10ms every ~100ms, we will get plenty of
   * aliasing in the sub 50Hz range. That can't be helped with this
   * simple approach.
   *
   * We should see less than 10kHz*10ms = 100 samples. You need 7 bits
   * to index 100 samples, each samle is 10 bits so the sum of all samples
   * won't necessarily fit into 16 bits. What a shame.
   */
  uint32_t sum = 0;
  uint16_t count = 0;
  unsigned long start_time = millis();
  while(millis()-start_time < 10) {
    sum += analogRead(TEMP_INP);
    count++;
  }
  // Disable thermistor again so it doesn't heat up / break
  digitalWrite(THERMISTOR_MOSFET, HIGH);
  double avg = (double) sum / count;

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
  return(filt_2nd_stage.processReading(temperature));
}

void setup() {
  // Setup relay outputs (make sure they're low by default)
  oven_off();
  pinMode(HEATINGROD1, OUTPUT);
  pinMode(HEATINGROD2, OUTPUT);

  pinMode(TEMP_INP, INPUT);

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
  plotter.AddTimeGraph( "Reflow oven variables", 6000,//18000, // about 3 minutes
                          "PID Setpoint (°C)", PID_setpoint,
                          "PID Input (°C)", PID_in,
                          "PID Output (%)", PID_out_percent,
                          "Oven state (on/off)", oven_is_on);

  // Configure PID controller
  control_PID.SetOutputLimits(0, pwm_window);
  // turn PID controller on
  control_PID.SetMode(AUTOMATIC);

  // Calculated using scipy.signal. Have a look at the "design" folder
  double fir_coeffs[] = {
        0.0367883 ,  0.05045512,  0.06366198,  0.07568267,  0.08583937,
        0.09354893,  0.09836316,  0.1       ,  0.09836316,  0.09354893,
        0.08583937,  0.07568267,  0.06366198,  0.05045512,  0.0367883 };
  filt_2nd_stage.setFilterCoeffs(fir_coeffs);
  tprofile.reset(); // Start the temperature profile
}

void loop() {
  thread_controller.run();
}
