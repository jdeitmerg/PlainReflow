/* This file is part of PlainReflow, a simple reflow oven controller.
 *
 * @file reflow_profile.cpp
 * @author Jonas Deitmerg
 * @date 2017
 * @copyright MIT License
 *
 * For more information, see https://github.com/jdeitmerg/PlainReflow
 */

#include "reflow_profile.h"
#include <Arduino.h>

/** Sets up a reflow value you can query for the current temperature.
 * The current temperatures value is linearly interpolated from the
 * setpoint before and after the current time.
 * If the current time is outside the window of the reflow profile,
 * the output is set to the first/last value.
 *
 * T is the array of temperatures.
 * 
 *      ^
 *      |
 * T[3] |                                    .*
 *      |                                  .'  *.
 * T[1] |                .***.           .'      *
 *      |             .*'     '**.     .'         *
 * T[2] |          .*'            '***'            *.
 *      |       .*'                                  *
 * T[0] |******'                                      *
 *      |                                              *.
 * T[4] |                                                ************
 *      |
 *      '------------------------------------------------------------->
 *              |          |         |        |          |
 *           times[0]   times[1]  times[2]  times[3]  times[4]
 */

reflow_profile::reflow_profile(unsigned int numvals,
                               const unsigned long times[],
                               const float temperatures[])
{
    nvals = numvals;
    timestamps = times;
    tempvals = temperatures;
    reset();
}

void reflow_profile::reset(void) {
    reset_timestamp = millis();
}

float reflow_profile::getval(void) {
    // Find interval we're currently in
    // Interval 0 is from 0 to times[0], 1 is from times[0] to times[1]
    // and so on.
    unsigned long ts = millis()-reset_timestamp;
    unsigned int interval = 0;
    while(timestamps[interval] <= ts) {
        interval++;
    }
    if(interval == 0) {
        // Not a ramp, we're just trying to get to the initial
        // temperature.
        return(tempvals[0]);
    }
    if(interval == nvals) {
        // We're done, not a ramp either.
        // Return last value.
        return(tempvals[nvals-1]);
    }
    // In any other case we're linearly ramping between two temperature
    // values
    unsigned long t_start = timestamps[interval-1];
    unsigned long interval_len = timestamps[interval]-t_start;
    // Temperature at beginning of interval
    float temp_start = tempvals[interval-1];
    // Temperature difference between end and beginning of interval
    float temp_interval = tempvals[interval] - tempvals[interval-1];

    // Change of temperature since beginning of current interval
    float delta_temp = float(ts - t_start)/float(interval_len)*temp_interval;
    return(temp_start + delta_temp);
}

