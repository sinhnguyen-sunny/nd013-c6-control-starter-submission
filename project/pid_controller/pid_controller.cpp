/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min) {
    /**
     * TODO: Initialize PID coefficients and output limits.
     **/
    kp_ = Kp;
    ki_ = Ki;
    kd_ = Kd;
    output_limit_max_ = output_lim_max;
    output_limit_min_ = output_lim_min;

    // Initialize errors and delta time to zero
    error_ = 0.0;
    derror_ = 0.0;
    ierror_ = 0.0;
    dt_ = 0.0;
}

void PID::UpdateError(double cte) {
    /**
     * TODO: Update PID errors based on cte.
     **/
    if (dt_ > 1e-6) {
        derror_ = (cte - error_) / dt_;
    } else {
        derror_ = 0.0;
    }

    ierror_ += cte * dt_;
    error_ = cte;
}

double PID::TotalError() {
    /**
     * TODO: Calculate and return the total error
     * The code should return a value in the interval [output_limit_min_, output_limit_max_]
     */
    double control = kp_ * error_ + ki_ * ierror_ + kd_ * derror_;

    if (control > output_limit_max_) {
        control = output_limit_max_;
    } else if (control < output_limit_min_) {
        control = output_limit_min_;
    }

    return control;
}

void PID::UpdateDeltaTime(double new_delta_time) {
    /**
     * TODO: Update the delta time with new value
     */
    dt_ = new_delta_time;
}
