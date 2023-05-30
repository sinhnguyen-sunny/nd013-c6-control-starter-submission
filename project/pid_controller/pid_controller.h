/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:
    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();

    /*
    * Update the delta time.
    */
    void UpdateDeltaTime(double new_delta_time);

private:
    /*
    * Errors
    */
    double error_;
    double derror_;
    double ierror_;

    /*
    * Coefficients
    */
    double kp_;
    double ki_;
    double kd_;

    /*
    * Output limits
    */
    double output_limit_max_;
    double output_limit_min_;

    /*
    * Delta time
    */
    double dt_;
};

#endif //PID_CONTROLLER_H
