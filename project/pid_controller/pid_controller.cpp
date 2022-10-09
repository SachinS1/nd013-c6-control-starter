/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
    this->Kpi = Kpi;
  this->Kii = Kii;
  this->Kdi = Kdi;
  this->output_lim_maxi = output_lim_maxi;
  this->output_lim_mini = output_lim_mini;
  error_sum = 0.0;
  prev_cte = 0.0;
  delta_time = 1;
  
  
  
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  
  cte_diff = cte - prev_cte;
  error_sum = error_sum + cte*delta_time;
  
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
  
  
  	double d_error = 0;
  	if (delta_time == 0)
      d_error = 0;
  	else
      d_error = Kdi*cte_diff/delta_time;
  	control = Kpi*prev_cte + d_error + Kii*error_sum;
      
      
    //to keep within bounds
      if (control>output_lim_maxi)
        control = output_lim_maxi;
      if (control < output_lim_mini)
        control = output_lim_mini;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  
  delta_time = new_delta_time;
  
}