#include "PID.h"
#include <iostream>
#include <math.h>       /* fabs */

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

    //cout<<"Initializing PID \n";

    this->Kp = Kp; // Proportionnal term
	  this->Ki = Ki; // Integral term
	  this->Kd = Kd; // Differential term


}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error = i_error + cte;

  //cout<<"P_error: "<<p_error;
  //cout<<", D_error: "<<d_error;
  //cout<<", I_error: "<<i_error;
  //cout<<endl;


}

double PID::TotalError() {
  double steer = -Kp * p_error -Kd * d_error - Ki * i_error;

  if (fabs(steer)>1.0){
    if (steer> 0){
      steer = 1.0;
    } else {
      steer = -1.0;
    }
  }

	return steer;

}
