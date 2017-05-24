#include "deadband.h"
#include <iostream>
#include <math.h>       /* fabs */

using namespace std;


deadband::deadband() {}

deadband::~deadband() {}

void deadband::Init(double dead_zone, double K) {

    cout<<"Initializing deadband controller \n";

    this->K = K; // Proportionnal term
	  this->dead_zone = dead_zone; // Integral term

  }




double deadband::deadband_control(double error) {
  double control = 0;

  if (fabs(error)>dead_zone){
    if (error>0){
      control = -K*(error-dead_zone);
    }else{
      control = -K*(error+dead_zone);
    }
  }

	return control;

}
