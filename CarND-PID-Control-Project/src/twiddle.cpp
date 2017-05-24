#include "twiddle.h"
#include <iostream>
#include <math.h>       /* fabs */
#include "Eigen/Dense"
using Eigen::VectorXd;

using namespace std;


twiddle::twiddle() {}

twiddle::~twiddle() {}

void twiddle::Init(double gamma, double wait_count, double set_speed, VectorXd Params) {

    cout<<"Initializing Filter \n";

    this->gamma = gamma;
    this->wait_count = wait_count;
    this->set_speed = set_speed;
    this->Params = Params;

    Prev_Params = Params;

    dParams << 0.01,0.0000001,0.01,0.02,0.05;
    //dParams << 0.0,0.0,0.0,0.0,0.0;

    prev_err = 0;
    count = 0;
    increase = 1;
    param_num = 0;
  }

void twiddle::savePrevious(double value) {
      prev_err = error;
  }

double twiddle::countIter(){
  count += 1;
  return count;
}

void twiddle::setCount(double value){
  count = value;
}


void twiddle::changeParam(double increase,VectorXd Params){

  this->increase = increase;
  this->Params = Params;
  if (increase == 1){
      Params[param_num] += dParams[param_num];
    } else{
      Params[param_num] -= 2*dParams[param_num];
    }


}


void twiddle::calcError(double cte,double speed){

  //cout<<", Prev_err : "<< prev_err;

  error = .05*(set_speed-speed)*(set_speed-speed)/set_speed/set_speed+ cte*cte;
  error = error + gamma*prev_err;
  //cout<<"Err : "<<error<<endl;
  prev_err = error;


}

VectorXd twiddle::updateparameters(){

  cout<<"Kp : "<<Params[0];
  cout<<",Ki : "<<Params[1];
  cout<<",Kd : "<<Params[2];
  cout<<",Ks : "<<Params[3];
  cout<<",Kst : "<<Params[4];
  cout<<endl;

  cout<<"Old_err : "<<old_err;
  cout<<",error : "<<error;
  cout<<endl;
  cout<<"increase : "<<increase;
  cout<<endl;


  if (increase == 1){
    if (old_err >= error){
        dParams[param_num] *= 1.1;
        old_err = error;
        param_num = param_num+1;
        if (param_num==5){
          param_num = 0;
        }

        return Params;
      } else {
        Params[param_num] -= 2*dParams[param_num];
        increase = 0;
        old_err = error;
        param_num = param_num+1;
        if (param_num==5){
          param_num = 0;
        }

        return Params;
      }

    }
  if (increase == 0){
    if (old_err >= error){
        dParams[param_num] *= 1.1;
        old_err = error;
        param_num = param_num+1;
        if (param_num==5){
          param_num = 0;
        }

        return Params;
      } else {


        dParams *= 0.9;
        Params[param_num] += dParams[param_num];
        increase = 1;
        old_err = error;
        param_num = param_num+1;
        if (param_num==5){
          param_num = 0;
        }

        return Params;

      }



  }

}
