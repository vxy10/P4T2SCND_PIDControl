#ifndef TW_H
#define TW_H

#include "Eigen/Dense"
using Eigen::VectorXd;

class twiddle {
public:
  /*
  * Errors
  */


  /*
  * Coefficients
  */
  double count = 0;
  double gamma = 0;
  double wait_count = 100;
  double set_speed = 0;
  double increase = 0;
  double decrease = 0;
  double param_num = 0;
  double prev_err;
  double old_err =0;
  double error;

  VectorXd Params = VectorXd(5);
  VectorXd Prev_Params = VectorXd(5);
  VectorXd dParams = VectorXd(5);
  /*
  * Constructor
  */
  twiddle();

  /*
  * Destructor.
  */
  virtual ~twiddle();

  /*
  * Initialize PID.
  */
  void Init(double gamma, double wait_count, double set_speed, VectorXd Params);

  double countIter();

  void setCount(double value);

  void calcError(double cte,double speed);

  void savePrevious(double value);

  void changeParam(double increase,VectorXd Params);

  VectorXd updateparameters();


};


#endif /* TW_H */
