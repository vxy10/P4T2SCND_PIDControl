#ifndef DB_H
#define DB_H

class deadband {
public:
  /*
  * Errors
  */


  /*
  * Coefficients
  */
  double K;
  double dead_zone;

  /*
  * Constructor
  */
  deadband();

  /*
  * Destructor.
  */
  virtual ~deadband();

  /*
  * Initialize PID.
  */
  void Init(double dead_zone, double K);

  /*
  * Calculate the deadzone error.
  */
  double deadband_control(double error);
};


#endif /* DB_H */
