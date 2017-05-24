#ifndef FIL_H
#define FIL_H

class filter {
public:
  /*
  * Errors
  */


  /*
  * Coefficients
  */
  double alpha;
  double prev_value;

  /*
  * Constructor
  */
  filter();

  /*
  * Destructor.
  */
  virtual ~filter();

  /*
  * Initialize PID.
  */
  void Init(double alpha);

  void savePrevious(double value);

  /*
  * Calculate the deadzone error.
  */
  double smooth(double value);
};


#endif /* FIL_H */
