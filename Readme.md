
## PID control for self-driving car

A better version with equations can be found [here](http://nbviewer.jupyter.org/github/vxy10/P4T2SCND_PIDControl/blob/master/Readme.ipynb)

In this project, I implemented a PID (Proportional-Integral-Derivative) control for letting a car drive around a race track. PID control is a simple control scheme where the error between desired and true state is taken as input, and its value, integral and derivatives are multipled by scalars to compute the commanded control input. The proportional term drives the error to zero, however, this results in error oscillating about the set point. To supress these oscillations a derivative term is introduced. Finally, due to modeling errors, set point not being zero or other errors, a control based on proportional and derivative term alone can have a drift. To avoid this drift, an integral term is introduced. The integral term accumulates error, and pushes the control in the opposite direction of accumulated error. This results in the steady state offset error to go to zero. PID control was tested by controlling a car driving around in a Unity simulator. The car simulator was provided by Udacity. The simulator returned cross track error and took throttle opening and steering angle as control inputs. I implemented two modes of control, first a simple/safe mode where the car drives around the track without touching the yellow lines, and second a Fast mode where the car drives around the race track to achieve maximum speed. In simulation, I was able to acheive a maximum speed of 78 mph. This mode however resulted in the car touching edges of the road in some occasions. Below I present control sysnthesis for each mode. 

### Safe mode: 

In safe mode, the objective was to drive around the track at a safe speed while not touching sides of the road. I therefore held throttle fixed at 0.3 and implemented a PID control that took cross track error as input and gave commanded steering angle as output. This control law can simply be written as, 

$$ Steering~angle = - \underbrace{K_p~ cte}_{proportional} - \underbrace{K_i~ \int cte ~dt}_{integral} - K_d \underbrace{\frac{d ~cte}{dt}}_{derivative} $$ 

The gains were manually tuned and set as, $K_p = 0.225$, $K_i = 0.0004$, and $K_d = 4$. The throttle was set to 0.3. The results of applying PID control for safe mode are presented in video below. As evident, the car does not cross the yellow lines.  

#### PID control for autonomous driving (Safe Mode) 

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/6CAoqJl8_2w/0.jpg)](https://www.youtube.com/watch?v=6CAoqJl8_2w)


### Fast mode:

In the Fast mode, I implemented control laws to command the throttle in addition to steering. As the objective was to go as fast as possible, the gains were set differently. I implemented a control for steering same as before,  

$$ Steering~angle = - \underbrace{K_p~ cte}_{proportional} - \underbrace{K_i~ \int cte ~dt}_{integral} - K_d \underbrace{\frac{d ~cte}{dt}}_{derivative} . $$ 

The throttle control was implemented as, 

$$ Throttle = K_{sp}(speed_{desired} - speed)- K_s (Steering~angle) - DB_{steer}- DB_{cte} $$ 

where, $K_{sp}$ defines a proportional controller for speed, $K_s$ a factor to allow slowing down for large steering angles, and DB stands for deadband controllers. Deadband controllers are controllers that act like a proportional controller, but have a dead-zone around zero error where control output is zero. The idea is to make a 'channel' to constraint the errors in. A simple deadzone controller for $x$ with deadzone given by $x_{db}$ can be implemented as, 

$$ f(x)= 
\begin{cases}
       -K(x-x_{db}) ,& \text{if }x \geq x_{db} \\
    0,           & \text{if} x_{db} \geq x \geq -x_{db} \\
    -K(x+x_{db}) ,& \text{if } - x_{db} \geq x 
\end{cases} $$ 


Deadband control has many benefits, most important being they prevent control chatter. In addition they can be useful to contrain error within certain bounds. All this is however true for a purely linear system, when interacting with a nonlinear system, a poorly designed deadband control can introduce unnecessary oscillations. For steering and throttle, the deadband was set to 0.35 and gains were set to 40 and 20 respectively. 

This control model had a total of 5 parameters, the 3 gains for PID control of steering and 2 gains for throttle control. These gains were initially set as $K_p = 0.1$, $K_i = 0.0004$, $K_d = 2$, $K_s = 2$  and $K_{sp} = 0.2$. A Twiddle algorithm with look window of 40 time points was implemented. As the error is not expected to be the same for different parts of the track, a discounted error function was implemented as follows. First cost at instant $i$ was computed,

$$  cost_i  = 0.05 \frac{(speed_{desired} - speed_i)^2}{speed_{desired}^2}  + cte_i^2$$

next all the previous costs were accumulated as 

$$  Objective_i  = cost_i + 0.8 Objective_{i-1}$$

The Twiddle algorithm tried to tune gains to minimize this objective function. Video below presents results of applying this control scheme to acheive a maximum speed of 80 mph. 


#### PID control for autonomous driving (78 mph run) 
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Zcfcy_gxqA0/0.jpg)](https://www.youtube.com/watch?v=Zcfcy_gxqA0)



Disclaimer: The high speed in fast mode came at the expense of complexity, where I implemented a deadband controller, twiddle, and filters for steering angle and throttle. The main purpose of doing so was for me to practice coding using OOP framework, hence the added complexity. 


# CarND-Controls-PID Environment settings. 
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./


```python

```
