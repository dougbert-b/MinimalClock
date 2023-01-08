/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Header file for speed_cntr.c.
 *
 * - File:               speed_cntr.h
 * - Compiler:           IAR EWAAVR 4.11A
 * - Supported devices:  All devices with a 16 bit timer can be used.
 *                       The example is written for ATmega48
 * - AppNote:            AVR446 - Linear speed control of stepper motor
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr@atmel.com
 *
 * $Name: RELEASE_1_0 $
 * $Revision: 1.2 $
 * $RCSfile: speed_cntr.h,v $
 * $Date: 2006/05/08 12:25:58 $
 *****************************************************************************/

#ifndef SPEED_CNTR_H
#define SPEED_CNTR_H


#include <Arduino.h>
#include <ESP8266WiFi.h>

/* *********************************************************************

  StepperCalc calc;
  calc.setParams(accel, decel, speed);
  calc.initRun(steps);
  while (calc.getRunState() != StepperCalc::STOP) {
    unsigned step_delay = calc.getStepDelay();
    step_the_motor(dir, step_delay);
    calc.calcNextStep();
  }
 ********************************************************************* */


class StepperCalc {

public:
  // Speed ramp states
  enum RampState {
    STOP = 0,
    ACCEL,
    RUN,
    DECEL
  };

  enum Direction {
    CW,
    CCW
  };

  StepperCalc();
  void setParams(long a, long d, long s);
  void initRun(signed int steps);
  long getStepDelay() const { return step_delay; }
  Direction getDirection() const { return dir; }
  RampState getRunState() const { return run_state; }
  void calcNextStep();


private:
  static unsigned long sqrt(unsigned long v);


  /*! \Brief Frequency of timer1 in [Hz].
   *
   * Modify this according to frequency used. 
   */
  // Delay time units are microseconds.
  static constexpr long T1_FREQ = 1000000;

  //! Number of (full)steps per round on stepper motor in use.
  static constexpr int STEPS = 4096;

  // Maths constants. To simplify maths when calculating in setParams().
  static constexpr double ALPHA = (2*3.14159/STEPS);                    // 2*pi/steps (i.e. radians per step)
  static constexpr long A_T_x100 = (long)(ALPHA*T1_FREQ*100);     // (ALPHA / T1_FREQ)*100
  static constexpr long T1_FREQ_148 = (int)((T1_FREQ*0.676)/100); // divided by 100 and scaled by 0.676
  static constexpr long A_SQ = (long)(ALPHA*2*10000000000);        // ALPHA*2*10000000000
  static constexpr long A_x20000 = (int)(ALPHA*20000);             // ALPHA*20000

  //! What part of the speed ramp we are in.
  RampState run_state;
  //! Direction stepper motor should move.
  Direction dir;
  
  //! accel  Acceleration to use, in 0.01*rad/sec^2.
  //! decel  Deceleration to use, in 0.01*rad/sec^2.  (Miat be positive.)
  //! speed  Max speed, in 0.01*rad/sec.
  long accel;
  long decel;
  long speed;

  //! Number of steps before we hit max speed.  Based on accel and speed, not
  //! step count
  long max_s_lim;
  

  //! Period of next timer delay. At start this value sets the acceleration rate.
  long step_delay;

  //! What step_pos to start decelaration
  long decel_start;
  //! Sets deceleration rate.  Should be negative.
  long decel_val;

  //! Minimum time delay (max speed)
  long min_delay;

  //! Counter used when accelerating/decelerating to calculate step_delay.
  long accel_count;

  // Remember the last step delay used when accelerating.
  long last_accel_delay;

  // Counting steps when moving.
  long step_count = 0;

  // Keep track of remainder from new_step-delay calculation to increase accurancy
  long rest = 0;
};



#endif
