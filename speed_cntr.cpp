/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Linear speed ramp controller.
 *
 * Stepper motor driver, increment/decrement the position and outputs the
 * correct signals to stepper motor.
 *
 * - File:               speed_cntr.c
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
 * $RCSfile: speed_cntr.c,v $
 * $Date: 2006/05/08 12:25:58 $
 *****************************************************************************/


#include "speed_cntr.h"

/*! \brief Set some of the parameters for a run.  These parementers are
 *  unlikely to change from run to run, and can be set only once
 *  if they do not ever need to change.
 *
 *  \param a  Acceleration to use, in 0.01*rad/sec^2.
 *  \param d  Decelration to use, in 0.01*rad/sec^2.
 *  \param s  Max speed, in 0.01*rad/sec.
 */

void
StepperCalc::setParams(long a, long d, long s)
{
  accel = a;
  decel = d;
  speed = s;

  // Find out after how many steps does the speed hit the max speed limit.
  // max_s_lim = speed^2 / (2*alpha*accel)
  max_s_lim = speed*speed/(A_x20000*accel/100);

  // If we hit max speed limit before 0,5 steps it will round to 0.
  // But in practice we need to move atleast 1 step to get any speed at all.
  if(max_s_lim == 0){
    max_s_lim = 1;
  }
}


/*! \brief Initialize a stepper run
 *
 *  Makes the stepper motor move the given number of steps.
 *  The acceleration, deceleration, and seed must have previously been set by
 *  a call to setParams().
 *  It accelerates with given acceleration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param steps  Number of steps in run
 */

void
StepperCalc::initRun(signed int steps)
{
  // Note: accel, decel, speed are always positive, even for reverse rotation.
  
  //! Number of steps before we must start deceleration (if accel does not hit max speed).
  long accel_lim;

  run_state = STOP;
  step_count = 0;
  rest = 0;

  // Set direction from sign on step value.
  if(steps < 0){
    dir = CCW;
    steps = -steps;
  }
  else{
    dir = CW;
  }

  // If moving only 1 step.
  if(steps == 1){
    // Move one step...
    accel_count = -1;
    // ...in DECEL state.
    run_state = DECEL;
    // Just a short delay 
    step_delay = 1000;
  }
  // Only move if number of steps to move is not zero.
  else if(steps != 0){
    // Refer to documentation for detailed information about these calculations.

    // Set max speed limit, by calc min_delay to use in timer.
    // min_delay = (alpha / tt)/ w
    min_delay = A_T_x100 / speed;

    // Set acceleration by calc the first (c0) step delay .
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
    step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100;

    // Find out after how many steps we must start deceleration.
    // n1 = (n1+n2)decel / (accel + decel)
    accel_lim = ((long)steps*decel) / (accel+decel);

    // We must accelrate at least 1 step before we can start deceleration.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // Use the limit we hit first to calc decel.
    if(accel_lim <= max_s_lim){
      decel_val = accel_lim - steps;
    } else {
      decel_val = -((long)max_s_lim*accel)/decel;
    }

    // We must decelrate at least 1 step to stop.
    if(decel_val == 0){
      decel_val = -1;
    }

    // Find step to start decleration.
    decel_start = steps + decel_val;

    // If the maximum speed is so low that we dont need to go via acceleration state.
    if(step_delay <= min_delay){
      step_delay = min_delay;
      run_state = RUN;
    }
    else{
      run_state = ACCEL;
    }

    // Reset counter.
    accel_count = 0;
  }

}




StepperCalc::StepperCalc()
{
  // Tells what part of speed ramp we are in.
  run_state = STOP;
}

/*! \brief Timer/Counter1 Output Compare A Match Interrupt.
 *
 *  Increments/decrements the position of the stepper motor,
 *  applies the step delay, and updates the step delay for the next call.
 *  A new step delay is calculated to follow wanted speed profile
 *  on basis of accel/decel parameters.
 */
void StepperCalc::calcNextStep()
{
  // Holds next delay period.
  long new_step_delay;

  switch(run_state) {
    case STOP:
      // A no-op
      step_count = 0;
      rest = 0;
      break;

    case ACCEL:
      step_count++;
      accel_count++;
      new_step_delay = step_delay - (((2 * (long)step_delay) + rest)/(4 * accel_count + 1));
      rest = ((2 * (long)step_delay)+rest)%(4 * accel_count + 1);
      // Check if we should start deceleration.
      if(step_count >= decel_start) {
        accel_count = decel_val;
        run_state = DECEL;
      }
      // Check if we hit max speed.
      else if(new_step_delay <= min_delay) {
        last_accel_delay = new_step_delay;
        new_step_delay = min_delay;
        rest = 0;
        run_state = RUN;
      }
      break;

    case RUN:
      step_count++;
      new_step_delay = min_delay;
      // Check if we should start decelration.
      if(step_count >= decel_start) {
        accel_count = decel_val;
        // Start decelration with same delay as accel ended with.
        new_step_delay = last_accel_delay;
        run_state = DECEL;
      }
      break;

    case DECEL:
      step_count++;
      accel_count++;
     
      // Check if we at last step
      if(accel_count >= 0){
        run_state = STOP;
        new_step_delay = 0;
        rest = 0;
      } else {
        new_step_delay = step_delay - (((2 * (long)step_delay) + rest)/(4 * accel_count + 1));
        rest = ((2 * (long)step_delay)+rest)%(4 * accel_count + 1);
      }
      break;
  }

  step_delay = new_step_delay;
}

/*! \brief Square root routine.  Static.
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 *
 *  \param x  Value to find square root of.
 *  \return  Square root of x.
 */
unsigned long
StepperCalc::sqrt(unsigned long x)
{
  register unsigned long xr;  // result register
  register unsigned long q2;  // scan-bit register
  register unsigned char f;   // flag (one bit)

  xr = 0;                     // clear result
  q2 = 0x40000000L;           // higest possible result bit
  do
  {
    if((xr + q2) <= x)
    {
      x -= xr + q2;
      f = 1;                  // set flag
    }
    else{
      f = 0;                  // clear flag
    }
    xr >>= 1;
    if(f){
      xr += q2;               // test flag
    }
  } while(q2 >>= 2);          // shift twice
  if(xr < x){
    return xr +1;             // add for rounding
  }
  else{
    return xr;
  }
}
