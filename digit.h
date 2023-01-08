/*
 * digit.h
 *
 *  Created on: 22.08.2021
 *      Author: Uwe
 */

#ifndef DIGIT_H_
#define DIGIT_H_

// settings
#define EIGHT_DIGIT 		false     	// true if your machine has 8 digits
#define ORIGIN_SENSOR 		true   	// true if you installed origin sensor
#define ORIGIN_COMPENSATION 0 		// compensation of origin mark position
#define ORIGIN_BRIGHTMARK	false		// true if origin mark is brighter than background
#define ORIGIN_THRES 		100      	// photo reflector sensor threshold
#define PRE_MOVE 			true		// true to start move earlier to reach the end position just in time
#define DEBUG 				false		// true for additional serial debug messages

#if EIGHT_DIGIT
#define DIGIT 8
#else
#define DIGIT 4
#endif

typedef struct {
  int v[DIGIT];
} Digit;


#endif /* DIGIT_H_ */
