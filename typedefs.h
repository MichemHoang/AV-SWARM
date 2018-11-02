#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string.h>

namespace DEFAULT{
	const int 	MAX_X		= 900;
	const int 	MAX_Y	  	= 1700;
	const int	RADIUS		= 30;
	const int	FRAME_RATE	= 30;
	const int 	EXIT_KEY  	= 27;
}

const uint32_t 	KEY_MASK  	= 0xEFFFFF;

struct vector2D{
	double X;
	double Y;
};

enum INIT {RANDOM, NONE};
enum MODE {FINDSWARM, FLOCK, SCATTER};

#endif
