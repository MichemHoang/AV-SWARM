#ifndef VECTOP_H_
#define VECTOP_H_

#include "math.h"
#include <stdlib.h>     /* srand, rand */
#include "typedefs.h"
#include <vector>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

//Calculate distance between 2 Point
double distance(double x1, double y1, double x2, double y2);

//vectMultiply
vector2D vectMultiply(vector2D A, double B);

//calculate vector length
double vectLength(vector2D A);

//negateVector
vector2D vectNegate(vector2D A);

//Normalize vector (aka divide by its length)
vector2D vectNormalize(vector2D A);

//Calculate angle between 2 vector 
double vectAngle(vector2D A, vector2D B);

//Calcualte distance from a point to a line
double distanceToLine(cv::Point A, cv::Point B, cv::Point C);

//Find intersection of a line with a perpendicular line with it through a point (This is way too long)
cv::Point findPerpendicularPoint(cv::Point A, cv::Point B, cv::Point C);

#endif


