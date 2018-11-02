#include "vectOp.h"

//Calculate distance between 2 Point
double distance(double x1, double y1, double x2, double y2){
	double result;
	result = sqrt(pow(abs(x1 - x2), 2) + pow(abs (y1 - y2), 2));
	return result;
};

//calculate vector length
double vectLength(vector2D A){
	double result;
	result = sqrt(pow(A.X, 2) + pow(A.Y, 2));
	return result;
};

//negateVector
vector2D vectNegate(vector2D A){
	A.X *= -1;
	A.Y *= -1;
	return A;
}

//Normalize vector (aka divide by its length)
vector2D vectNormalize (vector2D A){
	double len = vectLength(A);
	if (len != 0){
		A.X /= len;
		A.Y /= len;
	}
	return A;
}

//Multiply vector
vector2D vectMultiply(vector2D A, double B){
	A.X *= B;
	A.Y *= B;
	return A;
}

//Calculate angle between 2 vector
double vectAngle(vector2D A, vector2D B){
	double Angle;
	
	return Angle;
};

//Calcualte distance from a point to a line
double distanceToLine(cv::Point A, cv::Point B, cv::Point C){
	double result;
	result 	= 	abs((B.y - A.y) * C.x - (B.x - A.x) * C.y + B.x * A.y - A.x * B.y);
	result /=	sqrt(pow((B.y - A.y), 2) + pow((B.x - A.x), 2));
	return result;
}

//Find perpendiculat Point from a point to a line
cv::Point findPerpendicularPoint(cv::Point P0, cv::Point P1, cv::Point P2){
	vector2D directionVect, normalVect;
	
	//-----Find Direction vector of the line----- (or normal vector of the  
	directionVect.X = P0.x - P1.x;
	directionVect.Y = P0.y - P1.y;
	//-----Find Normal vector of the line------- (or direction vector of the perpendicular line
	normalVect.X 	= directionVect.Y * -1;
	normalVect.Y	= directionVect.X;
	//Calculate 
	double C0 = -directionVect.Y * P0.x + directionVect.X * P0.y; 
	double C1 = -normalVect.Y * P2.x + normalVect.X * P2.y;
	
	cv::Point result;
	result.y = (C0 * directionVect.X - C1 * directionVect.Y) / (pow(directionVect.X, 2) + pow(directionVect.Y, 2));
	result.x = (C1 * directionVect.X * -1 - C0 * directionVect.Y) /  (pow(directionVect.X, 2) + pow(directionVect.Y, 2));
	return result;
}
