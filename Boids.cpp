#include "Boids.h"

//Constructor
BOID::BOID (double x, double y){
	Position.x		=	x;
	Position.y		=	y;
	Velocity.X		= 	0;
	Velocity.Y	 	=	0;
	ObjectsDetected	=	false;
	PreviousWall.X	= 	-1;
	PreviousWall.Y	= 	-1;
};

//Constructor		
BOID::BOID (double x, double y, vector2D v){
	Position.x		=	x;
	Position.y		=	y;
	Velocity 		=	v;
	ObjectsDetected	=	false;
	PreviousWall.X	= 	-1;
	PreviousWall.Y	= 	-1;
};

//Check if avatar is still connected
bool BOID::getSignal (){
	return Signal;
};

//Find Swarm
void BOID::findSwarm(double X, double Y){
}

//set velocity vector
void BOID::goTo(double X, double Y){
	Velocity.X =  X - Position.x;
	Velocity.Y =  Y - Position.y;
	if (distance(Position.x, Position.y, X, Y) < DEFAULT::RADIUS * 1.5){
		Velocity.X =  0;
		Velocity.Y =  0;
	}
	Velocity = vectNormalize(Velocity);
}

//set scatter mode
void BOID::scatter (double X, double Y){
	Velocity.X =  - X + Position.x;
	Velocity.Y =  - Y + Position.y;
	if (distance(Position.x, Position.y, X, Y) > 70 * (rand() % 180 + 100) /100){
		Velocity.X =  0;
		Velocity.Y =  0;
	}
	Velocity = vectNormalize(Velocity);
}

//set Boid coordinates
void BOID::setPosition (double X, double Y){
	cv::Point A(X,Y);
	Position = A;
};		

//get Boid Coordinates
cv::Point BOID::getPosition (){
	return Position;
};		

//set object detect status
void BOID::setObjectDetect(bool status){
	ObjectsDetected = status;
};

bool BOID::objectDetected(){
	return ObjectsDetected;
}

//GetVelocity 
vector2D BOID::getVelocity (){
	return Velocity;
};

//setVelocity
void BOID::setVelocity (vector2D A){
	Velocity = A;
};











