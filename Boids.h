#ifndef BOIDS_H_
#define BOIDS_H_

#include <stdint.h>
#include "vectOp.h"

class BOID{
	private:
		bool 			Signal;
		cv::Point 		Position;
		vector2D		Velocity;
		bool			ObjectsDetected;
		
	public:
	
		BOID (double x, double y);									//Constructor
		
		BOID (double x, double y, vector2D velocity);				//Constructor
		
		bool getSignal ();											//Check if avatar is still connected
		
		void findSwarm (double X, double Y);						//Find Swarm
		
		void goTo (double X, double Y);								//Set velocity vector to go to a destination
		
		void scatter (double X, double Y);							//Set velocity to scatter from a point
		
		void setPosition (double X, double Y);						//Set Coordinate
		
		cv::Point getPosition ();
		
		vector2D getVelocity ();									//GetVelocity 
		
		void setVelocity (vector2D A);								//setVelocity
		
		void setObjectDetect(bool status);
		
		bool objectDetected();
		
		vector2D PreviousWall;
		
		cv::Point LastDestinationRecorded;
		
		int	WallDirection;
}; 

#endif



