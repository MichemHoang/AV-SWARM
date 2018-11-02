#ifndef SWARM_H_
#define SWARM_H_

#include "Boids.h"

class SWARM{
	private:
		std::vector<BOID> 	Boids;					//vector of all boids
		int					SwarmNo;				//Number of the swarm
		double 				Flock_radius;			//Radius where each boid find their local flockmate
		MODE				mode;					//SWARM mode
		std::vector<cv::Point> ObsVertice; 			//Obstacle vertices
		vector2D			Dest;					//Destination the flock is heading to
		vector2D			Center;					//Center of the flock
		vector2D cohension (int boidNo);			//steer to move toward the average position of local flockmates
		vector2D seperation (int boidNo);			//Steer to avoid crowding local flockmates
		vector2D alignment (int boidNo);			//steer towards the average heading of local flockmates
		
		
	public:	
		SWARM (int Number);											//Constructor
		
		SWARM (int Number, double radius, MODE init);				//Constructor
		
		SWARM (int Number, double radius, int MaxBoid, MODE init);	//Constructor
		
		vector2D findCenter();
		
		void setObstacleVertices(cv::Point * Obs);
		
		void update();												//Update the swarm
		
		void setFindSwarmMode();									//Find Boids to find swarm mode 
		
		void setScatterMode();										//Scatter
		
		void addBoid (BOID NewBoid);								//Add boid to the swarm
		
		void setFlockmateDistance (double dist);					//Set local flockmate radius
		
		std::vector<BOID> getBoids();								//Return all boids
		
		void setDestination (double X, double Y);					//setDestination
		
		vector2D avoidance (int boidNo);			//Avoid Obstacle
};

#endif

