#include "Swarm.h"

//Constructor
SWARM::SWARM (int Number){
	SwarmNo = Number;
	Flock_radius = 35;
	mode = FLOCK;
};

//Constructor
SWARM::SWARM (int Number, double radius, MODE Init){
	SwarmNo = Number;
	Flock_radius = radius;
	mode = Init;
	if (mode == FLOCK) setFindSwarmMode();
	else if (mode == SCATTER) setScatterMode();
};				

//Constructor
SWARM::SWARM (int Number, double radius, int MaxBoid, MODE Init){
	Flock_radius = radius;
	SwarmNo = Number;
	mode = Init;
	if (mode == FLOCK) setFindSwarmMode();
	else if (mode == SCATTER) setScatterMode();
	for (int i = 0; i < MaxBoid; i++){
		BOID A(rand() % 500 + 500, rand() % 500 + 200);
		addBoid(A);
	}
	//BOID A(500, 357);
	//addBoid(A);
};

//set SWARM destination
void SWARM::setDestination (double X, double Y){
	mode = FLOCK;
	double x_x, y_y;
	Dest.X = X;
	Dest.Y = Y;
	for (int i = 0; i < Boids.size(); i++){
		Boids[i].goTo(Dest.X, Dest.Y);
	}
}

//set obstacles list
void SWARM::setObstacleVertices(cv::Point * Obs){
	for (int i = 0; i < 5; i++)
		ObsVertice.push_back(Obs[i]);
};

//Update position and velocity of boids
void SWARM::update(){
	vector2D v, SE, AL, CO, AV;
	
	if (mode == FLOCK){
		findCenter();
		double max_Dist = -1;
		for (int i = 0; i < Boids.size(); i++){
			cv::Point BoidPosition = Boids[i].getPosition();
			double tmp = distance(Dest.X, Dest.Y, BoidPosition.x, BoidPosition.y);
			if (tmp > max_Dist) max_Dist = tmp;
		}
		if (distance(Center.X, Center.Y, Dest.X, Dest.Y) <= Flock_radius * 1.5 && max_Dist <= Flock_radius * (Boids.size()/15 + 2)){
			vector2D Velo;
			Velo.X = 0; Velo.Y = 0;
			for (int i = 0; i < Boids.size(); i++){
				Boids[i].setVelocity(Velo);
			}
		}
		else {
			for (int i = 0; i < Boids.size(); i++){
				Boids[i].goTo(Dest.X, Dest.Y);
			}
			for (int i = 0; i < Boids.size(); i++){
				v.X = 0;
				v.Y = 0;
				cv::Point BoidPosition;
				BoidPosition = Boids[i].getPosition();
				SE	=	seperation(i);
				AL	=	alignment(i);
				CO	=	cohension(i);
				AV	=	avoidance(i);
				
				vector2D CenterVelo;
				CenterVelo.X = Dest.X - Center.X;
				CenterVelo.Y = Dest.Y - Center.Y;
				CenterVelo = vectNormalize(CenterVelo);
				
				v.X	=	1.8 * AL.X + 2.2 * SE.X + 1.6 * CO.X +  1 * CenterVelo.X + 6 * AV.X;
				v.Y	=	1.8 * AL.Y + 2.2 * SE.Y + 1.6 * CO.Y +  1 * CenterVelo.Y + 6 * AV.Y;
				v	=	vectNormalize(v);
				
				
				Boids[i].setVelocity(v);
				BoidPosition.x	+= v.X * DEFAULT::FRAME_RATE * 2 / (12 * 1 )* ((rand() % 50 + 100) /100);
				BoidPosition.y	+= v.Y * DEFAULT::FRAME_RATE * 2 / (12 * 1 )* ((rand() % 50 + 100) /100);
				Boids[i].setPosition(BoidPosition.x, BoidPosition.y);
			}
		}
	} else if (mode == SCATTER){
		for (int i = 0; i < Boids.size(); i++){
			Boids[i].scatter(Center.X, Center.Y);
		}
		for (int i = 0; i < Boids.size(); i++){
			v.X = 0;
			v.Y = 0;
			vector2D A = Boids[i].getVelocity();
			cv::Point BoidPosition;
			BoidPosition = Boids[i].getPosition();
			SE	=	seperation(i);
			v.X	=	A.X  + 0.7 * SE.X;
			v.Y	=	A.Y  + 0.7 * SE.Y;
			v	=	vectNormalize(v);
			
			Boids[i].setVelocity(v);
			BoidPosition.x += v.X * DEFAULT::FRAME_RATE * 5 / 12 * ((rand() % 70 + 100) /100);
			BoidPosition.y += v.Y * DEFAULT::FRAME_RATE * 5 / 12 * ((rand() % 70 + 100) /100);
			Boids[i].setPosition(BoidPosition.x, BoidPosition.y);
		}
	}
};

//Add boids to the Swarm
void SWARM::addBoid(BOID newBoid){
	Boids.push_back(newBoid);
};

//return all boids
std::vector<BOID> SWARM::getBoids(){
	return Boids;
};

//Locate the center of the swarm, set Mode to find swarm
void SWARM::setFindSwarmMode(){
	mode = FINDSWARM;
	cv::Point BoidPosition;
	Dest.X = 0;
	Dest.Y = 0;
	for (int i = 0; i < Boids.size(); i++){
		BoidPosition = Boids[i].getPosition();
		Dest.X += BoidPosition.x;
		Dest.Y += BoidPosition.y;
	}
	Dest.X /= Boids.size();
	Dest.Y /= Boids.size();
};

vector2D SWARM::findCenter(){
	Center.X = 0;
	Center.Y = 0;
	cv::Point BoidPosition;
	for (int i = 0; i < Boids.size(); i++){
		BoidPosition = Boids[i].getPosition();
		Center.X += BoidPosition.x;
		Center.Y += BoidPosition.y;
	}
	Center.X /= Boids.size();
	Center.Y /= Boids.size();
};

//Set mode to scatter
void SWARM::setScatterMode(){
	mode = SCATTER;
	findCenter();
};	

//-------------------STEERING BEHAVIORS-------------------

//Steer to avoid crowding local flockmates
vector2D SWARM::seperation (int boidNo){
	vector2D velocity;
	velocity.X = 0;
	velocity.Y = 0;
	cv::Point BoidPosition;
	BoidPosition = Boids[boidNo].getPosition();	//get coordinate of target boid
	
	for (int i = 0; i < Boids.size(); i++){
		cv::Point MatePosition = Boids[i].getPosition();
		double Dist = distance(BoidPosition.x, BoidPosition.y, MatePosition.x, MatePosition.y);
		if (Dist <= Flock_radius && i != boidNo){			//local flockmate
			if (Dist != 0){
				velocity.X += (MatePosition.x - BoidPosition.x)/(Dist * Dist);
				velocity.Y += (MatePosition.y - BoidPosition.y)/(Dist * Dist);
			}
		}
	}
	velocity = vectNegate(velocity);
	velocity = vectNormalize(velocity);
	return velocity;
};
			
//steer towards the average heading of local flockmates
vector2D SWARM::alignment (int boidNo){
	vector2D velocity;
	velocity.X = 0;
	velocity.Y = 0;
	cv::Point BoidPosition;
	BoidPosition = Boids[boidNo].getPosition();	//get coordinate of target boid
	
	for (int i = 0; i < Boids.size(); i ++){
		cv::Point MatePosition = Boids[i].getPosition();
		if (distance(BoidPosition.x, BoidPosition.y, MatePosition.x, MatePosition.y) <= Flock_radius){			//local flockmate
			vector2D agent = Boids[i].getVelocity();
			velocity.X 	+=	agent.X;
			velocity.Y 	+=	agent.Y;
		}
	}
	velocity = vectNormalize(velocity);
	return velocity;
};
		
//steer to move toward the average position of local flockmates
vector2D SWARM::cohension (int boidNo){
	vector2D velocity;
	velocity.X = 0;
	velocity.Y = 0;
	cv::Point BoidPosition;
	BoidPosition = Boids[boidNo].getPosition();	//get coordinate of target boid
	
	for (int i = 0; i < Boids.size(); i ++){
		cv::Point MatePosition = Boids[i].getPosition();
		if (distance(BoidPosition.x, BoidPosition.y, MatePosition.x, MatePosition.y) <= Flock_radius){			//local flockmate
			velocity.X += MatePosition.x;
			velocity.Y += MatePosition.y;
		}
	}
	velocity = vectNormalize(velocity);
	return velocity;
};

//Steer to avoid object
vector2D SWARM::avoidance (int boidNo){
	vector2D velocity, perpendicular;
	velocity.X = 0;
	velocity.Y = 0;
	bool foundObstacle = false;
	double distance = 0;
	double minDistance = 1000;
	int	vertice;
	cv::Point BoidPosition;
	
	BoidPosition = Boids[boidNo].getPosition();	//get coordinate of target boid
	vector2D ToDest;
	ToDest.X = Dest.X - BoidPosition.x;
	ToDest.Y = Dest.Y - BoidPosition.y; 
	
	for (int i = 0; i < ObsVertice.size(); i++){
		cv::Point A(ObsVertice[i].x, ObsVertice[i].y);
		cv::Point B(ObsVertice[(i + 1) % ObsVertice.size()].x, ObsVertice[(i + 1) % ObsVertice.size()].y);
		distance = distanceToLine(A, B, BoidPosition);
		if (distance <= Flock_radius * 1.5){
			cv::Point Intersect = findPerpendicularPoint(A, B, BoidPosition);
			if ((Intersect.x >= A.x && Intersect.x <= B.x) || (Intersect.x >= B.x && Intersect.x <= A.x)){  //If obstacle is in boid range
				if (distance < minDistance) {
					minDistance	=	distance;
					velocity.X = A.x - B.x;
					velocity.Y = A.y - B.y;
					perpendicular.X = BoidPosition.x - Intersect.x;
					perpendicular.Y = BoidPosition.y - Intersect.y;
					foundObstacle	= true;	
					vertice = i;
				}
			}
		}
	}
	
	if (foundObstacle){
		//std::cout << "\nObjects detected\n";
		velocity 		= 	vectNormalize(velocity);
		cv::Point tmp(Dest.X, Dest.Y);
		if ((Boids[boidNo].objectDetected() || tmp == Boids[boidNo].LastDestinationRecorded ) 
			&& Boids[boidNo].PreviousWall.X == velocity.X && Boids[boidNo].PreviousWall.Y == velocity.Y){
			velocity		=	vectMultiply(velocity, 1.5 * Boids[boidNo].WallDirection);
		}
		else {
			//std::cout << "\new wall detected\n";
			Boids[boidNo].setObjectDetect(true);
			Boids[boidNo].PreviousWall.X 	=	velocity.X;
			Boids[boidNo].PreviousWall.Y 	= 	velocity.Y;
			Boids[boidNo].LastDestinationRecorded.x = Dest.X;
			Boids[boidNo].LastDestinationRecorded.y = Dest.Y;
			
			velocity		=	vectMultiply(velocity, 1.5);
			if (abs(ToDest.X) > abs(ToDest.Y)){
				if (ToDest.X  * velocity.X < 0) {
					velocity = vectMultiply(velocity, -1);
					Boids[boidNo].WallDirection = -1;
				} else Boids[boidNo].WallDirection = 1;
			} else {
				if (ToDest.Y  * velocity.Y < 0) {
					velocity = vectMultiply(velocity, -1);
					Boids[boidNo].WallDirection = -1;
				} else Boids[boidNo].WallDirection = 1;
			}
		}
		perpendicular 	= 	vectNormalize(perpendicular); 
		if (minDistance != 0){
			perpendicular	= 	vectMultiply(perpendicular, 1.3 * Flock_radius/(minDistance));
		} else {
			perpendicular.X = -perpendicular.Y;
			perpendicular.Y = perpendicular.X;
			perpendicular 	= 	vectNormalize(perpendicular); 
			perpendicular	= 	vectMultiply(perpendicular, 2);
		}
		
		velocity.X		+=	perpendicular.X;
		velocity.Y		+=	perpendicular.Y;
		velocity 		= 	vectNormalize(velocity);
	} else {
		Boids[boidNo].setObjectDetect(false);
	}
	return velocity;
};








