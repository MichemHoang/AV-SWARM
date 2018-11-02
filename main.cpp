#include "Swarm.h"
#include "typedefs.h"
#include <unistd.h>

using namespace std;
using namespace cv;

cv::Point Destination;
double Dest_X, Dest_Y;
bool RClicked 	= false;
bool LClicked 	= false;
bool LDBClicked = false;

void event_Handler(int event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_LBUTTONDOWN) {
        cout << "LEFT : (" << x << ", " << y << ")" << endl;
        LClicked = true;
        Dest_X = x;
        Dest_Y = y;
    }
    
    if (event == EVENT_RBUTTONDOWN) {
        cout << "RIGHT : (" << x << ", " << y << ")" << endl;
        RClicked = true;
        Dest_X = x;
        Dest_Y = y;
    }
    if (event == EVENT_LBUTTONDBLCLK){
		LDBClicked = true;
	}
}

int main()
{
	SWARM Michem(0, DEFAULT::RADIUS, 22, FINDSWARM);
    cv::Mat3b Canvas(DEFAULT::MAX_X, DEFAULT::MAX_Y, Vec3b(0, 0, 0));
    namedWindow("AV_SWARM");
    setMouseCallback("AV_SWARM", event_Handler);
	
	cv::Point Obj[5];
	cv::Point A0(300, 300);
	cv::Point A1(260, 360);
	cv::Point A2(280, 440);
	cv::Point A3(400, 310);
	cv::Point A4(490, 377);
	Obj[0] = A0; Obj[1] = A1; Obj[2] = A2; Obj[4] = A3; Obj[3] = A4;
	cv::Point *Obstacle = Obj;
	Michem.setObstacleVertices(Obj);
	
	/*
	BOID TestBoid(384, 455);
	Michem.addBoid(TestBoid);
	Michem.setDestination(167, 262);
	*/
	
	do {
		std::vector<BOID> AVs = Michem.getBoids();
		Canvas.setTo(Scalar(70, 70, 70));
		fillConvexPoly(Canvas, Obstacle, 5, cv::Scalar(134, 136, 147), 8, 0);
		for (int i = 0; i < AVs.size(); i++){
			cv::Point BoidPosition;
			BoidPosition = AVs[i].getPosition();
			vector2D Direction = AVs[i].getVelocity();
			circle(Canvas, BoidPosition, DEFAULT::RADIUS, cv::Scalar(255, 0, 255), 1, 8, 0);
			circle(Canvas, BoidPosition, 4, cv::Scalar(100 + i * 15, 100 + i * 15, 100 + i * 15), 1, 8, 0);
			line(Canvas, BoidPosition, cv::Point((BoidPosition.x + Direction.X * 15) , (BoidPosition.y + Direction.Y * 15)), cv::Scalar(255,191,0), 1, 8, 0);
			imshow("AV_SWARM", Canvas);
		}
		if (RClicked == true){
			RClicked = false;
			Michem.setDestination(Dest_X, Dest_Y);
		} else if (LClicked == true){
			LClicked = false;
			Michem.setScatterMode();
		} else if (LDBClicked == true){
			cv::waitKey();
			LDBClicked = false;
		}
		Michem.update();
	} while (((cv::waitKey(DEFAULT::FRAME_RATE) & KEY_MASK) != DEFAULT::EXIT_KEY));

    return 0;
}
