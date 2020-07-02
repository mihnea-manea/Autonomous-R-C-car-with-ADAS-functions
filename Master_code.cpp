#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;

//Image processing variables
Mat frameBGR, frame, Matrix, framePerspective, frameGray, frameThreshold, frameEdge, frameFinal,frameFinalDuplicate;
Mat ROIline;
int LeftLanePos, RightLanePos, frameCenter, laneCenter, result;

RaspiCam_Cv Camera;

stringstream ss, ss1, ss2;

vector<int> histrogramLane;

Point2f Source[] = {Point2f(38,160), Point2f(382,160), Point2f(5,210), Point2f(415,210)};//definirea puctelor pt aria de interes
Point2f Destination[] = {Point2f(80,0), Point2f(280,0), Point2f(80,240), Point2f(280,240)};


//Machine learning variables
CascadeClassifier Stop_Sign_Cascade, Object_Cascade;
Mat frame_stop, RoI_Stop, gray_Stop, frame_object, RoI_Object, gray_Object;
vector <Rect> Stop, Object;
int dist_stop, dist_object; 


void Setup ( int argc,char **argv, RaspiCam_Cv &Camera )
  {
    Camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,420 ) );
    Camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    Camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    Camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    Camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    Camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    Camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,50 ));

}

void Perspective() //uneste punctele si creaza o perspectiva, Scalar() defineste culoarea liniei cu care unesc punctele
{
	line(frame, Source[0], Source[1], Scalar(0,0,255), 2);
	line(frame, Source[1], Source[3], Scalar(0,0,255), 2);
	line(frame, Source[3], Source[2], Scalar(0,0,255), 2);
	line(frame, Source[2], Source[0], Scalar(0,0,255), 2);
	/*
	line(frame, Destination[0], Destination[1], Scalar(0,255,0), 2);
	line(frame, Destination[1], Destination[3], Scalar(0,255,0), 2);
	line(frame, Destination[3], Destination[2], Scalar(0,255,0), 2);
	line(frame, Destination[2], Destination[0], Scalar(0,255,0), 2);
	*/
	Matrix = getPerspectiveTransform(Source, Destination); //transforma perspectiva sursa in perspectiva destinatie
	
	warpPerspective(frame,framePerspective, Matrix, Size(400,240));//transformare perspectiva din frame ul original
	
}

void Capture()  // functia de capturare video
{
	Camera.grab();
	Camera.retrieve(frameBGR);
	cvtColor(frameBGR, frame, COLOR_BGR2RGB);//conversie intre spatiul de culori default bgr la rgb
	cvtColor(frameBGR, frame_stop, COLOR_BGR2RGB);
	cvtColor(frameBGR, frame_object, COLOR_BGR2RGB);
	
	
}

void Threshold() //aplicam o tranformare a imagini pt a obtine doar tonuri de gri si in final doar o imagine cu alb si negru
{
	cvtColor(framePerspective, frameGray, COLOR_RGB2GRAY);
	inRange(frameGray,150, 255,frameThreshold);
	Canny(frameGray, frameEdge, 100, 500, 3, false); //canny edge detetion function
	add(frameThreshold, frameEdge, frameFinal);//merging the black and white image with the canny edge detection image
	cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);//nu se poate converti dein gray to color, o folosesc pt altceva
	cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);//folosesc la histograma petru regiunea de interes, ca sa nu alterez imaginea finala (sa nu mi se taie partea de jos)
}


void Histrogram()
{
	histrogramLane.resize(360); //make it 400 length
	histrogramLane.clear();
	
	for(int i=0; i<360; i++) //it gives the width of the frame wich was changed to 400 for easier computations
	{
		ROIline  = frameFinalDuplicate(Rect(i,140,1,100)); //process the 400 tiny strips from reason of interest(ROI)
		divide(255, ROIline, ROIline);
		histrogramLane.push_back((int)(sum(ROIline)[0]));// fac suma pixelilor
		
	}
}

void LaneFinder()
{
	vector<int>:: iterator LeftPtr;
	LeftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 130); //caut prima linie in histograma, parcurg histograma pana aproape de jumate
	LeftLanePos = distance(histrogramLane.begin(), LeftPtr);
	
	vector<int>:: iterator RightPtr;
	RightPtr = max_element(histrogramLane.begin() + 230, histrogramLane.end()); //cauta doua linie in histograma, parcurg histograma de la pozitia 250 pana la capat
	RightLanePos = distance(histrogramLane.begin(), RightPtr);
	
	line(frameFinal, Point2f(LeftLanePos, 0), Point2f(LeftLanePos, 240), Scalar(0,255,0), 2); //tragem linie pt prima pozitie
	line(frameFinal, Point2f(RightLanePos, 0), Point2f(RightLanePos, 240), Scalar(0,255,0), 2); //tragem linie pt ultima pozitie
}

void LaneCenter()
{
	laneCenter = (RightLanePos-LeftLanePos)/2 + LeftLanePos; //calculez mijlocul dintre cele doua linii
	frameCenter = 180; //mijlocul cadrului DE CALIBRAT SA SE SUPRAPUNA PESTE MIJLOCUL CELOR DOUA LINII PE TRASEUL FINAL
	
	line(frameFinal, Point2f(laneCenter, 0), Point2f(laneCenter, 240), Scalar(0,255,0), 3); //desenarea liniei de la mijlocul celor doua linii marginale cu linie verde
	line(frameFinal, Point2f(frameCenter, 0), Point2f(frameCenter, 240), Scalar(255,0,0), 3); //desenarea liniei de la mijlocul frame-ului cu linie albastra
	
	result = laneCenter-frameCenter;
}


void StopSignDetection()
{
	if(!Stop_Sign_Cascade.load("//home//pi//Desktop//Integrare machine learning//Stop_cascade5.xml"))
	{
		printf("Unable to open stop cascade file");
	}
	
	RoI_Stop = frame_stop(Rect(210,0,210,240));
	cvtColor(RoI_Stop, gray_Stop, COLOR_RGB2GRAY);
	cvtColor(RoI_Stop, RoI_Stop, COLOR_BGR2RGB);
	equalizeHist( gray_Stop, gray_Stop); //equlalizes all the intesities of the garyscale image
	Stop_Sign_Cascade.detectMultiScale(gray_Stop, Stop);  //function in cascade class detecting stops and puttting them in vector Stop
	
	for(int i=0; i<Stop.size(); i++)
	{
		Point p1(Stop[i].x, Stop[i].y);
		Point p2(Stop[i].x + Stop[i].width, Stop[i].y + Stop[i].height); //use the points to draw a rectangle aropund the stop sign
		
		rectangle(RoI_Stop, p1, p2, Scalar(0, 0 ,255), 2);
		putText(RoI_Stop, "Stop Sign", p1, FONT_HERSHEY_PLAIN,1 , Scalar(0 , 0, 255), 2);
		
		dist_stop = (-0.8423) * (p2.x - p1.x) + 77.647; //ecuatia liniara pt obtinerea distantei pana la semnul stop
		ss1.str(" ");
		ss1.clear();
		ss1<<"D="<<dist_stop<<"cm"; //calculez distanta calculand marimea latimii in pixeli a dreptunghiului care contine semnul stop --> calculez distanta dintre masina si stop folosind ecuatii liniare --> s eintroduc m si c in ecuatia de mai sus
		putText(RoI_Stop, ss1.str(), Point2f(1,130), 0, 1, Scalar(0,0,255), 2);
		
	}
	
}



void ObjectDetection()
{
	if(!Object_Cascade.load("//home//pi//Desktop//Integrare machine learning//Object_cascade.xml"))
	{
		printf("Unable to open object cascade file");
	}
	
	RoI_Object = frame_object(Rect(0,0,420,240));
	cvtColor(RoI_Object, gray_Object, COLOR_RGB2GRAY);
	cvtColor(RoI_Object, RoI_Object, COLOR_BGR2RGB);
	equalizeHist( gray_Object, gray_Object); //equlalizes all the intesities of the garyscale image
	Object_Cascade.detectMultiScale(gray_Object, Object);  //function in cascade class detecting stops and puttting them in vector Stop
	
	for(int i=0; i<Object.size(); i++)
	{
		Point p1(Object[i].x, Object[i].y);
		Point p2(Object[i].x + Object[i].width, Object[i].y + Object[i].height); //use the points to draw a rectangle aropund the stop sign
		
		rectangle(RoI_Object, p1, p2, Scalar(0, 0 ,255), 2);
		putText(RoI_Object, "Object", p1, FONT_HERSHEY_PLAIN,1 , Scalar(0 , 0, 255), 2);
		
		dist_object = (-0.8423) * (p2.x - p1.x) + 77.647; //ecuatia liniara pt obtinerea distantei pana la semnul stop
		ss2.str(" ");
		ss2.clear();
		ss2<<"D="<<dist_object<<"cm"; //calculez distanta calculand marimea latimii in pixeli a dreptunghiului care contine obiectul --> calculez distanta dintre masina si stop folosind ecuatii liniare --> s eintroduc m si c in ecuatia de mai sus
		putText(RoI_Object, ss2.str(), Point2f(1,130), 0, 1, Scalar(0,0,255), 2);
		
	}
	
}





int main(int argc, char **argv)
{	
		using namespace std::this_thread;     // sleep_for, sleep_until
		using namespace std::chrono_literals; //s
		wiringPiSetup(); //folosim lbraria wiring pi
		pinMode(21, OUTPUT);
		pinMode(22, OUTPUT);
		pinMode(23, OUTPUT);
		pinMode(24, OUTPUT);
		Setup(argc, argv, Camera);
		cout<<"Connecting to camera"<<endl;
		if(!Camera.open())
		{
			cout<<"Failed to connect"<<endl;
			return -1;
		}
		cout<<"Camera Id = "<<Camera.getId()<<endl;
		
		while(1)
		{
			auto start = std::chrono::system_clock::now();//masurarea timpului dintre frame uri respectiv fps
			
			Capture();
			Perspective();
			Threshold();
			Histrogram();
			LaneFinder();
			LaneCenter();
			StopSignDetection();
			ObjectDetection();
			
			if(dist_stop>5 && dist_stop < 20) //first condition put because if there is no stop sign the dist_stop variable will be 0
			{
					digitalWrite(21, 1);
					digitalWrite(22, 1);
					digitalWrite(23, 1);   //send 7
					digitalWrite(24, 0);
					cout<<"Stop Sign"<<endl;
					dist_stop = 0;
					sleep_for(8s);
					digitalWrite(21, 0);                    //sto psign detection
					digitalWrite(22, 0);
					digitalWrite(23, 0);   //send 0
					digitalWrite(24, 0);
					cout<<"Forward"<<endl;
					sleep_for(500ms);
					
					goto Stop_Sign;
			}
			
			
			if(dist_object>5 && dist_object < 20) //first condition put because if there is no stop sign the dist_stop variable will be 0
			{
					digitalWrite(21, 1);
					digitalWrite(22, 1);
					digitalWrite(23, 1);   //send 7    obstacle detection
					digitalWrite(24, 0);
					cout<<"Obstacle"<<endl;
					dist_stop = 0;
					
					
					goto Obstacle;
			}
			
			
			
			
			if(result >= -10 && result <= 10)
			{
					digitalWrite(21, 0);
					digitalWrite(22, 0);
					digitalWrite(23, 0);   //send 0
					digitalWrite(24, 0);
					cout<<"Forward"<<endl;
				
			}
			
			/*else if(result > 5 && result < 10)
			{
					digitalWrite(21, 1);
					digitalWrite(22, 0); //send 1
					digitalWrite(23, 0);
					digitalWrite(24, 0);
					cout<<"Right1"<<endl;
				
			}*/
			
			else if(result >10 && result < 15)
			{
					digitalWrite(21, 0);
					digitalWrite(22, 1); //send 2
					digitalWrite(23, 0);
					digitalWrite(24, 0);
					cout<<"Right2"<<endl;
				
			}
			
			else if(result >= 15)
			{
					digitalWrite(21, 1);
					digitalWrite(22, 1); //send 3
					digitalWrite(23, 0);
					digitalWrite(24, 0);
					cout<<"Right3"<<endl;
				
			}
			/*else if(result < -5 && result > -10)
			{
					digitalWrite(21, 0);
					digitalWrite(22, 0); //send 4
					digitalWrite(23, 1);
					digitalWrite(24, 0);
					cout<<"Left1"<<endl;
				
			}*/
			
			else if(result < -10 && result > -15)
			{
					digitalWrite(21, 1);
					digitalWrite(22, 0); //send 5
					digitalWrite(23, 1);
					digitalWrite(24, 0);
					cout<<"Left2"<<endl;
				
			}
			
			else if(result <= -15)
			{
					digitalWrite(21, 0);
					digitalWrite(22, 1); //send 6
					digitalWrite(23, 1);
					digitalWrite(24, 0);
					cout<<"Left3"<<endl;
				
			}
			
			Stop_Sign:
			Obstacle:
			
			ss.str(" ");
			ss.clear();
			ss<<"Result = "<<result; 
			putText(framePerspective, ss.str(), Point2f(1,50), 0,1,Scalar(150,0,150), 2);//Scrie rezultul diferentei dintre mijloc si mijlocul liniilor pe img
			
			namedWindow("Video", WINDOW_KEEPRATIO);
			moveWindow("Video", 80, 100);
			resizeWindow("Video", 640, 480);
			imshow("Video", frameBGR); //if it does't work change all the frameBGR names to frame --> made this to see the true colors when we run the program
			
			namedWindow("Perspective", WINDOW_KEEPRATIO);
			moveWindow("Perspective", 730, 100);
			resizeWindow("Perspective", 640, 480);
			imshow("Perspective", framePerspective);
			
			namedWindow("FinalImage", WINDOW_KEEPRATIO);
			moveWindow("FinalImage", 80, 590);
			resizeWindow("FinalImage", 640, 480);
			imshow("FinalImage", frameFinal);
			
			namedWindow("Stop Sign", WINDOW_KEEPRATIO);
			moveWindow("Stop Sign", 730, 590);
			resizeWindow("Stop Sign", 640, 480);
			imshow("Stop Sign", RoI_Stop);
			
			namedWindow("Object", WINDOW_KEEPRATIO);
			moveWindow("Object", 1300, 100);
			resizeWindow("Object", 640, 480);
			imshow("Object", RoI_Object);

			
			waitKey(1);
			
			auto end = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_seconds = end-start;
    
			float t = elapsed_seconds.count();
			int FPS = 1/t;
			cout<<"FPS = "<<FPS<<endl;
			
		}
		return 0;
}
