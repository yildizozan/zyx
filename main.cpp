#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cstdio>
#include <wiringPi.h>
#include <softPwm.h>

#define KNOWN_DISTANCE 18.0
#define KNOWN_RADIUS 4.0

using namespace cv;
using namespace std;

#define motorBlack	11
#define motorGreen	13
#define motorRed	15
#define motorOrange	16

void find_focalLength_imgHeight();

double getCircle_x_coordinate();
double getCircle_y_coordinate();
double getCircle_z_coordinate();
double getCircleRadius();

double x_coordinate = 0.0;
double y_coordinate = 0.0;
double z_coordinate = 0.0;
double radius = 0.0;

double focalLength;
double image_height;

void find_focalLength_imgHeight()
{

    string image_path = samples::findFile("1.jpeg");
    Mat src = imread(image_path, IMREAD_COLOR);

    if (src.empty()) {
        printf(" Error opening image\n");
        return;
    }

    Mat gray;
    
    cvtColor(src, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    
    vector<Vec3f> circles;
    
    HoughCircles(gray, circles, HOUGH_GRADIENT, 1, gray.rows / 16, 100, 30, 1, 500 );
 
    Vec3i c = circles[0];
    Point center = Point(c[0], c[1]);

    float radius = c[2];

    image_height = gray.size().height;
    
    focalLength = (KNOWN_DISTANCE * radius) / KNOWN_RADIUS;
    
    cout << src.elemSize() << endl;
    cout << typeid(src.data[0]).name() << endl;
    
}

void getCircleProporties(VideoCapture cap) {
    //VideoCapture cap(0);


    //int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH)); //get the width of frames of the video
    //int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT)); //get the height of frames of the video

    //Size frame_size(frame_width, frame_height);
    //int frames_per_second = 5;


    //VideoWriter oVideoWriter("MyVideo.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), frames_per_second, frame_size, true);

    while (waitKey(30) != 'q') {

        Mat src;
        Mat gray;
        vector<Vec3f> circles;

        cap >> src;

        cvtColor(src, gray, COLOR_BGR2GRAY);

        medianBlur(gray, gray, 5);

        HoughCircles(gray, circles, HOUGH_GRADIENT, 1, gray.rows / 16, 100, 30, 1, 200);


        for (size_t i = 0; i < circles.size(); i++)
        {
            Vec3i c = circles[i]; Point center = Point(c[0], c[1]); x_coordinate = (double)c[0]; y_coordinate = (double)c[1]; circle(src, center, 1, Scalar(0, 100, 100), 3, LINE_AA); 
            radius = (double)c[2];

            circle(src, center, (int)radius, Scalar(255, 0, 255), 3, LINE_AA);

            z_coordinate = (KNOWN_RADIUS * focalLength * gray.size().height) / (radius * image_height);


            //float hit_time = distance / (focalLength * (radius * (gray.size().height / image_height)) / distance);


            string label = " x = " + to_string(x_coordinate - 316);
            string label_2 = " y = " + to_string(y_coordinate - 316);
            string label_3 = " z = " + to_string(z_coordinate);
            //string label_4 = " hit_time = " + to_string(hit_time);

            putText(src, label, Point(x_coordinate, y_coordinate), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
            putText(src, label_2, Point(x_coordinate, y_coordinate + 20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
            putText(src, label_3, Point(x_coordinate, y_coordinate + 40), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
            //putText(src, label_4, Point(x_coorinate, y_coorinate + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);

        }

        //oVideoWriter.write(src);



        imshow("window", src);


    }
    
}

int main(int argc, char** argv)
{
  if(wiringPiSetup() == -1) {
    cout << "Setup wiring pi failed";
    return 1;
  }	

  find_focalLength_imgHeight();

  VideoCapture cap(0);

  getCircleProporties(cap);
  
  return 0;
}


double getCircle_x_coordinate()
{
    return x_coordinate;
}

double getCircle_y_coordinate()
{
    return y_coordinate;
}

double getCircle_z_coordinate()
{
    return z_coordinate;
}

double getCircleRadius()
{
    return radius;
}
