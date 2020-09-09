#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <cstdio>

#include <pigpio.h>

#include "miniPID.h"

using namespace cv;
using namespace std;

#define KNOWN_DISTANCE 18.0
#define KNOWN_RADIUS 4.0

#define MOTOR_BLACK   17
#define MOTOR_GREEN   27
#define MOTOR_RED     22
#define MOTOR_ORANGE  23

#define PLATE_INITIAL_STATE 750

#define LIMIT 250

#define CAMERA_FPS	90
#define CAMERA_WIDTH	640
#define CAMERA_HEIGHT	480

MiniPID pid_x = MiniPID(10,0,0);
MiniPID pid_y = MiniPID(10,0,0);

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

void system_check();
void move_system(double x, double y);

int main(int argc, char** argv)
{
  if (gpioInitialise() < 0) {
    return -1;
  }

  gpioSetMode(MOTOR_BLACK, PI_OUTPUT);
  gpioSetMode(MOTOR_GREEN, PI_OUTPUT);
  gpioSetMode(MOTOR_RED, PI_OUTPUT);
  gpioSetMode(MOTOR_ORANGE, PI_OUTPUT);

  ////system_check();

  pid_x.setOutputLimits(-LIMIT,LIMIT);
  pid_x.setOutputRampRate(25);

  pid_y.setOutputLimits(-LIMIT,LIMIT);
  pid_y.setOutputRampRate(25);


  VideoCapture cap(0);
  if (!cap.isOpened())  // check if succeeded to connect to the camera
    CV_Assert("Cam open failed");

  cap.set(3,CAMERA_WIDTH);
  cap.set(4,CAMERA_HEIGHT);

  while (waitKey(30) != 'q') {
		double fps = cap.get(CAP_PROP_FPS);
    time_t start, end;
    time(&start);
    for(int i = 0; i < CAMERA_FPS; i++)
    {  
      Mat src, gray;
      vector<Vec3f> circles;

      cap >> src;
      cvtColor(src, gray, COLOR_BGR2GRAY);

      //medianBlur(gray, gray, 5);
      GaussianBlur( gray, gray, Size(9,9), 2, 2);

      HoughCircles(gray, circles, HOUGH_GRADIENT, 1, gray.rows / 16, 200, 50, 0, 0);
      Vec3i c;

      if (0 < circles.size())
      {
        c = circles[0]; 

        Point center = Point(c[0], c[1]); 

        x_coordinate = (double)c[0] - CAMERA_WIDTH / 2;
        y_coordinate = (double)c[1] - CAMERA_HEIGHT / 2;

        double out_x = pid_x.getOutput(x_coordinate,0.0);
        double out_y = pid_y.getOutput(y_coordinate,0.0);

        move_system(out_x, out_y);


        cout << endl << x_coordinate << ": " << out_x << endl << y_coordinate << ": " << out_y << endl;

        circle(src, center, 1, Scalar(0, 100, 100), 3, LINE_AA); 

        radius = (double)c[2];

        circle(src, center, (int)radius, Scalar(255, 0, 255), 3, LINE_AA);

        z_coordinate = (KNOWN_RADIUS * focalLength * gray.size().height) / (radius * image_height);


        //float hit_time = distance / (focalLength * (radius * (gray.size().height / image_height)) / distance);

        string label = " x = " + to_string(x_coordinate);
        string label_2 = " y = " + to_string(y_coordinate);
        string label_3 = " z = " + to_string(z_coordinate);
        //string label_4 = " hit_time = " + to_string(hit_time);

        putText(src, label, Point(x_coordinate, y_coordinate), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
        putText(src, label_2, Point(x_coordinate, y_coordinate + 20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
        putText(src, label_3, Point(x_coordinate, y_coordinate + 40), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
        //putText(src, label_4, Point(x_coorinate, y_coorinate + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);

      }

      imshow("window", src);
    }
    time(&end);
    double seconds = difftime(end, start);

    // Calculate frames per second
    fps = CAMERA_FPS / seconds;
    cout << "Estimated frames per second : " << fps << endl;

  }

  // Release video
  cap.release();

  return 0;
}


void move_system(double x, double y){
  cout << "Engines: " << endl << "Black: " << PLATE_INITIAL_STATE - x << endl << "Orange: " << PLATE_INITIAL_STATE + x << endl << "Green: " << PLATE_INITIAL_STATE + y << endl << "Red: " <<  PLATE_INITIAL_STATE - y << endl;

  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - x);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + x);

  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + y);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - y);
  //time_sleep(2);
}

void system_check() {
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE);
  time_sleep(2);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + 250);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE);
  time_sleep(1);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE + 250);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE);
  time_sleep(1);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + 250);
  time_sleep(1);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE + 250);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - 250);
  time_sleep(1);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE + 250);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + 250);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - 250);
  time_sleep(1);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE + 250);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + 250);
  // Three down, One up
  time_sleep(1);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE + 250);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - 250);
  time_sleep(1);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + 250);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - 250);
  time_sleep(1);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE + 250);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - 250);
  time_sleep(1);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - 250);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + 250);
  time_sleep(2);
  gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_RED, PLATE_INITIAL_STATE);
  gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE);
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
