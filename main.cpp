#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

#include <pigpio.h>

#include "miniPID.h"

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

#include "position.grpc.pb.h"

using namespace cv;
using namespace std;

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;

using position::OK;
using position::Position;
using position::PositionService;

#define CONNECTION_STRING   "orebron.com:50051"

#define MOTOR_BLACK   17
#define MOTOR_GREEN   27
#define MOTOR_RED     22
#define MOTOR_ORANGE  23

#define PLATE_INITIAL_STATE 750

#define BOUND 250

#define CAMERA_FPS      120
#define CAMERA_WIDTH    320
#define CAMERA_HEIGHT   240

#define CAMERA_HALF_WIDTH       ((uint16_t) (CAMERA_WIDTH) / (2))
#define CAMERA_HALF_HEIGHT      ((uint16_t) (CAMERA_HEIGHT) / (2))

#define CONSTANT_MAP_CAM_WIDTH      ((double) (BOUND) / (CAMERA_HALF_WIDTH))
#define CONSTANT_MAP_CAM_HEIGHT     ((double) (BOUND) / (CAMERA_HALF_HEIGHT))

MiniPID pid_x = MiniPID(1, 0, 1);
MiniPID pid_y = MiniPID(1, 0, 1);

void move_system(double x, double y);

void get_initial_state();

void system_check();

bool wantExit = false;

void stop(int signum) {
    printf("Caught signal %d\n", signum);
    wantExit = true;
}

const std::string black("\033[1;30m");
const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

int main(int argc, char **argv) {
    std::cout << "Hello, World!" << endl;

    // Configs
    cout << "PLATE INITIAL STATE: " << PLATE_INITIAL_STATE << endl;
    cout << "BOUND: " << BOUND << endl;

    cout << "CAMERA FPS: " << CAMERA_FPS << endl;

    cout << "CAMERA WIDTH: " << CAMERA_WIDTH << endl;
    cout << "CAMERA HEIGHT: " << CAMERA_HEIGHT << endl;

    cout << "CAMERA HALF WIDTH: " << CAMERA_HALF_WIDTH << endl;
    cout << "CAMERA HALF HEIGHT: " << CAMERA_HALF_HEIGHT << endl;

    cout << "CONSTANT MAP CAM WIDTH: " << CONSTANT_MAP_CAM_WIDTH << endl;
    cout << "CONSTANT MAP CAM HEIGHT: " << CONSTANT_MAP_CAM_HEIGHT << endl;

    if (gpioInitialise() < 0) {
        return EXIT_FAILURE;
    }

    // Signal Handler
    gpioSetSignalFunc(SIGINT, stop);

    gpioSetMode(MOTOR_BLACK, PI_OUTPUT);
    gpioSetMode(MOTOR_GREEN, PI_OUTPUT);
    gpioSetMode(MOTOR_RED, PI_OUTPUT);
    gpioSetMode(MOTOR_ORANGE, PI_OUTPUT);

    //get_initial_state();
    //system_check();
    get_initial_state();

    pid_x.setOutputLimits(-CAMERA_HALF_WIDTH, CAMERA_HALF_WIDTH);
    pid_y.setOutputLimits(-CAMERA_HALF_HEIGHT, CAMERA_HALF_HEIGHT);

    VideoCapture capture(0);
    if (!capture.isOpened())  // check if succeeded to connect to the camera
        CV_Assert("Cam open failed");

    cout << "CAP_PROP_FPS: " << capture.set(CAP_PROP_FPS, CAMERA_FPS) << endl;
    cout << "CAP_PROP_FRAME_WIDTH: " << capture.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH) << endl;
    cout << "CAP_PROP_FRAME_HEIGHT: " << capture.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT) << endl;

    auto channel = grpc::CreateChannel(CONNECTION_STRING, grpc::InsecureChannelCredentials());
    std::unique_ptr<PositionService::Stub> stub = PositionService::NewStub(channel);

    while (true) {
        Mat src, gray;
        vector<Vec3f> circles;

        capture >> src;
        cvtColor(src, gray, COLOR_BGR2GRAY);

        GaussianBlur(gray, gray, Size(7, 7), 1.5, 1.5);
        HoughCircles(gray, circles, HOUGH_GRADIENT, 1, CAMERA_WIDTH, 10, 20, 30, 50);

        double center_x = 0.0;
        double center_y = 0.0;

        if (!circles.empty()) {
            Vec3f c = circles[0];
            int16_t x_coordinate = c[0];
            int16_t y_coordinate = c[1];
            double radius = c[2];

            center_x = x_coordinate - CAMERA_HALF_WIDTH;
            center_y = y_coordinate - CAMERA_HALF_HEIGHT;

            Point center(cvRound(x_coordinate), cvRound(y_coordinate));

            // draw the circle center
            circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            // draw the circle outline
            circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);

            string label_x = " x = " + to_string(x_coordinate - CAMERA_HALF_WIDTH);
            string label_y = " y = " + to_string(y_coordinate - CAMERA_HALF_HEIGHT);

            putText(src, label_x, Point(CAMERA_HALF_WIDTH, CAMERA_HALF_HEIGHT), FONT_HERSHEY_PLAIN, 1.0,
                    CV_RGB(0, 255, 0), 2.0);
            putText(src, label_y, Point(CAMERA_HALF_WIDTH, CAMERA_HALF_HEIGHT + 20), FONT_HERSHEY_PLAIN, 1.0,
                    CV_RGB(0, 255, 0), 2.0);

        }

        cout << "\n" << "Position: " << center_x << "\t" << center_y << "\n";

        int16_t motor_axis_x_angle_val = pid_x.getOutput(center_x, 0.0);
        int16_t motor_axis_y_angle_val = pid_y.getOutput(center_y, 0.0);

        move_system(motor_axis_x_angle_val, motor_axis_y_angle_val);

        Position p;
        p.set_x(center_x * 10);
        p.set_y(center_y * 10);
        p.set_z(0.0);
        OK ok;
        ClientContext context;
        Status status = stub->NewPosition(&context, p, &ok);
        if (!status.ok()) {
            std::cout << "RPC Failure: " << status.error_message()
                      << ":" << status.error_details() << std::endl;
        }

        // Window
        //namedWindow("Grayscale", WINDOW_AUTOSIZE);
        //imshow("Grayscale", src);
        //waitKey(1);

        // Flush stdout
        std::cout << "\r\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F" << std::flush;

        if (wantExit) break;
    }

    cout << endl;

    // Release video
    capture.release();

    return EXIT_SUCCESS;
}

/**
 * O
 * @param x
 * @param y
 */
void move_system(double x, double y) {
    const int16_t X = x * CONSTANT_MAP_CAM_WIDTH;
    const int16_t Y = y * CONSTANT_MAP_CAM_HEIGHT;

    // Engines
    cout << "\n" << "ENGINES POWER-------" << "\n"
         << "\t" << green << PLATE_INITIAL_STATE + Y << "\t\n"
         << black << PLATE_INITIAL_STATE - X << "\t\t" << yellow << PLATE_INITIAL_STATE + X << "\n"
         << "\t" << red << PLATE_INITIAL_STATE - Y << "\t\n"
         << reset << "--------------------" << "\n";

    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - X);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + X);

    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + Y);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - Y);
}

void get_initial_state() {
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE);
}

void system_check() {
    time_sleep(2);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + BOUND);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + BOUND);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - BOUND);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - BOUND);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + BOUND);
    // Three down, One up
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - BOUND);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - BOUND);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE - BOUND);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + BOUND);
    time_sleep(1);
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE + BOUND);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + BOUND);
    time_sleep(2);
}
