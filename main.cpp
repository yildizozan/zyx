#include <iostream>
#include <thread>
#include <csignal>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pigpio.h>

#include "miniPID.h"

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

#include "position.grpc.pb.h"

using namespace std;
using namespace cv;

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

#define SCALAR_SCALE_CLIENT_APPS    10

#define MOTOR_BLACK   17
#define MOTOR_GREEN   27
#define MOTOR_RED     22
#define MOTOR_ORANGE  23

#define PLATE_INITIAL_STATE 750

#define BOUND 100

#define CAMERA_FPS      120
#define CAMERA_WIDTH    320
#define CAMERA_HEIGHT   240

#define CAMERA_HALF_WIDTH       ((uint16_t) (CAMERA_WIDTH) / (2))
#define CAMERA_HALF_HEIGHT      ((uint16_t) (CAMERA_HEIGHT) / (2))

#define CONSTANT_MAP_CAM_WIDTH      ((double) (BOUND) / (CAMERA_HALF_WIDTH))
#define CONSTANT_MAP_CAM_HEIGHT     ((double) (BOUND) / (CAMERA_HALF_HEIGHT))

#define KP  0.0
#define KI  0.0
#define KD  0.0

MiniPID pid_green = MiniPID(KP, KI, KD);
MiniPID pid_red = MiniPID(KP, KI, KD);
MiniPID pid_black = MiniPID(KP, KI, KD);
MiniPID pid_orange = MiniPID(KP, KI, KD);

const std::string black("\033[1;30m");
const std::string red("\033[0;31m");
const std::string green("\033[1;32m");
const std::string yellow("\033[1;33m");
const std::string cyan("\033[0;36m");
const std::string magenta("\033[0;35m");
const std::string reset("\033[0m");

/**
 * Global Allocations
 */
Mat frame;
std::unique_ptr<PositionService::Stub> stub;

bool stop = false;

void signal_handler(int signum) {
    stop = true;
}

void ThreadSendData(Position p) {
    OK ok;
    ClientContext context;
    Status status = stub->NewPosition(&context, p, &ok);
    /*
    if (!status.ok()) {
        std::cout << "RPC Failure: " << status.error_message() << ":" << status.error_details() << std::endl;
    }
     */
}

void ThreadEngineControl(double x, double y) {
    //const int16_t X = x * CONSTANT_MAP_CAM_WIDTH;
    //const int16_t Y = y * CONSTANT_MAP_CAM_HEIGHT;

    int16_t motor_green_angle_val = pid_green.getOutput(y, 0.0);
    int16_t motor_red_angle_val = pid_red.getOutput(y, 0.0);

    int16_t motor_black_angle_val = pid_black.getOutput(x, 0.0);
    int16_t motor_orange_angle_val = pid_orange.getOutput(x, 0.0);

    // Engines
    /*
    cout << "\n" << "ENGINES POWER-------" << "\n"
         << "\t" << green << PLATE_INITIAL_STATE + motor_green_angle_val << "\t\n"
         << black << PLATE_INITIAL_STATE - motor_black_angle_val << "\t\t" << yellow
         << PLATE_INITIAL_STATE + motor_orange_angle_val << "\n"
         << "\t" << red << PLATE_INITIAL_STATE - motor_red_angle_val << "\t\n"
         << reset << "--------------------" << "\n";
    */
    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - motor_black_angle_val);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + motor_orange_angle_val);

    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + motor_green_angle_val);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - motor_red_angle_val);
}

void ThreadEngineControl2(double x, double y) {
    int16_t motor_green_angle_val = pid_green.getOutput(-y, 0.0) * CONSTANT_MAP_CAM_HEIGHT;
    int16_t motor_red_angle_val = pid_red.getOutput(y, 0.0) * CONSTANT_MAP_CAM_HEIGHT;

    int16_t motor_black_angle_val = pid_black.getOutput(-x, 0.0) * CONSTANT_MAP_CAM_WIDTH;
    int16_t motor_orange_angle_val = pid_orange.getOutput(x, 0.0) * CONSTANT_MAP_CAM_WIDTH;

    // Engines
    cout << "\n" << "ENGINES POWER-------" << "\n"
         << "\t" << green << PLATE_INITIAL_STATE + motor_green_angle_val << "\t\n"
         << black << PLATE_INITIAL_STATE - motor_black_angle_val << "\t\t" << yellow
         << PLATE_INITIAL_STATE + motor_orange_angle_val << "\n"
         << "\t" << red << PLATE_INITIAL_STATE - motor_red_angle_val << "\t\n"
         << reset << "--------------------" << "\n";

    gpioServo(MOTOR_BLACK, PLATE_INITIAL_STATE - motor_black_angle_val);
    gpioServo(MOTOR_ORANGE, PLATE_INITIAL_STATE + motor_orange_angle_val);

    gpioServo(MOTOR_GREEN, PLATE_INITIAL_STATE + motor_green_angle_val);
    gpioServo(MOTOR_RED, PLATE_INITIAL_STATE - motor_red_angle_val);
}

void ThreadCaptureFrames(VideoCapture capture) {
    while (!stop) {
        capture >> frame;
    }
}

void ThreadProcessFrames() {

    while (!stop) {
        Mat gray;

        if (!frame.empty()) {
            double center_x = 0.0;
            double center_y = 0.0;

            vector<Vec3f> circles;

            cvtColor(frame, gray, COLOR_BGR2GRAY);

            GaussianBlur(gray, gray, Size(7, 7), 1.5, 1.5);
            HoughCircles(gray, circles, HOUGH_GRADIENT, 1, CAMERA_WIDTH, 60, 30, 12, 160);
            if (!circles.empty()) {
                Vec3f c = circles[0];
                int16_t x_coordinate = c[0];
                int16_t y_coordinate = c[1];
                double radius = c[2];

                center_x = x_coordinate - CAMERA_HALF_WIDTH;
                center_y = y_coordinate - CAMERA_HALF_HEIGHT;

                Point center(cvRound(x_coordinate), cvRound(y_coordinate));

                // draw the circle center
                circle(frame, center, 3, Scalar(0, 255, 0), -1, 8, 0);
                // draw the circle outline
                circle(frame, center, radius, Scalar(0, 0, 255), 3, 8, 0);

                string label_x = " x = " + to_string(x_coordinate - CAMERA_HALF_WIDTH);
                string label_y = " y = " + to_string(y_coordinate - CAMERA_HALF_HEIGHT);

                putText(frame, label_x, Point(CAMERA_HALF_WIDTH, CAMERA_HALF_HEIGHT), FONT_HERSHEY_PLAIN, 1.0,
                        CV_RGB(0, 255, 0), 2.0);
                putText(frame, label_y, Point(CAMERA_HALF_WIDTH, CAMERA_HALF_HEIGHT + 20), FONT_HERSHEY_PLAIN, 1.0,
                        CV_RGB(0, 255, 0), 2.0);

                // Window
                // imshow("Original", frame);
                //waitKey(1);
            }

            //cout << "\n" << "Position: " << center_x << "\t" << center_y << "\n";

            Position p;
            p.set_x(center_x * SCALAR_SCALE_CLIENT_APPS);
            p.set_y(center_y * SCALAR_SCALE_CLIENT_APPS);
            p.set_z(0.0);

            thread thread_engine_control(ThreadEngineControl, center_x, center_y);
            thread thread_send_data(ThreadSendData, p);

            thread_engine_control.join();
            thread_send_data.join();

        }

        // Flush stdout!
        //std::cout << "\r\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F" << std::flush;
        //std::cout << "\r\033[F\033[F\033[F\033[F\033[F\033[F" << std::flush;
    }
}

int main(int argc, char **argv) {
    const String keys =
            "{help h usage ? |      | print this message    }"
            "{@Kp            |<none>| Proportional value    }"
            "{@Ki            |<none>| Integral value        }"
            "{@Kd            |<none>| Derivative            }";
    CommandLineParser parser(argc, argv, keys);
    parser.about("Hello world! Zyx software v0.2.0");
    if (parser.has("help")) {
        parser.printMessage();
        return EXIT_FAILURE;
    }
    double Kp = parser.get<double>("@Kp");
    double Ki = parser.get<double>("@Ki");
    double Kd = parser.get<double>("@Kd");
    if (!parser.check()) {
        parser.printErrors();
        return EXIT_SUCCESS;
    }

    /*
    std::cout << "Hello, World!" << std::endl;

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
    */

    if (gpioInitialise() < 0) {
        return EXIT_FAILURE;
    }

    // Signal Handler
    gpioSetSignalFunc(SIGINT, signal_handler);
    gpioSetSignalFunc(SIGTERM, signal_handler);

    gpioSetMode(MOTOR_BLACK, PI_OUTPUT);
    gpioSetMode(MOTOR_GREEN, PI_OUTPUT);
    gpioSetMode(MOTOR_RED, PI_OUTPUT);
    gpioSetMode(MOTOR_ORANGE, PI_OUTPUT);

    pid_green = MiniPID(Kp, Ki, Kd);
    pid_red = MiniPID(Kp, Ki, Kd);
    pid_black = MiniPID(Kp, Ki, Kd);
    pid_orange = MiniPID(Kp, Ki, Kd);

    pid_green.setOutputLimits(-BOUND, BOUND);
    pid_green.setSetpointRange(CAMERA_HALF_HEIGHT);

    pid_red.setOutputLimits(-BOUND, BOUND);
    pid_red.setSetpointRange(CAMERA_HALF_HEIGHT);

    pid_black.setOutputLimits(-BOUND, BOUND);
    pid_black.setSetpointRange(CAMERA_HALF_WIDTH);

    pid_orange.setOutputLimits(-BOUND, BOUND);
    pid_orange.setSetpointRange(CAMERA_HALF_WIDTH);

    VideoCapture capture(0);
    if (!capture.isOpened())
        CV_Assert("Cam open failed");

    capture.set(CAP_PROP_FPS, CAMERA_FPS);
    capture.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    capture.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

    /*
    cout << "CAP_PROP_FPS: " << capture.set(CAP_PROP_FPS, CAMERA_FPS) << endl;
    cout << "CAP_PROP_FRAME_WIDTH: " << capture.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH) << endl;
    cout << "CAP_PROP_FRAME_HEIGHT: " << capture.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT) << endl;
    */

    auto channel = grpc::CreateChannel(CONNECTION_STRING, grpc::InsecureChannelCredentials());
    stub = PositionService::NewStub(channel);

    thread thread_capture_frames(ThreadCaptureFrames, capture);
    thread thread_image_processing(ThreadProcessFrames);

    thread_capture_frames.join();
    thread_image_processing.join();

    cout << endl;

    // Release video
    capture.release();

    return EXIT_SUCCESS;
}
