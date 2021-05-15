#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//These file locations will change when integrated with rest of project
#include "util/common.h" //RobotBase/vision
#include "util/message.h" //robogrinder SDK
#include "util/serial_port.h"
#include "TargetDetector.h"
#include "DataCollector.h"

#ifndef BIGBUFF_BIGBUFDETECTOR_H
#define BIGBUFF_BIGBUFDETECTOR_H


class BigbufDetector{
private:
    serial_port serialPort;
    TargetDetector targetDetector;
    DataCollector dataCollector;
    void outputToSerial(int pitch, int yaw);

    //returns a pointer to the array
    int* getPitchYaw(vector<Point2f> frontInputPoints);

    //values are for solvePNP function
    Mat cameraMatrix= (cv::Mat_<double>(3, 3) << 1293.5303221625442802, 0.3651215140945823, 355.9091806402759630,
            0.0000000000000000, 1293.9256252855957428, 259.1868664367483461,
            0.0000000000000000, 0.0000000000000000, 1.0000000000000000);;
    Mat distCoeffs = (cv::Mat_<double>(1, 5)
            << -0.2126367859619807, 0.2282910064864265, 0.0020583387355406, 0.0006136511397638, -0.7559987171745171);;
    std::vector<cv::Point3f> bigBuffTarget3DPoints = {{0,0,0}};

    bool debug = false;

public:
    void feed_im(cv::Mat frame, OtherParam otherParam);
    BigbufDetector(serial_port serialPort, bool debug = false);

    vector<float> getPitchYawSimple(vector<Point2f> points, int xdim, int ydim);
};

#endif //BIGBUFF_BIGBUFDETECTOR_H
