#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//These file locations will change when integrated with rest of project
#include "util/common.h" //RobotBase/vision
#include "util/message.h" //robogrinder SDK
#include "util/serial_port.h"

#ifndef BIGBUFF_BIGBUFDETECTOR_H
#define BIGBUFF_BIGBUFDETECTOR_H


/**
 * This class is for finding the targets of the big buff and predicting target location into the future.
 * Tests for this class can NOT be found in the main at https://github.com/robogrinder/CVB-Bigbuff
 *  WHO TF DELETED THE REPO???? I WAS STILL USING IT!!!!!!!!!!
 */
class BigbufDetector{
public:
    BigbufDetector(serial_port serialPort, bool debug = false);

    //meant to be used publicly
    void feed_im(cv::Mat frame, OtherParam otherParam);

    //This struct is used for returning target and center info from functions
    struct TargetAndCenter{
        vector<Point> targetRect;
        Point targetCenter;
        Point buffCenter;
        bool failed;
    };
    struct PitchAndYaw{
        int pitch;
        int yaw;
    };

    //for testing
    Point predictFutureTargetLocation(BigbufDetector::TargetAndCenter, double t, double currentTimeS);
    double timeSinceEpoch();

    BigbufDetector::TargetAndCenter getTargetAndCenterPoints(cv::Mat frame, _color color);

private:
    serial_port serialPort;

    void outputToSerial(int pitch, int yaw);

    BigbufDetector::PitchAndYaw getPitchYaw(vector<Point> points);
    double PI = 3.14159265;

    //if true, will print out messages to the console, and not use the serial port
    bool debug = false;

    //increments every cycle of feed_im, used for keeping time when debugging
    int callCount = 0;

/*
 * Vars and functions for finding the target / center
 */
    //Used for color thresholding
        //experimentally found values for doing target detection
        int Blue_minRed = 35, Blue_maxRed = 200;
        int Blue_minBlue = 220, Blue_maxBlue = 255;
        int Blue_minGreen = 220, Blue_maxGreen = 255;

        int Red_minRed = 20, Red_maxRed = 255;
        int Red_minBlue = 10, Red_maxBlue = 100;
        int Red_minGreen = 10, Red_maxGreen = 100;

    //Used for finding spoke with target
        //all of these "ratio" values are divided by 1000 before used
        int min_spoke_side_ratio = 0;
        int max_spoke_side_ratio = 480;
        int min_spoke_area_ratio = 0;
        int max_spoke_area_ratio = 430;

        //average side ratio: .62
        int min_target_side_ratio = 500;
        int max_target_side_ratio = 700;
        //average area ration : .85 (can be higher if very accurate rectangele)
        int min_target_area_ratio = 800;
        int max_target_area_ratio = 1000;

    //used for finding the bigbuff center
        //represents the distance to the center of the bigbuff in terms of the distance from the center of spoke to the center of the target
        int slopeCoeff = 3350;
        //distanceErrorCoeff * (distance from the center of spoke to the center of the target) = maxDistanceError
        int distanceErrorCoeff = 600;

    int element_size = 7;

    bool isSpoke(vector <Point> &contour);
    bool isTarget(vector <Point> &contour);
    Point getCenter(vector<Point> &contour);
    void display(Mat mask, const Mat& frame);

//  These variables are for finding the real angles the motors need to turn from where the target was found in the image

    float x, y, z, width = 140.0f, height = 60.0f;
    int OFFSET_YAW = 3600;
    int OFFSET_PITCH = 3600;

    std::vector<cv::Point3f> real_armor_points;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
            1293.5303221625442802, 0.3651215140945823, 355.9091806402759630,
            0.0000000000000000, 1293.9256252855957428, 259.1868664367483461,
            0.0000000000000000, 0.0000000000000000, 1.0000000000000000);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5)
            << -0.2126367859619807, 0.2282910064864265, 0.0020583387355406, 0.0006136511397638, -0.7559987171745171);

/*
 * vars for predicting where the target is going to go
*/

    double angVelocityAvg = 0;
    double lastSpokeAngle = 0;
    double timeOfLastPredictCall = 0;
};

#endif //BIGBUFF_BIGBUFDETECTOR_H
