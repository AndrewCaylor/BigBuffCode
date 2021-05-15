
#include "BigbufDetector.h"

BigbufDetector::BigbufDetector(serial_port serialPortIN, bool debug) : serialPort(serialPortIN) {

    this->dataCollector = DataCollector(0.0, debug);
    this->targetDetector = TargetDetector(debug);
    this->debug = debug;

    //initialize values needed to solve for pnp

    //TODO: find real values for bigBuffTarget3DPoints
    //Fill front object points(x-y-z order in cms)
    //It is square of side 12.8cms on Z=0 plane
//    bigBuffTarget3DPoints.push_back(Point3f (0, 0, 0));
//    bigBuffTarget3DPoints.push_back(Point3f (-12.8, 0, 0));
//    bigBuffTarget3DPoints.push_back(Point3f (-12.8, 12.8, 0));
//    bigBuffTarget3DPoints.push_back(Point3f (0, 12.8, 0));
};

/**
 * This function is repeatedly called by thread management while on BigBuff Thread
 * @param frame
 * @param otherParam
 */
void BigbufDetector::feed_im(cv::Mat frame, OtherParam otherParam){

    //TODO: probably do things to interact with the DataCollector class
    vector<Point2f> targetBounds = targetDetector.get2DTargetBounds(frame, otherParam.color);

    vector <float> pitchYaw = getPitchYawSimple(targetBounds, frame.cols, frame.rows);


    printf("pitch:%f yaw:%f\n", pitchYaw[0], pitchYaw[1]);

    if(!debug) {
        outputToSerial((int) pitchYaw[0], (int) pitchYaw[1]);
    }
}

/**
 * Sends data to the EE branch through the serial port
 * @param pitch
 * @param yaw
 */
void BigbufDetector::outputToSerial(int pitch, int yaw){
    struct serial_gimbal_data data;
    data.size = 6;
    data.rawData[0] = data.head;
    data.rawData[1] = data.id;
    data.rawData[2] = pitch;
    data.rawData[3] = pitch >> 8;
    data.rawData[4] = yaw;
    data.rawData[5] = yaw >> 8;

    this->serialPort.send_data(data);
}

/**
 * Inputs the points of the center of the BigBuff and outputs the pitch and yaw
 * @param frontInputPoints
 * @return a pointer to an array containing the pitch and yaw
 */
int * BigbufDetector::getPitchYaw(vector<Point2f> frontInputPoints){


    //info about solvePNP
    //https://learnopencv.com/head-pose-estimation-using-opencv-and-dlib/
    //https://answers.opencv.org/question/170817/obtain-camera-pose-and-camera-real-world-position-using-solvepnp-c/

    vector<Point2f>frontImagePoints;

    //Corresponding Image points detected in the same order as object points
    frontImagePoints.push_back(frontInputPoints[0]);
    frontImagePoints.push_back(frontInputPoints[1]);
    frontImagePoints.push_back(frontInputPoints[2]);
    frontImagePoints.push_back(frontInputPoints[3]);

    Mat rvec;
    Mat tvec;

    //TODO: find real values for bigBuffTarget3DPoints
    solvePnP(bigBuffTarget3DPoints, frontImagePoints, cameraMatrix, distCoeffs, rvec, tvec, false,
             SOLVEPNP_ITERATIVE);

    Point3f target_3d = {0, 0, 0};
    target_3d = cv::Point3f(tvec);
    printf("x:%f y:%f z:%f\n", target_3d.x, target_3d.y, target_3d.z);

    int pitch = int((atan2(target_3d.y - 80, target_3d.z) + (float) ( CV_PI / 1800)) * 0.6 * 10000);
    //int pitch = 15000;
    int yaw = int((-atan2(target_3d.x, target_3d.z) + (float) ( CV_PI / 1800)) * 0.4 * 10000);

    int out [2] = {pitch, yaw};
    return out;
}


/**
 * Inputs 2d screen points and outputs the pitch and yaw
 * @param points
 * @return a vector containing the pitch and yaw
 */
vector<float> BigbufDetector::getPitchYawSimple(vector<Point2f> points, int xdim, int ydim){
    float xtot = 0;
    float ytot = 0;

    for (auto & point : points) {
        xtot += point.x;
        ytot += point.y;
    }

    float xavg = xtot / points.size();
    float yavg = ytot / points.size();

    printf("x:%d y:%d \n", (int) xavg, (int) xdim);

    //TODO: replace these with actual values
    const float fovx = CV_PI;
    const float fovy = CV_PI;

    float xdiff = xavg - (float) xdim/2;
    float ydiff = yavg - (float) ydim/2;

    vector<float> out = {-(ydiff/ydim) * fovy, (xdiff/xdim) * fovx};
    return out;
}