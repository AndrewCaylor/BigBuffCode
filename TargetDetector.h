#include "opencv2/opencv.hpp"

#ifndef BIGBUFF_TARGETDETECTOR_H
#define BIGBUFF_TARGETDETECTOR_H

//These file locations will change when integrated with rest of project
#include "TargetDetector.h"
#include "DataCollector.h"

using namespace std;
using namespace cv;

class TargetDetector {
private:
    bool isSpoke(vector <Point> &contour);
    bool isTarget(vector <Point> &contour);
    bool isCenter(vector<Point> &contour);

    bool debug = false;

    //TODO: create thresholds that allow for red and blue
    //experimentally found values
    int min_red = 35, max_red = 200;
    int min_blue = 220, max_blue = 255;
    int min_green = 220, max_green = 255;
    //int max_spoke_area = 1050;
    //int min_spoke_area = 800;
    int min_spoke_side_ratio = 0;
    int max_spoke_side_ratio = 0;
    int min_spoke_area_ratio = 0;
    int max_spoke_area_ratio = 0;
    //int max_target_area = 2000;
    //int min_target_area = 0;
    int min_target_side_ratio = 0;
    int max_target_side_ratio = 0;
    int min_target_area_ratio = 0;
    int max_target_area_ratio = 0;

    //int max_center_area = 0;
    //int min_center_area = 0;
    int min_center_side_ratio = 0;
    int max_center_side_ratio = 0;
    int min_center_area_ratio = 0;
    int max_center_area_ratio = 0;

    int element_size = 9;
    int curr_contour_idx = 0;

public:
    TargetDetector(bool debug = false);
    std::vector<cv::Point2f> get2DTargetBounds(cv::Mat frame, uint8_t color);

    void display(Mat mask, const Mat& frame);

    Mat drawLines(Mat mat, vector<vector<Point>> lines);
};


#endif //BIGBUFF_TARGETDETECTOR_H
