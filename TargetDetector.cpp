#include "opencv2/opencv.hpp"
#include "TargetDetector.h"

using namespace cv;
using namespace std;


TargetDetector::TargetDetector(bool debug) {
    this->debug = debug;
}

//TODO: make these functions better
//1: find the rectangleiness of the contour
//2: maybe allow for detection of rectangles if they are at an angle
//3: use side ratios to help find the rectangles that are the right dimensions
bool TargetDetector::isSpoke(vector<Point> &contour) {
    //check that the area is in a certain range
    RotatedRect rect = minAreaRect(contour);
    float width = rect.size.width;
    float height = rect.size.height;
    float side_ratio = 0;
    if (width / height < 1) {
        side_ratio = width / height;
    }
    else {
        side_ratio = height / width;
    }
    float cont_ratio = contourArea(contour) / (width * height);
    return (side_ratio > ((double) min_spoke_side_ratio / 1000)) &&
           (side_ratio < ((double) max_spoke_side_ratio / 1000)) &&
           (cont_ratio > ((double) min_spoke_area_ratio / 1000)) &&
           (cont_ratio < ((double) max_spoke_area_ratio / 1000));
}

bool TargetDetector::isTarget(vector<Point> &contour) {
    //check that the area is in a certain range

    RotatedRect rect = minAreaRect(contour);
    float width = rect.size.width;
    float height = rect.size.height;
    float side_ratio = 0;
    if (width / height < 1) {
        side_ratio = width / height;
    }
    else {
        side_ratio = height / width;
    }
    float cont_ratio = contourArea(contour) / (width * height);
    return (side_ratio > ((double) min_target_side_ratio / 1000)) &&
            (side_ratio < ((double) max_target_side_ratio / 1000)) &&
            (cont_ratio > ((double) min_target_area_ratio / 1000)) &&
            (cont_ratio < ((double) max_target_area_ratio / 1000));
}

bool TargetDetector::isCenter(vector<Point> &contour) {
    //check that the area is in a certain range
    RotatedRect rect = minAreaRect(contour);
    float width = rect.size.width;
    float height = rect.size.height;
    float side_ratio = 0;
    if (width / height < 1) {
        side_ratio = width / height;
    }
    else {
        side_ratio = height / width;
    }
    float cont_ratio = contourArea(contour) / (width * height);
    return (side_ratio > ((double) min_center_side_ratio / 1000)) &&
           (side_ratio < ((double) max_center_side_ratio / 1000)) &&
           (cont_ratio > ((double) min_center_area_ratio / 1000)) &&
           (cont_ratio < ((double) max_center_area_ratio / 1000));
}

/**
 *
 * @param frame
 * @return points marking the corners of the target
 */
vector<Point2f> TargetDetector::get2DTargetBounds(Mat frame, uint8_t color){

    vector<Point> spoke;
    int spoke_num = -1;
    vector<Point> target;
    int target_num = -1;
    vector<Point> center;
    int center_num = -1;

    //does the in range stuff and puts the result into mask
    Mat mask;
    //creates bitwise array: 1 in areas where the rgb values are within mins and max
    inRange(frame, Scalar(min_blue,min_green,min_red),Scalar(max_blue,max_green,max_red), mask);

    //used for passing into the dilate and erode methods
    Mat element  = getStructuringElement(MORPH_RECT, Size(element_size, element_size), Point(-1, -1));

    //expands pixels
    dilate(mask, mask, element);
    //smooths the image
    erode(mask, mask, element);

    //put to 500 so we can see the image and it doesnt take up the whole screen
    resize(mask, mask, Size(500, 500));
    resize(frame, frame, Size(500, 500));

    vector<vector<cv::Point>> contours;
    vector<Vec4i> hierarchy;
    //hierarchy[0,1,2,3] = [Next, Previous, First_Child, Parent]
    //passes in the mask and assigns the contours found to contours variable
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    //looping through the contours to find the spoke contour
    for (int i = 0; i < contours.size(); i++) {
        vector<Point> curr_contour = contours[i];
        //target will be a child of the spoke
        if (hierarchy[i][2] >= 0 && isSpoke(curr_contour)) {
            spoke = curr_contour;
            spoke_num = i;
            break;
        }
    }

    //if there is no spoke return nothing
    if (spoke_num == -1) {
        // Display target's min_rect center
        if(debug){
            this->display(mask, frame);
        }
        return vector<Point2f> {};
    }

    //loop through the children of the spoke
    //hierarchy[i][0] is the next child of the contour
    //hierarchy[spoke_num][2] is the first child (greatest i)
    for (int i = hierarchy[spoke_num][2]; i >= 0; i = hierarchy[i][0]) {
        vector<Point> curr_contour = contours[i];
        if (isTarget(curr_contour)) {
            target = curr_contour;
            target_num = i;
            break;
        }
    }

    //ensure that we actually found a target
    if (target_num == -1) {
        // Display target's min_rect center
        if(debug){
            this->display(mask, frame);
        }

        vector<Point2f> {};
    }

    vector<Point> curr_contour = contours[target_num];
    RotatedRect rect = minAreaRect(curr_contour);

    Point2f pts[4];
    rect.points(pts);

    for (int i = hierarchy[spoke_num][2]; i >= 0; i = hierarchy[i][0]) {
        vector<Point> curr_contour = contours[i];
        if (isCenter(curr_contour)) {
            center = curr_contour;
            center_num = i;
            break;
        }
    }

    //ensure that we actually found a target
    if (center_num == -1) {
        // Display target's min_rect center
        if(debug){
            this->display(mask, frame);
        }

        vector<Point2f> {};
    }

    //draw the lines on the frame
    for (int i = 0; i < 4; i++) {
        line(frame, pts[i], pts[(i + 1) % 4], Scalar(i * 50, 0, 0), 3);
    }

    // Display target's min_rect center
    if(debug){
        this->display(mask, frame);
    }


    //converting to vector
    return vector<Point2f> (std::begin(pts), std::end(pts));
}

void TargetDetector::display(Mat mask, const Mat& frame){
    cv::imshow("mask", mask);
    cv::imshow("frame", frame);

    // Making a window to modify fields
    namedWindow("Fields",WINDOW_NORMAL);
    createTrackbar("red_min","Fields",&min_red,255);
    createTrackbar("red_max","Fields",&max_red,255);
    createTrackbar("blue_min","Fields",&min_blue,255);
    createTrackbar("blue_max","Fields",&max_blue,255);
    createTrackbar("green_min","Fields",&min_green,255);
    createTrackbar("green_max","Fields",&max_green,255);
    createTrackbar("min_spoke_side_ratio","Fields",&min_spoke_side_ratio,1000);
    createTrackbar("max_spoke_side_ratio","Fields",&max_spoke_side_ratio,1000);
    createTrackbar("min_spoke_area_ratio","Fields",&min_spoke_area_ratio,1000);
    createTrackbar("max_spoke_area_ratio","Fields",&max_spoke_area_ratio,1000);
    createTrackbar("min_target_side_ratio","Fields",&min_target_side_ratio,1000);
    createTrackbar("max_target_side_ratio","Fields",&max_target_side_ratio,1000);
    createTrackbar("min_target_area_ratio","Fields",&min_target_area_ratio,1000);
    createTrackbar("max_target_area_ratio","Fields",&max_target_area_ratio,1000);
    createTrackbar("min_center_side_ratio","Fields",&min_center_side_ratio,1000);
    createTrackbar("max_center_side_ratio","Fields",&max_center_side_ratio,1000);
    createTrackbar("max_center_side_ratio","Fields",&max_center_side_ratio,1000);
    createTrackbar("max_center_area_ratio","Fields",&max_center_area_ratio,1000);

    //createTrackbar("min_spoke_area","Fields",&min_spoke_area,2000);
    //createTrackbar("max_spoke_area","Fields",&max_spoke_area,2000);
    //createTrackbar("min_target_area","Fields",&min_target_area,2000);
    //createTrackbar("max_target_area","Fields",&max_target_area,2000);
    //createTrackbar("min_center_area","Fields",&min_center_area,2000);
    //createTrackbar("max_center_area","Fields",&max_center_area,2000);
    //createTrackbar("element_size","Fields",&element_size,30);
    //createTrackbar("curr_contour_idx","Fields",&curr_contour_idx,50);

    vector<Mat> ch;
    split(frame,ch);
    Mat R_ch,G_ch,B_ch;
    R_ch = ch[2]; G_ch = ch[1]; B_ch=ch[0];
    threshold(R_ch, R_ch, max_red,255,THRESH_TRUNC);
    threshold(R_ch, R_ch, min_red, 255, THRESH_TOZERO);
    threshold(B_ch, B_ch, max_blue,255,THRESH_TRUNC);
    threshold(B_ch, B_ch, min_blue, 255, THRESH_TOZERO);
    threshold(G_ch, G_ch, max_green,255,THRESH_TRUNC);
    threshold(G_ch, G_ch, min_green, 255, THRESH_TOZERO);

    resize(R_ch, R_ch, Size(500, 500));
    resize(B_ch, B_ch, Size(500, 500));
    resize(G_ch, G_ch, Size(500, 500));

    imshow("red", R_ch);
    imshow("blue", B_ch);
    imshow("green", G_ch);

    waitKey(0);
}


Mat TargetDetector::drawLines(Mat src, vector<vector<cv::Point>> vector) {
//    for( size_t i = 0; i < vector.size(); i++ )
//    {
//        for (int j = 0; j < vector[i].size() - 1; ++j) {
//            cv::line( src, vector[i][j], vector[i][j+1], cv::Scalar(0,100,0), 3);
//        }
//    }
    return src;
}

