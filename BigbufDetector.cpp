
#include "BigbufDetector.h"

#define B BigbufDetector

B::BigbufDetector(serial_port serialPortIN, bool debug) : serialPort(serialPortIN) {
    this->debug = debug;

    this->predictor = new Predictor(debug);

    /// solvepnp Data
    x = -width / 2;
    y = height / 2;
    z = 0;
    this->real_armor_points.emplace_back(x, y, z);
    x = width / 2;
    y = height / 2;
    z = 0;
    this->real_armor_points.emplace_back(x, y, z);
    x = width / 2;
    y = -height / 2;
    z = 0;
    this->real_armor_points.emplace_back(x, y, z);
    x = -width / 2;
    y = -height / 2;
    z = 0;
    this->real_armor_points.emplace_back(x, y, z);
};

/**
 * This function is repeatedly called by thread management while on the BigBuff Thread
 * @param frame
 * @param otherParam
 */
void B::feed_im(cv::Mat frame, OtherParam otherParam){
    double secondsInFutureToPredict = .5;

    TargetAndCenter targetAndCenter = this->getTargetAndCenterPoints(frame, (_color) otherParam.color);

    //if getTargetAndCenterPoints has a problem it will return an empty vector
    if(!targetAndCenter.failed){
        //creating temp variables for the struct variables
        vector<Point> targetPoints = targetAndCenter.targetRect;
        Point currentTargetCenter = targetAndCenter.targetCenter;
        Point buffCenter = targetAndCenter.buffCenter;

        double now;
        if(debug){
            //simulate 30 fps when debugging
            now = callCount * .033;
        }
        else{
            now  = timeSinceEpoch();
        }

        Point futureTargetCenterEstimate = this->predictor->predict(targetAndCenter, secondsInFutureToPredict, now);

        Point targetCenterDiff = futureTargetCenterEstimate - currentTargetCenter;

        vector<Point> futureTargetPoints;
        for (int i = 0; i < 4; i++) futureTargetPoints.emplace_back(
                    targetPoints[i] + targetCenterDiff);

        PitchAndYaw pitchYaw = getPitchYaw(futureTargetPoints);

        if(debug){
            std::cout << "pitch    " << pitchYaw.pitch << "yaw:      " << pitchYaw.yaw << endl;

            circle(frame, currentTargetCenter, 10, Scalar(0,0,255), -1);
            circle(frame, futureTargetCenterEstimate, 10, Scalar(0,255,255), -1);

            for (int i = 0; i < 4; i++) {
                line(frame, futureTargetPoints[i], futureTargetPoints[(i + 1) % 4], Scalar(0, 255, 255), 3);
            }

            this->display(frame, frame);
            waitKey(1);
        }
        else {
            outputToSerial( pitchYaw.pitch, pitchYaw.yaw);
        }
    }
}

/**
 * Sends data to the EE branch through the serial port.
 * CHECK WITH THE EE TEAM THAT THIS IS THE CORRECT PROTOCOL, IT MIGHT CHANGE
 * @param pitch
 * @param yaw
 */
void B::outputToSerial(int pitch, int yaw){
    struct serial_gimbal_data data;
    data.size = 8;
    data.rawData[0] = data.head;
    data.rawData[1] = yaw;
    data.rawData[2] = yaw >> 8;
    data.rawData[3] = pitch;
    data.rawData[4] = pitch >> 8;
    data.rawData[5] = 0x02;
    data.rawData[6] = 0xFF;
    uint8_t checkSum = data.rawData[0] + data.rawData[1] + data.rawData[2] + data.rawData[3] + data.rawData[4] + data.rawData[5] + data.rawData[6];
    data.rawData[7] = checkSum;

    this->serialPort.send_data(data);
}

/**
 * Inputs points of target and magically returns the pitch and yaw
 *
 * @param points
 * @return a vector containing the pitch and yaw
 */
PitchAndYaw B::getPitchYaw(vector<Point> points){

    vector<Point2f> points2f;
    cv::Mat(points).copyTo(points2f);

    cv::Mat rvec, tvec;
    cv::solvePnP(this->real_armor_points,
                 points2f, cameraMatrix,
                 distCoeffs, rvec, tvec);

    cv::Point3f target_3d;
    target_3d = cv::Point3f(tvec);
    int pitch = int((atan2(target_3d.y - 80, target_3d.z) + (float) (OFFSET_PITCH * CV_PI / 1800)) * 0.6 * 10000);
    int yaw = int((-atan2(target_3d.x, target_3d.z) + (float) (OFFSET_PITCH * CV_PI / 1800)) * 0.6 * 10000);

    PitchAndYaw out;
    out.pitch = pitch;
    out.yaw = yaw;

    return out;
}

/*
 * Target detection functions
 */

//1: find the rectangleiness of the contour
//2: maybe allow for detection of rectangles if they are at an angle
//3: use side ratios to help find the rectangles that are the right dimensions
bool B::isSpoke(vector<Point> &contour) {
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

bool B::isTarget(vector<Point> &contour) {
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

/**
 *
 * @param frame
 * @return [TARGET RECT POINTS , CENTER POINT]
 */
TargetAndCenter B::getTargetAndCenterPoints(Mat frame, _color color){

    //The targetAndCenter struct to return
    TargetAndCenter toReturn;

    vector<Point> spoke;
    int spoke_num = -1;
    vector<Point> target;
    int target_num = -1;
    vector<Point> center;
    int center_num = -1;

    //does the in range stuff and puts the result into mask
    Mat mask;

    //thresholds the frame based on BigBuff color
    if(color == BLUE){
        inRange(frame,
                Scalar(Blue_minBlue, Blue_minGreen, Blue_minRed),
                Scalar(Blue_maxBlue,Blue_maxGreen,Blue_maxRed),
                mask);
    }
    else if(color == RED){
        inRange(frame,
                Scalar(Red_minBlue, Red_minGreen, Red_minRed),
                Scalar(Red_maxBlue,Red_maxGreen,Red_maxRed),
                mask);
    }

    //used for passing into the dilate and erode methods
    Mat element  = getStructuringElement(MORPH_RECT, Size(element_size, element_size), Point(-1, -1));

    //expands pixels
    dilate(mask, mask, element);
    //smooths the image
    erode(mask, mask, element);

    vector<vector<cv::Point>> contours;
    vector<Vec4i> hierarchy;
    //hierarchy[0,1,2,3] = [Next, Previous, First_Child, Parent]
    //passes in the mask and assigns the contours found to contours variable
    findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    /// FINDING SPOKE
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
            printf("No spoke found");
        }
        toReturn.failed = true;
        return toReturn;
    }
    vector<Point> spoke_contour = contours[spoke_num];

    ///FINDING TARGET
    //loop through the children of the spoke
    //hierarchy[i][0] is the next child of the contour
    //hierarchy[spoke_num][2] is the first child (greatest i)
    for (int i = hierarchy[spoke_num][2]; i >= 0; i = hierarchy[i][0]) {
        vector<Point> curr_contour = contours[i];
        if (isTarget(curr_contour)) {
            target = curr_contour;
            target_num = i;

//UNCOMMENT TO DISPLAY TARGET INFO FOR DEBUGGING
//            RotatedRect rect = minAreaRect(curr_contour);
//            float width = rect.size.width;
//            float height = rect.size.height;
//            float side_ratio = 0;
//            if (width / height < 1) {
//                side_ratio = width / height;
//            }
//            else {
//                side_ratio = height / width;
//            }
//            float cont_ratio = contourArea(curr_contour) / (width * height);
//            printf("side ratio: %lf, contour ratio: %lf\n", side_ratio, cont_ratio);

            break;
        }
    }

    //ensure that we actually found a target
    if (target_num == -1) {
        // Display target's min_rect center
        if(debug){
            this->display(mask, frame);
            printf("No target found\n");
        }
        toReturn.failed = true;
        return toReturn;
    }
    vector<Point> target_contour = contours[target_num];

    ///FINDING BigBuff CENTER
    //calculating estimate location for the center
    Point target_center = getCenter(target_contour);
    Point spoke_center = getCenter(spoke_contour);

    double targetToSpokeDistance = norm(target_center - spoke_center);

    double deltaX = spoke_center.x - target_center.x;
    double deltaY = spoke_center.y - target_center.y;

    Point estimatedCenter = Point(
        target_center.x + ((double) this->slopeCoeff / 1000) * deltaX,
        target_center.y + ((double) this->slopeCoeff / 1000) * deltaY
    );

    //looping through the contours to find the center countour
    for (int i = 0; i < contours.size(); i++) {
        vector<Point> curr_contour = contours[i];
        //target will be a child of the spoke
        Point currCenter = getCenter(curr_contour);

        if (norm(estimatedCenter - currCenter) < targetToSpokeDistance * ((double) this->distanceErrorCoeff/1000)) {
            center = curr_contour;
            center_num = i;
            break;
        }
    }

    //ensure that we actually found a center
    if (center_num == -1) {
        // Display target's min_rect center
        if(debug){
            printf("no center found\n");
            circle(frame, estimatedCenter, 10, (0,0,255), -1);
            this->display(mask, frame);
        }
        toReturn.failed = true;
        return toReturn;
    }
    vector<Point> center_contour = contours[center_num];

    RotatedRect targetRect = minAreaRect(target_contour);
    Point2f targetPoints[4];
    targetRect.points(targetPoints);

    // display rects for target, spoke and center
    if(debug){
        //creating rectangles for displaying

        RotatedRect spoke_rect = minAreaRect(spoke_contour);
        Point2f spoke_pts[4];
        spoke_rect.points(spoke_pts);

        RotatedRect center_rect = minAreaRect(center_contour);
        Point2f center_pts[4];
        center_rect.points(center_pts);

        //draw the lines on the frame
        for (int i = 0; i < 4; i++) {
            line(frame, targetPoints[i], targetPoints[(i + 1) % 4], Scalar(255, 0, 0), 3);
            line(frame, spoke_pts[i], spoke_pts[(i + 1) % 4], Scalar(0, 255, 0), 3);
            line(frame, center_pts[i], center_pts[(i + 1) % 4], Scalar(0, 0, 255), 3);
        }
        circle(frame, estimatedCenter, ((int) (targetToSpokeDistance * ((double) this->distanceErrorCoeff/1000))), (0,0,255), 2);
//        this->display(mask, frame);
    }

    //creating output struct
    Point buffCenter = getCenter(center_contour);

    for(int i = 0; i < 4; i++)
        toReturn.targetRect.emplace_back(Point(targetPoints[i].x, targetPoints[i].y));

    toReturn.targetCenter = getCenter(toReturn.targetRect);
    toReturn.buffCenter = buffCenter;
    toReturn.failed = false;

    return toReturn;
}

void BigbufDetector::display(Mat mask, const Mat& frame){
    cv::imshow("mask", mask);
    cv::imshow("frame", frame);

    // Making a window to modify fields
    namedWindow("Fields",WINDOW_NORMAL);
    createTrackbar("red_min","Fields",&Blue_minRed,255);
    createTrackbar("red_max","Fields",&Blue_maxRed,255);
    createTrackbar("blue_min","Fields",&Blue_minBlue,255);
    createTrackbar("blue_max","Fields",&Blue_maxBlue,255);
    createTrackbar("green_min","Fields",&Blue_minGreen,255);
    createTrackbar("green_max","Fields",&Blue_maxGreen,255);
//    createTrackbar("min_spoke_side_ratio","Fields",&min_spoke_side_ratio,1000);
//    createTrackbar("max_spoke_side_ratio","Fields",&max_spoke_side_ratio,1000);
//    createTrackbar("min_spoke_area_ratio","Fields",&min_spoke_area_ratio,1000);
//    createTrackbar("max_spoke_area_ratio","Fields",&max_spoke_area_ratio,1000);
//    createTrackbar("min_target_side_ratio","Fields",&min_target_side_ratio,1000);
//    createTrackbar("max_target_side_ratio","Fields",&max_target_side_ratio,1000);
//    createTrackbar("min_target_area_ratio","Fields",&min_target_area_ratio,1000);
//    createTrackbar("max_target_area_ratio","Fields",&max_target_area_ratio,1000);
//    createTrackbar("center_to_target_min","Fields",&center_to_target_min,1000);
//    createTrackbar("center_to_target_max","Fields",&center_to_target_max,1000);
//
//    createTrackbar("slopeCoeff","Fields",&slopeCoeff,4000);
//    createTrackbar("distanceErrorCoeff","Fields",&distanceErrorCoeff,4000);


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
    threshold(R_ch, R_ch, Red_maxRed,255,THRESH_TRUNC);
    threshold(R_ch, R_ch, Red_minRed, 255, THRESH_TOZERO);
    threshold(B_ch, B_ch, Red_maxBlue,255,THRESH_TRUNC);
    threshold(B_ch, B_ch, Red_minBlue, 255, THRESH_TOZERO);
    threshold(G_ch, G_ch, Red_maxGreen,255,THRESH_TRUNC);
    threshold(G_ch, G_ch, Red_minGreen, 255, THRESH_TOZERO);

    resize(R_ch, R_ch, Size(500, 500));
    resize(B_ch, B_ch, Size(500, 500));
    resize(G_ch, G_ch, Size(500, 500));

    //put to 500 so we can see the image and it doesnt take up the whole screen
//    resize(mask, mask, Size(500, 500));
//    resize(frame, frame, Size(500, 500));

    imshow("red", R_ch);
    imshow("blue", B_ch);
    imshow("green", G_ch);

    waitKey(1);
}

/**
 *  Gets the center of a contour by finding the average of the points of the minAreaRect.
 *  This seems like the best way to do this.
 * @param contour
 * @return
 */
Point BigbufDetector::getCenter(vector<Point> &contour){

    RotatedRect rect = minAreaRect(contour);

    Point2f targetPoints[4];
    rect.points(targetPoints);

    Point total = Point(0, 0);

    for (Point2f & point : targetPoints) {
        total += (Point) point;
    }

    return total / 4;
}


/**
 * @return number of milliseconds since 1970 / 1000
 */
double BigbufDetector::timeSinceEpoch() {
    return double(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()) / 1000;
}