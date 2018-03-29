#include <ros/ros.h>
#define PI 3.1414

/*
bool Buoy::detectBuoy()
{
    if(!_image.empty())
    {
        Mat org,_finalImg, gray, edges,dst, org1;
        _finalImg = obj.run(_image,1,0);
//        imshow("max edge",obj.run(_image,1,0));
//        cvtColor(_finalImg, _imageHSV, CV_BGR2HSV);
//        Mat gray,dst;
        cvtColor(_finalImg,gray,CV_BGR2GRAY);
        adaptiveThreshold(gray,org,100,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,13,0);

//        fastNlMeansDenoising(org,dst,3,7,21 );
//        fastNlMeansDenoising(org, dst);
//        imshow("denoised ", dst);
//        inRange(_imageHSV,_lowerThreshYellow,_upperThreshYellow, org);
        // inRange(_imageHSV,_lowerThreshGreen,_upperThreshGreen, _imageBWGreen);

         imshow("org", org);

        Mat org2, erodeimg;
//        medianBlur(org, _imageBW, 3);

        erode(org, erodeimg, _kernelDilateErode);
        erode(erodeimg, erodeimg, _kernelDilateErode);
        imshow("erode", erodeimg);
        dilate(erodeimg, org2, _kernelDilateErode);
//        dilate(org2, org2, _kernelDilateErode);
        Canny(org2,edges,0,100);
//        imshow("eroded image", edges);
        imshow("dilated ", org2);
        imshow("canny", edges);
        waitKey(33);
        CBlobResult _blobs,_blobsClutter;
        CBlob * _currentBlob;
        IplImage _imageBWipl = org2;
        _blobs = CBlobResult(&_imageBWipl, NULL, 0);
        _blobs.Filter(_blobs, B_INCLUDE, CBlobGetArea(), B_INSIDE, 200, 5000);

//         _imageBW = Scalar(0, 0, 0);
        org2 = Scalar(0, 0, 0);

        cout << "number of blobs " << _blobs.GetNumBlobs() << endl;

        for(int i = 0; i < _blobs.GetNumBlobs(); i++)
        {
            _currentBlob = _blobs.GetBlob(i);
            _currentBlob->FillBlob(&_imageBWipl, Scalar(255));
        }

        Mat newimg;
//        Mat _imageBW2 = _imageBW.clone();
        Mat _imageBW2 = org2.clone();
        dilate(_imageBW2, newimg, _kernelDilateErode);
        imshow("newimg", newimg);
        Mat src;
        vector<Mat> channels;
        channels.push_back(org2);
        channels.push_back(org2);
        channels.push_back(org2);
        merge( channels, src);

        _contours.clear();
//        medianBlur(_imageBW2, _imageBW2, 5);
        imshow("imageBW2", _imageBW2);
        findContours(_imageBW2, _contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

        Point2f _centerBuff;
        float _radiusBuff;
        vector<Point> _contoursPolyBuff;
        _center.clear();
        _radius.clear();
        _contoursPoly.clear();

        _imageBW = Scalar(0, 0, 0);

        for(int i=0; i < _contours.size(); i++)
        {
            if(contourArea(_contours[i])>50)
            {
                approxPolyDP(_contours[i],_contoursPolyBuff,3,true);
                minEnclosingCircle((Mat)_contoursPolyBuff,_centerBuff,_radiusBuff);
                circle(_imageBW,_centerBuff,_radiusBuff,Scalar(255), -1);
                _center.push_back(_centerBuff);
                _radius.push_back(_radiusBuff);
                _contoursPoly.push_back(_contoursPolyBuff);
            }
        }

        Mat src_gray;
        cvtColor( src, src_gray, CV_BGR2GRAY );
        src = Scalar(0, 0, 0);

        GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
        imshow("src_gray", src_gray);

        /// Apply the Hough Transform to find the circles
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, 80, 180, 25, 30, 300 );

        //       /// Draw the circles detected
        if(circles.size() == 0){
            cout<<"NOTHING CIRCULAR" << endl;
            return false;
        }





        for( size_t i = 0; i < circles.size(); i++ )
        {
            cout << "circle wala for loop " << endl;
            cout << "circle area = " << endl;
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            _fcenter.push_back(center);
            _fradius.push_back(cvRound(circles[i][2]));
            // circle center
            circle( src, center, 3, Scalar(0, 255, 0), 3, 8, 0 );
            // circle outline
            circle( src, center, radius, Scalar(0, 0, 255), 1, 8, 0 );
        }

        
        for (int i = 0; i < _fradius.size(); ++i)
        {
            if(_fradius[i] > max){
                index = i;
                max = _fradius[i];
            }
        }
        cout << "<<<<<<<<<<     largest radius = " << max << "          >>>>>>>>>>>>>>>"<<endl;
        if(max == 0){
            cout << "max  = 0" << endl;
            return false;
        }
        else{
            circle(src,_fcenter[index],3,Scalar(255,255,0),1,8,0);
            circle(src,_fcenter[index], max,Scalar(0,255,0),1,8,0);
//            return true;
        }
        cout << "show src image" << endl;
        imshow("src", src);

        _fcenter.clear();
        _fradius.clear();
//        if(_center.size() > 0){
//            cout << "center size > 0 " << endl;
//            return true;
//        }
//        else{
//            cout << "returns false" << endl;
//            return false;
//        }

        return true;
    }
    else{
        cout << "no image loaded.." << endl;
        return false;
    }


    waitKey(33);
    return 0;
}
*/






/*

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <stdio.h>

#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

int canny_threshold = 200;
int hough_threshold = 100; // for 640x480, 80 for 320x240
static const string OPENCV_WINDOW = "Image window";

*/

/*
void hough_transform (Mat &src, Mat &dest)
{
    // imshow("bgr",src);
    Mat gray;
    cvtColor(src,gray,CV_BGR2GRAY);
    GaussianBlur(gray,gray,Size(5,5),2,2);
    vector<Vec3f> circles;
*/

   // HoughCircles(gray,circles,CV_HOUGH_GRADIENT,2,gray.rows/8,canny_threshold/*200*/,hough_threshold/*80 for 320x240, 100 for 640x480*/);
    /*
    Canny(gray, gray, MAX(canny_threshold/2,1), canny_threshold, 3);
    cvtColor(gray,gray,CV_GRAY2BGR);
    if (circles.size()==0) cout << "circle not found" << endl;
    else for (size_t i=0; i<circles.size(); i++) {
        Point center(cvRound(circles[i][0]),cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(gray,center,3,Scalar(0,0,255),-1,8,0);
        circle(gray,center,radius,Scalar(0,0,255),3,8,0);
        // if (i==0) {
        //     cout << "x = " << center.x << " , radius = " << radius << endl;
        // }
    }
    // imshow("circle",gray);
    dest = gray;
}

void videoCaptureCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    
    //VideoCapture cap; // open the default camera
    
    if(!cap.isOpened()) { // check if we succeeded
        cout << "Cap not opened";
        //return -1;
    }

    Mat edges;
    namedWindow("src",1);
    namedWindow("edges",1);
    createTrackbar("canny threshold", "edges", &canny_threshold, 255);
    createTrackbar("hough threshold", "edges", &hough_threshold, 255);

  //  for(;;)
  //  {
        Mat frame, hough;
        cap >> frame; // get a new frame from camera

        imshow("src", frame);
        cvtColor(frame, edges, CV_BGR2GRAY);
        hough_transform(frame, hough);
         
        imshow("edges", edges);
        imshow("edges", hough);
        //if(waitKey(30) >= 0) break;

    //}
    // the camera will be deinitialized automatically in VideoCapture destructor
//}



int main(int argc, char **argv)
{
    cout << "OpenCV version : " << CV_VERSION << endl;

    ros::init(argc, argv, "video");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/kinect2/qhd/image_color_rect", 30, videoCaptureCallback);
    //ros::Subscriber sub = n.subscribe("image_color_rect", 30, videoCaptureCallback);
    ros::spin();
    return 0;
}
*/







#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/topic.h>
#include <boost/shared_ptr.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

static const char OPENCV_WINDOW[] = "Image window";

int canny_threshold = 200;
int hough_threshold = 100; // for 640x480, 80 for 320x240


        int iLowH = 19;
        int iHighH = 36;

        int iLowS = 65;
        int iHighS = 255;

        int iLowV = 50;
        int iHighV = 255;

double aspectRatioLow = 0.35;
double aspectRatioHigh = 0.55;
double angleRange = 30.0;
int maxDistance = 1000; //mm
int actual_distance = 0;


typedef struct v { 
  float x;  
  float y;  
  float z;  
} vector3d;

float raw_depth_to_meters(int depth_value){ 
  float depth_value_f = (float) depth_value; 
  if (depth_value < 2047){ 
    float depth = 1.0 / (depth_value_f  * -0.0030711016 + 3.3309495161);
    return depth; 
  }
  return 0.0f; 
} 

vector3d get_handpos_xyz_from_cg(float cgx, float cgy, float cgz){
  double fx_d = 1.0 / 5.9421434211923247e+02;
  double fy_d = 1.0 / 5.9104053696870778e+02;
  double cx_d = 3.3930780975300314e+02;
  double cy_d = 2.4273913761751615e+02;

  float depth = raw_depth_to_meters(cgz);
  
  vector3d hand_pos = {   
    (float) (cgx - cx_d) * depth * fx_d,
    (float) (cgy - cy_d) * depth * fy_d,
    (float) depth };  

return hand_pos; 
}

void hough_transform (Mat &src, Mat &dest)
{
    // imshow("bgr",src);
    Mat gray;
    cvtColor(src,gray,CV_BGR2GRAY);
    GaussianBlur(gray,gray,Size(5,5),2,2);
    vector<Vec3f> circles;

    HoughCircles(gray,circles,CV_HOUGH_GRADIENT,2,gray.rows/8,canny_threshold/*200*/,hough_threshold/*80 for 320x240, 100 for 640x480*/);
    Canny(gray, gray, MAX(canny_threshold/2,1), canny_threshold, 3);
    cvtColor(gray,gray,CV_GRAY2BGR);
    if (circles.size()==0) cout << "circle not found" << endl;
    else for (size_t i=0; i<circles.size(); i++) {
        Point center(cvRound(circles[i][0]),cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(gray,center,3,Scalar(0,0,255),-1,8,0);
        circle(gray,center,radius,Scalar(0,0,255),3,8,0);
        // if (i==0) {
        //     cout << "x = " << center.x << " , radius = " << radius << endl;
        // }
    }
    // imshow("circle",gray);
    dest = gray;
}

class ImageConverter
{
    ros::NodeHandle nh_;
    ros::NodeHandle nhPriv;

    image_transport::ImageTransport it_;
    
    ros::Subscriber cameraInfoSub_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher image_pub_;

    RotatedRect canBoundingBox;
    Point2f canRectCoords;

    double rgbCols, rgbRows, depthW, depthH;

    public:
    ImageConverter(): it_(nh_)
    {
        nhPriv = ros::NodeHandle("~");

        // Subscribe to input video feed and publish output video feed
//        image_sub_ = it_.subscribe("/kinect2/qhd/image_color_rect", 1, &ImageConverter::imageCb, this);
        image_sub_ = it_.subscribe("/kinect2/sd/image_color_rect", 1, &ImageConverter::imageCb, this);
        depth_sub_ = it_.subscribe("/kinect2/sd/image_depth_rect", 1, &ImageConverter::depthImageCb, this);

        //image_pub_ = it_.advertise("/image_converter/output_video", 1);
        nhPriv.getParam("lowH", iLowH); nhPriv.getParam("highH", iHighH);
        nhPriv.getParam("lowS", iLowS); nhPriv.getParam("highS", iHighS);
        nhPriv.getParam("lowV", iLowV); nhPriv.getParam("highV", iHighV);

        nhPriv.getParam("aspect_ratio_low", aspectRatioLow);
        nhPriv.getParam("aspect_ratio_high", aspectRatioHigh);
        nhPriv.getParam("angle_range", angleRange);
        nhPriv.getParam("max_distance", maxDistance);

        cout << "lowH=" << iLowH << " highH=" << iHighH << endl;

        namedWindow(OPENCV_WINDOW,CV_WINDOW_NORMAL);
    }

    ~ImageConverter()
    {
        destroyWindow(OPENCV_WINDOW);
    }

    void cameraInfo(const sensor_msgs::CameraInfoPtr& msg)
    {
    		cout<< "Height: " <<msg->height << ", width: " << msg->width << endl;
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {  
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw an example circle on the video stream
        //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            //circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));

        /* HOUGH TRANSFORMATION

        Mat edges;
        createTrackbar("canny threshold", OPENCV_WINDOW, &canny_threshold, 255);
        createTrackbar("hough threshold", OPENCV_WINDOW, &hough_threshold, 255);

        Mat frame, hough;
        frame = cv_ptr->image; // get a new frame from camera

        cvtColor(frame, edges, CV_BGR2GRAY);
        hough_transform(frame, hough);
        
        // Update GUI Window
        imshow(OPENCV_WINDOW, edges);
        imshow(OPENCV_WINDOW, hough);

        //END HOUGH TRANSFORMATION */

        // edited 


        Mat imgOriginal = cv_ptr->image;
        Mat imgHSV;
        rgbCols = imgOriginal.cols; rgbRows = imgOriginal.rows;

        GaussianBlur(imgOriginal,imgOriginal,Size( 5, 5 ), 0, 0);

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological opening (removes small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (removes small holes from the foreground)
        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        

        // find contours (if always so easy to segment as your image, you could just add the black/rect pixels to a vector)
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::findContours(imgThresholded, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    /// Draw contours and find biggest contour (if there are other contours in the image, we assume the biggest one is the desired rect)
    // drawing here is only for demonstration!
    int biggestContourIdx = -1;
    float biggestContourArea = 0;
    // Mat drawing = Mat::zeros( imgThresholded.size(), CV_8UC3 );


    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar(0, 100, 0);
        drawContours( imgThresholded, contours, i, color, 1, 8, hierarchy, 0, Point() );

        float ctArea= contourArea(contours[i]);
        RotatedRect bbox = minAreaRect(contours[i]);
        float pomerStran = bbox.size.width/bbox.size.height;
//        cout << pomerStran << endl;

        if(ctArea > biggestContourArea
                && (pomerStran > aspectRatioLow && pomerStran < aspectRatioHigh)
                && (abs(bbox.angle) < angleRange))
                //&& actual_distance < maxDistance)
//        if(ctArea > biggestContourArea)
        {
            biggestContourArea = ctArea;
            biggestContourIdx = i;
        }
    }

    // if no contour found
    if(biggestContourIdx < 0)
    {
        cout << "no contour found" << endl;
        imshow("original", imgOriginal);
        imshow(OPENCV_WINDOW, imgThresholded);
        waitKey(15);
        return;
    }

    // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
    canBoundingBox = minAreaRect(contours[biggestContourIdx]);
    // one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines

    // draw the rotated rect
    Point2f corners[4];
    canBoundingBox.points(corners);
    line(imgThresholded, corners[0], corners[1], Scalar(255,255,255));
    line(imgThresholded, corners[1], corners[2], Scalar(255,255,255));
    line(imgThresholded, corners[2], corners[3], Scalar(255,255,255));
    line(imgThresholded, corners[3], corners[0], Scalar(255,255,255));

    circle(imgThresholded,canBoundingBox.center,10,Scalar( 0, 0, 255 ));
    canRectCoords = canBoundingBox.center;

    cout << "Pomer stran: " <<canBoundingBox.size.width/canBoundingBox.size.height << endl;
//        << ", sirka: " <<canBoundingBox.size.width << ", vyska: " << canBoundingBox.size.height
//        << " rot: " << canBoundingBox.angle
//        << " x:" << canBoundingBox.center.x << " y:" << canBoundingBox.center.y << endl;

    // display
    imshow(OPENCV_WINDOW, imgThresholded); //show the thresholded image

        circle(imgOriginal,canBoundingBox.center,10,Scalar( 0, 0, 255 ));
        imshow("original", imgOriginal);
        // imshow("Control", drawing);
        //imshow("Control", imgOriginal);
        //imshow("Original", imgOriginal); //show the original image

        //end---edited */

        waitKey(15);

        // Output modified video stream
        //image_pub_.publish(cv_ptr->toImageMsg());
    }

    void depthImageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;

        //sensor_msgs::ImageConstPtr ffff = ros::topic::waitForMessage<sensor_msgs::ImageConstPtr>("/kinect2/sd/image_color_rect");

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, ""); // CV_16UC1//sensor_msgs::image_encodings::BGR8
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat imgOriginal = cv_ptr->image;
        Mat depthf; //(height, width, CV_8UC1);
        imgOriginal.convertTo(depthf, CV_8UC1, 255.0/2048.0);

        double scaleX = imgOriginal.cols/rgbCols;
        double scaleY = imgOriginal.rows/rgbRows;
        double canX = canBoundingBox.center.x * scaleX;
        double canY = canBoundingBox.center.y * scaleY;
//        Point p((int)canX, (int)canY);
        Point p = canBoundingBox.center;
//        double z = imgOriginal.at<double>(10, 10);
//        double z = imgOriginal.at((int)canY, (int)canX));
        actual_distance = imgOriginal.at<uint16_t>(p);
        cout << "depth at x="<< canX << " y=" << canY << ": " << actual_distance << endl;
       	vector3d realWorldCoords = get_handpos_xyz_from_cg((float)canX, (float)canY, (float)actual_distance);
		cout << "RealWorldCoord: depth at x="<< realWorldCoords.x << " y=" << realWorldCoords.y << ": " << realWorldCoords.z << endl;

        circle(depthf,p,10,255);
        imshow("depth", depthf);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;

    namedWindow(OPENCV_WINDOW,CV_WINDOW_NORMAL);
    namedWindow("original",CV_WINDOW_NORMAL);
    namedWindow("depth",CV_WINDOW_NORMAL);
    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", OPENCV_WINDOW, &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", OPENCV_WINDOW, &iHighH, 179);

    cvCreateTrackbar("LowS", OPENCV_WINDOW, &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", OPENCV_WINDOW, &iHighS, 255);

    cvCreateTrackbar("LowV", OPENCV_WINDOW, &iLowV, 255);//Value (0 - 255)
    cvCreateTrackbar("HighV", OPENCV_WINDOW, &iHighV, 255);


    ros::spin();
    return 0;
}