#include "ros/ros.h"
#include <pthread.h>
// #include <image_transport/image_transport.h>
#include "std_msgs/Empty.h"
#include <cv_bridge/cv_bridge.h>

#include "CMT.h"
#include "gui.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstdio>
#include <string>
#include <math.h>
#include <vector>
#include <unistd.h>

#include <time.h>

#include "ImageConverter.h"

#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif
using namespace cv;

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using cv::Point;
using cv::Point2f;
using cv::findContours;
using std::cerr;
using std::istream;
using std::ifstream;
using std::stringstream;
using std::ofstream;
using std::cout;
using std::min_element;
using std::max_element;
using std::endl;
using ::atof;



static string OUT_FILE_COL_HEADERS =
	"Frame,Timestamp (ms),Active points,"\
	"Bounding box centre X (px),Bounding box centre Y (px),"\
	"Bounding box width (px),Bounding box height (px),"\
	"Bounding box rotation (degrees),"\
	"Bounding box vertex 1 X (px),Bounding box vertex 1 Y (px),"\
	"Bounding box vertex 2 X (px),Bounding box vertex 2 Y (px),"\
	"Bounding box vertex 3 X (px),Bounding box vertex 3 Y (px),"\
	"Bounding box vertex 4 X (px),Bounding box vertex 4 Y (px)";
pthread_mutex_t ImageConverter::send_CS = PTHREAD_MUTEX_INITIALIZER;

//typedef std::vector<Point2f>::const_iterator PointIt;

//bool &show_preview_main, bool &unbox_main, bool &mode_landing_main, bool &mode_tracking_main,

bool less_by_x(Point2f &lhs, Point2f &rhs)
{
    return lhs.x < rhs.x;
}

ImageConverter::ImageConverter(bool (&mode_main)[5],
                        CMT (&cmt_main)[3], Rect (&rect_main)[3], int &frame_main, int &npoint_init_main, double (&cen_prev_main)[2])
:it_(nh_)
{
    show_preview = mode_main[0];
    mode_landing = mode_main[1];
    mode_tracing = mode_main[2];
    exact_landing = mode_main[3];
    toggleview = mode_main[4];


    cmt = cmt_main[0];
    lcmt = cmt_main[1];
    rcmt = cmt_main[2];
    rect = rect_main[0];
    lrect = rect_main[1];
    rrect = rect_main[2];

	frame = frame_main;

    npoint_init = npoint_init_main;
    width_init = cen_prev_main[0];
    height_init = cen_prev_main[1];

	WIN_NAME = "CMT";
	WIN_NAME_SAMPLE = "sample image";

    // Subscrive to input video feed and publish output video feed. Video feed is /ardrone/image_raw.
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &ImageConverter::imageCallback, this);
    // image_sub_ = it_.subscribe(" 'Fill in this part as webcam feed' ", 1, &ImageConverter::imageCb, this);
}

ImageConverter::~ImageConverter()
{
	
}

void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		 ROS_ERROR("cv_bridge exception: %s", e.what());
		 return;
	}	
	
	Mat ardroneimage = cv_ptr->image;

    // Normal mode
    // Create window
    namedWindow(WIN_NAME);

	//Show preview until key is pressed
	if (show_preview)
	{
        Mat preview = ardroneimage;
        screenLog(preview, "Initial State. q to start OT, w to start AL");

        // Switch front cam if toggle view mode
        if (toggleview)
        {
            toggleCam_srv = nh_.serviceClient<std_srvs::Empty>(nh_.resolveName("ardrone/togglecam"),1);
            toggleCam_srv.call(toggleCam_srv_srvs);
        }

        // Given box in blue
        Point tl = Point(160,100);
        Point br = Point(480,260);
        rectangle(preview, tl, br, Scalar(255,0,0));

        // Display ardrone image
        imshow(WIN_NAME, preview);
		char k = waitKey(3);

        if (k == 'q')
        {
            CMT temp;
            cmt = temp;
            FILE_LOG(logINFO) << "Tracing mode. Initializing CMT with predefined box";
            // Parameter change to tracing mode
            mode_tracing = true;
            show_preview = false;

            // Initialization of CMT and Setting initial state values
            Mat preview_gray;
            if (preview.channels() > 1) {
                cvtColor(preview, preview_gray, CV_BGR2GRAY);
            } else {
                preview_gray = preview;
            }
            cmt.initialize(preview_gray, Rect(tl,br));
            cmt.consensus.estimate_rotation = true;
            npoint_init = cmt.points_active.size();
            Rect box_init = boundingRect(cmt.points_active);
            width_init = box_init.width;
            height_init = box_init.height;
		}
        else if (k == 'w')
        {
            CMT temp;
            lcmt = temp;
            FILE_LOG(logINFO) << "Landing mode. Initializing CMT with predefined image";
            // Parameter change to landing mode
            mode_landing = true;
            show_preview = false;

            Mat preview_gray;
            preview = imread("/home/xs3d/real.png", CV_LOAD_IMAGE_ANYDEPTH);
            if (preview.channels() > 1) {
                cvtColor(preview, preview_gray, CV_BGR2GRAY);
            } else {
                preview_gray = preview;
            }
            lcmt.initialize(preview_gray, Rect(tl,br));
            lcmt.consensus.estimate_rotation = true;
            npoint_init = lcmt.points_active.size();
            Rect box_init = boundingRect(lcmt.points_active);
            width_init = box_init.width;
            height_init = box_init.height;
        }
        else
        {
            //
        }
	}
    else if (mode_tracing)
    {
        // Main loop counter
        frame++;

        // Process CMT
        Mat im = ardroneimage;
        Mat im_gray;
        if (im.channels() > 1) {
            cvtColor(im, im_gray, CV_BGR2GRAY);
        } else {
            im_gray = im;
        }
        cmt.processFrame(im_gray);

        // Data from CMT
        Point2f center = cmt.center_pub;
        double cen_x = center.x;
        double cen_y = center.y;
        int npoints = cmt.points_active.size();

        // Display current center of the object
        drawObject(cen_x, cen_y, im);

        // cmd_vel message. xmove, ymove, zmove, angzmove for file-logging
        geometry_msgs::Twist cmdT;
        double xmove, ymove, zmove, angzmove;

        // Initialization cmdT message
        cmdT.linear.x = 0;
        cmdT.linear.y = 0;
        cmdT.linear.z = 0;
        cmdT.angular.z = 0;

        // Threshold value for xratio and yratio
        double max_thr = 1.05;
        double min_thr = 0.95;

        // Action when detected
        if (npoints != 0)
        {
            // Make box to calculate the area
            Rect box = boundingRect(cmt.points_active);
            double width = box.width;
            double height = box.height;

            // Parameter for decision. Xratio: compare the area of rectangle, Yratio: compare x position of the center
            double xratio = width / width_init;
            double yratio = cen_x / 320.0;

            // Decision
            if (xratio > max_thr)
            {
                cmdT.linear.x = -0.5;
                xmove = -1;
            }
            else if(xratio < min_thr)
            {
                cmdT.linear.x = 0.5;
                xmove = 1;
            }
            else
                cmdT.linear.x = 0;

            if (yratio > max_thr)
            {
                cmdT.linear.y = -0.5;
                ymove = -1;
            }
            else if (yratio < min_thr)
            {
                cmdT.linear.y = 0.5;
                ymove = 1;
            }
            else
                cmdT.linear.y = 0;

            FILE_LOG(logINFO) << "Current npoints : " << npoints << ", initial value: " << npoint_init;
            FILE_LOG(logINFO) << "Initial width: " << width_init << ", height: " << height_init;
            FILE_LOG(logINFO) << "Current width : " << width << ", height: " << height;
            FILE_LOG(logINFO) << "Xratio : " << xratio << ", Yratio: " << yratio;
            FILE_LOG(logINFO) << "Movement - x:" << xmove << ", y:" << ymove << ", z:" << zmove << ", yawz:" << angzmove;

            // Actual Movement
    //        pthread_mutex_lock(&send_CS);
            vel_pub	   = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("cmd_vel"),1);
            vel_sub	   = nh_.subscribe(nh_.resolveName("cmd_vel"),50, &ImageConverter::velCb, this);
            vel_pub.publish(cmdT);
    //        pthread_mutex_unlock(&send_CS);


        }

        // Display CMT with current image
        screenLog(im, "Object Tracing Mode. Press e to preview mode");
        char key = display(im, cmt);

        // Escaping mode
        if (key == 'e')
        {
            show_preview = true;
            mode_tracing = false;
        }

        // Frequency of tracing mode
        ros::Duration(1).sleep();
    }
    else if(mode_landing)
    {
        // Main loop counter
        frame++;

        // Process of CMT
        Mat im = ardroneimage;

        char key;
        // Action when detected
        if (exact_landing)
        {
            // Get into toggleview mode
            if (!toggleview)
            {
                toggleCam_srv = nh_.serviceClient<std_srvs::Empty>(nh_.resolveName("ardrone/togglecam"),1);
                toggleCam_srv.call(toggleCam_srv_srvs);
                toggleview = true;
                FILE_LOG(logINFO) << "Camera switched from front to bottom";
            }

            // Process of rcmt
            Mat im_gray;
            if (im.channels() > 1) {
                cvtColor(im, im_gray, CV_BGR2GRAY);
            } else {
                im_gray = im;
            }
            rcmt.processFrame(im_gray);

            // Data from rcmt
            Point2f center_tog = rcmt.center_pub;
            double cen_x_tog = center_tog.x;
            double cen_y_tog = center_tog.y;

            drawObject(cen_x_tog, cen_y_tog, im);

            double xmove = 0;
            double ymove = 0;

            // Land automatic if the center is at the center of vision
            if (cen_x_tog >= 320 * 1.3)
            {
                ymove = 1;

            }
            else if (cen_x_tog <= 320 * 0.7)
            {
                ymove = -1;
            }
            else
                ymove = 0;

            if (cen_y_tog >= 180 * 1.3)
            {
                ymove = 1;

            }
            else if (cen_y_tog <= 180 * 0.7)
            {
                ymove = -1;
            }
            else
                ymove = 0;

            if (xmove == 0 && ymove == 0)
            {
                land_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/land"),1);
                land_sub = nh_.subscribe(nh_.resolveName("ardrone/land"),1, &ImageConverter::landCb, this);
                land_pub.publish(std_msgs::Empty());
            }

            screenLog(im, "Exact Landing mode. Press e to preview mode");
            key = display(im, rcmt);

        }
//        else if ((npoints > npoint_init*0.7 || npoints < npoint_init*1.1) && !exact_landing)
        else if (true)
        {
            // Enable exact_landing mode
            exact_landing = true;

            // Initialize rcmt for exact landing. rcmt is for toggle cam.
            Point tl = Point(300,100);
            Point br = Point(450,180);
            CMT temp;
            rcmt = temp;
            FILE_LOG(logINFO) << "Exact Landing mode. Initializing CMT with predefined image";

            Mat preview_gray;
            Mat preview = imread("/home/xs3d/real2.png", CV_LOAD_IMAGE_ANYDEPTH);
            if (preview.channels() > 1) {
                cvtColor(preview, preview_gray, CV_BGR2GRAY);
            } else {
                preview_gray = preview;
            }
            rcmt.initialize(preview_gray, Rect(tl,br));
            rcmt.consensus.estimate_rotation = true;
        }
        else
        {
            Mat im_gray;
            if (im.channels() > 1) {
                cvtColor(im, im_gray, CV_BGR2GRAY);
            } else {
                im_gray = im;
            }
            lcmt.processFrame(im_gray);

            // Data from CMT
            Point2f center = lcmt.center_pub;
            double cen_x = center.x;
            double cen_y = center.y;
            int npoints = lcmt.points_active.size();

            // Display current center of the object
            drawObject(cen_x, cen_y, im);


            // Display lcmt with current image
            FILE_LOG(logINFO) << "Current npoints : " << npoints << ", initial value: " << npoint_init;
            screenLog(im, "Automatic Landing mode. Press e to preview mode");
            key = display(im, lcmt);
        }







        // Escaping mode
        if(key == 'e')
        {
            show_preview = true;
            mode_landing = false;
            toggleview = false;
        }

        // Frequency of landing mode
        ros::Duration(0.5).sleep();
	}
    else
    {
        //
    }

}

vector<float> ImageConverter::getNextLineAndSplitIntoFloats(istream& str)
{
	vector<float>   result;
	string                line;
	getline(str,line);

	stringstream          lineStream(line);
	string                cell;
	while(getline(lineStream,cell,','))
	{
		result.push_back(atof(cell.c_str()));
	}
	return result;
}

int ImageConverter::display(Mat im, CMT & cmt)
{
	//Visualize the output
	//It is ok to draw on im itself, as CMT only uses the grayscale image
	for(size_t i = 0; i < cmt.points_active.size(); i++)
	{
		circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
	}

	Point2f vertices[4];
	cmt.bb_rot.points(vertices);
	for (int i = 0; i < 4; i++)
	{
		line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
	}

	imshow(WIN_NAME, im);

	return waitKey(5);
}

string ImageConverter::write_rotated_rect(RotatedRect rect)
{
	Point2f verts[4];
	rect.points(verts);
	stringstream coords;

	coords << rect.center.x << "," << rect.center.y << ",";
	coords << rect.size.width << "," << rect.size.height << ",";
	coords << rect.angle << ",";

	for (int i = 0; i < 4; i++)
	{
		coords << verts[i].x << "," << verts[i].y;
		if (i != 3) coords << ",";
	}

	return coords.str();
}

void ImageConverter::drawObject(int x, int y, Mat &frame)
{
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!


	//'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

}

void ImageConverter::landCb(std_msgs::EmptyConstPtr)
{
// gui->addLogLine("sent: LAND");
}
void ImageConverter::velCb(const geometry_msgs::TwistConstPtr vel)
{
//
}

string ImageConverter::intToString(int number) {

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}
