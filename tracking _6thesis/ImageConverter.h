 #pragma once

 /**
 *  This file is part of pusc_kaist.
 *
 *  Copyright 2016 Dongjin Kim <wking@kaist.ac.kr> (KAIST)
 *
 *  pusc_kaist is free software: it's based on tum_ardrone from TUM
 *  (Technical University of Munich).
 *  Like tum_ardrone, pusc_kaist is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with pusc_kaist.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ImageConverter_H
#define ImageConverter_H

#include "ros/ros.h"
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <pthread.h>
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"


#include "gui.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif


// #include "std_msgs/Empty.h"

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
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

class ImageConverter
{
private:
    // Var from cv_bridge tutorial.
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

    // Message for ARDrone moving
    ros::Publisher land_pub;
	ros::Subscriber land_sub;
    ros::Publisher vel_pub;
    ros::Subscriber vel_sub;

    // Window name
    string WIN_NAME;
	string WIN_NAME_SAMPLE;

    // Var for changing mode
	bool show_preview;
	bool unbox;
	bool mode_landing;
	bool mode_tracking;

    // Var for CMT initialization
	CMT cmt;
    CMT lcmt;
    CMT rcmt;

	Rect rect;
    Rect lrect;
    Rect rrect;

    // Counting frame number
    int frame;

    // Tracking parameter: Number of initial active points, initial width, and initial height
    int npoint_init;
    double cen_x_prev;
    double cen_y_prev;


    // Var for send_CS
    static pthread_mutex_t send_CS;

public:
    ImageConverter(bool (&mode_main)[4],
                        CMT (&cmt_main)[3], Rect (&rect_main)[3], int &frame_main,
                            int &npoint_init_main, double (&cen_prev_main)[2]);
	~ImageConverter();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	vector<float> getNextLineAndSplitIntoFloats(istream& str);
	int display(Mat im, CMT & cmt);
	string write_rotated_rect(RotatedRect rect);
	void drawObject(int x, int y, cv::Mat &frame);
	void landCb(std_msgs::EmptyConstPtr);
    void velCb(const geometry_msgs::TwistConstPtr vel);
	string intToString(int number);
//    bool less_by_x(Point2f &lhs, Point2f &rhs);
};
#endif
