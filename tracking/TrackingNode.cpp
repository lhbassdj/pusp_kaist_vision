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
 
#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
// #include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encoding.h>
#include <image_transport/image_transport.h>
#include <pthread.h>
#include "std_msgs/Empty.h"
#include "TrackingNode.h"

using namespace cv;
using namespace std;

#include "CMT.h"
#include "gui.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif

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

static string WIN_NAME = "CMT";
static string OUT_FILE_COL_HEADERS =
    "Frame,Timestamp (ms),Active points,"\
    "Bounding box centre X (px),Bounding box centre Y (px),"\
    "Bounding box width (px),Bounding box height (px),"\
    "Bounding box rotation (degrees),"\
    "Bounding box vertex 1 X (px),Bounding box vertex 1 Y (px),"\
    "Bounding box vertex 2 X (px),Bounding box vertex 2 Y (px),"\
    "Bounding box vertex 3 X (px),Bounding box vertex 3 Y (px),"\
    "Bounding box vertex 4 X (px),Bounding box vertex 4 Y (px)";

vector<float> getNextLineAndSplitIntoFloats(istream& str)
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

int display(Mat im, CMT & cmt)
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

string write_rotated_rect(RotatedRect rect)
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


TrackingNode::TrackingNode()
: it_(nh_)
{
	// Subscrive to input video feed and publish output video feed
	// Video feed is /ardrone/image_raw.
	image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &TrackingNode::callback(const sensor_msgs::ImageConstPtr& msg), this);
	// image_sub_ = it_.subscribe(" 'Fill in this part as webcam feed' ", 1, &ImageConverter::imageCb, this);
	image_pub_ = it_.advertise("/image_converter/output_video", 1);
}

TrackingNode::~TrackingNode()
{

}

void TrackingNode::callback(const sensor_msgs::ImageConstPtr& msg)
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


	image_pub_.publish(cv_ptr->toImageMsg());
}