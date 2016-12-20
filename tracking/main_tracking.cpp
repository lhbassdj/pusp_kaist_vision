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
#include <pthread.h>

#include "CMT.h"

// Used for convert ardrone raw image to Mat format
#include "ImageConverter.h"

int main(int argc, char **argv)
{
    // Var for changing modes. mode_main[4] = {show_preview, mode_landing, mode_tracing, exact_landing, toggleview};
    bool mode_main[5] = {true, false, false, false, false};

    // Var for CMT initialization
    CMT cmt_main[3];
    Rect rect_main[3];

    // Counting frame number
    int frame_main = 0;

    // Number of initial active points
    int npoint_init_main = 0;

    // Left, right box width, height. wh_main[2] = {lwidth_init_main, lheight_init_main, rwidth_init_main, rheight_init_main};
    // New def. wh_main stores previous center position at 1 and 2.
    double cen_prev_main[2] = {0.0, 0.0};


	ros::init(argc, argv, "drone_tracking");
	ROS_INFO("Started TUM ArDrone Tracking Node.");

//    ImageConverter ic(show_preview_main, unbox_main, mode_landing_main, mode_tracking_main, cmt_main, rect_main, frame_main, npoint_init_main, leftmax_main, rightmax_main);

    ImageConverter ic(mode_main, cmt_main, rect_main, frame_main, npoint_init_main, cen_prev_main);

    ros::spin();
	return 0;
}
