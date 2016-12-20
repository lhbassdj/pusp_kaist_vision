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

ImageConverter::ImageConverter(bool (&mode_main)[4],
                        CMT (&cmt_main)[3], Rect (&rect_main)[3], int &frame_main, int &npoint_init_main, double (&cen_prev_main)[2])
:it_(nh_)
{
//	show_preview = show_preview_main;
//	unbox = unbox_main;
//	mode_landing = mode_landing_main;
//	mode_tracking = mode_tracking_main;

    show_preview = mode_main[0];
    unbox = mode_main[1];
    mode_landing = mode_main[2];
    mode_tracking = mode_main[3];


    cmt = cmt_main[0];
    lcmt = cmt_main[1];
    rcmt = cmt_main[2];
    rect = rect_main[0];
    lrect = rect_main[1];
    rrect = rect_main[2];

	frame = frame_main;

    npoint_init = npoint_init_main;
    cen_x_prev = cen_prev_main[0];
    cen_y_prev = cen_prev_main[1];

	WIN_NAME = "CMT";
	WIN_NAME_SAMPLE = "sample image";
	// Subscrive to input video feed and publish output video feed
	// Video feed is /ardrone/image_raw.
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

	// //Create a CMT object
	// CMT cmt;

	// //Initialization bounding box
	// Rect rect;

	//Parse args
	int challenge_flag = 0;
	int loop_flag = 0;
	int verbose_flag = 0;
	int bbox_flag = 0;
	int skip_frames = 0;
	int skip_msecs = 0;
	int output_flag = 0;
	string input_path;
	string output_path;

	const int detector_cmd = 1000;
	const int descriptor_cmd = 1001;
	const int bbox_cmd = 1002;
	const int no_scale_cmd = 1003;
	const int with_rotation_cmd = 1004;
	const int skip_cmd = 1005;
	const int skip_msecs_cmd = 1006;
	const int output_file_cmd = 1007;

	// struct option longopts[] =
	// {
	// 	//No-argument options
	// 	{"challenge", no_argument, &challenge_flag, 1},
	// 	{"loop", no_argument, &loop_flag, 1},
	// 	{"verbose", no_argument, &verbose_flag, 1},
	// 	{"no-scale", no_argument, 0, no_scale_cmd},
	// 	{"with-rotation", no_argument, 0, with_rotation_cmd},
	// 	//Argument options
	// 	{"bbox", required_argument, 0, bbox_cmd},
	// 	{"detector", required_argument, 0, detector_cmd},
	// 	{"descriptor", required_argument, 0, descriptor_cmd},
	// 	{"output-file", required_argument, 0, output_file_cmd},
	// 	{"skip", required_argument, 0, skip_cmd},
	// 	{"skip-msecs", required_argument, 0, skip_msecs_cmd},
	// 	{0, 0, 0, 0}
	// };

	// int index = 0;
	// int c;
	// while((c = getopt_long(argc, argv, "v", longopts, &index)) != -1)
	// {
	// 	switch (c)
	// 	{
	// 		case 'v':
	// 			verbose_flag = true;
	// 			break;
	// 		case bbox_cmd:
	// 			{
	// 				//TODO: The following also accepts strings of the form %f,%f,%f,%fxyz...
	// 				string bbox_format = "%f,%f,%f,%f";
	// 				float x,y,w,h;
	// 				int ret = sscanf(optarg, bbox_format.c_str(), &x, &y, &w, &h);
	// 				if (ret != 4)
	// 				{
	// 					cerr << "bounding box must be given in format " << bbox_format << endl;
	// 					// return 1;
	// 				}

	// 				bbox_flag = 1;
	// 				rect = Rect(x,y,w,h);
	// 			}
	// 			break;
	// 		case detector_cmd:
	// 			cmt.str_detector = optarg;
	// 			break;
	// 		case descriptor_cmd:
	// 			cmt.str_descriptor = optarg;
	// 			break;
	// 		case output_file_cmd:
	// 			output_path = optarg;
	// 			output_flag = 1;
	// 			break;
	// 		case skip_cmd:
	// 			{
	// 				int ret = sscanf(optarg, "%d", &skip_frames);
	// 				if (ret != 1)
	// 				{
	// 				  skip_frames = 0;
	// 				}
	// 			}
	// 			break;
	// 		case skip_msecs_cmd:
	// 			{
	// 				int ret = sscanf(optarg, "%d", &skip_msecs);
	// 				if (ret != 1)
	// 				{
	// 				  skip_msecs = 0;
	// 				}
	// 			}
	// 			break;
	// 		case no_scale_cmd:
	// 			cmt.consensus.estimate_scale = false;
	// 			break;
	// 		case with_rotation_cmd:
	// 			cmt.consensus.estimate_rotation = true;
	// 			break;
	// 		case '?':
	// 			// return 1;
	// 			break;
	// 	}

	// }

	// // Can only skip frames or milliseconds, not both.
	// if (skip_frames > 0 && skip_msecs > 0)
	// {
	//   cerr << "You can only skip frames, or milliseconds, not both." << endl;
	//   // return 1;
	// }

	// //One argument remains
	// if (optind == argc - 1)
	// {
	// 	input_path = argv[optind];
	// }

	// else if (optind < argc - 1)
	// {
	// 	cerr << "Only one argument is allowed." << endl;
	// 	// return 1;
	// }

	// //Set up logging
	// FILELog::ReportingLevel() = verbose_flag ? logDEBUG : logINFO;
	// Output2FILE::Stream() = stdout; //Log to stdout

	// //Challenge mode
	// if (challenge_flag)
	// {
	// 	//Read list of images
	// 	ifstream im_file("images.txt");
	// 	vector<string> files;
	// 	string line;
	// 	while(getline(im_file, line ))
	// 	{
	// 		files.push_back(line);
	// 	}

	// 	//Read region
	// 	ifstream region_file("region.txt");
	// 	vector<float> coords = getNextLineAndSplitIntoFloats(region_file);

	// 	if (coords.size() == 4) {
	// 		rect = Rect(coords[0], coords[1], coords[2], coords[3]);
	// 	}

	// 	else if (coords.size() == 8)
	// 	{
	// 		//Split into x and y coordinates
	// 		vector<float> xcoords;
	// 		vector<float> ycoords;

	// 		for (size_t i = 0; i < coords.size(); i++)
	// 		{
	// 			if (i % 2 == 0) xcoords.push_back(coords[i]);
	// 			else ycoords.push_back(coords[i]);
	// 		}

	// 		float xmin = *min_element(xcoords.begin(), xcoords.end());
    // 		float xmax = *max_element(xcoords.begin(), xcoords.end());
	// 		float ymin = *min_element(ycoords.begin(), ycoords.end());
	// 		float ymax = *max_element(ycoords.begin(), ycoords.end());

	// 		rect = Rect(xmin, ymin, xmax-xmin, ymax-ymin);
	// 		cout << "Found bounding box" << xmin << " " << ymin << " " <<  xmax-xmin << " " << ymax-ymin << endl;
	// 	}

	// 	else {
	// 		cerr << "Invalid Bounding box format" << endl;
	// 		// return 0;
	// 	}

	// 	//Read first image
	// 	Mat im0 = imread(files[0]);
	// 	Mat im0_gray;
	// 	cvtColor(im0, im0_gray, CV_BGR2GRAY);

	// 	//Initialize cmt
	// 	cmt.initialize(im0_gray, rect);

	// 	//Write init region to output file
	// 	ofstream output_file("output.txt");
	// 	output_file << rect.x << ',' << rect.y << ',' << rect.width << ',' << rect.height << std::endl;

	// 	//Process images, write output to file
	// 	for (size_t i = 1; i < files.size(); i++)
	// 	{
	// 		FILE_LOG(logINFO) << "Processing frame " << i << "/" << files.size();
	// 		Mat im = imread(files[i]);
	// 		Mat im_gray;
	// 		cvtColor(im, im_gray, CV_BGR2GRAY);
	// 		cmt.processFrame(im_gray);
	// 		if (verbose_flag)
	// 		{
	// 			display(im, cmt);
	// 		}
	// 		rect = cmt.bb_rot.boundingRect();
	// 		output_file << rect.x << ',' << rect.y << ',' << rect.width << ',' << rect.height << std::endl;
	// 	}

	// 	output_file.close();

	// 	// return 0;
	// }

	//Normal mode
	//Create window
	namedWindow(WIN_NAME);
	// namedWindow(WIN_NAME_SAMPLE);

	// VideoCapture cap;

	// //If no input was specified
	// if (input_path.length() == 0)
	// {
 //     		// cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('H','2','6','4'));
	// 	cap.open(0); //Open default camera device

	// }

	// //Else open the video specified by input_path
	// else
	// {
	// 	cap.open(input_path);

	// 	if (skip_frames > 0)
	// 	{
	// 	  cap.set(CV_CAP_PROP_POS_FRAMES, skip_frames);
	// 	}

	// 	if (skip_msecs > 0)
	// 	{
	// 	  cap.set(CV_CAP_PROP_POS_MSEC, skip_msecs);

	// 	  // Now which frame are we on?
	// 	  skip_frames = (int) cap.get(CV_CAP_PROP_POS_FRAMES);
	// 	}

	// 	show_preview = false;
	// }

	// //If it doesn't work, stop
	// if(!cap.isOpened())
	// {
	// 	cerr << "Unable to open video capture." << endl;
	// 	// return -1;
	// }

	//Show preview until key is pressed
	if (show_preview)
	{
		Mat preview;
		// cap >> preview;
		preview = ardroneimage;

		screenLog(preview, "Press a key to start selecting an object.");
		imshow(WIN_NAME, preview);

		char k = waitKey(3);
		if (k != -1) {
			show_preview = false;
		}

//        ros::Duration(0.5).sleep();
	}
	else if (!show_preview && unbox)
	{
		//Get initial image
		Mat im0;
		// cap >> im0;

		Mat sampleimage = imread("/home/xs3d/pattern2.png", CV_LOAD_IMAGE_ANYDEPTH);

		// im0 = sampleimage;
		im0 = ardroneimage;
		
		//If no bounding was specified, get it from user
		if (!bbox_flag)
		{
			rect = getRect(im0, WIN_NAME);
//            lrect = getRect(im0, WIN_NAME);
//            rrect = getRect(im0, WIN_NAME);
		}

		FILE_LOG(logINFO) << "Using " << rect.x << "," << rect.y << "," << rect.width << "," << rect.height
			<< " as initial bounding box.";

		//Convert im0 to grayscale
		Mat im0_gray;
		if (im0.channels() > 1) {
			cvtColor(im0, im0_gray, CV_BGR2GRAY);
		} else {
			im0_gray = im0;
		}

		//Initialize CMT
		cmt.initialize(im0_gray, rect);
        cmt.consensus.estimate_rotation = true;

//        lcmt.initialize(im0_gray, lrect);
//        lcmt.consensus.estimate_rotation = true;

//        rcmt.initialize(im0_gray, rrect);
//        rcmt.consensus.estimate_rotation = true;


        npoint_init = cmt.points_active.size();

//        Point2f center_init = cmt.center_pub;
//        vector<Point2f> left_points_init;
//        vector<Point2f> right_points_init;
//        for (int i=0;i<npoint_init;i++)
//        {
//            Point2f point_init = cmt.points_active[i];
//            if (point_init.x > center_init.x)
//                right_points_init.push_back(point_init);
//            else if (point_init.x < center_init.x)
//                left_points_init.push_back(point_init);
//            else
//                true;
//        }

//        Rect left_box_init = boundingRect(left_points_init);
//        Rect right_box_init = boundingRect(right_points_init);

//        lwidth_init = left_box_init.width;
//        lheight_init = left_box_init.height;

//        rwidth_init = right_box_init.width;
//        rheight_init = right_box_init.height;

        Rect box_init = boundingRect(cmt.points_active);

        lrect = box_init;
//        lwidth_init = box_init.width/2;
//        lheight_init = box_init.height;

//        rwidth_init = box_init.width/2;
//        rheight_init = box_init.height;

//        cen_x_prev = box_init.width;
//        cen_y_prev = box_init.height;

        cen_x_prev = 0;
        cen_y_prev = 0;
		unbox = false;
	}
	else
	{
		// //Open output file.
		// ofstream output_file;

		// if (output_flag)
		// {
        // 	// int msecs = (int) cap.get(CV_CAP_PROP_P _MSEC);

		// 	output_file.open(output_path.c_str());
		// 	output_file << OUT_FILE_COL_HEADERS << endl;
		// 	// output_file << frame << "," << msecs << ",";
		// 	output_file << cmt.points_active.size() << ",";
		// 	output_file << write_rotated_rect(cmt.bb_rot) << endl;
		// }

		//Main loop
		frame++;

		Mat im;

		//If loop flag is set, reuse initial image (for debugging purposes)
		// if (loop_flag) im0.copyTo(im);
		// else 

		im = ardroneimage;

		// cap >> im; //Else use next image in stream

		// if (im.empty()) break; //Exit at end of video stream

		Mat im_gray;
		if (im.channels() > 1) {
			cvtColor(im, im_gray, CV_BGR2GRAY);
		} else {
			im_gray = im;
		}

		//Let CMT process the frame
		cmt.processFrame(im_gray);

//        lcmt.processFrame(im_gray);
//        rcmt.processFrame(im_gray);

		//Output.
		// if (output_flag)
		// {
		// 	// // int msecs = (int) cap.get(CV_CAP_PROP_POS_MSEC);
		// 	// // output_file << frame << "," << msecs << ",";
		// 	// output_file << cmt.points_active.size() << ",";
		// 	// output_file << write_rotated_rect(cmt.bb_rot) << endl;
		// }
		// else
		// {
			//TODO: Provide meaningful output


        int npoints = cmt.points_active.size();
//        int npoints_l = lcmt.points_active.size();
//        int npoints_r = rcmt.points_active.size();
//        FILE_LOG(logINFO) << "#" << frame << " active: " << npoints;
//        FILE_LOG(logINFO) << "#" << frame << " active: " << npoints_l;
//        FILE_LOG(logINFO) << "#" << frame << " active: " << npoints_r;
//        FILE_LOG(logINFO) << "Left wiidth: " << lwidth_init << ", height: " << lheight_init;
//        FILE_LOG(logINFO) << "Right wiidth: " << rwidth_init << ", height: " << rheight_init;

		Point2f center = cmt.center_pub;
//        Size2f size = cmt.bb_rot.size;

        double center_x = center.x;
        double center_y = center.y;

//        vector<Point2f> left_points;
//        vector<Point2f> right_points;
//        for (int i=0;i<npoints;i++)
//        {
//            Point2f point = cmt.points_active[i];
//            if (point.x > x)
//                right_points.push_back(point);
//            else if (point.x < x)
//                left_points.push_back(point);
//            else
//                true;
//        }



//        auto x_comparator = [](const Point2f &a, const Point2f &b) { return a.x < b.x; };

        vector<Point2f> points = cmt.points_active;
//        float max_x = boost::max_element(cmt.points_active, x_comparator)->pt.x;
//        float min_x = boost::min_element(cmt.points_active, x_comparator)->pt.x;

//        bool (*leftptr) (const Point2f&, const Point2f&) = less_by_x;

        Point2f maxpt = *max_element(points.begin(), points.end(), less_by_x);
        Point2f minpt = *min_element(points.begin(), points.end(), less_by_x);

//        itr_min = std::min_element(cmt.points_active.begin(), cmt.points_active.end());

        FILE_LOG(logINFO) << "Maximum x value: " << maxpt.x;
        FILE_LOG(logINFO) << "Minimum x value: " << minpt.x;

        Rect box = boundingRect(cmt.points_active);
//        Rect lbox = boundingRect(lcmt.points_active);
//        Rect rbox = boundingRect(rcmt.points_active);
//        double rwidth = maxpt.x - center_x;
//        double lidth = center.x - minpt.x;

//        double lwidth = lbox.width;
//        double rwidth = rbox.width;
//        FILE_LOG(logINFO) << "l box width : " << lwidth;
//        FILE_LOG(logINFO) << "r box width : " << rwidth;


//        Rect left_box = boundingRect(left_points);
//        Rect right_box = boundingRect(right_points);


        drawObject(center_x, center_y, im);

		//Display image and then quit if requested.
		if (mode_landing)
		{
			screenLog(im, "Landing Mode. Press e to escape.");
			char key = display(im, cmt);
            if (npoints > npoint_init*0.9 && npoints < npoint_init*1.1)
			{
				screenLog(im, "Detected and landed");
                land_pub = nh_.advertise<std_msgs::Empty>(nh_.resolveName("ardrone/land"),1);
                land_sub = nh_.subscribe(nh_.resolveName("ardrone/land"),1, &ImageConverter::landCb, this);
                land_pub.publish(std_msgs::Empty());
            }
            if(key == 'e')
            {
                mode_landing = false;
                imshow(WIN_NAME, im);
            }
        }
        else if (mode_tracking)
        {
//            screenLog(im, "Tracking Mode. Press e to escape");
            char key = display(im, cmt);

            // TODO: check converstion (!)
            geometry_msgs::Twist cmdT;
            double xmove;
            double ymove;
            double zmove;
            double angzmove;

            double xratio;
            double yratio;

//            if (cen_x_prev != 0)
//                xratio = cen_x_prev / center_x;
//            else
//                xratio = 1;

//            if (cen_y_prev != 0)
//                yratio = cen_y_prev / center_y;
//            else
//                yratio = 1;F

            cmdT.linear.x = 0;
            cmdT.linear.y = 0;
            cmdT.linear.z = 0;
            cmdT.angular.z = 0;

            double width = box.width;
            double height = box.height;

            Rect box_init = lrect;
            xratio = double(box.width) / double(box_init.width);
            yratio = center_x / 320.0;

            double max_thr = 1.05;
            double min_thr = 0.95;


            if (xratio > max_thr)
            {
                cmdT.linear.x = -1;
                xmove = -1;
            }
            else if(xratio < min_thr)
            {
                xmove = 1;
                cmdT.linear.x = 1;
            }
            else
                cmdT.linear.x = 0;

            if (yratio > max_thr)
            {
                ymove = -1;
                cmdT.linear.y = -1;
            }
            else if (yratio < min_thr)
            {
                ymove = 1;
                cmdT.linear.y = 1;
            }
            else
                cmdT.linear.y = 0;


//            if (yratio > 1.05)
//            {
//                if (xratio > 1.025)
//                {
//                    cmdT.linear.x = -0.5;
//                    cmdT.linear.y = 0.3;
//                    cmdT.angular.z = -0.2;
//                    xmove = 1;
//                    ymove = 1;
//                    angzmove = -1;
//                }
//                else if (xratio < 0.975)
//                {
//                    cmdT.linear.x = -0.5;
//                    cmdT.linear.y = -0.3;
//                    cmdT.angular.z = 0.2;
//                    xmove = 1;
//                    ymove = -1;
//                    angzmove = 1;
//                }
//                else
//                {
//                    cmdT.linear.y = -0.5;
//                    ymove = -1;
//                }
//            }
//            else if (yratio < 0.95)
//            {
//                if (xratio > 1.025)
//                {
//                    cmdT.linear.x = 0.5;
//                    cmdT.linear.y = 0.3;
//                    cmdT.angular.z = -0.2;
//                    xmove = -1;
//                    ymove = 1;
//                    angzmove = -1;
//                }
//                else if (xratio < 0.975)
//                {
//                    cmdT.linear.x = 0.5;
//                    cmdT.linear.y = -0.3;
//                    cmdT.angular.z = 0.2;
//                    xmove = -1;
//                    ymove = -1;
//                    angzmove = 1;
//                }
//                else
//                {
//                    cmdT.linear.y = 0.5;
//                    ymove = 1;
//                }
//            }



            /*
            // X-axis movement
            if (box.height >= lheight_init*1.1)
            {
                cmdT.linear.x = -0.5;
                xmove = -1;
            }
            else if (box.height <= lheight_init*0.9)
            {
                cmdT.linear.x = 0.5;
                xmove = 1;
            }
            else
            {
                cmdT.linear.x = 0.0;
                xmove = 0.0;
            }

            // Z-axis movement
            if (center_y>=180*1.1)
            {
                cmdT.linear.z = -0.1;
                zmove = -1;
            }
            else if (center_y <= 180*0.9)
            {
                cmdT.linear.z = 0.1;
                zmove = 1;
            }
            else
            {
                cmdT.linear.z = 0.0;
                zmove = 0.0;
            }*/

            // Y-axis and Yawing movement
//            double rlratio = rwidth / lwidth;

//            int largest_area = 0;
//            int largest_contour_index =0;
//            Rect bounding_rect;
//            Mat thr(im.rows, im.cols, CV_8UC1);
//            Mat dst(im.rows, im.cols, CV_8UC1, Scalar::all(0));
//            cvtColor(im, thr, CV_BGR2GRAY);
//            threshold(thr, thr, 25, 255, THRESH_BINARY);

//            vector< vector<Point> > contours;
//            vector<Vec4i> hierarchy;

//            findContours(thr, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

//            for (int i=0; i<contours.size(); i++)
//            {
//                double a = contourArea(contours[i], false);
//                if (a > largest_area)
//                {
//                    largest_area = a;
//                    largest_contour_index = i;
//                    bounding_rect = boundingRect(contours[i]);
//                }
//            }
//            Scalar color(255, 255, 255);
//            drawContours(dst, contours, largest_contour_index, color, CV_FILLED, 8, hierarchy);
//            rectangle(im, bounding_rect, Scalar(0,255,0), 1, 8, 0);
//            imshow("src", im);
//            imshow("largest Contour", dst);
//            waitKey(3);

//            if (rlratio >= 0.8 && rlratio <= 1.2)
//            {
//                // Moving sideways. simple.
//                if (center_x >= 320*1.1)
//                {
//                    cmdT.linear.y = 0.1;
//                    ymove = 1;
//                }
//                else if (center_x <= 320*0.9)
//                {
//                    cmdT.linear.y = -0.1;
//                    ymove = -1;
//                }
//                else
//                {
//                    cmdT.linear.y = 0.0;
//                    ymove = 0;
//                }
//            }
//            else if(rwidth/lwidth > 1.2)
//            {
//                cmdT.linear.y = 0.1;
//                cmdT.angular.z = -0.1;
//                ymove = 1;
//                angzmove = -1;
//            }
//            else if (rwidth/lwidth < 0.8)
//            {
//                cmdT.linear.y = -0.1;
//                cmdT.angular.z = 0.1;
//                ymove = -1;
//                angzmove = -1;
//            }
//            else
//            {
//                ymove = 0;
//                angzmove = 0;
//            }

            FILE_LOG(logINFO) << "xratio : " << xratio << ", y:" << yratio;
            FILE_LOG(logINFO) << "box init width : " << box_init.width << ", y:" << box.width;
            FILE_LOG(logINFO) << "Movement - x:" << xmove << ", y:" << ymove << ", z:" << zmove << ", yawz:" << angzmove;
//            std::stringstream ss;
//            ss << "Movement - x:" << xmove << ", y:" << ymove << ", z:" << zmove << ", yawz:" << angzmove;

//            cmdT.linear.z = 0;
            cmdT.angular.z = 0;
//            cmdT.linear.x = 0;

//                screenLog(im, ss.str());
//            cmdT.angular.x = cmdT.angular.y = gui->useHovering ? 0 : 1;
            if (true)
            {
//                pthread_mutex_lock(&send_CS);

                vel_pub	   = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("cmd_vel"),1);
                vel_sub	   = nh_.subscribe(nh_.resolveName("cmd_vel"),50, &ImageConverter::velCb, this);
                vel_pub.publish(cmdT);
//                pthread_mutex_unlock(&send_CS);
            }


			if(key == 'e')
			{
				mode_tracking = false;
				imshow(WIN_NAME, im);
			}

            ros::Duration(0.4).sleep();
		}
		else
        {
            screenLog(im, "Press q to autonomous landing, w to tracking, r to initial state");
//            screenLog(im, intToString(mmx));
//            string left_box_width = intToString((int) left_box.width);
//            string right_box_width = intToString((int) right_box.width);

//            screenLog(im, left_box_width.append(right_box_width));
			char key = display(im, cmt);
			if(key == 'q')
			{
				mode_landing = true;
				imshow(WIN_NAME, im);
            }
            else if(key == 'w')
			{
				mode_tracking = true;
				imshow(WIN_NAME, im);
			}
            else if(key == 'r')
            {
                show_preview = true;
                unbox = true;
            }
		}

        cen_x_prev = center_x;
        cen_y_prev = center_y;




	}



	//Close output file.
	// if (output_flag) output_file.close();

	// return 0;
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
	// if (y - 25 > 0)
	// 	line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	// else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	// if (y + 25 < FRAME_HEIGHT)
	// 	line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	// else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	// if (x - 25 > 0)
	// 	line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	// else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	// if (x + 25 < FRAME_WIDTH)
	// 	line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	// else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

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
