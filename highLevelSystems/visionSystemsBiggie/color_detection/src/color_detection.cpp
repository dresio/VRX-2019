#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <color_detection/color_detection.h>
#include <string>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

# define M_PI 3.14159265358979323846  /* pi */

using namespace cv;
using namespace std;

/**
TODO:
Get this to work with live images
Get color comparison mats
**/

//commented conf b/c no shape detection
// //for angle calculations
// struct Vector2
// {
//   float x;
//   float y;
// };

// //stores information on the shape we are looking for
// struct ShapeInfo
// {
// 	bool by_angle;
// 	bool by_circularity;
// 	int num_contours;
// 	float angle_goal;
// };

// double contour_filter = 1;
// float area_threshold = 100;
// int pos_threshold = 100;
// float angle_threshold = 5;
// float circle_threshold = 20;
// int canny_threshold = 82;
float contrast_value = 1.5;
float brightness_value = -10;
int white_threshold = 40;
std::string pack_path;
int bin_size =256;

vector<Mat> red;
vector<Mat> green;
vector<Mat> blue;
vector<Mat> black;

//more shape detection things
// //I don't know if this actually works or if it is even being used still
// Point findCenter(vector<Point> contour)
// {
// 	Moments mom = moments(contour, false );
// 	Point to_return;
// 	to_return.x = int(mom.m10 / mom.m00) * 100;
// 	to_return.y = int(mom.m01 / mom.m00) * 100;

// 	return to_return;
// }

// //Works well enough
// float findAngle(Point point1, Point point2, Point point3)
// {
// 	float angle;
// 	//generate the two vectors and store them into points
// 	Vector2 vec1, vec2;
// 	//point 2 is the "orgin"
// 	vec1.x = point3.x - point2.x;
// 	vec1.y = point3.y - point2.y;
// 	vec2.x = point1.x - point2.x;
// 	vec2.y = point1.y - point2.y;

// 	angle = atan2(vec2.y, vec2.x) - atan2(vec1.y, vec1.x);

// 	//normalizing
// 	if (angle < -M_PI/2) angle += M_PI;
// 	if (angle > M_PI/2) angle -= M_PI;
// 	angle *= 57.2958;

// 	//ROS_INFO("Angle: %f", angle);

// 	return angle;
// }

//generates a histogram for color
void findColor(const Mat image, const Mat mask, vector<Point> contours, vector<Mat> &conf)
{
    Rect _boundingRect = boundingRect( contours );
	
	Mat mask_bin;
	//converts to binary (theshold didn't work)
	inRange(mask, Scalar(50, 50, 50), Scalar(255, 255, 255), mask_bin);
	
	// Seperate BGR
	vector<Mat> bgr;
	split( image(_boundingRect), bgr );

	int histSize = bin_size;

	float range[] = { 0, 255 } ;
	const float* histRange = { range };

	bool uniform = true; 
	bool accumulate = false;

	Mat b_hist, g_hist, r_hist;

	calcHist( &bgr[0], 1, 0, mask_bin(_boundingRect), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr[1], 1, 0, mask_bin(_boundingRect), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr[2], 1, 0, mask_bin(_boundingRect), r_hist, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 500;
	int bin_w = cvRound( (double) hist_w/histSize );

	Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	
	conf.push_back(b_hist);
	conf.push_back(g_hist);
	conf.push_back(r_hist);
}

//more shape detection
// //cuts down on contours
// vector<vector<Point> > filterContours(vector<vector<Point> > contours)
// {
// 	//approximates the values and ensures they are closed
// 	for(int i = 0; i < contours.size(); i++)
// 	{
// 		double epsilon = contour_filter/100*arcLength(contours[i],true);
// 		vector<Point> approx;
// 		approxPolyDP(contours[i],approx, epsilon,true);
// 		contours[i] = approx;
// 	}
	
// 	//removes single element contours or non existant contours
// 	for(int i = contours.size(); i > 1; i--)
// 	{
// 		float area = contourArea(contours[i-1]);
		
// 		if(contours[i-1].size() <= 2 || area < 100)
// 		{
// 			contours.erase(contours.begin() + i-1);
// 		}
// 	}
	
// 	//removes copy contours
// 	for(int i = contours.size(); i > 1; i--)
// 	{
// 		float area = contourArea(contours[i-1]);
// 		Point pos = findCenter(contours[i-1]);
// 		for(int j = 0; j < i-1; j++)
// 	 	{
// 			int change_area = abs(area - contourArea(contours[j]));
// 			float change_pos = abs(sqrt((pos.x - findCenter(contours[j]).x)^2 + (pos.y - findCenter(contours[j]).y)^2));
// 			if(change_area < area_threshold && change_pos < pos_threshold)
// 			{
// 				contours.erase(contours.begin()+ i-1);
// 			}
// 		}
// 	}

// 	return contours;
// }

//shape detectin functions
// //function for all other objects
// bool identifyObjects(vector<vector<Point> > contours, float angle_goal, int num_sides, Mat image, vector<Vec4i> hierarchy, vector<Mat> &conf)
// {
// 	Mat drawing = Mat::zeros( image.size(), CV_8UC3 );
// 	Mat mask = Mat::zeros( image.size(), CV_8UC3 );

// 	if(num_sides == 4)
// 	{
// 		contours = filterContours(contours);
// 	}

// 	//find shapes
//   	for( int i = 0; i< contours.size(); i++ )
//     {
// 		//crusiform
// 		if(contours[i].size() == num_sides)
// 		{
			
// 			bool is_shape = false;
// 			for(int j = 0; j < num_sides; j++)
// 			{
// 				float angle = findAngle(contours[i][j], contours[i][j+1], contours[i][j+2]);

// 				//ROS_INFO("Angle Thresh Value: %f", abs(angle-angle_goal));

// 				if(!((abs(angle-angle_goal)) < angle_threshold))
// 				{
// 					is_shape = true;
					
// 				}
// 				drawContours( drawing, contours, i, Scalar(255,0,225), 1, 8, hierarchy, 0, Point() );
// 			}

// 			if(is_shape)
// 			{
// 				drawContours( drawing, contours, i, Scalar(0,0,225), 1, 8, hierarchy, 0, Point() );
// 				drawContours(mask, contours, i, Scalar(225,225,225), CV_FILLED, 8, hierarchy, 0, Point());
// 				if(num_sides == 3)
// 				{
// 					ROS_INFO("Triangle Found");
// 				}
// 				else if(num_sides == 12)
// 				{
// 					ROS_INFO("Crusiform Found");
// 				}
// 				else if(num_sides == 4)
// 				{
// 					ROS_INFO("Rectangle Found");
// 				}
// 				findColor(image, mask, contours[i], conf);

// 				return true;
// 			}
// 		}	
// 	}
// 	return false;
// }

// //function for circular objects
// bool identifyObjects(vector<vector<Point> > contours, Mat image, vector<Vec4i> hierarchy, vector<Mat> &conf)
// {
// 	Mat drawing = Mat::zeros( image.size(), CV_8UC3 );
// 	Mat hough_image =  Mat::zeros( image.size(), CV_8UC3 );
// 	Mat mask = Mat::zeros( image.size(), CV_8UC3 );
// 	for(int i = 0; i < contours.size(); i++)
// 	{
// 		//circle
// 		if(contours[i].size() > 12)
// 		{
// 			bool is_circle = false;

// 			//check for circle with houghcircletransform
// 			vector<Vec3f> circles;
// 			drawContours( hough_image, contours, i, Scalar(255,0,225), CV_FILLED, 8, hierarchy, 0, Point() );
// 			cvtColor(hough_image, hough_image, CV_BGR2GRAY);
// 			GaussianBlur( hough_image, hough_image, Size( 31, 31 ), 20, 20 ); 
// 			inRange(hough_image, Scalar(10, 10, 10), Scalar(255, 255, 255), hough_image);

// 			HoughCircles( hough_image, circles, CV_HOUGH_GRADIENT, 1, hough_image.rows/8, 200, circle_threshold, 0, 100000 );
			
// 			if(circles.size() > 0)
// 			{
// 				is_circle = true;
// 			}

// 			drawContours( drawing, contours, i, Scalar(255,0,225), 1, 8, hierarchy, 0, Point() );

// 			hough_image =  Mat::zeros( image.size(), CV_8UC3 );

// 			if(is_circle)
// 			{
// 				drawContours( drawing, contours, i, Scalar(0,255,225), 1, 8, hierarchy, 0, Point() );
// 				drawContours(mask, contours, i, Scalar(225,225,225), CV_FILLED, 8, hierarchy, 0, Point());				
// 				ROS_INFO("Circle Found");
// 				findColor(image, mask, contours[i], conf);

// 				return true;
// 			} 
// 		}
// 	}
// 	return false;
// }

// //sorts contours by size
// vector<vector<Point> > sortContourArea(vector<vector<Point> > contours)
// {
// 	//bubble sort here
// 	bool sorted = false;
// 	while(!sorted)
// 	{
// 		int num_switch = 0;
// 		for(int i = 0; i < contours.size()-1; i++)
// 		{
// 			if(contourArea(contours[i]) < contourArea(contours[i+1]))
// 			{
// 				//switch places
// 				contours[i].swap(contours[i+1]);
// 				num_switch++;
// 			}
// 		}

// 		if(num_switch == 0)
// 			sorted = true;
// 	}
	
// 	return contours;
// }


//only works with given mask
// //splits image into its BGR histograms
// bool calcHistBGR(Mat image, vector<Mat> &conf)
// {
// 	Mat res, gray, blur, canny, drawing, contrast;

// 	contrast = Mat::zeros( image.size(), CV_8UC3 );
// 	//increasing contrast in image
// 	for(int y = 0; y < image.rows; y++)
// 	{
// 		for(int x = 0; x < image.cols; x++)
// 		{
// 			for(int c = 0; c < image.channels(); c++)
// 			{
// 				contrast.at<Vec3b>(y,x)[c] = saturate_cast<uchar>((contrast_value*image.at<Vec3b>(y,x)[c])+brightness_value);
// 			}
// 		}
// 	}

// 	//initializes black mats
// 	drawing = Mat::zeros( image.size(), CV_8UC3 );

// 	cvtColor(contrast, gray, cv::COLOR_RGB2GRAY);

// 	Canny( gray, canny, canny_threshold, canny_threshold*3, 3 );

// 	GaussianBlur( canny, blur, Size( 9, 9 ), 3, 3 );

// 	vector<vector<Point> > contours;
//   	vector<Vec4i> hierarchy;

// 	vector<vector<Point> > contours_circle;
// 	vector<Vec4i> hierarchy_circle;

// 	findContours( canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
// 	findContours( blur, contours_circle, hierarchy_circle, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

// 	contours = filterContours(contours);
// 	contours_circle = filterContours(contours_circle);

// 	for(int i = 0; i < contours.size(); i++)
// 	{
// 		drawContours( drawing, contours, i, Scalar(0,0,225), 1, 8, hierarchy, 0, Point() );
// 	}

// 	for(int i = 0; i < contours_circle.size(); i++)
// 	{
// 		drawContours( drawing, contours_circle, i, Scalar(0,0,225), 1, 8, hierarchy, 0, Point() );
// 	}

// 	contours = sortContourArea(contours);
// 	contours_circle = sortContourArea(contours_circle);

// 	//cycle through different shapes
// 	if(identifyObjects(contours_circle, image, hierarchy, conf))	//circle
// 	{
// 		return true;
// 	}
// 	else if(identifyObjects(contours, 90, 12, image, hierarchy, conf))	//crusiform
// 	{
// 		return true;
// 	}
// 	else if(identifyObjects(contours, 60, 3, image, hierarchy, conf))	//triangle
// 	{
// 		return true;
// 	}	
// 	else if(identifyObjects(contours, 90, 4, image, hierarchy, conf))	//rectangle
// 	{
// 		return true;
// 	}
// 	else	//nothing
// 	{
// 		return false;
// 	}
// }

//Calculates it for difficult objects but removes white
void calcHistBGRColorMask(Mat image, vector<Mat> &conf)
{
	// Seperate BGR
	vector<Mat> bgr;
	split( image, bgr );

	Mat mask;
	inRange(image, Scalar(0, 0, 200), Scalar(180, 20, 255), mask);
    bitwise_not(mask,mask);

	int histSize = bin_size;

	float range[] = { 0, 255 } ;
	const float* histRange = { range };

	bool uniform = true; 
	bool accumulate = false;

	Mat b_hist, g_hist, r_hist;

	calcHist( &bgr[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  	calcHist( &bgr[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 500;
	int bin_w = cvRound( (double) hist_w/histSize );

	Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
	
	conf.push_back(b_hist);
	conf.push_back(g_hist);
	conf.push_back(r_hist);
}

//displays histograms in a readable way
Mat getDisplayMat(vector<Mat> hist)
{

	if(hist.size()!=3)
		ROS_WARN("Display Input Mat is not the right size");
			
	int histSize = 256;

	float range[] = { 0, 255 } ;
	const float* histRange = { range };

	bool uniform = true; 
	bool accumulate = false;

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 500;
	int bin_w = cvRound( (double) hist_w/histSize );

	Mat display( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );


	// Draw for each channel
	for( int i = 1; i < 256; i++ )
	{
		line( display, Point( bin_w*(i-1), hist_h - cvRound(hist[0].at<float>(i-1)) ) ,
					Point( bin_w*(i), hist_h - cvRound(hist[0].at<float>(i)) ),
					Scalar( 255, 0, 0), 2, 8, 0  );
		line( display, Point( bin_w*(i-1), hist_h - cvRound(hist[1].at<float>(i-1)) ) ,
					Point( bin_w*(i), hist_h - cvRound(hist[1].at<float>(i)) ),
					Scalar( 0, 255, 0), 2, 8, 0  );
		line( display, Point( bin_w*(i-1), hist_h - cvRound(hist[2].at<float>(i-1)) ) ,
					Point( bin_w*(i), hist_h - cvRound(hist[2].at<float>(i)) ),
					Scalar( 0, 0, 255), 2, 8, 0  );
	}

	return display;

}

//grab data from config images
void loadHists()
{
	Mat red_b, red_g, red_r;
	string path = pack_path + "/config/images/";
	cv::FileStorage file(path + "red_b.hist", cv::FileStorage::READ);
	file["HA"] >> red_b;
	red.push_back(red_b);
	file = cv::FileStorage(path + "red_g.hist", cv::FileStorage::READ);
	file["HA"] >> red_g;
	red.push_back(red_g);
	file = cv::FileStorage(path + "red_r.hist", cv::FileStorage::READ);
	file["HA"] >> red_r;
	red.push_back(red_r);
	Mat green_b, green_g, green_r;
	file = cv::FileStorage(path + "green_b.hist", cv::FileStorage::READ);
	file["HA"] >> green_b;
	green.push_back(green_b);
	file = cv::FileStorage(path + "green_g.hist", cv::FileStorage::READ);
	file["HA"] >> green_g;
	green.push_back(green_g);
	file = cv::FileStorage(path + "green_r.hist", cv::FileStorage::READ);
	file["HA"] >> green_r;
	green.push_back(green_r);
	Mat blue_b, blue_g, blue_r;
	file = cv::FileStorage(path + "blue_b.hist", cv::FileStorage::READ);
	file["HA"] >> blue_b;
	blue.push_back(blue_b);
	file = cv::FileStorage(path + "blue_g.hist", cv::FileStorage::READ);
	file["HA"] >> blue_g;
	blue.push_back(blue_g);
	file = cv::FileStorage(path + "blue_r.hist", cv::FileStorage::READ);
	file["HA"] >> blue_r;
	blue.push_back(blue_r);
	Mat black_b, black_g, black_r;
	file = cv::FileStorage(path + "black_b.hist", cv::FileStorage::READ);
	file["HA"] >> black_b;
	black.push_back(black_b);
	file = cv::FileStorage(path + "black_g.hist", cv::FileStorage::READ);
	file["HA"] >> black_g;
	black.push_back(black_g);
	file = cv::FileStorage(path + "black_r.hist", cv::FileStorage::READ);
	file["HA"] >> black_r;
	black.push_back(black_r);
}

//finds colors for docking
bool findColor(color_detection::color_detection::Request  &req,
	 color_detection::color_detection::Response &res)
{
	Mat img;
	vector<Mat> compare;

	cv_bridge::CvImagePtr img_ptr;
	img_ptr = cv_bridge::toCvCopy(req.image, sensor_msgs::image_encodings::BGR8);
	img = img_ptr -> image;

	cvtColor(img, img, CV_RGB2HSV);

	vector<Mat> hist_img;
	calcHistBGRColorMask(img, hist_img);

	vector<float> conf;

	//compare blue and normalizes
	float sum_blue = 0;
	vector<float> blue_temp;
	for(int i = 0; i < 3; i++)	//comparing loop
	{	
		blue_temp.push_back(compareHist( hist_img[i], blue[i], CV_COMP_INTERSECT ) );
		sum_blue += compareHist( hist_img[i], blue[i], CV_COMP_INTERSECT );
	}
	for(int i = 0; i < 3; i++)	//normalizing and placement loop
	{
		blue_temp[i] /= sum_blue;
	}
	//compare green
	float sum_green = 0;
	vector<float> green_temp;
	for(int i = 0; i < 3; i++)	//comparing loop
	{
		green_temp.push_back(compareHist( hist_img[i], green[i], CV_COMP_INTERSECT ));
		sum_green += compareHist( hist_img[i], green[i], CV_COMP_INTERSECT );
	}
	for(int i = 0; i < 3; i++)	//normalizing and placement loop
	{
		green_temp[i] /= sum_green;
	}
	//compare red
	float sum_red = 0;
	vector<float> red_temp;
	for(int i = 0; i < 3; i++)
	{	
		red_temp.push_back(compareHist( hist_img[i], red[i], CV_COMP_INTERSECT ) );
		sum_red += compareHist( hist_img[i], red[i], CV_COMP_INTERSECT );
	}
	for(int i = 0; i < 3; i++)
	{
		red_temp[i]/=sum_red;
	}
	//compare black
	float sum_black = 0;
	vector<float> black_temp;
	for(int i = 0; i < 3; i++)
	{
		black_temp.push_back(compareHist( hist_img[i], black[i], CV_COMP_INTERSECT ) );
		sum_black+=compareHist( hist_img[i], black[i], CV_COMP_INTERSECT );
	}
	for(int i = 0; i < 3; i++)
	{
		black_temp[i]/=sum_black;
	}

	//three different channels
	//sends all h, then s, then v
	for(int i = 0; i < 3; i++)
	{
		conf.push_back(blue_temp[i]);
		conf.push_back(green_temp[i]);
		conf.push_back(red_temp[i]);
		conf.push_back(black_temp[i]);
	}

	//redo this later to remove conf variable
	if(conf[0] > conf[1] && conf[0] > conf[2])	
	{
		//res.color = color_shape_detection::color_shape_detection::BLUE;
		res.color = 0;
	}
	else if(conf[1] > conf[0] && conf[1] > conf[2])
	{
		//res.color = color_shape_detection::color_shape_detection::GREEN;
		res.color = 1;
	}
	else if(conf[2] > conf[0] && conf[2] > conf[1])
	{
		//res.color = color_shape_detection::color_shape_detection::RED;
		res.color = 2;
	}
	else if(conf[3] > conf[0] && conf[3] > conf[1] && conf[3] > conf[2])
	{
		//res.color = color_shape_detection::color_shape_detection::BLACK;
		res.color = 3;
	}
	else{
		res.color = 10;	//shouldn't get here
	}

	res.confidence_blue = blue_temp;
	res.confidence_green = green_temp;
	res.confidence_red = red_temp;
	res.confidence_black = black_temp;

} 

int main(int argc, char **argv)
{

	ros::init(argc, argv, "color_detection_service");
	ros::NodeHandle nh;

	nh.param("/path", pack_path, pack_path);

	loadHists();

	ros::ServiceServer service = nh.advertiseService("color_detection", findColor);
	ROS_INFO("Send image and will return the color of the object");

	ros::spin();

	return 0;

}
