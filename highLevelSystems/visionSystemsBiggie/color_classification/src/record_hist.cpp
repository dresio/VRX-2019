#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <color_classification/color_classification.h>
#include <string>

#define M_PI 3.14159265358979323846  /* pi */

using namespace cv;
using namespace std;

/**
TODO:
Get this to work with live images
Get color comparison mats
Make program that grabs images and adds them to the comparison mats 
**/

//for angle calculations
struct Vector2
{
  float x;
  float y;
};

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

std::string pack_path;
int DISPLAY;
int white_threshold = 40;
int bin_size =256;

//generates a histogram for color
void findColor(const Mat image, const Mat mask, vector<Point> contours, vector<Mat> &out)
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
	
	out.push_back(b_hist);
	out.push_back(g_hist);
	out.push_back(r_hist);
}

void calcHistBGRColorMask(Mat image, vector<Mat> &out)
{
	// Seperate BGR
	vector<Mat> bgr;
	split( image, bgr );

	Mat mask;
	inRange(image, Scalar(white_threshold, white_threshold, white_threshold), Scalar(255, 255, 255), mask);
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
	
	out.push_back(b_hist);
	out.push_back(g_hist);
	out.push_back(r_hist);
}

void displayHist(vector<Mat> hist, string name)
{
	if(hist.size()!=3)
		return;
			
	int histSize = bin_size;

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

}

vector<Mat> getColor(Mat image)
{
	Mat img;
	vector<Mat> compare;

	vector<Mat> hist_img;
	calcHistBGRColorMask(image, hist_img);

    return hist_img;
} 

//saves histogram to yaml files
void saveHist(vector<Mat> red, vector<Mat> green, vector<Mat> blue, vector<Mat> black, ros::NodeHandle nh)
{
    //save mat directly to computer to be loaded pack later
	string path = pack_path + "/config/images/";
	cv::FileStorage file(path + "red_b.hist", cv::FileStorage::WRITE);
	file << "HA" << red[0];
	file = cv::FileStorage(path + "red_g.hist", cv::FileStorage::WRITE);
	file << "HA" << red[1];
	file = cv::FileStorage(path + "red_r.hist", cv::FileStorage::WRITE);
	file << "HA" << red[2];
	file = cv::FileStorage(path + "green_b.hist", cv::FileStorage::WRITE);
	file << "HA" << green[0];
	file = cv::FileStorage(path + "green_g.hist", cv::FileStorage::WRITE);
	file << "HA" << green[1];
	file = cv::FileStorage(path + "green_r.hist", cv::FileStorage::WRITE);
	file << "HA" << green[2];
	file = cv::FileStorage(path + "blue_b.hist", cv::FileStorage::WRITE);
	file << "HA" << blue[0];
	file = cv::FileStorage(path + "blue_g.hist", cv::FileStorage::WRITE);
	file << "HA" << blue[1];
	file = cv::FileStorage(path + "blue_r.hist", cv::FileStorage::WRITE);
	file << "HA" << blue[2];
	file = cv::FileStorage(path + "black_b.hist", cv::FileStorage::WRITE);
	file << "HA" << black[0];
	file = cv::FileStorage(path + "black_g.hist", cv::FileStorage::WRITE);
	file << "HA" << black[1];
	file = cv::FileStorage(path + "black_r.hist", cv::FileStorage::WRITE);
	file << "HA" << black[2];
    
}

//streamline this later
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Histogram_Generator");
	ros::NodeHandle nh;

    nh.param("/path", pack_path, pack_path);
    nh.param("/display", DISPLAY, DISPLAY);

    int num_test_red;
    int num_test_green;
    int num_test_blue;
	int num_test_black;

    nh.param("/num_red", num_test_red, num_test_red);
    nh.param("/num_green", num_test_green, num_test_green);
    nh.param("/num_blue", num_test_blue, num_test_blue);
	nh.param("/num_black", num_test_black, num_test_black);

    cout << pack_path << endl;

    //**PROCESSING RED IMAGES**//
    //**---------------------**//
    ROS_WARN("PROCESSING %d RED IMAGES", num_test_red);
    Mat image_red;
    std::string path = pack_path + "/Data/LightTower/red test 1.png";
    std::string path_partial = pack_path + "/Data/LightTower/red test ";
    image_red = imread(path, CV_LOAD_IMAGE_COLOR);
	cvtColor(image_red, image_red, CV_RGB2HSV);

    vector<Mat> hist_red = getColor(image_red);

    for(int i = 2; i < num_test_red+1; i++)
    {
        //gets image from file
        std::string path_full = path_partial + patch::to_string(i) + ".png";
        Mat image_temp = imread(path_full, CV_LOAD_IMAGE_COLOR);
		cvtColor(image_temp, image_temp, CV_RGB2HSV);
        vector<Mat> hist_temp = getColor(image_temp);

        for(int i = 0; i < 3; i++)
        {
            Mat temp;
            cv::add(hist_red[i], hist_temp[i], temp);
            temp/=2;
            hist_red[i] = temp;
        }

    }

    //have to normalize new graph
    int hist_w = 512; int hist_h = 500;
	Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    normalize(hist_red[0], hist_red[0], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(hist_red[1], hist_red[1], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(hist_red[2], hist_red[2], 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    //**FINISHED RED IMAGES**//
    //**-------------------**//


    //**PROCESSING GREEN IMAGES**//
    //**-----------------------**//
    ROS_WARN("PROCESSING %d GREEN IMAGES", num_test_green);
    Mat image_green;
    path = pack_path + "/Data/LightTower/green test 1.png";
    path_partial = pack_path + "/Data/LightTower/green test ";
    image_green = imread(path, CV_LOAD_IMAGE_COLOR);
	cvtColor(image_green, image_green, CV_RGB2HSV);

    vector<Mat> hist_green = getColor(image_green);

    for(int i = 2; i < num_test_green+1; i++)
    {
        //gets image from file
        std::string path_full = path_partial + patch::to_string(i) + ".png";
        Mat image_temp = imread(path_full, CV_LOAD_IMAGE_COLOR);
		cvtColor(image_temp, image_temp, CV_RGB2HSV);
        vector<Mat> hist_temp = getColor(image_temp);

        for(int i = 0; i < 3; i++)
        {
            Mat temp;
            cv::add(hist_green[i], hist_temp[i], temp);
            temp/=2;
            hist_green[i] = temp;
        }

    }

    //have to normalize new graph
    normalize(hist_green[0], hist_green[0], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(hist_green[1], hist_green[1], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(hist_green[2], hist_green[2], 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    //**FINISHED GREEN IMAGES**//
    //**---------------------**//


    //**PROCESSING BLUE IMAGES**//
    //**----------------------**//
    ROS_WARN("PROCESSING %d BLUE IMAGES", num_test_blue);
    Mat image_blue;
    path = pack_path + "/Data/LightTower/blue test 1.png";
    path_partial = pack_path + "/Data/LightTower/blue test ";
    image_blue = imread(path, CV_LOAD_IMAGE_COLOR);
	cvtColor(image_blue, image_blue, CV_RGB2HSV);

    vector<Mat> hist_blue = getColor(image_blue);

    for(int i = 2; i < num_test_blue+1; i++)
    {
        //gets image from file
        std::string path_full = path_partial + patch::to_string(i) + ".png";
        Mat image_temp = imread(path_full, CV_LOAD_IMAGE_COLOR);
		cvtColor(image_temp, image_temp, CV_RGB2HSV);
        vector<Mat> hist_temp = getColor(image_temp);

        for(int i = 0; i < 3; i++)
        {
            Mat temp;
            cv::add(hist_blue[i], hist_temp[i], temp);
            temp/=2;
            hist_blue[i] = temp;
        }

    }

    //have to normalize new graph
    normalize(hist_blue[0], hist_blue[0], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(hist_blue[1], hist_blue[1], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(hist_blue[2], hist_blue[2], 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    //**FINISHED BLUE IMAGES**//
    //**---------------------**//


	//**PROCESSING BLACK IMAGES**//
    //**----------------------**//
    ROS_WARN("PROCESSING %d BLACK IMAGES", num_test_black);
    Mat image_black;
    path = pack_path + "/Data/LightTower/blank test 1.png";
    path_partial = pack_path + "/Data/LightTower/blank test ";
    image_black = imread(path, CV_LOAD_IMAGE_COLOR);

    vector<Mat> hist_black = getColor(image_black);

    for(int i = 2; i < num_test_black+1; i++)
    {
        //gets image from file
        std::string path_full = path_partial + patch::to_string(i) + ".png";
        Mat image_temp = imread(path_full, CV_LOAD_IMAGE_COLOR);
		cvtColor(image_temp, image_temp, CV_RGB2HSV);
        vector<Mat> hist_temp = getColor(image_temp);

        for(int i = 0; i < 3; i++)
        {
            Mat temp;
            cv::add(hist_black[i], hist_temp[i], temp);
            temp/=2;
            hist_black[i] = temp;
        }

    }

    //have to normalize new graph
    normalize(hist_black[0], hist_black[0], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(hist_black[1], hist_black[1], 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(hist_black[2], hist_black[2], 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    //**FINISHED BLACK IMAGES**//
    //**---------------------**//


    //now to save array to yaml file
    saveHist(hist_red, hist_green, hist_blue, hist_black, nh);
	ROS_WARN("FINISHED SAVING MATS");

	return 0;

}
