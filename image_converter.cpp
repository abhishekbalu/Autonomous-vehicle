#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <cmath>
#include "iostream"
#include "string.h"
#include <ros/ros.h>


using namespace std;
using namespace cv;



static double angle(Point pt1, Point pt2, Point pt0)
{
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
void setLabel(Mat& im, const string label, vector<Point>& contour)
{
        int fontface = FONT_HERSHEY_SIMPLEX;
        double scale = 0.8;
        int thickness = 2;
        int baseline = 0;

        Size text = getTextSize(label, fontface, scale, thickness, &baseline);
        Rect r = boundingRect(contour);

        Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
        rectangle(im, pt + Point(0, baseline), pt + Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
        putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  ros::NodeHandle nw;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Int16>("chatter", 1000);
  ros::Rate loop_rate(1000); 
 
  
  
  image_transport::ImageTransport it(nh);
  image_transport::ImageTransport at(nw);
  image_transport::Publisher pub1 = it.advertise("image", 1);
  image_transport::Publisher pub2 = at.advertise("image1", 1);
  
  sensor_msgs::ImagePtr msg1;
 // sensor_msgs::ImagePtr msg2;
  
  while (ros::ok())
  {
     std_msgs::Int16 msg;
    

    Mat src,src1,dst,dst1;
    VideoCapture cap0;

    cap0.open(2);
 
    VideoCapture cap1;

    cap1.open(1);
 


if(!cap0.isOpened())
{
  return -1;
}


if(!cap1.isOpened())
{
  return -1;
}
  sensor_msgs::ImagePtr msg1;
    sensor_msgs::ImagePtr msg2;


 while (nh.ok()) {


          while(waitKey(1) != 'Esc')
   		 {
   		 
                  int a=5;
                        cap0.read(src);
			cap1.read(src1);
			
			
			
 if(!src.empty()) {
 
      msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
      pub1.publish(msg1);
      cv::waitKey(1);
      
    }

 if(!src1.empty()) {
 
      msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src1).toImageMsg();
      pub2.publish(msg2);
      cv::waitKey(1);
      
    }

                             		   
//..................................................................... full frame
	Mat gray;
	cvtColor(src, gray, CV_BGR2GRAY);

	// Use Canny instead of threshold to catch squares with gradient shading
	Mat bw;
	Canny(gray, bw, 0, 50, 5);
          
	// Find contours
	vector<vector<Point> > contours;
	findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	vector<Point> approx;
	dst = src.clone();
             int aa =contours.size();
        for (int i = 0; i < aa; i++)
	{
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);

		// Skip small or non-convex objects 
		if (fabs(contourArea(contours[i])) < 100 || !isContourConvex(approx))
			continue;

		//if (approx.size() == 3)
		//{
		//	setLabel(dst, "TRI", contours[i]);    // Triangles
		//}
		else if (approx.size() >= 4 && approx.size() <= 6)
		{
			// Number of vertices of polygonal curve
			int vtc = approx.size();

			// Get the cosines of all corners
			vector<double> cos;
			for (int j = 2; j < vtc+1; j++)
				cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

			// Sort ascending the cosine values
			sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();

			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
				{
				setLabel(dst, "RECT", contours[i]);
				a =1;
				}
			else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
				{
				setLabel(dst, "PENTA", contours[i]);
				
				}
			else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
				{
				setLabel(dst, "HEXA", contours[i]);
				
		}
                }
		else
		{
			// Detect and label circles
			double area = contourArea(contours[i]);
			Rect r = boundingRect(contours[i]);
			int radius = r.width / 2;

			if (abs(1 - ((double)r.width / r.height)) <= 0.2 &&
			    abs(1 - (area / (CV_PI * pow(radius, 2)))) <= 0.2)
				{
				setLabel(dst, "CIR", contours[i]);
				a=2;
		}
		}
	}
	
	
	
	
	
	
	
	        Mat gray1;
	cvtColor(src1, gray1, CV_BGR2GRAY);

	// Use Canny instead of threshold to catch squares with gradient shading
	Mat bw1;
	Canny(gray1, bw1, 0, 50, 5);
          
	// Find contours1
	vector<vector<Point> > contours1;
	findContours(bw1.clone(), contours1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	vector<Point> approx1;
	dst1 = src1.clone();
             int bb =contours1.size();
        for (int i = 0; i < bb; i++)
	{
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		approxPolyDP(Mat(contours1[i]), approx1, arcLength(Mat(contours1[i]), true)*0.02, true);

		// Skip small or non-convex objects 
		if (fabs(contourArea(contours1[i])) < 100 || !isContourConvex(approx1))
			continue;

		//if (approx1.size() == 3)
		//{
		//	setLabel(dst1, "TRI", contours1[i]);    // Triangles
		//}
		else if (approx1.size() >= 4 && approx1.size() <= 6)
		{
			// Number of vertices of polygonal curve
			int vtc = approx1.size();

			// Get the cosines of all corners
			vector<double> cos;
			for (int j = 2; j < vtc+1; j++)
				cos.push_back(angle(approx1[j%vtc], approx1[j-2], approx1[j-1]));

			// Sort ascending the cosine values
			sort(cos.begin(), cos.end());

			// Get the lowest and the highest cosine
			double mincos = cos.front();
			double maxcos = cos.back();

			// Use the degrees obtained above and the number of vertices
			// to determine the shape of the contour
			if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
				{
				setLabel(dst1, "RECT", contours1[i]);
				a =1;
				}
			else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
				{
				setLabel(dst1, "PENTA", contours1[i]);
				
				}
			else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
				{
				setLabel(dst1, "HEXA", contours1[i]);
				
		}
                }
		else
		{
			// Detect and label circles
			double area = contourArea(contours1[i]);
			Rect r = boundingRect(contours1[i]);
			int radius = r.width / 2;

			if (abs(1 - ((double)r.width / r.height)) <= 0.2 &&
			    abs(1 - (area / (CV_PI * pow(radius, 2)))) <= 0.2)
				{
				setLabel(dst1, "CIR", contours1[i]);
				a=2;
		}
		}
	}
	
	

        //imshow("src", src);
	imshow("dst", dst);
       imshow("dst1", dst1);
	//imshow("dst2", dst2);
      //  imshow("dst1", dst3);
      //  imshow("dst2", dst4);
      //s  imshow("dst1", dst5);

  
  int b=1;
  int c=2;
  int d=3;
  int e=4;
 // int f=5;
  //int g=6;
    
if(a == 5)
{
   msg.data = 0;
    ROS_INFO("%d", msg.data);
}
if (a == 1)
{
      msg.data = b;
    ROS_INFO("%d", msg.data);
   
} 
if (a == 2)
{
      msg.data = c;
    ROS_INFO("%d", msg.data);
   
} 
if (a == 3)
{
      msg.data = d;
    ROS_INFO("%d", msg.data);
   
} 
if (a == 4)
{
      msg.data = e;
    ROS_INFO("%d", msg.data);
   
} 
/*if (a == 5)
{
      msg.data = f;
    ROS_INFO("%d", msg.data);
   
} 
if (a == 6)
{
      msg.data = g;
    ROS_INFO("%d", msg.data);
   
} */
    
    chatter_pub.publish(msg);
      ros::spinOnce();

    loop_rate.sleep();
   
  }

}

 return 0;
}



}


