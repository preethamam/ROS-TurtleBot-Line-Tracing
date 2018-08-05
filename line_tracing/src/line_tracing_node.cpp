#include <ros/ros.h>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <line_tracing/hsv_thresh.h>
#include <line_tracing/theta_stamped.h>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher img_pub_thresh;
  image_transport::Publisher img_pub_contour;
  ros::Publisher theta_pub_;
  ros::Subscriber thresh_sub_;
  // yellow
  int lowH = 0, lowS = 90, lowV = 50;
  int highH = 50, highS = 255, highV = 255;
  int k_size = 3;
  int e_size = 0;
  int d_size = 0;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    img_pub_thresh = it_.advertise("/output/thresholded", 1);
    img_pub_contour = it_.advertise("/output/contoured", 1);
    thresh_sub_ = nh_.subscribe("/hsv_thresh", 1, &ImageConverter::threshCb, this);
    theta_pub_ = nh_.advertise<line_tracing::theta_stamped>("/line_angle", 1);
  }
  void threshCb(line_tracing::hsv_thresh msg)
  {
    lowH = msg.lowH;
    highH = msg.highH;
    lowS = msg.lowS;
    highS = msg.highS;
    lowV = msg.lowV;
    highV = msg.highV;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_new(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr_new1(new cv_bridge::CvImage);
    cv::Mat img_orgin;
    cv::Mat img_filtered;
    cv::Mat img_hsv;
    cv::Mat img_thresh;
    cv::Mat img_eroded;
    cv::Mat img_dilated;
    cv::Mat img_contour;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;


    cv::Mat element_erode = getStructuringElement(cv::MORPH_ELLIPSE,
          cv::Size(2 * e_size + 1, 2 * e_size + 1),
          cv::Point(e_size, e_size) );

    cv::Mat element_dilate = getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(2 * d_size + 1, 2 * d_size + 1),
      cv::Point(d_size, d_size) );

    static int seq_i = 0;

    try
    {
      // msg to CVImagePtr
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // image processing
      cv::GaussianBlur(cv_ptr->image, img_filtered, cv::Size(3, 3), 0, 0); // change kernel size
      cv::cvtColor(img_filtered, img_hsv, cv::COLOR_BGR2HSV);
      cv::inRange(img_hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), img_thresh);
      //cv::erode(img_thresh, img_eroded, element_erode);
      //cv::dilate(img_eroded, img_dilated, element_dilate);
      // find contours
      cv::findContours(img_thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      // draw largest contours
      cv::cvtColor(img_thresh, img_contour, cv::COLOR_GRAY2BGR);
      double largest_area = 0;
      int largest_index = 0;
      for(int i=0; i<contours.size(); i++){
        double a = contourArea(contours[i], false);
        if(a>largest_area){
          largest_area = a;
          largest_index = i;
        }
      }     
      cv::drawContours(img_contour, contours, largest_index, cv::Scalar(0, 0, 255), 5, 8, hierarchy);
      
      // get two point p1, p2
      int rows = img_contour.rows;
      int cols = img_contour.cols;
      int x1 = 0, y1 = 0, n1 = 0;
      int x2 = 0, y2 = 0, n2 = 0;
      
      for (int i=0; i<contours[largest_index].size(); i++){
        if (contours[largest_index][i].y < (cols/2)){
          x1 += contours[largest_index][i].x;
          y1 += contours[largest_index][i].y;
          n1 += 1;
        }
        else{
          x2 += contours[largest_index][i].x;
          y2 += contours[largest_index][i].y;
          n2 += 1;
        }
      }

      if(n1 == 0 or n2 == 0)
      {
        std::cout<<"divided by zero"<<std::endl;
      }
      else
      {
        x1 /= n1;   y1 /= n1;
        x2 /= n2;   y2 /= n2;
        
        // draw two point p1, p2
        cv::Point p1(x1, y1);
        cv::Point p2(x2, y2);
        cv::circle(img_contour, p1, 8, cv::Scalar(255, 0, 0), -1);
        cv::circle(img_contour, p2, 8, cv::Scalar(255, 0, 0), -1);

        // calculate line angle
        float theta = atan2(float(x1-x2), float(y2-y1));
        theta = theta*180/3.141592; 
        std::cout<<"theta:"<<theta<<std::endl;
        
        // publish line angle
        line_tracing::theta_stamped msg;
        msg.header.seq = seq_i++;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/line_angle";
        msg.data = theta;
        theta_pub_.publish(msg);
      }    
      
      // Mat to msg
      cv_ptr_new->encoding = "mono8";
      cv_ptr_new->header.stamp = ros::Time::now();
      cv_ptr_new->header.frame_id = "/output";
      cv_ptr_new->image = img_thresh;
      
      cv_ptr_new1->encoding = "bgr8";
      cv_ptr_new1->header.stamp = ros::Time::now();
      cv_ptr_new1->header.frame_id = "/output1";
      cv_ptr_new1->image = img_contour;
     
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Output modified video stream
    img_pub_thresh.publish(cv_ptr_new->toImageMsg());
    img_pub_contour.publish(cv_ptr_new1->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_tracing");
  ImageConverter ic;
  ros::spin();
  return 0;
}
