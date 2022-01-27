#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <highgui.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/aruco.hpp>
//#include <opencv2/aruco/dictionary.hpp>
#include <vector>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "sensor_msgs/Imu.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <nav_msgs/Odometry.h>

// creating directory
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cmath>
#include <math.h>       /* atan2 */

#include <string>
#include <iostream>
#include <fstream>


using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;
using namespace nav_msgs;

#include <sstream>

// integer to string
template<typename T>
string ToString(T val)
{
	stringstream stream;
	stream << val;
	return stream.str();
}

ros::Publisher * cmd_vel_pub_;
double pi = 3.1415926;
enum Direction {FORWARD, LEFT, RIGHT, BACK, ROTATE};


// goals for zig-zag patterns
void set_goal_coverage(double &x, double &y, double &yaw, bool &finished, Direction &d) {
	static double length = 2; // forward length in meters
	static double width = 2;
	static double stride = 0.3;

	static int state = 0;
	static bool done = false;

	const double rotation = pi/2 - 0.13;
	switch(state) {
		case 0:
      // x front, y left, z up
		d = FORWARD;
		x += length;
		break;
		case 1:
		d = ROTATE;
      // turn left is + yaw
		yaw += rotation;
		break;
		case 2:
		d = LEFT;
		width -= stride;
		if(width > 0) {
			y += stride;
		} else {
			done = true;
		}
		break;
		case 3:
		d = ROTATE;
		yaw += rotation;
		break;
		case 4:
		d = BACK;
		x -= length;
		break;
		case 5:
		d = ROTATE;
		yaw -= rotation;
		break;
		case 6:
		d = LEFT;
		width -= stride;
		if(width > 0) {
			y += stride;
		} else {
			done = true;
			finished = done;
		}
		break;
		case 7:
		d = ROTATE;
		yaw -= rotation;
		state = -1;
		break;
	}
	state += 1;
}

// goals for going only forward
void set_goal_forward(double &x, double &y, double &yaw, bool &finished, Direction &d, bool &is_reverse) {
	static double length = 3;
	static int state = 0;
	static bool done = false;
	
	static int cycles = 1;
	if(cycles > 0) {
		d = FORWARD;
		x += length;
		cycles -= 1;
	} else {
		finished = true;
	}
}

// goals for the robot to go forward backward 'cycles' times
void set_goal_font_back(double &x, double &y, double &yaw, bool &finished, Direction &d, bool &is_reverse) {
	static double length = 1;
	static int state = 0;
	static bool done = false;
	
	static int cycles = 5;
	if(cycles > 0) {
		switch(state) {
			case 0:
			d = FORWARD;
			x += length;
			state = 1;
			is_reverse = false;
			break;
			case 1:
			cout << "setting to back" << endl;
			d = BACK;
			x -= length;
			cycles -= 1;
			state = 0;
			is_reverse = true;
			break;
		}
	} else {
		finished = true;
	}
}

// goals for the robot to make squares 'cycles' times
void set_goal_square(double &x, double &y, double &yaw, bool &finished, Direction &d) {
	static double length = 1; // in meters
	static double width = 1;

	static int state = 0;
	static bool done = false;

	const double rotation = pi/2;

	static int cycles = 5;
	if(cycles > 0) {
		switch(state) {
			case 0:
      // x front, y left, z up
			d = FORWARD;
			x += length;
			break;
			case 1:
			d = ROTATE;
      // left + yaw
			yaw += rotation;
			break;
			case 2:
			d = LEFT;
			y += width;
			break;
			case 3:
			d = ROTATE;
			yaw += rotation;
			break;
			case 4:
			d = BACK;
			x -= length;
			break;
			case 5:
			d = ROTATE;
			yaw += rotation;
			break;
			case 6:
			d = RIGHT;
			y -= width;
			state = -1;
			cycles -= 1;
			break;
		}
		state += 1;
	} else {
		finished = true;
	}
}

// for testing if the robot actually rotates by the values we set
void set_goal_rotate(double &x, double &y, double &yaw, bool &finished, Direction &d) {
	yaw += 3.14/2;
	d = ROTATE;
}
// for testing if the robot actually rotates by the values we set
void set_goal_rotate_once(double &x, double &y, double &yaw, bool &finished, Direction &d) {
	printf("set goal");
	static bool is_first = true;
	if(is_first) {
		yaw += 3.14/2 + 0.0;
		d = ROTATE;
		is_first = false;
	} else {
		finished = true;
	}
}

// entry point to set the robot do to different actions
void set_goal(double &x, double &y, double &yaw, bool &finished, Direction &d, bool &is_reverse) {
  //set_goal_rotate_once(x,y,yaw,finished,d);
  //set_goal_rotate(x,y,yaw,finished,d);
  //set_goal_coverage(x,y,yaw,finished,d);
	//set_goal_font_back(x,y,yaw,finished,d, is_reverse);
	set_goal_forward(x,y,yaw,finished,d, is_reverse);
}

// code to actually move the robot based on the current position
bool move_robot(double x, double y, double z, double yaw) {
	static double goal_x = 0;
	static double goal_y = 0;
	static double goal_yaw = 0;
	static bool reached = true;
	static bool finished = false;
	static Direction d;
	static double last_image_x = 0;
	static double last_image_y = 0;
	static double last_image_yaw = 0;
	static double image_interval = 0.1;
	static double image_rotate_interval = 0.1;
	static bool is_reverse = false;

    // if the goal is reached, get the next goal
	if(reached) {
		goal_x = x;
		goal_y = y;
		goal_yaw = yaw;
		last_image_x = x;
		last_image_y = y;
		last_image_yaw = yaw;
		set_goal(goal_x, goal_y, goal_yaw, finished, d, is_reverse);
		reached = false;

		// just for debug
		string dir = "none";
		switch(d){
			case FORWARD:
			dir = "forward";
			break;
			case ROTATE:
			dir = "rotate";
			break;
			case BACK:
			dir = "back";
			break;
		}
		cout << "set goal direction " << dir << endl;

		printf("goal: %f %f %f\n", goal_x, goal_y, goal_yaw);
	}
	printf("curr: %f %f %f\n", x, y, yaw);

    // the final goal has been reached, shutdown the robot
	if(finished) {
		printf("reached dest\n");
		ros::shutdown();
	}


    // construct the command to be sent to the robot
	geometry_msgs::Twist base_cmd;

    // make yaw to be [0, 2pi]
	if(abs(goal_yaw - yaw) > pi*2) {
		if(goal_yaw > yaw) {
			yaw += pi*2;
		} else {
			yaw -= pi*2;
		}
	}

	// rotation and moving forward are mutually exclusive
	if(d == ROTATE) {
		if(abs(goal_yaw - yaw) < 0.08) {
			reached = true;
		} else if ((goal_yaw - yaw) > 0) {
            // turn left means to increase yaw
			if(yaw > last_image_yaw + image_rotate_interval) {
				last_image_yaw = yaw;
				return true;
			}
			base_cmd.angular.z = +0.2;
		} else {
            // turn right means to decrease yaw
			if(yaw < last_image_yaw - image_rotate_interval) {
				last_image_yaw = yaw;
				return true;
			}
			base_cmd.angular.z = -0.2;
		}
	} else {
        // check if the goal is reached
		switch(d){
			case FORWARD:
			if(x > goal_x)
				reached = true;
			if(x > last_image_x + image_interval) {
				last_image_x = x;
				return true;
			} else {
				//cout << "not taking pic" << last_image_x + image_interval << " " << x << endl;
			}
			break;
			case BACK:
			if(x < goal_x)
				reached = true;
			if(x < last_image_x - image_interval) {
				last_image_x = x;
				return true;
			}
			break;
			case LEFT:
			if(y > goal_y)
				reached = true;
			if(y > last_image_y + image_interval) {
				last_image_y = y;
				return true;
			}
			break;
			case RIGHT:
			if(y < goal_y)
				reached = true;
			if(y < last_image_y - image_interval) {
				last_image_y = y;
				return true;
			}
			break;
		}
		// base_cmd.angular.z = 0;
		if(!reached) {
			if(is_reverse)
				base_cmd.linear.x = -0.1;
			else
				base_cmd.linear.x = 0.1;
		}



        //////////// experimental : try to correct it's path if deviating from the desired path to goal ////////
		// update angle, value of x is position in forward direction (which is the y variable in atan2 function call)
		// negative goal_angle means goal is on the right, and robot is moving towards left
		// left = +yaw
		float goal_angle = atan2(goal_x - x, goal_y - y) - pi/2;
		printf("goal yaw %f cur yaw %f gy %f y %f gx %f x %f\n", goal_angle, yaw, goal_y, y, goal_x, x);
		float angle_error = 0.001;
		if (abs(yaw - goal_angle) <= angle_error)
		{
			base_cmd.angular.z = 0;
		} else if (yaw > goal_angle) {
			base_cmd.angular.z = -0.1;
			printf("fixing towards right\n");
		} else {
			base_cmd.angular.z = 0.1;
			printf("fixing towards left\n");
		}
	}
	cmd_vel_pub_->publish(base_cmd);
	return false;
}

int pic_idx = 0;

// open file and write headers
void open_data_file(ofstream& myfile, string filename) {
	myfile.open (filename.c_str());
	myfile << "image_index" << ",";

	// location
	myfile << "pose->position.x" << ",";
	myfile << "pose->position.y" << ",";
	myfile << "pose->position.z" << ",";
	
	myfile << "yaw" << ",";

	myfile << "orientation.x" << ",";
	myfile << "orientation.y" << ",";
	myfile << "orientation.z" << ",";
	myfile << "orientation.w" << ",";

	// imu - 3D orientation
	myfile << "imu->yaw" << ",";
	
	myfile << "imu->orientation.x" << ",";
	myfile << "imu->orientation.y" << ",";
	myfile << "imu->orientation.z" << ",";
	myfile << "imu->orientation.w" << ",";

	myfile << "imu->angular_velocity.x" << ",";
	myfile << "imu->angular_velocity.y" << ",";
	myfile << "imu->angular_velocity.z" << ",";

	myfile << "imu->linear_acceleration.x" << ",";
	myfile << "imu->linear_acceleration.y" << ",";
	myfile << "imu->linear_acceleration.z" << ",";

	// wheel - 2D Pose
	myfile << "wheel->position.x" << ",";
	myfile << "wheel->position.y" << ",";
	myfile << "wheel->position.z" << ",";

	myfile << "wheel->orientation.x" << ",";
	myfile << "wheel->orientation.y" << ",";
	myfile << "wheel->orientation.z" << ",";
	myfile << "wheel->orientation.w" << ",";

	myfile << endl;
}

void save_data_file(ofstream& myfile,
	const PoseWithCovarianceStamped::ConstPtr &msg, 
	const ImageConstPtr& image,
	const ImuConstPtr& imu,
	const OdometryConstPtr& wheel
	) {

	const geometry_msgs::Pose * pose = &msg->pose.pose;
	const geometry_msgs::Point * position = &pose->position;
	const geometry_msgs::Quaternion * orientation = &pose->orientation;
	
	double yaw = tf::getYaw(*orientation);
	
	myfile << ToString<int>(pic_idx) << ",";
	// location
	myfile << pose->position.x << ",";
	myfile << pose->position.y << ",";
	myfile << pose->position.z << ",";
	
	
	myfile << yaw << ",";

	myfile << orientation->x << ",";
	myfile << orientation->y << ",";
	myfile << orientation->z << ",";
	myfile << orientation->w << ",";
	
	
	// imu - 3D orientation
	double imu_yaw = tf::getYaw(imu->orientation);
	myfile << imu_yaw << ",";
	
	myfile << imu->orientation.x << ",";
	myfile << imu->orientation.y << ",";
	myfile << imu->orientation.z << ",";
	myfile << imu->orientation.w << ",";

	myfile << imu->angular_velocity.x << ",";
	myfile << imu->angular_velocity.y << ",";
	myfile << imu->angular_velocity.z << ",";

	myfile << imu->linear_acceleration.x << ",";
	myfile << imu->linear_acceleration.y << ",";
	myfile << imu->linear_acceleration.z << ",";

	// wheel - 2D Pose
	myfile << wheel->pose.pose.position.x << ",";
	myfile << wheel->pose.pose.position.y << ",";
	myfile << wheel->pose.pose.position.z << ",";

	myfile << wheel->pose.pose.orientation.x << ",";
	myfile << wheel->pose.pose.orientation.y << ",";
	myfile << wheel->pose.pose.orientation.z << ",";
	myfile << wheel->pose.pose.orientation.w << ",";

	myfile << endl;
}

void odomCallback(
	const PoseWithCovarianceStamped::ConstPtr &msg, 
	const ImageConstPtr& image,
	const ImuConstPtr& imu,
	const OdometryConstPtr& wheel
	)
{
	cout << "odom callback" << endl;
	const geometry_msgs::Pose * pose = &msg->pose.pose;
	const geometry_msgs::Point * position = &pose->position;
	const geometry_msgs::Quaternion * orientation = &pose->orientation;
	double yaw = tf::getYaw(*orientation); // in degress
	//printf("current %f %f %f %f\n", position->x, position->y, position->z, yaw);

	static bool is_take_image = false;
	// wait till it becomes fully stopped
	static int image_wait_cycles_default = 26;
	static int image_wait_cycles = image_wait_cycles_default;

	static bool is_save_raw_data = true;
	static ofstream image_data_file;
	static ofstream all_data_file;
	static bool is_image_data_file_opened = false;
	static bool is_all_data_file_opened = false;


	static bool is_save_pic_data = is_take_image;
	static ofstream picturedata;
	static bool is_pic_file_opened = false;
	

	struct stat st = {0};

	// create the file to record data when images are taken
	if(!is_image_data_file_opened){
		if (stat("/home/turtlebot/bot_image", &st) == -1) {
			mkdir("/home/turtlebot/bot_image", 0700);
		}
		open_data_file(image_data_file, "/home/turtlebot/bot_image/image_data.txt");
		is_image_data_file_opened = true;
	}
	
	// create the file to record continuous data
	if(!is_all_data_file_opened){
		if (stat("/home/turtlebot/bot_image", &st) == -1) {
			mkdir("/home/turtlebot/bot_image", 0700);
		}
		open_data_file(all_data_file, "/home/turtlebot/bot_image/raw_data.txt");
		is_all_data_file_opened = true;
	}

	// save continuous data
	if(is_save_raw_data){
		save_data_file(all_data_file, msg, image, imu, wheel);
	}
	// is_take_image = is_save_pic_data;

    // either stop and take the image
    // or     let the robot move to next target
	if(is_take_image){
	    // wait for some cycles before comes to a complete stop
		if(image_wait_cycles > 0) {
			image_wait_cycles -= 1;
			geometry_msgs::Twist base_cmd;
			base_cmd.linear.x = 0;
			base_cmd.angular.z = 0;
			cmd_vel_pub_->publish(base_cmd);
			cout << "waiting to stop " << image_wait_cycles << endl;
			return;
		}

		// robot has been stopped now

		// save the position data
		if(is_save_raw_data){
			save_data_file(image_data_file, msg, image, imu, wheel);
		}

		// save image
		is_take_image = false;
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge Exception %s", e.what());
		}
		cv::imwrite( "/home/turtlebot/bot_image/" + ToString<int>(pic_idx++) + ".jpg", cv_ptr->image );


		image_wait_cycles = image_wait_cycles_default;
	} else {
	    // let the robot move to next target
		//cout << "not take image"<<endl;
		is_take_image = move_robot(position->x, position->y, position->z, yaw);
	}
}

int main(int argc, char** argv)
{
    // initialize the program as a node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

    // listening to multiple node
    // and pass messages with similar timestamp to the callback function

	// get list of nodes by running in terminal: rostopic list
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> odom_sub(nh, "/robot_pose_ekf/odom_combined", 400); // localization data
	message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_rect_color", 400); //
	message_filters::Subscriber<CameraInfo> info_sub(nh, "/camera/rgb/camera_info", 400);
	message_filters::Subscriber<Imu> imu_sub(nh, "/imu", 400);
	message_filters::Subscriber<Odometry> wheel_sub(nh, "/odom", 400);  // wheel encoder

    // listening to localization & imu & wheel encoder
	typedef sync_policies::ApproximateTime<PoseWithCovarianceStamped, Image, Imu, Odometry> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, image_sub, imu_sub, wheel_sub);
	sync.registerCallback(boost::bind(&odomCallback, _1, _2, _3, _4));


	// publish moving commands using a global variable pointer
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    cmd_vel_pub_ = &pub;

    // start running the program(node)
    cout << "spining" << endl;
    ros::spin();


  return 0;
}
