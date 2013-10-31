/*
 *  chess_calibration_data_colection_node.cpp
 *
 *  Created on: Sep 12, 2013
 *      Author: Daniel Oñoro Rubio
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/ros/conversions.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
using namespace std;

class ChessLidarDataColector {
public:
	ChessLidarDataColector(string output_files_names) {
		output_files_names_ = output_files_names;
		im_account_ = 0;

		// Image compression parameters
		compression_params_.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params_.push_back(0);
	}
	;

	~ChessLidarDataColector() {
		saveScans(output_files_names_);
		saveImageTimeStamps(output_files_names_);
	}
	;

	void callback(const sensor_msgs::LaserScanConstPtr& scan,
			const sensor_msgs::ImageConstPtr& msg) {

		if (laser_params_.empty()) {
			laser_params_.push_back(scan->angle_min);
			laser_params_.push_back(scan->angle_increment);
			laser_params_.push_back(scan->angle_max);
			/* Ranges types
			 1 for mm
			 2 for cm
			 3 for m
			 4 for km
			 */
			laser_params_.push_back(3); // Meters
		}

		// Get Image
		cv_bridge::CvImagePtr im_ptr;
		try {
			im_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::imshow("Camera", im_ptr->image);
		char key = cv::waitKey(10);

		// Save data
		if (key == 's' || key == 'S') {
			stringstream ss;
			ss << ++im_account_;
			string output = output_files_names_ + ss.str() + ".png";

			cv::imwrite(output, im_ptr->image,
					compression_params_);

			time_stamps_.push_back(scan->header.stamp.toSec());
			scan_ranges_.push_back(scan->ranges);
			ROS_INFO("Recorded image: %s\n", output.c_str());
		}

	}
	;

private:
	string output_files_names_;
	unsigned int im_account_;
	vector<int> compression_params_;
	vector<double> time_stamps_;
	vector<double> laser_params_;
	vector<vector<float> > scan_ranges_;

	void saveScans(string path) {
		string foutput = path + "_scans.txt";
		//Out file
		ofstream ffile(foutput.c_str());

		if (ffile.is_open()) {
			ffile.precision(19);
			for (unsigned int i = 0; i < scan_ranges_.size(); i++) {

				ffile << time_stamps_[i] << " " << laser_params_[0] << " "
						<< laser_params_[1] << " " << laser_params_[2] << " "
						<< laser_params_[3] << " " << scan_ranges_[i].size() << " ";

				for (unsigned int j = 0; j < scan_ranges_[i].size(); j++) {
					ffile <<  scan_ranges_[i][j] << " ";
				}
			}

			ffile << endl;
		} else {
			cerr << "Error while writing file: " << foutput << endl;
			exit(-1);
		}

		//close file
		ffile.close();
	}
	;

	void saveImageTimeStamps(string path) {
		string foutput = path + "_image_stamps.txt";

		//Out file
		ofstream ffile(foutput.c_str());

		if (ffile.is_open()) {
			ffile.precision(19);
			for (unsigned int i = 0; i < time_stamps_.size(); i++) {
				ffile << time_stamps_[i] << endl;
			}
		} else {
			cerr << "Error while writing file: " << foutput << endl;
			exit(-1);
		}

		//close file
		ffile.close();
	}
	;
};

int main(int argc, char** argv) {
	string laser_topic, camera_topic, output_files_names;
	ros::init(argc, argv, "chess_calibration_data_colection_node");
	ros::NodeHandle nh("~");

	// Getting parameters
	nh.param("laser_topic", laser_topic, string("/scan"));
	nh.param("camera_topic", camera_topic, string("/image_raw"));
	nh.param("output", output_files_names, string("output"));

	ChessLidarDataColector data_colector(output_files_names);

	cout << "Subscriging to: " << laser_topic << endl;
	cout << "Subscriging to: " << camera_topic << endl;

	// Constructor
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh,
			laser_topic, 1);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, camera_topic,
			1);
	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::LaserScan, sensor_msgs::Image> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),
			laser_sub, image_sub);
	sync.registerCallback(
			boost::bind(&ChessLidarDataColector::callback, &data_colector, _1,
					_2));
	ros::spin();

	ROS_INFO(
			"WARNING: devices are synchronized, so the speed is setted up to the slowest device!");
	ROS_INFO("Press s to save frame data\n");

	return 0;
}
