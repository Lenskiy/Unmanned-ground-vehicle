/*
 * traffic_light_detector_node.cpp
 *
 *  Created on: Sep 5, 2013
 *      Author: Daniel Oñoro Rubio
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>

#include <kut_ugv_msgs/WorldObject.h>
#include <kut_ugv_sensor_fusion/lidar_object_list.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>

#include <string.h>
#include <iostream>
using namespace std;

// Other functions
#include "SensorFusion.h"
#include "ImProcessingTools.h"

using namespace sf;

//#define DEBUG

// Template matching constant threshold
const static float TM_THRES = 0.80;
const static float LIDAR_IMAGE_THRES = 50; // Distance threshold (for lidar - image)
const static float CONT_DETECTION_DIST_THRES = 25; // Continuous detection distance threshold
const static float MIN_DETECTIONS = 5;
const static float TIME_ALIVE = 1;
const static cv::Size TM_SIZE = cv::Size(10, 10);

//static unsigned int file_count = 0;

class DetectionBuffer {
	class ObjectDetected {
	public:
		ObjectDetected(const ros::Time &time_stamp, unsigned int label,
				cv::Point pos) {
			time_stamp_ = time_stamp;
			label_ = label;
			pos_detections_ = 0;
			total_detections_ = 0;
			postion_ = pos;
		}
		;
		ros::Time time_stamp_;
		unsigned int label_;
		int pos_detections_;
		int total_detections_;
		cv::Point postion_;
	};

public:
	DetectionBuffer(int min_detections, float live_time, float thres) {
		min_detections_ = min_detections;
		live_time_ = live_time;
		curr_label_ = 0;
		thres_ = thres;
	}
	;

	~DetectionBuffer() {
	}
	;

	float getProbability(const ros::Time &time_stamp, cv::Point pos) {
		float dist;
		std::list<ObjectDetected>::iterator it, min_it = object_list_.begin();
		dist = computeDistance((*min_it).postion_, pos);
		it = min_it;
		for (it++; it != object_list_.end(); it++) {
			(*it).total_detections_++;
			float cur_dist = computeDistance((*it).postion_, pos);
			if (cur_dist < dist) {
				dist = cur_dist;
				min_it = it;
			}
		}

		// Check if the minimum distance is less than the threshold
		if (dist < CONT_DETECTION_DIST_THRES) {
			(*min_it).pos_detections_++;
			(*min_it).time_stamp_ = time_stamp; // Update time stamp
			(*min_it).postion_ = pos;
		} else {
			// Add new object
			object_list_.push_back(
					ObjectDetected(time_stamp, curr_label_++, pos));
			return 0;
		}

		// Clean old objects
		maintain();

		return 1
				/ (1
						+ exp(
								-((float) ((*min_it).pos_detections_
										- min_detections_))));
	}
	;

	float computeDistance(cv::Point p1, cv::Point p2) {
		return sqrt(pow((float)(p1.x - p2.x), 2) + pow((float)(p1.y - p2.y), 2));
	}

private:
	int min_detections_;
	float live_time_;
	float thres_;
	unsigned int curr_label_;
	std::list<ObjectDetected> object_list_;

	// Clean the list of old objects
	void maintain() {
		std::list<ObjectDetected>::iterator it;
		for (it = object_list_.begin(); it != object_list_.end(); it++) {
			float diff = abs(
					ros::Time::now().toSec() - (*it).time_stamp_.toSec());
			// Check life
			if (diff > live_time_) {
				object_list_.erase(it);
				it--;
			}
		}
	}
	;
};

class TrafficLightClassifier {
public:
	TrafficLightClassifier(std::string svm_model_file) {
		cv::Size trainingPadding_ = cv::Size(0, 0);
		cv::Size winStride_ = cv::Size(8, 8);

		HOG_.winSize = Size(16, 48); // Default training images size as used in paper
		HOG_.blockSize = Size(8, 8);
		HOG_.blockStride = Size(4, 4);
		HOG_.cellSize = Size(4, 4);

		SVM_.load(svm_model_file.c_str());
	}
	;

	float predict(const cv::Mat &image) {
		cv::Mat descriptor;

		// Make sure that the image has and appropriated size
		cv::Mat scaled_image;
		cv::cvtColor(image, scaled_image, CV_BGR2GRAY);
		cv::resize(image, scaled_image,
				cv::Size(HOG_.winSize.width, HOG_.winSize.height));

		calculateHOG(scaled_image, descriptor);

		return SVM_.predict(descriptor);
	}
	;

private:
	cv::HOGDescriptor HOG_; // Use standard parameters here
	cv::Size trainingPadding_;
	cv::Size winStride_;
	CvSVM SVM_;

	// Methods
	void calculateHOG(const cv::Mat& image, cv::Mat &descriptor) {
		// Check for mismatching dimensions
		if (image.cols != HOG_.winSize.width
				|| image.rows != HOG_.winSize.height) {
			ROS_ERROR(
					"Error: Image dimensions (%u x %u) do not match HOG window size (%u x %u)!\n",
					image.cols, image.rows, HOG_.winSize.width,
					HOG_.winSize.height);
			return;
		}
		std::vector<cv::Point> locations;
		vector<float> featureVector;
		HOG_.compute(image, featureVector, winStride_, trainingPadding_,
				locations);

		// Convert to to OpenCV matrix format
		descriptor = cv::Mat(featureVector.size(), 1, CV_32FC1,
				featureVector.data());
	}
	;
};

class TrafficLightDetector {
public:
	TrafficLightDetector(ros::NodeHandle &nh) {
		std::string output_topic, svm_model_file,
				arrow_template_file, camera_lidar_calibration_file;
		nh.param("output", output_topic, string("/traffic_light_detector"));
		nh.param("svm_model_url", svm_model_file, string("svm_model.yaml"));
		nh.param("arrow_template", arrow_template_file,
				string("arrow_template.png"));
		nh.param("calibration_url", camera_lidar_calibration_file,
				string("config_file.yaml"));
		nh.param("sky_frac", sky_fac_, 0.0);
		nh.param("ground_fac", ground_fac_, 0.6);
		nh.param("display", display_, true);

#ifdef DEBUG
		output_topic = "/world_object";
		svm_model_file = "/home/dani/catkin_ws/src/kut_ugv/kut_ugv_sensor_fusion/calib/traffic_light_model.yaml";
		camera_lidar_calibration_file = "/home/dani/catkin_ws/src/kut_ugv/kut_ugv_sensor_fusion/calib/calib.yaml";
		arrow_template_file = "/home/dani/catkin_ws/src/kut_ugv/kut_ugv_sensor_fusion/config/templates/arrow_template.png";
#endif

		detection_buffer_ = new DetectionBuffer(MIN_DETECTIONS, TIME_ALIVE,
				CONT_DETECTION_DIST_THRES);
		classifier_ = new TrafficLightClassifier(svm_model_file);
		arrow_template_left_ = cv::imread(arrow_template_file,
				CV_LOAD_IMAGE_GRAYSCALE);

		if (arrow_template_left_.empty()) {
			ROS_ERROR("Could not open template file!\n");
			exit(-1);
		}
		cv::resize(arrow_template_left_, arrow_template_left_, TM_SIZE, 0, 0,
				INTER_CUBIC);
		cv::flip(arrow_template_left_, arrow_template_right_, 1);
		lidar_to_image_ = new LidarImageConverter(
				camera_lidar_calibration_file);

		// Publisher
		pub_ = nh.advertise<kut_ugv_msgs::WorldObject>(output_topic, 1000);
	}
	;

	~TrafficLightDetector() {
		delete classifier_;
		delete lidar_to_image_;
		delete detection_buffer_;
	}
	;

	void callBack(const sensor_msgs::ImageConstPtr& im_msg,
			const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list) {
		// Get Image
		cv_bridge::CvImagePtr imgPtr;
		try {
			imgPtr = cv_bridge::toCvCopy(im_msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Size imSize = imgPtr->image.size();
		cv::Mat im2Show;
		cv::Mat imHSV;
		cv::Mat redMask, greenMask;

		// Get HSV image
		cv::cvtColor(imgPtr->image, imHSV, CV_BGR2HSV);

		// Segment image
		redMask = impt::getThresholdedImage(imHSV, cv::Scalar(170, 100, 120),
				cv::Scalar(30, 155, 135));
		greenMask = impt::getThresholdedImage(imHSV, cv::Scalar(70, 100, 120),
				cv::Scalar(30, 155, 135));

//		cv::erode(redMask,redMask,cv::Mat());
//		cv::dilate(redMask,redMask,cv::Mat());

		// Find bounding boxes
		vector<Rect> red_bound_rect, green_bound_rect;
		cv::Mat redMaskCopy, greenMaskCopy;
		redMask.copyTo(redMaskCopy);
		greenMask.copyTo(greenMaskCopy);
		computeBB(redMaskCopy, red_bound_rect);
		computeBB(greenMaskCopy, green_bound_rect);
		vector<unsigned int> red_idx, green_idx;
		filterBB(red_bound_rect, red_idx);
		filterBB(green_bound_rect, green_idx);

		// Calculate red traffic light BB
		vector<Rect> red_traffic_light_bb;
		float red_data_param[5] = { 0.5, 3, 5, 2, 6 };
		vector<float> red_params(red_data_param, red_data_param + 5);
		computerTrafficLightBB(red_bound_rect, red_idx, red_traffic_light_bb,
				red_params);
		// Calculate green traffic light BB
		vector<Rect> green_traffic_light_bb, arrow_green_tf_bb;
		float green_data_param[5] = { 0.5, 3, 5, 2, 1.15 };
		vector<float> green_params(green_data_param, green_data_param + 5);
		computerTrafficLightBB(green_bound_rect, green_idx,
				green_traffic_light_bb, green_params);
		// Second type of green traffic light for arrow
		float arrow_data_param[5] = { 1, 3, 5, 2, 2 };
		vector<float> arrow_params(arrow_data_param, arrow_data_param + 5);
		computerTrafficLightBB(green_bound_rect, green_idx, arrow_green_tf_bb,
				arrow_params);
		green_traffic_light_bb.insert(green_traffic_light_bb.end(),
				arrow_green_tf_bb.begin(), arrow_green_tf_bb.end());

		// Display
		imgPtr->image.copyTo(im2Show);

//		// Saving parameters
//		vector<int> compression_params;
//		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//		compression_params.push_back(0);

		std::vector<cv::Point2f> camera_points;
		fromLObject2Camera(object_list, camera_points);

		if (display_) {
			// Print red bounding boxes
			for (int i = 0; i < red_idx.size(); i++) {
				cv::rectangle(im2Show, red_bound_rect[red_idx[i]],
						cv::Scalar(0, 0, 255), 1);
			}

			// Print green bounding boxes
			for (int i = 0; i < green_idx.size(); i++) {
				cv::rectangle(im2Show, green_bound_rect[green_idx[i]],
						cv::Scalar(0, 255, 0), 1);
			}
		}

		// Print red traffic light bounding boxes
		for (int i = 0; i < red_traffic_light_bb.size(); i++) {

			cv::Rect r = red_traffic_light_bb[i];
			if (!((r.x < 0) || ((r.x + r.width) > imSize.width) || (r.y < 0)
					|| ((r.y + r.height) > imSize.height) || (r.width == 0)
					|| (r.height == 0) || (r.y < imSize.height * sky_fac_)
					|| (r.y > imSize.height * (1 - ground_fac_)))) {

				cv::Mat roi(imgPtr->image, r);
				cv::resize(roi, roi, cv::Size(16, 48));
				float response = classifier_->predict(roi);

				if (response == 1) {
					if (display_) {
						cv::rectangle(im2Show, red_traffic_light_bb[i],
								cv::Scalar(0, 0, 255), 2);
						cv::putText(im2Show, "Red Traffic Light",
								cv::Point(r.x - 100, r.y - 5),
								cv::FONT_HERSHEY_SIMPLEX, 0.5,
								cv::Scalar(255, 100, 0), 2);
					}

					// Get position
					cv::Point2f pos(0, 0);
					std::vector<unsigned int> idx_list;
					getLidarCandidates(camera_points, r.x + r.width / 2,
							LIDAR_IMAGE_THRES, imSize, idx_list);
					unsigned int closet_point_idx = computeCloserLidar(
							object_list, idx_list);
					if (closet_point_idx != -1) {
						pos =
								cv::Point2f(
										object_list->object[idx_list[closet_point_idx]].centroid.x,
										object_list->object[idx_list[closet_point_idx]].centroid.y);

						cv::circle(im2Show,
								camera_points[idx_list[closet_point_idx]], 8,
								cv::Scalar(0, 0, 255), -1);

						for (unsigned int j = 0; j < idx_list.size(); j++) {
							cv::circle(im2Show, camera_points[idx_list[j]], 5,
									cv::Scalar(255, 0, 0), -1);
						}

						float probability = detection_buffer_->getProbability(
								ros::Time::now(), cv::Point(r.x, r.y));
						if (probability > 0.5) {
							publishData(object_list->header, pos,
									kut_ugv_msgs::WorldObject::LIGHT_RED);
						}
					}

//					stringstream ss;
//					ss << file_count++;
//					cv::imwrite(
//							"/home/dani/traffic_light/traffic_light4_" + ss.str()
//									+ ".png", roi, compression_params);
				}
			}
		}

		// Print green traffic light bounding boxes
		for (int i = 0; i < green_traffic_light_bb.size(); i++) {

			cv::Rect r = green_traffic_light_bb[i];
			if (!((r.x < 0) || ((r.x + r.width) > imSize.width) || (r.y < 0)
					|| ((r.y + r.height) > imSize.height) || (r.width == 0)
					|| (r.height == 0) || (r.y < imSize.height * sky_fac_)
					|| (r.y > imSize.height * (1 - ground_fac_)))) {

				cv::Mat roi(imgPtr->image, r);
				cv::resize(roi, roi, cv::Size(16, 48));
				float response = classifier_->predict(roi);

				if (response == 1) {
					// Green light
					unsigned int type = kut_ugv_msgs::WorldObject::LIGHT_GREEN;

					if (display_) {
						cv::rectangle(im2Show, green_traffic_light_bb[i],
								cv::Scalar(0, 255, 0), 2);
						cv::putText(im2Show, "Green Traffic Light",
								cv::Point(r.x - 100, r.y - 5),
								cv::FONT_HERSHEY_SIMPLEX, 0.5,
								cv::Scalar(255, 100, 0), 2);
					}
					// Check arrow
					cv::Mat maskRoi(greenMask, green_bound_rect[green_idx[ i % green_traffic_light_bb.size()/2 ]]);
					cv::resize(maskRoi, maskRoi, TM_SIZE, 0, 0, INTER_CUBIC);
					cv::Point minLoc, maxLoc;

					if(display_)
					{
						imshow("arrow mask",maskRoi);
						imshow("green bb",roi);
					}

					// Left arrow
					cv::Mat response;
					cv::matchTemplate(maskRoi, arrow_template_left_, response,
							CV_TM_CCORR_NORMED);
//					cv::normalize( response, response, 0, 1, NORM_MINMAX, -1, Mat() );
					double minValL, maxValL;
					cv::minMaxLoc(response, &minValL, &maxValL, &minLoc,
							&maxLoc, Mat());
					// Right arrow
					cv::matchTemplate(maskRoi, arrow_template_right_, response,
							CV_TM_CCORR_NORMED);
//					cv::normalize( response, response, 0, 1, NORM_MINMAX, -1, Mat() );
					double minValR, maxValR;
					cv::minMaxLoc(response, &minValR, &maxValR, &minLoc,
							&maxLoc, Mat());

//					printf("Left arrow:\nMax: %f\nMin: %f\n",minValL,maxValL);
//					printf("Right arrow:\nMax: %f\nMin: %f\n\n",minValR,maxValR);

					// Check if right or left
					if (max(maxValL, maxValR) >= TM_THRES) {
						if (maxValL > maxValR)
							type = kut_ugv_msgs::WorldObject::LIGHT_ARROW_LEFT;
						else
							type = kut_ugv_msgs::WorldObject::LIGHT_ARROW_RIGHT;
					}

					if (display_) {
						if (type == kut_ugv_msgs::WorldObject::LIGHT_ARROW_LEFT)
							cv::putText(im2Show, "<-",
									cv::Point(r.x, r.y + r.height + 5),
									cv::FONT_HERSHEY_SIMPLEX, 0.5,
									cv::Scalar(255, 100, 0), 2);
						if (type
								== kut_ugv_msgs::WorldObject::LIGHT_ARROW_RIGHT)
							cv::putText(im2Show, "->",
									cv::Point(r.x, r.y + r.height + 5),
									cv::FONT_HERSHEY_SIMPLEX, 0.5,
									cv::Scalar(255, 100, 0), 2);
					}

					// Get position
					cv::Point2f pos(0, 0);
					std::vector<unsigned int> idx_list;
					getLidarCandidates(camera_points, r.x + r.width / 2,
							LIDAR_IMAGE_THRES, imSize, idx_list);
					unsigned int closet_point_idx = computeCloserLidar(
							object_list, idx_list);
					if (closet_point_idx != -1) {
						pos =
								cv::Point2f(
										object_list->object[idx_list[closet_point_idx]].centroid.x,
										object_list->object[idx_list[closet_point_idx]].centroid.y);

						cv::circle(im2Show,
								camera_points[idx_list[closet_point_idx]], 8,
								cv::Scalar(0, 0, 255), -1);

						for (unsigned int j = 0; j < idx_list.size(); j++) {
							cv::circle(im2Show, camera_points[idx_list[j]], 5,
									cv::Scalar(255, 0, 0), -1);
						}

						float probability = detection_buffer_->getProbability(
								ros::Time::now(), cv::Point(r.x, r.y));
						if (probability > 0.5) {
							publishData(object_list->header, pos, type);
						}
					}

//					stringstream ss;
//					ss << file_count++;
//					cv::imwrite(
//							"/home/dani/traffic_light/traffic_light4_" + ss.str()
//									+ ".png", roi, compression_params);
				}
			}
		}

		if (display_) {
			// Draw lidar projections
			for (unsigned int i = 0; i < camera_points.size(); i++) {
				cv::circle(im2Show, camera_points[i], 2,
						cv::Scalar(100, 255, 0), -1);
			}

			// Draw guides lines
			cv::line(im2Show, cv::Point(0, imSize.height * sky_fac_),
					cv::Point(imSize.width, imSize.height * sky_fac_),
					cv::Scalar(255, 0, 0), 1);
			cv::line(im2Show, cv::Point(0, imSize.height * (1 - ground_fac_)),
					cv::Point(imSize.width, imSize.height * (1 - ground_fac_)),
					cv::Scalar(255, 0, 0), 1);

			cv::imshow("Camera Results", im2Show);
			//		cv::imshow("Camera", imgPtr->image);

			char key = cv::waitKey(3);

			if (key == 'p') {
				impt::getPixelFromWindow(imHSV);

				Vec3b pixel = impt::getPixelClicked();

				printf("Pixel value: [%d,%d,%d]\n", pixel[0], pixel[1],
						pixel[2]);
			}
		}
	}
	;

private:
	ros::Publisher pub_;

	// Classifier
	TrafficLightClassifier *classifier_;

	// Projector
	LidarImageConverter *lidar_to_image_;

	// Buffer of dections
	DetectionBuffer *detection_buffer_;

	// Template
	cv::Mat arrow_template_left_, arrow_template_right_;
	double sky_fac_, ground_fac_;

	// Display
	bool display_;

	// Methods
	/*
	 * Compute bounding boxes
	 */
	void computeBB(const Mat &binIm, vector<Rect> &outputBB) {
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		/// Find contours
		cv::findContours(binIm, contours, hierarchy, CV_RETR_TREE,
				CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		outputBB.clear();
		outputBB.resize(contours.size());

		/// Approximate contours to polygons
		vector<vector<Point> > contours_poly(contours.size());
		for (int i = 0; i < contours.size(); i++) {
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
			outputBB[i] = boundingRect(Mat(contours_poly[i]));
		}
	}
	;

	/*
	 * Filter bounding boxes
	 */
	void filterBB(const vector<Rect> &boundRect, vector<unsigned int> &idx) {
		idx.clear();
		idx.reserve(boundRect.size());
		for (int i = 0; i < boundRect.size(); i++) {
			if (boundRect[i].height > 2 && boundRect[i].height < 15
					&& boundRect[i].width > 2 && boundRect[i].width < 15) {
				idx.push_back(i);
			}
		}
	}
	;

	/*
	 * Filter bounding boxes
	 */
	void computerTrafficLightBB(const vector<Rect> &boundRect,
			const vector<unsigned int> &idx, vector<Rect> &trafficLightBB,
			const vector<float> &params) {
		trafficLightBB.clear();
		trafficLightBB.reserve(boundRect.size());
		for (int i = 0; i < idx.size(); i++) {
			cv::Rect tlbb = boundRect[idx[i]]; // Traffic light bounding box

			// Resize according to the dimensions of the traffic light
			int w_offest = tlbb.width * params[0] + params[2];
			tlbb.width += w_offest;
			int h_offest = abs(tlbb.height - tlbb.width * params[1]);
			tlbb.height += h_offest;
			tlbb.x -= w_offest / params[3];
			tlbb.y -= h_offest / params[4];

			// Save bb
			trafficLightBB.push_back(tlbb);
		}
	}
	;

	void fromLObject2Camera(
			const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list,
			std::vector<cv::Point2f> &camera_points) {
		double data_point[3];
		data_point[1] = 0;

		camera_points.clear();
		camera_points.resize(object_list->object.size());

		for (unsigned int i = 0; i < object_list->object.size(); i++) {
			// Set 3D point from lidar
			data_point[0] = -object_list->object[i].centroid.y;
			data_point[2] = object_list->object[i].centroid.x;

			cv::Mat Pl(3, 1, CV_64F, data_point);
			// Convert 3D point to 2D image point

			cv::Point2f p = lidar_to_image_->from3d2image(Pl);

			// Save 2D image point
			camera_points[i] = p;
		}
	}
	;

	/*
	 * This function gets a list of camera points and returns a list of points
	 * that are to a distance lower to the threshold in x to the given X position.
	 */
	void getLidarCandidates(std::vector<cv::Point2f> camera_points,
			const float x, float thres, cv::Size imSize,
			std::vector<unsigned int> &idx_list) {

		idx_list.clear();
		if (camera_points.empty()) {
			return;
		}

		// Find first point on the image
		for (unsigned int i = 0; i < camera_points.size(); i++) {
			// Check if the point is on the image
			if ((camera_points[i].x > -thres)
					&& (camera_points[i].x < (imSize.width + thres))
					&& (camera_points[i].y > 0)
					&& (camera_points[i].y < imSize.height)) {

				float diff = abs(x - camera_points[i].x);
				if (diff < thres) {
					idx_list.push_back(i);
				}
			}
		}
	}
	;

	/*
	 * This function gets a list of camera points and returns a list of points
	 * that are to a distance lower to the threshold in x to the given X position.
	 */
	int computeCloserLidar(
			const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list,
			const std::vector<unsigned int> &idx_list) {

		// Check if no candidates
		if (idx_list.empty()) {
			return -1;
		}

		float best_dist;
		unsigned int best_idx = 0;

		// Calculate distance
		best_dist = sqrt(
				object_list->object[idx_list[0]].centroid.x
						* object_list->object[idx_list[0]].centroid.x
						+ object_list->object[idx_list[0]].centroid.y
								* object_list->object[idx_list[0]].centroid.y);

		// Find first point on the image
		for (unsigned int i = 1; i < idx_list.size(); i++) {
			// Check if the point is on the image

			// Calculate distance
			float curr_dist =
					sqrt(
							object_list->object[idx_list[i]].centroid.x
									* object_list->object[idx_list[i]].centroid.x
									+ object_list->object[idx_list[i]].centroid.y
											* object_list->object[idx_list[i]].centroid.y);

			if (curr_dist < best_dist) {
				best_dist = curr_dist;
				best_idx = i;
			}
		}

		return best_idx;
	}
	;

	void publishData(const std_msgs::Header &header, cv::Point2f pos,
			unsigned int type) {
		kut_ugv_msgs::WorldObject publish_me;

		publish_me.header.frame_id = header.frame_id;
		publish_me.header.stamp = header.stamp;
		publish_me.pose.position.x = pos.x;
		publish_me.pose.position.y = pos.y;
		publish_me.pose.position.z = 0;
		publish_me.pose.orientation.w = 1;
		publish_me.type = type;
		publish_me.id = 0;

		pub_.publish(publish_me);
	}
	;
};

int main(int argc, char** argv) {
	string laser_objects_topic, camera_topic;
	ros::init(argc, argv, "pedestrian_detection");
	ros::NodeHandle nh("~");

	nh.param("laser_objects_topic", laser_objects_topic, string("/objects"));
	nh.param("camera_topic", camera_topic, string("/image_raw"));

#ifdef DEBUG
	laser_objects_topic = "/bumper_laser/scan/objects";
	camera_topic = "/mono_cam/image_rect_color";
#endif

	TrafficLightDetector traffic_light_detector(nh);

	cout << "Subscribing topics: " << laser_objects_topic << endl;
	cout << "Subscribing topics: " << camera_topic << endl;

	// Subscribers
	message_filters::Subscriber<kut_ugv_sensor_fusion::lidar_object_list> lidar_labels_sub(
			nh, laser_objects_topic, 1);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, camera_topic,
			1);

	// Subscribe and synchronize
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
			kut_ugv_sensor_fusion::lidar_object_list> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),
			image_sub, lidar_labels_sub);

	sync.registerCallback(
			boost::bind(&TrafficLightDetector::callBack,
					&traffic_light_detector, _1, _2));

	ROS_INFO(
			"WARNING: devices are synchronized, so the speed is setted up to the slowest device!");

	ros::spin();

	return 0;
}

