#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "Classifier.hpp"
#include "ColorSignDetector.h"
#include "ihls.h"
#include "nhs.h"
#include "PostProcessing.h"

#include <kut_ugv_msgs/WorldObject.h>
#include <kut_ugv_sensor_fusion/lidar_object_list.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ImProcessingTools.h"

const static float DIST_THRES = 50; // Distance threshold
// Template matching constant threshold
const static float CONT_DETECTION_DIST_THRES = 25; // Continuous detection distance threshold
const static float MIN_DETECTIONS = 5;
const static float TIME_ALIVE = 1;

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
		return sqrt(pow((float)p1.x - p2.x, 2) + pow((float)p1.y - p2.y, 2));
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

/******************************************************************************/
/*                            Projector                             */
/******************************************************************************/
class LidarProjector {
public:
	LidarProjector(const cv::Mat& _K, const cv::Mat& _R, const cv::Mat& _t) {
		_K.copyTo(K);
		_R.copyTo(R);
		_t.copyTo(t);
	}
	;
	LidarProjector(std::string calibration_file) {
		loadCalibrationParams(calibration_file);
	}
	;

	virtual ~LidarProjector() {
	}
	;

	/*
	 * Convert a 3D point (X Y Z)' from 3d to image coordinates.
	 */
	cv::Point2f from3d2image(const cv::Mat& Pl) {
		cv::Mat Pi; // Point image
		double w;

		Pi = K * (R * Pl + t);
		w = Pi.at<double>(2, 0);

		return cv::Point2f(Pi.at<double>(0, 0) / w, Pi.at<double>(1, 0) / w);
	}
	;

	/*
	 * Convert a 3D point (X Y Z)' from 3d to image coordinates.
	 */
	cv::Point2f from3d2image(const cv::Point3f& Pw) {
		cv::Mat Pi; // Point image
		double w;

		double data_point[3];
		data_point[0] = Pw.x;
		data_point[1] = Pw.y;
		data_point[2] = Pw.z;
		cv::Mat PwMat(3, 1, CV_64F, data_point); // Point 3d image

		Pi = K * (R * PwMat + t);
		w = Pi.at<double>(2, 0);

		return cv::Point2f(Pi.at<double>(0, 0) / w, Pi.at<double>(1, 0) / w);
	}
	;

private:
	cv::Mat K;
	cv::Mat R;
	cv::Mat t;

	/*
	 * loadCalibrationParams data from a yaml file.
	 */
	bool loadCalibrationParams(std::string file) {
		cv::FileStorage fs2(file, cv::FileStorage::READ);

		ROS_INFO("Calibration path: %s\n", file.c_str());

		if (!fs2.isOpened()) {
			ROS_ERROR("Could not open Calibration file.\n");
			return false;
		}

		fs2["K"] >> K;
		fs2["R"] >> R;
		fs2["t"] >> t;

		fs2.release();

		return true;
	}
	;
};

static const char WINDOW[] = "Image window";

class TrafficSignDetectorNode {
public:
	TrafficSignDetectorNode(ros::NodeHandle &nh) :
			it_(nh) {
		string camera_topic, output_topic, svm_model_file, camli_file;
		nh.param("camera_topic", camera_topic, string("/mono_cam/image_rect_color"));
		nh.param("output", output_topic, string("/traffic_sign"));
        nh.param("svm_model_url", svm_model_file, string("/Users/artemlenskiy/ros/ugv_ws/src/kut_ugv/kut_ugv_sensor_fusion/calib/traffic_sign_svm.yaml"));
		//nh.param("svm_model_url", svm_model_file, string("svm_model.yaml"));
		nh.param("calibration_url", camli_file, string("/Users/artemlenskiy/ros/ugv_ws/src/kut_ugv/kut_ugv_sensor_fusion/calib/calib.yaml"));
        //nh.param("calibration_url", camli_file, string("config_file.yaml"));

		sign_pub_ = nh.advertise<kut_ugv_msgs::WorldObject>(output_topic, 1000);

		cout << svm_model_file << endl;

		classif_ = new ts::Classifier(svm_model_file);
		lidar_to_image_ = new LidarProjector(camli_file);
		detection_buffer_ = new DetectionBuffer(MIN_DETECTIONS, TIME_ALIVE,
				CONT_DETECTION_DIST_THRES);
	}
	;

	~TrafficSignDetectorNode() {
		delete classif_;
		delete lidar_to_image_;
		delete detection_buffer_;
	}
	;

	void callBack(const sensor_msgs::ImageConstPtr& im_msg,
			const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(im_msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

		cv::Mat frame_aux = cv_ptr->image(cv::Rect(0, 60, cv_ptr->image.cols, 200));
		cv::Mat frame;
		frame_aux.copyTo(frame);
		cv::Mat result;

		frame.copyTo(result);

		/// your code add here
		Processing(cv_ptr->image, frame, result, object_list);

		cv::imshow(WINDOW, result);
		cv::imshow("Lidar", cv_ptr->image);
		cv::waitKey(3);
	}
	;

private:
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher sign_pub_;
	ts::Classifier *classif_;
	LidarProjector *lidar_to_image_;

	// Buffer of dections
	DetectionBuffer *detection_buffer_;

	void Processing(cv::Mat& original, cv::Mat& frame, cv::Mat& result,
			const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list) {

//		// Saving parameters
//		vector<int> compression_params;
//		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//		compression_params.push_back(0);

		kut_ugv_msgs::WorldObject msg;
		cv::Mat blueRcg, greenRcg, redRcg;
		BoundaryBox redBB[255], greenBB[256], blueBB[256]; //preallocate memory to store rects
		cv::Scalar blueColor(255, 0, 0), yelColor(0, 255, 255), redColor(0, 0,
				255);
		int i;
		unsigned char redBBnum, greenBBnum, blueBBnum;
		///////////////////////
		cv::Mat blueColorsPart, maskBlue ;
	//cv::Mat greenColorsPart, maskGreen ;
	cv::Mat redColorsPart, maskRed ;
	cv::Mat HSV ;
	///////////////////
	
	char blueMemb[180][255] ;
	char greenMemb[180][255] ;
	char redMemb[180][255] ;
    
        int erosion_size = 2;
	///////////////////////////////
	
	//precalculate color membership functions
	constructColorMemb(blueMemb, 0.25, 'b') ;
	//constructColorMemb(greenMemb, 0.05, 'g') ;
	constructColorMemb(redMemb, 0.25, 'r') ;

	makeMat(blueColorsPart, maskBlue, frame) ;
	//makeMat(greenColorsPart, maskGreen, frame) ;
	makeMat(redColorsPart, maskRed, frame) ;

	cvtColor(frame, HSV, CV_BGR2HSV) ;

	colorFilter(HSV, maskBlue, blueMemb) ; // Fill up mask with ones for blue pixels in the input frame
	//colorFilter(HSV, maskGreen, greenMemb) ; // Fill up mask with ones for green pixels in the input frame
	colorFilter(HSV, maskRed, redMemb) ; // Fill up mask with ones for red pixels in the input frame

	frame.copyTo(blueColorsPart, maskBlue) ; // Filer out all non blue pixels.
	//frame.copyTo(greenColorsPart, maskGreen) ; // Filer out all non green pixels.
	frame.copyTo(redColorsPart, maskRed) ; // Filer out all non red pixels.
	////////////////////////////////////////////////////////
		/*cv::Mat nhs_red(frame.size(), CV_8UC1);
		cv::Mat nhs_blue(frame.size(), CV_8UC1);
		cv::Mat nhs_green(frame.size(), CV_8UC1);

		cv::Mat ihls_image = convert_rgb_to_ihls(frame);
		nhs_red = convert_ihls_to_nhs(ihls_image, 2);
		nhs_blue = convert_ihls_to_nhs(ihls_image, 1);
		nhs_green = convert_ihls_to_nhs(ihls_image, 0);

		PostProcessing r(nhs_red);
		Mat rimg = r.FilterImage();
			Mat temp ;
			frame.copyTo(temp, rimg) ;
			imshow("red", temp) ;
		PostProcessing b(nhs_blue);
		Mat bimg = b.FilterImage();
			Mat temp2 ;
			frame.copyTo(temp2, bimg) ;
			imshow("blue", temp2) ;

		char key = cv::waitKey(3);

		if (key == 'p') {
			impt::getPixelFromWindow(ihls_image);

			Vec3b pixel = impt::getPixelClicked();

			printf("Pixel value: [%d,%d,%d]\n", pixel[0], pixel[1],
					pixel[2]);
		}*/

		blueBBnum = blobFinder(maskBlue ,blueBB);//////////changed
		//	  greenBBnum = blobFinder(nhs_green, greenBB);
        cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                                                cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                cv::Point( erosion_size, erosion_size ) );
        
        /// Apply the erosion operation
        dilate( maskRed, maskRed, element );
		redBBnum = blobFinder(maskRed, redBB);
		imshow("red",maskRed) ;
		imshow("blue",maskBlue) ;

		// Project laser points
		std::vector<cv::Point2f> camera_points;
		fromLObject2Camera(object_list, camera_points);

		// Draw lidar projections
		for (unsigned int i = 0; i < camera_points.size(); i++) {
			cv::circle(original, camera_points[i], 2, cv::Scalar(255, 0, 0),
					-1);
		}

		cv::Size size(50, 50);

		char save_file[200];
		for (i = 0; i < blueBBnum; i++) {
			int w = (blueBB[i].x2 - blueBB[i].x1);
			int h = (blueBB[i].y2 - blueBB[i].y1);
			int sqr_size = max(w, h);
			int offset = (sqr_size * 0.25 + 5);

			cv::Rect r(blueBB[i].x1 - (offset + sqr_size / 2) / 2,
					blueBB[i].y1 - (offset + sqr_size / 2) / 2,
					sqr_size + (offset * 2), sqr_size + (offset * 2));

			if ((r.x > 0) && ((r.x + r.width) < frame.cols) && (r.y > 0)
					&& ((r.y + r.height) < frame.rows) && r.width > 25) {
					
					cv::rectangle(result, r, blueColor, 1, 0);
				
				blueRcg = frame(r);
				cv::resize(blueRcg, blueRcg, size); // When if showing 100x100, the other 30x30

				int response = (int) classif_->predict(blueRcg);

				msg.header = object_list->header;
				// Get position
				cv::Point2f pos(0, 0);
				std::vector<unsigned int> idx_list;
				getLidarCandidates(camera_points, r.x + r.width / 2, DIST_THRES, original.size(), idx_list);
				unsigned int closet_point_idx = computeCloserLidar(object_list, idx_list);
				if (closet_point_idx != -1) {
					pos = cv::Point2f(object_list->object[ idx_list[closet_point_idx] ].centroid.x,
							object_list->object[ idx_list[closet_point_idx] ].centroid.y);

					cv::circle(original, camera_points[ idx_list[closet_point_idx] ], 8,
							cv::Scalar(0, 0, 255), -1);

					for(unsigned int j = 0; j < idx_list.size(); j++)
					{
						cv::circle(original, camera_points[idx_list[j]], 5,
								cv::Scalar(0, 255, 0), -1);
					}

					float probability = detection_buffer_->getProbability(
							ros::Time::now(), cv::Point(r.x, r.y));
					if (probability > 0.5) {

						msg.pose.position.x = pos.x;
						msg.pose.position.y = pos.y;
						msg.pose.position.z = 0;
						msg.pose.orientation.w = 1;

						if (response == 4) {
							msg.type =
									kut_ugv_msgs::WorldObject::SIGN_PEDESTRIAN_ZONE;
							sign_pub_.publish(msg);
							cv::rectangle(result,
									cv::Point(blueBB[i].x1, blueBB[i].y1),
									cv::Point(blueBB[i].x2, blueBB[i].y2),
									yelColor, 2, 0);

							cv::putText(result, "C r o s s w a l k !",
									cv::Point(result.cols * 0.5,
											result.rows * 2 / 3),
									CV_FONT_HERSHEY_SIMPLEX, 0.5,
									cv::Scalar(255, 0, 0), 3);
							break;
						}
					}
				}

//					sprintf(save_file, "/home/dani/signs/katech_20130926_exp_900_%d.png", file_count++);
//					cv::imwrite(save_file, blueRcg,compression_params);
			} // End if checking bb in image
		}

		char savered[200];
		for (i = 0; i < redBBnum; i++) {
			int w = (redBB[i].x2 - redBB[i].x1);
			int h = (redBB[i].y2 - redBB[i].y1);
			int sqr_size = max(w, h);
			int offset = (sqr_size * 0.25 + 5);

			cv::Rect r(redBB[i].x1 - (offset + sqr_size / 2) / 2,
					redBB[i].y1 - (offset + sqr_size / 2) / 2,
					sqr_size + (offset * 2), sqr_size + (offset * 2));

			if ((r.x > 0) && ((r.x + r.width) < frame.cols) && (r.y > 0)
					&& ((r.y + r.height) < frame.rows) && r.width > 25) {
							cv::rectangle(result, r, redColor, 1, 0);
				//			cv::rectangle(result, cv::Point(redBB[i].x1, redBB[i].y1),
				//										cv::Point(redBB[i].x1+sqr_size, redBB[i].y1+sqr_size), blueColor, 1, 0);
				redRcg = frame(r);
				cv::resize(redRcg, redRcg, size); // When if showing 100x100, the other 30x30

				int response = (int) classif_->predict(redRcg);

				if (response != -1) {
//					cout << "Response: " << response << endl;

					msg.header = object_list->header;
					// Get position
					cv::Point2f pos(0, 0);
					std::vector<unsigned int> idx_list;
					getLidarCandidates(camera_points, r.x + r.width / 2, DIST_THRES, original.size(), idx_list);
					unsigned int closet_point_idx = computeCloserLidar(object_list, idx_list);
					if (closet_point_idx != -1) {
						pos = cv::Point2f(object_list->object[ idx_list[closet_point_idx] ].centroid.x,
								object_list->object[ idx_list[closet_point_idx] ].centroid.y);

						cv::circle(original, camera_points[ idx_list[closet_point_idx] ], 8,
								cv::Scalar(0, 0, 255), -1);

						for(unsigned int j = 0; j < idx_list.size(); j++){
							cv::circle(original, camera_points[idx_list[j]], 5, cv::Scalar(0, 255, 0), -1);
						}

						float probability = detection_buffer_->getProbability(
								ros::Time::now(), cv::Point(r.x, r.y));

//						cout << "Probability: " << probability << endl;

						if (probability > 0.5) {
							msg.pose.position.x = pos.x;
							msg.pose.position.y = pos.y;
							msg.pose.position.z = 0;
							msg.pose.orientation.w = 1;

							switch (response) {
							case 1:
								msg.type =
										kut_ugv_msgs::WorldObject::SIGN_SPEED_MAXIMUM;
								msg.data = 20;
								sign_pub_.publish(msg);
								cv::rectangle(result,
										cv::Point(redBB[i].x1, redBB[i].y1),
										cv::Point(redBB[i].x1 + sqr_size,
												redBB[i].y1 + sqr_size),
										yelColor, 2, 0);
								//cv::Point(redBB[i].x2 , redBB[i].y2), yelColor, 2, 0);

								cv::putText(result, "2 0 k m / h !",
										cv::Point(result.cols * 0.5,
												result.rows * 2 / 3),
										CV_FONT_HERSHEY_SIMPLEX, 0.5,
										cv::Scalar(0, 0, 255), 3);
								break;
							case 2:
								msg.type =
										kut_ugv_msgs::WorldObject::SIGN_SPEED_MINIMUM;
								msg.data = 20;
								sign_pub_.publish(msg);
								cv::rectangle(result,
										cv::Point(redBB[i].x1, redBB[i].y1),
										cv::Point(redBB[i].x1 + sqr_size,
												redBB[i].y1 + sqr_size),
										yelColor, 2, 0);
								//cv::Point(redBB[i].x2 , redBB[i].y2), yelColor, 2, 0);

								cv::putText(result, "Min 2 0 k m / h !",
										cv::Point(result.cols * 0.5,
												result.rows * 2 / 3),
										CV_FONT_HERSHEY_SIMPLEX, 0.5,
										cv::Scalar(0, 0, 255), 3);
								break;
							case 3:

								msg.type =
										kut_ugv_msgs::WorldObject::SIGN_CROSSROAD;
								sign_pub_.publish(msg);

								cv::rectangle(result,
										cv::Point(redBB[i].x1, redBB[i].y1),
										cv::Point(redBB[i].x1 + sqr_size,
												redBB[i].y1 + sqr_size),
										yelColor, 2, 0);
								//cv::Point(redBB[i].x2 , redBB[i].y2), yelColor, 2, 0);

								cv::putText(result, "C r o s s !",
										cv::Point(result.cols * 0.5,
												result.rows * 2 / 3),
										CV_FONT_HERSHEY_SIMPLEX, 0.5,
										cv::Scalar(0, 0, 255), 3);
								break;
							case 5:

								msg.type =
										kut_ugv_msgs::WorldObject::SIGN_ROAD_WORKS;
								sign_pub_.publish(msg);

								cv::rectangle(result,
										cv::Point(redBB[i].x1,redBB[i].y1),
										cv::Point(redBB[i].x1 + sqr_size,
												redBB[i].y1 + sqr_size),
										yelColor, 2, 0);
								//cv::Point(redBB[i].x2 , redBB[i].y2), yelColor, 2, 0);

								cv::putText(result, "R o a d w o r k s !",
										cv::Point(result.cols * 0.5,
												result.rows * 2 / 3),
										CV_FONT_HERSHEY_SIMPLEX, 0.5,
										cv::Scalar(0, 0, 255), 3);
								break;
							} // case

						} // if (probability > 0.5)

					} // if (idx != -1)

				} // if (response == -1)

//				sprintf(save_file, "/home/dani/signs/katech_20130926_exp_900_%d.png", file_count++);
//				cv::imwrite(save_file, redRcg,compression_params);
			} // End if checking bb in image
		};
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
			if ((camera_points[i].x > -thres) && (camera_points[i].x < (imSize.width + thres) )
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
			const kut_ugv_sensor_fusion::lidar_object_listConstPtr& object_list, const
			std::vector<unsigned int> &idx_list) {

		// Check if no candidates
		if (idx_list.empty()) {
			return -1;
		}

		float best_dist;
		unsigned int best_idx = 0;

		// Calculate distance
		best_dist = sqrt(
				object_list->object[ idx_list[0] ].centroid.x
						* object_list->object[ idx_list[0] ].centroid.x
						+ object_list->object[ idx_list[0] ].centroid.y
								* object_list->object[ idx_list[0] ].centroid.y);

		// Find first point on the image
		for (unsigned int i = 1; i < idx_list.size(); i++) {
			// Check if the point is on the image

			// Calculate distance
			float curr_dist = sqrt(
					object_list->object[ idx_list[i] ].centroid.x
							* object_list->object[ idx_list[i] ].centroid.x
							+ object_list->object[ idx_list[i] ].centroid.y
									* object_list->object[ idx_list[i] ].centroid.y);

			if (curr_dist < best_dist)
			{
				best_dist = curr_dist;
				best_idx = i;
			}
		}

		return best_idx;
	}
	;

};

int main(int argc, char** argv) {
	string laser_objects_topic, camera_topic;
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh("~");

	nh.param("laser_objects_topic", laser_objects_topic, string("/bumper_laser/scan/objects"));
	nh.param("camera_topic", camera_topic, string("/mono_cam/image_rect_color"));

//	laser_objects_topic = "/bumper_laser/scan/objects";
//	camera_topic = "/mono_cam/image_rect_color";

	TrafficSignDetectorNode traffic_sign_detector(nh);

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

	sync.registerCallback(boost::bind(&TrafficSignDetectorNode::callBack, &traffic_sign_detector, _1, _2));

	ros::spin();
	return 0;
}
