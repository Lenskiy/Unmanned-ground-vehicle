#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <ros/ros.h>
#include <vector>
#include "opencv2/ml/ml.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/gpu/gpu.hpp"

using namespace cv;

/******************************************************************************/
/*                                Classifier                                  */
/******************************************************************************/
namespace ts {

class Classifier {
public:
	Classifier(std::string svm_model_file) {
		cv::Size trainingPadding_ = cv::Size(0, 0);
		HOG_.winSize = Size(50, 50);
		HOG_.blockSize = Size(10, 10);
		HOG_.blockStride = Size(5, 5);
		HOG_.cellSize = Size(5, 5);

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
	cv::HOGDescriptor HOG_;
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
}
