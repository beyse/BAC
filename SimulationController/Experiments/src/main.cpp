//author: sebastian beyer 

#include <iostream>

#include <boost\filesystem.hpp>
#include <boost\foreach.hpp>
#include <boost\lexical_cast.hpp>

#include <vector>
#include <string>
#include <memory>

#include <opencv2\opencv.hpp>

#include <conio.h>
#include <iostream>
#include <sstream>

#include <opencv2/ml/ml.hpp>

using namespace std;
namespace fs = boost::filesystem;

class ImageProcessing {
public:
	ImageProcessing() {

	}

	void setImage(const cv::Mat& image) {
		cv::imshow("Simulation Output", image);
		cv::Mat img = image.clone();
		double t1 = 0.3, t2 = 0.8;
		cv::blur(img, img, cv::Size(11, 11));
		cv::imshow("Processed", img);

		cv::waitKey(1);
	}
};

int main() {
	
	std::string groundTruthDir = "C:\\Users\\Sebastian\\Desktop\\BAC\\GT-30-10-16";
	std::map<int /*frame number*/, cv::Mat> frames;
	std::map<int /*frame number*/, double> steeringAngles;
	

	//iterator over all files in directory
	fs::path targetDir(groundTruthDir.c_str());
	fs::directory_iterator it(targetDir), eod;
	std::string steeringAngleFilePath = "";

	BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod)) {
		if (fs::is_regular_file(p)) {
			if (p.has_extension()) {
				std::string extension = p.extension().string();
				if (extension == ".bmp") {
					std::string filename = p.filename().string();
					//remove extension
					size_t lastDotIndex = filename.find_last_of('.');
					filename = filename.substr(0, lastDotIndex);
					int frameNumber = boost::lexical_cast<int>(filename);
					cv::Mat image = cv::imread(p.string(), CV_LOAD_IMAGE_GRAYSCALE);
					frames.insert(std::pair<int, cv::Mat>(frameNumber, image));
				} else if (extension == ".csv") {
					steeringAngleFilePath = p.string();
				}
			}
		}
	}

	//read file with steering angles
	if (steeringAngleFilePath == "") {
		return -1;
	}

	std::ifstream steeringAngleFile(steeringAngleFilePath);
	std::string line;
	int number = 0;
	while (std::getline(steeringAngleFile, line)) {
		number++;
		double steeringAngle = boost::lexical_cast<double>(line);
		steeringAngles.insert(std::pair<int, double>(number, steeringAngle));
	}

	int attributes_per_sample = 64 * 32; //size of one image
	int number_of_training_samples = 10000; //number of images
	int number_of_classes = 31; //size of output vector for steering direction
	int hidden_layer_size = 5; //size of the hidden layer of neurons

	cv::Mat training_data = cv::Mat(number_of_training_samples, attributes_per_sample, CV_32FC1);
	cv::Mat training_classifications = cv::Mat(number_of_training_samples, number_of_classes, CV_32FC1, 0.0);

	for (int i = 0; i < number_of_training_samples; i++) {
		int classNumber = (int)steeringAngles[i + 1];
		classNumber = std::min(classNumber, ((number_of_classes -1)/2));
		classNumber = std::max(classNumber, -((number_of_classes - 1) / 2));
		classNumber += ((number_of_classes - 1) / 2);
		training_classifications.at<float>(i, classNumber) = 1.0;

		//cv::Rect region(0, i, number_of_classes, 1);
		//cv::GaussianBlur(training_classifications(region), training_classifications(region), cv::Size(31, 1), 2.0, 2.0, cv::BORDER_CONSTANT);
	}
	
	//training_classifications *= 5; //normalize the image to 1;

	//for (int i = 0; i < number_of_training_samples; i++) {
	//	int classNumber;
	//	if (steeringAngles[i] > 3.0) {
	//		classNumber = 2;
	//	} else if (steeringAngles[i] < -3.0) {
	//		classNumber = 0;
	//	} else {
	//		classNumber = 1;
	//	}
	//	training_classifications.at<float>(i, classNumber) = 1.0;
	//}

	for (int i = 0; i < number_of_training_samples; i++) {
		cv::Mat imageFloat;
		frames[i + 1].convertTo(imageFloat, CV_32FC1, (1 / 255.0));
		cv::Mat row = imageFloat.reshape(1, 1);

		row.copyTo(training_data(cv::Rect(0, i, row.cols, row.rows)));
	}

	std::vector<int> layerSizes = { attributes_per_sample, hidden_layer_size, number_of_classes };


	cv::Ptr<cv::ml::ANN_MLP> ann = cv::ml::ANN_MLP::create();

	ann->setLayerSizes(layerSizes);
	ann->setActivationFunction(cv::ml::ANN_MLP::SIGMOID_SYM);
	cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(training_data, cv::ml::SampleTypes::ROW_SAMPLE, training_classifications);
	trainData->setTrainTestSplitRatio(0.95);
	cv::TermCriteria termCriteria;
	
	termCriteria.maxCount = 1;
	termCriteria.epsilon = 10000;
	termCriteria.type = cv::TermCriteria::EPS + cv::TermCriteria::COUNT;
	ann->setTermCriteria(termCriteria);
	ann->setTrainMethod(cv::ml::ANN_MLP::RPROP);

	std::cout << "Starting to train network..." << std::endl;
	ann->train(trainData);
	cv::Mat out;
	cv::Mat idx;
	try {
		float error = ann->calcError(trainData, true, out);
		idx = trainData->getTestSampleIdx();
		std::cout << error << std::endl;
	} catch (std::exception e) {
		std::string w = e.what();
		std::cout << w << std::endl;
	}
	cv::namedWindow("image", cv::WINDOW_FREERATIO);
	for (int i = 0; i < idx.cols; i++) {
		int frameNr = idx.at<int>(0, i);
		int result = out.at<float>(i, 0);
		cv::Mat image = frames[frameNr + 1];
		std::cout << "\r";
		for (int c = 0; c < result; c++)
		{
			std::cout << "-";
		}
		std::cout << "*";
		for (int c = result+1; c < number_of_classes; c++)
		{
			std::cout << "-";
		}
		cv::imshow("image", image);
		cv::waitKey(-1);
	}
	
	return 0;
}

