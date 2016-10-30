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
	
	std::string groundTruthDir = "C:\\Users\\Sebastian\\Desktop\\ground_truth";
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
				if (extension == ".jpg") {
					std::string filename = p.filename().string();
					//remove extension
					size_t lastDotIndex = filename.find_last_of('.');
					filename = filename.substr(0, lastDotIndex);
					int frameNumber = boost::lexical_cast<int>(filename);
					cv::Mat image = cv::imread(p.string());
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

	cv::neura

	return 0;
}

