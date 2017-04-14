//author: sebastian beyer 

#include <iostream>

#include <boost/array.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/atomic.hpp>
#include <boost/filesystem.hpp>

#include <vector>
#include <string>
#include <memory>
#include <queue>

#include <opencv2\opencv.hpp>
#include <opencv2\ml.hpp>

#include <conio.h>

#include <thread>
#include <mutex>
#include <functional>

#include "pid.h"

using boost::asio::ip::tcp;

using namespace std;

struct VehicleInput {
	VehicleInput() {
		throttle = 0.0;
		brake = 0.0;
		steering = 0.0;
	}
	VehicleInput(const string &str) {
		fromString(str);
	}

	float throttle; //between 0 and 1
	float brake;	 //between 0 and 1
	float steering; //between -1 and +1

	string toString() {
		//converts this struct to a string
		//example: "0.3;0.0;-0.89"
		std::stringstream strstream;
		//std::string retval;
		//retval = (boost::format("%1.2f;%1.2f;%1.2f")
		//	% throttle
		//	% brake
		//	% steering).str();

		//return retval;
		std::stringstream steeringStream;
		std::string steeringString;
		steeringStream << steering;
		steeringString = steeringStream.str();
		if (steering == 0) {
			steeringString = "0.0";
		}
		if (steering == -1) {
			steeringString = "-1.0";
		}
		if (steering == 1) {
			steeringString = "1.0";
		}

		
		strstream
			<< throttle << ";"
			<< brake << ";"
			<< steeringString;

		return strstream.str();
	}
	void fromString(const std::string &input) {
		vector<string> fields;
		boost::algorithm::split(fields, input, boost::is_any_of(";"));
		if (fields.size() != 3) {
			return;
		}
		throttle = boost::lexical_cast<float, string>(fields[0]);
		brake = boost::lexical_cast<float, string>(fields[1]);
		steering = boost::lexical_cast<float, string>(fields[2]);

		return;
	}
};

//decodes 4 byte little endian encoded byte array to unsigned 32 bit integer
uint32_t decode(unsigned char* bytes) {

	uint32_t retval = bytes[0] + (bytes[1] << 8) + (bytes[2] << 16) + (bytes[3] << 24);
	return retval;
}

typedef std::function<void(cv::Mat &)> ImageCallbackFunction;

class Interface {
public:
	Interface(ImageCallbackFunction callback, string ip = "127.0.0.1", int steeringPort = 9994, int imagePort = 9993) {
		imageCallback = callback;
		steeringSocket.reset(new tcp::socket(io_service, tcp::endpoint(tcp::v4(), steeringPort)));
		steeringSocket->non_blocking(true);

		imageSocket.reset(new tcp::socket(io_service, tcp::endpoint(tcp::v4(), imagePort)));
		imageSocket->non_blocking(false);


		boost::asio::socket_base::receive_buffer_size bufSize(20000000);
		imageSocket->set_option(bufSize); //todo check if this brings some advantages

		connected = false;
	}
	bool Connect(string remoteIP, int remoteSteeringPort, int remoteImagePort) {
		//remote_endpoint = udp::endpoint(boost::asio::ip::address().from_string(remote_ip), remote_port);
		auto ip = boost::asio::ip::address().from_string(remoteIP);
		remoteSteeringEndpoint = tcp::endpoint(ip, remoteSteeringPort);
		remoteImageEndpoint = tcp::endpoint(ip, remoteImagePort);

		boost::system::error_code error = boost::asio::error::host_not_found;

		if (steeringSocket.get() == 0) {
			return false;
		}

		steeringSocket->connect(remoteSteeringEndpoint, error);
		if (error) {
			return false;
		}

		imageSocket->connect(remoteImageEndpoint, error);
		if (error) {
			return false;
		}

		connected = true;
		return true;
	}
	bool Send(VehicleInput input) {
		std::string message = input.toString();
		if (steeringSocket.get() == 0) {
			return false;
		}
		if (!connected) {
			return false;
		}
		size_t bytes_sent = steeringSocket->send(boost::asio::buffer(message));
		if (bytes_sent != message.length()) {
			return false;
		}
	}
	bool Receive() {
		if (imageSocket.get() == 0) {
			return false;
		}
		if (!connected) {
			return false;
		}

		size_t received = 0;
		uint32_t width = 0;
		uint32_t height = 0;
		uint32_t awaited = 0;

		{
			static boost::array<unsigned char, 1024> recv_buf;

			try {
				received = imageSocket->receive(boost::asio::buffer(recv_buf));
			}
			catch (boost::system::system_error e) {
				imageSocket->close();
				imageSocket = 0;
				return false;
			}


			if (received == 12) {
				awaited = decode(recv_buf.c_array());
				width = decode(recv_buf.c_array() + 4);
				height = decode(recv_buf.c_array() + 8);
				received = 0;
			}
		}
		try {
			imageSocket->send(boost::asio::buffer("ack"));
		}
		catch (boost::system::system_error e) {
			std::cout << e.what() << std::endl;

		}
		{
			//static boost::array<unsigned char, 1024 * 1024 * 100> recv_buf;
			boost::asio::streambuf buffer;
			boost::asio::streambuf::mutable_buffers_type bufs = buffer.prepare(awaited);

			received = 0;
			while (received < awaited) {

				try {
					//received += imageSocket->receive(boost::asio::buffer(recv_buf));
					received += imageSocket->read_some(bufs);
				}
				catch (boost::system::system_error e) {
					imageSocket->close();
					imageSocket = 0;
					return false;
				}

			}

			buffer.commit(received);

			try {
				imageSocket->send(boost::asio::buffer("ack"));
			}
			catch (boost::system::system_error e) {
				std::cout << e.what() << std::endl;
			}
			if (received > 0) {

				if (width > 0 && height > 0) {

					cv::Mat img(height, width, CV_8UC4, (void*)boost::asio::buffer_cast<const void*>(buffer.data()));
					cv::cvtColor(img, img, CV_BGRA2RGBA);
					//cv::imshow("img", img);
					//cv::waitKey(1);
					{//scope for mutex lock
						std::lock_guard<std::mutex> lock(callbackMutex); //create a lock to ensure callback function will only be executed once at a time
						imageCallback(img.clone());
					}
					//img.clone()
					return true;
				}
				else {
					return false;
				}

			}

		}

	}


private:
	boost::asio::io_service io_service;
	std::unique_ptr<tcp::socket> steeringSocket;
	std::unique_ptr<tcp::socket> imageSocket;
	tcp::endpoint remoteSteeringEndpoint;
	tcp::endpoint remoteImageEndpoint;
	ImageCallbackFunction imageCallback;
	std::mutex callbackMutex;
	bool connected;
};

boost::atomic<double> steeringBarValue;
struct SampleImage {
	std::string filePath;
	cv::Mat image;

	SampleImage(const cv::Mat &image_, const std::string &filePath_) {
		filePath = filePath_;
		image = image_.clone();
	}
};
class ImageProcessing {
private:
	cv::Ptr<cv::ml::ANN_MLP> ann;
	cv::Mat inputs;
	cv::Mat samples;
	int i;
	int j;
	int k;
	std::vector<SampleImage> sampleVector;
	std::string outDir;
	cv::Mat transformationMatrix;
	int sollwert;
	PID pidController;

public:
	ImageProcessing() {
		int attributes_per_sample = 64 * 32; //size of one image
		int hidden_layer_size = 15; //size of the hidden layer of neurons
		int number_of_classes = 1; //size of output vector for steering direction
		std::vector<int> layerSizes = { attributes_per_sample, 100, hidden_layer_size, number_of_classes };
		i = 0;
		j = 0;
		k = 0;
		ann = cv::ml::ANN_MLP::create();
		ann->setLayerSizes(layerSizes);
		ann->setActivationFunction(cv::ml::ANN_MLP::SIGMOID_SYM);
		cv::TermCriteria termCriteria;
		termCriteria.maxCount = 100;
		termCriteria.epsilon = 0.00001f;
		termCriteria.type = cv::TermCriteria::MAX_ITER;
		ann->setTermCriteria(termCriteria);
		ann->setTrainMethod(cv::ml::ANN_MLP::BACKPROP);
		ann->setBackpropMomentumScale(0.0001);
		ann->setBackpropWeightScale(0.02);
		sampleVector.clear();
		sampleVector.reserve(100);

		outDir = "C:\\ptemp\\trainingData11\\";
		namespace fs = boost::filesystem;

		if (!fs::exists(fs::path(outDir))) {
			fs::create_directory(fs::path(outDir));
		}

		cv::FileStorage fileStorage("C:\\ptemp\\transform.yml", cv::FileStorage::READ);
		fileStorage["transformationMatrix"] >> transformationMatrix;
		sollwert = -1;
		pidController = PID(0.05, 1.0, -1.0, 0.2, 0.0, 0.0);
	}

	double processImage(const cv::Mat& image) {

		cv::Mat orthoImage;
		double output = steeringByController(image, orthoImage);
		accumulateAndSaveImage(image, output);
		cv::Mat trainImage;
		double outputNetwork = 0.0;
		//outputNetwork = trainNetwork(image, output, trainImage);
		//cv::imshow("training result", trainImage);
		cv::imshow("orthoImage", orthoImage);
		cv::waitKey(1);

		if (steeringBarValue > 0.5) {
			std::cout << "drving with network power" << std::endl;
			return outputNetwork;
		}

		if (steeringBarValue < -0.5) {
			ann->save(outDir + "\\trainedNetwork.ann");
			std::cout << "saved network to: " + outDir + "\\trainedNetwork.ann" << std::endl;
		}


		return output;
	}

	private:
		void accumulateAndSaveImage(cv::Mat image, double steering) {
			
		std::string filename;
		if (steering >= 0) {
			filename = (boost::format("%05d_+%1.9f.png") % i % steering).str();
		}
		else {
			filename = (boost::format("%05d_-%1.9f.png") % i % (-steering)).str();
		}

		std::string filePath = outDir + filename;

		sampleVector.push_back(SampleImage(image, filePath));

		if (sampleVector.size() >= 50) {
			for (size_t d = 0; d < sampleVector.size(); d++) {
				cv::imwrite(sampleVector[d].filePath, sampleVector[d].image);
			}
			sampleVector.clear();
			sampleVector.reserve(50);
		}

		i++;
	}
		double steeringByController(cv::Mat image, cv::Mat &outputImage) {
			cv::Mat orthoImage;
			cv::Mat grayImage;

			try {
				cv::cvtColor(image, grayImage, CV_BGRA2GRAY);
				cv::warpPerspective(grayImage, orthoImage, transformationMatrix, cv::Size(1000, 1000), CV_INTER_NN);
				//cv::imshow("original image", grayImage);
			}
			catch (cv::Exception e) {
				std::string what = e.what();
				std::cout << what;
				return 0.0;
			}
			cv::Mat binOrtho;
			int windowSize = 501;
			double C = -100;
			cv::adaptiveThreshold(orthoImage, binOrtho, 255.0, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, windowSize, C);
			cv::Mat1b LOI1 = binOrtho(cv::Rect(cv::Point(0, 710), cv::Point(999, 710 + 1)));
			cv::Mat1b LOI2 = binOrtho(cv::Rect(cv::Point(0, 677), cv::Point(999, 677 + 1)));
			cv::Mat1b LOI3 = binOrtho(cv::Rect(cv::Point(0, 644), cv::Point(999, 644 + 1)));
			cv::Mat1b LOI4 = binOrtho(cv::Rect(cv::Point(0, 611), cv::Point(999, 611 + 1)));
			cv::Mat1b LOI5 = binOrtho(cv::Rect(cv::Point(0, 580), cv::Point(999, 580 + 1)));
			cv::Mat1b LOI = LOI1 | LOI2 | LOI3 | LOI4 | LOI5;
			//cv::imshow("LOI", LOI);
			cv::Mat colorImage;
			cv::cvtColor(binOrtho, colorImage, CV_GRAY2BGR);
			cv::line(colorImage, cv::Point(0, 710), cv::Point(999, 710), cv::Scalar(0, 0, 255));
			cv::line(colorImage, cv::Point(0, 677), cv::Point(999, 677), cv::Scalar(0, 0, 255));
			cv::line(colorImage, cv::Point(0, 644), cv::Point(999, 644), cv::Scalar(0, 0, 255));
			cv::line(colorImage, cv::Point(0, 611), cv::Point(999, 611), cv::Scalar(0, 0, 255));
			cv::line(colorImage, cv::Point(0, 580), cv::Point(999, 580), cv::Scalar(0, 0, 255));

			int leftEdge = -1;
			int rightEdge = LOI1.cols;

			for (int x = LOI1.cols - 1; x >= 0; x--) {
				if (LOI1(cv::Point(x, 0)) == 255) {
					rightEdge = x;
					break;
				}
			}

			for (int x = rightEdge - 150; x >= 0; x--) {
				if (LOI(cv::Point(x, 0)) == 255) {
					leftEdge = x;
					break;
				}
			}


			int middle = (int)std::round((rightEdge - leftEdge) / 2.0) + leftEdge;
			
			cv::line(colorImage, cv::Point(sollwert, 580), cv::Point(sollwert, 710), cv::Scalar(255, 204, 0));
			cv::line(colorImage, cv::Point(middle, 580), cv::Point(middle, 710), cv::Scalar(0, 255, 0));
			double output = 0.0;
			if (sollwert == -1) {
				sollwert = middle;
			} else {
				double processValue = 0.05*(sollwert - middle);
				output = pidController.calculate(0.0, processValue);
				//std::cout << output << std::endl;
			}
			outputImage = colorImage;
			return output;
		}
		double trainNetwork(cv::Mat image, double steering, cv::Mat &outputImage) {
			static int trainBatchSize = 10;
			cv::Mat imageFloat;
			cv::Mat currentImage;
			cv::Mat grayImage;
			cv::cvtColor(image, grayImage, CV_RGBA2GRAY);
			cv::resize(grayImage, currentImage, cv::Size(64, 32));
			cv::Mat equalizedImage;
			cv::equalizeHist(currentImage, equalizedImage);
			equalizedImage.convertTo(imageFloat, CV_32FC1, (1 / 128.0), -1.0);
			cv::Mat row = imageFloat.reshape(1, 1);
			cv::Mat training_classification = cv::Mat(1, 1, CV_32FC1, cv::Scalar(steering));

			if (j == 0)
			{
				inputs = cv::Mat(trainBatchSize, row.cols, CV_32FC1);
				samples = cv::Mat(trainBatchSize, 1, CV_32FC1);
			}

			row.copyTo(inputs(cv::Rect(0, j, row.cols, row.rows)));
			training_classification.copyTo(samples(cv::Rect(0, j, training_classification.cols, training_classification.rows)));

			if (j >= trainBatchSize -1) {
				
				if (steeringBarValue <= 0.5) {
					cv::Ptr<cv::ml::TrainData> trainData = cv::ml::TrainData::create(inputs, cv::ml::SampleTypes::ROW_SAMPLE, samples);
					try {
						if (ann->isTrained()) {
							ann->train(trainData, cv::ml::ANN_MLP::TrainFlags::UPDATE_WEIGHTS);
						}
						else {
							ann->train(trainData);
						}
					} catch (cv::Exception e) {
						std::string what = e.what();
						std::cout << what << std::endl;
					}
				
				} else {
					std::cout << "skipped training" << std::endl;
				}
				j = 0;
			}
			else
			{
				j++;
			}

			cv::Mat output;
			double predicted = 0.0;

			if (ann->isTrained()) {

				try
				{
					//ann->predict(row, output);
					//predicted = output.at<float>(0, 0);
				}
				catch (cv::Exception e)
				{
					std::string w = e.what();
					std::cout << e.what();
				}
			} else {
				std::cout << "ANN is not trained" << std::endl;
			}

			outputImage = cv::Mat(51, 200, CV_8UC3, cv::Scalar(0, 0, 0));
			int colPredicted = (int)std::round(100.0*(predicted + 1.0));
			int colSteering = (int)std::round(100.0*(steering + 1.0));

			cv::line(outputImage, cv::Point(0, 25), cv::Point(200, 25), cv::Scalar(255, 255, 255));
			cv::line(outputImage, cv::Point(colSteering, 0), cv::Point(colSteering, 24), cv::Scalar(255, 204, 0));
			cv::line(outputImage, cv::Point(colPredicted, 26), cv::Point(colPredicted, 50), cv::Scalar(0, 255, 0));
			return predicted;


		}
};





int main() {

	//cv::Mat transformationMatrix;
	//cv::FileStorage fileStorage("C:\\ptemp\\transform.yml", cv::FileStorage::READ);
	//fileStorage["transformationMatrix"] >> transformationMatrix;
	//
	//cv::Mat image = cv::imread("C:\\ptemp\\trainingData11\\00000_+0.000000000.png", CV_LOAD_IMAGE_GRAYSCALE);
	////cv::Mat image = cv::imread("C:\\ptemp\\trainingData4\\00460_+0.070000000.png", CV_LOAD_IMAGE_GRAYSCALE);

	//cv::Mat orthoImage;
	//cv::warpPerspective(image, orthoImage, transformationMatrix, cv::Size(1000, 1000), CV_INTER_LANCZOS4);
	//cv::imshow("orthoImage", orthoImage);
	//cv::Mat binOrtho;
	//int windowSize = 501;
	//double C = -100;
	//cv::adaptiveThreshold(orthoImage, binOrtho, 255.0, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, windowSize, C);
	//cv::Mat LOI1 = binOrtho(cv::Rect(cv::Point(363, 686), cv::Point(737, 686+1)));
	//cv::Mat LOI2 = binOrtho(cv::Rect(cv::Point(363, 600), cv::Point(737, 600+1)));
	//cv::Mat1b LOI = LOI1 | LOI2;
	//cv::Mat colorImage;
	//cv::cvtColor(binOrtho, colorImage, CV_GRAY2BGR);
	//cv::line(colorImage, cv::Point(363, 686), cv::Point(737, 686), cv::Scalar(0, 0, 255));
	//cv::line(colorImage, cv::Point(363, 600), cv::Point(737, 600), cv::Scalar(0, 0, 255));
	//
	//int leftEdge;
	//int rightEdge;

	//for (int x = 0; x < LOI.cols; x++) {
	//	if (LOI(cv::Point(x, 0)) == 255) {
	//		leftEdge = x;
	//		break;
	//	}
	//}
	//for (int x = LOI.cols - 1; x >= 0; x--) {
	//	if (LOI(cv::Point(x,0)) == 255) {
	//		rightEdge = x;
	//		break;
	//	}
	//}
	//int middle = (int)std::round((rightEdge - leftEdge) / 2.0) + leftEdge;
	//cv::line(colorImage, cv::Point(middle + 363, 600), cv::Point(middle+363, 686), cv::Scalar(0, 255, 255));


	//cv::waitKey(1);


	//cv::Mat image = cv::imread("C:\\Temp\\example.png");
	//return 0;

	int remoteImagePort = 9771;
	int remoteSteeringPort = 9772;

	std::cout << "remote image port: ";
	std::cin >> remoteImagePort;
	std::cout << "remote steering port: ";
	std::cin >> remoteSteeringPort;

	
	int valuePointer = 100;

	cv::namedWindow("steering wheel");
	cv::createTrackbar("steering: ", "steering wheel", &valuePointer, 200, [](int pos, void* userdata) {
		steeringBarValue = (pos - 100) / 100.0;
	});

	ImageProcessing virtualDriver;

	VehicleInput vehicleInput;

	ImageCallbackFunction imageCallback = [&](cv::Mat& image) {
		double angle = virtualDriver.processImage(image);
		vehicleInput.steering = angle;
		//vehicleInput.steering = steeringBarValue;
	};


	Interface blenderInterface(imageCallback);


	std::string remoteIP = "127.0.0.1";

	cout << "Trying to connect to " << remoteIP << ":" << remoteSteeringPort << " and " << remoteImagePort << "... " << endl;
	while (true) {

		if (!blenderInterface.Connect(remoteIP, remoteSteeringPort, remoteImagePort)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		else {
			break;
		}
	}

	cout << "Connected!" << endl;
	cout << "Press W, A, S, D or SPACE to send input to simulation:" << endl;
	std::thread receiverThread = std::thread([&] {
		cv::Mat img;
		while (true) {
			std::string message;
			bool ok;
			ok = blenderInterface.Receive();
			if (ok) {
				//cout << "received something" << endl;
			}
		}
	});

	while (true) {
		int key = cv::waitKey(2);
		vehicleInput.throttle = 1;
		switch (key) {
		case 'w':
			vehicleInput.throttle += 0.3;
			vehicleInput.brake = 0.0;
			break;
		case 's':
			vehicleInput.brake += 0.5;
			vehicleInput.throttle = 0.0;
			break;
		case 'a':
			vehicleInput.steering -= 0.3333;
			break;
		case 'd':
			vehicleInput.steering += 0.3333;
			break;
		case ' ':
			vehicleInput.throttle = 0.0;
			vehicleInput.brake = 0.0;
		case 'p':
			std::cout << "Pause... Press any key to continue" << std::endl;
			std::getchar();
		default:
			break;
		}

		blenderInterface.Send(vehicleInput.toString());
		std::string data;

	}

}

