//author: sebastian beyer 

#include <iostream>

#include <boost/array.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <vector>
#include <string>
#include <memory>

#include <opencv2\opencv.hpp>
#include <opencv2\ml.hpp>

#include <conio.h>

#include <thread>
#include <mutex>
#include <functional>

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
	VehicleInput(const VehicleInput &other) {
		throttle = other.throttle;
		brake = other.brake;
		steering = other.steering;
	}

	void setThrottle(float value) {
		boost::lock_guard<boost::mutex> lock(m_mutex);
		throttle = value;
	}
	void setBrake(float value) {
		boost::lock_guard<boost::mutex> lock(m_mutex);
		brake = value;
	}
	void setSteering(float value) {
		boost::lock_guard<boost::mutex> lock(m_mutex);
		steering = value;
	}

	void changeThrottle(float value) {
		boost::lock_guard<boost::mutex> lock(m_mutex);
		throttle += value;
	}
	void changeBrake(float value) {
		boost::lock_guard<boost::mutex> lock(m_mutex);
		brake += value;
	}
	void changeSteering(float value) {
		boost::lock_guard<boost::mutex> lock(m_mutex);
		steering += value;
	}


	float getThrottle() {
		boost::lock_guard<boost::mutex> lock(m_mutex);
		return throttle;
	}
	float setBrake() {
		boost::lock_guard<boost::mutex> lock(m_mutex);
		return brake;
	}
	float setSteering() {
		boost::lock_guard<boost::mutex> lock(m_mutex);
		return steering;
	}

	const std::string toString() {
		//converts this struct to a string
		//example: "0.3;0.0;-0.89"

		boost::lock_guard<boost::mutex> lock(m_mutex);

		std::string retval;
		retval = (boost::format("%1.2f;%1.2f;%1.2f;")
			% throttle
			% brake
			% steering).str();

		//std::stringstream strstream;
		//strstream
		//	<< throttle << ";"
		//	<< brake << ";"
		//	<< steering << ";";

		return retval;
	}
	void fromString(const std::string &input) {
		boost::lock_guard<boost::mutex> lock(m_mutex);

		vector<string> fields;
		boost::algorithm::split(fields, input, boost::is_any_of(";"));
		if (fields.size() != 3) {
			return;
		}
		throttle = boost::lexical_cast<float,string>(fields[0]);
		brake = boost::lexical_cast<float, string>(fields[1]);
		steering = boost::lexical_cast<float, string>(fields[2]);

		return;
	}

private:
	float throttle; //between 0 and 1
	float brake;	 //between 0 and 1
	float steering; //between -1 and +1
	boost::mutex m_mutex;
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
		const std::string message = input.toString();
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
			} catch (boost::system::system_error e) {
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
		} catch (boost::system::system_error e) {
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
				} catch (boost::system::system_error e) {
					imageSocket->close();
					imageSocket = 0;
					return false;
				}

			}

			buffer.commit(received);

			try {
				imageSocket->send(boost::asio::buffer("ack"));
			} catch(boost::system::system_error e) {
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


class ImageProcessing {
private:
	cv::Ptr<cv::ml::ANN_MLP> ann;
	cv::Mat retinaWeights;
public:
	ImageProcessing() {
		ann = cv::Algorithm::load<cv::ml::ANN_MLP>("C:\\Temp\\anotherNetwork.ann");
		bool trained = ann->isTrained();
		cv::Mat1d inputLayerWeights = ann->getWeights(0);
		cv::Mat1d halfWeights = cv::Mat1d(1, inputLayerWeights.cols / 2);
		for (int c = 0; c < inputLayerWeights.cols; c += 2) {
			int n = c / 2;
			halfWeights(cv::Point(n, 0)) = inputLayerWeights(cv::Point(c, 0));
		}
		halfWeights = halfWeights.reshape(0, 32);
		double min, max;
		cv::minMaxLoc(halfWeights, &min, &max);
		halfWeights.convertTo(retinaWeights, CV_8UC1, 255.0 / (max - min), -255.0*min / (max - min));
	}

	double processImage(const cv::Mat& image) {
		cv::Mat imageFloat;
		cv::Mat currentImage;
		cv::imshow("input image", image);
		
		cv::Mat grayImage;
		cv::cvtColor(image, grayImage, CV_RGBA2GRAY);
		currentImage = grayImage;
		cv::Mat bin;
		int windowSize = 501;
		double C = -100;
		cv::adaptiveThreshold(currentImage, bin, 255.0, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, windowSize, C);
		cv::Mat resizedBin;
		cv::resize(bin, resizedBin, cv::Size(64, 32));
		cv::Mat activationImage = retinaWeights.mul(resizedBin, 1 / 10.0);
		cv::Mat colorActivation;
		cv::applyColorMap(activationImage, colorActivation, cv::COLORMAP_JET);
		cv::resize(colorActivation, colorActivation, cv::Size(512, 256), 0.0, 0.0, CV_INTER_NN);
		
		cv::imshow("retina activation", colorActivation);
		resizedBin.convertTo(imageFloat, CV_32FC1, (1 / 128.0), -1.0);
		cv::Mat row = imageFloat.reshape(1, 1);
		cv::Mat output;
		try
		{
			ann->predict(row, output);
			
		}
		catch (cv::Exception e)
		{
			std::string w = e.what();
			std::cout << e.what();
		}
		double predicted = output.at<float>(0, 0);
		std::cout << "\rprediction:" << predicted << "       ";

		cv::Mat outputImage = cv::Mat(50, 200, CV_8UC3, cv::Scalar(0, 0, 0));
		int colPredicted = (int)std::round(100.0*(predicted + 1.0));

		cv::line(outputImage, cv::Point(colPredicted, 0), cv::Point(colPredicted, 50), cv::Scalar(0, 255, 0), 3);
		cv::imshow("output", outputImage);
		cv::waitKey(1);
		return predicted;
	}
};

int main() {
	//cv::Mat image = cv::imread("C:\\ptemp\\trainingData9\\00000_+0.000000000.png", CV_LOAD_IMAGE_GRAYSCALE);
	//int windowSize = 501;
	//double C = -100;
	//cv::adaptiveThreshold(image, image, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, windowSize, C);
	//cv::resize(image, image, cv::Size(64, 32));
	//
	//cv::Ptr<cv::ml::ANN_MLP> ann;
	//ann = cv::Algorithm::load<cv::ml::ANN_MLP>("C:\\Temp\\anotherNetwork.ann");
	//int cn = 0;
	//int rows = 32;
	//cv::Mat1d weights = ann->getWeights(0);// .reshape(cn, rows);
	//cv::Mat layerSizes = ann->getLayerSizes();
	//cv::Mat1d halfWeights = cv::Mat1d(1, weights.cols / 2);
	//cv::Mat1d otherHalf = cv::Mat1d(1, weights.cols / 2);
	//for (int c = 0; c < weights.cols; c+=2) {
	//	int n = c / 2;
	//	halfWeights(cv::Point(n, 0)) = weights(cv::Point(c, 0));
	//	otherHalf(cv::Point(n, 0)) = weights(cv::Point(c + 1, 0));
	//}
	//
	//cv::Mat w = halfWeights.reshape(0, 32);
	//double min, max;
	//cv::minMaxLoc(w, &min, &max);
	//w.convertTo(w, CV_8UC1, 255.0 / (max - min), -255.0*min / (max - min));
	//cv::Mat activation = w.mul(image, 1/255.0);



	int remoteImagePort = 9771;
	int remoteSteeringPort = 9772;

	std::cout << "remote image port: ";
	std::cin >> remoteImagePort;
	std::cout << "remote steering port: ";
	std::cin >> remoteSteeringPort;

	ImageProcessing virtualDriver;

	VehicleInput vehicleInput;

	ImageCallbackFunction imageCallback = [&](cv::Mat& image) {
		double angle = virtualDriver.processImage(image);
		vehicleInput.setSteering(angle);
	};


	Interface blenderInterface(imageCallback);
	

	std::string remoteIP = "127.0.0.1";

	cout << "Trying to connect to " << remoteIP << ":" << remoteSteeringPort << " and " << remoteImagePort <<"... "<< endl;
	while (true) {

		if (!blenderInterface.Connect(remoteIP, remoteSteeringPort, remoteImagePort)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		} else {
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
		vehicleInput.setThrottle(0.3);
		switch (key) {
		case 'w': 
			vehicleInput.changeThrottle(+0.3);
			vehicleInput.setBrake(0.0);
			break;
		case 's': 
			vehicleInput.changeBrake(+0.5);
			vehicleInput.setThrottle(0.0);
			break;
		case 'a':
			vehicleInput.changeSteering(-0.3333);
			break;
		case 'd':
			vehicleInput.changeSteering(0.3333);
			break;
		case ' ':
			vehicleInput.setThrottle(0.0);
			vehicleInput.setBrake(0.0);
		default:
			break;
		}

		blenderInterface.Send(vehicleInput);
		std::string data;

	}

}

