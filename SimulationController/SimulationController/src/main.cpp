//author: sebastian beyer 

#include <iostream>

#include <boost/array.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <vector>
#include <string>
#include <memory>

#include <opencv2\opencv.hpp>

#include <conio.h>

#include <thread>
#include <mutex>


//using boost::asio::ip::udp;
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
		strstream
			<< throttle << ";"
			<< brake << ";"
			<< steering;

		return strstream.str();
	}
	void fromString(const std::string &input) {
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
};

struct SimulationOutput {
	SimulationOutput() {
	}

	cv::Mat image;

};

uint32_t decode(unsigned char* bytes) { //decodes 4 byte little endian encoded byte array to unsigned 32 bit integer

	uint32_t retval = bytes[0] + (bytes[1] << 8) + (bytes[2] << 16) + (bytes[3] << 24);
	return retval;
}


class Interface {
public:
	Interface(string ip = "127.0.0.1", int steeringPort = 9994, int imagePort = 9993) {

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
	bool Send(const std::string &message) {
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
	bool Receive(cv::Mat &image) {
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
					cv::imshow("img", img);
					cv::waitKey(1);
					image = img.clone();
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
	boost::array<char, 1024> recv_buf;
	bool connected;
};


int main() {

	int remoteImagePort = 9771;
	int remoteSteeringPort = 9772;

	std::cout << "remote image port: ";
	std::cin >> remoteImagePort;
	std::cout << "remote steering port: ";
	std::cin >> remoteSteeringPort;

	Interface blenderInterface;
	VehicleInput vehicleInput;

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
			ok = blenderInterface.Receive(img);
			if (ok) {
				//cout << "received something" << endl;
			}
		}
	});

	while (true) {
		char key = _getche();
		
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
		default:
			break;
		}

		blenderInterface.Send(vehicleInput.toString());
		std::string data;

	}

}

