//author: sebastian beyer 

#include <iostream>

#include <boost/array.hpp>
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

uint32_t decode(char* bytes) { //decodes 4 byte little endian encoded byte array to unsigned 32 bit integer

	uint32_t retval = bytes[0] + (bytes[1] << 8) + (bytes[2] << 16) + (bytes[3] << 24);
	return retval;
}


class Interface {
public:
	Interface(string ip = "127.0.0.1", int port = 9993) {
		//socket.reset(new udp::socket(io_service, udp::endpoint(udp::v4(), 9999)));
		socket.reset(new tcp::socket(io_service, tcp::endpoint(tcp::v4(), port)));
		socket->non_blocking(false);
		boost::asio::socket_base::receive_buffer_size bufSize(20000000);
		socket->set_option(bufSize);
		connected = false;
	}
	bool Connect(string remote_ip, int remote_port) {
		//remote_endpoint = udp::endpoint(boost::asio::ip::address().from_string(remote_ip), remote_port);
		remote_endpoint = tcp::endpoint(boost::asio::ip::address().from_string(remote_ip), remote_port);
		boost::system::error_code error = boost::asio::error::host_not_found;

		if (socket.get() == 0) {
			return false;
		}

		socket->connect(remote_endpoint, error);
		if (error) {
			return false;
		}
		connected = true;
		return true;
	}
	bool Send(const std::string &message) {
		if (socket.get() == 0) {
			return false;
		}
		if (!connected) {
			return false;
		}
		std::lock_guard<std::mutex> lock(mutex);
		size_t bytes_sent = socket->send(boost::asio::buffer(message));
		if (bytes_sent != message.length()) {
			return false;
		}
	}
	bool Receive() {
		if (socket.get() == 0) {
			return false;
		}
		if (!connected) {
			return false;
		}

		std::lock_guard<std::mutex> lock(mutex);
		size_t received = 0;
		uint32_t width;
		uint32_t height;
		uint32_t awaited = 0;

		{
			static boost::array<char, 1024> recv_buf;

			try {
				received = socket->receive(boost::asio::buffer(recv_buf));
			}
			catch (boost::system::system_error e) {
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
			socket->send(boost::asio::buffer("ack"));
		}
		catch (boost::system::system_error e) {
			std::cout << e.what() << std::endl;

		}
		{
			static boost::array<char, 1024 * 1024 * 100> recv_buf;
			received = 0;
			while (received < awaited) {

				try {
					received += socket->receive(boost::asio::buffer(recv_buf));
				} catch (boost::system::system_error e) {
					return false;
				}

			}
			try {
				socket->send(boost::asio::buffer("ack"));
			} catch(boost::system::system_error e) {
				std::cout << e.what() << std::endl;
			}
			if (received > 0) {

//				cv::Mat img(height, width, CV_8UC4, (void*)recv_buf.data());
				cv::Mat img(height, width, CV_8UC3, (void*)recv_buf.data());

				//cv::cvtColor(img, img, CV_BGRA2RGBA);
				//std::memcpy((void*)img.data, recv_buf.data(), received);
				cv::imshow("img", img);
				cv::waitKey(1);
				return true;

			}

		}

	}


private:
	boost::asio::io_service io_service;
	std::unique_ptr<tcp::socket> socket;
	tcp::endpoint remote_endpoint;
	boost::array<char, 1024> recv_buf;
	bool connected;
	std::mutex mutex;
};


int main() {

	

	Interface blenderInterface;
	VehicleInput vehicleInput;

	std::string remote_ip = "127.0.0.1";
	int remote_port = 9981;
	cout << "Trying to connect to " << remote_ip << ":" << remote_port <<"... "<< endl;
	while (true) {

		if (!blenderInterface.Connect(remote_ip, remote_port)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		} else {
			break;
		}
	}

	cout << "Connected!" << endl;
	cout << "Press W, A, S, D or SPACE to send input to simulation:" << endl;

	std::thread receiverThread = std::thread([&] {
		
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

