/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_raw_ros2.cpp - libcamera raw video record app with ROS 2 support.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include "core/rpicam_encoder.hpp"
#include "encoder/null_encoder.hpp"
#include "output/output.hpp"

using namespace std::placeholders;

// Define LibcameraRaw as a subclass of RPiCamEncoder
class LibcameraRaw : public RPiCamEncoder
{
public:
	LibcameraRaw() : RPiCamEncoder() {}

protected:
	// Override createEncoder to use a null encoder
	void createEncoder() override { 
		encoder_ = std::make_unique<NullEncoder>(GetOptions()); 
	}
};

class LibcameraRawNode : public rclcpp::Node
{
public:
	LibcameraRawNode(LibcameraRaw &app) : Node("rpicam_raw_node"), app_(app)
	{
		image_pub_ = image_transport::create_publisher(this, "camera/image_raw");
		VideoOptions *options = app.GetOptions();

		app_.OpenCamera();
		app_.ConfigureVideo(LibcameraRaw::FLAG_VIDEO_RAW);
		app_.StartCamera();

		double framerate = options->framerate.value_or(30.0);
		int timer_interval_ms = static_cast<int>(1000.0 / framerate);

		event_loop(); // Start the event loop without a timer
		// timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval_ms), 
		// 								std::bind(&LibcameraRawNode::event_loop, this));
	}

private:
	void event_loop()
	{
		LibcameraRaw::Msg msg = app_.Wait();
		if (msg.type == LibcameraRaw::MsgType::Timeout)
		{
			RCLCPP_ERROR(this->get_logger(), "ERROR: Device timeout detected, attempting a restart!");
			app_.StopCamera();
			app_.StartCamera();
			return;
		}
		if (msg.type != LibcameraRaw::MsgType::RequestComplete)
		{
			RCLCPP_ERROR(this->get_logger(), "Unrecognized message type!");
			return;
		}

		CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
		libcamera::FrameBuffer *buffer = completed_request->buffers[app_.RawStream()];
		if (!buffer)
		{
			RCLCPP_ERROR(this->get_logger(), "ERROR: Received null buffer!");
			return;
		}

		// Map the buffer
		void *memory = nullptr;
		int fd = buffer->planes()[0].fd.get();
		if (fd >= 0)
		{
			memory = mmap(nullptr, buffer->planes()[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
			if (memory == MAP_FAILED)
			{
				RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to mmap buffer memory!");
				return;
			}
		}
		if (!memory)
		{
			RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to map buffer memory!");
			return;
		}

		// Get image dimensions
		libcamera::StreamConfiguration const &cfg = app_.RawStream()->configuration();
		int width = cfg.size.width;
		int height = cfg.size.height;

		auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
		img_msg->header.stamp = this->get_clock()->now();
		img_msg->height = height;
		img_msg->width = width;
		img_msg->encoding = "mono8";  // Changed to mono8 format
		img_msg->is_bigendian = 0;
		img_msg->step = width;  // 1 byte per pixel for 8-bit raw
		img_msg->data.assign(static_cast<uint8_t *>(memory), 
							static_cast<uint8_t *>(memory) + (width * height));

		image_pub_.publish(std::move(img_msg));

		// Unmap the buffer
		munmap(memory, buffer->planes()[0].length);
	}

	LibcameraRaw &app_;
	image_transport::Publisher image_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	try
	{
		LibcameraRaw app;  // Now correctly inherits from RPiCamEncoder
		VideoOptions *options = app.GetOptions();
		
		if (options->Parse(argc, argv))
		{
			// Disable any codec (h.264/libav) based operations.
			options->codec = "yuv420";
			options->denoise = "cdn_off";
			options->nopreview = true;

			if (options->verbose >= 2)
				options->Print();

			auto node = std::make_shared<LibcameraRawNode>(app);
			rclcpp::spin(node);
		}
	}
	catch (std::exception const &e)
	{
		std::cerr << "ERROR: *** " << e.what() << " ***" << std::endl;
		return -1;
	}

	rclcpp::shutdown();
	return 0;
}
