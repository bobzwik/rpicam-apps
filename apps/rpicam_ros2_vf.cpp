/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * rpicam_ros2.cpp - libcamera "hello world" app with ROS 2.
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include "core/rpicam_app.hpp"
#include "core/options.hpp"

using namespace std::placeholders;

class RPiCamNode : public rclcpp::Node
{
public:
    RPiCamNode(RPiCamApp &app) : Node("rpicam_node"), app_(app) // Pass app as reference
    {
        image_pub_ = image_transport::create_publisher(this, "camera/image_raw");

        app_.OpenCamera();
        app_.ConfigureViewfinder();
        app_.StartCamera();
        
        event_loop();
    }

private:
    void event_loop()
    {
        while (rclcpp::ok())
		{
            RPiCamApp::Msg msg = app_.Wait();
            if (msg.type == RPiCamApp::MsgType::Timeout)
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR: Device timeout detected, attempting a restart!");
                app_.StopCamera();
                app_.StartCamera();
                return;
            }
            if (msg.type == RPiCamApp::MsgType::Quit)
            {
                rclcpp::shutdown();
                return;
            }
            else if (msg.type != RPiCamApp::MsgType::RequestComplete)
            {
                RCLCPP_ERROR(this->get_logger(), "Unrecognized message type!");
                return;
            }

            CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
            libcamera::FrameBuffer *buffer = completed_request->buffers[app_.ViewfinderStream()];
            if (!buffer)
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR: Received null buffer!");
                return;
            }

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
    }

    RPiCamApp &app_;
    image_transport::Publisher image_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        RPiCamApp app;
        Options *options = app.GetOptions();
        
        if (options->Parse(argc, argv)) // Ensure options are set up before using them
        {
            if (options->verbose >= 2)
                options->Print();

            auto node = std::make_shared<RPiCamNode>(app); // Pass the app to the node
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

