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
        Options *options = app.GetOptions();
        app_.OpenCamera();
        app_.ConfigureViewfinder();
        app_.StartCamera();
        
        double framerate = options->framerate.value_or(30.0);
        int timer_interval_ms = static_cast<int>(1000.0 / framerate);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_interval_ms), 
                                         std::bind(&RPiCamNode::event_loop, this));
    }

private:
    void event_loop()
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

        // const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];
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

        auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
        img_msg->header.stamp = this->get_clock()->now();
        img_msg->height = 400;
        img_msg->width = 640;
        img_msg->encoding = "mono8";
        img_msg->is_bigendian = 0;
        img_msg->step = 640;
        img_msg->data.assign(static_cast<uint8_t *>(memory), static_cast<uint8_t *>(memory) + (640 * 400));

        image_pub_.publish(std::move(img_msg));
    }

    RPiCamApp &app_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
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

