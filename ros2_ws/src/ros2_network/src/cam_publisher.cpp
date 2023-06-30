#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <linux/videodev2.h>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "ros2_network/msg/cam_jpeg.hpp"

using namespace std::literals;

#define MMAP_CNT 1

static int xioctl(int fd, long unsigned int request, void *arg)
{
    int r;
    do
    {
        r = ioctl(fd, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

long convStr2Num(const char *inStr)
{
    char *e;
    long ret;

    if (inStr)
    {
        errno = 0;
        ret = strtoul(inStr, &e, 10);
        if ((*e == '\0') && (errno == 0))
        {
            return ret;
        }
    }
    std::cerr << "Cannot convert " << inStr << " to number." << std::endl;
    exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
    unsigned char *buffer;
    long image_width = 0;
    long image_height = 0;
    long publish_rate = 0;
    char *publisher_name;
    char *publisher_topic;
    int ret;

    // 0. check arguments and get 
    // get publisher_name, publisher_topic, publish_rate, image_width and image_height.
    if (argc != 6)
    {
        std::cerr << "You shoud give five arguments." << std::endl;
        std::cerr << argv[0] << " [node_name] [topic_name] [publish_rate(ms) image_width image_height]" << std::endl;
        exit(EXIT_FAILURE);
    }

    // set publisher_name and publisher_topic
    publisher_name = argv[1];
    publisher_topic = argv[2];
    publish_rate = convStr2Num(argv[3]);
    image_width = convStr2Num(argv[4]);
    image_height = convStr2Num(argv[5]);

    // 1. Open Video Device.
    int fd;
    fd = open("/dev/video0", O_RDWR, 0);
    if (fd == -1)
    {
        std::cerr << "Failed to open video device." << std::endl;
        return 1;
    }

    // 2. Querying video capabilities.
    struct v4l2_capability caps;
    memset(&caps, 0, sizeof(caps));
    ret = xioctl(fd, VIDIOC_QUERYCAP, &caps);
    if (-1 == ret)
    {
        std::cerr << "Failed to query capabilities." << std::endl;
        return 1;
    }
    std::cout << "bus_info	: " << caps.bus_info << std::endl;
    std::cout << "card		: " << caps.card << std::endl;
    std::cout << "driver	: " << caps.driver << std::endl;
    std::cout << "version	: " << caps.version << std::endl;

    // 3. Format Specification.
    {
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = image_width;
        fmt.fmt.pix.height = image_height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; // for JPEG
        fmt.fmt.pix.field = V4L2_FIELD_NONE;          // Images are in progressive format, not interlaced.

        ret = xioctl(fd, VIDIOC_S_FMT, &fmt);
        if (-1 == ret)
        {
            std::cerr << "Failed to set pixel format." << std::endl;
            return 1;
        }
    }

    // 4. Request Buffer
    {
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = MMAP_CNT;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        ret = xioctl(fd, VIDIOC_REQBUFS, &req);
        if (-1 == ret)
        {
            std::cerr << "Failed to request buffer." << std::endl;
            return 1;
        }

        // check whether allocate request buffer or not.
        if (MMAP_CNT > req.count)
        {
            std::cerr << "Can not allocate sufficient buffer." << std::endl;
            return 1;
        }
    }

    // 5. Query Buffer
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = 0;
        ret = xioctl(fd, VIDIOC_QUERYBUF, &buf);
        if (-1 == ret)
        {
            std::cerr << "Failed to query buffer." << std::endl;
            return 1;
        }

        std::cout << "buf.length : " << buf.length << std::endl;
        std::cout << "buf.m.offset : " << buf.m.offset << std::endl;

        buffer = (unsigned char *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    }

    // 6. Start Streaming
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ret = xioctl(fd, VIDIOC_STREAMON, &buf.type);
        if (-1 == ret)
        {
            std::cerr << "Start Capture" << std::endl;
            return 1;
        }
    }

    struct v4l2_buffer buf;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;

    // 7. ROS2 Initialization & create node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(publisher_name);
    auto publisher = node->create_publisher<ros2_network::msg::CamJpeg>(publisher_topic, 1);
    std::cout << "Node name(" << publisher_name << ") publish data by topic(" << publisher_topic << ")" << std::endl;

    // 8. Start loop
    rclcpp::WallRate loop(publish_rate * 1ms);
    while (rclcpp::ok())
    {
        // 9. Capture Image
        {
            // Connect buffer to queue for next capture.
            ret = xioctl(fd, VIDIOC_QBUF, &buf);
            if (-1 == ret)
            {
                std::cout << "VIDIOC_QBUF" << std::endl;
            }

            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd, &fds);
            struct timeval tv = {0, 0};
            tv.tv_sec = 2;
            ret = select(fd + 1, &fds, NULL, NULL, &tv);
            if (-1 == ret)
            {
                std::cerr << "Waiting for Frame" << std::endl;
                return 1;
            }

            memset(&(buf), 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            ret = xioctl(fd, VIDIOC_DQBUF, &buf);
            if (-1 == ret && buf.bytesused <= 0)
            {
                std::cerr << "Retrieving Frame" << std::endl;
                return 1;
            }
        }

        // FOR DEBUG. create jpeg file from buffer
        // {
        //     // open jpeg file
        //     std::ofstream ofs("output.jpg", std::ios::out | std::ios::binary);

        //     // write jpeg data to file
        //     ofs.write((const char *)buffer, buf.bytesused);

        //     // close file
        //     ofs.close();
        // }

        // 10. Publich camera image
        {
            // Get current time and convert string
            struct timeval tv;
            struct tm *local;
            char yearToUsecStr[21];
            gettimeofday(&tv, NULL);
            local = localtime(&tv.tv_sec);
            sprintf(yearToUsecStr, "%04d%02d%02d%02d%02d%02d%06ld",
                    local->tm_year + 1900, local->tm_mon + 1, local->tm_mday, local->tm_hour, local->tm_min, local->tm_sec, tv.tv_usec);

            auto msg = ros2_network::msg::CamJpeg();
            msg.yyyymmdd_hhmmss_string = yearToUsecStr;
            msg.unbounded_cam_data.assign(buffer, buffer + buf.bytesused);
            publisher->publish(msg);
            std::cout << "publish Jpeg data. date: " << yearToUsecStr << std::endl;
        }
        loop.sleep();
    }

    // 11. Turn off streaming.
    ret = xioctl(fd, VIDIOC_STREAMOFF, &buf.type);
    if (-1 == ret)
    {
        std::cout << "VIDIOC_STREAMOFF" << std::endl;
    }

    return 0;
}
