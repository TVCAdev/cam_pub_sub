#include <memory>
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <opencv2/opencv.hpp>
#include "cam_msg/msg/cam_jpeg.hpp"
using std::placeholders::_1;
using namespace cv;
namespace fs = std::filesystem;

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

class CamSubscriberSave : public rclcpp::Node
{
public:
    CamSubscriberSave(const char *node_name, const char *topic_name, const char *save_top_dir,
                      const char *check_area, const char *check_width, const char *check_height)
        : Node(node_name)
    {
        subscription_ = this->create_subscription<cam_msg::msg::CamJpeg>(
            topic_name, 10, std::bind(&CamSubscriberSave::topic_callback, this, _1));
        _save_top_dir = save_top_dir;
        _check_area = convStr2Num(check_area);
        _check_width = convStr2Num(check_width);
        _check_height = convStr2Num(check_height);
    }

private:
    void topic_callback(const cam_msg::msg::CamJpeg &msg)
    {
        RCLCPP_INFO(this->get_logger(), "recieved '%s' jpeg data.", msg.yyyymmdd_hhmmss_string.c_str());

        // Convert jpeg data to cv::Mat
        cv::Mat jpeg_cur_gray_mat;
        const unsigned char *jpeg_cur_buf = msg.unbounded_cam_data.data();
        size_t jpeg_cur_buf_size = msg.unbounded_cam_data.size();
        std::vector<uchar> jpeg_cur_buf_vec(jpeg_cur_buf, jpeg_cur_buf + jpeg_cur_buf_size - 1);
        cv::Mat jpeg_cur_gbr_mat = cv::imdecode(jpeg_cur_buf_vec, 1);
        cv::cvtColor(jpeg_cur_gbr_mat, jpeg_cur_gray_mat, COLOR_BGR2GRAY);

        // if previous jpeg data(gray) is not empty, calculate diff image
        if (!jpeg_old_gray_mat.empty())
        {
            // calculate diff image
            cv::Mat diff_image;
            cv::absdiff(jpeg_old_gray_mat, jpeg_cur_gray_mat, diff_image);

            // convert diff image to binary image
            cv::Mat bin_image;
            cv::threshold(diff_image, bin_image, 30, 255, THRESH_BINARY);

            // denoise image
            cv::Mat denoised_image;
            cv::morphologyEx(bin_image, denoised_image, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

            // outline extracting process
            std::vector<std::vector<Point>> contours;
            cv::findContours(denoised_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            // judgement motion base on contours
            bool motion = false;
            for (auto &contour : contours)
            {
                // calculate the area of the contour
                double area = contourArea(contour);

                // Calculate the bounding rectangle of the contour
                Rect rect = boundingRect(contour);

                // If the area and rectangle conditions are satisfied, it is judged as a moving object.
                if (area > _check_area && rect.width > _check_width && rect.height > _check_height)
                {
                    motion = true;
                    break;
                }
            }

            if (motion)
            {
                RCLCPP_INFO(this->get_logger(), "motion detected");

                // save to file
                std::string filename = "output_" + msg.yyyymmdd_hhmmss_string + ".jpg";

                // create file path
                std::string year = filename.substr(7, 4);
                std::string month = filename.substr(11, 2);
                std::string day = filename.substr(13, 2);
                std::string hour = filename.substr(15, 2);
                std::string savepath = _save_top_dir + "/" + year + "/" + month + "/" + day + "/" + hour + "/" + filename;

                // create directories if not exist.
                rcpputils::fs::create_directories(rcpputils::fs::path(savepath).parent_path());

                std::ofstream ofs(savepath, std::ios::out | std::ios::binary);
                ofs.write((const char *)msg.unbounded_cam_data.data(), msg.unbounded_cam_data.size());
                ofs.close();
            }
        }

        // save current jpeg data as old jpeg data.
        jpeg_old_gray_mat = jpeg_cur_gray_mat.clone();
    }
    rclcpp::Subscription<cam_msg::msg::CamJpeg>::SharedPtr subscription_;
    cv::Mat jpeg_old_gray_mat;
    std::string _save_top_dir;
    long _check_area;
    long _check_width;
    long _check_height;
};

int main(int argc, char *argv[])
{
    if (argc != 7)
    {
        std::cerr << "You shoud give three arguments." << std::endl;
        std::cerr << argv[0] << " [node_name] [topic_name] [save_top_dir] [check_area] [check_width] [check_height]" << std::endl;
        exit(EXIT_FAILURE);
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamSubscriberSave>(argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]));
    rclcpp::shutdown();
    return 0;
}