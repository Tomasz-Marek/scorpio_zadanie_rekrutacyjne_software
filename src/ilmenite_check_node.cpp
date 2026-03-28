#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <opencv2/opencv.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace recruitment_task
{

class IlmeniteCheckNode : public rclcpp::Node
{
public:
    explicit IlmeniteCheckNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("ilmenite_check", options)
    {
        RCLCPP_INFO(this->get_logger(), "IlmeniteCheckNode started.");

        process_samples();
    }

private:
    void process_samples()
    {
        // TODO: set path to samples folder
        const fs::path samples_folder = "assets/ilmenite_samples";
        // TODO: get image paths from folder
        std::vector<fs::path> image_paths = get_image_paths(samples_folder);
        // TODO: if no images found, print warning
        if (image_paths.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No images found in folder: %s", samples_folder.c_str());
            return;
        }
        // TODO: loop through images
        for (const auto& image_path : image_paths)
        {
            // TODO: process each image
            process_single_image(image_path);
        }

    }

    std::vector<fs::path> get_image_paths(const fs::path& folder_path)
    {
        std::vector<fs::path> image_paths;

        // TODO: check if folder exists
        if (!fs::exists(folder_path))
        {
            RCLCPP_WARN(this->get_logger(), "This path does not exist: %s", folder_path.c_str());
            return;
        }
        // TODO: check if path is a directory
        if (!fs::is_directory(folder_path))
        {
            RCLCPP_WARN(this->get_logger(), "This path is not directory: %s", folder_path.c_str());
            return;
        }
        // TODO: iterate through directory entries
        // TODO: keep only jpg / jpeg / png files
        for (const auto& entry : fs::directory_iterator(folder_path))
        {
            if (!entry.is_regular_file())
            {
                RCLCPP_WARN(this->get_logger(), "This entry is not regular file: %s", entry.c_str());
                return;
            }
            fs::path image_path = entry.path();

            std::string extension = image_path.extension().string();

            if (extension != ".jpg")
            {
                RCLCPP_WARN(this->get_logger(), "This file is not .jpg: %s", image_path.c_str());
            }

            image_paths.push_back(image_path);
        }

        return image_paths;

    }

    void process_single_image(const fs::path& image_path)
    {
        // TODO: print current file name
        // TODO: load image using cv::imread
        // TODO: check if image is empty
        // TODO: print image width and height
    }
};

}  // namespace recruitment_task

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(recruitment_task::IlmeniteCheckNode)