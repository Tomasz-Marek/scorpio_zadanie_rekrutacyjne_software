#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
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
        // Set path to samples folder
        const fs::path samples_folder = "/home/tomek/scorpio_ws/src/scorpio_zadanie_rekrutacyjne_software/assets/ilmenite_samples";

        // Get image paths from folder
        std::vector<fs::path> image_paths = get_image_paths(samples_folder);
        if (image_paths.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No images found in folder: %s", samples_folder.c_str());
            return;
        }

        // Loop through images
        for (const auto& image_path : image_paths)
        {
            // Process each image
            process_single_image(image_path);
        }

    }

    std::vector<fs::path> get_image_paths(const fs::path& folder_path)
    {
        std::vector<fs::path> image_paths;

        // Check if folder exists
        if (!fs::exists(folder_path))
        {
            RCLCPP_WARN(this->get_logger(), "This path does not exist: %s", folder_path.c_str());
            return image_paths;
        }

        // Check if path is a directory
        if (!fs::is_directory(folder_path))
        {
            RCLCPP_WARN(this->get_logger(), "This path is not directory: %s", folder_path.c_str());
            return image_paths;
        }

        // iterate through directory entries, keep only jpg / jpeg / png files
        for (const auto& entry : fs::directory_iterator(folder_path))
        {
            const fs::path image_path = entry.path();

            
            if (!entry.is_regular_file())
            {
                continue;
            }

            std::string extension = image_path.extension().string();
            
            if (extension != ".jpg" && extension != ".jpeg" && extension != ".png")
            {
                RCLCPP_WARN(this->get_logger(), "This file extension is not supported: %s", image_path.c_str());
                continue;
            }

            image_paths.push_back(image_path);
        }

        std::sort(image_paths.begin(), image_paths.end());
        return image_paths;

    }

    void process_single_image(const fs::path& image_path)
    {
        // Check if path exist and print its name
        if (!fs::exists(image_path))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid image path: %s", image_path.c_str());
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Inspecting sample: %s", image_path.filename().c_str());
        
        // Load image using cv::imread
        cv::Mat sample_image = cv::imread(image_path.string());
        
        // Check if image is empty
        if (sample_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Error while reading the image: %s", image_path.c_str());
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "Sample Image width and height: %d x %d", sample_image.cols, sample_image.rows);

        // Convert image to greyscale
        cv::Mat gray_image;
        cv::cvtColor(sample_image, gray_image, cv::COLOR_BGR2GRAY);
        // RCLCPP_INFO(this->get_logger(), "Grayscale Image width and height: %d x %d", gray_image.cols, gray_image.rows);
        save_debug_image(gray_image, image_path, "gray");

        // Blur image
        cv::Mat blur_image;
        cv::GaussianBlur(gray_image, blur_image, cv::Size(3, 3), 2);
        save_debug_image(blur_image, image_path, "blur");

        // Threshold image
        cv::Mat thresh_image;
        cv::threshold(blur_image, thresh_image, 125, 255, cv::THRESH_BINARY_INV);
        // RCLCPP_INFO(this->get_logger(), "Threshold Image width and height: %d x %d", thresh_image.cols, thresh_image.rows);
        save_debug_image(thresh_image, image_path, "threshold");

        // Morph image
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
        cv::Mat morph_image;
        cv::morphologyEx(thresh_image, morph_image, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(morph_image, morph_image, cv::MORPH_CLOSE, kernel);
        save_debug_image(morph_image, image_path, "morphology"); 

        const cv::Mat clean_image = morph_image;

        // Calculate percent of ilmenite in sample
        int count_ilmenite = cv::countNonZero(clean_image);
        double ilmenite_percent = (static_cast<double>(count_ilmenite) / (clean_image.cols * clean_image.rows)) * 100;
        RCLCPP_INFO(this->get_logger(), "%s - %.2f %%", image_path.filename().c_str(), ilmenite_percent);

    }

    void save_debug_image(const cv::Mat& image, const fs::path& original_path, const std::string& suffix)
{

    fs::path debug_folder = "/home/tomek/scorpio_ws/src/scorpio_zadanie_rekrutacyjne_software/assets/debug";
    fs::create_directories(debug_folder);

    std::string base_name = original_path.stem().string();
    std::string file_name = base_name + "_" + suffix + ".png";

    fs::path output_path = debug_folder / file_name;

    bool success = cv::imwrite(output_path.string(), image);

    if (!success)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to save debug image: %s", output_path.c_str());
    }
    else
    {
        // RCLCPP_INFO(this->get_logger(), "Saved debug image: %s", output_path.c_str());
    }
}
};

}  // namespace recruitment_task

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(recruitment_task::IlmeniteCheckNode)