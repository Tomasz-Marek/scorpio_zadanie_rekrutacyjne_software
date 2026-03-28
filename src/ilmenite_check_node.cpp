#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
        // TODO: set path to samples folder
        const fs::path samples_folder = "/home/tomek/scorpio_ws/src/scorpio_zadanie_rekrutacyjne_software/assets/ilmenite_samples";
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
            return image_paths;
        }
        // TODO: check if path is a directory
        if (!fs::is_directory(folder_path))
        {
            RCLCPP_WARN(this->get_logger(), "This path is not directory: %s", folder_path.c_str());
            return image_paths;
        }
        // TODO: iterate through directory entries
        // TODO: keep only jpg / jpeg / png files
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
        // TODO: print current file name
        if (!fs::exists(image_path))
        {
            RCLCPP_WARN(this->get_logger(), "Invalid image path: %s", image_path.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Inspecting sample: %s", image_path.filename().c_str());
        
        // TODO: load image using cv::imread
        cv::Mat sample_image = cv::imread(image_path.string());
        
        // TODO: check if image is empty
        if (sample_image.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Error while reading the image: %s", image_path.c_str());
            return;
        }

        // TODO: print image width and height
        RCLCPP_INFO(this->get_logger(), "Sample Image width and height: %d x %d", sample_image.cols, sample_image.rows);

        cv::Mat gray_image;
        cv::cvtColor(sample_image, gray_image, cv::COLOR_BGR2GRAY);

        // TODO: print image width and height
        RCLCPP_INFO(this->get_logger(), "Grayscale Image width and height: %d x %d", gray_image.cols, gray_image.rows);
        save_debug_image(gray_image, image_path, "gray");

        cv::Mat blur_mask;
        cv::GaussianBlur(gray_image, blur_mask, cv::Size(3, 3), 0);


        cv::Mat ilmenite_mask;
        cv::threshold(gray_image, ilmenite_mask, 100, 255, cv::THRESH_BINARY_INV);
        // TODO: print image width and height
        RCLCPP_INFO(this->get_logger(), "Threshold Image width and height: %d x %d", ilmenite_mask.cols, ilmenite_mask.rows);
        save_debug_image(ilmenite_mask, image_path, "mask");

        
        int count_ilmenite = cv::countNonZero(ilmenite_mask);

        double ilmenite_percent = (static_cast<double>(count_ilmenite) / (ilmenite_mask.cols * ilmenite_mask.rows)) * 100;

        RCLCPP_INFO(this->get_logger(), "%s - %.2f %%", image_path.filename().c_str(), ilmenite_percent);

        // cv::imshow("Sample Image", sample_image);
        // cv::imshow("Gray Image", gray_image);
        // cv::imshow("Ilmenite Mask", ilmenite_mask);
        // cv::waitKey(0);
    }
    void save_debug_image(const cv::Mat& image, const fs::path& original_path, const std::string& suffix)
{
    // folder debugowy
    fs::path debug_folder = "/home/tomek/scorpio_ws/src/scorpio_zadanie_rekrutacyjne_software/assets/debug";

    // utwórz folder jeśli nie istnieje
    fs::create_directories(debug_folder);

    // nazwa bazowa pliku (bez rozszerzenia)
    std::string base_name = original_path.stem().string();

    // budujemy nazwę pliku wyjściowego
    std::string file_name = base_name + "_" + suffix + ".png";

    // pełna ścieżka
    fs::path output_path = debug_folder / file_name;

    // zapis obrazu
    bool success = cv::imwrite(output_path.string(), image);

    if (!success)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to save debug image: %s", output_path.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Saved debug image: %s", output_path.c_str());
    }
}
};

}  // namespace recruitment_task

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(recruitment_task::IlmeniteCheckNode)