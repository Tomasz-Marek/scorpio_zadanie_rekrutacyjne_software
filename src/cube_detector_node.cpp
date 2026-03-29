#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <limits>
#include <string>
#include <vector>
#include <cmath>

namespace recruitment_task {

struct ColorRange{
  cv::Scalar lower;
  cv::Scalar upper;
};
struct DetectionParams {
  double min_area = 30.0;
  double max_area = 2000.0;

  int min_width = 10;
  int max_width = 70;

  int min_height = 10;
  int max_height = 70;

  double min_aspect_ratio = 0.5;
  double max_aspect_ratio = 1.5;

  double min_fill_ratio = 0.5;

  cv::Size morphology_kernel = cv::Size(3, 3);

  bool debug = true;
};

struct CubeDetection {
  bool detected = false;
  cv::Rect bounding_box;
  cv::Point center;
  std::vector<cv::Point> contour;
  double contour_area = 0.0;
  double extent = 0.0;
  double solidity = 0.0;
  double aspect_ratio = 0.0;
  double score = -1.0;
};

struct CubeConfig {
  std::string name;
  std::vector<ColorRange> color_ranges;
  DetectionParams params;
  cv::Scalar draw_color_bgr;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cube_publisher;
};


class CubeDetectorNode : public rclcpp::Node {
public:
  explicit CubeDetectorNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("cube_detector", options)
  {
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/cube_detector/detected_cubes/image", rclcpp::QoS(10));

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/zed/zed_node/left_raw/image_raw_color",
      rclcpp::QoS(10),
      std::bind(&CubeDetectorNode::image_callback, this, std::placeholders::_1));
    
    red_cube_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/cube_detector/red_cube/position_on_frame", rclcpp::QoS(10));

    green_cube_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/cube_detector/green_cube/position_on_frame", rclcpp::QoS(10));

    blue_cube_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/cube_detector/blue_cube/position_on_frame", rclcpp::QoS(10));

    white_cube_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/cube_detector/white_cube/position_on_frame", rclcpp::QoS(10));
    initialize_cube_configs();
    RCLCPP_INFO(this->get_logger(), "CubeDetectorNode started.");
  }

private:

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr red_cube_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr green_cube_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr blue_cube_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr white_cube_publisher_;
  
  std::vector<CubeConfig> cube_configs_;

  void initialize_cube_configs();
  CubeDetection detect_cube(const cv::Mat& BGR_frame, const CubeConfig& cube_config);
  cv::Mat clean_mask(const cv::Mat& mask, const DetectionParams& params);

  void publish_cube_position(const CubeConfig& cube_config, const CubeDetection& detection){

    geometry_msgs::msg::Point point_msg;
    point_msg.z = 0.0;

      if (detection.detected)
      {
        point_msg.x= detection.center.x;
        point_msg.y = detection.center.y;
      }
      else
      {
        point_msg.x= std::numeric_limits<double>::lowest();
        point_msg.y = std::numeric_limits<double>::lowest();
      }

      cube_config.cube_publisher->publish(point_msg);

  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

      cv::Mat annotated = frame.clone();

      // Use only the upper half of the frame for detection
      int roi_height = static_cast<int>(0.5 * frame.rows);
      if (roi_height <= 0) {
        roi_height = 1;
      }

      cv::Rect top_roi(0, 0, frame.cols, roi_height);
      cv::Mat frame_top = frame(top_roi);

      for (const auto& cube_config : cube_configs_) {
        CubeDetection detection = detect_cube(frame_top, cube_config);

        // Shift detection back to full-frame coordinates
        if (detection.detected) {
          detection.bounding_box.y += top_roi.y;
          detection.center.y += top_roi.y;
        }

        publish_cube_position(cube_config, detection);

        if (detection.detected) {
          cv::rectangle(annotated, detection.bounding_box, cube_config.draw_color_bgr, 2);

          cv::putText(
            annotated,
            cube_config.name,
            cv::Point(detection.bounding_box.x, detection.bounding_box.y - 5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cube_config.draw_color_bgr,
            2
          );
        }
      }

      auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg();
      image_publisher_->publish(*output_msg);
    }
    catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }
};

void CubeDetectorNode::initialize_cube_configs()
{
  CubeConfig red_cube;
  red_cube.name = "red";
  ColorRange red_range_lower, red_range_upper;
  red_range_lower.lower = cv::Scalar(4, 190, 110);
  red_range_lower.upper = cv::Scalar(16, 255, 255);
  red_range_upper.lower = cv::Scalar(176, 190, 110);
  red_range_upper.upper = cv::Scalar(179, 255, 255);
  red_cube.color_ranges.push_back(red_range_lower);
  red_cube.color_ranges.push_back(red_range_upper);
  red_cube.draw_color_bgr = cv::Scalar(0, 0, 255);
  red_cube.cube_publisher = red_cube_publisher_;

  CubeConfig green_cube;
  green_cube.name = "green";
  ColorRange green_range;
  green_range.lower = cv::Scalar(52, 95, 35);
  green_range.upper = cv::Scalar(76, 255, 170);
  green_cube.color_ranges.push_back(green_range);
  green_cube.draw_color_bgr = cv::Scalar(0, 255, 0);
  green_cube.cube_publisher = green_cube_publisher_;

  CubeConfig blue_cube;
  blue_cube.name = "blue";
  ColorRange blue_range;
  blue_range.lower = cv::Scalar(92, 90, 55);
  blue_range.upper = cv::Scalar(122, 255, 235);
  blue_cube.color_ranges.push_back(blue_range);
  blue_cube.draw_color_bgr = cv::Scalar(255, 0, 0);
  blue_cube.cube_publisher = blue_cube_publisher_;

  CubeConfig white_cube;
  white_cube.name = "white";
  ColorRange white_range;
  white_range.lower = cv::Scalar(0, 0, 185);
  white_range.upper = cv::Scalar(179, 40, 255);
  white_cube.color_ranges.push_back(white_range);
  white_cube.draw_color_bgr = cv::Scalar(255, 255, 255);
  white_cube.cube_publisher = white_cube_publisher_;
  white_cube.params.min_area = 60.0;
  white_cube.params.max_area = 1800.0;
  white_cube.params.min_width = 6;
  white_cube.params.max_width = 50;
  white_cube.params.min_height = 6;
  white_cube.params.max_height = 50;
  white_cube.params.min_aspect_ratio = 0.65;
  white_cube.params.max_aspect_ratio = 1.35;
  white_cube.params.min_fill_ratio = 0.45;


  cube_configs_.push_back(red_cube);
  cube_configs_.push_back(green_cube);
  cube_configs_.push_back(blue_cube);
  cube_configs_.push_back(white_cube);
}

CubeDetection CubeDetectorNode::detect_cube(const cv::Mat& BGR_frame, const CubeConfig& cube_config) {
  CubeDetection detection;
  double best_score = -1.0;

  cv::Mat hsv_frame;
  cv::cvtColor(BGR_frame, hsv_frame, cv::COLOR_BGR2HSV);

  // Build color mask from one or more HSV ranges
  cv::Mat color_mask = cv::Mat::zeros(hsv_frame.size(), CV_8UC1);
  for (const auto& color_range : cube_config.color_ranges) 
  {
    cv::Mat partial_mask;
    cv::inRange(hsv_frame, color_range.lower, color_range.upper, partial_mask);
    cv::bitwise_or(color_mask, partial_mask, color_mask);
  }

  // Remove small noise and close small holes
  cv::Mat cleaned_mask = clean_mask(color_mask, cube_config.params);

  // Find external contours on the cleaned color mask
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(cleaned_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) 
  {
    double area = cv::contourArea(contour);
    if (area < cube_config.params.min_area || area > cube_config.params.max_area) 
    {
      continue;
    }

    cv::Rect bbox = cv::boundingRect(contour);
    if (bbox.width <= 0 || bbox.height <= 0) 
    {
      continue;
    }

    // Filter by bounding box size
    if (bbox.width < cube_config.params.min_width || bbox.width > cube_config.params.max_width || bbox.height < cube_config.params.min_height || bbox.height > cube_config.params.max_height) 
    {
      continue;
    }

    // Cube should look roughly square
    double aspect_ratio = static_cast<double>(bbox.width) / static_cast<double>(bbox.height);
    if (aspect_ratio < cube_config.params.min_aspect_ratio || aspect_ratio > cube_config.params.max_aspect_ratio) 
    {
      continue;
    }

    double rect_area = static_cast<double>(bbox.area());
    if (rect_area <= 0.0) 
    {
      continue;
    }

    // Fill ratio tells how well the contour fills its bounding box
    double fill_ratio = area / rect_area;
    if (fill_ratio < cube_config.params.min_fill_ratio) 
    {
      continue;
    }

    // EXTRA VALIDATION ONLY FOR WHITE CUBE
    if (cube_config.name == "white") 
    {
      // Extract candidate ROI from the original BGR frame
      cv::Mat roi_bgr = BGR_frame(bbox).clone();
      if (roi_bgr.empty()) 
      {
        continue;
      }

      // Convert ROI to grayscale
      cv::Mat roi_gray;
      cv::cvtColor(roi_bgr, roi_gray, cv::COLOR_BGR2GRAY);

      // Apply CLAHE to enhance local contrast
      cv::Mat roi_clahe;
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
      clahe->apply(roi_gray, roi_clahe);

      // Slight blur to stabilize edges
      cv::Mat roi_blur;
      cv::GaussianBlur(roi_clahe, roi_blur, cv::Size(3, 3), 0);

      // Detect edges inside the white candidate
      cv::Mat roi_edges;
      cv::Canny(roi_blur, roi_edges, 40, 120);

      // Find inner contours inside the candidate ROI
      std::vector<std::vector<cv::Point>> roi_contours;
      cv::findContours(roi_edges, roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      bool white_structure_ok = false;
      double best_white_inner_score = -1.0;

      for (const auto& roi_contour : roi_contours) 
      {
        double inner_area = cv::contourArea(roi_contour);
        if (inner_area < 20.0) 
        {
          continue;
        }

        cv::Rect inner_box = cv::boundingRect(roi_contour);
        if (inner_box.width <= 0 || inner_box.height <= 0) 
        {
          continue;
        }

        // Inner candidate should also be roughly rectangular
        double inner_aspect_ratio = static_cast<double>(inner_box.width) / static_cast<double>(inner_box.height);

        if (inner_aspect_ratio < 0.65 || inner_aspect_ratio > 1.45) 
        {
          continue;
        }

        double inner_rect_area = static_cast<double>(inner_box.area());
        if (inner_rect_area <= 0.0) 
        {
          continue;
        }

        double inner_fill_ratio = inner_area / inner_rect_area;
        if (inner_fill_ratio < 0.35) 
        {
          continue;
        }

        // Approximate contour to check if it looks like a quadrilateral
        std::vector<cv::Point> approx;
        double epsilon = 0.03 * cv::arcLength(roi_contour, true);
        cv::approxPolyDP(roi_contour, approx, epsilon, true);

        // Count how much of the ROI is actually edge-supported
        double edge_density = static_cast<double>(cv::countNonZero(roi_edges(inner_box))) / inner_rect_area;

        double rectangle_bonus = (approx.size() == 4) ? 0.35 : 0.0;
        double inner_score = inner_fill_ratio + edge_density + rectangle_bonus;

        if (inner_score > best_white_inner_score) 
        {
          best_white_inner_score = inner_score;
          white_structure_ok = true;
        }
      }

      // Reject white candidate if no meaningful inner rectangular structure was found
      if (!white_structure_ok) 
      {
        continue;
      }

      // Slightly boost white candidates that pass the structural validation
      double white_score = area + 200.0 * best_white_inner_score;

      if (white_score > best_score) 
      {
        const auto moments = cv::moments(contour);
        if (moments.m00 <= 1e-6) 
        {
          continue;
        }

        best_score = white_score;
        detection.detected = true;
        detection.bounding_box = bbox;
        detection.center = cv::Point(static_cast<int>(moments.m10 / moments.m00), static_cast<int>(moments.m01 / moments.m00));
        detection.contour = contour;
        detection.contour_area = area;
        detection.aspect_ratio = aspect_ratio;
        detection.extent = fill_ratio;
        detection.score = white_score;
      }

      continue;
    }

    // STANDARD SELECTION FOR RED / GREEN / BLUE 
    double score = area;

    if (score > best_score) 
    {
      const auto moments = cv::moments(contour);
      if (moments.m00 <= 1e-6) 
      {
        continue;
      }

      best_score = score;
      detection.detected = true;
      detection.bounding_box = bbox;
      detection.center = cv::Point(static_cast<int>(moments.m10 / moments.m00), static_cast<int>(moments.m01 / moments.m00));
      detection.contour = contour;
      detection.contour_area = area;
      detection.aspect_ratio = aspect_ratio;
      detection.extent = fill_ratio;
      detection.score = score;
    }
  }

  return detection;
}

cv::Mat CubeDetectorNode::clean_mask(const cv::Mat& mask, const DetectionParams& params) {
  cv::Mat cleaned_mask = mask.clone();

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, params.morphology_kernel);
  cv::morphologyEx(cleaned_mask, cleaned_mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(cleaned_mask, cleaned_mask, cv::MORPH_CLOSE, kernel);

  return cleaned_mask;
}

}  // namespace recruitment_task

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(recruitment_task::CubeDetectorNode)