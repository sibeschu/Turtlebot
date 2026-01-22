#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

class BirdViewNode : public rclcpp::Node
{
public:
  BirdViewNode() : Node("bird_view")
  {
    board_w_ = this->declare_parameter<int>("board_w", 9);
    board_h_ = this->declare_parameter<int>("board_h", 6);
    cell_px_ = this->declare_parameter<double>("cell_px", 60.0);
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/image_rect_color");

    if (board_w_ <= 1 || board_h_ <= 1) {
      throw std::runtime_error("board_w and board_h must be > 1");
    }

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BirdViewNode::onImage, this, std::placeholders::_1));

    cv::namedWindow("camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("bird_view", cv::WINDOW_AUTOSIZE);

    RCLCPP_INFO(get_logger(), "Subscribing to: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Chessboard: %dx%d", board_w_, board_h_);
  }

  ~BirdViewNode() override
  {
    cv::destroyAllWindows();
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    const cv::Mat frame = cv_ptr->image;
    if (frame.empty()) return;

    cv::Mat vis = frame.clone();

    const cv::Size board_sz(board_w_, board_h_);
    std::vector<cv::Point2f> corners;

    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    const bool found = cv::findChessboardCorners(frame, board_sz, corners, flags);

    if (found) {
      cv::Mat gray;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));

      cv::drawChessboardCorners(vis, board_sz, corners, found);

      // Source points in camera image
      cv::Point2f imgPts[4];
      imgPts[0] = corners.at(0);
      imgPts[1] = corners.at(board_w_ - 1);
      imgPts[2] = corners.at((board_h_ - 1) * board_w_);
      imgPts[3] = corners.at((board_h_ - 1) * board_w_ + (board_w_ - 1));

      // Destination rectangle in pixels (bird view)
      const float dst_w = static_cast<float>((board_w_ - 1) * cell_px_);
      const float dst_h = static_cast<float>((board_h_ - 1) * cell_px_);

      cv::Point2f dstPts[4];
      dstPts[0] = cv::Point2f(0.f, 0.f);
      dstPts[1] = cv::Point2f(dst_w, 0.f);
      dstPts[2] = cv::Point2f(0.f, dst_h);
      dstPts[3] = cv::Point2f(dst_w, dst_h);

      cv::Mat H = cv::getPerspectiveTransform(imgPts, dstPts);

      cv::Mat bird;
      cv::warpPerspective(frame, bird, H, frame.size(), cv::INTER_LINEAR);
      cv::imshow("bird_view", bird);
    }

    cv::imshow("camera", vis);

    // Needed for OpenCV windows to refresh
    const int key = cv::waitKey(1);
    if (key == 27) { // ESC
      rclcpp::shutdown();
    }
  }

  int board_w_{9}, board_h_{6};
  double cell_px_{60.0};
  std::string image_topic_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BirdViewNode>());
  rclcpp::shutdown();
  return 0;
}
