// include open cv
#include <opencv2/opencv.hpp>
#include <random>
#include <string>

int main() {
  int height = 500;
  int width = 1000;
  // Create a way to visualize the dtmf signal amplitude
  cv::Mat background(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  // Put text of the dtmf frequencies
  std::string dtmf_freqs[8] = {"697",  "770",  "852",  "941",
                               "1209", "1336", "1477", "1633"};
  for (int i = 0; i < 8; i++) {
    cv::putText(background, dtmf_freqs[i],
                cv::Point(0, height / 8 * i + height / 16),
                cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                static_cast<double>(height) / 400., cv::Scalar(255, 255, 255));
  }
  // Draw divider line
  // Position the line beside the text
  int linePos = cv::getTextSize(dtmf_freqs[7], cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                                static_cast<double>(height) / 400., 1, nullptr)
                    .width +
                10;
  cv::line(background, cv::Point(linePos, 0), cv::Point(linePos, height),
           cv::Scalar(255, 255, 255));

  while (1) {
    // Draw the dtmf signal amplitude on a copy of the background
    cv::Mat dtmf_signal;
    background.copyTo(dtmf_signal);
    // make random number generator (replacement for the dtmf signal amplitude)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, width - linePos - 1);
    // Show the dtmf signal amplitude
    for (int i = 0; i < 8; i++) {
      // Draw the dtmf signal amplitude
      cv::rectangle(dtmf_signal,
                    cv::Point(linePos, height / 8 * i + height / 16),
                    cv::Point(linePos + dis(gen), height / 8 * i + height / 34),
                    cv::Scalar(255, 255, 255), -1);
    }

    // Show the window
    cv::setWindowTitle("w1", "DTMF Signal Amplitude");
    cv::imshow("w1", dtmf_signal);
    char key = cv::waitKey(100);
    switch (key) {
    case 'q':
      return 0;
    case 'p':
      cv::waitKey(0);
    default:
      break;
    }
  }
  return 0;
}