#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[]) {
  // autodetect and open the video capture device
  cv::VideoCapture cap(atoi(argv[1]));
  if (!cap.isOpened()) {
    printf("Not able to open camera\n");
    return 1;
  }

  // grad and display the frame
  cv::namedWindow("Camera");
  cv::Mat frame;
  for (;;) {
    cap >> frame;
    cv::imshow("Camera", frame);
    if (cv::waitKey(30) >= 0) {
      break;
    }
  }

  return 0;
}
