#ifndef __TK_KINECT_H__
#define __TK_KINECT_H__

#include <cstdint>
#include <vector>
#include <libfreenect.hpp>
#include <opencv2/core/core.hpp>
#include <thread>
#include <mutex>

class KinectDevice : public Freenect::FreenectDevice {
  private:
    std::mutex video_lock;
    std::mutex depth_lock;
    std::vector<uint8_t> depth_buffer;
    std::vector<uint8_t> video_buffer;
    std::vector<uint16_t> gamma_buffer;
    bool new_depth_frame;
    bool new_video_frame;
    cv::Mat depthMat;
    cv::Mat videoMat;

    bool new_distance_frame;
    cv::Mat distanceMat;
    double raw2meters(uint16_t raw);

  public:
    KinectDevice(freenect_context *_ctx, int _index);
    ~KinectDevice();
    // Do not call directly even in child
    void DepthCallback(void *data, uint32_t timestamp);
    // Do not call directly even in child
    void VideoCallback(void *data, uint32_t timestamp);
    bool getDepth(cv::Mat &output);
    bool getVideo(cv::Mat &output);
    bool getDistance(cv::Mat &output);

    int rows;
    int cols;
};

#endif
