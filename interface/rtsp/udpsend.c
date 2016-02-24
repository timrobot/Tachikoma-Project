#include <gstreamer-1.0/gst/gst.h>
#include <gstreamer-1.0/gst/gstelement.h>
#include <gstreamer-1.0/gst/gstpipeline.h>
#include <gstreamer-1.0/gst/gstutils.h>
#include <gstreamer-1.0/gst/app/gstappsrc.h>
#include <opencv2/corecore.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <signal.h>
#include <cstdlib>
#include <cstring>
#include "udplink.h"

static GMainLoop *loop;
static int stopsig;
static unsigned char gst_initialized;
static cv::VideoCapture cap;

static void callback_need_data(
    GstElement *appsrc,
    guint unused_size,
    gpointer user_data) {
  static GstClockTime timestamp = 0;
  GstBuffer *buffer;
  guint size;
  GstFlowReturn ret;

  // grab camera frame
  cv::Mat image;
  cap.read(image);
  uchar *data = image.data;

  // create a new gst buffer
  size = image.cols * image.rows * image.channels();
  buffer = gst_buffer_new_allocate(NULL, size, NULL);

  // copy over the data
  gst_buffer_fill(buffer, 0, data, size);

  // timer stuff
  GST_BUFFER_PTS(buffer) = timestamp;
  GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 20); // 10 fps?
  timestamp += GST_BUFFER_DURATION(buffer);

  // put the buffer onto the appsrc element
  g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
  if (ret != GST_FLOW_OK) {
    g_main_loop_quit(loop);
  }
}

int start_streaming(udpout_t *vidout, const char *hoset, const int port) {
  // set up gsreamer
  GstStateChangeReturn ret;
  gboolean link_ok;

  // create elements
  vidout->source  = (void *)gst_element_factory_make("appsrc", "source");
  vidout->queue   = (void *)
  vidout->convert = (void *)gst_element_factory_make("videoconvert", "convert");
}
