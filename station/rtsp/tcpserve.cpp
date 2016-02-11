#include <gstreamer-1.0/gst/gst.h>
#include <gstreamer-1.0/gst/gstelement.h>
#include <gstreamer-1.0/gst/gstpipeline.h>
#include <gstreamer-1.0/gst/gstutils.h>
#include <gstreamer-1.0/gst/app/gstappsrc.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <signal.h>
#include <cstdlib>
#include <cstring>
#include "tcplink.h"


// Look at gstscripts/start_mod.sh for shell script-based code
const char *HOST = "192.168.43.57";
const int PORT = 9001;

static GMainLoop *loop;
static int stopsig;
static unsigned char gst_initialized;
static cv::VideoCapture cap;

static void cb_need_data(GstElement *appsrc, guint unused_size, gpointer user_data) {
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
  //memcpy(GST_BUFFER_DATA(buffer), data, GST_BUFFER_SIZE(buffer));
  gst_buffer_fill(buffer, 0, data, size);

  // timer stuff
  GST_BUFFER_PTS(buffer) = timestamp;
  GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 20); // 10 fps
  timestamp += GST_BUFFER_DURATION(buffer);

  // put the buffer onto the appsrc element
  g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
  if (ret != GST_FLOW_OK) {
    g_main_loop_quit(loop);
  }
}

int start_streaming(rtsplink_t *vidout, const char *host, const int port) {
  // set up gstreamer
  GstStateChangeReturn ret;
  gboolean link_ok;

  // create elements
  //vidout->src = (void *)gst_element_factory_make("v4l2src", "src");
  vidout->src = (void *)gst_element_factory_make("appsrc", "src");
  vidout->cvt = (void *)gst_element_factory_make("videoconvert", "cvt");
  vidout->enc = (void *)gst_element_factory_make("x264enc", "enc");
  vidout->mux = (void *)gst_element_factory_make("mpegtsmux", "mux");
  vidout->sink = (void *)gst_element_factory_make("tcpserversink", "sink");

  // modify the element's properties
  //g_object_set((GstElement *)vidout->src, "device", "/dev/video0", NULL);
//  g_object_set(G_OBJECT(vidout->src), "caps",
//      gst_caps_new_simple("video/x-raw",
//        "format", G_TYPE_STRING, "BGR",
//        "width", G_TYPE_INT, 640, // send over a single video for now
//        "height", G_TYPE_INT, 480,
//        "framerate", GST_TYPE_FRACTION, 0, 1, NULL),
//      NULL);
  gst_util_set_object_arg(G_OBJECT((GstElement *)vidout->enc), "tune", "zerolatency");
  gst_util_set_object_arg(G_OBJECT((GstElement *)vidout->enc), "pass", "quant");
  g_object_set((GstElement *)vidout->enc, "quantizer", 20, NULL);
  g_object_set((GstElement *)vidout->sink, "host", host, NULL);
  g_object_set((GstElement *)vidout->sink, "port", port, NULL);

  // create capabilites
  vidout->caps = (void *)gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, "BGR",
      "width", G_TYPE_INT, 640,
      "height", G_TYPE_INT, 480,
      "framerate", GST_TYPE_FRACTION, 0, 1, NULL,
      NULL);

  // create pipeline
  printf("creating pipeline\n");
  vidout->pipeline = (void *)gst_pipeline_new("vidpipeline");
  if (!vidout->src ||
      !vidout->cvt ||
      !vidout->enc ||
      !vidout->mux ||
      !vidout->sink ||
      !vidout->pipeline) {
    g_printerr("not all elements created %p %p %p %p %p %p\n",
      vidout->src,
      vidout->cvt,
      vidout->enc,
      vidout->mux,
      vidout->sink,
      vidout->pipeline);
    memset(vidout, 0, sizeof(rtsplink_t)); // TODO
    return -1;
  }

  // build pipeline
  printf("building pipeline\n");
  gst_bin_add_many(
      GST_BIN((GstElement *)vidout->pipeline),
      (GstElement *)vidout->src,
      (GstElement *)vidout->cvt,
      (GstElement *)vidout->enc,
      (GstElement *)vidout->mux,
      (GstElement *)vidout->sink,
      NULL);
  link_ok = gst_element_link_filtered(
      (GstElement *)vidout->src,
      (GstElement *)vidout->cvt,
      (GstCaps *)vidout->caps);
  gst_caps_unref((GstCaps *)vidout->caps);
  if (link_ok != TRUE) {
    g_printerr("Source and encoder could not be linked\n");
    goto error;
  }
  link_ok = gst_element_link_many(
      (GstElement *)vidout->cvt,
      (GstElement *)vidout->enc,
      (GstElement *)vidout->mux,
      (GstElement *)vidout->sink,
      NULL);
  if (link_ok != TRUE) {
    g_printerr("src, enc, mux, and sink could not be linked\n");
    goto error;
  }

  // setup the appsrc
  g_object_set(G_OBJECT(vidout->src),
      "stream-type", 0,
      "format", GST_FORMAT_TIME, NULL);
  g_signal_connect(vidout->src, "need-data", G_CALLBACK(cb_need_data), NULL);

  // start playing
  printf("playing\n");
  ret = gst_element_set_state((GstElement *)vidout->pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Unable to set the pipeline to the playing state\n");
    goto error;
  }
  return 0;

error:
  gst_object_unref((GstElement *)vidout->pipeline);
  memset(vidout, 0, sizeof(rtsplink_t));
  return -1;
}

void stop_streaming(rtsplink_t *vidout) {
  gst_element_set_state((GstElement *)vidout->pipeline, GST_STATE_NULL);
  gst_object_unref((GstElement *)vidout->pipeline);
  memset(vidout, 0, sizeof(rtsplink_t));
}

void stopprog(int v) {
  stopsig = 1;
  printf("SIGINT recv'd\n");
}

int main(int argc, char *argv[]) {

  // init camera
  cap.open(0);

  // init gst
  gst_init(NULL, NULL);
  loop = g_main_loop_new(NULL, FALSE);
  gst_initialized = 1;

  // set up pipeline
  rtsplink_t vidout;
  int ret = start_streaming(&vidout, HOST, PORT);
  if (ret == -1) {
    printf("stopping due to error\n");
    return 1;
  }

  // run the program
  g_main_loop_run(loop);

  // clean up if the program stops
  stop_streaming(&vidout);
  g_main_loop_unref(loop);
  return 0;
}
