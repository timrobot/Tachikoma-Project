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
#include "tcprecv.h"


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

static void cb_new_pad(GstElement *element, GstPad *pad, gpointer data) {
  GstPad *sinkpad;
  GstElement *demux = (GstElement *)data;
  sinkpad = gst_element_get_static_pad(demux, "sink");
  gst_pad_link(pad, sinkpad);
  gst_object_unref(sinkpad);
}

int start_streaming(tcprecv_t *vidin, const char *host, const int port) {
  // set up gstreamer
  GstStateChangeReturn ret;
  gboolean link_ok;

  // create elements
  //vidin->src = (void *)gst_element_factory_make("v4l2src", "src");
  vidin->src = (void *)gst_element_factory_make("tcpclientsrc", "src");
  vidin->demux = (void *)gst_element_factory_make("tsdemux", "demux");
  vidin->parse = (void *)gst_element_factory_make("h264parse", "parse");
  vidin->dec = (void *)gst_element_factory_make("avdec_h264", "dec");
  vidin->sink = (void *)gst_element_factory_make("xvimagesink", "sink"); // change this later

  // modify the element's properties
  g_object_set((GstElement *)vidin->src, "host", host, NULL);
  g_object_set((GstElement *)vidin->src, "port", port, NULL);

  // create capabilites
//  vidin->caps = (void *)gst_caps_new_simple("video/x-raw",
//      "format", G_TYPE_STRING, "BGR",
//      "width", G_TYPE_INT, 640,
//      "height", G_TYPE_INT, 480,
//      "framerate", GST_TYPE_FRACTION, 0, 1, NULL,
//      NULL);

  // create pipeline
  printf("creating pipeline\n");
  vidin->pipeline = (void *)gst_pipeline_new("vidpipeline");
  if (!vidin->src ||
      !vidin->demux ||
      !vidin->parse ||
      !vidin->dec ||
      !vidin->sink ||
      !vidin->pipeline) {
    g_printerr("not all elements created %p %p %p %p %p %p\n",
      vidin->src,
      vidin->demux,
      vidin->parse,
      vidin->dec,
      vidin->sink,
      vidin->pipeline);
    memset(vidin, 0, sizeof(tcprecv_t)); // TODO
    return -1;
  }

  // build pipeline
  printf("building pipeline\n");
  gst_bin_add_many(
      GST_BIN((GstElement *)vidin->pipeline),
      (GstElement *)vidin->src,
      (GstElement *)vidin->demux,
      (GstElement *)vidin->parse,
      (GstElement *)vidin->dec,
      (GstElement *)vidin->sink,
      NULL);
  link_ok = gst_element_link(
      (GstElement *)vidin->src,
      (GstElement *)vidin->demux);
  if (link_ok != TRUE) { g_printerr("elements could not be linked\n"); goto error; } else { printf("good to go\n"); }
//  link_ok = gst_element_link(
//      (GstElement *)vidin->demux,
//      (GstElement *)vidin->parse);
//  if (link_ok != TRUE) { g_printerr("elements could not be linked\n"); goto error; } else { printf("good to go\n"); }
  link_ok = gst_element_link(
      (GstElement *)vidin->parse,
      (GstElement *)vidin->dec);
  if (link_ok != TRUE) { g_printerr("elements could not be linked\n"); goto error; } else { printf("good to go\n"); }
  link_ok = gst_element_link(
      (GstElement *)vidin->dec,
      (GstElement *)vidin->sink);
  if (link_ok != TRUE) { g_printerr("elements could not be linked\n"); goto error; } else { printf("good to go\n"); }

  // construct signals for later building
  g_signal_connect(vidin->demux, "pad-added", G_CALLBACK(cb_new_pad), NULL);

  // start playing
  printf("playing\n");
  ret = gst_element_set_state((GstElement *)vidin->pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Unable to set the pipeline to the playing state\n");
    goto error;
  }
  return 0;

error:
  gst_object_unref((GstElement *)vidin->pipeline);
  memset(vidin, 0, sizeof(tcprecv_t));
  return -1;
}

void stop_streaming(tcprecv_t *vidin) {
  gst_element_set_state((GstElement *)vidin->pipeline, GST_STATE_NULL);
  gst_object_unref((GstElement *)vidin->pipeline);
  memset(vidin, 0, sizeof(tcprecv_t));
}

int main(int argc, char *argv[]) {

  // init gst
  gst_init(NULL, NULL);
  loop = g_main_loop_new(NULL, FALSE);
  gst_initialized = 1;

  // set up pipeline
  tcprecv_t vidin;
  int ret = start_streaming(&vidin, HOST, PORT);
  if (ret == -1) {
    printf("stopping due to error\n");
    return 1;
  }

  // run the program
  g_main_loop_run(loop);

  // clean up if the program stops
  stop_streaming(&vidin);
  g_main_loop_unref(loop);
  return 0;
}
