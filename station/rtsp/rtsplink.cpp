#include <gstreamer-1.0/gst/gst.h>
#include <gstreamer-1.0/gst/gstelement.h>
#include <gstreamer-1.0/gst/gstpipeline.h>
#include <gstreamer-1.0/gst/gstutils.h>
#include <gstreamer-1.0/gst/app/gstappsrc.h>
#include <cstdio>
#include <signal.h>
#include <cstdlib>
#include <cstring>
#include "rtsplink.h"

// Look at gstscripts/start_mod.sh for shell script-based code
const char *HOST = "192.168.1.114";
const int PORT = 9001;
static int stopsig;
static unsigned char gst_initialized;

int start_streaming(rtsplink_t *vidout, const char *host, const int port) {
  // set up gstreamer
  GstStateChangeReturn ret;
  gboolean link_ok;
  if (!gst_initialized) {
    gst_init(NULL, NULL);
  }

  // create elements
  vidout->src = (void *)gst_element_factory_make("v4l2src", "src");
  vidout->enc = (void *)gst_element_factory_make("x264enc", "enc");
  vidout->mux = (void *)gst_element_factory_make("mpegtsmux", "mux");
  vidout->sink = (void *)gst_element_factory_make("tcpserversink", "sink");

  // modify the element's properties
  g_object_set((GstElement *)vidout->src, "device", "/dev/video0", NULL);
  gst_util_set_object_arg(G_OBJECT((GstElement *)vidout->enc), "tune", "zerolatency");
  gst_util_set_object_arg(G_OBJECT((GstElement *)vidout->enc), "pass", "quant");
  g_object_set((GstElement *)vidout->enc, "quantizer", 20, NULL);
  g_object_set((GstElement *)vidout->sink, "host", host, NULL);
  g_object_set((GstElement *)vidout->sink, "port", port, NULL);

  // create capabilites
  vidout->caps = (void *)gst_caps_new_simple("video/x-raw",
      "width", G_TYPE_INT, 640,
      "height", G_TYPE_INT, 480,
      NULL);

  // create pipeline
  printf("creating pipeline\n");
  vidout->pipeline = (void *)gst_pipeline_new("vidpipeline");
  if (!vidout->src ||
      !vidout->enc ||
      !vidout->mux ||
      !vidout->sink ||
      !vidout->pipeline) {
    g_printerr("not all elements created %p %p %p %p %p\n",
      vidout->src,
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
      (GstElement *)vidout->enc,
      (GstElement *)vidout->mux,
      (GstElement *)vidout->sink,
      NULL);
  link_ok = gst_element_link_filtered(
      (GstElement *)vidout->src,
      (GstElement *)vidout->enc,
      (GstCaps *)vidout->caps);
  gst_caps_unref((GstCaps *)vidout->caps);
  if (link_ok != TRUE) {
    g_printerr("Source and encoder could not be linked\n");
    goto error;
  }
  link_ok = gst_element_link_many(
      (GstElement *)vidout->enc,
      (GstElement *)vidout->mux,
      (GstElement *)vidout->sink,
      NULL);
  if (link_ok != TRUE) {
    g_printerr("Encoder, mux, and sink could not be linked\n");
    goto error;
  }

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
  rtsplink_t vidout;
  int ret = start_streaming(&vidout, HOST, PORT);
  if (ret == -1) {
    printf("stopping due to error\n");
    return 1;
  }
  while (!stopsig) ;
  stop_streaming(&vidout);
  return 0;
}
