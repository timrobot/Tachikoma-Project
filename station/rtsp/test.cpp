#include <gst/gst.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

static GMainLoop *loop;
cv::VideoCapture cap;

static void cb_need_data(GstElement *appsrc, guint unused_size, gpointer user_data) {
  static gboolean white = FALSE;
  static GstClockTime timestamp = 0;
  GstBuffer *buffer;
  guint size;
  GstFlowReturn ret;


  // grab cam frame
  cv::Mat image;
  cap.read(image);
  uchar *data = image.data;
  size = image.cols * image.rows * image.channels();

  buffer = gst_buffer_new_allocate (NULL, size, NULL);

  /* this makes the image black/white */
  //gst_buffer_memset (buffer, 0, white ? 0xff : 0x0, size);
  //white = !white;
  gst_buffer_fill(buffer, 0, data, size);

  GST_BUFFER_PTS (buffer) = timestamp;
  GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 10);

  timestamp += GST_BUFFER_DURATION (buffer);

  g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);

  if (ret != GST_FLOW_OK) {
    /* something wrong, stop pushing */
    g_main_loop_quit (loop);
  }
}

int main(int argc, char *argv[]) {

  // open cam
  cap.open(0);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 640); 
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480); 

  GstElement *pipeline, *appsrc, *conv, *videosink;

  /* init GStreamer */
  gst_init (&argc, &argv);
  loop = g_main_loop_new (NULL, FALSE);

  /* setup pipeline */
  pipeline = gst_pipeline_new ("pipeline");
  appsrc = gst_element_factory_make ("appsrc", "source");
  conv = gst_element_factory_make ("videoconvert", "conv");
  videosink = gst_element_factory_make ("xvimagesink", "videosink");

  /* setup */
  g_object_set (G_OBJECT (appsrc), "caps",
      gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, "BGR",
        "width", G_TYPE_INT, 640,
        "height", G_TYPE_INT, 480,
        "framerate", GST_TYPE_FRACTION, 0, 1,
        NULL), NULL);
  gst_bin_add_many (GST_BIN (pipeline), appsrc, conv, videosink, NULL);
  gst_element_link_many (appsrc, conv, videosink, NULL);

  /* setup appsrc */
  g_object_set (G_OBJECT (appsrc),
      "stream-type", 0,
      "format", GST_FORMAT_TIME, NULL);
  g_signal_connect (appsrc, "need-data", G_CALLBACK (cb_need_data), NULL);

  /* play */
  gst_element_set_state (pipeline, GST_STATE_PLAYING);
  g_main_loop_run (loop);

  /* clean up */
  gst_element_set_state (pipeline, GST_STATE_NULL);
  gst_object_unref (GST_OBJECT (pipeline));
  g_main_loop_unref (loop);

  return 0;
}
