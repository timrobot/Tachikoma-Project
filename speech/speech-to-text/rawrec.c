#include <gst/gst.h>
#include <stdlib.h>
#include <string.h>
#include "rawrec.h"

static unsigned char gst_initiallized;

/** Start the recording of the audio device.
 *  @param rr
 *    the struct containing the necessary information for recording
 *  @param fsinkloc
 *    the filename of the destination
 *  @return 0 on success, -1 on error
 */
int start_recording(rawrec_t *rr, char *fsinkloc) {
  GstStateChangeReturn ret;
  gboolean link_ok;

  // init
  if (!gst_initiallized) {
    gst_init(NULL, NULL);
  }

  // create elements
  rr->source = (void *)gst_element_factory_make("alsasrc", "audsrc");
  rr->sink = (void *)gst_element_factory_make("filesink", "audsink");
  rr->buffer = (void *)gst_element_factory_make("queue", "audque");

  // modify the sink's properties
  g_object_set((GstElement *)rr->sink, "location", fsinkloc, NULL);
  rr->filesink_loc = fsinkloc;

  // create capabilities (pocketsphinx)
  rr->caps = (void *)gst_caps_new_simple("audio/x-raw-int",
      "endianness", G_TYPE_INT, 1234,
      "signed", G_TYPE_BOOLEAN, TRUE,
      "width", G_TYPE_INT, 16,
      "height", G_TYPE_INT, 16,
      "rate", G_TYPE_INT, 16000,
      "channels", G_TYPE_INT, 1,
      "depth", G_TYPE_INT, 16,
      NULL);

  // create pipeline
  rr->pipeline = (void *)gst_pipeline_new("audpipeline");
  if (!rr->source || !rr->sink || !rr->pipeline || !rr->caps || !rr->buffer) {
    g_printerr("Not all elements created\n");
    memset(rr, 0, sizeof(rawrec_t));
    return -1;
  }

  // build pipeline
  gst_bin_add_many(
      GST_BIN((GstElement *)rr->pipeline),
      (GstElement *)rr->source,
      (GstElement *)rr->buffer,
      (GstElement *)rr->sink,
      NULL);
  link_ok = gst_element_link_filtered(
      (GstElement *)rr->source,
      (GstElement *)rr->buffer,
      (GstCaps *)rr->caps);
  gst_caps_unref((GstCaps *)rr->caps);
  if (link_ok != TRUE) {
    g_printerr("Source & buffer could not be linked.\n");
    goto error;
  }
  link_ok = gst_element_link((GstElement *)rr->buffer, (GstElement *)rr->sink);
  if (link_ok != TRUE) {
    g_printerr("Buffer & sink could not be linked.\n");
    goto error;
  }

  // start playing
  ret = gst_element_set_state((GstElement *)rr->pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    g_printerr("Unable to set the pipeline to the playing state\n");
    goto error;
  }
  return 0;

error:
  gst_object_unref((GstElement *)rr->pipeline);
  memset(rr, 0, sizeof(rawrec_t));
  return -1;
}

/** Stop recording of the audio device.
 *  @param rr
 *    the struct of the pipeline information
 */
void stop_recording(rawrec_t *rr) {
  // free
  gst_element_set_state((GstElement *)rr->pipeline, GST_STATE_NULL);
  gst_object_unref((GstElement *)rr->pipeline);
  memset(rr, 0, sizeof(rawrec_t));
}
