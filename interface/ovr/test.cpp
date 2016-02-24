#include <gst/gst.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

typedef struct gst_app {
  GstPipeline *pipeline;
  GstElement *src[2];
  GstCaps *vcaps[2];
  GstElement *crop[2];
  GstElement *distort[2];
  GstElement *box[2];
  GstElement *mixer;
  GstElement *convert;
  GstElement *sink;
  GMainLoop *loop;
} gst_app_t;

static gst_app_t gst_app;

static gboolean bus_callback(GstBus *bus, GstMessage *message, gpointer *ptr) {
  gst_app_t *app = (gst_app_t *)ptr;

  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR: {
      gchar *debug;
      GError *err;
      
      gst_message_parse_error(message, &err, &debug);
      g_print("Error: %s\n", err->message);
      g_error_free(err);
      g_free(debug);
      g_main_loop_quit(app->loop);
    }
    break;
    case GST_MESSAGE_WARNING: {
      gchar *debug;
      GError *err;
      gchar *name;

      gst_message_parse_warning(message, &err, &debug);
      g_print("Name of src %s\n", name ? name : "nil");
      g_error_free(err);
      g_free(debug);
    }
    break;
    case GST_MESSAGE_EOS: {
      g_print("End of Stream\n");
      g_main_loop_quit(app->loop);
    }
    break;
    case GST_MESSAGE_STATE_CHANGED: {
    }
    break;
    default: {
      g_print("got message %s\n", gst_message_type_get_name(GST_MESSAGE_TYPE(message)));
    }
    break;
  }
  return TRUE;
}

static void pad_added_mix_handler(GstElement *src, GstPad *pad, CustomData *data) {
  
}

static void start_video_buffer(const char *cam1name, const char *cam2name) {
  gst_app_t *app = &gst_app;
  GstBus *bus;
  gboolean link_ok;
  GstStateChangeReturn state_ret;
  gint i;

  gst_init(NULL, NULL);

  // SET UP EVERYTHING
  app->pipeline = (GstPipeline *)gst_pipeline_new("mypipeline");
  bus = gst_pipeline_get_bus(app->pipeline);
  gst_bus_add_watch(bus, (GstBusFunc)bus_callback, app);
  gst_object_unref(bus);

  app->src[0] = gst_element_factory_make("v4l2src", "mysrc");
  app->src[1] = gst_element_factory_make("v4l2src", "mysrc");
  g_object_set(G_OBJECT(app->src[0]), "device", cam1name, NULL);
  g_object_set(G_OBJECT(app->src[1]), "device", cam2name, NULL);
  for (i = 0; i < 2; i++) {
    gchar ename[64];
    app->vcaps[i] = gst_caps_new_simple("video/x-raw",
        "width", G_TYPE_INT, 640, "height", G_TYPE_INT, 480, NULL);
    sprintf(ename, "mycropper%d", i);
    app->crop[i] = gst_element_factory_make("videocrop", ename);
    g_object_set(G_OBJECT(app->crop[i]),
        "top", 0, "left", 128, "right", 128, "bottom", 0, NULL);
    sprintf(ename, "mydistort%d", i);
    app->distort[i] = gst_element_factory_make("barreldistort", ename);
    sprintf(ename, "mybox%d", i);
    app->box[i] = gst_element_factory_make("videobox", ename);
  }
  app->mixer = gst_element_factory_make("videomixer", "mymixer");
  /* optional output */
  app->convert = gst_element_factory_make("videoconvert", "myconvert");
  app->sink = gst_element_factory_make("xvimagesink", "myvsink");

  g_assert(app->src[0]);
  g_assert(app->src[1]);
  g_assert(app->vcaps[0]);
  g_assert(app->vcaps[1]);
  g_assert(app->crop[0]);
  g_assert(app->crop[1]);
  g_assert(app->distort[0]);
  g_assert(app->distort[1]);
  g_assert(app->box[0]);
  g_assert(app->box[1]);
  g_assert(app->mixer);
  g_assert(app->convert);
  g_assert(app->sink);

  // build pipeline
  gst_bin_add_many(GST_BIN(app->pipeline),
      app->src[0],
      app->src[1],
      app->crop[0],
      app->crop[1],
      app->distort[0],
      app->distort[1],
      app->box[0],
      app->box[1],
      app->mixer,
      app->convert,
      app->sink,
      NULL);

  // link them together
  link_ok = gst_element_link_filtered(app->src[0], app->crop[0], app->vcaps[0]);
  gst_caps_unref(app->vcaps[0]);
  if (link_ok != TRUE) {
    g_printerr("Cannot link src and crop\n");
    return;
  }

  link_ok = gst_element_link_filtered(app->src[1], app->crop[1], app->vcaps[1]);
  gst_caps_unref(app->vcaps[1]);
  if (link_ok != TRUE) {
    g_printerr("Cannot link src and crop\n");
    return;
  }

  link_ok = gst_element_link_many(app->crop[0], app->distort[0], app->box[0], NULL);
  if (!link_ok) {
    g_printerr("Cannot link crop, distort, box\n");
    return;
  }

  link_ok = gst_element_link_many(app->crop[1], app->distort[1], app->box[1], NULL);
  if (!link_ok) {
    g_printerr("Cannot link crop, distort, box\n");
    return;
  }

  link_ok = gst_element_link_many(app->mixer, app->convert, app->sink, NULL);
  if (!link_ok) {
    g_printerr("Cannot link mixer, convert, sink\n");
    return;
  }

  g_signal_connect(app->box, "pad-added", G_CALLBACK(pad_added_mix_handler), &app);
  state_ret = gst_element_set_state((GstElement *)app->pipeline, GST_STATE_PLAYING);
  g_warning("set state returned %d\n", state_ret);

  // here is the loop
  app->loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(app->loop);

  state_ret = gst_element_set_state((GstElement *)app->pipeline, GST_STATE_NULL);
  g_warning("set state null returned %d\n", state_ret);

  return;
}

int main(int argc, char *argv[]) {
  printf("opening video source...\n");
  start_video_buffer("/dev/video1", "/dev/video2");
  printf("done with program\n");
  return 0;
}
