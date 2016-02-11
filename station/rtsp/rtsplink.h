#ifndef __TK_RTSPLINK_H__
#define __TK_RTSPLINK_H__

typedef struct {
  void *src;
  void *enc;
  void *mux;
  void *sink;
  void *caps;
  void *pipeline;
} rtsplink_t;

#endif
