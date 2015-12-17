#ifndef __TK_TCPSERVE_H__
#define __TK_TCPSERVE_H__

typedef struct {
  void *src;
  void *cvt;
  void *enc;
  void *mux;
  void *sink;
  void *caps;
  void *pipeline;
} rtsplink_t;

#endif
