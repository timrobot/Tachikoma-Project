#ifndef __TK_TCPRECV_H__
#define __TK_TCPRECV_H__

typedef struct {
  void *src;
  void *demux;
  void *parse;
  void *dec;
  void *sink;
  void *caps;
  void *pipeline;
} tcprecv_t;

#endif
