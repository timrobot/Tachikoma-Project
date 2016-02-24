#ifndef __TK_UDPLINK_H__
#define __TK_UDPLINK_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  void *source;
  void *queue;
  void *convert;
  void *encode;
  void *sink;
} udpout_t;

typedef struct {
  void *source;
  void *queue;
  void *decode;
  void *sink;
} udpin_t;

#ifdef __cplusplus
}
#endif

#endif
