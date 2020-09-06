#ifndef __HTTPLINK_H__
#define __HTTPLINK_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct httplink {
  char *hostname;
  char ipaddr[128];
  int socket_fd;
  int connected;
} httplink_t;

typedef struct httpresponse {
  int statusCode;
  char *message;
} httpresponse_t;

int httplink_connect(httplink_t *connection, char *hostname, int port);
int httplink_send(httplink_t *connection, char *endpt, char *type,
    char *data, char *content_type);
char *httplink_recv(httplink_t *connection, char *recvbuf);
httpresponse_t httplink_request(char *endpt, char *type,
    char *data, char *content_type, char *recvbuf);
/*
 * void httplink_request_async(char *endpt, char *type,
 *   char *data, char *content_type, void (*callbackFn)(char *), char *recvbuf,
 *   int timeout);
 */
int httplink_disconnect(httplink_t *connection);

#define httplink_get(url) \
  httplink_request(url, "GET", NULL, NULL, NULL)

#define httplink_post(url, data, content_type) \
  httplink_request(url, "POST", data, content_type, NULL)

#ifdef __cplusplus
}
#endif

#endif
