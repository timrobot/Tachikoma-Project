#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include "httplink.h"

// TODO: Make work with function handlers
//
// Difference between terminology
// - hostname: eg. google.com (could also be 127.0.0.1)
// - addr: eg. 127.0.0.1
// - endpt: eg. /endpt
// - port: eg. 8080

#define BUFSIZE 2048
static char msgbuf[BUFSIZE];
static char response[BUFSIZE];
static char innerbuf[BUFSIZE];

/** Connect to the main server for our robot's manual interface.
 *  @param connection
 *    the connection information for the server
 *  @param hostname
 *    the hostname of the server
 *  @param port
 *    optional port to provide
 *  @return 0 on success, -1 otherwise
 */
int httplink_connect(httplink_t *connection, char *hostname, int port) {
  struct hostent *he;
  struct in_addr **addr_list;
  struct sockaddr_in main_server;
  int i;
  connection->hostname = hostname;

  // url -> ip
  if ((he = gethostbyname(connection->hostname)) == NULL) {
    return -1;
  }
  addr_list = (struct in_addr **)he->h_addr_list;
  for (i = 0; addr_list[i] != NULL; i++) {
    strcpy(connection->ipaddr, inet_ntoa(*addr_list[i]));
  }

  // open socket connection
  if ((connection->socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    return -1;
  }
  if (port == -1) {
    port = 80; // default port for http
  }
  main_server.sin_addr.s_addr = inet_addr(connection->ipaddr);
  main_server.sin_family = AF_INET;
  main_server.sin_port = htons(port); // http protocol
  if (connect(connection->socket_fd,
      (struct sockaddr *)&main_server,
      sizeof(main_server)) == -1) {
    return -1;
  }
  connection->connected = 1;
  return 0;
}

/** Send a REST request to the main server
 *  @param connection
 *    the connection information for the server
 *  @param endpt
 *    the endpoint to send the request to, "/" for main page
 *  @param type
 *    either "GET" or "POST" or "get" or "post"
 *  @param data
 *    the data to send over (only for httplink POST)
 *  @param content_type
 *    the type of data such as "application/x-www-form-urlencoded" or
 *    "application/json"
 *  @return n bytes sent over, -1 otherwise
 */
int httplink_send(httplink_t *connection, char *endpt, char *type,
    char *data, char *content_type) {
  const char *typestr;
  if (data != NULL && strlen(data) > BUFSIZE / 2) {
    fprintf(stderr, "[httplink] Error: message is too long\n");
    return -1;
  }
  if (strcmp(type, "get") == 0) {
    typestr = "GET";
  } else if (strcmp(type, "post") == 0) {
    typestr = "POST";
  } else {
    typestr = type;
  }
  if (strcmp(typestr, "POST") == 0) {
    if (content_type == NULL) {
      content_type = (char *)"application/x-www-form-urlencoded";
    }
    sprintf(msgbuf, "%s %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %zd\r\n\r\n"
        "%s\r\n\r\n",
        typestr, endpt, connection->hostname, content_type, strlen(data), data);
  } else {
    sprintf(msgbuf, "%s %s HTTP/1.1\r\n"
        "Host: %s\r\n\r\n",
        typestr, endpt, connection->hostname);
  }
  return send(connection->socket_fd, msgbuf, strlen(msgbuf), MSG_NOSIGNAL);
}

/** Try and receive a response from the main server
 *  @param connection
 *    the connection information for the server
 *  @param recvbuf
 *    an optional receiving buffer
 *  @return a message if it is received, otherwise NULL
 */
char *httplink_recv(httplink_t *connection, char *recvbuf) {
  int n;
  if (recvbuf == NULL) {
    recvbuf = innerbuf;
  }
  n = recv(connection->socket_fd, innerbuf, sizeof(innerbuf) - sizeof(char), 0);
  if (n <= 0) {
    return NULL;
  }
  recvbuf[n] = '\0';
  return recvbuf;
}

/** Disconnect the connection
 *  @param connection
 *    the connection information for the server
 *  @return 0
 */
int httplink_disconnect(httplink_t *connection) {
  if (connection->connected) {
    close(connection->socket_fd);
    connection->socket_fd = -1;
    connection->connected = 0;
  }
  return 0;
}

/** Send a request, once received while blocking, send to callback
 *  @param connection
 *    the connection information for the server
 *  @param url
 *    the full url, including the hostname, port, and endpoint
 *  @param type
 *    either "GET" or "POST" or "get" or "post"
 *  @param data
 *    the data to send over (only for httplink POST)
 *  @param content_type
 *    the type of data such as "application/x-www-form-urlencoded" or
 *    "application/json" (only for httplink POST)
 *  @param recvbuf
 *    an optional receiving buffer
 *  @return http response
 */
httpresponse_t httplink_request(char *url, char *type,
    char *data, char *content_type, char *recvbuf) {
  httplink_t connection; // use an anonymous connection

  // connect to server
  char hostname[128], *startpt, *endpt;
  endpt = ((endpt = strstr(url, "://")) != NULL ? &endpt[3] : url);
  startpt = endpt; // remove the http://
  if (strstr(endpt, "/") != NULL) {
    int hostlen;
    endpt = strstr(endpt, "/");
    hostlen = ((unsigned long long)endpt - (unsigned long long)startpt) /
      sizeof(char);
    strncpy(hostname, startpt, hostlen);
    hostname[hostlen] = '\0';
  } else {
    endpt = (char *)"/";
    strcpy(hostname, startpt);
  }
  int port = -1;
  char *portptr = strstr(hostname, ":");
  if (portptr != NULL) {
    portptr[0] = '\0';
    port = atoi(&portptr[1]);
  }
  httplink_connect(&connection, hostname, port);

  // send message
  httplink_send(&connection, endpt, type, data, content_type);

  // receive message
  int i;
  int running_length = 0;
  if (recvbuf == NULL) {
    recvbuf = innerbuf;
  }
  response[0] = '\0';
  for (i = 0; i < 100; i++) {
    size_t content_length;
    char *bufstart, *bufend, lenbuf[16];
    int n = recv(connection.socket_fd, &response[running_length],
        sizeof(response) - (running_length + 1) * sizeof(char), 0);
    if (n > 0) {
      running_length += n;
      response[running_length] = '\0';
      i = 0;
    } else {
      continue;
    }

    // grab the content length
    if ((bufstart = strstr(response, "Content-Length: ")) == NULL) {
      continue;
    }
    bufstart = &bufstart[strlen("Content-Length: ")];
    if ((bufend = strstr(bufstart, "\r\n")) == NULL) {
      continue;
    }
    n = ((unsigned long long)bufend - (unsigned long long)bufstart) /
      sizeof(char);
    strncpy(lenbuf, bufstart, n);
    lenbuf[n] = '\0';
    content_length = atoi(lenbuf);

    // grab message start
    if ((bufstart = strstr(bufend, "\r\n\r\n")) == NULL) {
      continue;
    }
    bufstart = &bufstart[4];
    if (strlen(bufstart) < content_length) {
      continue;
    }
    strcpy(recvbuf, bufstart);
    break;
  }

  // disconnect
  httplink_disconnect(&connection);

  httpresponse_t res;
  if (i < 100) {
    char header[16];
    sscanf(response, "%s %d ", header, &res.statusCode);
    res.message = recvbuf;
  } else {
    res.statusCode = 504;
    res.message = NULL;
  }
  return res;
}
