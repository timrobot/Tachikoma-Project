#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include "httplink.h"

// TODO: Make work with function handlers

static char msgbuf[1024];
static char response[1024];

/** Connect to the main server for our robot's manual interface.
 *  @param connection
 *    the connection information for the server
 *  @param hostname
 *    the hostname of the server
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
 *  @param addr
 *    the addr to send the request to, "/" for main page
 *  @param type
 *    either "GET" or "POST" or "get" or "post"
 *  @param data
 *    the data to send over (only for httplink POST)
 *  @return n bytes sent over, -1 otherwise
 */
int httplink_send(httplink_t *connection, char *addr, char *type, char *data) {
  const char *typestr;
  if (strlen(data) > 512) {
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
    sprintf(msgbuf, "%s %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Content-Type: application/x-www-form-urlencoded\r\n\r\n"
        "Content-Length: %zd\r\n"
        "%s\r\n\r\n",
        typestr, addr, connection->hostname, strlen(data), data);
  } else {
    sprintf(msgbuf, "%s %s HTTP/1.1\r\n"
        "Host: %s\r\n\r\n",
        typestr, addr, connection->hostname);
  }
  return send(connection->socket_fd, msgbuf, strlen(msgbuf), 0);
}

/** Try and receive a response from the main server
 *  @param connection
 *    the connection information for the server
 *  @return a message if it is received, otherwise NULL
 */
char *httplink_recv(httplink_t *connection) {
  int n;
  n = recv(connection->socket_fd, response, sizeof(response) - sizeof(char), 0);
  response[n] = '\0';
  if (n == -1) {
    return NULL;
  }
  return strstr(response, "\r\n\r\n") + sizeof(char) * 4;
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
