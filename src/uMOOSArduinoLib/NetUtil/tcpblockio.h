/* tcpblockio.h */

/* must be linked with -lnsl and -lsocket on solaris */

#ifndef _DEMO_TCPBLOCKIO_H
#define _DEMO_TCPBLOCKIO_H

#include <sys/socket.h>

int writeblock(int fd, void *buffer, int nbytes);

int readblock(int fd, void *buffer, int nbytes);

int openclient(char *server_port, char *server_node,
		struct sockaddr *server_addr,
		struct sockaddr *client_addr);

int openlistener(char *listen_port, char *listen_name,
		struct sockaddr *listen_address);

#endif
