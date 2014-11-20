/* Mike Bogochow
 * Modified server_agent and listener functions.
 */

/* server2tcp.c - a server program that uses the socket interface to tcp
 *		  This server serves multiple clients at a time.
 *		  For each client it creates an agent thread that
 *		  repeatedly reads the name of a file
 *		  then the data content of a file and simply prints it.
 *		must be linked with -lnsl and -lsocket on solaris
 */

#define _POSIX_C_SOURCE 200809L
#define _ISOC99_SOURCE
#define _XOPEN_SOURCE 700
#define __EXTENSIONS__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "server.h"
#include "boyer_moore.h"
#include "plcs.h"
#include "plcs_net.h"
#include "print.h"
#include "no_sigpipe.h"
#include "tcpblockio.h"
#include "print_time.h"

#define BUFSIZE		1024			/* size of data buffers */
#define TEXT_SIZE	 256			/* size of text buffer */

struct thread_params {
	int		client_fd;
	struct sockaddr	client;
};

/* the agent is now a separate thread */
void * server_agent(void *params)
{
  struct thread_params *tparams;
  unsigned long tid;
  char *search_string = NULL;
  char *file_name = NULL;
  struct options *options = NULL;
  struct options cmd_options;
  struct dir_entry *head = NULL;
  int shift_table[SHIFT_TABLE_LEN];
  struct stats stats = STATS_INITIALIZER;
  struct net net = NET_INITIALIZER;
  struct our_header *header;
  time_type start_time;
  time_type end_time;
  double rate;
  double dtime;

  tparams = (struct thread_params *)params;
  net.fd = tparams->client_fd;
  tid = (unsigned long)pthread_self();

  get_time(&start_time);
  saprint(tparams->client_fd, tid, "starting");

  for (header = malloc(sizeof *header);;
          free(header), header = malloc(sizeof *header))
  { // Loop for receiving messages from client
    readblock(tparams->client_fd, (void *)header, sizeof(struct our_header));
    ntohheader(header);

    if (header->message_type == 1)
    { // Receive options from client
      if (header->var_length != sizeof(struct options))
      {
        fprintf(stderr, "Invalid size (%i) for message type 1 from client on "
            "fd %i\n", header->var_length, tparams->client_fd);
        continue;
      }

      options = malloc(header->var_length);
      readblock(tparams->client_fd, (void *)options, header->var_length);
      ntohopt(options);
      saprint(tparams->client_fd, tid, "client options");
      print_options(options);
    }

    else if (header->message_type == 2)
    { // Receive search string from client
      if (options == NULL)
      {
        saprint(tparams->client_fd, tid, "Invalid client message order.  "
            "Must receive message 1 before message 2\n");
        // return?
      }

      search_string = malloc((header->var_length + 1) * sizeof(char));
      readblock(tparams->client_fd, (void *)search_string, header->var_length);
      search_string[header->var_length] = '\0';

      get_shift_table(shift_table, search_string, strlen(search_string));
    }

    else if (header->message_type == 3)
    { // Receive file name from client
      if (options == NULL || search_string == NULL)
      {
        saprint(tparams->client_fd, tid, "Invalid client message order.  "
            "Must receive both messages 1 and 2 before message 3\n");
        // return?
      }

      file_name = malloc((header->var_length + 1) * sizeof(char));
      readblock(tparams->client_fd, (void *)file_name, header->var_length);
      file_name[header->var_length] = '\0';

      free(header);
      break; // done reading when get message 3
    }

    else
    {
      fprintf(stderr, "Invalid message type (%i) received from client %i on "
          "server agent %lu\n", header->message_type, tparams->client_fd, tid);
    }
  }

  // Do searching of file
  set_default_options(options);
  cmd_options = *options;
  cmd_options.flags &= ~QUIET;
  cmd_options.flags &= ~FOLLOW_SYM;

  // TODO: check the return values from file_mode and if -1, send msg_6 to client to terminate
  file_mode(file_name, search_string, shift_table, cmd_options,
      *options, head, &stats, &net);

  // Wait until all searching threads have terminated
  pthread_mutex_lock(&net.mut);
  while (net.thread_count > 0) // Wait for search to complete
    pthread_cond_wait(&net.done, &net.mut);
  pthread_mutex_unlock(&net.mut);

  // Calculate time and rate
  get_time(&end_time);
  dtime = time_diff(&start_time, &end_time);
  if (dtime == 0)
    rate = 0;
  else
    rate = (double)((double)stats.byte_read / dtime) / (double)1000000;

  // Print stats and send to client
  print_stats(stderr, &stats, dtime, rate);
  send_msg_6(tparams->client_fd, &stats);

  saprint(tparams->client_fd, tid, "terminating agent thread");

  // Send stats to client
  if (search_string != NULL)
    free(search_string);

  if (file_name != NULL)
    free(file_name);

  if (options != NULL)
    free(options);

  free(tparams);

  return (void *)0;
}	/* server_agent */

void listener(char *server_port, char *interface_name, int argc, char *argv[])
{
  socklen_t len;
  int listening_fd, errcode;
  struct sockaddr server;
  struct sockaddr_in *iptr;
  char text_buf[TEXT_SIZE];
  struct thread_params *params;
  pthread_t agent_id;
  char client_addrv4[IPv4_MAX];
  int client_port;

	if (argc == 0 || argv == NULL) {} //random usage to get rid of unused warning

	no_sigpipe();
	/* establish a server "listening post" */
	listening_fd = openlistener(server_port, interface_name, &server);
	if (listening_fd < 0)
		exit(EXIT_FAILURE);

	/* we are now successfully established as a server */
	/* infinite loop -- server serves forever, must be killed by ^C */
	for (; ;) {
		iptr = (struct sockaddr_in *)&server;
		if (inet_ntop(iptr->sin_family, &iptr->sin_addr, text_buf,
			TEXT_SIZE) == NULL) {
			perror("inet_ntop server");
			break;
		}
		pthread_mutex_lock(&print_mut);
		fprintf(stderr, "\nrplcsd listening at IP address %s port %d\n\n",
			text_buf, ntohs(iptr->sin_port));
		pthread_mutex_unlock(&print_mut);
		/* accept a client connection (block until one arrives) */
		params = malloc(sizeof(struct thread_params));
		if (params == NULL) {
			perror("listener malloc");
			break;
		}
		len = sizeof(params->client);
		if ((params->client_fd
			= accept(listening_fd, &params->client, &len)) < 0) {
			perror("rplcsd accept");
			break;
		}

		/* we are now successfully connected to a remote client */
		iptr = (struct sockaddr_in *)&(params->client);
    if (inet_ntop(iptr->sin_family, &iptr->sin_addr, client_addrv4, IPv4_MAX) == NULL)
    {
      perror("inet_ntop client");
      break;
    }
    client_port = ntohs(iptr->sin_port);
    fprintf(stderr, "connected to client %s:%i on fd %i\n", client_addrv4,
        client_port, params->client_fd);

		errcode = pthread_create(&agent_id, NULL, server_agent, params);
    if (errcode != 0)
    {
      fprintf(stderr, "pthread_create server agent: %s\n", strerror(errcode));
      break;
    }

    else
    { // thread successfully created, detach it
      errcode = pthread_detach(agent_id);
      if (errcode != 0)
      {
        fprintf(stderr, "pthread_detach server agent: %s\n", strerror(errcode));
        break;
      }
    }
	}	/* infinite loop */

	if (close(listening_fd) < 0) {
		perror("rplcsd close");
	}
	pthread_exit(NULL);
}	/* listener */
