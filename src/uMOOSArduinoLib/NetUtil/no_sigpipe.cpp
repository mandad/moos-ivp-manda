/* no_sigpipe.c - ignore sigpipe signals -- handle the error synchronously */

#define _POSIX_C_SOURCE 200809L
#define _ISOC99_SOURCE
#define _XOPEN_SOURCE 700
#define __EXTENSIONS__

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "no_sigpipe.h"

void
no_sigpipe(void)
{
	struct sigaction action;


	action.sa_handler = SIG_IGN;
	sigemptyset(&action.sa_mask);
	action.sa_flags = 0;
	if (sigaction(SIGPIPE, &action, NULL)  < 0) {
		perror("sigaction SIGPIPE");
		exit(EXIT_FAILURE);
	}
}	/* no_sigpipe */

/* vi: set autoindent tabstop=8 shiftwidth=8 : */
