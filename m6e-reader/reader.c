#include "reader.h"

void handle_pipe(int sig)
{

}

void *interrupt()
{
  	struct sigaction action;
  	action.sa_handler = handle_pipe;
  	sigemptyset(&action.sa_mask);
  	action.sa_flags = 0;
  	sigaction(SIGPIPE, &action, NULL);
	return NULL;
}

int main(int argc, char **argv)
{
	int ret = -1;
	interrupt();
  	sys_config_init();  
  	sys_config_load(0);
	
	mid_task_init();	
	mid_timer_init();

	mid_connect();

//	telnetd_init(1);
	shell_play_init();

//	gpio_init();

	server_start();

	return 0;
}
