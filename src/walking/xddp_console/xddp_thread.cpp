#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <iostream>

#include <unistd.h>
#include <termios.h>

// #include <definitions.h>

#if defined( __XENO__ ) || defined( __COBALT__ )
    static const std::string pipe_prefix("/proc/xenomai/registry/rtipc/xddp/");
#else
    static const std::string pipe_prefix("/tmp/");
#endif


// turn off canonical mode (and echo mode to suppress echoing).
char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

void * xddp_nrt_thread(void *)
{
    int xddp_sock, nbytes;
    char c;
    std::string line;


    std::string pipe_name("boards_console");
    std::string pipe(pipe_prefix + pipe_name);
    xddp_sock = open(pipe.c_str(), O_WRONLY);

    if (xddp_sock < 0 ) {
        printf ("error : %s\n", strerror (errno));
        exit(1);
    }

    std::cout << "Using "<< pipe << std::endl;

    while (1) {

#if 0
        std::getline(std::cin, line);
        //std::cout << line << std::endl;
        nbytes = write(xddp_sock, (void*)line.c_str(), line.length());
#else
        c = getch();
        std::cout << ">>" << c << std::endl;
        nbytes = write(xddp_sock, (void*)&c, sizeof(c));
#endif
        if (nbytes < 0) {
            perror("write");
            break;
        }
        sleep(0.2);
    }

    return 0;
}


int main(void) {

    struct sched_param param;
    int policy = SCHED_OTHER;

    param.sched_priority = sched_get_priority_max(policy);
    if(sched_setscheduler(0, policy, &param) == -1) {
            perror("sched_setscheduler failed");
            exit(-1);
    }

    xddp_nrt_thread(0);
    return 0;
}
