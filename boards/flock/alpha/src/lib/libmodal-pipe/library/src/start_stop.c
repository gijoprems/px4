/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/


#define _GNU_SOURCE

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h> // for system()
#include <unistd.h> // for access()
#include <errno.h>
#include <limits.h> // for PATH_MAX
#include <fcntl.h>
#include <pthread.h>
#include <syscall.h>

#include <modal_start_stop.h>
#include <modal_pipe_common.h>

#include "misc.h"


// this is an extern variable!!
volatile int main_running=0; // main() while loop should check this for shutdown

static int name_to_pid_file(const char* name, char* path)
{
    if(strlen(name)<=0){
        fprintf(stderr, "ERROR process name for PID file must be >=1 character long\n");
        return -1;
    }
    sprintf(path, "/run/%s.pid", name);
    return 0;
}

int make_pid_file(const char* name)
{
    // construct the full file path from desired name
    char path[PATH_MAX];
    if(name_to_pid_file(name, path)) return -1;

    // start by checking if a pid file exists
    if(access(path, F_OK ) == 0){
        fprintf(stderr,"ERROR: in make_pid_file, file already exists, a new one was not written\n");
        fprintf(stderr,"You have either called this function twice, or you need to \n");
        fprintf(stderr,"call kill_existing_process() BEFORE make_pid_file()\n");
        return 1;
    }

    // open new file for writing
    FILE* fd = fopen(path, "w");
    if(fd == NULL){
        perror("ERROR in make_pid_file");
        return -1;
    }
    pid_t current_pid = getpid();
    fprintf(fd,"%d",(int)current_pid);
    fflush(fd);
    fclose(fd);
    return 0;
}


int kill_existing_process(const char* name, float timeout_s)
{
    int old_pid, i, ret, num_checks;

    // sanity checks
    if(timeout_s<0.1f){
        fprintf(stderr, "ERROR in kill_existing_process, timeout_s must be >= 0.1f\n");
        return -4;
    }

    // construct the full file path from desired name
    char path[PATH_MAX];
    if(name_to_pid_file(name, path)) return -1;

    // start by checking if a pid file exists
    if(access(path, F_OK)){
        // PID file missing, nothing is running
        return 0;
    }
    if(access(path, W_OK)){
        fprintf(stderr, "ERROR, in kill_existing_process, don't have write access \n");
        fprintf(stderr, "to PID file. Existing process is probably running as root.\n");
        fprintf(stderr, "Try running 'sudo kill'\n");
        return -3;
    }
    // attempt to open PID file if it fails something very wrong with it
    FILE* fd = fopen(path, "r");
    if(fd==NULL){
        fprintf(stderr, "WARNING, in kill_existing_process, PID file exists but is not\n");
        fprintf(stderr, "readable. Attempting to delete it.\n");
        remove(path);
        return -2;
    }
    // try to read the current process ID
    ret=fscanf(fd,"%d", &old_pid);
    fclose(fd);
    if(ret!=1){
        // invalid contents, just delete pid file
        fprintf(stderr, "WARNING, in kill_existing_process, PID file exists but contains\n");
        fprintf(stderr, "invalid contents. Attempting to delete it.\n");
        remove(path);
        return -2;
    }

    // if the file didn't contain a PID number, remove it and
    // return -2 indicating weird behavior
    if(old_pid == 0){
        fprintf(stderr, "WARNING, in kill_existing_process, PID file exists but contains\n");
        fprintf(stderr, "invalid contents. Attempting to delete it.\n");
        remove(path);
        return -2;
    }

    // check if it's our own pid, if so return 0
    if(old_pid == (int)getpid()) return 0;

    // now see if the process for the read pid is still running
    if(getpgid(old_pid) < 0){
        // process not running, remove the pid file
        remove(path);
        return 0;
    }

    printf("existing instance of %s found, attempting to stop it\n", name);

    // process must be running, attempt a clean shutdown
    if(kill((pid_t)old_pid, SIGINT)==-1){
        if(errno==EPERM){
            fprintf(stderr, "ERROR in kill_existing_process, insufficient permissions to stop\n");
            fprintf(stderr, "an existing process which is probably running as root.\n");
            fprintf(stderr, "Try running 'sudo kill' to stop it.\n\n");
            return -3;
        }
        remove(path);
        return -2;
    }

    // check every 0.1 seconds to see if it closed
    num_checks=timeout_s/0.1f;
    for(i=0; i<=num_checks; i++){
        // check if PID has stopped
        if(getpgid(old_pid)==-1){
            // succcess, it shut down properly
            remove(path);
            return 1;
        }
        else usleep(100000);
    }

    // otherwise force kill the program if the PID file never got cleaned up
    kill((pid_t)old_pid, SIGKILL);
    for(i=0; i<=num_checks; i++){
        // check if PID has stopped
        if(getpgid(old_pid)==-1){
            // succcess, it shut down properly
            remove(path);
            return 1;
        }
        else usleep(100000);
    }

    // delete the old PID file if it was left over
    remove(path);
    // return -1 indicating the program had to be killed
    fprintf(stderr, "WARNING in kill_existing_process, process failed to\n");
    fprintf(stderr, "close cleanly and had to be killed.\n");
    return -1;
}


int remove_pid_file(const char* name)
{
    // construct the full file path from desired name
    char path[PATH_MAX];
    if(name_to_pid_file(name, path)) return -1;

    // if PID file exists, remove it
    if(access(path, F_OK ) == 0) return remove(path);
    return 0;
}


// // Obtain a backtrace and print it to stdout.
// // this only traces within the shared lib, so only really useful if you copy
// // into your own binary
// static void print_trace()
// {
//  void *array[10];
//  char **strings;
//  int size, i;

//  size = backtrace (array, 10);
//  strings = backtrace_symbols (array, size);
//  if(strings != NULL){
//      printf ("Obtained %d stack frames.\n", size);
//      for (i = 0; i < size; i++){
//          printf ("\t%s\n", strings[i]);
//      }
//  }

//  free (strings);
//  return;
// }

static void segfault_handler(__attribute__ ((unused)) int signum, __attribute__ ((unused)) siginfo_t *info, __attribute__ ((unused)) void *context)
{
    char buf[16];
    pthread_getname_np(pthread_self(), buf, 16);

    fprintf(stderr, "\nSegmentation fault:\n");
    fprintf(stderr, "Fault thread: %s(tid: %lu)\n", buf, syscall(SYS_gettid));
    fprintf(stderr, "Fault address: %p\n", info->si_addr);

    switch (info->si_code){
    case SEGV_MAPERR:
        fprintf(stderr, "Address not mapped.\n");
        break;
    case SEGV_ACCERR:
        fprintf(stderr, "Access to this address is not allowed.\n");
        break;
    default:
        fprintf(stderr, "Unknown reason.\n");
        break;
    }

    main_running=0;

    // reset the signal handler to prevent infinite loop, this shouldn't be,
    // necessary with the SA_RESETHAND flag but is on some platforms
    struct sigaction action;
    action.sa_sigaction = NULL;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_SIGINFO | SA_RESETHAND;
    action.sa_handler = NULL;
    if(sigaction(SIGSEGV, &action, NULL) < 0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return;
    }

    return;
}


static void shutdown_signal_handler(int signo)
{
    switch(signo){
    case SIGINT: // normal ctrl-c shutdown interrupt
        main_running=0;
        fprintf(stderr, "\nreceived SIGINT Ctrl-C\n");
        break;
    case SIGTERM: // catchable terminate signal
        main_running=0;
        fprintf(stderr, "\nreceived SIGTERM\n");
        break;
    case SIGHUP:
        // terminal closed or disconnected, carry on anyway
        fprintf(stderr, "\nreceived SIGHUP, continuing anyway\n");
        break;
    default:
        fprintf(stderr, "\nreceived signal %d\n", signo);
        break;
    }
    return;
}


int enable_signal_handler(void)
{
    // make the sigaction struct for shutdown signals
    // sa_handler and sa_sigaction is a union, only set one
    struct sigaction action;
    action.sa_handler = shutdown_signal_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;

    // set actions
    if(sigaction(SIGINT, &action, NULL) < 0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    if(sigaction(SIGTERM, &action, NULL) < 0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    if(sigaction(SIGHUP, &action, NULL) < 0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }

    // different handler for segfaults
    // here we want SIGINFO too so we use sa_sigaction intead of sa_handler
    // also use RESETHAND to stop infinite loops (doesn't work on all platforms)
    struct sigaction seg_action;
    sigemptyset(&seg_action.sa_mask);
    seg_action.sa_flags = SA_SIGINFO | SA_RESETHAND;
    seg_action.sa_sigaction = segfault_handler;

    // set action
    if(sigaction(SIGSEGV, &seg_action, NULL) < 0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    return 0;
}


int disable_signal_handler(void)
{
    // reset all to defaults
    struct sigaction action;
    action.sa_handler = SIG_DFL;

    if(sigaction(SIGINT, &action, NULL)<0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    if(sigaction(SIGTERM, &action, NULL)<0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    if(sigaction(SIGABRT, &action, NULL) < 0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    if(sigaction(SIGHUP, &action, NULL)<0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    if(sigaction(SIGSEGV, &action, NULL)<0){
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    return 0;
}


int pipe_set_process_priority(int priority)
{
    struct sched_param param;
    param.sched_priority = priority;
    int policy;

    // priority 0 means linux default
    if(priority==0){
        policy = SCHED_OTHER;
    }
    // otherwise set RT FIFO scheduler
    else{
        policy = SCHED_FIFO;
        const int max_pri = sched_get_priority_max(SCHED_FIFO);
        const int min_pri = sched_get_priority_min(SCHED_FIFO);
        if(priority>max_pri || priority<min_pri){
            fprintf(stderr,"ERROR in %s, priority must be between %d & %d\n", __FUNCTION__, min_pri, max_pri);
            return -1;
        }
    }

    // set policy and priority for calling process ID
    int ret = sched_setscheduler(0, policy, &param);
    if(ret==-1){
        fprintf(stderr, "WARNING Failed to set priority, errno = %d\n", errno);
        fprintf(stderr, "This may be because the FIFO scheduler is not available when running in a console.\n");
        fprintf(stderr, "It should work properly when run as a systemd background process on boot.\n");
    }
    // check
    ret = sched_getscheduler(0);
    if(ret!=policy){
        fprintf(stderr, "WARNING: failed to set scheduler\n");
        return -1;
    }

    return 0;
}


// set thread to 0 to act on the calling pthread
int pipe_pthread_print_properties(pthread_t thread)
{
    int policy;
    struct sched_param param;
    if(thread==0) thread=pthread_self();

    // get parameters from pthread_t
    if(pthread_getschedparam(thread, &policy, &param)){
        perror("ERROR: pthread_getschedparam");
        return -1;
    }

    printf("policy=%s, priority=%d\n",
        (policy == SCHED_FIFO)  ? "SCHED_FIFO" :
        (policy == SCHED_RR)    ? "SCHED_RR" :
        (policy == SCHED_OTHER) ? "SCHED_OTHER" :
        "???",
        param.sched_priority);
    return 0;
}

// set thread to 0 to act on the calling pthread
int pipe_pthread_set_priority(pthread_t thread, int priority)
{
    struct sched_param param;
    param.sched_priority = priority;
    int policy;
    if(thread==0) thread=pthread_self();

    // priority 0 means linux default
    if(priority==0){
        policy = SCHED_OTHER;
    }
    // otherwise set RT FIFO scheduler
    else{
        policy = SCHED_FIFO;
        const int max_pri = sched_get_priority_max(SCHED_FIFO);
        const int min_pri = sched_get_priority_min(SCHED_FIFO);
        if(priority>max_pri || priority<min_pri){
            fprintf(stderr,"ERROR in %s, priority must be between %d & %d\n", __FUNCTION__, min_pri, max_pri);
            return -1;
        }
    }

    errno = pthread_setschedparam(thread, policy, &param);
    if(errno){
        perror("ERROR in pipe_pthread_set_priority");
        return -1;
    }
    return 0;
}


int pipe_pthread_create(pthread_t *thread, void*(*func)(void*), void* arg, int priority)
{
    pthread_attr_t pthread_attr;
    struct sched_param pthread_param;

    // sanity checks
    if(thread==NULL || func==NULL){
        fprintf(stderr,"ERROR in rc_pthread_create: received NULL pointer\n");
        return -1;
    }

    // apq8096 has a bug in glibc that doesn't let us set FIFO priorities in
    // pthreads when they are created sometimes when launching from a console.
    #ifdef PLATFORM_APQ8096
    priority = 0;
    #endif

    // necessary attribute initialization
    pthread_attr_init(&pthread_attr);

    // if user is requesting an RT priority, make sure we have permission to
    // do explicit scheduling
    if(priority!=0){
        // print warning if no permissions
        errno = pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
        if(errno){
            perror("ERROR: pthread_attr_setinheritsched: ");
            return -1;
        }

        // set scheduling policy
        const int max_pri = sched_get_priority_max(SCHED_FIFO);
        const int min_pri = sched_get_priority_min(SCHED_FIFO);
        if(priority>max_pri || priority<min_pri){
            fprintf(stderr,"ERROR in %s, priority must be between %d & %d\n", __FUNCTION__, min_pri, max_pri);
            return -1;
        }

        // set policy to attributes
        errno = pthread_attr_setschedpolicy(&pthread_attr, SCHED_FIFO);
        if(errno){
            perror("ERROR: pthread_attr_setschedpolicy");
                return -1;
        }

        // set priority in attributes
        pthread_param.sched_priority = priority;
        errno = pthread_attr_setschedparam(&pthread_attr, &pthread_param);
        if(errno){
            perror("ERROR: pthread_attr_setschedparam");
            return -1;
        }
    }

    // create the thread
    errno=pthread_create(thread, &pthread_attr, func, arg);
    pthread_attr_destroy(&pthread_attr);
    if(errno==EPERM){
        fprintf(stderr,"\nWARNING: in pipe_pthread_create setting scheduling policy\n");
        fprintf(stderr,"This will likely be fine when running as a background process\n\n");
        fflush(stderr);

        // try again
        memset(thread, 0, sizeof(pthread_t));
        errno=pthread_create(thread, NULL, func, arg);
        if(errno!=0){
            perror("ERROR: in pipe_pthread_create ");
            return -1;
        }
        fprintf(stderr, "starting thread with default priority succeeded\n");
        fflush(stderr);
    }
    if(errno){
        perror("ERROR: in pipe_pthread_create: ");
        return -1;
    }

    return 0;
}




// static void _check_and_set_affinity(pthread_t thread)
// {
//  cpu_set_t cpuset;
//  pthread_t thread;
//  if(thread==0){
//      thread = pthread_self();
//  }

//  /* Set affinity mask to include CPUs 7 only */
//  CPU_ZERO(&cpuset);
//  CPU_SET(7, &cpuset);
//  if(pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)){
//      perror("pthread_setaffinity_np");
//  }

//   Check the actual affinity mask assigned to the thread
//  if(pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpuset)){
//      perror("pthread_getaffinity_np");
//  }
//  printf("Camera processing thread is now locked to the following cores:");
//  for (int j = 0; j < CPU_SETSIZE; j++){
//      if(CPU_ISSET(j, &cpuset)) printf(" %d", j);
//  }
//  printf("\n");

//  // only do this once on start
//  has_set = 1;

//  return;
// }
