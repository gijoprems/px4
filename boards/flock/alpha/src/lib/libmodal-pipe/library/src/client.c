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


#define _GNU_SOURCE // for pthread_timed_join, F_GETPIPE_SZ, and F_SETPIPE_SZ
#include <stdio.h>  // for fprintf
#include <unistd.h> // for read() & write()
#include <errno.h>  // to check error in read() and write()
#include <sys/ioctl.h>
#include <fcntl.h>  // for O_WRONLY & O_RDONLY
#include <string.h> // for strlen()
#include <stdlib.h> // for malloc
#include <pthread.h>
#include <signal.h> // for pthread_kill
#include <limits.h> // for PATH_MAX

#include <modal_pipe_client.h>
#include <modal_start_stop.h>
#include "misc.h"

// shorten these defines to improve code readability
#define N_CH        PIPE_CLIENT_MAX_CHANNELS
#define DIR_LEN     MODAL_PIPE_MAX_DIR_LEN
#define NAME_LEN    MODAL_PIPE_MAX_NAME_LEN
#define PATH_LEN    MODAL_PIPE_MAX_PATH_LEN

// sensible limit on number of duplicate names per client
#define MAX_NAMES   8

// struct to define the state of each channel
typedef struct client_channel_t{
    int                 running;            ///< flag to indicate if helper thread should shut down
    int                 claimed;            ///< flag to indicate that the channel has been claimed, even if it is not yet running
    int                 data_fd;            ///< this is the file descriptor to read data from
    int                 control_fd;         ///< one control pipe per channel, store fd here
    char*               buf;                ///< read buffers on heap
    int                 buf_len;            ///< length of read buffers
    char                pipe_dir[DIR_LEN];  ///< path to server's pipe directory
    char                name[NAME_LEN];     ///< client name
    char                req_path[PATH_LEN]; ///< path to request pipe
    char                data_path[PATH_LEN];///< path to data pipe
    int                 flags;              ///< copy of flags passed to client_init function
    pthread_t           helper_thread_id;   ///< only one helper pthread per channel
    int                 helper_priority;    ///< 0 for linux default priority, 1-99 for RT FIFO
    int                 helper_enabled;     ///< flag to indicate if we started a helper thread
    int                 helper_ready;       ///< flag that the helper thread sets once its running
    client_simple_cb*   simple_cb;          ///< callback for the simple helper
    client_camera_cb*   camera_cb;          ///< callback for the camera helper
    client_pc_cb*       point_cb;           ///< callback for the point cloud helper
    client_connect_cb*  connect_cb;         ///< optional callback set when server connects
    client_disc_cb*     disconnect_cb;      ///< optional callback set when server disconnects
    void*               cb_context_simple;  ///< context pointer for simple callback
    void*               cb_context_camera;  ///< context pointer for camera callback
    void*               cb_context_point;   ///< context pointer for point callback
    void*               cb_context_connect; ///< context pointer for connect callback
    void*               cb_context_dc;      ///< context pointer for disconnect
} client_channel_t;

// array of structs defining the state of each channel
static client_channel_t c[N_CH];
// set to 1 to enable debug prints
#define en_debug (c[ch].flags & CLIENT_FLAG_EN_DEBUG_PRINTS)
// each channel gets a mutex to protect during init or cleanup
static pthread_mutex_t mtx[N_CH];
// One mutex to protect the claimed field of all channels
static pthread_mutex_t claim_mtx;


// reset a channel to all 0's as if the program just started
// don't lock the mutex in here, the calling function should do that
static void _clean_channel(int ch)
{
    if(ch<0 || ch>=N_CH) return;

    // free the read buffer if allocated
    if(c[ch].buf != NULL){
        if(en_debug) printf("freeing buffer for client channel %d\n", ch);
        free(c[ch].buf);
        c[ch].buf = 0;
        c[ch].buf_len = 0;
    }

    // close and cleanup file descriptors
    if(en_debug) printf("closing FDs for client channel %d\n", ch);
    if(c[ch].data_fd){
        close(c[ch].data_fd);
        c[ch].data_fd = 0;
    }
    if(c[ch].control_fd){
        close(c[ch].control_fd);
        c[ch].control_fd = 0;
    }

    // zero out static stuff
    c[ch].running = 0;
    memset(c[ch].pipe_dir, 0, DIR_LEN);
    memset(c[ch].name, 0, NAME_LEN);
    memset(c[ch].data_path, 0, PATH_LEN);
    memset(c[ch].req_path, 0, PATH_LEN);
    c[ch].flags = 0;
    c[ch].helper_thread_id = 0;
    c[ch].helper_enabled = 0;

    // All that's left are callbacks which we should leave alone so the user
    // only needs to set them once without them getting wiped.
    return;
}


// dummy function to catch the USR1 signal
static void _sigusr_cb(__attribute__((unused)) int sig)
{
    return;
}


/**
 * @brief      read from a pipe with error checks
 *
 *             used by the helper thread to consense the code.
 *
 * @param[in]  ch             channel
 * @param      buf            The buffer
 * @param[in]  bytes_to_read  The bytes to read
 *
 * @return     -1 if helper should go back to beginning of the loop, otherwise
 *             returns the number of bytes read
 */
static int _read_helper(int ch, char* buf, int bytes_to_read)
{
    int bytes_read;

    if(c[ch].data_fd == 0){
        if(en_debug){
            fprintf(stderr, "channel %d helper tried to read from closed fd\n", ch);
        }
        return -1;
    }

    if(buf == NULL){
        fprintf(stderr, "ERROR channel %d helper tried to read into NULL buffer\n", ch);
        return -1;
    }

    // try a read
    if(en_debug){
        fprintf(stderr,"ch %2d trying to read %d bytes\n", ch, bytes_to_read);
    }

    errno = 0;
    bytes_read = read(c[ch].data_fd, buf, bytes_to_read);

    if(en_debug){
        printf("ch %2d read returned %d, errno: %d\n", ch, bytes_read, errno);
    }


    // quickly check if running changed while we were reading
    if(!c[ch].running){
        if(en_debug){
            printf("helper thread for channel %d stopping by request\n", ch);
        }
        return -1;
    }

    // TODO on VOXL1 errno used to be set when we returned from READ due to the
    // FIFO being deleted by ther server, but this doesn't seem to be the case
    // anymore so just check for no bytes read. Investigate why the behavior is
    // different and make sure this is still robust on APQ8096
    //if(bytes_read<=0 && errno){

    // do a little error handling
    if(bytes_read<=0){

        // helpful print showing read() actually told us
        if(en_debug){
            fprintf(stderr, "ch %2d server likely disconnected\n", ch);
            perror("errno=");
        }

        // server disconnected or some other unexpected error, if not in
        // auto-reconnect mode just exit the thread
        if(c[ch].flags & CLIENT_FLAG_DISABLE_AUTO_RECONNECT){
            c[ch].running = 0;
        }

        // close file descriptors to indicate the disconnect
        if(c[ch].data_fd!=0){
            close(c[ch].data_fd);
            c[ch].data_fd = 0;
        }
        if(c[ch].control_fd!=0){
            close(c[ch].control_fd);
            c[ch].control_fd = 0;
        }

        // inform client of the disconnect
        if(c[ch].disconnect_cb){
            c[ch].disconnect_cb(ch, c[ch].cb_context_dc);
        }
        return -1;
    }

    return bytes_read;
}


static int _check_cam_meta(int ch, camera_image_metadata_t meta, int* bytes_to_read)
{
    // indicate there is nothing to read untill we pass all of our checks
    *bytes_to_read = 0;

    // validate the packet magic number
    if(meta.magic_number != CAMERA_MAGIC_NUMBER){
        fprintf(stderr, "ERROR: invalid metadata, magic number=%d, expected %d\n", meta.magic_number, CAMERA_MAGIC_NUMBER);
        return -1;
    }

    // also check that the size makes sense since we are about to allocate
    // size_bytes on the heap and this could go wrong
    if(meta.size_bytes > (meta.width*meta.height*10)){
        fprintf(stderr, "ERROR: received unreasonably large camera frame size\n");
        return -1;
    }

    // allocate a buffer for the frame if it's not allocated already
    if(c[ch].buf == NULL){
        c[ch].buf_len = meta.size_bytes;
        c[ch].buf = malloc(meta.size_bytes);
        if(c[ch].buf==NULL){
            perror("ERROR: allocating memory for image buffer");
            return -1;
        }
    }

    // Realloc more memory if the frame size grew
    if(c[ch].buf_len<meta.size_bytes){
        c[ch].buf = realloc(c[ch].buf, meta.size_bytes);
        if(c[ch].buf==NULL){
            perror("ERROR: reallocating memory for image buffer");
            printf("requested frame buffer was %d bytes\n", meta.size_bytes);
            return -1;
        }
        c[ch].buf_len = meta.size_bytes;
    }

    // passed all checks, indicate there is data to read
    *bytes_to_read = meta.size_bytes;
    return 0;
}


static int _check_point_meta(int ch, point_cloud_metadata_t meta, int* bytes_to_read)
{
    // indicate there is nothing to read untill we pass all of our checks
    *bytes_to_read = 0;

    if(meta.magic_number != POINT_CLOUD_MAGIC_NUMBER){
        fprintf(stderr, "invalid metadata, magic number=%d, expected %d\n", meta.magic_number, POINT_CLOUD_MAGIC_NUMBER);
        return -1;
    }

    // next to allocate or reallocate the read buffer if necessary, see how many
    // bytes we need to fit in the point cloud
    int size_bytes = pipe_point_cloud_meta_to_size_bytes(meta);
    if(size_bytes<0){
        return -1;
    }

    // allocate a buffer if it's not allocated already
    if(c[ch].buf == NULL){
        c[ch].buf_len = size_bytes;
        c[ch].buf = malloc(size_bytes);
        if(c[ch].buf==NULL){
            perror("ERROR: allocating memory for point cloud");
            return -1;
        }
    }

    // Realloc more memory if the size grew
    if(c[ch].buf_len<size_bytes){
        c[ch].buf = realloc(c[ch].buf, size_bytes);
        if(c[ch].buf==NULL){
            perror("ERROR: reallocating memory for point cloud");
            printf("requested buffer was %d bytes\n", size_bytes);
            return -1;
        }
        c[ch].buf_len = size_bytes;
    }

    // passed all checks, indicate there is data to read
    *bytes_to_read = size_bytes;
    return 0;
}

static int _connect_to_server(int ch)
{
    // Check server is online by seeing if request pipe exists
    if(!_exists(c[ch].req_path)) return PIPE_ERROR_SERVER_NOT_AVAILABLE;

    // passed all the validity checks, lock the mutex so we can start
    pthread_mutex_lock(&mtx[ch]);

    // find next index open for that name. e/g/ client0, client1, client2...
    // up to MAX_NAMES
    char newpath[PATH_LEN];
    char newname[NAME_LEN+16];
    int  newname_len = 0;
    int i;
    for(i=0;i<MAX_NAMES;i++){
        // construct a new path with the current dir, name, and index i
        sprintf(newpath,"%s%s%d", c[ch].pipe_dir, c[ch].name, i);
        if(!_exists(newpath)){
            // name with this index doesn't exist yet, good, use it!
            newname_len = sprintf(newname, "%s%d", c[ch].name, i);
            strcpy(c[ch].data_path, newpath);
            break;
        }
    }

    // error if we quit the for loop without finding an open name
    if(newname_len == 0){
        pthread_mutex_unlock(&mtx[ch]);
        return PIPE_ERROR_REACHED_MAX_NAME_INDEX;
    }

    if(en_debug){
        printf("requesting name %s\n", newname);
        printf("with complete path %s\n", newpath);
    }

    // open request pipe for writing
    int request_pipe_fd = open(c[ch].req_path, O_WRONLY|O_NONBLOCK);
    if(request_pipe_fd <= 0){
        perror("ERROR in pipe_client_init_channel opening request pipe");
        if(errno==ENXIO){
            fprintf(stderr, "Most likely the server stopped without cleaning up\n");
            fprintf(stderr, "Client is cleaning up pipes for the server\n");
            _remove_recursive(c[ch].pipe_dir);
        }
        pthread_mutex_unlock(&mtx[ch]);
        return PIPE_ERROR_FILE_IO;
    }

    // send in our request with the desired name and close when done
    int bytes_written = write(request_pipe_fd, newname, newname_len+1);
    if(bytes_written != (newname_len+1)){
        perror("ERROR in pipe_client_init_channel writing to request pipe");
        pthread_mutex_unlock(&mtx[ch]);
        return PIPE_ERROR_FILE_IO;
    }
    close(request_pipe_fd); // don't need request pipe anymore

    // try to open the control pipe while we wait for server to service the
    // request. The control pipe is optional and may be missing which is fine.
    int dirlen = strlen(c[ch].pipe_dir);
    char control_path[dirlen+1+7];
    strcpy(control_path,c[ch].pipe_dir);
    strcat(control_path,"control");
    int ctrl_fd = open(control_path, O_WRONLY);
    if(ctrl_fd<=0){
        // no problem if control doesn't exist. server just never enabled it
        if(errno!=ENOENT){
            perror("ERROR in pipe_client_init_channel opening control pipe");
            pthread_mutex_unlock(&mtx[ch]);
            return PIPE_ERROR_FILE_IO;
        }
    }
    else{
        c[ch].control_fd = ctrl_fd;
    }

    // wait for new pipe to be created by the server
    // timeout after 1 second
    const int open_sleep_us = 1000;
    const int attempts = 500;
    for(i=0;i<attempts;i++){
        c[ch].data_fd = open(c[ch].data_path, O_RDONLY);
        if(c[ch].data_fd>0) break;
        usleep(open_sleep_us);
    }

    // check if we failed after several attempts
    if(c[ch].data_fd<=0){
        if(c[ch].control_fd) close(c[ch].control_fd);
        c[ch].control_fd = 0;
        c[ch].data_fd = 0;
        pthread_mutex_unlock(&mtx[ch]);
        return PIPE_ERROR_TIMEOUT;
    }

    // Now we have our own data pipe fd open!
    if(en_debug) printf("connected after %d attempt(s)\n", i+1);
    pthread_mutex_unlock(&mtx[ch]);

    // run the connect callback if set
    if(c[ch].connect_cb){
        c[ch].connect_cb(ch, c[ch].cb_context_connect);
    }

    return 0;
}

int pipe_client_flush(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(c[ch].data_fd==0){
        return PIPE_ERROR_NOT_CONNECTED;
    }

    int bytes = pipe_client_bytes_in_pipe(ch);
    if(bytes == 0) return 0;

    char *buf = (char *)malloc(bytes+1);
    if(buf == NULL){
        return PIPE_ERROR_OTHER;
    }

    read(c[ch].data_fd, buf, bytes);
    free(buf);
    return 0;
}


// function for simple helper thread
static void* _helper_func(void* context)
{
    int ch = (long)context;
    int bytes_read;
    int bytes_to_read;
    camera_image_metadata_t cam_meta;
    point_cloud_metadata_t point_meta;

    // flags indicating which helper mode we are in
    int is_simple_helper = c[ch].flags & CLIENT_FLAG_EN_SIMPLE_HELPER;
    int is_camera_helper = c[ch].flags & CLIENT_FLAG_EN_CAMERA_HELPER;
    int is_point_helper  = c[ch].flags & CLIENT_FLAG_EN_POINT_CLOUD_HELPER;

    // catch the SIGUSR1 signal which we use to quit the blocking read
    struct sigaction action;
    action.sa_handler = _sigusr_cb;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGUSR1, &action, NULL);

    c[ch].helper_ready = 1;
    if(en_debug) printf("starting helper thread for channel %d\n", ch);

    // simple helper needs a read buffer, the other helpers will figure out how
    // to allocate their buffers based on metadata later
    if(is_simple_helper && c[ch].buf == 0){
        c[ch].buf = malloc(c[ch].buf_len);
        if(c[ch].buf==NULL){
            perror("ERROR: allocating memory for simple helper");
            return NULL;
        }
    }

    // primary helper loop, manages reading the pipe with different checks and
    // behaviors depending on which helper mode we are in
    while(c[ch].running){

        // first check is a pipe open, we may need to connect/reconnect
        if(c[ch].data_fd==0 && !(c[ch].flags & CLIENT_FLAG_DISABLE_AUTO_RECONNECT)){
            int ret = _connect_to_server(ch);
            if(ret<0){
                if(en_debug){
                    pipe_print_error(ret);
                    printf("going to sleep, will try to reconnect shortly\n");
                }
                usleep(500000);
                continue;
            }
        }

        // first read from pipe. This is it for the simple helper, but other
        // helpers may make multiple reads after this.
        if(is_simple_helper){
            bytes_read = _read_helper(ch, c[ch].buf, c[ch].buf_len);
        }
        else if(is_camera_helper){
            bytes_read = _read_helper(ch, (char*)&cam_meta, sizeof(camera_image_metadata_t));
        }
        else if(is_point_helper){
            bytes_read = _read_helper(ch, (char*)&point_meta, sizeof(point_cloud_metadata_t));
        }

        // go back to beginning of the loop if the read helper says to do so
        if(bytes_read<=0) continue;

        // simple helper is done here, send callback if requested
        if(is_simple_helper){
            if(c[ch].simple_cb){
                c[ch].simple_cb(ch, c[ch].buf, bytes_read, c[ch].cb_context_simple);
            }
            continue;
        }

        // camera and point cloud helpers just read in metadata, so check it
        if(is_camera_helper && _check_cam_meta(ch, cam_meta, &bytes_to_read)<0){
            pipe_client_flush(ch);
            continue;
        }
        else if(is_point_helper && _check_point_meta(ch, point_meta, &bytes_to_read)<0){
            pipe_client_flush(ch);
            continue;
        }

        // now read the data, this may take multiple reads, for example stereo
        // frames may come in left, then right. YUV color images may come in as
        // Y first, followed by the UV bytes. This should only take 1 or 2 reads
        // if the server makes 1 or 2 writes. Try 10 times to be sure.
        int total_read = 0;
        int tries = 0;
        while(c[ch].running && tries<10 && total_read<bytes_to_read){
            // try a read just like before
            bytes_read = _read_helper(ch, c[ch].buf+total_read, bytes_to_read-total_read);
            // break on error, since we didn't read in the data this will send
            // us back to the beginning of the primary (outer) while loop.
            if(bytes_read<0) break;
            // keep track of bytes read and how many tries it's taken
            total_read += bytes_read;
            tries++;
        }

        // check if we failed to read enough bytes after 10 attempts
        if(total_read != bytes_to_read){
            fprintf(stderr, "ERROR: only read %d bytes of data, expected %d\n", total_read, bytes_to_read);
            continue;
        }

        // send callbacks if set!
        if(is_camera_helper && c[ch].camera_cb){
            c[ch].camera_cb(ch, cam_meta, c[ch].buf, c[ch].cb_context_camera);
        }
        else if(is_point_helper && c[ch].point_cb){
            c[ch].point_cb(ch, point_meta, (void*)c[ch].buf, c[ch].cb_context_point);
        }

    } // end of helper while loop!

    if(en_debug){
        printf("Exiting helper thread for channel %d\n", ch);
    }

    return NULL;
}

static void _safe_unclaim(int ch)
{
    pthread_mutex_lock(&claim_mtx);
    c[ch].claimed = 0;
    pthread_mutex_unlock(&claim_mtx);
    return;
}

int pipe_client_open(int ch, const char* name_or_location, const char* client_name, int flags, int buf_len)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }

    // claim the channel before we get to setting it up so nothing else will
    // claim it in the mean time with pipe_client_get_next_available_channel()
    pthread_mutex_lock(&claim_mtx);
    int old_claim = c[ch].claimed;
    c[ch].claimed = 1;
    pthread_mutex_unlock(&claim_mtx);

    // more sanity checks
    if(c[ch].running){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, channel %d already running\n", __FUNCTION__, ch);
        return PIPE_ERROR_OTHER;
    }
    // helper specific sanity checks
    if((flags & CLIENT_FLAG_EN_SIMPLE_HELPER) && (buf_len<1)){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, buffer length should be >0\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }
    if((flags & CLIENT_FLAG_EN_POINT_CLOUD_HELPER) && (buf_len<1)){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, buffer length should be >0\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }

    // make sure multiple helpers aren't enabled
    int n_helpers = 0;
    if(flags & CLIENT_FLAG_EN_SIMPLE_HELPER)        n_helpers++;
    if(flags & CLIENT_FLAG_EN_CAMERA_HELPER)        n_helpers++;
    if(flags & CLIENT_FLAG_EN_POINT_CLOUD_HELPER)   n_helpers++;
    if(n_helpers>1){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, can't enable multiple helpers\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }

    // if no helpers are set, auto-reconnect won't work. The user didn't need to
    // set this flag, but do it here anyway so we can check later if needed
    if(n_helpers<1){
        c[ch].helper_enabled = 0;
        flags |= CLIENT_FLAG_DISABLE_AUTO_RECONNECT;
    }
    else{
        c[ch].helper_enabled = 1;
    }

    // validity checking
    char dir[MODAL_PIPE_MAX_DIR_LEN];
    if(pipe_expand_location_string(name_or_location, dir)<0){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, invalid name or location: %s\n", __FUNCTION__, name_or_location);
        return PIPE_ERROR_INVALID_ARG;
    }

    // validate the client's name string
    int client_namelen = strlen(client_name);
    if(client_namelen<1){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, empty name string provided\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }
    if(client_namelen>=(NAME_LEN-1)){
        if(!old_claim) _safe_unclaim(ch);
        // check >= NAME_LEN-1 since we will append a digit to the end later
        fprintf(stderr, "ERROR in %s, name string is too long\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }
    if(strchr(client_name, '/')!=NULL){
        if(!old_claim) _safe_unclaim(ch);
        fprintf(stderr, "ERROR in %s, name string can't contain '/'\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }

    // passed our sanity checks, now start setting up the channel
    pthread_mutex_lock(&mtx[ch]);
    strcpy(c[ch].pipe_dir, dir);
    strcpy(c[ch].name, client_name);
    c[ch].flags = flags;

    // construct request path for later use
    strcpy(c[ch].req_path,dir);
    strcat(c[ch].req_path,"request");

    // simple helper uses user-specified buffer length. Other helpers may
    // realloc the buffer but will use the user-specified buffer size as a
    // starting point
    c[ch].buf_len = buf_len;
    pthread_mutex_unlock(&mtx[ch]);

    // if opening paused, there is nothing left to do since everything below
    // is just connecting to the server and starting helper threads.
    if(flags & CLIENT_FLAG_START_PAUSED){
        return 0;
    }

    // now "resume" the channel which will start the helper thread if enabled
    // or connect to server if no helper if enabled
    return pipe_client_resume(ch);
}


int pipe_client_resume(int ch)
{
    int ret = 0;

    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(c[ch].running){//Already running, nothing to resume
        return 0;
    }

    pthread_mutex_lock(&claim_mtx);
    if(!c[ch].claimed || c[ch].pipe_dir[0] == 0){
        pthread_mutex_unlock(&claim_mtx);
        fprintf(stderr, "ERROR in %s, attempted to resume channel %d which has not been set up\n", __FUNCTION__, ch);
        return PIPE_ERROR_OTHER;
    }
    pthread_mutex_unlock(&claim_mtx);

    // When not in auto-reconnect mode, try to connect to the server now and
    // return the error that caused the failure.
    // The helper thread will do that for us in auto-reconnect, mode.
    if(c[ch].flags & CLIENT_FLAG_DISABLE_AUTO_RECONNECT){
        ret = _connect_to_server(ch);
        if(ret<0) return ret;
    }

    // mark channel as running, switch to 0 to tell potential helper threads
    // that we should shut down. This also indicates the channel has been
    // initialized
    c[ch].running = 1;

    // start a helper if requested
    if(c[ch].helper_enabled){
        pthread_mutex_lock(&mtx[ch]);

        if(en_debug) printf("spawning helper pthread with thread priority %d\n", c[ch].helper_priority);
        ret = pipe_pthread_create(&c[ch].helper_thread_id, _helper_func, (void*)(long)ch, c[ch].helper_priority);

        pthread_mutex_unlock(&mtx[ch]);
        //1 ms sleep to give the helper thread a chance to initialize before we return
        usleep(1000);
    }

    return ret;
}


int pipe_client_get_next_available_channel()
{
    int ret = PIPE_ERROR_OTHER;

    pthread_mutex_lock(&claim_mtx);
    for(int ch = 0; ch < N_CH; ch++){
        if(!c[ch].claimed){
            c[ch].claimed = 1;
            ret = ch;
            break;
        }
    }
    pthread_mutex_unlock(&claim_mtx);

    return ret;
}


int pipe_client_get_info(int ch, pipe_info_t* info)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(!c[ch].data_fd){
        fprintf(stderr, "ERROR in %s, channel %d not initialized yet\n", __FUNCTION__, ch);
        return PIPE_ERROR_NOT_CONNECTED;
    }

    return pipe_get_info(c[ch].pipe_dir, info);
}


cJSON* pipe_client_get_info_json(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return NULL;
    }
    if(!c[ch].data_fd){
        fprintf(stderr, "ERROR in %s, channel %d not initialized yet\n", __FUNCTION__, ch);
        return NULL;
    }

    return pipe_get_info_json(c[ch].pipe_dir);
}


int pipe_client_bytes_in_pipe(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(!c[ch].data_fd){
        fprintf(stderr, "ERROR in %s, channel %d not initialized yet\n", __FUNCTION__, ch);
        return PIPE_ERROR_NOT_CONNECTED;
    }

    // lock mutex before working on the pipe fd
    pthread_mutex_lock(&mtx[ch]);

    // use ioctl to find bytes in the pipe
    int n_bytes;
    int ret = ioctl(c[ch].data_fd, FIONREAD, &n_bytes);
    if(ret){
        perror("ERROR in pipe_client_bytes_in_pipe");
        n_bytes = PIPE_ERROR_FILE_IO;
    }

    // done, unlock and return
    pthread_mutex_unlock(&mtx[ch]);
    return n_bytes;
}


int pipe_client_get_pipe_size(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(!c[ch].data_fd){
        fprintf(stderr, "ERROR in %s, channel %d not initialized yet\n", __FUNCTION__, ch);
        return PIPE_ERROR_NOT_CONNECTED;
    }

    pthread_mutex_lock(&mtx[ch]);
    int ret = fcntl(c[ch].data_fd, F_GETPIPE_SZ);
    pthread_mutex_unlock(&mtx[ch]);
    return ret;
}


int pipe_client_set_pipe_size(int ch, int size_bytes)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(!c[ch].data_fd){
        fprintf(stderr, "ERROR in %s, channel %d not initialized yet\n", __FUNCTION__, ch);
        return PIPE_ERROR_NOT_CONNECTED;
    }

    // use fctl with mutex protection
    pthread_mutex_lock(&mtx[ch]);
    errno = 0;
    int new_size = fcntl(c[ch].data_fd, F_SETPIPE_SZ, size_bytes);
    pthread_mutex_unlock(&mtx[ch]);

    // error check
    if(new_size<size_bytes){
        perror("ERROR failed to set pipe size");
        if(errno == EPERM){
            fprintf(stderr, "You may need to be root to make a pipe that big\n");
        }
        // if fcntl fails, it may return 0 instead of the actual size, so fetch
        // the new size and return that instead.
        return pipe_client_get_pipe_size(ch);
    }

    // if fcntl was successful it returned the new size in bytes, so return it
    return new_size;
}


int pipe_client_set_simple_helper_cb(int ch, client_simple_cb* cb, void* context)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].cb_context_simple = context;
    c[ch].simple_cb = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_client_set_camera_helper_cb(int ch, client_camera_cb* cb, void* context)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].cb_context_camera = context;
    c[ch].camera_cb = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_client_set_point_cloud_helper_cb(int ch, client_pc_cb* cb, void* context)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].cb_context_point = context;
    c[ch].point_cb = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_client_set_helper_thread_priority(int ch, int priority)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(priority<0 || priority>99){
        fprintf(stderr, "ERROR in %s, priority should be between 0 & 99\n", __FUNCTION__);
        return PIPE_ERROR_INVALID_ARG;
    }
    if(c[ch].running){
        fprintf(stderr, "ERROR in %s, call this before pipe_client_open()\n", __FUNCTION__);
        return PIPE_ERROR_OTHER;
    }

    pthread_mutex_lock(&mtx[ch]);
    c[ch].helper_priority = priority;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_client_set_connect_cb(int ch, client_connect_cb* cb, void* context)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].cb_context_connect = context;
    c[ch].connect_cb = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_client_set_disconnect_cb(int ch, client_disc_cb* cb, void* context)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    pthread_mutex_lock(&mtx[ch]);
    c[ch].cb_context_dc = context;
    c[ch].disconnect_cb = cb;
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_client_is_connected(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return 0;
    }

    // start assuming we are disconnected
    int ret = 0;

    // simple check if the pipe file descriptor is valid. If we disconnected
    // then the helper thread would have set it to 0
    pthread_mutex_lock(&mtx[ch]);
    if(c[ch].data_fd>0){
        ret=1;
    }
    pthread_mutex_unlock(&mtx[ch]);
    return ret;
}


int pipe_client_get_fd(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }

    // lock the mutex before poking at the data struct
    pthread_mutex_lock(&mtx[ch]);

    // return the fd or -1 if not initialized yet
    int ret;
    if(c[ch].data_fd>0){
        ret = c[ch].data_fd;
    }
    else{
        fprintf(stderr, "ERROR in %s, channel not initialized yet\n", __FUNCTION__);
        ret = -1;
    }

    // unlock and return
    pthread_mutex_unlock(&mtx[ch]);
    return ret;
}


int pipe_client_send_control_cmd(int ch, const void* cmd)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(c[ch].data_fd==0){
        return PIPE_ERROR_NOT_CONNECTED;
    }
    if(c[ch].control_fd==0){
        return PIPE_ERROR_CTRL_NOT_AVAILABLE;
    }
    pthread_mutex_lock(&mtx[ch]);
    int len = strlen(cmd)+1;
    if(write(c[ch].control_fd, cmd, len)!=len){
        perror("ERROR writing to control pipe");
        pthread_mutex_unlock(&mtx[ch]);
        return -1;
    }
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


int pipe_client_send_control_cmd_bytes(int ch, const void* data, int bytes)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
    if(c[ch].data_fd==0){
        return PIPE_ERROR_NOT_CONNECTED;
    }
    if(c[ch].control_fd==0){
        return PIPE_ERROR_CTRL_NOT_AVAILABLE;
    }
    // simple write to our open fd
    pthread_mutex_lock(&mtx[ch]);
    if(write(c[ch].control_fd, data, bytes)!=bytes){
        perror("ERROR writing to control pipe");
        pthread_mutex_unlock(&mtx[ch]);
        return -1;
    }
    pthread_mutex_unlock(&mtx[ch]);
    return 0;
}


// used by both pause and close
static int _stop_helper_and_remove_pipe(int ch, int in_own_thread)
{
    int ret = 0;

    // signal to stop running and disable callback
    c[ch].running = 0;

    // if helper thread is running, quit it
    if(c[ch].helper_enabled){

        // Make sure that the help thread has actually set up its signal handler
        int counter = 0;
        while(c[ch].helper_ready == 0){
            if(en_debug){
                printf("Waiting for helper thread to start in order to pause\n");
            }
            if(counter++ > 20){
                fprintf(stderr, "ERROR in %s, timeout waiting for helper to be ready\n", __FUNCTION__);
                ret = -1;
                break;
            }
            usleep(100000);
        }

        // send signal to thread, this is just to make the blocking read quit
        if(!in_own_thread){

            if(en_debug) printf("sending SIGUSR1 to stop blocking reads in helper%d\n", ch);

            pthread_kill(c[ch].helper_thread_id, SIGUSR1);

            if(en_debug) printf("trying to join helper thread channel %d\n", ch);

            if(ret==0){

            #ifdef __ANDROID__
                errno = pthread_join(c[ch].helper_thread_id, NULL);
            #else
                // do a timed join, 1 second timeout
                struct timespec thread_timeout;
                clock_gettime(CLOCK_REALTIME, &thread_timeout);
                thread_timeout.tv_sec += 1;
                errno = pthread_timedjoin_np(c[ch].helper_thread_id, NULL, &thread_timeout);
            #endif
                c[ch].helper_ready = 0;
                if(errno==ETIMEDOUT){
                    fprintf(stderr, "WARNING, %s timed out joining read thread\n", __FUNCTION__);
                }
            }
        }
    }

    // close the pipe to let the server know we are stopping the read
    if(c[ch].data_fd){
        close(c[ch].data_fd);
        c[ch].data_fd = 0;
    }

    // remove the data pipe from the file system. This indicates to any other
    // clients that the data pipe name is now free to use
    if(c[ch].data_path[0]!=0){
        if(en_debug) printf("deleting pipe: %s\n", c[ch].data_path);
        remove(c[ch].data_path);
    }

    return ret;
}


void pipe_client_pause(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return;
    }

    // nothing to do if not running
    if(c[ch].running==0) return;

    // slightly different behavior if closing from within our own helper thread
    int in_own_thread = pthread_equal(pthread_self(), c[ch].helper_thread_id);

    // lock the channel mutex, stop helper, and remove the pipe
    pthread_mutex_lock(&mtx[ch]);
    _stop_helper_and_remove_pipe(ch, in_own_thread);
    pthread_mutex_unlock(&mtx[ch]);

    // We used to exit out own thread here which worked on newer kernels but
    // crashes on 64-bit APQ8096 with an old kernel, so don't do it
    // if(in_own_thread) pthread_exit(0);

    return;
}


void pipe_client_close(int ch)
{
    // sanity checks
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return;
    }

    // nothing to do if not running
    if(c[ch].running==0) return;

    // slightly different behavior if closing from within our own helper thread
    pthread_t this_helper = c[ch].helper_thread_id;
    int in_own_thread = pthread_equal(pthread_self(), this_helper);
    if(en_debug && in_own_thread){
        fprintf(stderr, "calling close from within the helper thread\n");
    }

    // lock the channel mutex, stop helper, and remove the pipe
    pthread_mutex_lock(&mtx[ch]);
    _stop_helper_and_remove_pipe(ch, in_own_thread);

    // zero out the channel struct so we start fresh next time
    _clean_channel(ch);
    pthread_mutex_unlock(&mtx[ch]);

    // unclaim now we are done putting the channel back to a clean state
    pthread_mutex_lock(&claim_mtx);
    c[ch].claimed = 0;
    pthread_mutex_unlock(&claim_mtx);

    /*
    // We used to exit out own thread here which worked on newer kernels but
    // crashes on 64-bit APQ8096 with an old kernel, so don't do it
    if(in_own_thread){
        if(en_debug){
            fprintf(stderr, "calling pthread_detatch from within our own helper thread\n");
        }
        pthread_exit(0);
    }
    */

    return;
}


void pipe_client_close_all(void)
{
    for(int i=0; i<N_CH; i++) pipe_client_close(i);
    return;
}




// DEPRECATED
void pipe_client_print_error(int e)
{
    return pipe_print_error(e);
}

// DEPRECATED
int pipe_client_get_info_string(int ch, char* buf, int buf_len)
{
    if(ch<0 || ch>=N_CH){
        fprintf(stderr, "ERROR in %s, channel should be between 0 & %d\n", __FUNCTION__, N_CH-1);
        return PIPE_ERROR_CHANNEL_OOB;
    }
#ifndef __ANDROID__
    if(c[ch].data_fd==0){
        return PIPE_ERROR_NOT_CONNECTED;
    }
#endif

    char info_path[PATH_LEN+16];
    strcpy(info_path, c[ch].pipe_dir);
    strcat(info_path,"info");

    int fd = open(info_path, O_RDONLY);
    if(fd < 0){
        return PIPE_ERROR_INFO_NOT_AVAILABLE;
    }
    int num_bytes = read(fd, buf, buf_len);
    close(fd);

    return num_bytes;
}

// DEPRECATED
int pipe_client_construct_full_path(char* in, char* out)
{
    return pipe_expand_location_string(in, out);
}


// DEPRECATED
int pipe_client_init_channel(int ch, char* name_or_location, const char* client_name, int flags, int buf_len)
{
    return pipe_client_open(ch, name_or_location, client_name, flags, buf_len);
}

// DEPRECATED
void pipe_client_close_channel(int ch)
{
    pipe_client_close(ch);
    return;
}
