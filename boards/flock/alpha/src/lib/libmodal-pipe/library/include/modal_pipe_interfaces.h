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



#ifndef MODAL_PIPE_INTERFACES_H
#define MODAL_PIPE_INTERFACES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <modal_pipe_common.h>
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
// Apriltag/aruco Detection
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for translations/rotation
 * readings making it unique as an identifier.
 *
 * Also it spells "VOXL" in ASCII
 */
#define TAG_DETECTION_MAGIC_NUMBER (0x564F584C)


/**
 * A tag can be flagged by the user as fixed, static, or dynamic.
 *
 * "fixed": The tag is at a known location in space as described by the
 * T_tag_wrt_fixed vector and R_tag_to_fixed rotation matrix. These fixed tags
 * are used by voxl-vision-px4 for apriltag relocalization.
 *
 * "static": A static tag can be trusted to be static (not moving) but does not
 * have a known location. For example, a landing pad.
 *
 * "dynamic": A dynamic tag can be expected to be in motion such as an object to
 * be tracked or followed dynamically.
 *
 * If a tag is detected that has not been listed by the user as a known tag
 * in /etc/modalai/tag_locations.conf then it will be listed as unknown.
 */
#define TAG_LOCATION_TYPE_STRINGS {"unknown", "fixed","static","dynamic"}
#define N_TAG_LOCATION_TYPES    4

#define TAG_LOCATION_UNKNOWN    0
#define TAG_LOCATION_FIXED      1
#define TAG_LOCATION_STATIC     2
#define TAG_LOCATION_DYNAMIC    3

// max name length of a tag name
#define TAG_NAME_LEN 64


/**
 * describes an apriltag, aruco, or similar detection. Provides the tag's position and rotation
 * relative to the camera that detected it.
 *
 * This packet is 252 bytes long.
 */
typedef struct tag_detection_t{
    uint32_t magic_number;                  ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
    int32_t  id;                            ///< id number of the tag
    float    size_m;                        ///< size of the tag in meters
    int64_t  timestamp_ns;                  ///< timestamp at the middle of the frame exposure in monotonic time
    char     name[TAG_NAME_LEN];            ///< optional name of the tag
    int      loc_type;                      ///< location type
    float    T_tag_wrt_cam[3];              ///< location of the tag with respect to camera frame in meters.
    float    R_tag_to_cam[3][3];            ///< rotation matrix from tag frame to camera frame
    float    T_tag_wrt_fixed[3];            ///< only set if location type is LOCATION_FIXED
    float    R_tag_to_fixed[3][3];          ///< only set if location type is LOCATION_FIXED
    char     cam[MODAL_PIPE_MAX_DIR_LEN];   ///< camera pipe where the detection was made
    int      reserved;                      ///< reserved field
} __attribute__((packed)) tag_detection_t;



/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 16 packets because the
 * tag detection packet is 252-bytes long and this givea a buffer just under 4k
 *
 * Note this is NOT the size of the pipe which can hold more. This is just the
 * read buffer size allocated on the heap into which data from the pipe is read.
 */
#define TAG_DETECTION_RECOMMENDED_READ_BUF_SIZE (sizeof(tag_detection_t) * 16)


/**
 * We recommend tof servers use a pipe size of 64kB which is also the Linux
 * default pipe size. This allows 260 apriltag detections to be stored in the
 * pipe before losing data.
 */
#define TAG_DETECTION_RECOMMENDED_PIPE_SIZE (64*1024)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as an tag_detection_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate anapriltag_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an tag_detection_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
tag_detection_t* pipe_validate_tag_detection_t(char* data, int bytes, int* n_packets);


/**
 * @brief      convert a tag location type id number to string
 *
 * For example TAG_LOCATION_FIXED will return the string "fixed"
 *
 * @param[in]  i     location type id, e.g. TAG_LOCATION_FIXED
 *
 * @return     const char8 string of the format
 */
const char* pipe_tag_location_type_to_string(int i);




////////////////////////////////////////////////////////////////////////////////
// Camera and Images
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. Spells "VOXL" in ASCII
 */
#define CAMERA_MAGIC_NUMBER (0x564F584C)


// Common image formats for use by a camera server. This is not an exhaustive
// list and custom values not included here can be used as long as the server
// and client both agree on the image format.
#define IMAGE_FORMAT_RAW8           0   // 8-bit gray image, used for tracking camera
#define IMAGE_FORMAT_NV12           1
#define IMAGE_FORMAT_STEREO_RAW8    2   // 8-bit gray, two sequential images
#define IMAGE_FORMAT_H264           3
#define IMAGE_FORMAT_H265           4
#define IMAGE_FORMAT_RAW16          5   // 16-bit image, for disparity maps or HDR gray images
#define IMAGE_FORMAT_NV21           6   // Android NV21 format from hal3
#define IMAGE_FORMAT_JPG            7
#define IMAGE_FORMAT_YUV422         8   // Standard YUV422 with YUYV mapping scheme
#define IMAGE_FORMAT_YUV420         9
#define IMAGE_FORMAT_RGB            10  // 24-bits per pixel
#define IMAGE_FORMAT_FLOAT32        11  // 32-bit float per pixel, for depth map
#define IMAGE_FORMAT_STEREO_NV21    12  // Android NV21 format from hal3, two sequential images
#define IMAGE_FORMAT_STEREO_RGB     13  // 24-bits per pixel, two sequential images
#define IMAGE_FORMAT_YUV422_UYVY    14  // YUV422 with alternate UYVY mapping scheme
#define IMAGE_FORMAT_STEREO_NV12    15
// NOTE: when updating this list, also update pipe_image_format_to_string() in
// src/interfaces.c to print the new name

/**
 * The metadata for the camera image. One of these is sent before every frame
 */
typedef struct camera_image_metadata_t
{
    uint32_t magic_number;  ///< set to CAMERA_MAGIC_NUMBER
    int64_t  timestamp_ns;  ///< timestamp in apps-proc clock-monotonic of beginning of exposure
    int32_t  frame_id;      ///< iterator from 0++ starting from first frame when server starts on boot
    int16_t  width;         ///< image width in pixels
    int16_t  height;        ///< image height in bytes
    int32_t  size_bytes;    ///< size of the image, for stereo this is the size of both L&R together
    int32_t  stride;        ///< bytes per row
    int32_t  exposure_ns;   ///< exposure in microseconds
    int16_t  gain;          ///< ISO gain (100, 200, 400, etc..)
    int16_t  format;        ///< raw8, nv12, etc
    int16_t  framerate;     ///< expected framerate hz
    int16_t  reserved;      ///< extra reserved bytes
} __attribute__((packed)) camera_image_metadata_t;


/**
 * @brief      convert an image format id number to string
 *
 * For example IMAGE_FORMAT_RAW8 will return the string "RAW8"
 *
 * @param[in]  i     image format id, e.g. IMAGE_FORMAT_RAW8
 *
 * @return     const char8 string of the format
 */
const char* pipe_image_format_to_string(int i);



////////////////////////////////////////////////////////////////////////////////
// TOF
////////////////////////////////////////////////////////////////////////////////

/**
 * This packet describes a complete data reading from a PMD TOF sensor.
 *
 * This is a BIG packet, 693516 bytes!
 */


/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for translations/rotation
 * readings making it unique as an identifier.
 *
 * Also it spells "VOXL" in ASCII
 */
#define TOF_MAGIC_NUMBER (0x564F584C)

#define MPA_TOF_WIDTH  224
#define MPA_TOF_HEIGHT 172
#define MPA_TOF_SIZE   (MPA_TOF_WIDTH * MPA_TOF_HEIGHT)

typedef struct tof_data_t{
    uint32_t magic_number;                  ///< Unique 32-bit number used to signal the beginning of a packet while parsing a data stream.
    int64_t  timestamp_ns;                  ///< timestamp in nanoseconds
    float    points      [MPA_TOF_SIZE][3]; ///< Point cloud (x,y,z in meters)
    float    noises      [MPA_TOF_SIZE];    ///< noise value for each point (meters)
    uint8_t  grayValues  [MPA_TOF_SIZE];    ///< IR grayvalue for each point
    uint8_t  confidences [MPA_TOF_SIZE];    ///< Confidence value for each point
} __attribute__((packed)) tof_data_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 4 packets because the
 * tof data packet is massive and we wont be expecting to get them at more than
 * 15-30 hz
 *
 * Note this is NOT the size of the pipe which can hold more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define TOF_RECOMMENDED_READ_BUF_SIZE   (sizeof(tof_data_t) * 4)


/**
 * We recommend tof servers use a pipe size of 64 mb. This allows 6 seconds
 * of tof data at 15 hz to be stored
 */
#define TOF_RECOMMENDED_PIPE_SIZE   (1024 * 1024 * 64)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a tof_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a tof_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an tof_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
tof_data_t* pipe_validate_tof_data_t(char* data, int bytes, int* n_packets);



////////////////////////////////////////////////////////////////////////////////
// IMU
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for accel, gyro, or
 * temperature readings making it unique as an identifier.
 *
 * Also it spells "VOXL" in ASCII
 */
#define IMU_MAGIC_NUMBER  (0x564F584C)


/**
 * If a device cannot read temperature or temperature reading is disabled then
 * IMU_INVALID_TEMPERATURE_VALUE should be present in the temp_c field to
 * indicate this.
 */
#define IMU_INVALID_TEMPERATURE_VALUE (FLT_MIN)


/**
 * This is the data structure that imu servers should make available to clients
 * on the data pipe. (40 bytes long)
 */
typedef struct imu_data_t{
    uint32_t magic_number;  ///< Set to IMU_IMAGE_MAGIC_NUMBER for frame syncing
    float accl_ms2[3];      ///< XYZ acceleration in m/s^2
    float gyro_rad[3];      ///< XYZ gyro rotation in rad/s
    float temp_c;           ///< temp in C, IMU_INVALID_TEMPERATURE_VALUE if no thermometer present
    uint64_t timestamp_ns;  ///< timestamp in nanoseconds, uses system clock_monotonic
} __attribute__((packed)) imu_data_t;



/**
 * You don't have to use this read buffer size, but it is HIGHLY
 * recommended to use a multiple of the packet size so that you never read a
 * partial packet which would throw the reader out of sync. Here we use a nice
 * number of 400 packets which is perhaps more than necessary but only takes a
 * little under 16K of memory which is minimal.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define IMU_RECOMMENDED_READ_BUF_SIZE   (sizeof(imu_data_t) * 400)


/**
 * We recommend IMU servers use a 128k pipe size. This means every client would
 * get their own buffer of over 3.2 seconds of IMU data at 1000hz. Clients can
 * increase this buffer if they wish. voxl-imu-server uses this as its default.
 */
#define IMU_RECOMMENDED_PIPE_SIZE   (128 * 1024)


/**
 * @brief      Use this to simultaneously validate that the data from a pipe
 *             contains valid imu data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as an imu_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate an imu_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid imu_data_t packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an imu_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
imu_data_t* pipe_validate_imu_data_t(char* data, int bytes, int* n_packets);





////////////////////////////////////////////////////////////////////////////////
// Point Cloud
////////////////////////////////////////////////////////////////////////////////

/**
 * Points clouds are sent similar to camera frames with a metadata struct
 * followed by a sequence of points in float[3] format whose length is
 * determined by the metadata struct.
 *
 * Since each point is 3 floats in XYZ, the payload length will be 12*n_points
 * long.
 *
 * Use the point-cloud helper when initializing the modal pipe client to let
 * the helper pick apart the metadata and read in the right amount of data.
 */


/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for translations/rotation
 * readings making it unique as an identifier.
 *
 * Also it spells "VOXL" in ASCII
 */
#define POINT_CLOUD_MAGIC_NUMBER (0x564F584C)

#define POINT_CLOUD_FORMAT_FLOAT_XYZ        0   // 12-bytes per point, float, XYZ
#define POINT_CLOUD_FORMAT_FLOAT_XYZC       1   // 16-bytes per point, float, XYZ followed by confidence float
#define POINT_CLOUD_FORMAT_FLOAT_XYZRGB     2   // 15-bytes per point, float, XYZ followed by 8-bit RGB
#define POINT_CLOUD_FORMAT_FLOAT_XYZCRGB    3   // 19-bytes per point, float, XYZ followed by confidence float and 8-bit RGB
#define POINT_CLOUD_FORMAT_FLOAT_XY         4   //  8-bytes per point, float, XY
#define POINT_CLOUD_FORMAT_FLOAT_XYC        5   // 12-bytes per point, float, XY  followed by confidence float


typedef struct point_cloud_metadata_t{
    uint32_t magic_number;      ///< Unique 32-bit number used to signal the beginning of a packet while parsing a data stream.
    int64_t  timestamp_ns;      ///< timestamp in nanoseconds
    uint32_t n_points;          ///< number of points following the metadata struct
    uint32_t format;            ///< point cloud format
    uint32_t id;                ///< optional id, meaning is denoted by individual servers
    char server_name[32];       ///< optional server name, to specify the source of this pointcloud
    uint32_t reserved;          ///< reserved bytes
} __attribute__((packed)) point_cloud_metadata_t;



const char* pipe_point_cloud_format_to_string(int i);

/**
 * @brief      return the expected number of bytes of point cloud data that
 *             should follow a metadata struct in the stream.
 *
 *             different point cloud formats take up different numbers of bytes.
 *             For a metadata struct containing the number of bytes and format,
 *             this returns how many bytes are to be expects.
 *
 * @param[in]  meta  The metadata struct
 *
 * @return     expected number of bytes or -1 on error or invalid metadata
 *             struct.
 */
int pipe_point_cloud_meta_to_size_bytes(point_cloud_metadata_t meta);



////////////////////////////////////////////////////////////////////////////////
// mavlink_message_t
//
// Functions to interface with a stream of mavlink_message_t structs. Much like
// our own data structs, mavlink_message_t structs are of a fixed length and
// contain a magic number. Therefore they can be send neatly over a pipe.
//
// While it is possible to send a normal encoded mavlink stream over a pipe,
// this is much easier and saves the CPU from needing to pack/unpack/checksum
// the stream and is therefore more efficient and easier to read. Afterall,
// programs only want access to the final mavlink_message_t in the end. The
// unpacking and decoding of mavlink messages from PX4 is accelerated on the
// SDSP on VOXL already. In userspace land we just send the final
// mavlink_message_t around via pipes.
////////////////////////////////////////////////////////////////////////////////

#ifdef MAVLINK_H // mavlink is optional, most clients won't use it

/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 56 packets which is
 * perhaps more than necessary but only takes a little under 4 pages of memory
 * which is minimal.
 *
 * mavlink_message_t structs are 291 bytes long
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define MAVLINK_MESSAGE_T_RECOMMENDED_READ_BUF_SIZE (sizeof(mavlink_message_t) * 56)


/**
 * We recommend mavlink servers use the typical 1M pipe size for mavlink which
 * can contain up to 3603 messages.
 */
#define MAVLINK_MESSAGE_T_RECOMMENDED_PIPE_SIZE (1024 * 1024)


/**
 * @brief      Use this to simultaneously validate that the data from a pipe
 *             contains valid mavlink_message_t data, find the number of valid
 *             packets contained in a single read from the pipe, and cast the
 *             raw data buffer as a mavlink_message_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a mavlink_message_t array separate from the pipe read
 *             buffer. The data can be read straight out of the pipe read
 *             buffer.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid imu_data_t packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to a mavlink_message_t* struct for convenience. If there was
 *             an error then NULL is returned and n_packets is set to 0
 */
mavlink_message_t* pipe_validate_mavlink_message_t(char* data, int bytes, int* n_packets);

#endif // MAVLINK_H

////////////////////////////////////////////////////////////////////////////////
// 4DOF pose
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for translations/rotation
 * readings making it unique as an identifier.
 *
 * Also it spells "VOXL" in ASCII
 */
#define POSE_4DOF_MAGIC_NUMBER (0x564F584C)


/**
 * 4DOF pose (position and yaw)
 *
 * This is used to describe the position and orientation of the drone body in
 * fixed frame where roll and pitch are not needed for the fixed-frame pipe
 * input to voxl-vision-px4.
 *
 * Data is sent in with the position as the position of the center of mass of
 * the drone's body with respect to the fixed coordinate frame. The fixed
 * coordinate frame does not have to be north-aligned but should keep with the
 * NED convention of x pointing forward, Y to the right, and Z down.
 *
 * Yaw follows the right hand rule, e.g. positive yaw indicates rotation of the
 * body to the right about the Z axis. Yaw should be in +-PI
 *
 * This packet is 44 bytes long
 */
typedef struct pose_4dof_t{
    uint32_t magic_number;  ///< Unique 32-bit number used to signal the beginning of a packet while parsing a data stream.
    int64_t timestamp_ns;   ///< timestamp in apps-proc clock-monotonic
    double p[3];            ///< meters
    double yaw;             ///< radians, between +- PI
} __attribute__((packed)) pose_4dof_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 23 packets which is
 * perhaps more than necessary but only takes a little under 1kB of memory
 * which is minimal.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define POSE_4DOF_RECOMMENDED_READ_BUF_SIZE (sizeof(pose_4dof_t) * 23)


/**
 * We recommend pose servers use a 64k pipe size. This means every client would
 * get their own buffer of 49 seconds of data at 30hz. Clients can
 * increase this buffer if they wish.
 * 64K is also the Linux Kernel default pipe size.
 */
#define POSE_4DOF_RECOMMENDED_PIPE_SIZE (64 * 1024)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a pose_4dof_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a pose_4dof_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an pose_4dof_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
pose_4dof_t* pipe_validate_pose_4dof_t(char* data, int bytes, int* n_packets);






////////////////////////////////////////////////////////////////////////////////
// 6DOF pose with velocity
////////////////////////////////////////////////////////////////////////////////

/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for translations/rotation
 * readings making it unique as an identifier.
 *
 * Also it spells "VOXL" in ASCII
 */
#define POSE_VEL_6DOF_MAGIC_NUMBER (0x564F584C)


/**
 * Position and velocity in 6DOF, this is basically a stripped down VIO packet.
 *
 * This is how voxl-vision-px4 publishes the position and velocity of the drone
 * body in both local and fixed frame.
 *
 * Packet is 84 bytes long.
 */
typedef struct pose_vel_6dof_t{
    uint32_t magic_number;         ///< Unique 32-bit number used to signal the beginning of a packet while parsing a data stream.
    int64_t timestamp_ns;          ///< Timestamp in clock_monotonic system time of the provided pose.
    float T_child_wrt_parent[3];   ///< Translation of the body with respect to parent frame in meters.
    float R_child_to_parent[3][3]; ///< Rotation matrix from body to parent frame.
    float v_child_wrt_parent[3];   ///< Velocity of the body with respect to the parent frame.
    float w_child_wrt_child[3];    ///< Angular velocity of the body about its X Y and Z axes respectively. Essentially filtered gyro values with internal biases applied.
} __attribute__((packed)) pose_vel_6dof_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 48 packets which is
 * perhaps more than necessary but only takes a little under 2kB of memory
 * which is minimal.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define POSE_6DOF_RECOMMENDED_READ_BUF_SIZE (sizeof(pose_vel_6dof_t) * 24)


/**
 * We recommend pose servers use a 64k pipe size. This means every client would
 * get their own buffer of 26 seconds of data at 30hz. Clients can
 * increase this buffer if they wish.
 * 64K is also the Linux Kernel default pipe size.
 */
#define POSE_6DOF_RECOMMENDED_PIPE_SIZE (64 * 1024)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a pose_vel_6dof_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a pose_vel_6dof_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an pose_vel_6dof_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
pose_vel_6dof_t* pipe_validate_pose_vel_6dof_t(char* data, int bytes, int* n_packets);





////////////////////////////////////////////////////////////////////////////////
// VIO
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      Common interface for vio server-client communication
 *
 *             VINS and QVIO servers may offer different algorithm-specific
 *             interfaces but should also offer this common generic interface
 *             for simple applications.
 *
 *             The variable names in the data structure for describing
 *             translations and rotations adhere to the following naming
 *             convention for clarity and readability.
 *
 *             Translations are written as T_A_wrt_B meaning the translation
 *             (position) of the origin of frame A with respect to frame B.
 *             Rotations are written as R_A_to_B meaning a rotation matrix which
 *             rotates a vector in the coordinate frame of A to be represented
 *             as a vector in coordinate frame B.
 *
 *             The following coordinate frames are used:
 *
 *             VIO FRAME: Centered wherever the IMU was when VIO started and
 *             aligned with wherever the IMU was VIO started. This is NOT
 *             aligned with gravity and the user must make use of the provided
 *             gravity vector if they wish to align the VIO data to gravity.
 *
 *             IMU FRAME: Centered about IMU and aligned with the VOXL “common”
 *             IMU coordinate frame, NOT to the imu frame from any one specific
 *             chip since VOXL has multiple imus. T_imu_wrt_VIO and R_imu_to_vio
 *             describe the position and rotation of the IMU frame with respect
 *             to VIO frame. For a diagram explaining the VOXL “common” imu
 *             frame see https://docs.modalai.com/camera-imu-coordinate-frames/
 *
 *             CAMERA FRAME: Standard open-cv camera frame with X right, Y down,
 *             and Z out the lens. This relation is initially read from a config
 *             file but is optimized further in real time by the VIO algorithm
 *             allowing the user to make use of better estimation of the camera
 *             mounting angle to compensate for manufacturing tolerances.
 *
 *             VIO GRAVITY ALIGNED FRAME: Centered wherever VIO started with yaw
 *             aligned to IMU but roll/pitch aligned with gravity. VIO FRAME
 *             does not align with Z along gravity but with wherever the IMU was
 *             on initialization. If you application requires alignment with
 *             gravity then you can follow the example here:
 *             https://gitlab.com/voxl-public/voxl-vision-px4/-/blob/78feacef924ce64e4e1c4c3f3a8ae14d1b38630b/src/geometry.c#L321
 */



/**
 * The overall status of the VIO algorithm is described with three different
 * fields. The primary ‘state’ field is used to check the overall state of the
 * algorithm. The ‘state’ field can be VIO_STATE_FAILED, VIO_STATE_INITIALIZING,
 * or VIO_STATE_OK.
 *
 * When ‘state’ reports VIO_STATE_FAILED, the error_code field is set. Similarly
 * to posix errno, the ‘error_code’ contains the last thrown internal error. If
 * state==VIO_STATE_OK then the value contained in ‘error_code’ is undefined and
 * can be ignored.
 *
 * While the algorithm is running normally and state==VIO_STATE_OK then the user
 * can monitor the overall quality of the position estimate with the ‘quality’
 * field. This is a unitless measure of the VIO quality that is always positive
 * and increases with more accurate pose estimates. This is mostly derived from
 * the covariance matrix but also takes into account the number of features
 * being tracked and uncertainties in the algorithm’s behavior while
 * initializing.
 */
#define VIO_STATE_FAILED            0
#define VIO_STATE_INITIALIZING      1
#define VIO_STATE_OK                2

// codes 0-15 match mvVISLAM codes, don't change those!!
#define ERROR_CODE_COVARIANCE       (1<<0)  // Reset: cov not pos definite
#define ERROR_CODE_IMU_OOB          (1<<1)  // Reset: IMU exceeded range (out of bounds)
#define ERROR_CODE_IMU_BW           (1<<2)  // Reset: IMU bandwidth too low
#define ERROR_CODE_NOT_STATIONARY   (1<<3)  // Reset: not stationary at initialization
#define ERROR_CODE_NO_FEATURES      (1<<4)  // Reset: no features for x seconds
#define ERROR_CODE_CONSTRAINT       (1<<5)  // Reset: insufficient constraints from features
#define ERROR_CODE_FEATURE_ADD      (1<<6)  // Reset: failed to add new features
#define ERROR_CODE_VEL_INST_CERT    (1<<7)  // Reset: exceeded instant velocity uncertainty
#define ERROR_CODE_VEL_WINDOW_CERT  (1<<8)  // Reset: exceeded velocity uncertainty
#define ERROR_CODE_DROPPED_IMU      (1<<10) // Dropped IMU samples
#define ERROR_CODE_BAD_CAM_CAL      (1<<11) // Intrinsic camera cal questionable
#define ERROR_CODE_LOW_FEATURES     (1<<12) // Insufficient good features to initialize
#define ERROR_CODE_DROPPED_CAM      (1<<13) // Dropped camera frame
#define ERROR_CODE_DROPPED_GPS_VEL  (1<<14) // Dropped GPS velocity sample
#define ERROR_CODE_BAD_TIMESTAMP    (1<<15) // Sensor measurements with bad time stamps
#define ERROR_CODE_IMU_MISSING      (1<<16) // Missing IMU data
#define ERROR_CODE_CAM_MISSING      (1<<17) // Missing camera frames
#define ERROR_CODE_CAM_BAD_RES      (1<<18) // camera resolution unsupported
#define ERROR_CODE_CAM_BAD_FORMAT   (1<<19) // camera format unsupported
#define ERROR_CODE_UNKNOWN          (1<<20) // Unknown error
#define ERROR_CODE_STALLED          (1<<21) // frame processing stalled


/**
 * The following commands can be sent to the VIO server over its control pipe.
 * Commanding a reset will force the algorithm to reinitialize. This will take
 * an indeterminant amount of time and may require the IMU to detect a
 * stationary position before the reset completes if the server is configured to
 * only initialize when stationary. A hard reset will go back to reporting a
 * position of 0,0,0 after reinitializing. A soft reset will try to continue
 * reporting the last estiamted position after reset.
 *
 * More commands may be added to expand functionality in the future
 */
#define RESET_VIO_SOFT "reset_vio_soft"
#define RESET_VIO_HARD "reset_vio_hard"


/**
 * Unique 32-bit number used to signal the beginning of a data packet while
 * parsing a data stream. If this were to be cast as a float it would have a
 * value of 5.7x10^13 which is an impossible value for translations/rotation
 * readings making it unique as an identifier.
 *
 */
#define VIO_MAGIC_NUMBER (0x5455524)




/**
 * This is the data structure that vio servers should make available to clients
 * on the data pipe.
 *
 * totals 324 bytes
 *
 *
 *  covariance matrix entries defined as follows:
    uint8 COVARIANCE_MATRIX_X_VARIANCE=0
    uint8 COVARIANCE_MATRIX_Y_VARIANCE=6
    uint8 COVARIANCE_MATRIX_Z_VARIANCE=11
    uint8 COVARIANCE_MATRIX_ROLL_VARIANCE=15
    uint8 COVARIANCE_MATRIX_PITCH_VARIANCE=18
    uint8 COVARIANCE_MATRIX_YAW_VARIANCE=20
    uint8 COVARIANCE_MATRIX_VX_VARIANCE=0
    uint8 COVARIANCE_MATRIX_VY_VARIANCE=6
    uint8 COVARIANCE_MATRIX_VZ_VARIANCE=11
    uint8 COVARIANCE_MATRIX_ROLLRATE_VARIANCE=15
    uint8 COVARIANCE_MATRIX_PITCHRATE_VARIANCE=18
    uint8 COVARIANCE_MATRIX_YAWRATE_VARIANCE=20
 */
typedef struct vio_data_t{
    uint32_t magic_number;         ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
    int32_t quality;                 ///< Quality is be >0 in normal use with a larger number indicating higher quality. A positive quality does not guarantee the algorithm has initialized completely.
    int64_t timestamp_ns;          ///< Timestamp in clock_monotonic system time of the provided pose.
    float T_imu_wrt_vio[3];        ///< Translation of the IMU with respect to VIO frame in meters.
    float R_imu_to_vio[3][3];      ///< Rotation matrix from IMU to VIO frame.
    float pose_covariance[21];     ///<  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
    float vel_imu_wrt_vio[3];      ///< Velocity of the imu with respect to the VIO frame.
    float velocity_covariance[21]; ///<  Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
    float imu_angular_vel[3];      ///< Angular velocity of the IMU about its X Y and Z axes respectively. Essentially filtered gyro values with internal biases applied.
    float gravity_vector[3];       ///< Estimation of the current gravity vector in VIO frame. Use this to estimate the rotation between VIO frame and a gravity-aligned VIO frame if desired.
    float T_cam_wrt_imu[3];        ///< Location of the optical center of the camera with respect to the IMU.
    float R_cam_to_imu[3][3];      ///< Rotation matrix from camera frame to IMU frame.
    uint32_t error_code;           ///< bitmask that can indicate multiple errors. may still contain errors if state==VIO_STATE_OK
    uint16_t n_feature_points;     ///< Number of optical feature points currently being tracked.
    uint8_t state;                 ///< This is used to check the overall state of the algorithm. Can be VIO_STATE_FAILED, VIO_STATE_INITIALIZING, or VIO_STATE_OK.
    uint8_t reserved;              ///< extra byte reserved for future use
} __attribute__((packed)) vio_data_t;


/**
 * You don't have to use this read buffer size, but it is HIGHLY recommended to
 * use a multiple of the packet size so that you never read a partial packet
 * which would throw the reader out of sync. Here we use 26 packets which is
 * perhaps more than necessary but only takes a little under 1 page of memory
 * which is minimal.
 *
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define VIO_RECOMMENDED_READ_BUF_SIZE   (sizeof(vio_data_t) * 26)


/**
 * We recommend VIO servers use a 64k pipe size. This means every client would
 * get their own buffer of 14 seconds of VIO data at 30hz. Clients can
 * increase this buffer if they wish. voxl-qvio-server uses this as its default.
 * 64K is also the Linux Kernel default pipe size.
 */
#define VIO_RECOMMENDED_PIPE_SIZE   (64 * 1024)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a vio_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a vio_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an vio_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
vio_data_t* pipe_validate_vio_data_t(char* data, int bytes, int* n_packets);


// max number of features reported by the extended data struct. More features
// may be available. TODO output a full point cloud.
#define VIO_MAX_REPORTED_FEATURES 64


typedef enum vio_point_quality_t{
    LOW,     ///< additional low-quality points collected for e.g. collision avoidance
    MEDIUM,  ///< Points that are not "in state"
    HIGH     ///< Points that are "in state"
} vio_point_quality_t;


typedef struct vio_feature_t{
    uint32_t  id;               ///< unique ID for feature point
    int32_t cam_id;             ///< ID of camera which the point was seen from (typically first)
    float pix_loc[2];           ///< pixel location in the last frame
    float tsf[3];               ///< location of feature in vio frame (relative to init location)
    float p_tsf[3][3];          ///< covarience of feature location
    float depth;                ///< distance from camera to point
    float depth_error_stddev;   ///< depth error in meters
    vio_point_quality_t point_quality;
} vio_feature_t;


/**
 * This is the extended version of the vio_data_t struct, adding in extra debug fields
 * along with feature locations + quality.
 *
 * totals 5268 bytes
 */
typedef struct ext_vio_data_t{
    vio_data_t v;
    int32_t last_cam_frame_id;
    int64_t last_cam_timestamp_ns;
    float imu_cam_time_shift_s;
    float gravity_covariance[3][3];
    float gyro_bias[3];
    float accl_bias[3];
    uint32_t n_total_features;   ///< total features, in-state and out-of-state listed in the following array
    vio_feature_t features[VIO_MAX_REPORTED_FEATURES];
}  __attribute__((packed)) ext_vio_data_t;


/**
 * Note this is NOT the size of the pipe which can hold much more. This is just
 * the read buffer size allocated on the heap into which data from the pipe is
 * read.
 */
#define EXT_VIO_RECOMMENDED_READ_BUF_SIZE   (sizeof(ext_vio_data_t) * 10)


/**
 * @brief      Use this to simultaneously validate that the bytes from a pipe
 *             contains valid data, find the number of valid packets
 *             contained in a single read from the pipe, and cast the raw data
 *             buffer as a vio_data_t* for easy access.
 *
 *             This does NOT copy any data and the user does not need to
 *             allocate a vio_data_t array separate from the pipe read buffer.
 *             The data can be read straight out of the pipe read buffer, much
 *             like reading data directly out of a mavlink_message_t message.
 *
 *             However, this does mean the user should finish processing this
 *             data before returning the pipe data callback which triggers a new
 *             read() from the pipe.
 *
 * @param[in]  data       pointer to pipe read data buffer
 * @param[in]  bytes      number of bytes read into that buffer
 * @param[out] n_packets  number of valid packets received
 *
 * @return     Returns the same data pointer provided by the first argument, but
 *             cast to an vio_data_t* struct for convenience. If there was an
 *             error then NULL is returned and n_packets is set to 0
 */
ext_vio_data_t* pipe_validate_ext_vio_data_t(char* data, int bytes, int* n_packets);


/**
 * @brief      print a human-readable string representation of a VIO state to
 *             stdout
 *
 * @param[in]  s     state to print
 */
void pipe_print_vio_state(int s);


/**
 * @brief      print a human-readable string representation of a VIO error code
 *             to stdout
 *
 * @param[in]  e     error to print
 */
void pipe_print_vio_error(uint32_t e);


/**
 * @brief      populates a user-allocated buffer with a human-readable string
 *             representing a provided VIO Error Code e.
 *
 *             e is a bitfield that may contain many errors, all of which will
 *             be printed up to the specified length of the buffer. We suggest a
 *             buffer length of about 256, but you can go above or below
 *             depending on your application. We do not allow anything smaller
 *             than 20 since that's pretty useless.
 *
 * @param[in]  e        VIO error code
 * @param      str      user-allocated buffer
 * @param[in]  buf_len  The buffer length (must be >= 20)
 *
 * @return     0 on success, -1 on failure
 */
int pipe_construct_vio_error_string(uint32_t e, char* str, size_t buf_len);


#ifdef __cplusplus
}
#endif

#endif // MODAL_PIPE_INTERFACES_H
