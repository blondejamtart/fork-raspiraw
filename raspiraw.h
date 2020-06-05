//
// Created by falcon on 31/03/2020.
//

#ifndef FORK_RASPIRAW_RASPIRAW_H
#define FORK_RASPIRAW_RASPIRAW_H

/*
Copyright (c) 2015, Raspberry Pi Foundation
Copyright (c) 2015, Dave Stevenson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define VERSION_STRING "0.0.2"

#define _GNU_SOURCE

#include <ctype.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define I2C_SLAVE_FORCE 0x0706

#include "interface/vcos/vcos.h"
#include "bcm_host.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_connection.h"

#include <sys/ioctl.h>

#include "raw_header.h"

#define DEFAULT_I2C_DEVICE 0

#define I2C_DEVICE_NAME_LEN 13    // "/dev/i2c-XXX"+NULL

enum bayer_order
{
    //Carefully ordered so that an hflip is ^1,
    //and a vflip is ^2.
    BAYER_ORDER_BGGR,
    BAYER_ORDER_GBRG,
    BAYER_ORDER_GRBG,
    BAYER_ORDER_RGGB
};

struct sensor_regs
{
    uint16_t reg;
    uint16_t data;
};

struct mode_def
{
    struct sensor_regs *regs;
    int num_regs;
    int width;
    int height;
    MMAL_FOURCC_T encoding;
    enum bayer_order order;
    int native_bit_depth;
    uint8_t image_id;
    uint8_t data_lanes;
    unsigned int min_vts;
    int line_time_ns;
    uint32_t timing1;
    uint32_t timing2;
    uint32_t timing3;
    uint32_t timing4;
    uint32_t timing5;
    uint32_t term1;
    uint32_t term2;
    int black_level;
};

struct sensor_def
{
    char *name;
    struct mode_def *modes;
    int num_modes;
    struct sensor_regs *stop;
    int num_stop_regs;

    uint8_t i2c_addr;        // Device I2C slave address
    int i2c_addressing;        // Length of register address values
    int i2c_data_size;        // Length of register data to write

    //  Detecting the device
    int i2c_ident_length;        // Length of I2C ID register
    uint16_t i2c_ident_reg;        // ID register address
    uint16_t i2c_ident_value;    // ID register value

    // Flip configuration
    uint16_t vflip_reg;        // Register for VFlip
    int vflip_reg_bit;        // Bit in that register for VFlip
    uint16_t hflip_reg;        // Register for HFlip
    int hflip_reg_bit;        // Bit in that register for HFlip
    int flips_dont_change_bayer_order;    // Some sensors do not change the
    // Bayer order by adjusting X/Y starts
    // to compensate.

    uint16_t exposure_reg;
    int exposure_reg_num_bits;

    uint16_t vts_reg;
    int vts_reg_num_bits;

    uint16_t gain_reg;
    int gain_reg_num_bits;

    uint16_t xos_reg;
    int xos_reg_num_bits;

    uint16_t yos_reg;
    int yos_reg_num_bits;
};

enum
{
    CommandHelp,
    CommandMode,
    CommandHFlip,
    CommandVFlip,
    CommandExposure,
    CommandGain,
    CommandOutput,
    CommandWriteHeader,
    CommandTimeout,
    CommandSaveRate,
    CommandBitDepth,
    CommandCameraNum,
    CommandExposureus,
    CommandI2cBus,
    CommandAwbGains,
    CommandRegs,
    CommandHinc,
    CommandVinc,
    CommandVoinc,
    CommandHoinc,
    CommandBin44,
    CommandFps,
    CommandWidth,
    CommandHeight,
    CommandLeft,
    CommandTop,
    CommandVts,
    CommandLine,
    CommandWriteHeader0,
    CommandWriteHeaderG,
    CommandWriteTimestamps,
    CommandWriteEmpty,
};

enum operation
{
    EQUAL,  //Set bit to value
    SET,    //Set bit
    CLEAR,  //Clear bit
    XOR     //Xor bit
};

typedef struct pts_node
{
    int idx;
    int64_t pts;
    struct pts_node *nxt;
} *PTS_NODE_T;

typedef struct
{
    int mode;
    int hflip;
    int vflip;
    int exposure;
    int gain;
    char *output;
    int capture;
    int write_header;
    int timeout;
    int saverate;
    int bit_depth;
    int camera_num;
    int exposure_us;
    int i2c_bus;
    double awb_gains_r;
    double awb_gains_b;
    char *regs;
    int hinc;
    int vinc;
    int voinc;
    int hoinc;
    int bin44;
    double fps;
    int width;
    int height;
    int left;
    int top;
    char *write_header0;
    char *write_headerg;
    char *write_timestamps;
    int write_empty;
    PTS_NODE_T ptsa;
    PTS_NODE_T ptso;
} RASPIRAW_PARAMS_T;

void update_regs(const struct sensor_def *sensor, struct mode_def *mode, int hflip, int vflip, int exposure, int gain);

static int i2c_rd(int fd, uint8_t i2c_addr, uint16_t reg, uint8_t *values, uint32_t n, const struct sensor_def *sensor);

const struct sensor_def *probe_sensor(void);

void send_regs(int fd, const struct sensor_def *sensor, const struct sensor_regs *regs, int num_regs);

void start_camera_streaming(const struct sensor_def *sensor, struct mode_def *mode);

void stop_camera_streaming(const struct sensor_def *sensor);

MMAL_STATUS_T create_filenames(char **finalName, char *pattern, int frame);

static void callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

uint32_t order_and_bit_depth_to_encoding(enum bayer_order order, int bit_depth);

void modReg(struct mode_def *mode, uint16_t reg, int startBit, int endBit, int value, enum operation op);

int camera_main(RASPIRAW_PARAMS_T cfg, void (*callback)(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer));

void modRegBit(struct mode_def *mode, uint16_t reg, int bit, int value, enum operation op);

void modReg(struct mode_def *mode, uint16_t reg, int startBit, int endBit, int value, enum operation op);

#endif //FORK_RASPIRAW_RASPIRAW_H
