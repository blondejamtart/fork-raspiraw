//
// Created by falcon on 31/03/2020.
//

#include "raspiCam.h"


raspiCam::raspiCam() {

}

raspiCam::~raspiCam() {

}

int raspiCam::start() {
        RASPIRAW_PARAMS_T cfg = {
            .mode = 0,
            .hflip = 0,
            .vflip = 0,
            .exposure = -1,
            .gain = -1,
            .output = NULL,
            .capture = 0,
            .write_header = 0,
            .timeout = 5000,
            .saverate = 20,
            .bit_depth = -1,
            .camera_num = -1,
            .exposure_us = -1,
            .i2c_bus = DEFAULT_I2C_DEVICE,
            .regs = NULL,
            .hinc = -1,
            .vinc = -1,
            .voinc = -1,
            .hoinc = -1,
            .bin44 = 0,
            .fps = -1,
            .width = -1,
            .height = -1,
            .left = -1,
            .top = -1,
            .write_header0 = NULL,
            .write_headerg = NULL,
            .write_timestamps = NULL,
            .write_empty = 0,
            .ptsa = NULL,
            .ptso = NULL,
    };
    return camera_main(cfg);
}