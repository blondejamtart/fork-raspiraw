//
// Created by falcon on 31/03/2020.
//

#include "raspiCam.h"


raspiCam::raspiCam() {

}

raspiCam::~raspiCam() {

}

static void raspiCam::callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    static int count = 0;
    vcos_log_error("Buffer %p returned, filled %d, timestamp %llu, flags %04X", buffer, buffer->length, buffer->pts,
                   buffer->flags);
    if (running) {
        RASPIRAW_PARAMS_T *cfg = (RASPIRAW_PARAMS_T *) port->userdata;

        if (!(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)) {
//            // Save every Nth frame
//            // SD card access is too slow to do much more.
//            FILE *file;
//            char *filename = NULL;
//            if (create_filenames(&filename, cfg->output, count) == MMAL_SUCCESS)
//            {
//                file = fopen(filename, "wb");
//                if (file)
//                {
//                    if (cfg->ptso)  // make sure previous malloc() was successful
//                    {
//                        cfg->ptso->idx = count;
//                        cfg->ptso->pts = buffer->pts;
//                        cfg->ptso->nxt = malloc(sizeof(*cfg->ptso->nxt));
//                        cfg->ptso = cfg->ptso->nxt;
//                    }
//                    if (!cfg->write_empty)
//                    {
//                        if (cfg->write_header)
//                            fwrite(brcm_header, BRCM_RAW_HEADER_LENGTH, 1, file);
//                        fwrite(buffer->data, buffer->length, 1, file);
//                    }
//                    fclose(file);
//                }
//                free(filename);
//            }
            // Yeet every frame via ZeroMQ
            // TODO: do zmq send here!
        }
        buffer->length = 0;
        mmal_port_send_buffer(port, buffer);
    } else {
        mmal_buffer_header_release(buffer);
    }
}

int raspiCam::start() {
    RASPIRAW_PARAMS_T cfg = {
            mode: 0,
            hflip:  0,
            vflip: 0,
            exposure: -1,
            gain: -1,
            output: NULL,
            capture: 0,
            write_header: 0,
            timeout: 5000,
            saverate: 1,
            bit_depth: -1,
            camera_num: -1,
            exposure_us: -1,
            i2c_bus: DEFAULT_I2C_DEVICE,
            regs: NULL,
            hinc: -1,
            vinc: -1,
            voinc: -1,
            hoinc: -1,
            bin44: 0,
            fps: 500,
            width: -1,
            height: -1,
            left: -1,
            top: -1,
            write_header0: NULL,
            write_headerg: NULL,
            write_timestamps: NULL,
            write_empty: 0,
            ptsa: NULL,
            ptso: NULL,
    };
    return camera_main(cfg, &(raspiCam::callback));
}