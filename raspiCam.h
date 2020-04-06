//
// Created by falcon on 31/03/2020.
//

#ifndef FORK_RASPIRAW_RASPICAM_H
#define FORK_RASPIRAW_RASPICAM_H

#include "raspiraw.h"

class raspiCam {
public:
    raspiCam();
    ~raspiCam();

private:
    int start();
    virtual void callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);
};

#endif //FORK_RASPIRAW_RASPICAM_H

