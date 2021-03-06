//
// Created by jsz on 3/8/20.
//

#ifndef SERIAL_PORT_SERIAL_PORT_H
#define SERIAL_PORT_SERIAL_PORT_H

#include "message.h"
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstdio>

#include "common.h"
class serial_port {
public:
    serial_port();
    serial_port(char* serial_name, int buadrate);
    ~serial_port() = default;
    //struct serial_gimbal_data pack_gimbal_data(const struct gimbal_msg &data);
    void send_data(const struct serial_gimbal_data &data);
    void restart_serial_port();
    void recive_data(struct serial_recive_data &receiveData);
    bool success_{};
    int fd{};
    int last_fd{};
private:
    int buadrate_{};
    char* serial_name_{};

};


#endif //SERIAL_PORT_SERIAL_PORT_H
