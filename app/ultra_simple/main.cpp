/*
 *  SLAMTEC LIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <pigpio.h> // Include pigpio library

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms) {
    while (ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}
#endif

using namespace sl;

// Define GPIO pin for MOTOCTL
#define MOTOCTL_GPIO 18

void print_usage(int argc, const char * argv[]) {
    printf("Usage:\n"
           " For serial channel\n %s --channel --serial <com port> [baudrate]\n"
           " The baudrate used by different models is as follows:\n"
           "  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
           "A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
           " For udp channel\n %s --channel --udp <ipaddr> [port NO.]\n"
           " The T1 default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n",
           argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver * drv) {
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // Check if operation succeeded
        printf("SLAMTEC Lidar health status: %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error: SLAMTEC lidar internal error detected. Reboot the device to retry.\n");
            return false;
        } else {
            return true;
        }
    } else {
        fprintf(stderr, "Error: Cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed;
void ctrlc(int) {
    ctrl_c_pressed = true;
}

int main(int argc, const char * argv[]) {
    const char * opt_is_channel = NULL; 
    const char * opt_channel = NULL;
    const char * opt_channel_param_first = NULL;
    sl_u32 opt_channel_param_second = 0;
    sl_u32 baudrateArray[2] = {115200, 256000};
    sl_result op_result;
    int opt_channel_type = CHANNEL_TYPE_SERIALPORT;

    bool useArgcBaudrate = false;
    IChannel* _channel;

    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", SL_LIDAR_SDK_VERSION);

    if (argc > 1) {
        opt_is_channel = argv[1];
    } else {
        print_usage(argc, argv);
        return -1;
    }

    if (strcmp(opt_is_channel, "--channel") == 0) {
        opt_channel = argv[2];
        if (strcmp(opt_channel, "-s") == 0 || strcmp(opt_channel, "--serial") == 0) {
            opt_channel_param_first = argv[3];
            if (argc > 4) opt_channel_param_second = strtoul(argv[4], NULL, 10);
            useArgcBaudrate = true;
        } else if (strcmp(opt_channel, "-u") == 0 || strcmp(opt_channel, "--udp") == 0) {
            opt_channel_param_first = argv[3];
            if (argc > 4) opt_channel_param_second = strtoul(argv[4], NULL, 10);
            opt_channel_type = CHANNEL_TYPE_UDP;
        } else {
            print_usage(argc, argv);
            return -1;
        }
    } else {
        print_usage(argc, argv);
        return -1;
    }

    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if (!opt_channel_param_first) {
#ifdef _WIN32
            opt_channel_param_first = "\\\\.\\com3";
#elif __APPLE__
            opt_channel_param_first = "/dev/tty.SLAB_USBtoUART";
#else
            opt_channel_param_first = "/dev/ttyUSB0";
#endif
        }
    }

    // Initialize pigpio
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Error: Failed to initialize pigpio library.\n");
        return -1;
    }

    // Set MOTOCTL GPIO pin as output and ensure motor is off initially
    gpioSetMode(MOTOCTL_GPIO, PI_OUTPUT);
    gpioWrite(MOTOCTL_GPIO, PI_LOW);

    // Create driver instance
    ILidarDriver * drv = *createLidarDriver();
    if (!drv) {
        fprintf(stderr, "Error: Insufficient memory, exiting.\n");
        gpioTerminate();
        return -1;
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if (useArgcBaudrate) {
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK(drv->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);
                if (SL_IS_OK(op_result)) connectSuccess = true;
            }
        } else {
            size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
            for (size_t i = 0; i < baudRateArraySize; ++i) {
                _channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK(drv->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);
                    if (SL_IS_OK(op_result)) {
                        connectSuccess = true;
                        break;
                    }
                }
            }
        }
    }

    if (!connectSuccess) {
        fprintf(stderr, "Error: Cannot connect to the specified serial port %s.\n", opt_channel_param_first);
        goto on_finished;
    }

    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\nFirmware Ver: %d.%02d\nHardware Rev: %d\n",
           devinfo.firmware_version >> 8,
           devinfo.firmware_version & 0xFF,
           (int)devinfo.hardware_version);

    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    // Start motor using GPIO
    gpioWrite(MOTOCTL_GPIO, PI_HIGH);
    printf("Motor started.\n");

    drv->startScan(0, 1);

    // Fetch scan data
    signal(SIGINT, ctrlc);
    while (!ctrl_c_pressed) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);
        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (size_t i = 0; i < count; ++i) {
                printf("Angle: %.2f Distance: %.2f Quality: %d\n",
                       (nodes[i].angle_z_q14 * 90.0f) / 16384.0f,
                       nodes[i].dist_mm_q2 / 4.0f,
                       nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
        }
    }

    drv->stop();
    delay(200);

    // Stop motor using GPIO
    gpioWrite(MOTOCTL_GPIO, PI_LOW);
    printf("Motor stopped.\n");

on_finished:
    if (drv) delete drv;
    gpioTerminate();
    return 0;
}

