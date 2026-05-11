#pragma once
#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// Copy this file to wifi_config.h and edit. wifi_config.h is .gitignored.
//
// HOST_IP is the ROS 2 laptop running imu_udp_node. UDP_PORT must match the
// node's listening port (default 8888).

#define WIFI_SSID        "your_ssid"
#define WIFI_PASSWORD    "your_password"
#define HOST_IP          "192.168.1.100"
#define UDP_PORT         8888

#endif // WIFI_CONFIG_H
