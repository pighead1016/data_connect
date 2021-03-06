#ifndef  __UART_DRIVER_H
#define  __UART_DRIVER_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>

#include <semaphore.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <errno.h>
#include <malloc.h>
#include <termios.h>
#include "math.h"
#include <stdbool.h>
#include <sys/time.h>

#define PACKSIZE  1811
#define PACKLEN   (PACKSIZE/5-2)
 



#pragma pack(1)
typedef struct _lslidar_response_measurement_node_t
{
	unsigned char sync_quality;
	unsigned short angle_q6_checkbit; //�Ƕ�
	unsigned short distance_q2;            //����
	
} lslidar_response_measurement_node_t;
static void *CreatePthread(void *data);
int open_serial(std::string port,speed_t baud);
#endif
