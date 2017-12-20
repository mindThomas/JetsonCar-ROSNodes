#include <ros/ros.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "node.h"
#include "crc.h"

#include "std_msgs/Int32.h"
#include "std_msgs/UInt16.h"

termios old_tios;

/* Sets up a serial port for RTU communications */
int connect_serial(const char* dev_name, int baudrate, int* portd)
{
    termios tios;
    speed_t speed;
    int flags;

    /* The O_NOCTTY flag tells UNIX that this program doesn't want
       to be the "controlling terminal" for that port. If you
       don't specify this then any input (such as keyboard abort
       signals and so forth) will affect your process

       Timeouts are ignored in canonical input mode or when the
       NDELAY option is set on the file via open or fcntl */
    flags = O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL;
#ifdef O_CLOEXEC
    flags |= O_CLOEXEC;
#endif

    *portd = open(dev_name, flags);
    if (*portd == -1) {
            fprintf(stderr, "ERROR Can't open the device %s (%s)\r\n",
                    dev_name, strerror(errno));
        return -1;
    }

    /* Save */
    tcgetattr(*portd, &old_tios);

    memset(&tios, 0, sizeof(struct termios));

    /* C_ISPEED     Input baud (new interface)
       C_OSPEED     Output baud (new interface)
    */
    switch (baudrate) {
    case 110: speed = B110; break;
    case 300: speed = B300; break;
    case 600: speed = B600; break;
    case 1200: speed = B1200; break;
    case 2400: speed = B2400; break;
    case 4800: speed = B4800; break;
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
#ifdef B57600
    case 57600: speed = B57600; break;
#endif
#ifdef B115200
    case 115200: speed = B115200; break;
#endif
#ifdef B230400
    case 230400: speed = B230400; break;
#endif
    default:
        speed = B9600;
        fprintf(stderr, "WARNING Unknown baud rate %d for %s (B9600 used)\r\n",
                baudrate, dev_name);
    }

    /* Set the baud rate */
    if ((cfsetispeed(&tios, speed) < 0) ||
        (cfsetospeed(&tios, speed) < 0)) {
        close(*portd);
        *portd = -1;
        return -1;
    }

    /* C_CFLAG      Control options
       CLOCAL       Local line - do not change "owner" of port
       CREAD        Enable receiver
    */
    tios.c_cflag |= (CREAD | CLOCAL);
    /* CSIZE, HUPCL, CRTSCTS (hardware flow control) */

    /* CSIZE        Bit mask for data bits */
    tios.c_cflag &= ~CSIZE;
    tios.c_cflag |= CS8;


    /* Stop bit (1 or 2) */
    tios.c_cflag &=~ CSTOPB;
    ///tios.c_cflag |= CSTOPB;

    /* PARENB       Enable parity bit
       PARODD       Use odd parity instead of even */
    // no parity
    tios.c_iflag &= ~INPCK;
    tios.c_cflag &=~ PARENB;

    /* Raw input */
    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* Software flow control is disabled */
    tios.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* Raw ouput */
    tios.c_oflag &= ~OPOST;

    /* Unused because we use open with the NDELAY option */
    tios.c_cc[VMIN] = 0;
    tios.c_cc[VTIME] = 0;

    if (tcsetattr(*portd, TCSAFLUSH/*TCSANOW*/, &tios) < 0) {
        close(*portd);
        *portd = -1;
        return -1;
    }

    return 0;
}

int disconnect_serial(int* portd) {
    if (*portd != -1) {
        tcsetattr(*portd, TCSANOW, &old_tios);
        close(*portd);
        *portd = -1;
    }
    return 0;
}

int _flush(int* portd) {
    return tcflush(*portd, TCIOFLUSH);
}

int _select(int* portd, fd_set *rset, struct timeval *tv)
{
    int s_rc;
    while ((s_rc = select(*portd+1, rset, NULL, NULL, tv)) == -1) {
        if (errno == EINTR) {
            fprintf(stderr, "A non blocked signal was caught\n");
            /* Necessary after an error */
            FD_ZERO(rset);
            FD_SET(*portd, rset);
        } else {
            return -1;
        }
    }

    return s_rc;
}

const char* uart_name = "/dev/ttyACM5";  // ttyTHS2 = Jetson UART1 located on Serial Port Header J17
int baudrate = 115200;

int _read(int* portd, uint8_t* in_buff, size_t in_buff_size, timeval byte_to)
{
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(*portd, &rset);
    timeval tv = {1, 0};
    int rc;
    int received = 0;

    while (_select(portd, &rset, &tv) > 0)
    {
        if (received >= in_buff_size) {
            _flush(portd);
            errno = ENOBUFS;
            return received;
        }

        tv.tv_sec = byte_to.tv_sec;
        tv.tv_usec = byte_to.tv_usec;
        rc = read(*portd, in_buff + received,
                1);//std::min((size_t)8, in_buff_size - received));

        if (rc == 0) {
            errno = ECONNRESET;
            rc = -1;
        }

        if (rc == -1) {
            fprintf(stderr, "ERROR while read, try reconnect: %s\r\n", strerror(errno));
            if (errno == ECONNRESET || errno == ECONNREFUSED ||
                 errno == EBADF) {
                int saved_errno = errno;
                disconnect_serial(portd);
                connect_serial(uart_name, baudrate, portd);
                /* Could be removed by previous calls */
                errno = saved_errno;
            }
            return -1;
        }

        received += rc;
    }

    errno = 0;
    return received;
}

int _write(int* portd, const uint8_t* out_buff, size_t out_buff_size)
{
    return write(*portd, out_buff, out_buff_size);
}


int STM32_fd = -1;
LiDAR_Measurement LiDAR_Distance[360];

ros::Publisher pubEncoderFront, pubEncoderRear, pubEncoderTimestamp;
ros::Publisher pubRCThrottle, pubRCSteering;

int main(int argc, char** argv)
{
  int ret = 0;
  uint8_t buffer[10*PACKAGE_LENGTH];
  struct termios uart_attr;  

  ros::init(argc, argv, "stm32_node");
  ros::NodeHandle nh;
  ROS_INFO("Launching node");

  pubEncoderFront = nh.advertise<std_msgs::Int32>("encoder/front", 10);
  pubEncoderRear = nh.advertise<std_msgs::Int32>("encoder/rear", 10);
  pubEncoderTimestamp = nh.advertise<std_msgs::Int32>("encoder/timestamp", 10);
  pubRCThrottle = nh.advertise<std_msgs::UInt16>("rc/throttle", 10);
  pubRCSteering = nh.advertise<std_msgs::UInt16>("rc/steering", 10);

  crcInit();

  // Open Serial port to STM32

  /* The O_NOCTTY flag tells UNIX that this program doesn't want
     to be the "controlling terminal" for that port. If you
     don't specify this then any input (such as keyboard abort
     signals and so forth) will affect your process

     Timeouts are ignored in canonical input mode or when the
     NDELAY option is set on the file via open or fcntl */
  /*STM32_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NDELAY | O_NOCTTY);

  if(STM32_fd < 0){
    printf("error opening UART %s, aborting..\n", uart_name);
    return STM32_fd;
  }

  uart_attr.c_ospeed = 0;
  if(tcgetattr(STM32_fd, &uart_attr) < 0) {
    printf("failed to read uart attibute, aborting \n");
    return STM32_fd;
  }
  cfsetspeed(&uart_attr, B921600); // set baud rate - however it doesn't matter because of USB SPP (emulated Serial port over USB)



  if(tcsetattr(STM32_fd, TCSANOW, &uart_attr) < 0) {
    printf("failed to set uart attibute, aborting \n");
    return STM32_fd;
  }

  if(tcsetattr(STM32_fd, TCSAFLUSH, &uart_attr) < 0) {
    printf("failed to set uart attibute, aborting \n");
    return STM32_fd;
  }*/

  if (connect_serial(uart_name, baudrate, &STM32_fd) < 0) {
          fprintf(stderr, "Error open serial port\r\n");
          return -1;
      }

  //sleep(2);

  //tcflush(STM32_fd, TCIOFLUSH);
  _flush(&STM32_fd);

  // Manual flush
  ret = 1;
  while (ret > 0) {
    ret = read(STM32_fd, buffer, 1);
  }

  printf("Flushed serial port\n");

  int byte_timeout_mks = std::max(
          (10/*bits*/ * 1000000.0/*microseconds*/ / baudrate) * 50/*bytes - minimum pause*/ + 1
          , 2000.0 );
  timeval byte_to = {0, byte_timeout_mks};

  int packIndex;
  ros::Rate loop_rate(1000);
  while (nh.ok()) {

    ret = _read(&STM32_fd, buffer, 10*PACKAGE_LENGTH, byte_to); // because of USB endpoint communication, packages will be packed together and read in a bunch - so we allow buffering/processing of up to 10 packages pr. spin
    if (ret > 0) {
        packIndex = 0;
        while (packIndex < (ret-1)) {
            if (buffer[packIndex] == HEADER_BYTE && buffer[packIndex+1] == HEADER_BYTE) {
                switch (ParsePackage(&buffer[packIndex])) {
                case 0: // OK
                    break;
                case 1:
                    printf("Incorrect package header\n");
                    break;
                case 2:
                    printf("Incorrect checksum\n");
                    break;
                case -1:
                    printf("Unknown package ID\n");
                    break;
                default:
                    printf("An error occured parsing the received package\n");
                    break;
                }
                packIndex += PACKAGE_LENGTH;
            } else {
                packIndex++;
            }
        }
    }
    ros::spinOnce();

    //loop_rate.sleep();
  }

  close(STM32_fd);
}

int ParsePackage(uint8_t * package)
{
	crc calcCRC, packCRC;
	if (package[0] != HEADER_BYTE || package[1] != HEADER_BYTE) return 1;
	calcCRC = crcFast(package, (PACKAGE_LENGTH-2)); // calculate CRC based on package content
	packCRC = ((uint16_t)package[PACKAGE_LENGTH-2] << 8) | package[PACKAGE_LENGTH-1];
	
	if (calcCRC != packCRC) return 2;
    //printf("Verified package structure and checksum\n");

	switch (package[2]) {
		case CMD_LIDAR:            
            return ParseLiDARMeasurement(package);
			break;
        case CMD_ENCODER:            
            return ParseEncoderMeasurement(package);
            break;
        case CMD_RC:            
            return ParseRCMeasurement(package);
            break;
	default:
		return -1;
		break;
	}
}

int ParseLiDARMeasurement(uint8_t * package)
{
	int i;
    uint16_t ID[4];
	
	for (i = 0; i < 4; i++) {
		ID[i] = ((uint16_t)package[3+3*i] << 1) | (package[3+3*i+1] >> 7);
	}
    //printf("Detected ID %d, %d, %d, %d\n", ID[0], ID[1], ID[2], ID[3]);
//	ros::Time::now();
    return 0; // OK
}

int ParseEncoderMeasurement(uint8_t * package)
{
    std_msgs::Int32 Timestamp;
    std_msgs::Int32 EncFront, EncRear;

    Timestamp.data = *(uint32_t*)&package[3];
    EncFront.data = *(int32_t*)&package[7];
    EncRear.data = *(int32_t*)&package[11];

    pubEncoderTimestamp.publish(Timestamp);
    pubEncoderFront.publish(EncFront);
    pubEncoderRear.publish(EncRear);

    //printf("%lu: Encoder Front %ld - Encoder Rear %ld\n", Timestamp, EncFront.data, EncRear.data);

    return 0; // OK
}

int ParseRCMeasurement(uint8_t * package)
{
    uint32_t Timestamp;
    std_msgs::UInt16 Throttle, Steering;

    Timestamp = *(uint32_t*)&package[3];
    Throttle.data = *(uint16_t*)&package[7];
    Steering.data = *(uint16_t*)&package[9];

    pubRCThrottle.publish(Throttle);
    pubRCSteering.publish(Steering);

    //printf("%lu: RC Throttle %lu - RC Steering %lu\n", Timestamp, Throttle, Steering);

    return 0; // OK
}









