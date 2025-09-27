/* 
 * Copyright (C) 2025 John Donoghue
 *
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
 */

#include <iostream>
#include <string>
#include <algorithm>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "serial_device.h"
#include "exception.h"

namespace librevisa {
namespace serial {

#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_TOGGLE(x,y) ((x) ^= (y))
#define BITMASK_CHECK(x,y) ((x) & (y))
#define BITMASK_CHECK_VALUE(x,y,z) (((x) & (y)) == (z))


serial_device::serial_device(const std::string &dev)
 : fd(-1), timeout(10.0)
{
        int flags = O_RDWR | O_NOCTTY | O_SYNC | O_NDELAY;
        // O_SYNC - All writes immediately effective, no buffering
        // O_NOCTTY - Do not make serialport terminal the controlling terminal for the process
        // O_NDELAY - Do not care what state the DCD signal line is in. Used for open only, later disabled.

        name = dev;

        fd = ::open (dev.c_str (), flags);

        if (fd < 0) {
                // error
                throw exception(VI_ERROR_SYSTEM_ERROR);
        }

        if(! isatty (fd)) {
                int err = errno;
                serial_device::close ();
                throw exception(VI_ERROR_SYSTEM_ERROR);
        }

        if (tcgetattr (fd, &config) < 0) {
                int err = errno;
                serial_device::close ();
                throw exception(VI_ERROR_SYSTEM_ERROR);
        }

        // Clear all settings
        config.c_iflag = 0; // Input modes
        config.c_oflag = 0; // Output modes
        config.c_cflag = CS8 | CREAD | CLOCAL; // Control modes, 8n1
        config.c_lflag = 0; // Local modes
        config.c_cc[VMIN] = 0;
        config.c_cc[VTIME] = 5;

        // set an intial baudrate
        cfsetospeed (&config, B9600);
        cfsetispeed (&config, B9600);

        if (tcsetattr (fd, TCSANOW, &config) < 0) {
                int err = errno;
                serial_device::close ();
                throw exception(VI_ERROR_SYSTEM_ERROR);
        }

        // Disable NDELAY
        if (fcntl (fd, F_SETFL, 0) < 0) {
                int err = errno;
                serial_device::close ();
                throw exception(VI_ERROR_SYSTEM_ERROR);
        }

        // all good
}

serial_device::~serial_device()
{
      close();
}

void serial_device::close()
{
        if (fd >= 0) {
                ::close (fd);
                fd = -1;
        }
}

int serial_device::read (uint8_t *buf, unsigned int len)
{
        if (fd < 0) {
                return -1;
        }

        size_t bytes_read = 0;
        ssize_t read_retval = -1;

        double maxwait = timeout;
        bool blocking_read = (timeout < 0);

        // While not interrupted in blocking mode
        while (bytes_read < len)
        {
                read_retval = ::read (fd, (void *)(buf + bytes_read), len - bytes_read);

                if (read_retval < 0) {
                        break;
                }

                bytes_read += read_retval;

                // Timeout while in non-blocking mode
                if (read_retval == 0 && !blocking_read) {
                        // no waiting
                        if (config.c_cc[VTIME] == 0)
                                break;

                        maxwait -= (double)config.c_cc[VTIME]/10.0;

                        // actual timeout
	                if (maxwait <= 0)
                                break;
	        }
        }

        return bytes_read;
}

int serial_device::write(uint8_t *buf, unsigned int len)
{
      if (fd < 0) {
              return -1;
      }

      int ret = ::write (fd, buf, len);
      if (ret > 0)
              byteswritten += ret;
      return ret;
}

int serial_device::set_timeout (double newtimeout)
{
        if (fd < 0) {
                return -1;
        }

        if (newtimeout < -1) {
                return -1;
        }

        timeout = newtimeout;

        // into 10ths of a second
        newtimeout *= 10;

        // Disable custom timeout, enable blocking read
        if (newtimeout < 0) {
                newtimeout = 5;
        }
        // Enable custom timeout, disable blocking read
        else {
               if (newtimeout > 5) newtimeout = 5;
        }

        if (config.c_cc[VTIME] != (unsigned char) newtimeout) {
                config.c_cc[VMIN] = 0;
                config.c_cc[VTIME] = (unsigned char) newtimeout; 

                if (tcsetattr (fd, TCSANOW, &config) < 0) {
                        return -1;
                }
        }

        return 0;
}

double serial_device::get_timeout (void) const
{
        if (timeout < 0)
                return -1;
        else
                return timeout;
}

int serial_device::set_stopbits (unsigned short stopbits)
{
  if (fd < 0)
    {
      return -1;
    }

  /*
   * CSTOPB Send two stop bits, else one.
   */

  if (stopbits == 1)
    {
      // Set to one stop bit
      BITMASK_CLEAR (config.c_cflag, CSTOPB);
    }
  else if (stopbits == 2)
    {
      // Set to two stop bits
      BITMASK_SET (config.c_cflag, CSTOPB);
    }
  else
    {
      return -1;
    }

  if (tcsetattr (fd, TCSANOW, &config) < 0)
    {
      return -1;
    }

  return 0;
}

int serial_device::get_stopbits (void) const
{
  if (fd < 0)
    {
        return -1;
    }

  if (BITMASK_CHECK (config.c_cflag, CSTOPB))
    return 2;
  else
    return 1;
}

int serial_device::set_databits (unsigned short bytesize)
{
  if (fd < 0)
    {
      return -1;
    }

  tcflag_t c_bytesize = 0;

  switch (bytesize)
    {
    case 5: c_bytesize = CS5; break;
    case 6: c_bytesize = CS6; break;
    case 7: c_bytesize = CS7; break;
    case 8: c_bytesize = CS8; break;

    default:
        return -1;
    }

  // Clear bitmask CSIZE
  BITMASK_CLEAR (config.c_cflag, CSIZE);

  // Apply new
  BITMASK_SET (config.c_cflag, c_bytesize);

  if (tcsetattr (fd, TCSANOW, &config) < 0)
    {
      return -1;
    }

  return 0;
}

int serial_device::get_databits (void) const
{
  if (fd < 0)
    {
      return -1;
    }

  int retval = -1;

  if (BITMASK_CHECK_VALUE (config.c_cflag, CSIZE, CS5))
    retval = 5;
  else if (BITMASK_CHECK_VALUE (config.c_cflag, CSIZE, CS6))
    retval = 6;
  else if (BITMASK_CHECK_VALUE (config.c_cflag, CSIZE, CS7))
    retval = 7;
  else if (BITMASK_CHECK_VALUE (config.c_cflag, CSIZE, CS8))
    retval = 8;

  return retval;
}

int serial_device::set_baudrate (unsigned int baud)
{
  if (fd < 0)
    {
      return -1;
    }

  speed_t baud_rate = 0;

  switch (baud)
    {
    case 0:
      baud_rate = B0; break;
    case 50:
      baud_rate = B50; break;
    case 75:
      baud_rate = B75; break;
    case 110:
      baud_rate = B110; break;
    case 134:
      baud_rate = B134; break;
    case 150:
      baud_rate = B150; break;
    case 200:
      baud_rate = B200; break;
    case 300:
      baud_rate = B300; break;
    case 600:
      baud_rate = B600; break;
    case 1200:
      baud_rate = B1200; break;
    case 1800:
      baud_rate = B1800; break;
    case 2400:
      baud_rate = B2400; break;
    case 4800:
      baud_rate = B4800; break;
    case 9600:
      baud_rate = B9600; break;
    case 19200:
      baud_rate = B19200; break;
    case 38400:
      baud_rate = B38400; break;
#ifdef B57600
    case 57600:
      baud_rate = B57600; break;
#endif
#ifdef B115200
    case 115200:
      baud_rate = B115200; break;
#endif
#ifdef B230400
    case 230400:
      baud_rate = B230400; break;
#endif
#ifdef B460800
    case 460800:
      baud_rate = B460800; break;
#endif
#ifdef B500000
    case 500000:
      baud_rate = B500000; break;
#endif
#ifdef B576000
    case 576000:
      baud_rate = B576000; break;
#endif
#ifdef B921600
    case 921600:
      baud_rate = B921600; break;
#endif
#ifdef B1000000
    case 1000000:
      baud_rate = B1000000; break;
#endif
#ifdef B1152000
    case 1152000:
      baud_rate = B1152000; break;
#endif
#ifdef B2000000
    case 2000000:
      baud_rate = B2000000; break;
#endif
#ifdef B3000000
    case 3000000:
      baud_rate = B3000000; break;
#endif
#ifdef B3500000
    case 3500000:
      baud_rate = B3500000; break;
#endif
#ifdef B4000000
    case 4000000:
      baud_rate = B4000000; break;
#endif
    default:
      return -1;
    }

  cfsetospeed (&config, baud_rate);
  cfsetispeed (&config, baud_rate);

  if (tcsetattr (fd, TCSANOW, &config) < 0) 
    {
	    return -1;
    }

  return 0;
}

int serial_device::get_baudrate (void) const
{
  if (fd < 0)
    {
      return -1;
    }

  int retval = -1;

  speed_t baudrate = cfgetospeed (&config);

  if (baudrate == B0)
    retval = 0;
  else if (baudrate == B50)
    retval = 50;
  else if (baudrate == B75)
    retval = 75;
  else if (baudrate == B110)
    retval = 110;
  else if (baudrate == B134)
    retval = 134;
  else if (baudrate == B150)
    retval = 150;
  else if (baudrate == B200)
    retval = 200;
  else if (baudrate == B300)
    retval = 300;
  else if (baudrate == B600)
    retval = 600;
  else if (baudrate == B1200)
    retval = 1200;
  else if (baudrate == B1800)
    retval = 1800;
  else if (baudrate == B2400)
    retval = 2400;
  else if (baudrate == B4800)
    retval = 4800;
  else if (baudrate == B9600)
    retval = 9600;
  else if (baudrate == B19200)
    retval = 19200;
  else if (baudrate == B38400)
    retval = 38400;
  else if (baudrate == B57600)
    retval = 57600;
  else if (baudrate == B115200)
    retval = 115200;
  else if (baudrate == B230400)
    retval = 230400;

  return retval;
}

int serial_device::flush (unsigned short queue_selector)
{
  if (fd < 0)
    {
      return -1;
    }

  /*
   * TCIOFLUSH Flush both pending input and untransmitted output.
   * TCOFLUSH Flush untransmitted output.
   * TCIFLUSH Flush pending input.
   */

  int flag;

  switch (queue_selector)
    {
    case 0: flag = TCOFLUSH; break;
    case 1: flag = TCIFLUSH; break;
    case 2: flag = TCIOFLUSH; break;
    default:
      return -1;
    }

  return ::tcflush (fd, flag);
}

int serial_device::sendbreak (unsigned short ms)
{
  if (fd < 0)
    {
      return -1;
    }

  return ::tcsendbreak (fd, ms);
}

int serial_device::set_parity (const std::string &newparity)
{
  if (fd < 0)
    {
      return -1;
    }

  // Convert string to lowercase
  std::string parity = newparity;
  std::transform (parity.begin (), parity.end (), parity.begin (), ::tolower);

  /*
   * PARENB Enable parity generation on output and parity checking for input.
   * PARODD If set, then parity for input and output is odd; otherwise even parity is used.
   */

  if (parity == "n" || parity == "none")
    {
      // Disable parity generation/checking
      BITMASK_CLEAR (config.c_cflag, PARENB);
    }
  else if (parity == "e" || parity == "even")
    {
      // Enable parity generation/checking
      BITMASK_SET (config.c_cflag, PARENB);

      // Set to Even
      BITMASK_CLEAR (config.c_cflag, PARODD);

    }
  else if (parity == "o" || parity == "odd")
    {
      // Enable parity generation/checking
      BITMASK_SET (config.c_cflag, PARENB);

      // Set to Odd
      BITMASK_SET (config.c_cflag, PARODD);

    }

  if (tcsetattr (fd, TCSANOW, &config) < 0) 
    {
	    return -1;
    }

  return 0;
}

std::string serial_device::get_parity (void) const
{
  if (!BITMASK_CHECK (config.c_cflag, PARENB))
    return "none";
  else if (BITMASK_CHECK (config.c_cflag, PARODD))
    return "odd";
  else
    return "even";
}

int serial_device::set_flowcontrol (const std::string &newctrl)
{
  if (fd < 0)
    {
      return -1;
    }

  // Convert string to lowercase
  std::string ctrl = newctrl;
  std::transform (ctrl.begin (), ctrl.end (), ctrl.begin (), ::tolower);

  if (ctrl == "n" || ctrl == "none")
    {
      BITMASK_CLEAR (config.c_iflag, IXON | IXOFF | IXANY);
#ifdef CNEW_RTSCTS
      BITMASK_CLEAR (config.c_cflag, CNEW_RTSCTS);
#else
      BITMASK_CLEAR (config.c_cflag, CRTSCTS);
#endif
    }
  else if (ctrl == "h" || ctrl == "hardware")
    {
      BITMASK_CLEAR (config.c_iflag, IXON | IXOFF | IXANY);
#ifdef CNEW_RTSCTS
      BITMASK_SET (config.c_cflag, CNEW_RTSCTS);
#else
      BITMASK_SET (config.c_cflag, CRTSCTS);
#endif
    }
  else if (ctrl == "s" || ctrl == "software")
    {
      BITMASK_CLEAR (config.c_iflag, IXANY);
      BITMASK_SET (config.c_iflag, IXON | IXOFF);
#ifdef CNEW_RTSCTS
      BITMASK_CLEAR (config.c_cflag, CNEW_RTSCTS);
#else
      BITMASK_CLEAR (config.c_cflag, CRTSCTS);
#endif
    }
  else
    {
      return -1;
    }

  return tcsetattr (fd, TCSANOW, &config);
}

std::string serial_device::get_flowcontrol (void) const
{
#ifdef CNEW_RTSCTS
  if (BITMASK_CHECK (config.c_cflag, CNEW_RTSCTS))
#else
  if (BITMASK_CHECK (config.c_cflag, CRTSCTS))
#endif
    return "hardware";
  else if (BITMASK_CHECK (config.c_iflag, IXON | IXOFF | IXANY))
    return "software";
  else
    return "none";
}

int serial_device::get_control_line_status (void)
{
  int status = 0;
  if (fd >= 0)
    {
        ioctl (fd, TIOCMGET, &status);
    }
  return status;
}

bool serial_device::get_control_line (const std::string &control_signal)
{
  int status = get_control_line_status ();

  if (control_signal == "DTR")
    return (status & TIOCM_DTR);
  else if (control_signal == "RTS")
    return (status & TIOCM_RTS);
  else if (control_signal == "CTS")
    return (status & TIOCM_CTS);
  else if (control_signal == "DSR")
    return (status & TIOCM_DSR);
  else if (control_signal == "CD")
    return (status & TIOCM_CD);
  else if (control_signal == "RI")
    return (status & TIOCM_RI);

  return 0;
}

int serial_device::set_control_line (const std::string &control_signal, bool set)
{

  int status = get_control_line_status ();

  int signal;

  if (control_signal == "DTR")
    signal = TIOCM_DTR;
  else if (control_signal == "RTS")
    signal = TIOCM_RTS;
  else
    {
      return -1;
    }

  if (set)
    status |= signal;
  else
    status &= ~signal;

  return ioctl (fd, TIOCMSET, &status);
}

int serial_device::get_numbytesavailable (void) const
{
  int available = 0;
  if (fd >=0)
    {
      ioctl (fd, FIONREAD, &available);
    }
  return available;
}


}
}
