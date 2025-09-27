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

#ifndef librevisa_serial_device_h_
#define librevisa_serial_device_h_

#include <string>
#include <termios.h>
#include <cstdint>

namespace librevisa {
namespace serial {

class serial_device
{
public:
        serial_device(const std::string &dev);
        virtual ~serial_device();

        void close(void);

      int write(uint8_t* /* buffer */, unsigned int /* buffer size */);
      int read(uint8_t* /* buffer */, unsigned int /* buffer size */);

      int flush(unsigned short /* stream select */);
      int sendbreak(unsigned short /* ms */);

      int set_timeout(double /* timeout */);
      double get_timeout(void) const;

      int set_baudrate(unsigned int /* baudrate */);
      int get_baudrate(void) const;

      int set_databits(unsigned short /* size */);
      int get_databits(void) const;

      int set_parity(const std::string& /* parity */);
      std::string get_parity() const;

      int set_flowcontrol(const std::string& /* flow */);
      std::string get_flowcontrol() const;

      int set_stopbits(unsigned short /* stop bits */);
      int get_stopbits(void) const;

      bool get_control_line(const std::string &);
      int set_control_line(const std::string &, bool);
      int get_control_line_status(void);

      int get_numbytesavailable(void) const;

      std::string get_name() const { return name; }

private:
      std::string name;

      int fd;
      struct termios config;
      double timeout;
      unsigned long byteswritten;
};


}
}

#endif
