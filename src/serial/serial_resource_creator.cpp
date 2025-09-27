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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "serial_resource_creator.h"

#include "resource_manager.h"

#include "findlist.h"

#include "util.h"
#include "exception.h"

#include <dirent.h>
#include <sys/stat.h>

#include <sstream>
#include <iomanip>
#include <cstdlib>

namespace librevisa {
namespace serial {

serial_resource::creator::creator()
{
        default_resource_manager.register_creator(*this);
}

serial_resource::creator::~creator() throw()
{
        default_resource_manager.unregister_creator(*this);
}

resource *serial_resource::creator::create(std::vector<std::string> const &components) const
{
        // two options here:
        // 1. we are passed a ASRLX string
        // 2. we are passed a alias (in the case /dev/ttyXXXXX ?

        std::string const &transp = components[0];

	//  regardless of what passed we should see at least 4 characters?
        if(transp.size() < 4)
            return 0;

        if ( ((transp[0] | 0x20) != 'a' ||
              (transp[1] | 0x20) != 's' ||
              (transp[2] | 0x20) != 'r' ||
              (transp[3] | 0x20) != 'l') &&
             ((transp[0] != '/') ||
              (transp[1] != 'd') ||
              (transp[2] != 'e') ||
              (transp[3] != 'v'))) {

                return 0;
	}

        // get list of devices we know about
        std::list<device_info> devs;
        list_devices(devs);

        // now look for a match
        if ((transp[0] | 0x20) == 'a' &&
            (transp[1] | 0x20) == 's' &&
            (transp[2] | 0x20) == 'r' &&
            (transp[3] | 0x20) == 'l') {

                // look via index
 
                char const *cursor = transp.data() + 4;

                int devicenum = atoi(cursor);
                int cnt = 0;
                for (std::list<device_info>::const_iterator it=devs.begin();it!=devs.end();++it) {
                        cnt ++;

                        if (cnt == devicenum) {
                                const device_info &dev = *it;
                               return new serial_resource(dev.device_name);
                        }
                }

        } else {
                // device name
                for (std::list<device_info>::const_iterator it=devs.begin();it!=devs.end();++it) {
                        // create and return
                        const device_info &dev = *it;

                        if(dev.device_name == transp) {
                                return new serial_resource(dev.device_name);
                        }
                }
        }

	return 0;
}

void serial_resource::creator::list_devices(std::list<device_info> &devs) const
{
        DIR *dir;
        struct dirent *ent;
        int cnt = 0;

        // if have /sys/class/tty/DEVICE/device
        //   is it a link -> is link = True
        //  devpath = /sys/class/tty/DEVICE/device  ** get real path
        //  subsystem = base name of real file devpath/subsystem
        // if subsystem = usb-serial _. read from interface path, else read from devpath

        if ((dir = opendir("/sys/class/tty")) != NULL) {
                while ((ent = readdir(dir)) != NULL) {
                        std::string name = ent->d_name;
		        bool is_serial = false;
                        if (name.rfind("ttyS") || name.rfind("ttyUSB") || name.rfind("ttyACM")) {
                                std::string path = "/sys/class/tty/" + name + "/device";
                                struct stat st;
                                if (stat(path.c_str(), &st) == 0) {
                                        char *rname = realpath((path + "/subsystem").c_str(), NULL);
                                        std::string subsystem = rname ? rname : "";
                                        subsystem = subsystem.substr(subsystem.find_last_of('/')+1);
                                        free(rname);

                                        if (subsystem == "serial_base") {
                                                // built in serial
                                                is_serial = true;
                                        }
                                        if (subsystem == "usb-serial")  {
                                                // use real base name of path
                                                path = path + "/../";
                                                is_serial = true;
                                        }
                                        if (subsystem == "usb") {
                                               // use path
                                               is_serial = true;
                                        }


                                        if (is_serial) {
                                                cnt ++;
                                                std::ostringstream ss;
                                                ss << "ASRL";
                                                ss << cnt;
                                                ss << "::";
				                ss << "INSTR";

                                                devs.push_back(device_info("/dev/" + name, ss.str(), name));
                                        }
                                }
                        }
                }
                closedir(dir);
        }
}

void serial_resource::creator::find(findlist &list) const
{
        std::list<device_info> devs;
        list_devices(devs);
        for (std::list<device_info>::const_iterator it=devs.begin();it!=devs.end();++it) {
                list.add((*it).resource_string);
        }
}

serial_resource::creator const serial_resource::creator::instance;

}
}
