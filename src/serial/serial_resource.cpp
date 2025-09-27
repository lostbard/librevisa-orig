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

#include <string.h>

#include "serial_resource.h"

#include "exception.h"
#include <cstring>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

namespace librevisa {
namespace serial {

serial_resource::serial_resource(const std::string &devname) :
        device(0)
{
        device = new serial_device(devname);
}

serial_resource::~serial_resource() throw()
{
	if (device) {
                delete device;
	}
}

ViStatus serial_resource::Close()
{
        delete this;
        return VI_SUCCESS;
}

ViStatus serial_resource::GetAttribute(ViAttr attr, void *attrState)
{
        switch(attr) {
		case VI_ATTR_INTF_TYPE:
			{
                                   *reinterpret_cast<ViUInt16*>(attrState) = VI_INTF_ASRL;
				   break;
			}
		case VI_ATTR_ASRL_BAUD:
			{
				int baud = device->get_baudrate();
				if(baud > 0)
				{
                                   *reinterpret_cast<ViUInt32*>(attrState) = baud;
				}
				break;
			}
		case VI_ATTR_ASRL_STOP_BITS:
			{
				int stopbits = device->get_stopbits();
				if(stopbits >= 0)
				{
                                   *reinterpret_cast<ViUInt16*>(attrState) = stopbits;
				}
				break;
			}

		case VI_ATTR_ASRL_DATA_BITS:
			{
				int databits = device->get_databits();
				if(databits > 0)
				{
                                   *reinterpret_cast<ViUInt16*>(attrState) = databits;
				}
				break;
			}
		case VI_ATTR_ASRL_PARITY:
			{
                                std::string parity = device->get_parity();
                                if (parity == "none") *reinterpret_cast<ViUInt16*>(attrState) = VI_ASRL_PAR_NONE;
                                if (parity == "odd") *reinterpret_cast<ViUInt16*>(attrState) = VI_ASRL_PAR_ODD;
                                if (parity == "even") *reinterpret_cast<ViUInt16*>(attrState) = VI_ASRL_PAR_EVEN;
				break;
			}
		case VI_ATTR_ASRL_AVAIL_NUM:
			{
				int avail = device->get_numbytesavailable();
				if(avail >= 0)
				{
                                   *reinterpret_cast<ViUInt32*>(attrState) = avail;
				}
				break;
			}
		case VI_ATTR_INTF_INST_NAME:
			{
				std::string name = device->get_name();
				char * inst_name = reinterpret_cast<ViChar*>(attrState);
				int len = name.length();
				if (len > VI_FIND_BUFLEN-1)
			                len = VI_FIND_BUFLEN-1;
				memcpy(inst_name, name.c_str(), len);
				inst_name[len] = '\0';
				break;
			}
                default:
                        return resource::GetAttribute(attr, attrState);
        }
        return VI_SUCCESS;
}

ViStatus serial_resource::SetAttribute(ViAttr attr, ViAttrState attrState)
{
        switch(attr) {
                // any we handle
                default:
                        return resource::SetAttribute(attr, attrState);
        }

        return VI_SUCCESS;
}

ViStatus serial_resource::Write(ViBuf buf, ViUInt32 size, ViUInt32 *result)
{
	if (device) {
		int wrote = device->write(buf, size);
		if (wrote >= 0) {
			*result = wrote;
                        return VI_SUCCESS;
		}
	}
        return VI_ERROR_IO;
}

ViStatus serial_resource::Read(ViBuf payload_buf, ViUInt32 payload_buf_size, ViUInt32 *result)
{
	if (device) {
		int read = device->read(payload_buf, payload_buf_size);

	        if(read >= 0) {
        		*result=read;
                        return VI_SUCCESS;

	        }

	}

        return VI_ERROR_IO;
}

ViStatus serial_resource::ReadSTB(ViUInt16 *retStatus)
{
        return VI_ERROR_IO;
}


}
}
