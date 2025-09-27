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

#ifndef librevisa_serial_resource_h_
#define librevisa_serial_resource_h_ 1

#include "instrument_resource.h"

//#include <libserial.h>
#include "serial_device.h"

//#include "serial_string.h"
#include <string>
//#include <cstdint>

namespace librevisa {
namespace serial {

class serial_resource :
        public instrument_resource
{
private:
        serial_resource(const std::string &devname);
        ~serial_resource() throw();

        virtual ViStatus Write(ViBuf, ViUInt32, ViUInt32 *);
        virtual ViStatus Read(ViBuf, ViUInt32, ViUInt32 *);
        virtual ViStatus Close();
        virtual ViStatus ReadSTB(ViUInt16 *);

        virtual ViStatus GetAttribute(ViAttr, void *);
        virtual ViStatus SetAttribute(ViAttr, ViAttrState);

	serial_device *device;

        class creator;
};

}
}

#endif
