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

#ifndef librevisa_serial_resource_creator_h_
#define librevisa_serial_resource_creator_h_ 1

#include "serial_resource.h"
#include "resource_creator.h"

#include  <list>

namespace librevisa {
namespace serial {

class serial_resource::creator :
        public resource_creator
{
    private:
        creator();
        ~creator() throw();

        virtual resource *create(std::vector<std::string> const &) const;
        virtual void find(findlist &) const;

        class device_info {
            public:
                device_info(const std::string &n, const std::string &r, const std::string &a) {
                    device_name = n;
                    resource_string = r;
                    alias = a;
                }
                virtual ~device_info() {}
                std::string device_name;
                std::string resource_string;
                std::string alias;
        };

        void list_devices(std::list<device_info> &devs) const;

        static creator const instance;
};

}
}

#endif
