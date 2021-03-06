/* mbino - basic mbed APIs for the Arduino platform
 * Copyright (c) 2017 Thomas Kemmer
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you
 * may not use this file except in compliance with the License.  You
 * may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 */
#ifndef MBINO_I2C_OBJECT_H
#define MBINO_I2C_OBJECT_H

// HACK to avoid pulling in the whole Wire library stuff...

#if defined(WIRE_HAS_END)

// assume Wire.h was already included, the global Wire object is
// available, and this is all C++ from now on...

#include "hal/i2c_api.h"
#include "platform/mbed_error.h"
#include "../common_objects.h"

extern "C" {
    inline void i2c_init(i2c_t* obj, PinName sda, PinName scl) {
        if (sda == I2C_SDA && scl == I2C_SCL) {
            obj->wire = &Wire;
        } else {
            error("I2C pin mapping failed");
        }
        obj->wire->begin();
    }

    inline void i2c_frequency(i2c_t* obj, long hz) {
        obj->wire->setClock(hz);
    }

    inline int i2c_read(i2c_t* obj, int address, char* data, int length, int stop) {
        // FIXME: nread > length?
        uint8_t nread = obj->wire->requestFrom(address >> 1, length, stop);
        int n = 0;
        // FIXME: available() < nread?
        while (n != nread && obj->wire->available()) {
            data[n++] = obj->wire->read();
        }
        return n;
    }

    inline int i2c_write(i2c_t* obj, int address, const char* data, int length, int stop) {
        obj->wire->beginTransmission(address >> 1);
        int n = obj->wire->write(data, length);
        // FIXME: error mapping?
        switch (obj->wire->endTransmission(stop)) {
        case 0:
            return n;
        case 1:
            return n;  // ???
        case 2:
            return -1;
        case 3:
            return -1;  // ???
        default:
            return -2;  // ???
        }
    }

    // TODO: other i2c functions
}

#endif

#endif
