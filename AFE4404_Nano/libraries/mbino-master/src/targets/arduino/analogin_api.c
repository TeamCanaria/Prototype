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
#include "hal/analogin_api.h"

#include <Arduino.h>

void analogin_init(analogin_t *obj, PinName pin)
{
#ifdef ADC_RESOLUTION
    analogReadResolution(ADC_RESOLUTION);
#endif
    obj->pin = pin;
}

uint16_t analogin_read_u16(analogin_t *obj)
{
    uint16_t value = analogRead(obj->pin);
#ifdef ADC_RESOLUTION
    return (value << (16 - ADC_RESOLUTION)) | (value >> (2 * ADC_RESOLUTION - 16));
#else
    return (value << 6) | (value >> 4);  // default is 10 bits
#endif
}
