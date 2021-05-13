/**
 * SMuFF Firmware
 * Copyright (C) 2019 Technik Gegg
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
 *
 */
#pragma once

#include <stdlib.h>
#include <Arduino.h>

class StrPrint : Print {
private:
    String data;
public:
    virtual size_t write(uint8 ch) {
        data += (char)ch;
        return 1;
    };
    virtual size_t write(const char *str) {
        data += str;
        return strlen(str);
    };
    virtual size_t write(const void *buf, uint32 len) {
        char tmp[len];
        memcpy(tmp, buf, len);
        tmp[len] = 0;
        data += String(tmp);
        return len;
    };

    StrPrint() {
        data.reserve(80);
    }

    const char* toString() {
        return data.c_str();
    }

    const String& get() {
        return data;
    }
};