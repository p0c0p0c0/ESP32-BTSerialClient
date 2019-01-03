/*
	Copyright (c) 2019 Evan Harvey <evanwork1234@gmail.com>

	Permission is hereby granted, free of charge, to any person
	obtaining a copy of this software and associated documentation
	files (the "Software"), to deal in the Software without
	restriction, including without limitation the rights to use,
	copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the
	Software is furnished to do so, subject to the following
	conditions:

	The above copyright notice and this permission notice shall be
	included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
	OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef _BLUETOOTH_SERIAL_CLIENT_H_
#define _BLUETOOTH_SERIAL_CLIENT_H_

#include "sdkconfig.h"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)

#include "Arduino.h"
#include "Stream.h"

class BTSerialClient: public Stream
{
    public:

        BTSerialClient(void);
        ~BTSerialClient(void);

        bool begin(String localName=String(), String connectName=String());
        int available(void);
        int peek(void);
        bool hasClient(void);
        int read(void);
        size_t write(uint8_t c);
        size_t write(const uint8_t *buffer, size_t size);
        size_t print(const char *str);
        size_t println(const char *str);
        void flush();
        void end(void);

};

#endif

#endif
