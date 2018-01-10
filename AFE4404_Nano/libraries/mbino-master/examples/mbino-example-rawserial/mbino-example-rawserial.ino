#include "mbed.h"

RawSerial pc(USBTX, USBRX);

void setup() {
    pc.puts("Hello World!\r\n");
}

void loop() {
    // echo input back to terminal
    pc.putc(pc.getc() + 1);
}

#ifndef ARDUINO
int main() {
    setup();
    for (;;) {
        loop();
    }
}
#endif
