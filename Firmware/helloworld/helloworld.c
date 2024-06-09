#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    //stdio_usb_init();
    stdio_init_all();

    char ch;

    u_int16_t N = 0;

    while(true){
    ch = getchar();
    //wait for the overflow at 2**16 bytes
    if(N == 0){
        printf("0");
    }
    N++;
    }

    return 0;
}