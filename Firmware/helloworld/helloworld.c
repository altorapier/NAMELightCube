#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    //stdio_usb_init();
    stdio_init_all();

    char ch;

    u_int16_t N = 0;

    while(true){
    ch = getchar();
    if(N == 0){
        printf("0");
    }
    N++;
    }

    return 0;
}