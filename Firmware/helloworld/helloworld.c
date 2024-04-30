#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    char ch;

    while(true){
    ch = getchar();
    printf("Echo: %c\n", ch);
    }

    return 0;
}