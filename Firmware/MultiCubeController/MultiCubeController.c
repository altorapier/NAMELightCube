#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"



#define outputPin 5
#define clkPin 1
#define loadPin 4

#define layerSelA 6
#define layerSelB 7
#define layerSelC 8
#define layerSelD 9
#define layerSelE 11
#define layerSelF 10


//Cube data pins
#define dataPin1 12
#define dataPin2 13
#define dataPin3 14
#define dataPin4 15
#define dataPin5 16
#define dataPin6 17
#define dataPin7 18
#define dataPin8 19
#define dataPin9 20

// Define the 1 frames, each 24 by 24 by 32 pixels and 3 colors deep
bool frame[24][24][32][3] = {false};

void OutputFrame(){

    uint32_t led = 0;
    const uint32_t mask = 0b00000000000111111111000000000000;

    const int ColSeq[3] = {1,2,0};
    int c;

    while(1){
    

    for(int k = 0; k < 32; k++){

        // Load new layer data into the shift registers
        for(int cn = 0; cn < 3; cn++){
        for(int j = 0; j < 8; j++){
            for(int i = 0; i < 8; i++){

                c = ColSeq[cn];

                //Set all the data pins

                led = 0;

                led += frame[i   ][j   ][k][c] << 0;
                led += frame[i+ 8][j   ][k][c] << 1;
                led += frame[i+16][j   ][k][c] << 2;

                led += frame[i   ][j+ 8][k][c] << 3;
                led += frame[i+ 8][j+ 8][k][c] << 4;
                led += frame[i+16][j+ 8][k][c] << 5;

                led += frame[i   ][j+16][k][c] << 6;
                led += frame[i+ 8][j+16][k][c] << 7;
                led += frame[i+16][j+16][k][c] << 8;

                led = led << 12;

                gpio_put_masked(mask,led);
                

                busy_wait_at_least_cycles(10); // very short 10 cycle delay 10*8ns per cycle, ~80ns

                //Pulse clk pin for at least 20ns
                gpio_put(clkPin,1);
                busy_wait_at_least_cycles(10);
                gpio_put(clkPin,0);
                busy_wait_at_least_cycles(10);

            }
        }
        }

        //Turn off output while setting layer selector pins
        gpio_put(outputPin,1);
        busy_wait_at_least_cycles(10);

        //Configure vertical column pins selection
        if( (k & 0b00000001) != 0 ) {gpio_put(layerSelA,1);}
                                else{gpio_put(layerSelA,0);}

        if( (k & 0b00000010) != 0 ) {gpio_put(layerSelB,1);}
                                else{gpio_put(layerSelB,0);}

        if( (k & 0b00000100) != 0 ) {gpio_put(layerSelC,1);}
                                else{gpio_put(layerSelC,0);}

        if( (k & 0b00001000) != 0 ) {gpio_put(layerSelD,1);}
                                else{gpio_put(layerSelD,0);}

        // slightly different because disabling and enabling multiplexer IO chips
        if( (k & 0b00010000) != 0 ){
            gpio_put(layerSelE,0);
            gpio_put(layerSelF,1);
        }
        else{
            gpio_put(layerSelE,1);
            gpio_put(layerSelF,0);
        }

        //Pulse load pin to move inputted data to display out
        gpio_put(loadPin,1);
        busy_wait_at_least_cycles(10);
        gpio_put(loadPin,0);

        // Enable output again
        gpio_put(outputPin,0);

        //Limit Framerate, display current layer for 1ms
    }


    }

}

int main() {

    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    gpio_init(clkPin);
    gpio_set_dir(clkPin, GPIO_OUT);

    gpio_init(dataPin1);
    gpio_set_dir(dataPin1, GPIO_OUT);
    gpio_init(dataPin2);
    gpio_set_dir(dataPin2, GPIO_OUT);
    gpio_init(dataPin3);
    gpio_set_dir(dataPin3, GPIO_OUT);
    gpio_init(dataPin4);
    gpio_set_dir(dataPin4, GPIO_OUT);
    gpio_init(dataPin5);
    gpio_set_dir(dataPin5, GPIO_OUT);
    gpio_init(dataPin6);
    gpio_set_dir(dataPin6, GPIO_OUT);
    gpio_init(dataPin7);
    gpio_set_dir(dataPin7, GPIO_OUT);
    gpio_init(dataPin8);
    gpio_set_dir(dataPin8, GPIO_OUT);
    gpio_init(dataPin9);
    gpio_set_dir(dataPin9, GPIO_OUT);

    gpio_put(dataPin1,0);
    gpio_put(dataPin2,0);
    gpio_put(dataPin3,0);
    gpio_put(dataPin4,0);
    gpio_put(dataPin5,0);
    gpio_put(dataPin6,0);
    gpio_put(dataPin7,0);
    gpio_put(dataPin8,0);
    gpio_put(dataPin9,0);

    gpio_init(loadPin);
    gpio_set_dir(loadPin, GPIO_OUT);

    gpio_init(outputPin);
    gpio_set_dir(outputPin, GPIO_OUT);

    gpio_init(layerSelA);
    gpio_set_dir(layerSelA, GPIO_OUT);
    gpio_init(layerSelB);
    gpio_set_dir(layerSelB, GPIO_OUT);
    gpio_init(layerSelC);
    gpio_set_dir(layerSelC, GPIO_OUT);
    gpio_init(layerSelD);
    gpio_set_dir(layerSelD, GPIO_OUT);
    gpio_init(layerSelE);
    gpio_set_dir(layerSelE, GPIO_OUT);
    gpio_init(layerSelF);
    gpio_set_dir(layerSelF, GPIO_OUT);

    gpio_put(clkPin,0);
    gpio_put(loadPin,0);
    gpio_put(outputPin,1); //Set pin high to initially have display turned off

    gpio_put(layerSelA,0);
    gpio_put(layerSelB,0);
    gpio_put(layerSelC,0);
    gpio_put(layerSelD,0);
    gpio_put(layerSelE,1); // Active low
    gpio_put(layerSelF,0); // Active low


    //Start the display
    multicore_launch_core1(OutputFrame);


    int i = 0;
    int j = 0;
    int k = 0;

    char ch0;
    char ch1;
    char ch2;

     while(1){


        //get a character from the serial buffer
        ch0 = getchar();

        // This character should not appear in normal update commands
        if(0b01111111 == ch0){
            // Clear whole cube command
            for(int k = 0; k < 32; k++){
                for(int j = 0; j < 24; j++){
                    for(int i = 0; i < 24; i++){
                        frame[i][j][k][0] = false;
                        frame[i][j][k][1] = false;
                        frame[i][j][k][2] = false;

                    }
                }
            }
        }
        // Check if ch0 is the first byte in the 3 byte LED update
        else if(0b01000000 & ch0){
            
            ch1 = getchar();
            ch2 = getchar();

            i = ch0 & 0b00011111;
            j = ch1 & 0b00011111;
            k = ch2 & 0b00011111;

            frame[i][j][k][0] = ch0>>5 & 0b00000001;
            frame[i][j][k][1] = ch1>>5 & 0b00000001;
            frame[i][j][k][2] = ch2>>5 & 0b00000001;

        }


     }

    return 0;
}