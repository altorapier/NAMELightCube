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

#define dataPin 3

// Define the 2 frames, each 8 by 8 by 32 pixels and 3 colors deep
bool frame[2][8][8][32][3] = {false};

int frameSelect = 0;

void OutputFrame(){

    bool led = false;

    const int ColSeq[3] = {1,2,0};
    int c;

    while(1){
    

    for(int k = 0; k < 32; k++){
        // Load new layer data into the shift registers
        for(int cn = 0; cn < 3; cn++){
        for(int j = 0; j < 8; j++){
            for(int i = 0; i < 8; i++){

                c = ColSeq[cn];
                led = frame[frameSelect][i][j][k][c];

                led = led; // Led on with low signal

                gpio_put(dataPin,led);
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

    gpio_init(dataPin);
    gpio_set_dir(dataPin, GPIO_OUT);

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
    gpio_put(dataPin,0);
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

    char ch;

    int frameEdit = 1;

     while(1){


        //get a character from the serial buffer
        ch = getchar();

        // define first bit as no data, starting, reset

        if(0b10000000 & ch){
            i = 0;
            j = 0;
            k = 0;
        }
        else{

            frame[frameEdit][i][j][k][0] = ch>>0 & 0b00000001;
            frame[frameEdit][i][j][k][1] = ch>>1 & 0b00000001;
            frame[frameEdit][i][j][k][2] = ch>>2 & 0b00000001;
            
            i++;

            frame[frameEdit][i][j][k][0] = ch>>3 & 0b00000001;
            frame[frameEdit][i][j][k][1] = ch>>4 & 0b00000001;
            frame[frameEdit][i][j][k][2] = ch>>5 & 0b00000001;

            i++;

            if(i==8){
            i = 0;
            j++;
            }
            if(j==8){
                j = 0;
                k++;
            }
            if(k==32){
                k = 0;
                if(frameSelect == 0){
                    frameSelect = 1;
                    frameEdit = 0;
                }
                else{
                    frameSelect = 0;
                    frameEdit = 1;
                }
            }

            //frameSelect
            //frame[2][8][8][32][3]

        }


     }

    return 0;
}