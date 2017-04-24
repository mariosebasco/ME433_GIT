#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "ILI9163C.h"
#include <stdio.h>


#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userubmit the link to your repid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module


int main() {
    
    //char buffer[100];    

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
            
    SPI1_init(); //Initialize the SPI communication
    LCD_init(); //Initialize LCD
       
    __builtin_enable_interrupts();          

    LCD_clearScreen(CYAN);
    
    char buffer[50];
    int counter = 0;
    int frames = 0;
    float FPS = 0;
    int time = _CP0_GET_COUNT();
    int frame_time = _CP0_GET_COUNT();
    
    
    while(1) {   
        
        sprintf(buffer,"Hello World! %d  ",counter);
        counter ++;
        LCD_writeString(buffer,110,96,BLACK); //print hello world string
        LCD_drawBar(counter, 110, 75,BLACK); // Print bar
        sprintf(buffer,"FPS: %.2f  ",FPS);
        LCD_writeString(buffer,90,60,RED);// print FPS
        frames ++;
        
        if (counter == 101) { // if 100 go back to 0
            counter = 0;            
        }
        
        FPS = ((float) frames)*24000000/(_CP0_GET_COUNT() - frame_time); // find the FPS 
        frames = 0;
        frame_time = _CP0_GET_COUNT();
        
        while(_CP0_GET_COUNT() < time + 48000000/10) {;}  //Maintain it at 5Hz      
        time = _CP0_GET_COUNT();
    }
    
    return 0;
}

