#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"
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

#define SLAVE_ADDR 0b1101010

void WHOAMI() {
    char master_read0;
    i2c_master_start(); // Send the start signal
    i2c_master_send((SLAVE_ADDR << 1)); //send the slave address
    i2c_master_send(0x0F); //Send the WHOAMI register
    i2c_master_restart(); //send restart to start reading
    i2c_master_send((SLAVE_ADDR << 1) | 1); //send with a read bit now
    master_read0 = i2c_master_recv(); //read the bit
    i2c_master_ack(1); //send a nack - master doesnt need more bytes
    i2c_master_stop(); // stop transmission
    LCD_writeChar(master_read0,100,100,BLACK);
}

void IMU_init() {
    //Need to change three registers
    i2c_master_start(); //send start signal
    i2c_master_send((SLAVE_ADDR << 1)); //Send the address with a 0 bit to write
    i2c_master_send(0x10); //address register --> CTRL1_XL
    i2c_master_send(0b10000010); //set CTRL1_XL to the right values
    i2c_master_stop(); //Finished, stop the master
    
    i2c_master_start(); //Now we're going to change the CTRL2_G register
    i2c_master_send((SLAVE_ADDR << 1));
    i2c_master_send(0x11);
    i2c_master_send(0b10001000);
    i2c_master_stop();   
    
    i2c_master_start(); //Now we're going to change the CTRL3_C register
    i2c_master_send((SLAVE_ADDR << 1));
    i2c_master_send(0x12);
    i2c_master_send(0x04);
    i2c_master_stop();
}

signed short concatenate(unsigned char LOW,unsigned char HIGH) {
    signed short concat_short;
    concat_short = (HIGH << 8) | LOW;
    
    return concat_short;
    
}

void LCD_print_array(signed short *array) { //Print out the numbers for debugging
    char buffer[100];
    sprintf(buffer,"AX: %.2f   ",array[4]*0.0061);
    LCD_writeString(buffer,110,110,BLACK);
    sprintf(buffer,"AY: %.2f   ",array[5]*0.0061);
    LCD_writeString(buffer,110,100,BLACK);
    sprintf(buffer,"AZ: %.2f   ",array[6]*0.0061);
    LCD_writeString(buffer,110,90,BLACK);
}

void LCD_print_IMU_bars(float AX, float AY) {
    LCD_drawBar(AX,64,64,BLACK,XDIR);
    LCD_drawBar(AY,64,64,BLACK,YDIR);
    /*
    char buffer[100];
    sprintf(buffer,"AX: %.2f   ",AX);
    LCD_writeString(buffer,110,110,BLACK);
     */
}
 

void IMU_read() {
    //Going to read all seven values -- temp, position, and angle
    //These values are each 16 bytes so we need to grab them as 8 bytes and then concatenate them
    unsigned char master_read_L;
    unsigned char master_read_H;
    signed short concat_short;
    signed short data_array[7];
    int i;
    i2c_master_start();
    i2c_master_send((SLAVE_ADDR << 1));
    i2c_master_send(0x20); //We are going to start by reading from the temp register
    i2c_master_restart();
    i2c_master_send((SLAVE_ADDR << 1) | 1); //Gonna start reading
    for (i = 0;i < 14; i++) {
        if (i%2 == 0) { // if zero you are reading LOW data
            master_read_L = i2c_master_recv();
            i2c_master_ack(0); //Keep sending bytes
        }
        else { //you are reading HIGH data
            master_read_H = i2c_master_recv();
            concat_short = concatenate(master_read_L,master_read_H);
            data_array[i/2] = concat_short;
            if (i == 13) {
                i2c_master_ack(1); //master wants no more readings
            }
            else {
                i2c_master_ack(0); //Keep sending data
            }
        }
    }
    
    i2c_master_stop();
    //Now we have an array of shorts let's print them to the LCD screen
    //LCD_print_array(data_array); //Used it for testing and debugging
    
    //Finally we can draw the bars on the screen
    LCD_print_IMU_bars(data_array[4]*0.0061, data_array[5]*0.0061);

}


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

    //Set up I2C
    SPI1_init(); //Initialize the SPI communication for LCD
    i2c_master_setup(); //Initialize I2C for IMU
    LCD_init(); //Initialize LCD
    IMU_init(); //Initialize IMU
    
    __builtin_enable_interrupts();
    
    LCD_clearScreen(CYAN);
    
    //WHOAMI(); //Works well
    int time;
    time = _CP0_GET_COUNT();
    //unsigned char debug1;
    //unsigned char debug2;
    //char buffer[100];
    
    while(1) {
        IMU_read();
        /*
        i2c_master_start();
        i2c_master_send((SLAVE_ADDR << 1));
        i2c_master_send(0x2C); //We are going to start by reading from the AZ register
        i2c_master_restart();
        i2c_master_send((SLAVE_ADDR << 1) | 1);
        debug1 = i2c_master_recv();
        i2c_master_ack(1);
        i2c_master_stop();
        
        i2c_master_start();
        i2c_master_send((SLAVE_ADDR << 1));
        i2c_master_send(0x2D); //We are going to start by reading from the AZ register
        i2c_master_restart();
        i2c_master_send((SLAVE_ADDR << 1) | 1);
        debug2 = i2c_master_recv();
        i2c_master_ack(1);
        i2c_master_stop();
        
        signed short AZ = ((debug2 << 8) | debug1);
        
        sprintf(buffer,"AZ: %d   ",AZ);
        LCD_writeString(buffer,110,110,BLACK);
         */        
        
        
        while ((_CP0_GET_COUNT() - time) < 48000000/10) {;} //Keep it at 5HZ
        time = _CP0_GET_COUNT();
    }
    
    return 0;
}

