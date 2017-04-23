#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include "i2c_master_noint.h"

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

#define SLAVE_ADDR 0x20

void initExpander() {
    //Set up GP0-3 as outputs and GP4-7 as inputs
    i2c_master_start(); //send start signal
    i2c_master_send((SLAVE_ADDR << 1)); //Send the address with a 0 bit to write
    i2c_master_send(0x00); //address register --> IODIR
    i2c_master_send(0b11110000); //set pins to outputs and inputs
    i2c_master_stop(); //Finished, stop the master
}

void setExpander(char level) {
    i2c_master_start(); //send start signal
    i2c_master_send((SLAVE_ADDR << 1)); //Send the address with a 0 bit to write
    i2c_master_send(0x0A);//select the OLAT register
    if (level) {
        i2c_master_send(1);//sets logic high on pin GP0
    }
    else {
        i2c_master_send(0);//sets logic low on pin GP0
    }
    
    i2c_master_stop();
}

char getExpander() {
    char master_read0;
    i2c_master_start();
    i2c_master_send((SLAVE_ADDR << 1)); //slave address followed by a write bit
    i2c_master_send(0x09); // register -> GPIO
    i2c_master_restart();
    i2c_master_send((SLAVE_ADDR << 1) | 1); //send with a read bit now
    master_read0 = i2c_master_recv(); //read the bit
    i2c_master_ack(1); //send a nack - master doesnt need more bytes
    i2c_master_stop(); // stop transmission
    
    //We want to read only GP7 though and return 1 if high 0 if low
    master_read0 = master_read0 >> 7;
    
    return master_read0;   
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
    i2c_master_setup();
    initExpander();
    
    __builtin_enable_interrupts();          

    while(1) {
        while(getExpander()) { //while the button is pushed (GP7 read LOW) LED is off (GP0 set LOW)
            setExpander(1);
        }
        setExpander(0);
 
    }
    
    return 0;
}

