#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>

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

#define CS LATAbits.LATA4

unsigned char spi_io(unsigned char o) {
    SPI1BUF = o;
    while(!SPI1STATbits.SPIRBF) {;}
    return SPI1BUF;
}

void init_SPI() {
    TRISAbits.TRISA4 = 0;
    CS = 1;
    
    SPI1CON = 0; //turn off SPI module and reset it
    SPI1BUF; //Clear the rx buffer by reading from it
    SPI1BRG = 0x1000; //
    SPI1STATbits.SPIROV = 0;  //Clear the overflow bit
    SPI1CONbits.CKE = 1;
    SPI1CONbits.MSTEN = 1;
    SPI1CONbits.ON = 1;
}

void setVoltage(int channel, unsigned char voltage) {
    CS = 0;
    if (channel == 1) { //send to input A 0111
        unsigned int part1 = 0b01110000;
        unsigned int part2 = (unsigned int)voltage >> 4;
        unsigned int result = part1 | part2;
        unsigned char data = spi_io((unsigned char)result);
        unsigned int result2 = (unsigned int)voltage << 4;
        data = spi_io((unsigned char)result2);
    }
    else { //send to input B 1111
        unsigned int part1 = 0b11110000;
        unsigned int part2 = (unsigned int)voltage >> 4;
        unsigned int result = part1 | part2;
        unsigned char data = spi_io((unsigned char)result);
        unsigned int result2 = (unsigned int)voltage << 4;
        data = spi_io((unsigned char)result2);  
    }
    CS = 1;
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here

    __builtin_enable_interrupts();
    
    //Map outputs
    RPA1Rbits.RPA1R = 0b0011; //map A1 to SD01
    init_SPI();
   
    //Create a square wave and sine wave
    unsigned char sine_wave[100];
    unsigned char tri_wave[100];
    int i = 0;
    for (i = 0;i < 100; i++) {
     sine_wave[i] = 255.0/2.0 + 255.0/2.0*sin(i*2.0*3.1415/100.0);
     tri_wave[i] = i*255.0/100.0;
    }
                   
    while(1) {
        for (i = 0; i < 100;i++) {            
            setVoltage(0,tri_wave[i]);
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() < 48000000/2/2000) {;}
            setVoltage(1,sine_wave[i]);           
            _CP0_SET_COUNT(0);
            while(_CP0_GET_COUNT() < 48000000/2/2000) {;}
        }             
    }
}