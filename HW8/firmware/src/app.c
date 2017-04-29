/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "i2c_master_noint.h"
#include "ILI9163C.h"
#include <stdio.h>

#define SLAVE_ADDR 0b1101010


APP_DATA appData;


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





void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    SPI1_init(); //Initialize the SPI communication for LCD
    i2c_master_setup(); //Initialize I2C for IMU
    LCD_init(); //Initialize LCD
    IMU_init(); //Initialize IMU
    
    LCD_clearScreen(CYAN);
     
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {                        
            int time;
            time = _CP0_GET_COUNT();
            
            IMU_read();
            
            while ((_CP0_GET_COUNT() - time) < 48000000/10) {;} //Keep it at 5HZ            
            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
