/**
  EUSART Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    eusart.c

  @Summary
    This is the generated driver implementation file for the EUSART driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  @Description
    This header file provides implementations for driver APIs for EUSART.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC16LF1559
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include "eusart.h"

/**
  Section: Macro Declarations
*/
#define EUSART_TX_BUFFER_SIZE 16
#define EUSART_RX_BUFFER_SIZE 16

/**
  Section: Global Variables
*/

volatile uint8_t eusartTxHead = 0;
volatile uint8_t eusartTxTail = 0;
volatile uint8_t eusartTxBuffer[EUSART_TX_BUFFER_SIZE];
volatile uint8_t eusartTxBufferRemaining;

volatile uint8_t eusartRxHead = 0;
volatile uint8_t eusartRxTail = 0;
volatile uint8_t eusartRxBuffer[EUSART_RX_BUFFER_SIZE];
volatile uint8_t eusartRxCount;

/**
  Section: EUSART APIs
*/

void EUSART_Initialize(void)
{
    // disable interrupts before changing states
    PIE1bits.RCIE = 0;
    PIE1bits.TXIE = 0;

    // Set the EUSART module to the options selected in the user interface.

    // SCKP Non-Inverted; BRG16 16bit_generator; WUE disabled; ABDEN enabled; 
    BAUDCON = 0x09;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
    RCSTA = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
    TXSTA = 0x24;

    // Baud Rate = 9600; SPBRGL 64; 
    SPBRGL = 0x40;

    // Baud Rate = 9600; SPBRGH 3; 
    SPBRGH = 0x03;


    // initializing the driver state
    eusartTxHead = 0;
    eusartTxTail = 0;
    eusartTxBufferRemaining = sizeof(eusartTxBuffer);

    eusartRxHead = 0;
    eusartRxTail = 0;
    eusartRxCount = 0;

    // enable receive interrupt
    PIE1bits.RCIE = 1;
}

uint8_t EUSART_Read(void)
{
    uint8_t readValue  = 0;
    
    while(0 == eusartRxCount)
    {
    }

    readValue = eusartRxBuffer[eusartRxTail++];
    if(sizeof(eusartRxBuffer) <= eusartRxTail)
    {
        eusartRxTail = 0;
    }
    PIE1bits.RCIE = 0;
    eusartRxCount--;
    PIE1bits.RCIE = 1;

    return readValue;
}

void EUSART_Write(uint8_t txData)
{
    while(0 == eusartTxBufferRemaining)
    {
    }

    if(0 == PIE1bits.TXIE)
    {
        TXREG = txData;
    }
    else
    {
        PIE1bits.TXIE = 0;
        eusartTxBuffer[eusartTxHead++] = txData;
        if(sizeof(eusartTxBuffer) <= eusartTxHead)
        {
            eusartTxHead = 0;
        }
        eusartTxBufferRemaining--;
    }
    PIE1bits.TXIE = 1;
}

char getch(void)
{
    return EUSART_Read();
}

void putch(char txData)
{
    EUSART_Write(txData);
}

void EUSART_Transmit_ISR(void)
{

    // add your EUSART interrupt custom code
    if(sizeof(eusartTxBuffer) > eusartTxBufferRemaining)
    {
        TXREG = eusartTxBuffer[eusartTxTail++];
        if(sizeof(eusartTxBuffer) <= eusartTxTail)
        {
            eusartTxTail = 0;
        }
        eusartTxBufferRemaining++;
    }
    else
    {
        PIE1bits.TXIE = 0;
    }
}

void EUSART_Receive_ISR(void)
{

    if(1 == RCSTAbits.OERR)
    {
        // EUSART error - restart

        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }

    // buffer overruns are ignored
    eusartRxBuffer[eusartRxHead++] = RCREG;
    if(sizeof(eusartRxBuffer) <= eusartRxHead)
    {
        eusartRxHead = 0;
    }
    eusartRxCount++;
}

void UART_SendSignedLong(int32_t value)
{
    uint8_t d = (uint8_t)0;

    if (value < 0)
    {
        putch('-');
        value = ~value;
        value++;
    }
    else
        putch(' ');

    while (value >= (int32_t)1000000000) { value -= (int32_t)1000000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int32_t) 100000000) { value -= (int32_t) 100000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int32_t)  10000000) { value -= (int32_t)  10000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int32_t)   1000000) { value -= (int32_t)   1000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int32_t)    100000) { value -= (int32_t)    100000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int32_t)     10000) { value -= (int32_t)     10000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int32_t)      1000) { value -= (int32_t)      1000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int32_t)       100) { value -= (int32_t)       100; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int32_t)        10) { value -= (int32_t)        10; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int32_t)         1) { value -= (int32_t)         1; d++; } putch(d + (uint8_t)0x30);
    putch((uint8_t)USART_DELIMITER);
}

void UART_SendLong(uint32_t value)
{
    uint8_t d = (uint8_t)0;

    while (value >= (uint32_t)1000000000) { value -= (uint32_t)1000000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint32_t) 100000000) { value -= (uint32_t) 100000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint32_t)  10000000) { value -= (uint32_t)  10000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint32_t)   1000000) { value -= (uint32_t)   1000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint32_t)    100000) { value -= (uint32_t)    100000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint32_t)     10000) { value -= (uint32_t)     10000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint32_t)      1000) { value -= (uint32_t)      1000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint32_t)       100) { value -= (uint32_t)       100; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint32_t)        10) { value -= (uint32_t)        10; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint32_t)         1) { value -= (uint32_t)         1; d++; } putch(d + (uint8_t)0x30);
    putch((uint8_t)USART_DELIMITER);
}

void UART_SendShortLong(uint24_t value)
{
    uint8_t d = (uint8_t)0;

    while (value >= (uint24_t)10000000) { value -= (uint24_t)10000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint24_t) 1000000) { value -= (uint24_t) 1000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint24_t)  100000) { value -= (uint24_t)  100000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint24_t)   10000) { value -= (uint24_t)   10000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint24_t)    1000) { value -= (uint24_t)    1000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint24_t)     100) { value -= (uint24_t)     100; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint24_t)      10) { value -= (uint24_t)      10; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint24_t)       1) { value -= (uint24_t)       1; d++; } putch(d + (uint8_t)0x30);
    putch((uint8_t)USART_DELIMITER);
}

void UART_SendSignedShortLong(int24_t value)
{
    uint8_t d = (uint8_t)0;

    if (value < 0)
    {
        putch('-');
        value = ~value;
        value++;
    }
    else
        putch(' ');

    while (value >= (int24_t)1000000) { value -= (int24_t)1000000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int24_t) 100000) { value -= (int24_t) 100000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int24_t)  10000) { value -= (int24_t)  10000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int24_t)   1000) { value -= (int24_t)   1000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int24_t)    100) { value -= (int24_t)    100; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int24_t)     10) { value -= (int24_t)     10; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int24_t)      1) { value -= (int24_t)      1; d++; } putch(d + (uint8_t)0x30);
    putch((uint8_t)USART_DELIMITER);
}

void UART_SendInt(uint16_t value)
{
    uint8_t d = (uint8_t)0;

    while (value >= (uint16_t)10000) { value -= (uint16_t)10000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint16_t) 1000) { value -= (uint16_t) 1000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint16_t)  100) { value -= (uint16_t)  100; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint16_t)   10) { value -= (uint16_t)   10; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint16_t)    1) { value -= (uint16_t)    1; d++; } putch(d + (uint8_t)0x30);
    putch((uint8_t)USART_DELIMITER);
}

void UART_SendSignedInt(int16_t value)
{
    uint8_t d = (uint8_t)0;

    if (value < 0)
    {
        putch('-');
        value = ~value;
        value++;
    }
    else
        putch(' ');

    while (value >= (int16_t)10000) { value -= (int16_t)10000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int16_t) 1000) { value -= (int16_t) 1000; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int16_t)  100) { value -= (int16_t)  100; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int16_t)   10) { value -= (int16_t)   10; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int16_t)    1) { value -= (int16_t)    1; d++; } putch(d + (uint8_t)0x30);
    putch((uint8_t)USART_DELIMITER);
}

void UART_SendChar(uint8_t value)
{
    uint8_t d = (uint8_t)0;

    while (value >= (uint8_t)100) { value -= (uint8_t)100; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint8_t) 10) { value -= (uint8_t) 10; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (uint8_t)  1) { value -= (uint8_t)  1; d++; } putch(d + (uint8_t)0x30);
    putch((uint8_t)USART_DELIMITER);
}

void UART_SendSignedChar(int8_t value)
{
    uint8_t d = (uint8_t)0;

    if (value < 0)
    {
        putch('-');
        value = ~value;
        value++;
    }
    else
        putch(' ');

    while (value >= (int8_t)100) { value -= (int8_t)100; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int8_t) 10) { value -= (int8_t) 10; d++; } putch(d + (uint8_t)0x30); d = (uint8_t)0;
    while (value >= (int8_t)  1) { value -= (int8_t)  1; d++; } putch(d + (uint8_t)0x30);
    putch((uint8_t)USART_DELIMITER);
}

#define UART_OUTPUT_NIBBLE(value, shift)                            \
    nibble = (uint8_t) value >> shift;                              \
    if (nibble <= 9) { nibble += 0x30; } else { nibble += 0x37; }   \
    putch(nibble);

void UART_SendChar_Hex(uint8_t value)
{
    uint8_t nibble;

    UART_OUTPUT_NIBBLE(value,         4);
    UART_OUTPUT_NIBBLE(value & 0x0F,  0);
}

void UART_SendNewLine(void)
{
    putch((uint8_t)'\r');
    putch((uint8_t)'\n');
}

//void UART_Receive_Service(void)
//{
//    static  uint8_t             buffer[2]           = {0,0};
//    static  enum UART_STATES    state               = UART_STATE_idle;
//    static  uint8_t             startAddress        = 0;
//    static  uint8_t             first               = 0;
//    static  uint8_t             count               = 0;
//    static  uint8_t             readCount           = 0;
//    static  uint8_t             i                   = 0;
//            uint8_t             data                = RCREG;
//
//    /* Overrun error detected */
//    if (OERR)
//    {
//        CREN = 0;                                   /* Reset the UART module    */
//        NOP();
//        NOP();
//        CREN = 1;
//
//        putch(UART_NACK);                    /* Send ASCII NACK to host  */
//        state = UART_STATE_read;
//        return;
//    }
//
//    switch(state)
//    {
//        case UART_STATE_idle:
//            count = 0;                              /* So far, no data received */
//            i     = 0;
//            first = 1;                              /* First byte incoming      */
//            if      (data == 'W') { state = UART_STATE_write; }
//            else if (data == 'R') { state = UART_STATE_read;  }
//            else                  { /* Error! */              }
//            break;
//
//        case UART_STATE_read:                        /* Receive read byte count*/
//            if (data == '\n')                     /* End of a packet and start to send data to host*/
//            {
//                for (i = 0; i < readCount; i++)
//                {
//                    UART_SendChar_Hex(MEMORY_Read((uint8_t)(startAddress + i)));    /* Send data */
//                }
//                UART_SendNewLine();
//                state = UART_STATE_idle;
//            }
//            else if (data != '\r')
//            {
//                buffer[count] = data;
//                count++;
//                if(count == 2)
//                {
//                    count = 0;
//                    readCount = UART_HexToBinary(buffer);
//                }
//            }
//            break;
//
//        case UART_STATE_write:
//           if      (data == '\n') { state = UART_STATE_idle; } /* End of packet */
//           else if (data == 'R')  { state = UART_STATE_read; } /* Read register after setting the start address */
//           else
//           {
//               buffer[count] = data;
//               count++;
//               if (count == 2)
//               {
//                    count = 0;
//
//                    if (first == 0)
//                    {
//                        MEMORY_Write((uint8_t)(startAddress + i), UART_HexToBinary(buffer));
//                        i++;
//                    }
//                    else
//                    {
//                        first = 0;
//                        startAddress = UART_HexToBinary(buffer);
//                    }
//               }
//           }
//           break;
//    }
//}

uint8_t UART_HexToBinary(uint8_t* data)
{
    uint8_t output = 0;
    
    if ((data[0] & 0x40) == 1) { output  = data[0] - 0x37; } /* data[0] is A-F */
    else                       { output  = data[0] - 0x30; } /* data[0] is 0-9 */
    
    output <<= 4;
    
    if ((data[1] & 0x40) == 1) { output += data[1] - 0x37; } /* data[0] is A-F */
    else                       { output += data[1] - 0x30; } /* data[0] is 0-9 */
    
    return output;
}
/**
  End of File
*/
