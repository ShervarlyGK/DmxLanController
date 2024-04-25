
#ifndef __INC_DMX_H
#define __INC_DMX_H

#define __AVR_LIBC_DEPRECATED_ENABLE__ 1

#include <EEPROM.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#if ARDUINO >= 100
  #include "Arduino.h"
#else
   #include "WProgram.h"
#endif
#include <Ethernet3.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <stdlib.h>

//#define        USE_INTERBYTE_DELAY     // rare cases of equipment non full DMX-512 compliant, need this

// *** comment UARTs not used ***
//#define        USE_UART0
//#define        USE_UART1
#define        USE_UART2
#define        USE_UART3

#define RxNotPermanent
#define TxNotPermanent

// New DMX modes *** EXPERIMENTAL ***
#define        DMX512            (0)    // DMX-512 (250 kbaud - 512 channels) Standard USITT DMX-512
#define        DMX1024           (1)    // DMX-1024 (500 kbaud - 1024 channels) Completely non standard - TESTED ok
#define        DMX2048           (2)    // DMX-2048 (1000 kbaud - 2048 channels) called by manufacturers DMX1000K, DMX 4x or DMX 1M ???

// DMX-512  (250 kbaud - 512 channels) Standard USITT DMX-512
#define        IBG_512           (10)                      // interbyte gap [us]
#define        DMX_512           ((F_CPU/(250000*8))-1)    // 250 kbaud
#define        BREAK_512         ( F_CPU/(100000*8))       // 90.9 kbaud

// DMX-1024 (500 kbaud - 1024 channels) Completely non standard
#define        IBG_1024          (5)                       // interbyte gap [us]
#define        DMX_1024          ((F_CPU/(500000*8))-1)    // 500 kbaud
#define        BREAK_1024        ( F_CPU/(200000*8))       // 181.8 kbaud

// DMX-2048 (1000 kbaud - 2048 channels) Non standard, but used by manufacturers as DMX1000K or DMX-4x or DMX 1M ???
#define        IBG_2048          (2)                       // interbyte gap [us] + nop's to reach 2.5 uS
#define        DMX_2048          ((F_CPU/(1000000*8))-1)   // 1000 kbaud
#define        BREAK_2048        ( F_CPU/(400000*8))       // 363.6 kbaud

// Inline assembly: do nothing for one clock cycle.
#define        nop()             __asm__ __volatile__("nop")

#ifdef __cplusplus
extern "C" { 
#endif
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #if defined(USE_UART0)
      void SIG_USART0_RECV  (void) __attribute__((__always_inline__));
      void SIG_USART0_TRANS (void) __attribute__((__always_inline__));
    #endif
    #if defined(USE_UART1)
      void SIG_USART1_RECV  (void) __attribute__((__always_inline__));
      void SIG_USART1_TRANS (void) __attribute__((__always_inline__));
    #endif
    #if defined(USE_UART2)  
      void SIG_USART2_RECV  (void) __attribute__((__always_inline__));
      void SIG_USART2_TRANS (void) __attribute__((__always_inline__));
    #endif
    #if defined(USE_UART3)  
      void SIG_USART3_RECV  (void) __attribute__((__always_inline__));
      void SIG_USART3_TRANS (void) __attribute__((__always_inline__));
    #endif
  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    #if defined(USE_UART0)
      void USART_RX_vect    (void) __attribute__((__always_inline__));
      void USART_TX_vect    (void) __attribute__((__always_inline__));
    #endif
  #endif
#ifdef __cplusplus
};
#endif

class CArduinoDmx 
{ 
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #if defined(USE_UART0)
      friend void SIG_USART0_RECV  (void);
      friend void SIG_USART0_TRANS (void);
    #endif
    #if defined(USE_UART1)
      friend void SIG_USART1_RECV  (void);
      friend void SIG_USART1_TRANS (void);
    #endif
    #if defined(USE_UART2)  
      friend void SIG_USART2_RECV  (void);
      friend void SIG_USART2_TRANS (void);
    #endif
    #if defined(USE_UART3)  
      friend void SIG_USART3_RECV  (void);
      friend void SIG_USART3_TRANS (void);
    #endif
  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    #if defined(USE_UART0)
      friend void USART_RX_vect    (void);
      friend void USART_TX_vect    (void);
    #endif
  #endif
  
public:
   enum {IDLE, BREAK, STARTB, STARTADR};     // RX DMX states
   enum {TXBREAK, TXSTARTB, TXDATA};         // TX DMX states
  
   volatile uint8_t    *RxBuffer;            // array of RX DMX values
   volatile uint8_t    *TxBuffer;            // array of TX DMX values

private:
   uint8_t     gRxState;
   uint8_t    *gRxPnt;
   uint8_t     IndicatorCount;
   uint8_t     USARTstate;    
   uint8_t     RxByte;     
   uint8_t     RxState;
   uint8_t     mUART;
   uint8_t     gTxState;
   uint16_t    RxCount;
   uint16_t    gCurTxCh;    
   uint16_t    rx_channels;                  // rx channels number
   uint16_t    tx_channels;                  // tx channels number
   uint16_t    rx_address;                   // rx start address
   uint16_t    tx_address;                   // tx start address
   int8_t      rx_led;                       // rx indicator led pin
   int8_t      tx_led;                       // tx indicator led pin
   int8_t      control_pin;                  // max485 input/output selection pin
   uint8_t     dmx_mode;                     // Standard USITT DMX512 = 0, non standard DMX1024 = 1, non standard DMX2048 (DMX1000K) = 2
   uint8_t     speed_dmx;
   uint8_t     speed_break;
   uint16_t    CurTxCh;
   uint8_t     TxState;
   uint8_t    *RxPnt;
   
#if defined(USE_INTERBYTE_DELAY)   
   void        delay_gap          ();
#endif

public:
   void        stop_dmx           ();
   void        set_speed          (uint8_t mode);
   void        set_control_pin    (int8_t  pin)        { control_pin     = pin;      }
   void        init_rx            (uint8_t mode);  // Standard USITT DMX512 = 0, non standard DMX1024 = 1, non standard DMX2048 (DMX1000K) = 2
   void        set_rx_address     (uint16_t address)   { rx_address      = address;  }
   void        set_rx_channels    (uint16_t channels)  { rx_channels     = channels; }
   void        init_tx            (uint8_t mode);  // Standard USITT DMX512 = 0, non standard DMX1024 = 1, non standard DMX2048 (DMX1000K) = 2
   void        set_tx_address     (uint16_t address)   { tx_address      = address;  }
   void        set_tx_channels    (uint16_t channels)  { tx_channels     = channels; }

   void        attachTXInterrupt  (void (*isr)(uint8_t uart))      { TXisrCallback   = isr; }   // register the user TX callback
   void        attachRXInterrupt  (void (*isr)(uint8_t uart))      { RXisrCallback   = isr; }   // register the user RX callback

   void        (*TXisrCallback)   (uint8_t uart);
   void        (*RXisrCallback)   (uint8_t uart);

   inline void Process_ISR_RX     (uint8_t  rx_isr_number);
   inline void Process_ISR_TX     (uint8_t  tx_isr_number);
   
   void       RxIntEn();               //Для режима RxNotPermanent
   void       TxIntEn();               //Для режима TxNotPermanent

public:
   CArduinoDmx                    (uint8_t uart)       { rx_address      = 1; 
                                                         rx_channels     = 8;
                                                         tx_address      = 1; 
                                                         tx_channels     = 8;
                                                         mUART           = uart; }
 
};

#if defined(USE_UART0)
  extern CArduinoDmx ArduinoDmx0;
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #if defined(USE_UART1)
    extern CArduinoDmx ArduinoDmx1;
  #endif
  #if defined(USE_UART2)
    extern CArduinoDmx ArduinoDmx2;
  #endif
  #if defined(USE_UART3)
    extern CArduinoDmx ArduinoDmx3;
  #endif
#endif

#endif

//************** End of "lib_dmx.h" ***********************************************
//*********************************************************************************

#if defined(USE_UART0)
  CArduinoDmx ArduinoDmx0(0);
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #if defined(USE_UART1)
    CArduinoDmx ArduinoDmx1(1);
  #endif
  #if defined(USE_UART2)
    CArduinoDmx ArduinoDmx2(2);
  #endif
  #if defined(USE_UART3)
    CArduinoDmx ArduinoDmx3(3);
  #endif
#endif

// *************** DMX Transmision Initialisation ****************
void CArduinoDmx::init_tx(uint8_t mode)
{
  cli();          //disable interrupts
  stop_dmx();                         //stop uart
  dmx_mode = mode;
  set_speed(dmx_mode);
  
  if(control_pin != -1)
  {
    pinMode(control_pin,OUTPUT);        // max485 I/O control
    digitalWrite(control_pin, HIGH);    // set 485 as output
  }
  
  if(mUART == 0)
  {
    pinMode(1, OUTPUT);
    UBRR0H   = 0;
    UBRR0L   = speed_dmx;  
    UCSR0A  |= (1<<U2X0);
    UCSR0C  |= (3<<UCSZ00)|(1<<USBS0);
    UCSR0B  |= (1<<TXEN0) |(1<<TXCIE0);
    UDR0     = 0;                        //start USART 0
  }
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  else if(mUART == 1)
  {
    pinMode(18, OUTPUT); 
    UBRR1H   = 0;
    UBRR1L   = speed_dmx;   
    UCSR1A  |= (1<<U2X1);
    UCSR1C  |= (3<<UCSZ10)|(1<<USBS1);
    UCSR1B  |= (1<<TXEN1) |(1<<TXCIE1);
    UDR1     = 0;                       //start USART 1
  }
  else if(mUART == 2)
  {
    pinMode(16, OUTPUT); 
    UBRR2H   = 0;
    UBRR2L   = speed_dmx;   
    UCSR2A  |= (1<<U2X2);
    UCSR2C  |= (3<<UCSZ20)|(1<<USBS2);
    UCSR2B  |= (1<<TXEN2) |(1<<TXCIE2);
    UDR2     = 0;                       //start USART 2
  }
  else if(mUART == 3)
  {
    pinMode(14, OUTPUT); 
    UBRR3H   = 0;
    UBRR3L   = speed_dmx;    
    UCSR3A  |= (1<<U2X3);
    UCSR3C  |= (3<<UCSZ30)|(1<<USBS3);
    UCSR3B  |= (1<<TXEN3) |(1<<TXCIE3);
    UDR3     = 0;                       //start USART 3
  }
#endif

  gTxState = BREAK;                             // start with break
  TxBuffer = (uint8_t*)malloc(tx_channels);     // allocate mem for buffer
  memset((uint8_t*)TxBuffer, 0, tx_channels);   // fill buffer with 0's
  sei();          //enable interrupts
}

// ************************ DMX Stop ***************************
void CArduinoDmx::stop_dmx()
{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if(mUART == 0)
  {
    UCSR0B &= ~((1<<RXCIE0) | (1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0));
  }
  else if(mUART == 1)
  {
    UCSR1B &= ~((1<<RXCIE1) | (1<<TXCIE1) | (1<<RXEN1) | (1<<TXEN1));
  }
  else if(mUART == 2)
  {
    UCSR2B &= ~((1<<RXCIE2) | (1<<TXCIE2) | (1<<RXEN2) | (1<<TXEN2));
  }
  else if(mUART == 3)
  {
    UCSR3B &= ~((1<<RXCIE3) | (1<<TXCIE3) | (1<<RXEN3) | (1<<TXEN3));
  }
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  if(mUART == 0)
  {
    UCSR0B &= ~((1<<RXCIE0) | (1<<TXCIE0) | (1<<RXEN0) | (1<<TXEN0));
  }
#endif
}

// *************** DMX Reception Initialisation ***************************************
void CArduinoDmx::init_rx(uint8_t mode)
{
  cli();          //disable interrupts
  stop_dmx();
  dmx_mode = mode;
  set_speed(dmx_mode);
  
  if(control_pin != -1)
  {
    pinMode(control_pin,OUTPUT);        //max485 I/O control
    digitalWrite(control_pin, LOW);     //set 485 as input
  }
  
  if(mUART == 0)
  {
    pinMode(0, INPUT); 
    UBRR0H   = 0;           //*** делитель частоты
    UBRR0L   = speed_dmx;   //***
    UCSR0A  |= (1<<U2X0);   //*** режим удвоения частоты
    UCSR0C  |= (3<<UCSZ00)|(1<<USBS0);  //*** посылка 8 бит
    UCSR0B  |= (1<<RXEN0) |(1<<RXCIE0); //*** разрешение приема и разрешение прерыв по приему данных
  }
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  else if(mUART == 1)
  {
    pinMode(19, INPUT); 
    UBRR1H   = 0;
    UBRR1L   = speed_dmx;
    UCSR1A  |= (1<<U2X1);
    UCSR1C  |= (3<<UCSZ10)|(1<<USBS1);
    UCSR1B  |= (1<<RXEN1) |(1<<RXCIE1);
  }
  else if(mUART == 2)
  {
    pinMode(17, INPUT); 
    UBRR2H   = 0;
    UBRR2L   = speed_dmx;
    UCSR2A  |= (1<<U2X2); 
    UCSR2C  |= (3<<UCSZ20)|(1<<USBS2);
    UCSR2B  |= (1<<RXEN2) |(1<<RXCIE2);
  }
  else if(mUART == 3)
  {
    pinMode(15, INPUT); 
    UBRR3H   = 0;
    UBRR3L   = speed_dmx; 
    UCSR3A  |= (1<<U2X3);
    UCSR3C  |= (3<<UCSZ30)|(1<<USBS3);
    UCSR3B  |= (1<<RXEN3) |(1<<RXCIE3);
  }
#endif
  
  gRxState = IDLE;
  RxBuffer = (uint8_t*)malloc(rx_channels);   // allocate mem for buffer
  memset((uint8_t*)RxBuffer, 0, rx_channels); // fill buffer with 0's
  sei();          //enable interrupts
}


// *************** DMX Reception ISR **************************************************

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #if defined(USE_UART0)
    ISR (SIG_USART0_RECV)
    {
      ArduinoDmx0.Process_ISR_RX(0);
    }
  #endif
  #if defined(USE_UART1)
    ISR (SIG_USART1_RECV)
    {
      ArduinoDmx1.Process_ISR_RX(1);
    }
  #endif
  #if defined(USE_UART2)
    ISR (SIG_USART2_RECV)
    {
      ArduinoDmx2.Process_ISR_RX(2);
    }
  #endif
  #if defined(USE_UART3)
    ISR (SIG_USART3_RECV)
    {
      ArduinoDmx3.Process_ISR_RX(3);
    }
  #endif
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #if defined(USE_UART0)
    ISR (USART_RX_vect)
    {
      ArduinoDmx0.Process_ISR_RX(0);
    }  
  #endif
#endif

//*****************************************************************
void CArduinoDmx::Process_ISR_RX(uint8_t rx_isr_number)
{
  if(rx_isr_number == 0)
  {
    USARTstate = UCSR0A;              //get state
    RxByte     = UDR0;                //get data
    RxState    = gRxState;            //just get once from SRAM!!! в RxState копируется состояние gRxState на момент входа в процедуру
    if (USARTstate &(1<<FE0))         //check for break (этот флаг устанавливается, если стоп-бит равен 0)
    {         
      UCSR0A  &= ~(1<<FE0);           //reset flag
      if (RxByte ==0)           //*** GS *** aditional check for "BREAK" ***
          {RxCount  = rx_address;          //reset frame counter
           gRxState = BREAK;}
        else RxState = gRxState = IDLE;  
    }
  }
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

  else if(rx_isr_number == 1)
  {
    USARTstate = UCSR1A;              //get state
    RxByte     = UDR1;                //get data
    RxState    = gRxState;            //just get once from SRAM!!!
    if (USARTstate &(1<<FE1))         //check for break
    {         
      UCSR1A  &= ~(1<<FE1);           //reset flag
      if (RxByte ==0)           //*** GS *** aditional check for "BREAK" ***
          {RxCount  = rx_address;          //reset frame counter
           gRxState = BREAK;}
        else RxState = gRxState = IDLE;  
    }
  }
  else if(rx_isr_number == 2)
  {
    USARTstate = UCSR2A;              //get state
    RxByte     = UDR2;                //get data
    RxState    = gRxState;            //just get once from SRAM!!!
    if (USARTstate &(1<<FE2))         //check for break
    {         
      UCSR2A  &= ~(1<<FE2);           //reset flag
      if (RxByte ==0)           //*** GS *** aditional check for "BREAK" ***
          {RxCount  = rx_address;          //reset frame counter
           gRxState = BREAK;}
        else RxState = gRxState = IDLE;   
    }
  }
  else if(rx_isr_number == 3)
  {
    USARTstate = UCSR3A;              //get state
    RxByte     = UDR3;                //get data
    RxState    = gRxState;            //just get once from SRAM!!!
    if (USARTstate &(1<<FE3))         //check for break
    {         
      UCSR3A  &= ~(1<<FE3);           //reset flag
      if (RxByte ==0)           //*** GS *** aditional check for "BREAK" ***
           {RxCount  = rx_address;          //reset frame counter
            gRxState = BREAK;}
       else RxState = gRxState = IDLE;
    }
  }
#endif
//=====================================
  if (RxState == BREAK)
  {
    if (RxByte == 0)  //получен нулевой байт
    {
      gRxState = STARTB;              //normal start code detected
      gRxPnt   = ((uint8_t*)RxBuffer + 1);
    }
    else 
      gRxState = IDLE;
  }
  //===================================
  else if (RxState == STARTB)
  {
    if (--RxCount == 0)               //start address reached?
    {
      gRxState   = STARTADR;
      RxBuffer[0]= RxByte;
    }
  }
  //===================================
  else if (RxState == STARTADR)
  {
    RxPnt  = gRxPnt;
    *RxPnt = RxByte;
    if (++RxPnt >= (RxBuffer + rx_channels))  //all ch received?
    {
      gRxState= IDLE;
      if (*RXisrCallback) RXisrCallback(mUART);   // fire callback for read data
  
#if defined(RxNotPermanent)
 
      if(rx_isr_number == 0)
            UCSR0B  &= ~((1<<RXEN0) |(1<<RXCIE0)); //*** снимаем разрешение приема и разрешение прерыв по приему данных
      else if(rx_isr_number == 1)  
            UCSR1B  &= ~((1<<RXEN1) |(1<<RXCIE1));
      else if(rx_isr_number == 2) 
            UCSR2B  &= ~((1<<RXEN2) |(1<<RXCIE2));
      else if(rx_isr_number == 3)     
            UCSR3B  &= ~((1<<RXEN3) |(1<<RXCIE3));
  
#endif      
    }
    else 
    {
      gRxPnt = RxPnt;
    }
  }
//======================================             
}

#if defined(RxNotPermanent)

void CArduinoDmx::RxIntEn()
  {
   if(mUART == 0){
      UCSR0A  &= ~((1<<FE0)|(1<<RXC0)); UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);} //*** разрешение приема и разрешение прерыв по приему данных
   else if(mUART == 1){
      UCSR1A  &= ~((1<<FE1)|(1<<RXC1)); UCSR1B |= (1<<RXEN1)|(1<<RXCIE1);} 
   else if(mUART == 2){
      UCSR2A  &= ~((1<<FE2)|(1<<RXC2)); UCSR2B |= (1<<RXEN2)|(1<<RXCIE2);} 
   else if(mUART == 3){
      UCSR3A  &= ~((1<<FE3)|(1<<RXC3)); UCSR3B |= (1<<RXEN3)|(1<<RXCIE3);} 
  }
#endif

// *************** DMX Transmision ISR ******************************************************************

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #if defined(USE_UART0)
    ISR(SIG_USART0_TRANS)
    {
      ArduinoDmx0.Process_ISR_TX(0);
    }
  #endif
  #if defined(USE_UART1)
    ISR(SIG_USART1_TRANS)
    {
      ArduinoDmx1.Process_ISR_TX(1);
    }
  #endif
  #if defined(USE_UART2)
    ISR(SIG_USART2_TRANS)
    {
      ArduinoDmx2.Process_ISR_TX(2);
    }
  #endif
  #if defined(USE_UART3)
    ISR(SIG_USART3_TRANS)
    {
      ArduinoDmx3.Process_ISR_TX(3);
    }
  #endif
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  #if defined(USE_UART0)
    ISR(USART_TX_vect)
    {
      ArduinoDmx0.Process_ISR_TX(0);
    }
  #endif
#endif


void CArduinoDmx::Process_ISR_TX(uint8_t tx_isr_number)
{
  TxState = gTxState;
  
  if(tx_isr_number == 0)
  {
    if (TxState == TXBREAK) //BREAK + MAB
    {
      UBRR0H   = 0;
      UBRR0L   = speed_break;
      UDR0     = 0;                   //send break
      gTxState = TXSTARTB;
    }
    else if (TxState == TXSTARTB)
    {
      UBRR0H   = 0;
      UBRR0L   = speed_dmx;
      UDR0     = 0;                   //send start byte
      gTxState = TXDATA;
      gCurTxCh = 0;
    }
    else
    {
      #if defined(USE_INTERBYTE_DELAY)
        delay_gap();
      #endif    
      CurTxCh = gCurTxCh;
      UDR0 = TxBuffer[CurTxCh++];       //send data
      if (CurTxCh == tx_channels)
      {
        if (*TXisrCallback) TXisrCallback(0); // fire callback for update data
        gTxState = TXBREAK;   // new break if all ch sent
        
#if defined(TxNotPermanent)
        UCSR0B  &= (0xFF^(1<<TXCIE0));   // снимаем разрешение прерывания по завершению передачи
#endif        
      }
      else 
      {
        gCurTxCh = CurTxCh;
      }
    }
  }
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//=====================================================================
  else if(tx_isr_number == 1)
  {
    if (TxState == TXBREAK)
    {
      UBRR1H   = 0;
      UBRR1L   = speed_break;
      UDR1     = 0;                   //send break
      gTxState = TXSTARTB;
    }
    else if (TxState == TXSTARTB)
    {
      UBRR1H   = 0;
      UBRR1L   = speed_dmx;
      UDR1     = 0;                   //send start byte
      gTxState = TXDATA;
      gCurTxCh = 0;
    }
    else
    {
      #if defined(USE_INTERBYTE_DELAY)
        delay_gap();
      #endif    
      CurTxCh = gCurTxCh;
      UDR1 = TxBuffer[CurTxCh++];       //send data
      if (CurTxCh == tx_channels)
      {
        if (*TXisrCallback) TXisrCallback(1); // fire callback for update data
        gTxState = TXBREAK;   // new break if all ch sent

#if defined(TxNotPermanent)
        UCSR1B  &= (0xFF^(1<<TXCIE1));   // снимаем разрешение прерывания по завершению передачи
#endif               
      }
      else 
      {
        gCurTxCh = CurTxCh;
      }
    }
  }
//================================================================  
  else if(tx_isr_number == 2)
  {
    if (TxState == TXBREAK)
    {
      UBRR2H   = 0;
      UBRR2L   = speed_break;
      UDR2     = 0;                   //send break
      gTxState = TXSTARTB;
    }
    else if (TxState == TXSTARTB)
    {
      UBRR2H   = 0;
      UBRR2L   = speed_dmx;
      UDR2     = 0;                   //send start byte
      gTxState = TXDATA;
      gCurTxCh = 0;
    }
    else
    {
      #if defined(USE_INTERBYTE_DELAY)
        delay_gap();
      #endif   
      CurTxCh = gCurTxCh;
      UDR2 = TxBuffer[CurTxCh++];       //send data
      if (CurTxCh == tx_channels)
      {
        if (*TXisrCallback) TXisrCallback(2); // fire callback for update data
        gTxState = TXBREAK;   // new break if all ch sent

#if defined(TxNotPermanent)
        UCSR2B  &= (0xFF^(1<<TXCIE2));   // снимаем разрешение прерывания по завершению передачи
#endif               
      }
      else 
      {
        gCurTxCh = CurTxCh;
      }
    }
  }

//===============================================================  
  else if(tx_isr_number == 3)
  {
    if (TxState == TXBREAK)
    {
      UBRR3H   = 0;
      UBRR3L   = speed_break;
      UDR3     = 0;                   //send break
      gTxState = TXSTARTB;
    }
    else if (TxState == TXSTARTB)
    {
      UBRR3H   = 0;
      UBRR3L   = speed_dmx;
      UDR3     = 0;                   //send start byte
      gTxState = TXDATA;
      gCurTxCh = 0;
    }
    else
    {
      #if defined(USE_INTERBYTE_DELAY)
        delay_gap();
      #endif
      CurTxCh = gCurTxCh;
      UDR3 = TxBuffer[CurTxCh++];       //send data
      if (CurTxCh == tx_channels)
      {
        if (*TXisrCallback) TXisrCallback(3); // fire callback for update data
        gTxState = TXBREAK;   // new break if all ch sent

#if defined(TxNotPermanent)
        UCSR3B  &= (0xFF^(1<<TXCIE3));   // снимаем разрешение прерывания по завершению передачи
#endif               
      }
      else 
      {
        gCurTxCh = CurTxCh;
      }
    }
  }
#endif
}

//=====================================================
#if defined(TxNotPermanent)

void CArduinoDmx::TxIntEn()
{  
  if(mUART == 0)
     UCSR0B  |= (1<<TXCIE0);   // разрешение прерывания по завершению передачи
  else if(mUART == 1)
     UCSR1B  |= (1<<TXCIE1);
  else if(mUART == 2)
     UCSR2B  |= (1<<TXCIE2);
  else if(mUART == 3)
     UCSR3B  |= (1<<TXCIE3);
}
#endif


//===================================================================================================================
void CArduinoDmx::set_speed(uint8_t mode)
{
  if(mode == 0)
  {
    speed_dmx   = DMX_512;    // DMX-512  (250 kbaud - 512 channels) Standard USITT DMX-512
    speed_break = BREAK_512;
  }
  else if(mode == 1)
  {
    speed_dmx   = DMX_1024;   // DMX-1024 (500 kbaud - 1024 channels) Completely non standard, but usefull ;)
    speed_break = BREAK_1024;
  }
  else if(mode == 2)
  {
    speed_dmx   = DMX_2048;   // DMX-2048 (1000 kbaud - 2048 channels) Used by manufacturers as DMX1000K, DMX-4x or DMX-1M ???
    speed_break = BREAK_2048;
  }  
}

#if defined(USE_INTERBYTE_DELAY)

void CArduinoDmx::delay_gap() // rare cases of equipment non full DMX-512 compliant, need this
{
  if(dmx_mode == 0)
  {
    _delay_us(IBG_512);
  }
  else if(dmx_mode == 1)
  {
    _delay_us(IBG_1024);
  }
  else if(dmx_mode == 2)
  {
    _delay_us(IBG_2048);
  }
}
#endif


//=====End of DMX files =================================================================================================
//=======================================================================================================================


LiquidCrystal_I2C lcd(0x3F,16,2);
byte mac[]={0xDE,0xAD,0xBE,0x71,0x72,0x61};
byte ipaddr[]={192,168,1,11};
EthernetUDP Udp;

CArduinoDmx *ODmxp = &ArduinoDmx2;
CArduinoDmx *IDmxp = &ArduinoDmx3;

#define DmxOChLast 64  
         // не более 239 каналов
#define DmxIChLast 40 

#define indCharL 50
#define chGrarL 10
#define PrecShift 9
#define Tick10ms 2

#define InpChNum  40
#define DestChArL  80

#define UdpPacRecL 508
#define UdpPacMaxL 1024
#define UdpPacMaxL2 250    
                    //максимальная длина добавочного скрипта
#define indx_t byte

unsigned long timems;
byte procerror=0;
byte busyflag=0;            //устанавливается в processtep() и в scriptstep() если выполняется операция
byte scrresident=0;         //устанавливается, если скрипт резидентный
byte indealchs[indCharL];
struct chgr_t {int stepsleft; long longlevel; long longstep; byte grtype;} chgrps [chGrarL];
struct sktxt_t {unsigned int scrtm; unsigned long scrtms0;
                int lbind = -1; byte lpcnt =0; byte inloop =0;    //управляющие циклом переменные 
               } scrktxreg, scrktxadd;

byte sourcharr[InpChNum];
byte destcharr[DestChArL];

int scripti= -1, scriptl=0;
int scripti2= -1, scriptl2=0;

//unsigned int scripttm;
//unsigned long scripttms0;
byte scriptp[UdpPacMaxL];
byte scriptp2[UdpPacMaxL2];  

//int labelind = -1; byte loopcount =0, inloop =0;    //управляющие циклом переменные 

int bufbusyl=0;
byte udpbuff[UdpPacMaxL];

unsigned packnum=0;

byte memory[DmxOChLast];

//============================Processing====================================================
enum grTypes {Empty=0, DmxLED, DmxFilament, GsFilament};

void grclear(indx_t i)
  { chgrps[i].stepsleft=0;  chgrps[i].grtype=Empty; chgrps[i].longlevel = chgrps[i].longstep =0;
  }

void procesreset()
  { for(indx_t i=0; i < indCharL; i++) indealchs[i]=0;
    for(indx_t i=0; i < chGrarL; i++) grclear(i);
  }


void dmxlgrstep(indx_t *ch_ip, indx_t gr_i ) 
  { long x;
    uint8_t value, channl;
    indx_t ch_i1;
    
    ch_i1 = *ch_ip -1;
    x = (chgrps[gr_i].longlevel += chgrps[gr_i].longstep);    if( x<0 ) x=0;
    value = x >> PrecShift; 
    channl =indealchs[*ch_ip];
    do
      { *(ODmxp->TxBuffer + channl -1)= value;  //загрузка нового значения канала в буфер dmx
        channl = indealchs[++(*ch_ip)]; 
      }
    while( channl );                      //обрабатываем все каналы
    if( --(chgrps[gr_i].stepsleft) ==0 )  //проверка остатка шагов и удаление инф.о группе, если шагов не осталось
      {
       for(indx_t i=ch_i1; i < *ch_ip; i++) indealchs[i]=0;
       grclear(gr_i)  ;
      }
  }

void processtep()
  { indx_t gr_i, ch_i=0;
    do {    
        for (; ch_i <indCharL; ch_i++) if( indealchs[ch_i] ) break; //пропускаем нули
        if( ch_i >= indCharL )  return;   //групп не найдено
        
        gr_i = indealchs[ch_i++] -1;    //индекс группы смещен на -1 от номера группы
        if( ! indealchs[ch_i] ) { procerror |= 2; continue;} //!!! нет списка каналов  ====2=!!!!!
        if( chgrps[gr_i].stepsleft==0 )
                                { procerror |= 4; continue;} //!!! остаток шагов группы равен нулю  ====4=!!!!!
        busyflag=1;         //---установка признака занятости------------                                
        switch ( chgrps[gr_i].grtype )
          { case DmxLED:
              dmxlgrstep( &ch_i, gr_i ); //ch_i указывает на номер первого канала группы в массиве indealchs[]
              break;
            case Empty: procerror |= 1; //!!!тип группы "0"                     ====1=!!!!!
            case DmxFilament:
            case GsFilament:
            default:
              for (; ch_i <indCharL; ch_i++) if( ! indealchs[ch_i] ) break; //пропускаем ненули
              break;
          } 
       } while (1);
  }

indx_t dmxlgrinit(byte firstch, byte chnum, byte value, int dt)  // dt/100=seconds; chnum-колич каналов в группе
  { byte oldvalue, flag;
    indx_t gr_i, ch_i, i1=0; 
    int stepnum;
    if(chnum==0 || firstch==0) { procerror |= 1; return(-1); }  //!!! недопустимые нулевые параметры ====1=!!!!!
    if(firstch > DmxOChLast)   { procerror |= 8; return(-1); }  //!!! недопустимый номер канала  ====8=!!!!!
    if(dt > 1020) dt=1020;
//-----ищем пустую группу
    for( gr_i=0; gr_i < chGrarL; gr_i++)  if( chgrps[gr_i].grtype==Empty) break;
    if( gr_i >= chGrarL) {procerror |= 16; return(-1); };        //!!! все группы заняты ====16=!!!!!!!!
    
    chgrps[gr_i].grtype = DmxLED;
    chgrps[gr_i].stepsleft = stepnum = dt ? dt/Tick10ms : 1;
    chgrps[gr_i].longlevel = long( oldvalue = *(ODmxp->TxBuffer + firstch -1) ) << PrecShift;
//    memory[firstch -1] = oldvalue;            //запоминаем бывшее значение канала в памяти
    if( value >= oldvalue )
           chgrps[gr_i].longstep =  ( long(value - oldvalue) << PrecShift ) / stepnum;
      else chgrps[gr_i].longstep = -( long(oldvalue - value) << PrecShift ) / stepnum;
//-----ищем место в indealchs...
    for ( ch_i=0; ch_i <indCharL-2-chnum ; ch_i++)
        { if( indealchs[ch_i] ) {i1 = ch_i +1; continue;} //пропускаем ненули, i1 указывает на символ после ненуля
            else if( ch_i >= i1 +chnum +2 ) 
                    { indealchs[++i1] = gr_i +1;  //номер группы сдвинут на +1, чтобы не было нулевой группы
                      indealchs[++i1] = firstch;  return(++i1); // возвращаем индекс ячейки второго канала в группе
                    }
        }
    procerror |= 32;               return(-1);      //!!! не найдено место в indealchs[]  ====32=!!!!!
  }
  

void dmxlchadd(indx_t ch_i, byte chanel)
  { byte prevch;
    if(ch_i==0 || chanel==0) { procerror |= 1; return; }           //!!! недопустимые нулевые параметры ====1=!!!!
    if(chanel > DmxOChLast)   { procerror |= 8; return; }  //!!! недопустимый номер канала ====8=!!!!
    prevch = indealchs[ch_i -1];
    if(prevch==0 || indealchs[ch_i] !=0 || indealchs[ch_i +1] !=0) 
                             { procerror |= 64; return; };          //!!! неверные значения в indealchs[]  ====64=!!!!!!
                             
//    *(memory + chanel -1) =  *(ODmxp->TxBuffer + chanel -1);       //запоминаем бывшее значение канала в памяти               
    *(ODmxp->TxBuffer + chanel -1)= *(ODmxp->TxBuffer + prevch -1);  //начальные значения каналов выравниваем по первому в группе 
    indealchs[ch_i] = chanel;
  }

//===============================Projection=================================================

void projreset()
  { for(byte i=0; i < InpChNum; i++) sourcharr[i]=0;
  }

  
void projection()
  { byte i1, i2, value;   //отображение входных (source) каналов начинается с первого канала
                          //списки каналов назначения находятся в массиве destcharr[];
                          //sourcharr[] содержит границы этих списков
    i1=0;
    for( byte i=0; i < InpChNum; i++)
      { i2=sourcharr[i];
        value= *(IDmxp->RxBuffer + i);
        while(i1 < i2)  *(ODmxp->TxBuffer + destcharr[i1++]) = value;
      }
  }

void addprojnew(byte inpch, byte destch)    //inpchan = 1...4...
  { byte idx;
    inpch--;  destch--;   //смещение канала в буфере 0...3...
    if(inpch >= InpChNum)   { procerror |= 128 + 1; return; };   //недопустимый номер канала источника или назначения ===129=!!!!!
    if(destch > DmxOChLast) { procerror |= 128 + 1; return; };   //недопустимый номер канала источника или назначения ===129=!!!!!
    
    if(inpch >0) idx = sourcharr[inpch -1];
        else     idx =0;
    if(idx >= DestChArL)  { procerror |= 128 + 2; return; };   //массив назначений переполнен ===130=!!!!!
    
    for(byte i=inpch; i<InpChNum; i++) if(sourcharr[i] != idx) procerror |= 128 + 4;
                                          //ошибка применения функции или структуры sourcharr[]  ===132=!!!!!
    destcharr[idx] = destch;
    for(byte i=inpch; i<InpChNum; i++)  sourcharr[i]=idx +1; 
  }

void addprojnext(byte inpch, byte destch)   //inpchan = 1...4...
  { byte idx;
    inpch--;  destch--;   //смещение канала в буфере 0...3...
    
    if(inpch >= InpChNum) { procerror |= 128 + 1; return; };   //недопустимый номер канала источника ===129=!!!!!
    
    idx = sourcharr[inpch];
    if(idx >= DestChArL)  { procerror |= 128 + 2; return; };   //массив назначений переполнен ===130=!!!!!
    
    for(byte i=inpch+1; i<InpChNum; i++) if(sourcharr[i] != idx) procerror |= 128 + 4;
                                          //ошибка применения функции или структуры sourcharr[]  ===132=!!!!!
    destcharr[idx] = destch;
    sourcharr[inpch] = ++idx;
    for(byte i=inpch+1; i<InpChNum; i++)  sourcharr[i]=idx; 
  }

byte projlastsrch()
  { byte previdx= sourcharr[InpChNum-1];
    if( previdx ==0 ) return 0;
    for(byte i=InpChNum-2; i >=0; i--) 
      { if( sourcharr[i] > previdx) procerror |= 128 + 4;  //ошибка структуры sourcharr[]  ===132=!!!!!
        if( sourcharr[i] == previdx ) continue;
        return i+2;       
      }
    return 1;
  }
  
//==================================Scripts=================================================================
//int scripti= -1, scriptl=0;
//unsigned int scripttm;
//unsigned long scripttms0;
//byte scriptp[UdpPacMaxL];              
//char frame[]="!@250 #2/s50 s1,2/ @100 L #2/0,0 @100 #3/250,250 @250 #3/0,250 r3 @250 #4/250,250 @250 #4/0,250 @500 #2,4/220,500 @500 #2,4/0,400 @500 #2,3,4/0,0 $";               
//int labelind = -1; byte loopcount =0, inloop =0;    //управляющие циклом переменные 

enum ScriptCmd { Htetr =0xF0, Proces, ProcVal, Delay, BackVal, StoreVal, ProjSet, ProjRes, Label, Repeat, CmdEnd };  //BackVal-берем значение из memory +++Ver10++++++++++++++++

 void scriptstep(byte * scriptp, int& scripti, int scriptl, byte scrresident, struct sktxt_t* kontext )
  { byte ch, ch_i, value, istep; int ch1i;  indx_t nxchi; unsigned int dt;
    
    if( scripti <0 ) return;
    if(scripti ==0)           // инициализация обработки скрипта
      { kontext->scrtm=0; kontext->scrtms0 = millis();
        kontext->inloop =0; kontext->lbind =-1;   }
    busyflag=1;       
    if( millis()-kontext->scrtms0 < (unsigned long) kontext->scrtm *10 ) return; //время не настало

    while(scripti < scriptl)
      {switch( *(scriptp + scripti))
        { case Proces:
          
            ch1i= ++scripti;           //указывает на 1-ый номер канала
            while( ((ch = *(scriptp+scripti)) & Htetr) != Htetr)          //пропускаем номера каналов
                    {scripti++; if(scripti >= scriptl) {procerror |= 128 + 8; scripti =-1; return; }} //====136=!!!!!!!!
                                                  //нарушена структура скрипта
            if(scripti == ch1i) {procerror |= 128 + 16; scripti =-1; return; }                        //====144=!!!!!!!!
            scripti++;    //указывает на байт после разделителя
            switch( ch )
              {case ProcVal:
                  value = *(scriptp + scripti); istep=3;
                  dt = ( *(scriptp + scripti+1) <<8) + *(scriptp + scripti+2); break;
               case BackVal:                                                              // from memory
                  value = memory[ *(scriptp + ch1i) -1 ]; istep=2;
                  dt = ( *(scriptp + scripti) <<8) + *(scriptp + scripti+1); break;
               default: procerror |= 128 + 8; scripti =-1; return;  //номер канала отсутствует или больше 239 ====136=!!!!!!!!
              }
            
            nxchi = dmxlgrinit( *(scriptp + ch1i), scripti -ch1i -1, value, dt);
            for(ch1i++; ch1i < scripti-1; ch1i++)   dmxlchadd( nxchi++, *(scriptp + ch1i));
            scripti += istep;                                   
            break;            
          case Delay:
            dt = ( *(scriptp + scripti+1) <<8) + *(scriptp + scripti+2);
            if( kontext->scrtm > 0xFFFF - dt ) {procerror |= 128 + 8 +2; scripti =-1; return; }   //слишком долгий скрипт ===138=!!!!!
            kontext->scrtm += dt;
            scripti +=3;  return;
            
          case StoreVal:         // запоминаем значения каналов в memory
            scripti++;           //указывает на 1-ый номер канала
            while( ((ch = *(scriptp+scripti)) & Htetr) != Htetr)          //перебираем номера каналов
                    {memory[ch-1] = *(ODmxp->TxBuffer + ch -1); scripti++; 
                     if(scripti >= scriptl) {procerror |= 128 + 8; scripti =-1; return; }       //====136=!!!!!!!!
                    }
            if( ch != ProcVal) {procerror |= 128 + 8; scripti =-1; return; }                    //====136=!!!!!!!!
            scripti++;  break;
            
          case ProjRes:
            projreset();  
            scripti++;   break;
            
          case ProjSet:
            scripti++; ch_i = *(scriptp + scripti++);
            if( *(scriptp + scripti++) != ProcVal) {procerror |= 128 + 8; scripti =-1; return; }   //====136=!!!!!!!!
            ch = *(scriptp + scripti++);  if( (ch & Htetr) == Htetr) {procerror |= 128 + 8; scripti =-1; return; }   //====136=!!!!!!!!
            addprojnew(ch_i, ch);
            while( ((ch = *(scriptp+scripti)) & Htetr) != Htetr)   
              { addprojnext(ch_i, ch);  scripti++;
              }
            if( ch != CmdEnd ) {procerror |= 128 + 8; scripti =-1; return; }                      //====136=!!!!!!!!
            scripti++; break;
            
          case Label:
            kontext->lbind = scripti++;  break;
            
          case Repeat:            // проверку на невложенность циклов СДЕЛАТЬ В Convert !!!!!!!!
            if( kontext->lbind < 0 )  {procerror |= 128 + 8 +1; scripti =-1; return; }  // ошибка организации цикла ==137=!!!!!!!!
            if( ! kontext->inloop ) 
              { kontext->inloop =1; scripti++; kontext->lpcnt = *(scriptp+scripti);
                if( ((kontext->lpcnt & Htetr) == Htetr) || (kontext->lpcnt ==0) )  {procerror |= 128 + 8 +1; scripti =-1; return; }  //==137=!!!!!!!!
              }
             else
                if( --(kontext->lpcnt) ==0 ) { kontext->inloop =0; kontext->lbind =-1; scripti +=2; break; };  //нормальный выход из цикла
              
            scripti = kontext->lbind;  break;     //выполняем переход на начало тела цикла
              
          default: procerror |= 128 + 8; scripti =-1; return;  //нарушена структура скрипта        ====136=!!!!!!!!
        }
    } 
  //---------------------------завершение или зацикливание, если скрипт резидентный
  if(scrresident) {scripti = 0;  return;} 
  scripti = -1;  return;  //нормальное завершение скрипта
  }                      
//   Htetr =0xF0, Proces, ProcVal, Delay, BackVal, StoreVal, ProjSet, ProjRes, Label, Repeat, CmdEnd  - управляющие коды скрипта

//=============================================================
void scriptcancel()
  {scripti = -1; scripti2 = -1;
   scrresident=0;
  }

//==============================================CONVERTion==================================================  
byte convert1(byte * udpbuf, byte * scriptp, int& scriptl)
  { byte * scriptp0; byte * bufendp;
    union int2by {int tm; int vlw;
                  byte by[2];
                  byte vl;     }val;
    byte ch_i, ch_o;
    
    bufendp = udpbuf + bufbusyl; scriptp0 = scriptp;
    if( (*(udpbuf++)) & 0xC0 ) {procerror |= 128 + 32; return(0); } //неверный первый байт пакета или ошибка структуры пакета =160======

    while( udpbuf < bufendp )
      switch( *(udpbuf++))
        {
          case '#':
              *(scriptp++) = Proces;
              do {val.vl = atoi((char*)udpbuf); if( (val.vl >239) || val.vl ==0) {procerror |= 128+16; return(0);}
                                                                        //номер канала отсутствует или больше 239 =144=======
                  *(scriptp++) = val.vl; 
                  while( isdigit( *udpbuf)) udpbuf++ ;
                 } while( *(udpbuf++)!='/');
              if( *udpbuf != 's')
                { val.vl = atoi((char*)udpbuf);   while( isdigit( *udpbuf)) udpbuf++ ;    //обработка '/'
                  *(scriptp++) =ProcVal;  *(scriptp++) = val.vl;
                }
               else { udpbuf++; *(scriptp++) =BackVal; }                    //обработка '/s'
               
              val.tm = atoi((char*)++udpbuf); while( isdigit( *udpbuf)) udpbuf++ ; 
               *(scriptp++) = val.by[1];  *(scriptp++) = val.by[0];   break;
              
          case '@':
              *(scriptp++) = Delay;
              val.tm = atoi((char*)udpbuf); while( isdigit( *udpbuf)) udpbuf++ ;
                *(scriptp++) = val.by[1];  *(scriptp++) = val.by[0];  break;
                
          case '$': scriptl =scriptp-scriptp0; return(1);   //успешное завершение, установка длины скрипта scriptl

          case 's':  
              *(scriptp++) = StoreVal; 
              do { val.vl = atoi((char*)udpbuf); if( (val.vl >239) || val.vl ==0) {procerror |= 128+16; return(0);}
                                                                       //номер канала отсутствует или больше 239 ===144====
                   *(scriptp++) = val.vl;  while( isdigit( *udpbuf)) udpbuf++ ;
                 } while( *(udpbuf++)!='/');
              *(scriptp++) =ProcVal;  break;
              
          case 'L': 
              *(scriptp++) = Label;  break;
              
          case 'r':
              if( val.vlw = atoi((char*)udpbuf) )
                if( val.vlw <= 255 ) 
                  { *(scriptp++) = Repeat;  *(scriptp++) = val.vl; }
              while( isdigit( *udpbuf)) udpbuf++ ;  break; 
                 
          case '*': *(scriptp++) = ProjRes;  break;
//--------------------------------------------------------------------- здесь и в других местах доделать контроль диапазона значений с исп. union
          case '+': ch_i = atoi((char*)udpbuf); while( isdigit( *udpbuf)) udpbuf++ ;
              if( *udpbuf != '/') { procerror |= 128+64+8; return(0); };  //ошибка при разборе пакета   ===200=======
              ch_o = atoi((char*)++udpbuf);  while( isdigit( *udpbuf)) udpbuf++;
              *(scriptp++) = ProjSet;  *(scriptp++) = ch_i;  *(scriptp++) = ProcVal;  *(scriptp++) = ch_o;
              while ( *udpbuf == ',') 
                { ch_o = atoi((char*)++udpbuf);  while( isdigit( *udpbuf)) udpbuf++;  *(scriptp++) = ch_o; };
              *(scriptp++) = CmdEnd;  break;

          case ' ': break;          
          case 0x0A: break;  
          case 0x0D: break;         //допустимы пробелы, ВК, ПС между тегами скрипта
          
          default : procerror |= 128+32; return(0);       //неверный тип пакета или ошибка структуры пакета  =160======
        }
    procerror |= 128+32; return(0);  //                                                           ====160======
  }
/*
void convert2(byte * udpbuf)
  { byte ch_i, ch_o;
  
    if( *(udpbuf++) != '=') {procerror |= 128 + 32; return; } //неверный тип пакета или ошибка структуры пакета
    while(1)
      switch( *(udpbuf++))
        {
          case '+': ch_i = atoi((char*)udpbuf); while(isdigit( *(++udpbuf)));
              if( *udpbuf != '/') { procerror |= 128+64+8; return; };  //ошибка при разборе пакета
              ch_o = atoi((char*)++udpbuf);  while(isdigit( *(++udpbuf)));
              addprojnew(ch_i, ch_o);
              while ( *udpbuf == ',') 
                { ch_o = atoi((char*)++udpbuf);  while(isdigit( *(++udpbuf))); 
                  addprojnext(ch_i, ch_o);
                };
              break;

          case '*': projreset();
              break;
          case '$': return;                          //успешное завершение
          case ' ': break;                              //допустимы пробелы между тегами скрипта
          default : procerror |= 128+32; return;   //неверный тип пакета или ошибка структуры пакета =160=!!!!!!!!
        }  
  }
 */ 
//=================================================GETPACKET==============================================  

// int bufbusyl=0;
// byte udpbuff[UdpPacMaxL];

//-------------------------очистка буфера Ethernet контроллера  
void udpclear()
  { byte errbuf[16];
    while( Udp.available() ) 
         { Udp.read(errbuf, 16); 
           procerror |= 128+64;   //аварийная очистка буфера - либо ошибка "1" либо ?               =192=!!!!!!!!!!!!!!!!!
         }
  }
  
void getpack()
  {    
   switch ( udpbuff[0] )
//экстренный скрипт
    { case '!' :  if( udpbuff[bufbusyl-1] != '$')  
                    { procerror |= 128+64+2; bufbusyl =0; return; };  //пакет не завершен символом "$" и отвергнут  =194=!!!!!!!!
                  procesreset();  scriptcancel();          //прекращаем выполнение предыдущего скрипта в т.ч. резидентного и выполняем экстренный
                  if( convert1(udpbuff, scriptp, scriptl) ) scripti=0;     //установка scripti в 0 запускает скрипт
                  bufbusyl = 0;         //признак того, что udp-буфер обработан
                  break;
//регулярный скрипт                  
      case '=' :  if( scripti >= 0) return;       //выполняется прeдыдущий скрипт;
                  if( udpbuff[bufbusyl-1] != '$')  
                    { procerror |= 128+64+2; bufbusyl =0; return; };  //пакет не завершен символом "$" и отвергнут  =194=!!!!!!!!
                  
                  if( convert1(udpbuff, scriptp, scriptl) ) scripti=0;
                  bufbusyl = 0;
                  break;
//резидентный скрипт
      case '>' :  if( scripti >= 0) return;       //выполняется прeдыдущий скрипт;
                  if( udpbuff[bufbusyl-1] != '$')  
                    { procerror |= 128+64+2; bufbusyl =0; return; };  //пакет не завершен символом "$" и отвергнут  =194=!!!!!!!!
                  if( convert1(udpbuff, scriptp, scriptl) ) {scripti=0; scrresident=1;}
                  bufbusyl = 0;
                  break; 
//добавочный скрипт
      case '%' :  if( scripti2 >= 0) return;       //выполняется прeдыдущий добавочный скрипт;
                  if( udpbuff[bufbusyl-1] != '$')  
                    { procerror |= 128+64+2; bufbusyl =0; return; };  //пакет не завершен символом "$" и отвергнут  =194=!!!!!!!!
                  if( bufbusyl > UdpPacMaxL2 ) {procerror |= 128+64+1;}; // Udp-пакет добавочного скрипта больше допустимого =193=!!!!!!!
                  
                  if( convert1(udpbuff, scriptp2, scriptl2) ) scripti2=0; 
                  bufbusyl = 0;
                  break;      
                  
 //     case '=' :  if( udpbuff[bufbusyl-1] != '$')  
 //                   { procerror |= 128+64+2; bufbusyl =0; return; };  //пакет не завершен символом "$" и отвергнут  =194=!!!!!!!!
 //                 convert2( udpbuff );
 //                 bufbusyl = 0;
 //                 break;
                  
      default :   procerror |= 128+64+32+4; bufbusyl =0; return;  //  пакет не идентифицирован по первому символу =228=!!!!!!!!!!!!!
    }
  }


    
//===================================SETUP======================================================================= 
//DIOL_11 отличается от DmxIOLan11 использованием индикатора 1602 и модуля Ethernet W5500 
#define TestPin1 7
#define Btn1pin 30
#define Btn2pin 31

void setup() {
  indx_t ch_i;
  
  pinMode(53, OUTPUT); //Написано, что так надо для работы с W5100
  pinMode(4, OUTPUT); digitalWrite(4, HIGH); //Отключаем SD-карту
  
  pinMode(TestPin1, OUTPUT); //тестовые средства
  digitalWrite(TestPin1, HIGH); // LED - Off
  
  pinMode(Btn1pin, INPUT); digitalWrite(Btn1pin, HIGH);
  pinMode(Btn2pin, INPUT); digitalWrite(Btn2pin, HIGH);
  
  lcd.init();  lcd.backlight(); lcd.cursor();     // stscr_init();
//  delay(1000);
//  lcd.noDisplay();
  
  Ethernet.begin(mac, ipaddr);
  Udp.begin(4000);

  ODmxp->set_control_pin(9);   // Arduino output pin for MAX485 input/output control (connect to MAX485-1 pins 2-3)
                               // pinMode выполняется в процедуре  
  ODmxp->set_tx_address(1);    // set rx1 start address
  ODmxp->set_tx_channels(DmxOChLast); // 2 to 512 channels in DMX512 mode.
  ODmxp->init_tx(DMX512);    // 

  IDmxp->set_control_pin(8);   // Arduino output pin for MAX485 input/output control (connect to MAX485-1 pins 2-3) 
  IDmxp->set_rx_address(1);    // set rx1 start address
  IDmxp->set_rx_channels(DmxIChLast); // 2 to 512 channels in DMX512 mode.
 // Установленное здесь количество каналов должно быть не более реального передаваемого пультом, иначе режим RxNotPermanent не включается !!
     // 
 
  procesreset();  //Очистка массивов конвейера
  projreset();    //Очистка массива проекций
  
   ODmxp->TxIntEn();
  IDmxp->init_rx(DMX512);
  
  timems = millis()+ Tick10ms * 10;
  }

//-----------------------------------------
void lcdprnt(unsigned int n, byte npos)
    { byte dig; unsigned int n1;
//      lcd.rightToLeft(); 
      if( npos >5) npos =5;
      for(byte i = 0; i < npos; i++) 
        { //n1 = n/10; dig = n - n1*10; 
         // lcd.write(48+dig);
          n = n1;
        }
//      lcd.leftToRight();
    }
    

//============================ MENU, Buttons & LCDscreen=====================================================

#define BtnLongPr  15 //* 0.1sec
#define Menu1showtm  40 //* 0.1sec
#define Menu1donetm  10 //* 0.1sec

byte wavetstgrgo =0;  //номер группы для которой запущен WAVE-test....
byte wavetsttime =0;  //счетчик времени в тиках для этого теста

enum FMenuCmd {m_show =1, m_sshow, m_go, m_tmout, m_done, m_bt1S, m_bt2S, m_bt2L};
byte btn1state=0, btn2state=0, inmenu1=0, inmenu2=0;
int btn1time, btn2time, menu1time=0;

//------------------------------------------menu 1--------------------------------------------------------------------------------------
enum Menu1item           {     GenerlCmd=1, Ini255Cmd, TstGr1cmd,    TstGr2cmd,  TstGr3cmd,  Menu1TOP };

enum Menu12item              {                           M12cmd1=1,         M12cmd2,        M12cmd3,         M12cmd4,         M12cmd5,         Menu12TOP};                              
static char tmenu1[][Menu12TOP][16]={{"General Cmd",    "Error Reset",      "-",            "-",             "-",              "-" },
                                     {"Ini255Ls Cmd",   "Clear List",       "Do Ini255",    "BlackOut List", "-",              "-"},
                                     {"TstGrp 1 Cmd",   "Clear List&Val",   "Send MIN Val", "Send MAX Val",  "MinMax WAVE",   "BlackOut Group"},
                                     {"TstGrp 2 Cmd",   "Clear List&Val",   "Send MIN Val", "Send MAX Val",  "MinMax WAVE",   "BlackOut Group"},
                                     {"TstGrp 3 Cmd",   "Clear List&Val",   "Send MIN Val", "Send MAX Val",  "MinMax WAVE",   "BlackOut Group"}};
//--------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------menu 2------------------------------------------------------------------------------------------------------------------------------
static char tmenu2[][16]={"","Initial 255 Chs", "IO Chan Transit", "Test Group 1", "TstGr1 Values", "Test Group 2", "TstGr2 Values",  "Test Group 3", "TstGr3 Values"};
enum Menu2item           {    Ini255set=1,       IOtransit,          TstGr1,         TstGr1v,         TstGr2,         TstGr2v,          TstGr3,         TstGr3v,   Menu2TOP };
static char tmenu2s[][16]={"","Ini255 Chs",     "IO Transit",       "Test Grp 1",   "TstGr1 Val",    "Test Grp 2",   "TstGr2 Val",     "Test Grp 3",   "TstGr3 Val"};

#define Ini255MaxN  10
#define TstGrMaxN   10
#define TransMaxN   8

enum TstGrIndx                {TstGR1i=0, TstGR2i, TstGR3i, TstGRiTOP};
byte ini255list [Ini255MaxN], testgr [TstGRiTOP][TstGrMaxN];
byte ini255n=0,               testgrn[TstGRiTOP];
byte trsourch, trdestlist [TransMaxN];

enum TstGrValues {MinVal=0, MaxVal, Tau01s, GrValMaxN}; //GrValMaxN - это количество параметров группы- всегда последняя константа
byte tstvalues[TstGRiTOP][GrValMaxN]={0,255,10, 0,255,10, 0,255,10};
static char tvalmenu [][16]={"MIN val=","MAX val=","Tm 0.1s="};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void menuerr() {}
//--------------------------------------------------------------------------

void ftstgrsendv(byte groupi, byte value, byte tau )
  { indx_t ch_i;
    if( testgrn[groupi] ==0 ) return;
    ch_i= dmxlgrinit( testgr [groupi][0], testgrn[groupi] , value, tau);
    for( byte i= 1; i < testgrn[groupi] ; i++ ) dmxlchadd(ch_i++, testgr [groupi][i]);
  }

//------------------------------------------------------------------------
byte ftstgrproc(byte groupi, byte cmd)
  { indx_t ch_i; byte value=0;
    switch( cmd )                   //"Clear List&Val", "Send MIN Val", "Send MAX Val", "MinMax WAVE",   "BlackOut Group"
      { case M12cmd1 : testgrn[groupi] =0; 
                       tstvalues[groupi][MinVal] =0; tstvalues[groupi][MaxVal] =255; tstvalues[groupi][Tau01s] =10; return 1;
           
        case M12cmd2 : value =tstvalues[groupi][MinVal]; ftstgrsendv(groupi, value, 0 );  return 1;
        case M12cmd3 : value =tstvalues[groupi][MaxVal]; ftstgrsendv(groupi, value, 0 );  return 1;
        
        case M12cmd4 : value =tstvalues[groupi][MinVal]; ftstgrsendv(groupi, value, 0 );
                       value =tstvalues[groupi][MaxVal]; ftstgrsendv(groupi, value, tstvalues[groupi][Tau01s] ); 
                       wavetstgrgo =groupi+1;   wavetsttime = tstvalues[groupi][Tau01s] *10 / Tick10ms;     return 1;
                       
        case M12cmd5 : ftstgrsendv(groupi, 0, 0 );  return 1;               
        default      : menuerr(); break;
      }
    return 0;
  }
  
//--------------------------------------------------------------------------
byte fini255proc(byte cmd)
  { indx_t ch_i; byte value=0;
    switch( cmd )                   //"Clear List",   "Do Ini255",    "BlackOut List",
      { case M12cmd1 : ini255n =0; return 1;  
      
        case M12cmd2 : value =255;
        case M12cmd3 : if( ! ini255n ) return 0;
                       ch_i= dmxlgrinit( ini255list [0], ini255n, value, 0);
                       for( byte i= 1; i < ini255n; i++ ) dmxlchadd(ch_i++, ini255list [i]); return 1;
                       
        default      : menuerr(); break;
      }
    return 0;
  }
  

//-------------------------------------------------------------------------
void fmenu1 (byte cmd)
  { static byte inmenu12, doneflag;
    byte gr_i;
    if(cmd == m_done ) {doneflag =1; lcd.setCursor(0,1); lcd.print("    **DONE**    "); menu1time = Menu1donetm *10 / Tick10ms; return;};
    if(cmd == m_tmout)
      { if( ! inmenu1 )   return;
        if( --menu1time ) return;
        inmenu1 = inmenu12 = doneflag =0;  return;  //выход из меню1
      }
    if( doneflag ) return;  
    if( !inmenu1 ) inmenu12 =0;  
    if( !inmenu12 )
      { switch (cmd)
          { case m_show  : 
            case m_bt1S  : if(++inmenu1 >= Menu1TOP ) inmenu1 =1;
                           menu1time = Menu1showtm *10 / Tick10ms; 
                           lcd.clear(); lcd.print(" MENU 1 ->"); lcd.setCursor(0,1); lcd.print(tmenu1[inmenu1 -1][0]);
                           return;
            case m_go    : 
            case m_bt2S  : case m_bt2L :
                           inmenu12 =1; fmenu1(m_show); return; 
          }  
      }
    else                  //второй уровень: inmenu12 != 0; 
      { switch (cmd)
          { case m_bt1S  : inmenu12++; if(inmenu12 >= Menu12TOP ) inmenu12 =1;
            case m_show  : menu1time = Menu1showtm *10 / Tick10ms; 
                           lcd.clear(); lcd.print("M1=>");  lcd.print(tmenu1[inmenu1 -1][0]);
                           lcd.setCursor(0,1);  lcd.print(tmenu1[inmenu1 -1][inmenu12]);
                           return;
            case m_bt2S  : case m_bt2L :
                           gr_i =TstGR1i;
                           switch( inmenu1 )
                             { case GenerlCmd : switch( inmenu12 )
                                                  { case M12cmd1 : procerror =0; fmenu1(m_done); return;
                                                    default      : break;
                                                  }
                                                break;  
                               case Ini255Cmd : if( fini255proc( inmenu12 ) ) {fmenu1(m_done); return;};
                                                break;    
                               case TstGr3cmd : gr_i++;
                               case TstGr2cmd : gr_i++;
                               case TstGr1cmd : if ( ftstgrproc(gr_i, inmenu12 ) ) {fmenu1(m_done); return;};
                                                break;
                               default        : break;
                             }
                           inmenu1 = inmenu12 =0; return; 
          }  
      } 
  }
  
//------------------------------------------------------  

void showlist( byte * list, byte i, byte col)     //вывод на lcd списка из i значений из массива *list; col - с какой позиции
   { char row1[16+5-col], *row1p; 
     row1p =row1; 
     while( i>0 && row1p <(row1+16-col) ) 
        { itoa( *( list+i-1 ), row1p, 10); row1p += strlen(row1p); i--;
          if( i ) { *row1p =','; row1p++; }
        }
     *row1p ='\0';
     for( char *chp = row1; chp < row1p; chp++ ) lcd.write(*chp); 
   }
//--------------------------------------------------------
void showgrval (byte grni, byte grvi)
  {
   lcd.setCursor(0,1); lcd.print(tvalmenu[grvi]);
   lcd.setCursor(9,1); lcd.print("    ");
   lcd.setCursor(9,1); lcd.print(tstvalues[grni][grvi]);
  }
  
//----------------------------- F MENU 2-----------------------------------------------------
void fmenu2 (byte cmd)
  { static byte menu2step, chnl, grni, grvi;      //menu2step - номер шага выполнения команды;
    byte val, gr_i;                               // 1-проведена инициализация экрана и алгоритма выполнения команды
    
    switch (cmd)
      { case m_show  : if(inmenu2 >= Menu2TOP ) inmenu2 =1;
                       menu2step =0; chnl=0;
                       lcd.clear(); lcd.print(" MENU 2 =>"); lcd.setCursor(0,1); lcd.print(tmenu2[inmenu2]);
                       return;
        case m_sshow : lcd.clear(); lcd.print("M2=>"); lcd.print(tmenu2s[inmenu2]); return;
                         
        case m_bt1S  : if( ! menu2step ) { inmenu2++;  fmenu2(m_show); return;};  // перебор команд меню2
                       switch (menu2step)  
                         { case  1 : switch( inmenu2 )
                                       { case Ini255set : case TstGr1 : case TstGr2  : case TstGr3 : 
                                                          if( chnl < DmxOChLast) chnl++; 
                                                          lcd.setCursor(0,1); lcd.print( chnl ); return;
                                         default        : break;
                                       }
                                     break;
                           case  2 : switch( inmenu2 )
                                       { case TstGr1v : case TstGr2v : case TstGr3v :
                                                        grvi++; if( grvi >= GrValMaxN) grvi=0; showgrval(grni, grvi); return;
                                         default      : break;
                                       }
                                     break;
                           case  3 : switch( inmenu2 )
                                       { case TstGr1v : case TstGr2v : case TstGr3v :
                                                        tstvalues[grni][grvi]++; showgrval(grni, grvi); return;
                                         default      : break;
                                       }
                                     break;  
                           default : break; 
                         }
        case m_bt2S  : if( ! menu2step )
                         { menu2step =1; fmenu2( m_sshow );
                           chnl =0; gr_i = TstGR1i;
                           switch( inmenu2 )                 //инициализация выполнения и отображения выбранной команды
                              { case Ini255set : lcd.setCursor(3,1); lcd.print(":");
                                                 showlist(ini255list, ini255n, 4);    //4=col - номер позиции на экране с которой выводим: 0,1,2...
                                                 lcd.setCursor(0,1); return;
                                                 
                                case IOtransit : lcd.setCursor(0,1); lcd.print("IChn= ");

                                case TstGr3    : gr_i++;                          
                                case TstGr2    : gr_i++;                
                                case TstGr1    : lcd.setCursor(3,1); lcd.print(":");
                                                 showlist(testgr[gr_i], testgrn[gr_i], 4);      lcd.setCursor(0,1); return;
                                                 
                                case TstGr3v   : gr_i++;                 
                                case TstGr2v   : gr_i++;                                                  
                                case TstGr1v   : grvi =MinVal; showgrval(gr_i, grvi); menu2step =2; return;
                                                                
                                default        : break;
                              }
                           return;
                         }
                       switch (menu2step)  
                         { case  1 : switch( inmenu2 )
                                       { case Ini255set : case TstGr1 : case TstGr2  : case TstGr3 : 
                                                          if( chnl+10 < DmxOChLast ) chnl +=10; else chnl =1;
                                                          lcd.setCursor(0,1); lcd.print( chnl ); return;
                                         default        : break;
                                       }
                                     break;
                           case  2 : switch( inmenu2 )
                                       { case TstGr1v : case TstGr2v : case TstGr3v :
                                                        lcd.setCursor(9,1); menu2step=3; return;
                                         default      : break;
                                       }
                                     break;
                           case  3 : switch( inmenu2 )
                                       { case TstGr1v : case TstGr2v : case TstGr3v :
                                                        tstvalues[grni][grvi]+=10; showgrval(grni, grvi); return;
                                         default      : break;
                                       }
                                     break;               
                           default : break; 
                         }
        case m_bt2L  : if( ! menu2step ) return;
                       gr_i = TstGR1i;
                       switch( inmenu2 )
                          { case Ini255set : if( ini255n >= Ini255MaxN ) { menuerr(); return;}
                                             ini255list[ ini255n++ ] =chnl;
                                             menu2step =0; fmenu2( m_bt2S ); return;
                                             
                            case TstGr3    : gr_i++;                 
                            case TstGr2    : gr_i++;                 
                            case TstGr1    : if( testgrn[gr_i] >= TstGrMaxN ) { menuerr(); return;}
                                             testgr[gr_i][ testgrn[gr_i]++ ] =chnl;
                                             menu2step =0; fmenu2( m_bt2S ); return;

                            case TstGr1v   : case TstGr2v : case TstGr3v :
                                             if(menu2step != 3)             return;
                                             menu2step=2; fmenu2(m_bt1S);   return;
                            default        : return;
                          }
      }
  }
  
//-------------------------------------------------------------------------------------------
void btn1short()
  { if( inmenu2 ) fmenu2( m_bt1S );
       else { fmenu1(m_bt1S);}
  }

void btn1long()
  { inmenu1 =0;
    if( inmenu2 ) {inmenu2 =0;  return;}     //выход из режима меню2...                   
    inmenu2 =1;  fmenu2(m_show);             //вход в меню2
  }

void btn2short()
  { if( inmenu2 ) { fmenu2( m_bt2S ); return; };
        
    if( inmenu1 ) {fmenu1( m_bt2S ); return;}   //выполнение пункта меню1 и выход
  }

void btn2long()
  { if( inmenu2 ) { fmenu2( m_bt2L ); return; };
    if( inmenu1 ) {fmenu1( m_bt2L ); return;}   //выполнение пункта меню1 и выход
  }
  
//--------------------------------------------------------------------------  
void btnprocessing()
  { if(digitalRead(Btn1pin)==LOW) 
            switch ( btn1state )
                { case 0 : btn1state =1; btn1time=1; break;
                  case 1 : btn1time++; 
                           if(btn1time > BtnLongPr *10 /Tick10ms) {btn1state =2; btn1long();}
                  case 2 : break;
                }
       else switch ( btn1state )
                { case 0 : break;
                  case 1 : btn1short();
                  case 2 : btn1state =0; 
                }

    if(digitalRead(Btn2pin)==LOW) 
            switch ( btn2state )
                { case 0 : btn2state =1; btn2time=1; break;
                  case 1 : btn2time++; 
                           if(btn2time > BtnLongPr *10 /Tick10ms) {btn2state =2; btn2long();}
                  case 2 : break;
                }
       else switch ( btn2state )
                { case 0 : break;
                  case 1 : btn2short();
                  case 2 : btn2state =0; 
                }
    fmenu1( m_tmout );               
  }

//--------------------------------------------------------------------------
static char Versn[]="DIOL-127";

  void stscr_init()
    { lcd.clear();
      lcd.print(Versn);
      lcd.setCursor(9,0);lcd.print("Er=");
      lcd.setCursor(0,1);lcd.print("0    packs rcv");
      
//      lcd.setCursor(0,3);lcd.print("Loop= ");
    }
//----------------------------------------------------------------------------------------------
void statescreen()
  {  
   static byte inited, old_prcerr, old_busyf, blinkc;
   static int old_packn;
   
   if(++blinkc >20) blinkc=0;
   if( inmenu1 || inmenu2 ) { inited =0; return; }
   if( ! inited )
     { inited =1; stscr_init();}
     
   if( procerror != old_prcerr || inited ==1 ) {lcd.setCursor(12,0);lcd.print(procerror); old_prcerr =procerror;}
   if( packnum != old_packn || inited ==1 ) {lcd.setCursor(0,1); lcd.print(packnum); old_packn = packnum;}
   
   if( busyflag != old_busyf || inited ==1 ) { lcd.setCursor(15,1); 
                                               if(busyflag ) lcd.print(">"); else lcd.print(" "); 
                                               old_busyf = busyflag;}
   inited =2;

   
//    lcd.setCursor(12,1);lcd.print(scripti); lcd.print("   ");
//    if(inloop) {lcd.setCursor(6,3);lcd.print(loopcount); lcd.print("   ");}
//      else {lcd.setCursor(6,3);lcd.print("       ");};
//    lcd.setCursor(9,3);  lcd.print("        ");
 //   lcdprnt(scripttm,5); 
 //   for(byte i=1; i <8; i++ ) lcd.write(32);
  }
  
//=========================================================================================================  
void loop() 
  { 
    if( millis() < timems ) {digitalWrite(TestPin1, LOW); return;} //ожидаем очередного тика
    digitalWrite(TestPin1, HIGH);      // Индикация свободного времени процессора
    timems = millis()+ Tick10ms * 10;
    
    btnprocessing();    //обрабатывает нажатия кнопок и управление меню
    if( wavetstgrgo )   //отработка второй полуволны WAVE-теста
      if( ! wavetsttime-- )   
         { ftstgrsendv( wavetstgrgo -1, tstvalues[wavetstgrgo -1][MinVal], tstvalues[wavetstgrgo -1][Tau01s] ); wavetstgrgo =0;}

    projection();          //отображение входных каналов на выходные
    busyflag=0;
         
//    scriptstep(scriptp);  
    scriptstep(scriptp, scripti, scriptl, scrresident, &scrktxreg);   //обработка текущей позиции регулярного скрипта
    scriptstep(scriptp2, scripti2, scriptl2, 0, &scrktxadd);      //обработка текущей позиции добавочного скрипта
    
    processtep();         //выдача команд в каналы, находящиеся в обработке
    
    ODmxp->TxIntEn();

    if( Udp.parsePacket() ) 
      { if(bufbusyl ==0) 
          { packnum++;                  //  lcd.setCursor(0,1); lcd.print(packnum);
            bufbusyl = Udp.available(); 
            if( bufbusyl > UdpPacMaxL ) {bufbusyl = UdpPacMaxL ;procerror |= 128+64+1;}; //слишком большой Udp-пакет(усечен) =193=!!!!!!!
            if( bufbusyl > UdpPacRecL ) {procerror |= 128+64+32+1;}; // Udp-пакет больше рекомендованного(508байт) ===========225=!!!!!!!
            Udp.read(udpbuff, bufbusyl);
            udpclear();
          } else   {  procerror |= 128+64+32+2;                   //буфер не освобожден к приходу очередного пакета =226=!!!!!!!!!!!!!
                      if(scrresident) { bufbusyl=0;}   //возможна потеря информации
                   }   
      }
    if( bufbusyl ) getpack();

    statescreen();
    
    IDmxp->RxIntEn();
  }

  /*    
   if(digitalRead(32)==LOW) button32=1;       //***инициализация
        else if (button32)
                { button31=0;
                  procesreset();  //Очистка массивов конвейера
                  projreset();    //Очистка массива проекций
                  for(int i=0; i < DmxOChLast; i++) *(ODmxp->TxBuffer+i)=0x0; //Обнуление выходного буфера 
                }

 
*/
