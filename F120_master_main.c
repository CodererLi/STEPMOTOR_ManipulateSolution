//-----------------------------------------------------------------------------
// F12x_SMBus_Master_Multibyte.c
//-----------------------------------------------------------------------------
// Copyright 2006 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// Example software to demonstrate the C8051F12x SMBus interface in
// Master mode.
// - Interrupt-driven SMBus implementation
// - Only master states defined (no slave or arbitration)
// - multiple-byte SMBus data holders used for each transmit and receive
// - Timer3 used by SMBus for SCL low timeout detection
// - SCL frequency defined by <SMB_FREQUENCY> constant
// - ARBLOST support included
// - supports multiple-byte writes and multiple-byte reads
// - Pinout:
//    P0.0 -> SDA (SMBus)
//    P0.1 -> SCL (SMBus)
//
//    P1.6 -> LED
//
//    all other port pins unused
//
// How To Test:
//
// 1) Download code to a 'F12x device that is connected to a SMBus slave.
// 2) Run the code:
//         a) The test will indicate proper communication with the slave by
//            toggling the LED on and off each time a value is sent and
//            received.
//         b) The best method to view the proper functionality is to run to
//            the indicated line of code in the TEST CODE section of main and
//            view the SMB_DATA_IN and SMB_DATA_OUT variable arrays in the
//            Watch Window.
//
//
// FID:            12X000018
// Target:         C8051F12x
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
// Command Line:   None
//
// Release 1.0
//    -Initial Revision (TP)
//    -19 APR 2006
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <C8051F120.h>                 // SFR declarations

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define  SYSCLK         24500000L       // System clock frequency in Hz

#define  SMB_FREQUENCY  10000L         // Target SCL clock rate
                                       // This example supports between 10kHz
                                       // and 100kHz
#define BAUDRATE     115200            // Baud rate of UART in bps

#define  WRITE          0x00           // WRITE direction bit
#define  READ           0x01           // READ direction bit

// Device addresses (7 bits, lsb is a don't care)
#define  SLAVE_TURN_ADDR     0xF0           // Device address for slave target
#define  SLAVE_RUN_ADDR      0xF2

#define  MY_ADDR        0x02           // Address of this SMBus device
                                       // (dummy value since this device does
                                       // not have any defined slave states)

#define  SMB_BUS_ERROR  0x00           // (all modes) BUS ERROR
#define  SMB_START      0x08           // (MT & MR) START transmitted
#define  SMB_RP_START   0x10           // (MT & MR) repeated START
#define  SMB_MTADDACK   0x18           // (MT) Slave address + W transmitted;
                                       //    ACK received
#define  SMB_MTADDNACK  0x20           // (MT) Slave address + W transmitted;
                                       //    NACK received
#define  SMB_MTDBACK    0x28           // (MT) data byte transmitted;
                                       //    ACK rec'vd
#define  SMB_MTDBNACK   0x30           // (MT) data byte transmitted;
                                       //    NACK rec'vd
#define  SMB_MTARBLOST  0x38           // (MT) arbitration lost
#define  SMB_MRADDACK   0x40           // (MR) Slave address + R transmitted;
                                       //    ACK received
#define  SMB_MRADDNACK  0x48           // (MR) Slave address + R transmitted;
                                       //    NACK received
#define  SMB_MRDBACK    0x50           // (MR) data byte rec'vd;
                                       //    ACK transmitted
#define  SMB_MRDBNACK   0x58           // (MR) data byte rec'vd;
                                       //    NACK transmitted


#define  NUM_BYTES_WR_TURN   2              // Number of bytes to write
                                       // Master -> Slave
#define  NUM_BYTES_RD_TURN   1              // Number of bytes to read
                                       // Master <- Slave
#define  NUM_BYTES_WR_RUN    1

#define  NUM_BYTES_RD_RUN    1

#define  UART_RECEIVE   4
//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------

// Global holder for SMBus data
// All receive data is written here
unsigned char SMB_DATA_IN_TURN[NUM_BYTES_RD_TURN];
unsigned char SMB_DATA_IN_RUN[NUM_BYTES_RD_RUN];

// Global holder for SMBus data.
// All transmit data is read from here
unsigned char SMB_DATA_OUT_TURN[NUM_BYTES_WR_TURN];
unsigned char SMB_DATA_OUT_RUN[NUM_BYTES_WR_RUN];


unsigned char TARGET;                  // Target SMBus slave address

bit SMB_BUSY;                          // Software flag to indicate when the
                                       // SMB_Read() or SMB_Write() functions
                                       // have claimed the SMBus

bit SMB_RW;                            // Software flag to indicate the
                                       // direction of the current transfer

unsigned short int TurnStepNumber;
unsigned char TurnStepNumber_HighByte;
bit TurnDirection;
bit RunDirection;
unsigned char TurnAngle;

bit RX_READY = 0;
bit START = 0;
bit VelocitySCALE = 1;
unsigned char UART_RECEIVE_ARRAY[UART_RECEIVE];
unsigned char UART_RECEIVE_COUNT = 0;
unsigned short int CheckTransit;
unsigned char CheckTransit_LowByte;
unsigned char TurnAngle;
unsigned char StateOrder; 


unsigned char TurnCounter = 0;
unsigned char a;
unsigned char b;

// 16-bit SFR declarations
sfr16    RCAP3    = 0xCA;              // Timer3 reload registers
sfr16    TMR3     = 0xCC;              // Timer3 counter registers
sfr16 RCAP2    = 0xca;                 // Timer2 capture/reload
sfr16 TMR2     = 0xcc;                 // Timer2

sbit LED = P1^6;                       // LED on P1.6

sbit SDA = P0^2;                       // SMBus on P0.2
sbit SCL = P0^3;                       // and P0.3

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void SYSCLK_Init(void);
void Port_Init(void);
void SMBus_Init(void);
void UART0_Init (void);
void Timer3_Init(void);

void SMBus_ISR(void);
void Timer3_ISR(void);

void SMB_Write (void);
void SMB_Read (void);
void T0_Wait_ms (unsigned char ms);

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------

void MAIN (void)
{
   volatile unsigned char data_count;  // SMB_DATA_IN and SMB_DATA_OUT counter
   unsigned char i;                    // Dummy variable counters

   WDTCN = 0xde;                       // Disable watchdog timer
   WDTCN = 0xad;

   SYSCLK_Init ();                     // Set internal oscillator to a setting
                                       // of 24.5 MHz

   // If slave is holding SDA low because of an improper SMBus reset or error
   while(!SDA)
   {
      // Provide clock pulses to allow the slave to advance out
      // of its current state. This will allow it to release SDA.
      XBR1 = 0x40;                     // Enable Crossbar
      SCL = 0;                         // Drive the clock low
      for(i = 0; i < 255; i++);        // Hold the clock low
      SCL = 1;                         // Release the clock
      while(!SCL);                     // Wait for open-drain
                                       // clock output to rise
      for(i = 0; i < 10; i++);         // Hold the clock high
      XBR1 = 0x00;                     // Disable Crossbar
   }

   Port_Init ();                       // Initialize Crossbar and GPIO

   // Turn off the LED before the test starts
   LED = 0;

   SMBus_Init ();                      // Configure and enable SMBus
    UART0_Init ();
   Timer3_Init ();                     // Configure and enable Timer3

   EIE1 |= 0x02;                       // Enable the SMBus interrupt

   EA = 1;                             // Global interrupt enable


   SFRPAGE = SMB0_PAGE;
   SI = 0;

// TEST CODE-------------------------------------------------------------------

   while (1)
   {  
      while(RX_READY)
      {  
             RX_READY = 0;
         START = StateOrder >> 3;
         VelocitySCALE = StateOrder >> 2;
         TurnDirection = StateOrder;
         RunDirection = StateOrder >> 1;
         TurnStepNumber = TurnAngle / 0.45;
         TurnStepNumber_HighByte = TurnStepNumber >> 8;
            // SMBus Write Sequence
           //if(TurnCounter == 0){
         for (data_count = 0; data_count < NUM_BYTES_WR_TURN; data_count++)
         {   
            if(data_count == 0)
            {
               SMB_DATA_OUT_TURN[data_count] = TurnStepNumber_HighByte;
               if(START)
               {
                  SMB_DATA_OUT_TURN[data_count] = SMB_DATA_OUT_TURN[data_count] | 0x80;
               }else
               {
                  SMB_DATA_OUT_TURN[data_count] = SMB_DATA_OUT_TURN[data_count] | 0x00;
               }
               if(TurnDirection)
               {
                  SMB_DATA_OUT_TURN[data_count] = SMB_DATA_OUT_TURN[data_count] | 0x40;
               }else
               {
                  SMB_DATA_OUT_TURN[data_count] = SMB_DATA_OUT_TURN[data_count] | 0x00;
               }
            }else
            {
               SMB_DATA_OUT_TURN[data_count] = (unsigned char) TurnStepNumber;
            }
         }

         TARGET = SLAVE_TURN_ADDR;
         SMB_Write();                     // Initiate SMBus write
         while(SMB_BUSY);

         for(data_count = 0; data_count < NUM_BYTES_WR_RUN; data_count++)
         {  
            SMB_DATA_OUT_RUN[data_count] = 0;
            if(START)
               {
                  SMB_DATA_OUT_RUN[data_count] = SMB_DATA_OUT_RUN[data_count] | 0x40;
               }else
               {
                  SMB_DATA_OUT_RUN[data_count] = SMB_DATA_OUT_RUN[data_count] | 0x00;
               }
               if(VelocitySCALE)
               {
                  SMB_DATA_OUT_RUN[data_count] = SMB_DATA_OUT_RUN[data_count] | 0x20;
               }else
               {
                  SMB_DATA_OUT_RUN[data_count] = SMB_DATA_OUT_RUN[data_count] | 0x00;
               }
               if(RunDirection)
               {
                  SMB_DATA_OUT_RUN[data_count] = SMB_DATA_OUT_RUN[data_count] | 0x01;
               }else
               {
                  SMB_DATA_OUT_RUN[data_count] = SMB_DATA_OUT_RUN[data_count] | 0x00;
               }
         }
         TARGET = SLAVE_RUN_ADDR;
         SMB_Write();                     // Initiate SMBus write
         while(SMB_BUSY);

         //LED = ~LED;
         T0_Wait_ms (2);

      }
      
   }

// END TEST CODE---------------------------------------------------------------

}

//-----------------------------------------------------------------------------
// Initialization Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This routine initializes the system clock to use the internal oscillator
// at 6.125 MHz (24.5 / 4 MHz).
//
void SYSCLK_Init (void)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = CONFIG_PAGE;              // Set SFR page

   OSCICN = 0x83;                      // Set internal oscillator to run
                                       // at its maximum frequency

   CLKSEL = 0x00;                      // Select the internal osc. as
                                       // the SYSCLK source

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
                                       // detector
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configure the Crossbar and GPIO ports.
//
// P0.0   digital   push-pull    UART TX
// P0.1   digital   open-drain    UART RX
// P0.2     digital   open-drain    SMBus SDA
// P0.3   digital   open-drain    SMBus SCL
// P1.6   digital   push-pull     LED
//
// all other port pins unused
//
// Note: If the SMBus is moved, the SCL and SDA sbit declarations must also
// be adjusted.
//
void PORT_Init (void)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = CONFIG_PAGE;

   P0MDOUT = 0x01;                     // All P0 pins open-drain output except UART TX

   P1MDOUT |= 0x40;                    // Make the LED (P1.6) a push-pull
                                       // output

   XBR0 = 0x05;                        // Enable SMBus on the crossbar
   XBR2 = 0x40;                        // Enable crossbar and weak pull-ups

   P0 = 0xFF;

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page detector
}

//-----------------------------------------------------------------------------
// SMBus_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// The SMBus peripheral is configured as follows:
// - SMBus enabled
// - Assert Acknowledge low (AA bit = 1b)
// - Free and SCL low timeout detection enabled
//
void SMBus_Init (void)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = SMB0_PAGE;

   SMB0CN = 0x07;                      // Assert Acknowledge low (AA bit = 1b);
                                       // Enable SMBus Free timeout detect;
                                       // Enable SCL low timeout detect

   // SMBus clock rate (derived approximation from the Tlow and Thigh equations
   // in the SMB0CR register description)
   SMB0CR = 257 - (SYSCLK / (8 * SMB_FREQUENCY));

   SMB0ADR = MY_ADDR;                  // Set own slave address.

   SMB0CN |= 0x40;                     // Enable SMBus;

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page detector
}

//-----------------------------------------------------------------------------
// UART0_Init   Variable baud rate, Timer 2, 8-N-1
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configure UART0 for operation at <baudrate> 8-N-1 using Timer2 as
// baud rate source.
//
//-----------------------------------------------------------------------------
void UART0_Init (void)
{
   char SFRPAGE_SAVE;

   SFRPAGE_SAVE = SFRPAGE;             // Preserve SFRPAGE

   SFRPAGE = TMR2_PAGE;

   TMR2CN = 0x00;                      // Timer in 16-bit auto-reload up timer
                                       // mode
   TMR2CF = 0x08;                      // SYSCLK is time base; no output;
                                       // up count only
   RCAP2 = - ((long) SYSCLK/BAUDRATE/16);
   TMR2 = RCAP2;
   TR2= 1;                             // Start Timer2

   SFRPAGE = UART0_PAGE;

   SCON0 = 0x50;                       // 8-bit variable baud rate;
                                       // 9th bit ignored; RX enabled
                                       // clear all flags
   SSTA0 = 0x15;                       // Clear all flags; enable baud rate
                                       // doubler (not relevant for these
                                       // timers);
                                       // Use Timer2 as RX and TX baud rate
                                       // source;
   ES0 = 1;  
   IP |= 0x10;

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
}

//-----------------------------------------------------------------------------
// Timer3_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Timer3 configured for use by the SMBus low timeout detect feature as
// follows:
// - Timer3 in auto-reload mode
// - SYSCLK/12 as Timer3 clock source
// - Timer3 reload registers loaded for a 25ms overflow period
// - Timer3 pre-loaded to overflow after 25ms
// - Timer3 enabled
//
void Timer3_Init (void)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = TMR3_PAGE;

   TMR3CN = 0x00;                      // Timer3 external enable off;
                                       // Timer3 in timer mode;
                                       // Timer3 in auto-reload mode

   TMR3CF = 0x00;                      // Timer3 uses SYSCLK/12
                                       // Timer3 output not available
                                       // Timer3 counts up

   RCAP3 = -(SYSCLK/12/40);            // Timer3 configured to overflow after
   TMR3 = RCAP3;                       // ~25ms (for SMBus low timeout detect)

   EIE2 |= 0x01;                       // Timer3 interrupt enable
   TR3 = 1;                            // Start Timer3

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page detector
}

//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SMBus Interrupt Service Routine (ISR)
//-----------------------------------------------------------------------------
//
// SMBus ISR state machine
// - Master only implementation - no slave or arbitration states defined
// - All incoming data is written to global array <SMB_DATA_IN>
// - All outgoing data is read from global array <SMB_DATA_OUT>
//
void SMBUS_ISR (void) interrupt 7
{
   bit FAIL = 0;                       // Used by the ISR to flag failed
                                       // transfers

   static unsigned char sent_byte_counter;
   static unsigned char rec_byte_counter;

   // Status code for the SMBus (SMB0STA register)
   switch (SMB0STA)
   {
      // Master Transmitter/Receiver: START condition transmitted.
      // Load SMB0DAT with slave device address.
      case SMB_START:

      // Master Transmitter/Receiver: repeated START condition transmitted.
      // Load SMB0DAT with slave device address
      case SMB_RP_START:
         SMB0DAT = TARGET;             // Load address of the slave.
         SMB0DAT &= 0xFE;              // Clear the LSB of the address for the
                                       // R/W bit
         SMB0DAT |= SMB_RW;            // Load R/W bit
         STA = 0;                      // Manually clear STA bit

         rec_byte_counter = 1;         // Reset the counter
         sent_byte_counter = 1;        // Reset the counter

         break;

      // Master Transmitter: Slave address + WRITE transmitted.  ACK received.
      // For a READ: N/A
      //
      // For a WRITE: Send the first data byte to the slave.
      case SMB_MTADDACK:
             if(TARGET == SLAVE_TURN_ADDR)
             {
                SMB0DAT = SMB_DATA_OUT_TURN[sent_byte_counter-1];
                sent_byte_counter++;
             }else
             {
                SMB0DAT = SMB_DATA_OUT_RUN[sent_byte_counter-1];
                sent_byte_counter++;
         }

         break;

      // Master Transmitter: Slave address + WRITE transmitted.  NACK received.
      // Restart the transfer.
      case SMB_MTADDNACK:
         STA = 1;                      // Restart transfer
         break;

      // Master Transmitter: Data byte transmitted.  ACK received.
      // For a READ: N/A
      //
      // For a WRITE: Send all data.  After the last data byte, send the stop
      //  bit.
      case SMB_MTDBACK:
             if(TARGET == SLAVE_TURN_ADDR)
             {
                if (sent_byte_counter <= NUM_BYTES_WR_TURN)
                {
                     // send data byte
                     SMB0DAT = SMB_DATA_OUT_TURN[sent_byte_counter-1];
                     sent_byte_counter++;
                }
                else
                {
                     STO = 1;                   // Set STO to terminate transfer
                     SMB_BUSY = 0;              // And free SMBus interface
                }
             }else
             {
                if (sent_byte_counter <= NUM_BYTES_WR_RUN)
                {
                     // send data byte
                     SMB0DAT = SMB_DATA_OUT_RUN[sent_byte_counter-1];
                     sent_byte_counter++;
                }
                else
                {
                     STO = 1;                   // Set STO to terminate transfer
                     SMB_BUSY = 0;              // And free SMBus interface
                }
               }
         

         break;

      // Master Transmitter: Data byte transmitted.  NACK received.
      // Restart the transfer.
      case SMB_MTDBNACK:
         STA = 1;                      // Restart transfer

         break;

      // Master Receiver: Slave address + READ transmitted.  ACK received.
      // For a READ: check if this is a one-byte transfer. if so, set the
      //  NACK after the data byte is received to end the transfer. if not,
      //  set the ACK and receive the other data bytes.
      //
      // For a WRITE: N/A
      case SMB_MRADDACK:
             if(TARGET == SLAVE_TURN_ADDR)
             {
                if (rec_byte_counter == NUM_BYTES_RD_TURN)
                {
                     AA = 0;                    // Only one byte in this transfer,
                                                             // send NACK after byte is received
                }
                else
                {
                     AA = 1;                    // More than one byte in this transfer,
                                                             // send ACK after byte is received
                }
             }else
             {
                if (rec_byte_counter == NUM_BYTES_RD_RUN)
                {
                     AA = 0;                    // Only one byte in this transfer,
                                                             // send NACK after byte is received
                }
                else
                {
                     AA = 1;                    // More than one byte in this transfer,
                                                             // send ACK after byte is received
                }
             }
         

         break;

      // Master Receiver: Slave address + READ transmitted.  NACK received.
      // Restart the transfer.
      case SMB_MRADDNACK:
         STA = 1;                      // Restart transfer

         break;

      // Master Receiver: Data byte received.  ACK transmitted.
      // For a READ: receive each byte from the slave.  if this is the last
      //  byte, send a NACK and set the STOP bit.
      //
      // For a WRITE: N/A
      case SMB_MRDBACK:
             if(TARGET == SLAVE_TURN_ADDR)
             {
                if (rec_byte_counter < NUM_BYTES_RD_TURN)
                {
                     SMB_DATA_IN_TURN[rec_byte_counter-1] = SMB0DAT; // Store received byte
                     AA = 1;                    // Send ACK to indicate byte received
                     rec_byte_counter++;        // Increment the byte counter
                }
                else
                {
                     AA = 0;                    // Send NACK to indicate last byte
                                                             // of this transfer
                }
             }else
             {
                if (rec_byte_counter < NUM_BYTES_RD_RUN)
                {
                     SMB_DATA_IN_RUN[rec_byte_counter-1] = SMB0DAT; // Store received byte
                     AA = 1;                    // Send ACK to indicate byte received
                     rec_byte_counter++;        // Increment the byte counter
                }
                else
                {
                     AA = 0;                    // Send NACK to indicate last byte
                                                             // of this transfer
                }
             }
         

         break;

      // Master Receiver: Data byte received.  NACK transmitted.
      // For a READ: Read operation has completed.  Read data register and
      //  send STOP.
      //
      // For a WRITE: N/A
      case SMB_MRDBNACK:
             if(TARGET == SLAVE_TURN_ADDR)
             {
                SMB_DATA_IN_TURN[rec_byte_counter-1] = SMB0DAT; // Store received byte
                STO = 1;
                SMB_BUSY = 0;
                AA = 1;                      // Set AA for next transfer
             }else
             {
                SMB_DATA_IN_RUN[rec_byte_counter-1] = SMB0DAT; // Store received byte
                STO = 1;
                SMB_BUSY = 0;
                AA = 1;                      // Set AA for next transfer
             }

         break;

      // Master Transmitter: Arbitration lost.
      case SMB_MTARBLOST:

         FAIL = 1;                     // Indicate failed transfer
                                       // and handle at end of ISR

         break;

      // All other status codes invalid.  Reset communication.
      default:
         FAIL = 1;

         break;
   }

   if (FAIL)                           // If the transfer failed,
   {
      SMB0CN &= ~0x40;                 // Reset communication
      SMB0CN |= 0x40;
      STA = 0;
      STO = 0;
      AA = 0;

      SMB_BUSY = 0;                    // Free SMBus

      FAIL = 0;
   }

   SI = 0;                             // Clear interrupt flag
}

//-----------------------------------------------------------------------------
// UART0_Interrupt
//-----------------------------------------------------------------------------
//
// This routine is invoked whenever a character is entered or displayed on the
// Hyperterminal.
//
//-----------------------------------------------------------------------------

void UART0_Interrupt (void) interrupt 4
{
    SFRPAGE = UART0_PAGE;

   if (RI0 == 1)
   {  
      //LED = ~LED;
      RI0 = 0;
      if(UART_RECEIVE_COUNT == 0)
      {  
         UART_RECEIVE_ARRAY[UART_RECEIVE_COUNT] = SBUF0;
         if( SBUF0 == 0xAA )
         {
            //UART_RECEIVE_ARRAY[UART_RECEIVE_COUNT] = SBUF0;
            UART_RECEIVE_COUNT++;
         }else
         {
            UART_RECEIVE_ARRAY[UART_RECEIVE_COUNT] = 0;
         }
      }else if (UART_RECEIVE_COUNT > 0 && UART_RECEIVE_COUNT < 3)
      {
         UART_RECEIVE_ARRAY[UART_RECEIVE_COUNT] = SBUF0;
         UART_RECEIVE_COUNT++;
      }else
      {
         UART_RECEIVE_ARRAY[UART_RECEIVE_COUNT] = SBUF0;
         CheckTransit = (unsigned short int)(UART_RECEIVE_ARRAY[0] + UART_RECEIVE_ARRAY[1] + UART_RECEIVE_ARRAY[2]);
         CheckTransit_LowByte = (unsigned char)CheckTransit;
         if(CheckTransit_LowByte == UART_RECEIVE_ARRAY[UART_RECEIVE_COUNT])
         {
            StateOrder = UART_RECEIVE_ARRAY[1];
            TurnAngle = UART_RECEIVE_ARRAY[2];
            RX_READY = 1;
            LED = ~LED;
         }
         UART_RECEIVE_COUNT = 0;
      }
      
   }

   if (TI0 == 1)
   {
      TI0 = 0;
   }
}

//-----------------------------------------------------------------------------
// Timer3 Interrupt Service Routine (ISR)
//-----------------------------------------------------------------------------
//
// A Timer3 interrupt indicates an SMBus SCL low timeout.
// The SMBus is disabled and re-enabled if a timeout occurs.
//
void Timer3_ISR (void) interrupt 14
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = SMB0_PAGE;

   SMB0CN &= ~0x40;                    // Disable SMBus
   SMB0CN |= 0x40;                     // Re-enable SMBus

   SFRPAGE = SFRPAGE_SAVE;             // Switch back to the Timer3 SFRPAGE

   TF3 = 0;                            // Clear Timer3 interrupt-pending flag
   SMB_BUSY = 0;                       // Free bus
}

//-----------------------------------------------------------------------------
// Support Functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SMB_Write
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Writes a single byte to the slave with address specified by the <TARGET>
// variable.
// Calling sequence:
// 1) Write target slave address to the <TARGET> variable
// 2) Write outgoing data to the <SMB_DATA_OUT> array
// 3) Call SMB_Write()
//
void SMB_Write (void)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = SMB0_PAGE;

   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)
   SMB_RW = 0;                         // Mark this transfer as a WRITE
   STA = 1;                            // Start transfer

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page detector
}

//-----------------------------------------------------------------------------
// SMB_Read
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Reads a single byte from the slave with address specified by the <TARGET>
// variable.
// Calling sequence:
// 1) Write target slave address to the <TARGET> variable
// 2) Call SMB_Write()
// 3) Read input data from <SMB_DATA_IN> array
//
void SMB_Read (void)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = SMB0_PAGE;

   while (SMB_BUSY);                   // Wait for bus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)
   SMB_RW = 1;                         // Mark this transfer as a READ

   STA = 1;                            // Start transfer

   while (SMB_BUSY);                   // Wait for transfer to complete

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page detector
}

//-----------------------------------------------------------------------------
// T0_Wait_ms
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) unsigned char ms - number of milliseconds to wait
//                        range is full range of character: 0 to 255
//
// Configure Timer0 to wait for <ms> milliseconds using SYSCLK as its time
// base.
//
void T0_Wait_ms (unsigned char ms)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = TIMER01_PAGE;

   TCON &= ~0x30;                      // Stop Timer0; Clear TF0
   TMOD &= ~0x0f;                      // 16-bit free run mode
   TMOD |=  0x01;

   CKCON |= 0x08;                      // Timer0 counts SYSCLKs

   while (ms) {
      TR0 = 0;                         // Stop Timer0
      TH0 = -(SYSCLK/1000 >> 8);       // Overflow in 1ms
      TL0 = -(SYSCLK/1000);
      TF0 = 0;                         // Clear overflow indicator
      TR0 = 1;                         // Start Timer0
      while (!TF0);                    // Wait for overflow
      ms--;                            // Update ms counter
   }

   TR0 = 0;                            // Stop Timer0

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page detector
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
