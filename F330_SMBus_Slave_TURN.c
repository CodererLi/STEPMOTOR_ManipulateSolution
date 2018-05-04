//-----------------------------------------------------------------------------
// F33x_SMBus_Slave_Multibyte.c
//-----------------------------------------------------------------------------
// Copyright 2006 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// Example software to demonstrate the C8051F33x SMBus interface in Slave mode
// - Interrupt-driven SMBus implementation
// - Only slave states defined
// - Multi-byte SMBus data holders used for both transmit and receive
// - Timer1 used as SMBus clock rate (used only for free timeout detection)
// - Timer3 used by SMBus for SCL low timeout detection
// - ARBLOST support included
// - supports multiple-byte writes and multiple-byte reads
// - Pinout:
//    P0.0 -> SDA (SMBus)
//    P0.1 -> SCL (SMBus)
//
//    P1.3 -> LED
//
//    P2.0 -> C2D (debug interface)
//
//    all other port pins unused
//
// How To Test:
//
// 1) Verify that J6 is not populated.
// 2) Download code to a 'F33x device that is connected to a SMBus master.
// 3) Run the code.  The slave code will copy the write data to the read
//    data, so a successive write and read will effectively echo the data
//    written.  To verify that the code is working properly, verify on the
//    master that the data received is the same as the data written.
//
// FID:            33X000009
// Target:         C8051F33x
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
// Command Line:   None
//
// Release 1.0
//    -Initial Revision (TP)
//    -30 MAR 2006
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <C8051F330.h>

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define  SYSCLK         24500000       // System clock frequency in Hz

#define  SMB_FREQUENCY  10000          // Target SMBus frequency
                                       // This example supports between 10kHz
                                       // and 100kHz

#define  WRITE          0x00           // SMBus WRITE command
#define  READ           0x01           // SMBus READ command

#define  SLAVE_TURN_ADDR     0xF0           // Device addresses (7 bits,
                                       // lsb is a don't care)

// Status vector - top 4 bits only
#define  SMB_SRADD      0x20           // (SR) slave address received
                                       //    (also could be a lost
                                       //    arbitration)
#define  SMB_SRSTO      0x10           // (SR) STOP detected while SR or ST,
                                       //    or lost arbitration
#define  SMB_SRDB       0x00           // (SR) data byte received, or
                                       //    lost arbitration
#define  SMB_STDB       0x40           // (ST) data byte transmitted
#define  SMB_STSTO      0x50           // (ST) STOP detected during a
                                       //    transaction; bus error
// End status vector definition

#define  NUM_BYTES_WR   2              // Number of bytes to write
                                       // Slave <- Master
#define  NUM_BYTES_RD   1              // Number of bytes to read
                                       // Slave -> Master


//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------

// Global holder for SMBus data.
// All receive data is written here
// NUM_BYTES_WR used because an SMBus write is Master->Slave
unsigned char SMB_DATA_IN[NUM_BYTES_WR];

// Global holder for SMBus data.
// All transmit data is read from here
// NUM_BYTES_RD used because an SMBus read is Slave->Master
unsigned char SMB_DATA_OUT[NUM_BYTES_RD];
unsigned short int StepCommand;
bit TurnDirection;
bit START = 0;
unsigned short int StepNumber = 0;

//bit LastTurnDirection;
//unsigned short int LastStepNumber = 0;

bit DATA_READY = 0;                    // Set to '1' by the SMBus ISR
                                       // when a new data byte has been
                                       // received.
unsigned short int count = 0;
unsigned int code SPEED[240]=
{0xEF62,0xF338,0xF581,0xF719,0xF836,0xF907,0xF9BA,0xFA45,0xFAB2,0xFB10,
 0xFB62,0xFBAA,0xFBE5,0xFC17,0xFC47,0xFC70,0xFC92,0xFCB5,0xFCD3,0xFCEE,
 0xFD06,0xFD1C,0xFD31,0xFD44,0xFD57,0xFD67,0xFD76,0xFD83,0xFD91,0xFD9D,
 0xFDA8,0xFDB3,0xFDBD,0xFDC7,0xFDD0,0xFDD8,0xFDE1,0xFDE7,0xFDEF,0xFDF6,
 0xFDFC,0xFE02,0xFE08,0xFE0E,0xFE13,0xFE18,0xFE1D,0xFE22,0xFE26,0xFE2A,
 0xFE2E,0xFE32,0xFE35,0xFE39,0xFE3D,0xFE40,0xFE43,0xFE46,0xFE48,0xFE4B,
 0xFE4E,0xFE50,0xFE53,0xFE55,0xFE58,0xFE5A,0xFE5C,0xFE5E,0xFE60,0xFE62,
 0xFE63,0xFE65,0xFE67,0xFE68,0xFE6A,0xFE6C,0xFE6D,0xFE6E,0xFE70,0xFE71,
 0xFE73,0xFE74,0xFE75,0xFE76,0xFE77,0xFE78,0xFE79,0xFE7A,0xFE7C,0xFE7C,
 0xFE7D,0xFE7E,0xFE7F,0xFE80,0xFE81,0xFE81,0xFE82,0xFE83,0xFE84,0xFE84,
 0xFE85,0xFE86,0xFE87,0xFE87,0xFE88,0xFE88,0xFE89,0xFE89,0xFE8A,0xFE8A,
 0xFE8B,0xFE8B,0xFE8C,0xFE8C,0xFE8D,0xFE8D,0xFE8E,0xFE8E,0xFE8E,0xFE8F,
 0xFE8F,0xFE90,0xFE90,0xFE90,0xFE90,0xFE91,0xFE91,0xFE91,0xFE92,0xFE92,
 0xFE92,0xFE92,0xFE93,0xFE93,0xFE93,0xFE93,0xFE94,0xFE94,0xFE94,0xFE94,
 0xFE95,0xFE95,0xFE95,0xFE96,0xFE96,0xFE96,0xFE96,0xFE96,0xFE96,0xFE97,
 0xFE97,0xFE97,0xFE97,0xFE97,0xFE97,0xFE98,0xFE98,0xFE98,0xFE98,0xFE98,
 0xFE98,0xFE98,0xFE98,0xFE98,0xFE99,0xFE99,0xFE99,0xFE99,0xFE99,0xFE99,
 0xFE99,0xFE99,0xFE99,0xFE99,0xFE9A,0xFE9A,0xFE9A,0xFE9A,0xFE9A,0xFE9A,
 0xFE9A,0xFE9A,0xFE9A,0xFE9A,0xFE9A,0xFE9A,0xFE9A,0xFE9A,0xFE9A,0xFE9B,
 0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,
 0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,
 0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9B,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,
 0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,
 0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C,0xFE9C};

// 16-bit SFR declarations
sfr16    TMR3RL   = 0x92;              // Timer3 reload registers
sfr16    TMR3     = 0x94;              // Timer3 counter registers
sfr16 TMR2RL   =  0xCA;                // Timer2 Reload Register
sfr16 TMR2  =  0xCC;                   // Timer2 Register
sbit PUL1=P1^0;
sbit PUL2=P1^1;
sbit PUL3=P1^2;
sbit PUL4=P0^2;
sbit DIR1=P1^4;
sbit DIR2=P1^5;
sbit DIR3=P1^6;
sbit DIR4=P1^7;   
//sbit LED=P2^0;

//end 16-bit SFR declarations
//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void SMBus_Init (void);
void Timer1_Init (void);
void Timer2_Init (void); 
void Timer3_Init (void);
void Port_Init (void);

void SMBus_ISR (void);
void Timer2_ISR(void);
void Timer3_ISR (void);
//void T0_Wait_ms (unsigned int ms);

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
//
// Main routine performs all configuration tasks, then waits for SMBus
// communication.
//
void main (void)
{
   unsigned char i;

   PCA0MD &= ~0x40;                    // WDTE = 0 (Disable watchdog
                                       // timer)


   OSCICN |= 0x03;                     // Set internal oscillator to highest
                                       // setting of 24500000

   Port_Init();                        // Initialize Crossbar and GPIO
   Timer1_Init();                      // Configure Timer1 for use
                                       // with SMBus baud rate
    Timer2_Init();
   Timer3_Init ();                     // Configure Timer3 for use with
                                       // SCL low timeout detect

   SMBus_Init ();                      // Configure and enable SMBus

   EIE1 |= 0x01;                       // Enable the SMBus interrupt

//   LED = 1;

   EA = 1;                             // Global interrupt enable

   // Initialize the outgoing data array in case a read is done before a
   // write
   for (i = 0; i < NUM_BYTES_RD; i++)
   {
      SMB_DATA_OUT[i] = 0xFD;
   }

   while(1)
   {
      while(!DATA_READY);              // New SMBus data received?

      DATA_READY = 0;

      // Copy the data from the input array to the output array
      StepCommand = SMB_DATA_IN[0];      // Store 8-high-bit incoming data
      StepCommand = (StepCommand << 8);  // left offset 8-bit
      StepCommand = StepCommand + SMB_DATA_IN[1];
      TurnDirection = (StepCommand & 0x4000) >> 14;
      START = (StepCommand & 0x8000) >> 15;
      StepNumber = (StepCommand & 0x3FFF);
      //LED = ~LED;
      if(START)
      {
         TR2 = 1;
      }else
         {
            TR2 = 0;
         }
   }
}

//-----------------------------------------------------------------------------
// Initialization Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SMBus_Init()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// SMBus configured as follows:
// - SMBus enabled
// - Slave mode not inhibited
// - Timer1 used as clock source. The maximum SCL frequency will be
//   approximately 1/3 the Timer1 overflow rate
// - Setup and hold time extensions enabled
// - Bus Free and SCL Low timeout detection enabled
//
void SMBus_Init (void)
{
   SMB0CF = 0x1D;                      // Use Timer1 overflows as SMBus clock
                                       // source;
                                       // Enable slave mode;
                                       // Enable setup & hold time
                                       // extensions;
                                       // Enable SMBus Free timeout detect;
                                       // Enable SCL low timeout detect;

   SMB0CF |= 0x80;                     // Enable SMBus;
}

//-----------------------------------------------------------------------------
// Timer1_Init()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Timer1 configured as the SMBus clock source as follows:
// - Timer1 in 8-bit auto-reload mode
// - SYSCLK or SYSCLK / 4 as Timer1 clock source
// - Timer1 overflow rate => 3 * SMB_FREQUENCY
// - The resulting SCL clock rate will be ~1/3 the Timer1 overflow rate
// - Timer1 enabled
//
void Timer1_Init (void)
{

// Make sure the Timer can produce the appropriate frequency in 8-bit mode
// Supported SMBus Frequencies range from 10kHz to 100kHz.  The CKCON register
// settings may need to change for frequencies outside this range.
#if ((SYSCLK/SMB_FREQUENCY/3) < 255)
   #define SCALE 1
      CKCON |= 0x08;                   // Timer1 clock source = SYSCLK
#elif ((SYSCLK/SMB_FREQUENCY/4/3) < 255)
   #define SCALE 4
      CKCON |= 0x01;
      CKCON &= ~0x0A;                  // Timer1 clock source = SYSCLK / 4
#endif

   TMOD = 0x20;                        // Timer1 in 8-bit auto-reload mode

   // Timer1 configured to overflow at 1/3 the rate defined by SMB_FREQUENCY
   TH1 = -(SYSCLK/SMB_FREQUENCY/SCALE/3);

   TL1 = TH1;                          // Init Timer1

   TR1 = 1;                            // Timer1 enabled
}

//-----------------------------------------------------------------------------
// Timer2_Init
//-----------------------------------------------------------------------------
//
void Timer2_Init(void)
{
   TMR2CN &= ~0x01;
   TMR2RL = SPEED[2];
   TMR2 = TMR2RL;                      // Init the Timer2 register

   TR2 = 0;
   ET2 = 1;                            // Timer2 interrupt enabled
}

//-----------------------------------------------------------------------------
// Timer3_Init()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Timer3 configured for use by the SMBus low timeout detect feature as
// follows:
// - Timer3 in 16-bit auto-reload mode
// - SYSCLK/12 as Timer3 clock source
// - Timer3 reload registers loaded for a 25ms overflow period
// - Timer3 pre-loaded to overflow after 25ms
// - Timer3 enabled
//
void Timer3_Init (void)
{
   TMR3CN = 0x00;                      // Timer3 configured for 16-bit auto-
                                       // reload, low-byte interrupt disabled

   CKCON &= ~0x40;                     // Timer3 uses SYSCLK/12

   TMR3RL = -(SYSCLK/12/40);           // Timer3 configured to overflow after
   TMR3 = TMR3RL;                      // ~25ms (for SMBus low timeout detect):
                                       // 1/.025 = 40

   EIE1 |= 0x80;                       // Timer3 interrupt enable
   TMR3CN |= 0x04;                     // Start Timer3
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
// P0.0   digital   open-drain    SMBus SDA
// P0.1   digital   open-drain    SMBus SCL
//
// P2.0   digital   push-pull     LED
//
// all other port pins unused
//
void PORT_Init (void)
{
   P0MDOUT = 0x04;                     // All P0 pins open-drain output

   //P2MDOUT |= 0x01;                    // Make the LED (P2.0) a push-pull
                                       // output
    P1MDOUT = 0xFF;

   XBR0 = 0x04;                        // Enable SMBus pins
   XBR1 = 0x40;                        // Enable crossbar and weak pull-ups

   P0 = 0xFF;
}


//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// SMBus Interrupt Service Routine (ISR)
//-----------------------------------------------------------------------------
//
// SMBus ISR state machine
// - Slave only implementation - no master states defined
// - All incoming data is written to global variable <SMB_data_IN>
// - All outgoing data is read from global variable <SMB_data_OUT>
//
void SMBus_ISR (void) interrupt 7
{
   static unsigned char sent_byte_counter;
   static unsigned char rec_byte_counter;

   if (ARBLOST == 0)
   {
      switch (SMB0CN & 0xF0)           // Decode the SMBus status vector
      {
         // Slave Receiver: Start+Address received
         case  SMB_SRADD:

            STA = 0;                   // Clear STA bit

            sent_byte_counter = 1;     // Reinitialize the data counters
            rec_byte_counter = 1;

            if((SMB0DAT&0xFE) == (SLAVE_TURN_ADDR&0xFE)) // Decode address
            {                          // If the received address matches,
               ACK = 1;                // ACK the received slave address

               if((SMB0DAT&0x01) == READ) // If the transfer is a master READ,
               {
                  // Prepare outgoing byte
                  SMB0DAT = SMB_DATA_OUT[sent_byte_counter-1];
                  sent_byte_counter++;
               }
            }
            else                       // If received slave address does not
            {                          // match,
               ACK = 0;                // NACK received address
            }
            break;

         // Slave Receiver: Data received
         case  SMB_SRDB:

            if (rec_byte_counter < NUM_BYTES_WR)
            {
               // Store incoming data
               SMB_DATA_IN[rec_byte_counter-1] = SMB0DAT;
               rec_byte_counter++;

               ACK = 1;                // ACK received data
            }
            else
            {
               // Store incoming data
               SMB_DATA_IN[rec_byte_counter-1] = SMB0DAT;

               DATA_READY = 1;         // Indicate new data fully received
               ACK = 1;                // ACK received data
            }

            break;

         // Slave Receiver: Stop received while either a Slave Receiver or
         // Slave Transmitter
         case  SMB_SRSTO:

            STO = 0;                   // STO must be cleared by software when
                                       // a STOP is detected as a slave
            break;

         // Slave Transmitter: Data byte transmitted
         case  SMB_STDB:

            if (ACK == 1)              // If Master ACK's, send the next byte
            {
               if (sent_byte_counter <= NUM_BYTES_RD)
               {
                  // Prepare next outgoing byte
                  SMB0DAT = SMB_DATA_OUT[sent_byte_counter-1];
                  sent_byte_counter++;
               }
            }                          // Otherwise, do nothing
            break;

         // Slave Transmitter: Arbitration lost, Stop detected
         //
         // This state will only be entered on a bus error condition.
         // In normal operation, the slave is no longer sending data or has
         // data pending when a STOP is received from the master, so the TXMODE
         // bit is cleared and the slave goes to the SRSTO state.
         case  SMB_STSTO:

            STO = 0;                   // STO must be cleared by software when
                                       // a STOP is detected as a slave
            break;

         // Default: all other cases undefined
         default:

            SMB0CF &= ~0x80;           // Reset communication
            SMB0CF |= 0x80;
            STA = 0;
            STO = 0;
            ACK = 0;
            break;
      }
   }
   // ARBLOST = 1, Abort failed transfer
   else
   {
      STA = 0;
      STO = 0;
      ACK = 0;
   }

   SI = 0;                             // Clear SMBus interrupt flag
}

void Timer2_ISR (void) interrupt 5
{
      if (count < StepNumber)
      {
         if (TurnDirection)
         {
            DIR1 = 1;
            DIR2 = 1;
            DIR3 = 1;
            DIR4 = 1;            
         } else
         {  
            DIR1 = 0;
            DIR2 = 0;
            DIR3 = 0;
            DIR4 = 0;
         }
         PUL1 = ~PUL1;
         PUL2 = ~PUL2;
         PUL3 = ~PUL3;
         PUL4 = ~PUL4; 

         count++;
         // if(count < StepNumber/5)
         // {
         //    accelerate = 1;
         //    decelerate = 0;
         //    deaccdone = 0;
         // }else if(count > StepNumber*4/5)
         // {
         //    accelerate = 0;
         //    decelerate = 1;
         //    accdone = 0;
         // }
      }else
      {
         count = 0;
         TR2 = 0;
         //LastTurnDirection = TurnDirection;
         //LastStepNumber = StepNumber;
      }
   
    TF2H = 0;
 }

//-----------------------------------------------------------------------------
// Timer3 Interrupt Service Routine (ISR)
//-----------------------------------------------------------------------------
//
// A Timer3 interrupt indicates an SMBus SCL low timeout.
// The SMBus is disabled and re-enabled here
//
void Timer3_ISR (void) interrupt 14
{
   SMB0CF &= ~0x80;                    // Disable SMBus
   SMB0CF |= 0x80;                     // Re-enable SMBus
   TMR3CN &= ~0x80;                    // Clear Timer3 interrupt-pending flag
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------