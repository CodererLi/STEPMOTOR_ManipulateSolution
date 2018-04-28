//-----------------------------------------------------------------------------
// F33x_SMBus_Slave.c
//-----------------------------------------------------------------------------
// Program Description:
//
//  software to demonstrate the C8051F33x SMBus interface in Slave mode
// - Interrupt-driven SMBus implementation
// - Only slave states defined
// - 1-byte SMBus data holder used for both transmit and receive
// - Timer1 used as SMBus clock rate (used only for free timeout detection)
// - Timer3 used by SMBus for SCL low timeout detection
// - ARBLOST support included					    
// - Pinout:
//    P0.0 -> SDA (SMBus)												
//    P0.1 -> SCL (SMBus)
//    P0.2 ->pulse of motor 1 and 2
//	  P0.3 ->pulse of motor 3 and 4
//	  P0.4 ->direction of motor 1 and 2
//	  P0.5 ->direction of motor 3 and 4
//
//    P2.0 -> C2D (debug interface)
//
//    all other port pins unused
//

// 
//
// FID:            33X000010
// Target:         C8051F330
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
// Command Line:   None
//
// Release 1.0
//    -Initial Revision (TP)
//    -8 MAR 2009
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <C8051F330.h>                 // SFR declarations


//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
						    
#define  SYSCLK         24500000       // System clock frequency in Hz

#define  SMB_FREQUENCY  10000          // Target SMBus frequency
                                       // This example supports between 10kHz
                                       // and 100kHz

#define  WRITE          0x00           // SMBus WRITE command
#define  READ           0x01           // SMBus READ command

#define  SLAVE_RUN_ADDR     0xF2           // Device addresses (7 bits,
                                       // lsb is a don't care)
//#define  TRUE           1

//-----------------------------------------------------------------------------
// Status vector - top 4 bits only
//-----------------------------------------------------------------------------
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
// 16-bit SFR declarations
//-----------------------------------------------------------------------------
sfr16	TMR3RL	=  0x92;			// Timer3 reload registers
sfr16	TMR3    =  0x94;			// Timer3 counter registers
sfr16	TMR2RL	=  0xCA;			// Timer2 Reload Register
sfr16	TMR2 	=  0xCC;			// Timer2 Register

sbit	LED		=  P2^0;			// LED on port P2.0
sbit	PUL1	=  P0^2;			//P0.2 ->pulse of motor 1 and 2
sbit	PUL2	=  P0^3;			//P0.3 ->pulse of motor 3 and 4
sbit	DIR1	=  P0^4;			//P0.4 ->direction of motor 1 and 2  
sbit	DIR2	=  P0^5;			//P0.5 ->direction of motor 3 and 4

//end 16-bit SFR declarations


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
unsigned short int StepNumber = 0;

bit DATA_READY = 0;                    // Set to '1' by the SMBus ISR
                                       // when a new data byte has been
                                       // received.
unsigned short int count = 0;
unsigned int code SPEED[400]=
{0x6892,0xcb87,0xe003,0xe8dc,0xedd1,0xf0f8,0xf32a,0xf4c4,0xf5ff,0xf6f8,0xf7c1,0xf867,0xf8f3,0xf96a,0xf9d1,0xfa2a,
0xfa78,0xfabd,0xfafb,0xfb32,0xfb64,0xfb91,0xfbbb,0xfbe0,0xfc03,0xfc23,0xfc40,0xfc5b,0xfc74,0xfc8c,0xfca2,0xfcb6,
0xfcc9,0xfcdb,0xfcec,0xfcfc,0xfd0b,0xfd19,0xfd27,0xfd34,0xfd40,0xfd4b,0xfd56,0xfd60,0xfd6a,0xfd74,0xfd7d,0xfd86,
0xfd8e,0xfd96,0xfd9d,0xfda4,0xfdab,0xfdb2,0xfdb8,0xfdbf,0xfdc5,0xfdca,0xfdd0,0xfdd5,0xfdda,0xfddf,0xfde4,0xfde8,
0xfded,0xfdf1,0xfdf5,0xfdf9,0xfdfd,0xfe01,0xfe04,0xfe08,0xfe0b,0xfe0f,0xfe12,0xfe15,0xfe18,0xfe1b,0xfe1e,0xfe20,
0xfe23,0xfe26,0xfe28,0xfe2b,0xfe2d,0xfe2f,0xfe31,0xfe34,0xfe36,0xfe38,0xfe3a,0xfe3c,0xfe3e,0xfe40,0xfe41,0xfe43,
0xfe45,0xfe47,0xfe48,0xfe4a,0xfe4b,0xfe4d,0xfe4e,0xfe50,0xfe51,0xfe53,0xfe54,0xfe55,0xfe56,0xfe58,0xfe59,0xfe5a,
0xfe5b,0xfe5c,0xfe5d,0xfe5f,0xfe60,0xfe61,0xfe62,0xfe63,0xfe64,0xfe65,0xfe65,0xfe66,0xfe67,0xfe68,0xfe69,0xfe6a,
0xfe6b,0xfe6b,0xfe6c,0xfe6d,0xfe6e,0xfe6e,0xfe6f,0xfe70,0xfe70,0xfe71,0xfe72,0xfe72,0xfe73,0xfe74,0xfe74,0xfe75,
0xfe75,0xfe76,0xfe76,0xfe77,0xfe78,0xfe78,0xfe79,0xfe79,0xfe7a,0xfe7a,0xfe7a,0xfe7b,0xfe7b,0xfe7c,0xfe7c,0xfe7d,
0xfe7d,0xfe7d,0xfe7e,0xfe7e,0xfe7f,0xfe7f,0xfe7f,0xfe80,0xfe80,0xfe80,0xfe81,0xfe81,0xfe81,0xfe82,0xfe82,0xfe82,
0xfe83,0xfe83,0xfe83,0xfe83,0xfe84,0xfe84,0xfe84,0xfe85,0xfe85,0xfe85,0xfe85,0xfe86,0xfe86,0xfe86,0xfe86,0xfe86,
0xfe87,0xfe87,0xfe87,0xfe87,0xfe88,0xfe88,0xfe88,0xfe88,0xfe88,0xfe89,0xfe89,0xfe89,0xfe89,0xfe89,0xfe89,0xfe8a,
0xfe8a,0xfe8a,0xfe8a,0xfe8a,0xfe8a,0xfe8a,0xfe8b,0xfe8b,0xfe8b,0xfe8b,0xfe8b,0xfe8b,0xfe8b,0xfe8c,0xfe8c,0xfe8c,
0xfe8c,0xfe8c,0xfe8c,0xfe8c,0xfe8c,0xfe8c,0xfe8d,0xfe8d,0xfe8d,0xfe8d,0xfe8d,0xfe8d,0xfe8d,0xfe8d,0xfe8d,0xfe8d,
0xfe8e,0xfe8e,0xfe8e,0xfe8e,0xfe8e,0xfe8e,0xfe8e,0xfe8e,0xfe8e,0xfe8e,0xfe8e,0xfe8e,0xfe8f,0xfe8f,0xfe8f,0xfe8f,
0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe8f,0xfe90,0xfe90,0xfe90,
0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,0xfe90,
0xfe90,0xfe90,0xfe90,0xfe90,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,
0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,
0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe91,0xfe92,
0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,
0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,
0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,
0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92,0xfe92};


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

   LED = 1;

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
      TurnDirection = (StepCommand & 0x8000)>>15;
      StepNumber = (StepCommand & 0x7FFF);
      LED = ~LED;
      TR2 = 1;
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
   TMR2RL = SPEED[300];
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
// P1.3   digital   push-pull     LED
//
// all other port pins unused
//
void PORT_Init (void)
{
   P0MDOUT = 0x3C;                     // All P0 pins open-drain output

   P2MDOUT |= 0x01;                    // Make the LED (P2.0) a push-pull
                                       // output

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

            if((SMB0DAT & 0xFE) == (SLAVE_RUN_ADDR & 0xFE)) // Decode address
            {                          // If the received address matches,
               ACK = 1;                // ACK the received slave address

               if((SMB0DAT & 0x01) == READ) // If the transfer is a master READ,
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
         } else
         {  
            DIR1 = 0;
            DIR2 = 0;
         }
				 PUL1 = ~PUL1;
         PUL2 = ~PUL2;

         count++;
         
      }else
      {
         count = 0;
         TR2 = 0;
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