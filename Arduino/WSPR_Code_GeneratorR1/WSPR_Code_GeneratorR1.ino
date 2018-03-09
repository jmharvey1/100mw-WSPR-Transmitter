/*
 coding based on material found at:
 http://www.g4jnt.com/wspr_coding_process.pdf
 http://physics.princeton.edu/pulsar/K1JT/WSPR_Instructions.TXT
K1JT FN20 30
 */
/*
 * To work using an UNO, as the Arduino part of the project.
 * There were couple of changes needed to make it go.
 * The first thing you'll need to do, is find these two lines in the sketch.
 *   int S[162];
 *   int D[162];
 * and change their variable types from "int" to "byte"
 * This will "free up" some SRAM and will allow the UNO run stably.
 * Next, make sure that the encoder switch's "CLK" pin is connected to your UNO's digital pin 2. Not digital pin 1 as I have it here with the Micro. Pin 2 on the UNO is the interrupt pin, where as on the Micro pin 1 is used.
 * Next, find these lines in the sketch:
 *         const int PinCLK=1;  // Used for generating interrupts using CLK signal
 *         const int PinDT=14;  // Used for reading DT signal
 *         const int PinSW=16;  // Used for the push button switch
 * and make the following respective changes:
 *         const int PinCLK=2; 
 *         const int PinDT=3; 
 *         const int PinSW=4;
 * The UNO doesn't support digital pins 14 & 16, so you will need to move these switch connections to UNO pins 3 & 4.
 * If for some reason you'd rather use pins other than 3 & 4, it should be no problem, just be sure to change the "PinDT" & "PinSW" variables to match your wiring.    
*
*/


 #include <U8glib.h>
#if defined(__AVR_ATmega32U4__)
  //Code in here will only be compiled if an Arduino Leonardo is used.
  //#define InterruptId 1 // SWR Bridge (Leonardo)
  #define InterruptId 3  // AD9850 / SS Micro w 128x64 OLED display VFO 
#endif
//#if defined(__AVR_ATmega16U4__)
//  //Code in here will only be compiled if an Arduino Uno is used.
//  #define InterruptId 0
//#endif

#if defined(__AVR_ATmega328P__)
  //Code in here will only be compiled if an Arduino Uno is used.
  #define InterruptId 0
#endif

/* // Original SWR bridge Leonardo Config 
 #define W_CLK 8       // Pin 8 - connect to AD9850 module word load clock pin (CLK)
 #define FQ_UD 9       // Pin 9 - connect to freq update pin (FQ)
 #define DATA 10       // Pin 10 - connect to serial data load pin (DATA)
 #define RESET 11      // Pin 11 - connect to reset pin (RST).
*/ 
// Pin Assignment for SS Micro to DDS AD9850 mapping
// Original SWR bridge Leonardo Config 
 #define W_CLK 10       // Pin 8 - connect to AD9850 module word load clock pin (CLK)
 #define FQ_UD 0       // Pin 9 - connect to freq update pin (FQ)
 #define DATA 6        // Pin 10 - connect to serial data load pin (DATA)
 #define RESET 11      // Pin 11 - connect to reset pin (RST).
 #define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }
 //OLED Display Configuration Options
//U8GLIB_SH1106_128X64 u8g(3, 4, 7, 6 , 5); // SW SPI Com: CLK = 3, MOSI = 4, CS = 7, dc = 6, RES = 5
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST); // Fast I2C / TWI 
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);  // Display which does not send AC


/// Station Info & setup /////
char *MySta = "KW4KD " ;//" K1JT ";//"KW4KD " ;//Needs to be six characters w/ 3rd character bieng the number in the callsign; Use Space character to pad/froce the callsign to meet these reuirements
char *GridId = "EM75" ;//"FN20";//"EM75" ;//"EM75jc";
int PwrLvl = 20; //30; //10; // 0 = 1mw ; 10 = 10 mw; 20 = 100mw; 30=1Watt; & 37= 5Watts 17dbm= 60mw
float Cf = 7.040100e6; //Center of the 40M band allocation this value is over written later in the code, When the user operates the encoder SW
float OffsetFrq = +65.0; //This can range from -100.0 to +100; Shifts the actual transmitted signal by th amount specified here  
int sndwaitintrvl = 5 ; // sndwaitintrvl = 0 means begin new automated transmission every two minutes; 1 means start a new transmision every 4 minutes, & 5 means start a new transmision every 12 minutes

//DDS AD9850 Correction
double WWV0Freq = 9999902.0; //For initial Setup & calibration set WWV0Freq = 100000000.0; Place the unit in "Cal" mode, & "Zero Beat" With WWV (10Mhz). 
//                             Note: The encoder switch will change the DDS freq, while in "Cal" mode.  
//                             When found, set variable "WWV0Freq" equal to the display "Freq:" reading  
//Arduino Clk Frequency correction; Number of miliseconds per Xmit interval (2 minutes) that need to be added or subtracted to keep the time in sync
double msCorrection = 248; 
double xtalfctr;
long NxtBitTime;
int OptionCnt = 5; //Total test option count; Note Last option is to DDS Calibration.
//double Start_Freq; 
bool debug = false;

char buf1 [32]; //OLED Display support
char buf2 [32]; //OLED Display support
char buf3 [32]; //OLED Display support
char buf4 [32]; //OLED Display support
char buf5 [32]; //OLED Display support
int skipcnt =0;
char StaInfo[32]; //OLED Display support

byte buf[4];
//// Rotory Sw Global Variables ////

//float Cf = 10.140200e6; //Center of the 30M band allocation
//Next Symbol wait interval;  682.7ms or 0.6827 seconds is the Ideal interval (assuming no other delays in the send loop)
int BitPeriod = 587;// This value Gave best results for AD9850, 128x84 OLED I2C display w/ SSD1306 driver, and SS Micro. 
//int BitPeriod = 606;// This value Gave best results for AD9850, 128x84 OLED SPY display w/ SH1106 driver, and Leonardo.  

unsigned long StrtSnd ; //used as a marker to measure the actual duration of the transmission
unsigned long nextSnd = 0; // the calculated value to compare to the current MS() returned value to start a nother automated transmit cycle
bool AutoSnd = false; 
float SndTime = 110.6; // Initial value shown here is for reference only; i.e, ideal transmit time 110.6 Seconds
bool TurnDetected;
bool SwRead = false;
bool SndWSPR = false;
//bool ClkLow = false;
int IntCnt = 0;
int IncDecVal = 1;
int virtualPosition =1;
unsigned long WaitInterval = micros();
const int PinCLK=1;  // Used for generating interrupts using CLK signal
const int PinDT=14;  // Used for reading DT signal
const int PinSW=16;  // Used for the push button switch
//// End of Rotory Sw Global Variables ////


///******** Begin Rotory Sw Interrupt Service Routine *******///

void isr()  { 
   unsigned long ThisIntTime = micros();
   if (ThisIntTime < WaitInterval) return;
   WaitInterval = ThisIntTime+80000; //move the next valid interrupt out by 3 milliSeconds
   int cnt =0;
   int DtHIcnt =0;
   int ClkHIcnt =0;
   bool ClkLow = true;
   bool DtLow = true;
   detachInterrupt(InterruptId);
   int SmplCnt = 30;
   while(cnt <SmplCnt*2){
    //if(analogRead(PinDT)>= 500) DtHIcnt++;//digitalRead
    if(digitalRead(PinDT)== HIGH) DtHIcnt++;//
    if (digitalRead(PinCLK)== HIGH) ClkHIcnt++;
    cnt++;
    
   }
   if(DtHIcnt>SmplCnt+1) DtLow = !DtLow;
   if(ClkHIcnt>SmplCnt+1) ClkLow =!ClkLow;
   if( ClkLow == DtLow) IncDecVal = -1;
   else IncDecVal = +1;
//   if (ClkLow) {
//    Serial.print ("cL/");
//   }
//   else Serial.print ("cH/");
//   if (DtLow) Serial.print (" dL; ");
//   else Serial.print (" dH; "); 
//  if (IncDecVal>0) Serial.println("+ ");
//  else  Serial.println("- ");
   if(SndWSPR && virtualPosition == OptionCnt){
    
     if(IncDecVal == +1) Cf = Cf +1.0;
     if(IncDecVal == -1) Cf = Cf -1.0;
     sendFrequency(Cf);
     attachInterrupt (InterruptId,isr,CHANGE);
     //TurnDetected = true;
     return;
   }
   virtualPosition = virtualPosition+ IncDecVal;
    if( IncDecVal > 0 && virtualPosition >= OptionCnt+1 ) virtualPosition = 1;
    if( IncDecVal < 0 && virtualPosition <=0 ) virtualPosition = OptionCnt;
//    sprintf (buf,"Count = %d", virtualPosition);
//    Serial.println ("**");

   attachInterrupt (InterruptId,isr,CHANGE);
   TurnDetected = true;
} // Interrupt service routine End

///******** End of Rotory Sw Interrupt Service Routine *******///


 // transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data)
{
  for (int i=0; i<8; i++, data>>=1) {
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
  }
}

 // frequency calc from datasheet page 8 = <sys clock> * <frequency tuning word>/2^32
void sendFrequency(double frequency) {
  frequency = frequency*(WWV0Freq/10.0e6);//double WWV0Freq = 9999875.0;
  int32_t freq = frequency * 4294967295/125000000;  // note 125 MHz clock on 9850
  for (int b=0; b<4; b++, freq>>=8) {
    tfr_byte(freq & 0xFF);
  }
  tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
  pulseHigh(FQ_UD);  // Done!  Should see output
}

void setup() {
  //Initialize serial and wait for port to open:
  if (debug){
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for Leonardo only
    }
  }

// setup encoder
   pinMode(PinCLK,INPUT);
   pinMode(PinDT,INPUT);  
   pinMode(PinSW,INPUT);
   attachInterrupt (InterruptId,isr,CHANGE);   // Use interrupt 0 [Uno] or 1 [Leonardo]; It is always connected to pin 2 on Arduino UNO
   TurnDetected = false;
   SndWSPR = false;
   virtualPosition = 1;
   SwRead = false;
// end encoder setup
  // prints title with ending line break
  if (debug) Serial.println("WSPR XMT ENCODER By: KW4KD - April 2017");

  // configure arduino data pins for output
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);

  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);  // this pulse enables serial mode - Datasheet page 12 figure 10
  sendFrequency(1.0);  // set AD9850 output to 1 Hz
  xtalfctr =(120000+msCorrection)/120000; //msCorrection = 240; 
  BitPeriod = int(BitPeriod*xtalfctr);
  sprintf (StaInfo,"%s %s %dDbm", MySta, GridId, PwrLvl);
  StrtSnd =millis();
  Splash_Screen(1);
}


void loop() {
  unsigned long N=0;
  
  for(int i=0; i<6; i++){
    byte thisByte = MySta[i];
//    if (debug) PrintByteMap(thisByte);
    int MapVal = mapASC2WSPR(thisByte);
/*
      N1 = [Ch 1] The first character can take on any of the 37 values including [sp],
      N2 = N1 * 36 + [Ch 2] but the second character cannot then be a space so can have 36 values
      N3 = N2 * 10 + [Ch 3] The third character must always be a number, so only 10 values are possible.
      N4 = 27 * N3 + [Ch 4] – 10]
      N5 = 27 * N4 + [Ch 5] – 10] Characters at the end cannot be numbers,
      N6 = 27 * N5 + [Ch 6] – 10] so only 27 values are possible.
*/
    switch (i) {
      case 0:
        N =  MapVal;
        break;
      case 1:
        N = (N * 36) + MapVal;
        break;    
      case 2:
        N = (N * 10) + MapVal;
        break;    
      case 3:
        N = (N * 27) + MapVal-10;
        break;    
      case 4:
        N = (N * 27) + MapVal-10;
        break;    
      case 5:
        N = (N * 27) + MapVal-10;
        break;    
    }
   if (debug){
    Serial.print("Callsign Code: ");
    Serial.println(N);
   }
    
  }

  /*
   * Locator
      Designating the four locator characters as [Loc 1] to [Loc 4], the first two can each take
      on the 18 values ‘A’ to ‘R’ only so are allocated numbers from 0 – 17. The second pair
      can take only the values 0 – 9.
      Another integer M is formed from:
      M1 = (179 - 10 * [Loc 1] - [Loc 3] ) * 180 + 10 * [Loc 2] + [Loc 4]
      Which gives a range of values from ‘AA00’ = 32220 to ‘RR99’ = 179, 
   */
   unsigned long M=0;
   int LocVal[4];
  for(int i=0; i<4; i++){
    byte thisByte = GridId[i];
//    if (debug) PrintByteMap(thisByte);
    LocVal[i] = mapASC2WSPR(thisByte);
    if(i<2) LocVal[i] = LocVal[i] -10;
  }
  M = (179 - 10 * LocVal[0] - LocVal[2] ) * 180 + 10 * LocVal[1] + LocVal[3];
   if (debug){
    Serial.print("Grid Code: ");
    Serial.println(M);
   } 

 /*
  * Power level, [Pwr] is taken as a value from 0 – 60. Although only certain values will work
    with the WSJT / WSPR software (just those ending in 0, 3 or 7) 
    M = M1 * 128 + [Pwr] + 64 Which gives a final range of values for M up to a maximum
    of 4124287, and fits into 22 bits (222 = 4194304) 
  */
   M = (M * 128) + PwrLvl + 64;
   if (debug){
   Serial.print("Grid Code + Power: ");
   Serial.println(M); 
   Serial.print(highByte(M),HEX);
   Serial.println(lowByte(M),HEX);
   }
   /*   
    *   Bit Packing 
    *   The two integers N and M are truncated and combined so the 50 bits sit end-to-end as
        callsign-locator-power. These are placed into an array of eleven 8-bit bytes c[0] to c[10],
        so the first element c[0] contains the most significant 8 bits part of the callsign, c[1] the
        next 8 bits and so on. (Note that c[3] contains both the 4 LSBs of the callsign and 4
        MSBs of the locator, and that c[6] contains just the two LSBs of M occupying the most
        significant bit positions. The lowest six bits of c[6] are set to 0, with the remaining bytearray
        elements [c7] to c[10] set to zero. Only the left-most 81 of these 88 total bits are
        subsequently used. 

    */
   byte ByteBufN[4];
   byte ByteBufM[4];
   byte c[10];
   Int2ByteArray(N);
   memcpy( ByteBufN, buf, 4 );
   Int2ByteArray(M); //map the 4 bytes of this 32 bit integer to a array called "buf"
   memcpy( ByteBufM, buf, 4 );
//   if (debug){
//     Serial.print("N: ");
//     for(int i=0; i<4; i++){
//       Serial.print(ByteBufN[i],HEX);
//       Serial.print(" ");
//     }
//     Print32BitBinary (N);
//     Serial.println("");
//     Serial.print("M: ");
//     for(int i=0; i<4; i++){
//       Serial.print(ByteBufM[i],HEX);
//       Serial.print(" ");
//     }
//     Print32BitBinary (M);
//     Serial.println("");
//   }
   
   for(int i=0; i<3; i++){
    c[i] = ((ByteBufN[i]<<4)&240)+ (((ByteBufN[i+1])&240)>>4);
   }
   c[3] =  ((ByteBufN[3]<<4)&240);
   c[3] =  c[3]+ (((ByteBufM[1]&63)>>2)&15);
   c[4] = (((ByteBufM[1]&15)<<6) &192)+(((ByteBufM[2]&252)>>2 &63));
   c[5] = (((ByteBufM[2]&15)<<6) &192)+(((ByteBufM[3]&252)>>2 &63));
   c[6] = (((ByteBufM[3]&15)<<6) &192);
  //zero out/initialize "c" register
   for(int i=7; i<10; i++){
    c[i] = 0;
   }
    //debug = true;//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX 
//    if (debug){  
//     Serial.print("C[HEX]: ");
//     for(int i=0; i<10; i++){
//       Serial.print(c[i],HEX);
//       Serial.print(" ");
//     }
//    Serial.println("");
//   
//    //   for(int i=0; i<4; i++){
//    //     PrintBinary(ByteBufN[i]);//Serial.print(ByteBufN[i],BIN);
//    //     //Serial.print(" ");
//    //   }
//    //   Serial.println("");
//    //
//    //    for(int i=0; i<4; i++){
//    //     PrintBinary(ByteBufM[i]);//Serial.print(ByteBufM[i],BIN);
//    //     //Serial.print(" ");
//    //   }
//    //   Serial.println("");
//   
//     Serial.print("C: ");
//      for(int i=0; i<10; i++){
//       PrintBinary(c[i]);//Serial.print(c[i],BIN);
//       //Serial.print(" ");
//     }
//     Serial.println("");
//   }
   //debug = false;//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
/*   
 *    Convolutional Encoding
      The data is now expanded to add FEC with a rate ½, constraint length 32, convolutional
      encoder.
      The 81 bits (including the 31 trailing zeros) are read out MSB first in the order:
      c[0] MSB… c[0] LSB., c[1] MSB…c[1] LSB ………… c[11]
      (or adopting the alternative view, one-at-a-time from the left hand end of the string)
      The bits are clocked simultaneously into the right hand side, or least significant position,
      of two 32 bit shift registers [Reg 0] and [Reg 1]. Each shift register feeds an ExclusiveOR
      parity generator from feedback taps described respectively by the 32 bit values
      0xF2D05351 and 0xE4613C47. Parity generation starts immediately the first bit
      appears in the registers (which must be initially cleared) and continues until the registers
      are flushed by the final 31st zero being clocked into them.
      Each of the 81 bits shifted in generates a parity bit from each of the generators , a total of
      162 bits in all. For each bit shifted in, the resulting two parity bits are taken in turn, in the
      order the two feedback tap positions values are given, to give a stream of 162 output
      bits.
 *    
 */
  byte S[162];
  byte D[162];
  int strmptr =0;
 // setup and initialize shift 32 bit registers
 byte ShftReg1[4];
 byte ShftReg2[4];
  for(int i=0; i<4; i++){
   ShftReg1[i] = 0;
 }
 for(int i=0; i<4; i++){
   ShftReg2[i] = 0;
 }
 // now start encoding process by shifting the 81 data bits out starting with the MS bit of the satation Id
 for(int BitShftCnt=0; BitShftCnt<81; BitShftCnt++){
  if (debug){
    Serial.print("BitShftCnt: ");
    Serial.println(BitShftCnt);
  }
  int outbit =1;
  int inbit =0;
  int DataBit = ((c[0]&128)>>7); //get current MS bit

  //move remaing bits one position left
  for(int cNdx=9; cNdx>-1; cNdx--){
    outbit = ((c[cNdx]&128)>>7);
    c[cNdx] = (((c[cNdx]<<1)&254)+ inbit);
    inbit = outbit;
  }
  inbit =DataBit;

  //shift new databit into the two 32 bit shift registers
  for(int Reg1Ndx=3; Reg1Ndx>-1; Reg1Ndx--){
    outbit = ((ShftReg1[Reg1Ndx]&128)>>7);
    ShftReg1[Reg1Ndx] = (((ShftReg1[Reg1Ndx]<<1)&254)+ inbit);
    inbit = outbit;
  }
  inbit =DataBit;
  for(int Reg2Ndx=3; Reg2Ndx>-1; Reg2Ndx--){
    outbit = ((ShftReg2[Reg2Ndx]&128)>>7);
    ShftReg2[Reg2Ndx] = (((ShftReg2[Reg2Ndx]<<1)&254)+ inbit);
    inbit = outbit;
  }

  // Now perform xor parity check/gen
 /*
  *  The parity generation process is :
     Shift the next source bit into the LSB of both [Reg 0] and [Reg 1],
     moving the existing data in each one place left
       Take the contents of [Reg 0]
    AND with 0xF2D05351
    Calculate the single bit parity (XOR) of the resulting sum.
    Append to the output data stream
    
    Take the contents of [Reg 1]
    AND with 0xE4613C47
    Calculate the single bit parity (XOR) of the resulting sum.
    Append to the output data stream 
  * 
  */
  unsigned int Reg1WrdL = (unsigned int)ShftReg1[3] + 256*((unsigned int)ShftReg1[2]);
  unsigned int Reg1WrdH = (unsigned int)ShftReg1[1] + 256*((unsigned int)ShftReg1[0]);
  unsigned int Reg2WrdL = (unsigned int)ShftReg2[3] + 256*((unsigned int)ShftReg2[2]);
  unsigned int Reg2WrdH = (unsigned int)ShftReg2[1] + 256*((unsigned int)ShftReg2[0]);
  unsigned int Reg1ParityMaskH = 0xF2D0;
  unsigned int Reg1ParityMaskL = 0x5351;
  unsigned int Reg2ParityMaskH = 0xE461;
  unsigned int Reg2ParityMaskL = 0x3C47;
  unsigned int Reg1AndL = Reg1WrdL & Reg1ParityMaskL;
  unsigned int Reg1AndH = Reg1WrdH & Reg1ParityMaskH;
  unsigned int Reg2AndL= (Reg2WrdL & Reg2ParityMaskL);
  unsigned int Reg2AndH= (Reg2WrdH & Reg2ParityMaskH);
  //unsigned int Reg1And = Reg1AndL + 512*(Reg1AndH);
  //unsigned int Reg2And = Reg2AndL + 512*(Reg2AndH); 
  if (debug){
   Serial.println("");
   Serial.print(" ShftReg1: ");
   for(int i=0; i<4; i++){
    PrintBinary(ShftReg1[i]);//Serial.print(ShftReg1[i],BIN);
   }
   Serial.println("");

    Serial.print(" Reg1MASK: ");
    PrintBinary(((Reg1ParityMaskH>>8)&255));
    PrintBinary(((Reg1ParityMaskH>>0)&255));
    PrintBinary(((Reg1ParityMaskL>>8)&255));
    PrintBinary(((Reg1ParityMaskL>>0)&255));
    Serial.println("");
  
    Serial.print(" Reg1And:  ");
    PrintBinary(((Reg1AndH>>8)&255));
    PrintBinary(((Reg1AndH>>0)&255));
    PrintBinary(((Reg1AndL>>8)&255));
    PrintBinary(((Reg1AndL>>0)&255));
  }
  // Start First Parity Bit calc
  int ParityBit = 0;//DataBit;
  ParityBit = CalcXORParity(ParityBit, Reg1AndL);
  ParityBit = CalcXORParity(ParityBit, Reg1AndH);
  S[strmptr] = ParityBit;
  strmptr += 1;
//  if (debug){
//    Serial.print(" Parity: ");
//    Serial.print(ParityBit);
//    Serial.println("");
//    Serial.println("");
//
//    Serial.print(" ShftReg2: ");
//     for(int i=0; i<4; i++){
//       PrintBinary(ShftReg2[i]);//Serial.print(ShftReg2[i],BIN);
//       //Serial.print(" ");
//     }
//     Serial.println("");
//  
//    Serial.print(" Reg2MASK: ");
//    PrintBinary(((Reg2ParityMaskH>>8)&255));
//    PrintBinary(((Reg2ParityMaskH>>0)&255));
//    PrintBinary(((Reg2ParityMaskL>>8)&255));
//    PrintBinary(((Reg2ParityMaskL>>0)&255));
//    Serial.println("");
//
//
//    Serial.print(" Reg2And:  ");
//    PrintBinary(((Reg2AndH>>8)&255));
//    PrintBinary(((Reg2AndH>>0)&255));
//    PrintBinary(((Reg2AndL>>8)&255));
//    PrintBinary(((Reg2AndL>>0)&255));
//  }

  
  // Start 2nd Parity Bit calc
  ParityBit = 0;
  ParityBit = CalcXORParity(ParityBit, Reg2AndL);
  ParityBit = CalcXORParity(ParityBit, Reg2AndH);
  S[strmptr] = ParityBit;
  strmptr += 1;
  if (debug){
    Serial.print(" Parity: ");
    Serial.print(ParityBit);
    Serial.println("");
    Serial.println("");
  }
  // End of parity bit calc
 }
  //debug = true;//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 if (debug){
  Serial.print("strmptr: ");
  Serial.println(strmptr);
 }

/*
 * Initialise a counter, P to zero
  Take each 8-bit address from 0 to 255, referred to here as I
  Bit-reverse I to give a value J.
  For example, I = 1 gives J = 128, I = 13 J = 176 etc.
  If the resulting bit-reversed J yields a value less than 162 then :
  Set Destination bit D[J] = source bit S[P]
  Increment P
  Stop when P = 162 
 * 
 */
 int I =0;
 unsigned int J =0;
 for(int i=0; i<162; i++){// Clear destination Array
  D[i] = 0; 
 }
 
 for(int P=0; P<162; P++){
  J =255;
  while (J>=162){
    J=BitFlip(I);
   I +=1; 
  }
//  if (debug){
//    Serial.print("P: ");
//    Serial.print(P);
//    Serial.print(" J: ");
//    Serial.println(J);
//  }
  D[J] = S[P];
//  Serial.print(" D[J]: ");
//  Serial.println(D[J]);
 }
 /*
  * Merge With Sync Vector:
    The 162 bits of data are now merged with 162 bits of a pseudo random synchronisation
    word having good auto-correlation properties. Each source bit is combined with a sync
    bit taken in turn from the table below to give a four-state symbol value:
     Symbol[n] = Sync[n] + 2 * Data[n] 
  * 
  */

 int sync[] ={1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,
              0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,
              1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,
              1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0 
 };
 for(int i=0; i<162; i++){
   D[i] = sync[i] + 2 * D[i] ;
   
//   Serial.print("Cnt: ");
//   Serial.print(i);
//   Serial.print(" Tone: ");
//   Serial.print(D[i]);
//   Serial.print(" sync[]: ");
//   Serial.println(sync[i]);
 }
 /*
  * Modulation
    Each symbol represents a frequency shift of 12000 / 8192, or approximately 1.46Hz, per
    Symbol Value giving four-level Multi-FSK modulation. See table:
              0 = Cf -2.197
              1 = Cf -0.732
              2 = Cf +0.732
              3 = Cf +2.197
    The transmitted symbol length is the reciprocal of the tone spacing, 
    or approximately 0.683 seconds, 
    so the complete message of 162 symbols takes around 110.6 seconds to send 
    and occupies a bandwidth of approximately 6Hz, 
  * 
  */
// At this point we are in a loop waiting for USER input via the rotory encoder/push button
  while(!SndWSPR){
    
   if(AutoSnd){
     float Time2go =(float)((nextSnd - millis())/1000);
     if(Time2go <=0.0) Time2go=0.0;
     int MinCnt = (int) (Time2go/60);
     int SecCnt = (Time2go)-(MinCnt*60);
     sprintf (buf1,"Nxt Tx: %02d:%02d", MinCnt, SecCnt);
     }
   Splash_Screen(virtualPosition);
   ReadPushButton();
     
    if (TurnDetected){
      TurnDetected = !TurnDetected;
      loop;
    }
    if(AutoSnd && millis()>nextSnd) SndWSPR = true;
    if(SndWSPR){  //send code; float Cf = 10.140100e6; int BitPeriod = 682; ms or 0.6827 seconds
      float TxFreq;
      StrtSnd =millis();
      if(!AutoSnd){
       nextSnd = StrtSnd;
       AutoSnd =true;
      }
      nextSnd = nextSnd+((1+sndwaitintrvl)*(120000+msCorrection)); //double msCorrection = 240; the calculated value to compare to the current MS() returned value to start another automated transmit cycle
      for(int i=0; i<162; i++){
       int ToneCd =D[i] ;
     
       switch (ToneCd) {
         case 0: // 
          TxFreq = Cf -2.197;
          break;
    
         case 1: // 
          TxFreq  = Cf -0.732;
          break;
    
          case 2: // 
          TxFreq  = Cf +0.732;
          break;
    
         case 3: // 
          TxFreq = Cf+2.197;
          break;
       }
       if(virtualPosition != OptionCnt) TxFreq = TxFreq+OffsetFrq;
       else TxFreq = Cf;
       sendFrequency(TxFreq);  // set AD9850 output to New TX freq
       char str_Step[12];
       dtostrf(TxFreq/1e6, 9, 6, str_Step);
       sprintf (buf1,"Symbl: %d",(ToneCd));
       sprintf (buf2,"Count: %d",(162- i));  
       sprintf (buf3,"%s Mhz", str_Step);
       ShowInfo( buf4, buf2, buf1, buf3, StaInfo);
       NxtBitTime = StrtSnd +(long)((i+1)*682.7*xtalfctr);
       while(NxtBitTime > millis());
      } //end "FOR LOOP"
      SndWSPR = false;
      sendFrequency(1.0);  // set AD9850 output back to 1 Hz
      SndTime = ((float)((millis())-StrtSnd))/(xtalfctr*1000.0);
      Splash_Screen(virtualPosition);
    } //end of if(SndWSPR)
    else{
      
      loop;
    }
  continue;
 }//End while(!SndWSPR) loop
}// End "MAIN LOOP"

/*###############################################################################################################################*/

//void PrintByteMap(byte TstByte){
//  int MapVal = mapASC2WSPR(TstByte);
//  // prints value unaltered, i.e. the raw binary version of the
//  Serial.write(TstByte);
//  Serial.print(", dec: ");
//  Serial.print(TstByte);
//  if( MapVal >-1 ){
//      Serial.print(", WSPR: ");
//      Serial.println(MapVal);
//  }
//  else Serial.println(", This Character is NOT Supported");
//}

int mapASC2WSPR(byte ASCchr){
  int WSPRVal = -1;
  if(ASCchr == 32){
    WSPRVal =36;
  }
  if(ASCchr >= 48 & ASCchr <= 57){
    WSPRVal = ASCchr -48;
  }
  if(ASCchr >= 65 & ASCchr <= 90){
    WSPRVal = ASCchr -55;
  }
  if(ASCchr >= 97 & ASCchr <= 122){
    WSPRVal = ASCchr -87;
  }
  return WSPRVal;
}
void Int2ByteArray(unsigned long IntVAL){ 
  //byte buf[4];  
  buf[3] = IntVAL & 255;
  buf[2] = (IntVAL >> 8)  & 255;
  buf[1] = (IntVAL >> 16) & 255;
  buf[0] = (IntVAL >> 24) & 255;
  //return buf;
}

void Print32BitBinary (unsigned long data){
  byte buf[4];
  buf[0] = data & 255;
  buf[1] = (data >> 8)  & 255;
  buf[2] = (data >> 16) & 255;
  buf[3] = (data >> 24) & 255;
  for(int i=3; i>-1; i--){
    char* BinMap[8];
    for(int BPtr =0; BPtr<8; BPtr++){
      if(((buf[i]>>BPtr)&1) == 1) BinMap[BPtr] = "1";
      else  BinMap[BPtr] = "0";
    }
    for(int BPtr =7; BPtr>-1; BPtr--){  
       Serial.print(BinMap[BPtr]);
    }
    Serial.print(" ");  
  }
  Serial.println("");
}

void PrintBinary(unsigned int data){ // print 8 bit binary (including leading zeros
  char* BinMap[8];
  for(int BPtr =0; BPtr<8; BPtr++){
    if(((data>>BPtr)&1) == 1) BinMap[BPtr] = "1";
    else  BinMap[BPtr] = "0";
  }
  for(int BPtr =7; BPtr>-1; BPtr--){  
     Serial.print(BinMap[BPtr]);
  }
  Serial.print(" ");
}

int BitFlip( int Data){
 byte X = (byte)Data;
 byte Y = 0;
 for(int BPtr =7; BPtr>-1; BPtr--){
  Y = ((Y<<1)&254);
  Y = Y+(X&1);
  X = (X>>1);
 }
 return (int)Y;
}

int CalcXORParity(int XORsum, unsigned int ParityWrd){
  //int XORsum = 0;
  for(int i=0; i<16; i++){
//    Serial.print(" ParityWrd: ");
//    Print32BitBinary (ParityWrd);
    XORsum = ((XORsum & 1) ^ (ParityWrd & 1));
    ParityWrd = ((ParityWrd>>1)&0xFFFF);
//    Serial.print(" XORsum: ");
//    Serial.println(XORsum);
  }
  return XORsum;
}

int NuRow(int oldrow){
   {
    int pixcnt = 13;
    oldrow += pixcnt;
    return oldrow; 
   }
}

// Splash Screen
void Splash_Screen(int option){
   char str_Band[12];
   char str_Step[10];
   char str_SndTime[10];
   float TxFreq;
   switch (option) {
     case 1: //Set Freq Limits for 30_MTRS 
      Cf = 10.140200e6; //Center of the 30M band allocation
      sprintf (str_Band, " 30"); 
      break;

     case 2: //Set Freq Limits for 40_MTRS 
      Cf = 7.040100e6;
      sprintf (str_Band, " 40"); 
      break;

     case 3: //Set Freq Limits for 80_MTRS 
      Cf = 3.594100e6;
      sprintf (str_Band, " 80"); 
      break;

    case 4: //Set Freq Limits for 160_MTRS 
      Cf = 1.836600e6;
      sprintf (str_Band, "160"); 
      break;  

    case 5: //Set Freq for 10_MMhz 
      Cf = 10.0000e6;
      sprintf (str_Band, "Cal 10Mhz ");
      break;  

      
   }
   if(virtualPosition != OptionCnt) TxFreq= Cf+OffsetFrq;
   else TxFreq= Cf;
   dtostrf((TxFreq)/1e6, 9, 6, str_Step);
   dtostrf(SndTime, 5,1, str_SndTime);
   //sprintf (buf5,"HELLO", MinCnt);//sprintf (buf5,"Nxt Tx: %02d:%02d", MinCnt, SecCnt);
   sprintf (buf3,"Frq:%sMhz", str_Step);
   sprintf (buf4,"  %sM WSPR TX", str_Band);
   sprintf (buf2,"Tx Period: %s S", str_SndTime);
   ShowInfo( buf4, buf3, buf2, buf1, "AD9850-128x64 OLED" );
   
//   u8g.firstPage();
//   do {
//    int row = 11; //first OLED row to print on
//    u8g.setFont(u8g_font_unifont);
//    u8g.setPrintPos(0, row);
//    u8g.print(buf4);
//    row = NuRow(row);
//    u8g.setPrintPos(0, row);
//    u8g.print(buf3);
//    u8g.setFont(u8g_font_7x13);
//    row = NuRow(row);
//    u8g.setPrintPos(0, row);
//    u8g.print(buf2);
//    row = NuRow(row);
//    row = NuRow(row);
//    u8g.setPrintPos(0, row);
//    u8g.print("AD9850-128x64 OLED");
//    //u8g.print(buf5);
//  } while( u8g.nextPage() );


}
 ////////////////////////////////////////////////////////////////////////////////////
void ReadPushButton(){   //Begin DeBounce & Read Rotory Push Button
  int cnt =0;
  //if(analogRead(PinSW)>= 700 && SwRead){
  if(digitalRead(PinSW)== HIGH && SwRead){
    
    while (digitalRead(PinSW)== HIGH && cnt < 5) {
      delay(20);
      //if(analogRead(PinSW)>= 700) cnt++;
      if(digitalRead(PinSW)== HIGH) cnt++;
      else cnt =0;
    }
    if (cnt == 5){
      SwRead = !SwRead;
      cnt = 0;
    }
  }
    
  //while (analogRead(PinSW)<= 200 && cnt < 5) {
  while (digitalRead(PinSW)== LOW && cnt < 5) {  
    delay(20);
    //if(analogRead(PinSW)<= 200) cnt++;
    if(digitalRead(PinSW)== LOW) cnt++;
    else cnt =0;
  }
  if (!SwRead && cnt == 5){
    SwRead = !SwRead;
    SndWSPR = true;  
//    virtualPosition=0;              // if cnt == 5, then reset counter to ZERO
//    Serial.print ("Reset = ");      // Using the word RESET instead of COUNT here to find out a buggy encoder
//    Serial.println (virtualPosition);
//    sprintf (buf,"Reset = %d", virtualPosition);
  }
  //End of DeBounce & Read Rotory Push Button code 

 }

void ShowInfo( char *line1, char *line2, char *line3, char *line4, char *line5){
 u8g.firstPage();
 do {
    int row = 11; //first OLED row to print on
    u8g.setFont(u8g_font_unifont);
    u8g.setPrintPos(0, row);
    u8g.print(line1); //buf4
     row = NuRow(row);
    u8g.setPrintPos(0, row);
    u8g.print(line2); //buf2
    row = NuRow(row);
    u8g.setPrintPos(0, row);
    u8g.print(line3); //buf1
    row = NuRow(row);
    u8g.setPrintPos(0, row);
    u8g.print(line4); //buf3
    u8g.setFont(u8g_font_7x13);
    row = NuRow(row);
    u8g.setPrintPos(0, row);
    u8g.print(line5); //u8g.print("KW4KD EM75 0DBm");  //buf4
   //u8g.print(StaInfo);
 } while( u8g.nextPage() ); 
}
 
 
//void CalcCntDwn(){ 
//  float Time2go = 600;//(float)((nextSnd - millis())/1000);
//  //if(Time2go>0){
//   //int MinCnt = (int) (Time2go/60);
//   //int SecCnt = ((int)Time2go-MinCnt)*60;
//   //sprintf (buf5,"Nxt Tx: %02d:%02d", MinCnt, SecCnt);
//  //}
//  return;
//  
//}

