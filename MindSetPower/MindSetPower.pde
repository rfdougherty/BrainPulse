/*
 * MindSetGamma.pde
 *
 * Arduino Interface to the Neurosky MindSet EEG headset.
 *
 * Computes the power spectrum on a running window of data.
 *
 * 2011.06.27 Bob Dougherty <bobd@stanford.edu> wrote it.
 * 
 */
 
#include <MindSet.h>
#include <SSD1306.h>
#include <Flash.h>
#include <fix_fft.h>

#define VERSION "0.5"

#define DEFAULT_REFRESH_INTERVAL 3

// Sample interval is ~1.95ms (1000/512)
#define FFTWINBITS 6
#define FFTWIN (1<<FFTWINBITS)
unsigned long dataReadyMicros;
// We need a circular buffer to implement our running window. The buffer
// must always have the current window available as a contiguous block. The
// fastest way to do that is to maintain a buffer that is twice as big, with
// two sequential copies of the data. [MORE HERE]
// E.g., for a 3-element buffer, ("^^^" denotes the contiguous block):
// 1 2 3 1 2 3
//       ^ ^ ^
// 4 2 3 4 2 3
//   ^ ^ ^
// 4 5 3 4 5 3
//     ^ ^ ^
// 4 5 6 4 5 6
//       ^ ^ ^
// 7 5 6 7 5 6
//   ^ ^ ^
// 7 8 6 7 8 6
//     ^ ^ ^
// 7 8 9 7 8 9
//       ^ ^ ^
// ...
// For buffer size n (n=3 here):
// // initialize buffer with old data (or zeros)
// for(i=0; i<n; i++){ buff[i] = data[i]; buff[i+n] = data[i]; }
// curBufferIndex = 1
// while(true){
//   buff[curBufferIndex-1] = newData; 
//   buff[curBufferIndex-1+n] = newData;
//   DO SOMETHING WITH &(buff[curBufferIndex]) HERE
//   curBufferIndex++; if(curBufferIndex>n) curBufferIndex=1;
// }
int gDataBuffer[FFTWIN*2];
int gRealBuffer[FFTWIN];
int gImagBuffer[FFTWIN];
int gCurBufferIndex;


#if defined(__AVR_AT90USB1286__)
  // Teensy2.0++ has LED on D6
  #define LED_ERR 6
  HardwareSerial btSerial = HardwareSerial();
  // Pin definitions for the OLED graphical display
  #define OLED_DC 24
  #define OLED_RESET 25
  #define OLED_SS 20
  #define OLED_CLK 21
  #define OLED_MOSI 22
#elif defined(__AVR_ATmega32U4__)
  // Teensy2.0 has LED on pin 11
  #define LED_ERR 11
  HardwareSerial btSerial = HardwareSerial();
  // Pin definitions for the OLED graphical display
  #define OLED_DC 11
  #define OLED_RESET 13
  #define OLED_SS 0
  #define OLED_CLK 1
  #define OLED_MOSI 2
#else
  // Assume Arduino (LED on pin 13)
  #define LED_ERR 13
  Serial btSerial = Serial();
  // Pin definitions for the OLED graphical display
  #define OLED_DC 11
  #define OLED_RESET 13
  #define OLED_SS 12
  #define OLED_CLK 10
  #define OLED_MOSI 9
#endif

SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_SS);

#define BAUDRATE 115200

#define LED_RED 9
#define LED_BLU 4
#define LED_GRN 5

#define SQUARE(a) ((a)*(a))

byte g_displayUpdateInterval;
MindSet g_mindSet;

void setup() {
  // Set up the serial port on the USB interface
  Serial.begin(BAUDRATE);
  Serial << F("*********************************************************\n");
  Serial << F("* MindSet ERP version ") << VERSION << F("\n");
  Serial << F("*********************************************************\n\n");
  Serial << "Starting up...\n";
  btSerial.begin(BAUDRATE);
  
  g_displayUpdateInterval = DEFAULT_REFRESH_INTERVAL;

  // Configure LED pins
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);
  pinMode(LED_ERR, OUTPUT);
  
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.display(); // show splashscreen
  for(int i=0; i<FFTWIN*2; i++)
    gDataBuffer[i] = 0; 
  gCurBufferIndex = 1;
  
  for(int i=0; i<255; i+=2){
    analogWrite(LED_RED, i);
    analogWrite(LED_GRN, i);
    analogWrite(LED_BLU, i);
    delay(5);
  }
  for(int i=255; i>=0; i-=2){
    analogWrite(LED_RED, i);
    analogWrite(LED_GRN, i);
    analogWrite(LED_BLU, i);
    delay(5);
  }
  
  // Attach the callback function to the MindSet packet processor
  g_mindSet.attach(dataReady);

  Serial << F("MindSet ERP Ready.\n\n");

}

//
// Main program loop. 
//
void loop() {
  // Need a buffer for the line of text that we show.
  static char stringBuffer[SSD1306_LCDLINEWIDTH+1];
  static unsigned long lastDataMicros;
  
  // We just feed bytes to the MindSet object as they come in. It will
  // call our callback whenever a complete data packet has been received and parsed.
  if(btSerial.available()) 
    g_mindSet.process(btSerial.read());
  
  // Turn off the LED if the counter has expired
  if((millis()-flickDutyStartMs)>flickDutyMs){
    analogWrite(LED_RED,0);
    analogWrite(LED_GRN,0);
    analogWrite(LED_BLU,0);
  }
  
  // Raw values from the MindSet are about -2048 to 2047
  if(repCount>=16){
    unsigned long diffMillis = (dataReadyMicros-lastDataMicros)/1000/8;
    Serial << F("\n");
    snprintf(stringBuffer, SSD1306_LCDLINEWIDTH+1, "%01d %02d %02d %03d %03d %04d ",
              g_mindSet.errorRate()>>5, min(g_mindSet.attention(),99), min(g_mindSet.meditation(),99), g_mindSet.gamma1()>>5, g_mindSet.gamma2()>>5, diffMillis);
    repCount = 0;
    lastDataMicros = dataReadyMicros;
    for(byte i=0; i<flickPeriod; i++){
      // bit-shift division, with rounding:
      Serial << ((buffer[i]+4)>>3) << F(",");
      buffer[i] = 0;
    }

  }
}


// 
// MindSet callback. 
// This function will be called whenever a new data packet is ready.
//
void dataReady() {
  //static char str[64];
  //if(g_mindSet.errorRate()<127 && g_mindSet.attention()>0)
  //  analogWrite(LED_RED, g_mindSet.attention()*2);
  //if(g_mindSet.errorRate()<127 && g_mindSet.meditation()>0)
  //  analogWrite(LED_BLU, g_mindSet.meditation()*2);
  
  dataReadyMicros = micros();
  
  if(g_mindSet.errorRate() == 0)
    digitalWrite(LED_ERR, HIGH);
  else
    digitalWrite(LED_ERR, LOW);
  
  // MindSet raw values are in the range [-2048,2047]
  gCurDataBuffer[gCurBufferIndex-1] = (byte)(g_mindSet.raw()>>4); 
  gCurDataBuffer[gCurBufferIndex-1+FFTWIN] = gCurDataBuffer[gCurBufferIndex-1];

  // Do the FFT
  // FFT result length is half the input data length (e.g., 64 samples in => 32 bins out).  
  // The first bin cannot be used, so in reality our result is 31 bins.
  //
  // The maximum frequency we can measure is half of the sampling rate (Nyquist)
  // so if we sample at 256Hz our maximum frequency is 128Hz.
  // So if we have 32 bins covering 128Hz, each bin represents 4Hz range, and the 
  // lowest bin (bin 1) is 4Hz - 8Hz
  //  0: DC                     // 16:  64 -  68 Hz
  //  1:   4 -   8 Hz           // 17:  68 -  72 Hz
  //  2:   8 -  12 Hz           // 18:  72 -  76 Hz
  //  3:  12 -  16 Hz           // 19:  76 -  80 Hz
  //  4:  16 -  20 Hz           // 20:  80 -  84 Hz
  //  5:  20 -  24 Hz           // 21:  84 -  88 Hz
  //  6:  24 -  28 Hz           // 22:  88 -  92 Hz
  //  7:  28 -  32 Hz           // 23:  92 -  96 Hz
  //  8:  32 -  36 Hz           // 24:  96 - 100 Hz
  //  9:  36 -  40 Hz           // 25: 100 - 104 Hz
  // 10:  40 -  44 Hz           // 26: 104 - 108 Hz
  // 11:  44 -  48 Hz           // 27: 108 - 112 Hz
  // 12:  48 -  52 Hz           // 28: 112 - 116 Hz
  // 13:  52 -  56 Hz           // 29: 116 - 120 Hz
  // 14:  56 -  60 Hz           // 30: 120 - 124 Hz
  // 15:  60 -  64 Hz           // 31: 124 - 128 Hz
  memset(gImagBuffer,0);
  memcpy(&(gDataBuffer[gCurBufferIndex]), gRealBuffer);
  fix_fft(gRealBuffer, gImagBuffer, FFTWINBITS);
  // Compute the gamma band power
  int gammaIndex = 10;
  int gammaPower = isqrt((unsigned long)(gRealBuffer[gammaIndex] * gRealBuffer[gammaIndex])
                         (unsigned long)(gImagBuffer[gammaIndex] * gImagBuffer[gammaIndex]));
  
  gCurBufferIndex++;
  if(gCurBufferIndex>n) 
    gCurBufferIndex = 1;

  refreshDisplay();
}

void refreshDisplay(){
  // We need to keep track of the current x,y data value. 0,0 is at the
  // upper left, so we want to flip Y and thus initialize by the height,
  // which is the bottom of the display.
  static byte curX;
  static byte curY = SSD1306_LCDHEIGHT;
  // The current data frame. Used to know when we are due for a display refresh.
  static byte curRefreshFrame;
  static char stringBuffer[SSD1306_LCDLINEWIDTH+1];
  static unsigned long lastDataMicros;
  
  curRefreshFrame++;
  unsigned long diffMillis = (dataReadyMicros-lastDataMicros)/1000/8;
  lastDataMicros = dataReadyMicros;
  
  curY -= g_mindSet.gamma1()>>7;
  //oled.drawline(curX, 10, curX, 14, WHITE);
  if(curRefreshFrame>g_displayUpdateInterval){ 
    snprintf(stringBuffer, SSD1306_LCDLINEWIDTH+1, "%01d %02d %02d %03d %03d %04d ",
              g_mindSet.errorRate()>>5, min(g_mindSet.attention(),99), min(g_mindSet.meditation(),99), 
              g_mindSet.gamma1()>>7, g_mindSet.gamma2()>>7, diffMillis);
    // Update the running plot
    // Clear the graph just in front of the current x position:
    oled.fillrect(curX+1, 8, 16, SSD1306_LCDHEIGHT-8, BLACK);
    // Clip the y-values to the plot area (SSD1306_LCDHEIGHT-1 at the bottom to 16 at the top)
    if(curY>SSD1306_LCDHEIGHT-1) curY = SSD1306_LCDHEIGHT-1;
    else if(curY<16) curY = 16;
    // Plot the pixel for the current data point
    oled.setpixel(curX, curY, WHITE);
    // Update the blue LED
    analogWrite(LED_BLU, curY<SSD1306_LCDHEIGHT ? (SSD1306_LCDHEIGHT-1-curY)*2 : 0);
    // Reset the y-position accumulator:
    curY = SSD1306_LCDHEIGHT-1;
    // Increment x, and check for wrap-around
    curX++;
    if(curX>=SSD1306_LCDWIDTH){
      curX = 0;
      // If we have wrapped around, we need to clear the first row.
      oled.drawrect(0, 8, 1, SSD1306_LCDHEIGHT-8, BLACK);
    }
    // Draw the status string at the top:
    oled.drawstring(0, 0, stringBuffer);
    // Finished drawing the the buffer; copy it to the device:
    oled.display();
    curRefreshFrame = 0;
  }
}

/*
 * Integer square-root approximation, by Jim Ulery. 
 * from http://www.azillionmonkeys.com/qed/sqroot.html
 */
unsigned int isqrt(unsigned long val) {
  unsigned long temp, g=0, b = 0x8000, bshft = 15;
  do {
    if(val >= (temp = (((g << 1) + b)<<bshft--))) {
      g += b;
      val -= temp;
    }
  } while(b >>= 1);
  return g;
}

