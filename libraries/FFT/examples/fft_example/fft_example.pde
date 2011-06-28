/*
 *
 * Fixed-point FFT code from http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1286718155
 *
 * This example should give an fft with
 *  - sampling rate:           1ms
 *  - frequency resolution:   7.8Hz per band (x64) 
 *  - frequency range:        7.8Hz to 500Hz
 *
 * there is a lot that can be improved:
 *  - fix bugs
 *  - replace FIX_MPY by inline assembler
 *  - do sampling with timer and interrupts
 *  - use fix_fftr instead of fix_fft to save ram (i did not get the math right)
 */

#include <fix_fft.h>

char im[128];
char data[128];

void loop(){
  int static i = 0;
  static long tt;
  int val;
  
   if (millis() > tt){
	if (i < 128){
	  val = analogRead(pin_adc);
	  data[i] = val / 4 - 128;
	  im[i] = 0;
	  i++;  
	  
	}
	else{
	  //this could be done with the fix_fftr function without the im array.
	  fix_fft(data,im,7,0);
	  // I am only interessted in the absolute value of the transformation
	  for (i=0; i< 64;i++){
	     data[i] = sqrt(data[i] * data[i] + im[i] * im[i]);
	  }
	  
	  //do something with the data values 1..64 and ignore im
	  show_big_bars(data,0);
	}
    
    tt = millis();
   }
} 


