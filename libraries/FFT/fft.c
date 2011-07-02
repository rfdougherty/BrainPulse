/************************************************************************
	fft.c

    FFT Audio Analysis
    Copyright (C) 2011 Simon Inns

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Email: simon.inns@gmail.com

************************************************************************/

#ifndef FFT_C
#define FFT_C

#include <htc.h>
#include "fft.h"

// fix_fft.c - Fixed-point in-place Fast Fourier Transform

// All data are fixed-point short integers, in which -32768
// to +32768 represent -1.0 to +1.0 respectively. Integer
// arithmetic is used for speed, instead of the more natural
// floating-point.
//
// For the forward FFT (time -> freq), fixed scaling is
// performed to prevent arithmetic overflow, and to map a 0dB
// sine/cosine wave (i.e. amplitude = 32767) to two -6dB freq
// coefficients.
//
// Written by:  Tom Roberts  11/8/89
// Made portable:  Malcolm Slaney 12/15/94 malcolm@interval.com
// Enhanced:  Dimitrios P. Bouras  14 Jun 2006 dbouras@ieee.org
// Ported to PIC18F:  Simon Inns 20110104

/*
  fix_fft() - perform forward fast Fourier transform.
  fr[n],fi[n] are real and imaginary arrays, both INPUT AND
  RESULT (in-place FFT), with 0 <= n < 2**m
*/
void fix_fft(short fr[], short fi[], short m)
{
	long int mr = 0, nn, i, j, l, k, istep, n, shift;
	short qr, qi, tr, ti, wr, wi;

	n = 1 << m;
	nn = n - 1;

	/* max FFT size = N_WAVE */
	//if (n > N_WAVE) return -1;

	/* decimation in time - re-order data */
	for (m=1; m<=nn; ++m)
	{
		l = n;
		do
		{
			l >>= 1;
		} while (mr+l > nn);
		
		mr = (mr & (l-1)) + l;
		if (mr <= m) continue;
		
		tr = fr[m];
		fr[m] = fr[mr];
		fr[mr] = tr;
		ti = fi[m];
		fi[m] = fi[mr];
		fi[mr] = ti;
	}

	l = 1;
	k = LOG2_N_WAVE-1;
	
	while (l < n)
	{
		/*
		  fixed scaling, for proper normalization --
		  there will be log2(n) passes, so this results
		  in an overall factor of 1/n, distributed to
		  maximize arithmetic accuracy.

		  It may not be obvious, but the shift will be
		  performed on each data point exactly once,
		  during this pass.
		*/
		
		// Variables for multiplication code
		long int c;
		short b;
		
		istep = l << 1;
		for (m=0; m<l; ++m)
		{
			j = m << k;
			/* 0 <= j < N_WAVE/2 */
			wr =  pgm_read_int_near(Sinewave + j+N_WAVE/4);
			wi = -pgm_read_int_near(Sinewave + j);

			wr >>= 1;
			wi >>= 1;
			
			for (i=m; i<n; i+=istep)
			{
				j = i + l;
				
				// Here I unrolled the multiplications to prevent overhead
				// for procedural calls (we don't need to be clever about 
				// the actual multiplications since the pic has an onboard
				// 8x8 multiplier in the ALU):
				
				// tr = FIX_MPY(wr,fr[j]) - FIX_MPY(wi,fi[j]);
				c = ((long int)wr * (long int)fr[j]);
				c = c >> 14;
				b = c & 0x01;
				tr = (c >> 1) + b;
				
				c = ((long int)wi * (long int)fi[j]);
				c = c >> 14;
				b = c & 0x01;
				tr = tr - ((c >> 1) + b);
				
				// ti = FIX_MPY(wr,fi[j]) + FIX_MPY(wi,fr[j]);
				c = ((long int)wr * (long int)fi[j]);
				c = c >> 14;
				b = c & 0x01;
				ti = (c >> 1) + b;
				
				c = ((long int)wi * (long int)fr[j]);
				c = c >> 14;
				b = c & 0x01;
				ti = ti + ((c >> 1) + b);
				
				qr = fr[i];
				qi = fi[i];
				qr >>= 1;
				qi >>= 1;

				fr[j] = qr - tr;
				fi[j] = qi - ti;
				fr[i] = qr + tr;
				fi[i] = qi + ti;
			}
		}
		
		--k;
		l = istep;
	}
}

#endif 

