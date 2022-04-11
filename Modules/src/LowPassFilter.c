/*
  June 2012

  BaseFlightPlus Rev -

  An Open Source STM32 Based Multicopter

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick

  Designed to run on Naze32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

//#include "board.h"
#include "LowPassFilter.h"
///////////////////////////////////////////////////////////////////////////////

fourthOrderData_t fourthOrder100Hz[15];
fourthOrderData_t fourthOrder200Hz[3];

fifthOrderData_t fifthOrder100Hz[7];
secondOrderData_t secondOrder100Hz[7];

sixthOrderData_t  sixthOrder100Hz[7];
eighthOrderData_t  eighthOrder100Hz[7];

///////////////////////////////////////////////////////////////////////////////
//  4th Order Low Pass Filter for 100 Hz Data
///////////////////////////////////////////////////////////////////////////////

//#define B0_100HZ  0.001893594048567f//100hz
//#define B1_100HZ -0.002220262954039f
//#define B2_100HZ  0.003389066536478f
//#define B3_100HZ -0.002220262954039f
//#define B4_100HZ  0.001893594048567f
//#define A1_100HZ -3.362256889209355f
//#define A2_100HZ  4.282608240117919f
//#define A3_100HZ -2.444765517272841f
//#define A4_100HZ  0.527149895089809f//??????2????



//#define B0_100HZ  0.0002141509//50hz
//#define B1_100HZ  0.0008566037
//#define B2_100HZ  0.001284906f
//#define B3_100HZ  0.0008566037
//#define B4_100HZ  0.0002141509
//#define A1_100HZ  -3.425455
//#define A2_100HZ  4.479272
//#define A3_100HZ  -2.643718f
//#define A4_100HZ  0.5933269

//fc=25hz
#define B0_100HZ  0.00001504626
#define B1_100HZ  0.00006018503
#define B2_100HZ  0.00009027754
#define B3_100HZ  0.00006018503
#define B4_100HZ  0.00001504626
#define A1_100HZ  -3.725385
#define A2_100HZ  5.226004
#define A3_100HZ  -3.270902
#define A4_100HZ  0.7705239

//fc=75hz, from IMU_app
//#define B0_100HZ  9.726342E-04
//#define B1_100HZ  3.890537E-03
//#define B2_100HZ  5.835806E-03
//#define B3_100HZ  3.890537E-03
//#define B4_100HZ  9.726342E-04
//#define A1_100HZ  -3.103944E+00
//#define A2_100HZ  3.774453E+00
//#define A3_100HZ  -2.111238E+00
//#define A4_100HZ  4.562908E-01

////fs=1000hz,fc=75hz, fdatool design
//#define B0_100HZ  0.000848015373819484
//#define B1_100HZ  0.00339206149527794
//#define B2_100HZ  0.00508809224291691
//#define B3_100HZ  0.00339206149527794
//#define B4_100HZ  0.000848015373819484
//#define A1_100HZ  -3.2396080335111
//#define A2_100HZ  4.14420227001934
//#define A3_100HZ  -2.45985942643803
//#define A4_100HZ  0.569637405029393

//~10hz,?????????(?????(???)??????+—0.05m2s??,??????,????????????
//#define B0_100HZ  7.999845533961459e-007
//#define B1_100HZ  3.199938213584584e-006
//#define B2_100HZ  4.799907320376875e-006
//#define B3_100HZ  3.199938213584584e-006
//#define B4_100HZ  7.999845533961459e-007
//#define A1_100HZ  -3.843521068870213
//#define A2_100HZ  5.543304153983421
//#define A3_100HZ  -3.555449460935613
//#define A4_100HZ  0.855679175575259


#define B20_100HZ  0.0000004149425//10hz
#define B21_100HZ  0.000001659770
#define B22_100HZ  0.000002489655
#define B23_100HZ  0.000001659770
#define B24_100HZ  0.0000004149425
#define A21_100HZ  -3.893453
#define A22_100HZ  5.688233
#define A23_100HZ  -3.695783
#define A24_100HZ  0.9010106
    
//double B60_100HZ =  0.00001797538;//75hz6?
//double B61_100HZ = 0.0001078523;
//double B62_100HZ = 0.0002696307;
//double B63_100HZ = 0.0003595076;
//double B64_100HZ = 0.0002696307;
//double B65_100HZ = 0.0001078523;
//double B66_100HZ = 0.00001797538;
//double A61_100HZ =  -4.921746;
//double A62_100HZ =  10.35734;
//double A63_100HZ =  -11.89764;
//double A64_100HZ =  7.854533;
//double A65_100HZ =  -2.822109;
//double A66_100HZ =  0.4307710 ; 

//double B60_100HZ =  1.771089E-06;//25hz6?
//double B61_100HZ = 1.062654E-05;
//double B62_100HZ = 2.656634E-05;
//double B63_100HZ = 3.542179E-05;
//double B64_100HZ = 2.656634E-05;
//double B65_100HZ = 1.062654E-05;
//double B66_100HZ =  1.771089E-06;
//double A61_100HZ = -5.330512E+00;
//double A62_100HZ =  1.196611E+01;
//double A63_100HZ =  -1.447067E+01;
//double A64_100HZ =  9.937710E+00;
//double A65_100HZ =  -3.673283E+00;
//double A66_100HZ =  5.707561E-01;

double B60_100HZ = 3.1362209E-04;//25hz6?
double B61_100HZ = 1.88173609E-03;
double B62_100HZ = 4.7043249E-03;
double B63_100HZ = 6.2724299E-03;
double B64_100HZ = 4.7043249E-03;
double B65_100HZ = 1.88173609E-03;
double B66_100HZ = 3.1363109E-04;
double A61_100HZ = -5.691653E+00;
double A62_100HZ = 1.353172E+01;
double A63_100HZ = -1.719986E+01;
double A64_100HZ = 1.232689E+01;
double A65_100HZ = -4.722721E+00;
double A66_100HZ = 7.556340E-01;

#define B80_100HZ  7.999845533961459e-007//20hz
#define B81_100HZ  3.199938213584584e-006
#define B82_100HZ  4.799907320376875e-006
#define B83_100HZ  3.199938213584584e-006
#define B84_100HZ  7.999845533961459e-007
#define B85_100HZ  0.
#define B86_100HZ  0.
#define B87_100HZ  0.
#define B88_100HZ  0.
#define A81_100HZ  -3.843521068870213
#define A82_100HZ  5.543304153983421
#define A83_100HZ  -3.555449460935613
#define A84_100HZ  0.855679175575259
#define A85_100HZ  0.
#define A86_100HZ  0. 
#define A87_100HZ  0.
#define A88_100HZ  0.

//#define B0_100HZ  7.999845533961459e-007//???10hz,?????????(?????(???)??????+—0.05m2s??,??????,????????????
//#define B1_100HZ  3.199938213584584e-006
//#define B2_100HZ  4.799907320376875e-006
//#define B3_100HZ  3.199938213584584e-006
//#define B4_100HZ  7.999845533961459e-007
//#define A1_100HZ  -3.843521068870213
//#define A2_100HZ  5.543304153983421
//#define A3_100HZ  -3.555449460935613
//#define A4_100HZ  0.855679175575259

//#define BT0_100HZ  0.000101f
//#define BT1_100HZ  0.004246f
//#define BT2_100HZ  0.01666f
//#define BT3_100HZ  0.01244f
//#define BT4_100HZ  0.001765f
//#define BT5_100HZ  0.00002336f
//#define AT1_100HZ -3.202f
//#define AT2_100HZ  3.897f
//#define AT3_100HZ -2.621f
//#define AT4_100HZ  0.9082f
//#define AT5_100HZ  -0.1287f

#define BT0_100HZ  0.0001012f
#define BT1_100HZ  0.004259f
#define BT2_100HZ  0.01674f
#define BT3_100HZ  0.01252f
#define BT4_100HZ  0.001782f
#define BT5_100HZ  0.00002368f
#define AT1_100HZ -3.017f
#define AT2_100HZ  3.893f
#define AT3_100HZ -2.623f
#define AT4_100HZ  0.9141f
#define AT5_100HZ -0.1309f

//#define BT0_100HZ  0.000003671f//50hz
//#define BT1_100HZ  0.0001804f
//#define BT2_100HZ  0.0008249f
//#define BT3_100HZ  0.0007134f
//#define BT4_100HZ  0.0001167f
//#define BT5_100HZ  0.000001776f
//#define AT1_100HZ -3.99f
//#define AT2_100HZ  6.45f
//#define AT3_100HZ  -5.272f
//#define AT4_100HZ  2.175f
//#define AT5_100HZ -0.3618f

#define BS0_100HZ  0.0008663387//10hz
#define BS1_100HZ  0.001732678
#define BS2_100HZ  0.0008663387
#define AS1_100HZ  -1.919129
#define AS2_100HZ  0.9225943

#define BH0_100HZ  0.8001102//50hz??
#define BH1_100HZ  -1.600220
#define BH2_100HZ  0.8001102
#define AH1_100HZ  -1.911437
#define AH2_100HZ  0.6441715



float computeFourthOrder100Hz(float currentInput, fourthOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/50)

    float output;

    output = B0_100HZ * currentInput + B1_100HZ * filterParameters->inputTm1 + B2_100HZ * filterParameters->inputTm2 + B3_100HZ * filterParameters->inputTm3 + B4_100HZ * filterParameters->inputTm4 - A1_100HZ * filterParameters->outputTm1 - A2_100HZ * filterParameters->outputTm2 - A3_100HZ * filterParameters->outputTm3 - A4_100HZ * filterParameters->outputTm4;
//		output=currentInput*0.2+filterParameters->inputTm1*0.2+filterParameters->inputTm2*0.2+filterParameters->inputTm3*0.2+filterParameters->inputTm4*0.2;

    filterParameters->inputTm4 = filterParameters->inputTm3;
    filterParameters->inputTm3 = filterParameters->inputTm2;
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm4 = filterParameters->outputTm3;
    filterParameters->outputTm3 = filterParameters->outputTm2;
    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}

float computeEighthOrder100Hz(float currentInput, eighthOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/50)

    float output;

    output = B80_100HZ * currentInput + B81_100HZ * filterParameters->inputTm1
           + B82_100HZ * filterParameters->inputTm2 + B83_100HZ * filterParameters->inputTm3
           + B84_100HZ * filterParameters->inputTm4 + B85_100HZ * filterParameters->inputTm5
           + B86_100HZ * filterParameters->inputTm6 + B87_100HZ * filterParameters->inputTm7
           + B88_100HZ * filterParameters->inputTm8 - A81_100HZ * filterParameters->outputTm1
           - A82_100HZ * filterParameters->outputTm2 - A83_100HZ * filterParameters->outputTm3 
           - A84_100HZ * filterParameters->outputTm4 - A85_100HZ * filterParameters->outputTm5
           - A86_100HZ * filterParameters->outputTm6 - A87_100HZ * filterParameters->outputTm7
           - A88_100HZ * filterParameters->outputTm8;
    
    filterParameters->inputTm8 = filterParameters->inputTm7;
    filterParameters->inputTm7 = filterParameters->inputTm6;
    filterParameters->inputTm6 = filterParameters->inputTm5;
    filterParameters->inputTm5 = filterParameters->inputTm4;
    filterParameters->inputTm4 = filterParameters->inputTm3;
    filterParameters->inputTm3 = filterParameters->inputTm2;
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm8 = filterParameters->outputTm7;
    filterParameters->outputTm7 = filterParameters->outputTm6;
    filterParameters->outputTm6 = filterParameters->outputTm5;
    filterParameters->outputTm5 = filterParameters->outputTm4;
    filterParameters->outputTm4 = filterParameters->outputTm3;
    filterParameters->outputTm3 = filterParameters->outputTm2;
    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}

float computeSixthOrder100Hz(float currentInput, sixthOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/50)

    float output;

    output = B60_100HZ * currentInput + B61_100HZ * filterParameters->inputTm1 + B62_100HZ * filterParameters->inputTm2 + B63_100HZ * filterParameters->inputTm3 + B64_100HZ * filterParameters->inputTm4 + B65_100HZ * filterParameters->inputTm5  + B66_100HZ * filterParameters->inputTm6 - A61_100HZ * filterParameters->outputTm1 - A62_100HZ * filterParameters->outputTm2  - A63_100HZ * filterParameters->outputTm3 - A64_100HZ * filterParameters->outputTm4 - A65_100HZ * filterParameters->outputTm5 - A66_100HZ * filterParameters->outputTm6;   
    
    filterParameters->inputTm6 = filterParameters->inputTm5;
    filterParameters->inputTm5 = filterParameters->inputTm4;
    filterParameters->inputTm4 = filterParameters->inputTm3;
    filterParameters->inputTm3 = filterParameters->inputTm2;
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm6 = filterParameters->outputTm5;
    filterParameters->outputTm5 = filterParameters->outputTm4;
    filterParameters->outputTm4 = filterParameters->outputTm3;
    filterParameters->outputTm3 = filterParameters->outputTm2;
    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}

float computeFourthOrder100Hz2(float currentInput, fourthOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/50)

    float output;

    output = B20_100HZ * currentInput + B21_100HZ * filterParameters->inputTm1 + B22_100HZ * filterParameters->inputTm2 + B23_100HZ * filterParameters->inputTm3 + B24_100HZ * filterParameters->inputTm4 - A21_100HZ * filterParameters->outputTm1 - A22_100HZ * filterParameters->outputTm2 - A23_100HZ * filterParameters->outputTm3 - A24_100HZ * filterParameters->outputTm4;

    filterParameters->inputTm4 = filterParameters->inputTm3;
    filterParameters->inputTm3 = filterParameters->inputTm2;
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm4 = filterParameters->outputTm3;
    filterParameters->outputTm3 = filterParameters->outputTm2;
    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}


float computeFifthOrder100Hz(float currentInput, fifthOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/50)

    float output;

    output = BT0_100HZ * currentInput + BT1_100HZ * filterParameters->inputTm1 + BT2_100HZ * filterParameters->inputTm2 + BT3_100HZ * filterParameters->inputTm3 + BT4_100HZ * filterParameters->inputTm4 + BT5_100HZ * filterParameters->inputTm5 - AT1_100HZ * filterParameters->outputTm1 - AT2_100HZ * filterParameters->outputTm2 - AT3_100HZ * filterParameters->outputTm3 - AT4_100HZ * filterParameters->outputTm4 - AT5_100HZ * filterParameters->outputTm5;
    
    filterParameters->inputTm5 = filterParameters->inputTm4;
    filterParameters->inputTm4 = filterParameters->inputTm3;
    filterParameters->inputTm3 = filterParameters->inputTm2;
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm5 = filterParameters->outputTm4;
    filterParameters->outputTm4 = filterParameters->outputTm3;
    filterParameters->outputTm3 = filterParameters->outputTm2;
    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}

float computeSecondOrder100Hz(float currentInput, secondOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/50)

    float output;

    output = BS0_100HZ * currentInput + BS1_100HZ * filterParameters->inputTm1 + BS2_100HZ * filterParameters->inputTm2  - AS1_100HZ * filterParameters->outputTm1 - AS2_100HZ * filterParameters->outputTm2;
    
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}


float computeSecondOrder100HzH(float currentInput, secondOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/50)

    float output;

    output = BH0_100HZ * currentInput + BH1_100HZ * filterParameters->inputTm1 + BH2_100HZ * filterParameters->inputTm2  - AH1_100HZ * filterParameters->outputTm1 - AH2_100HZ * filterParameters->outputTm2;
    
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}
///////////////////////////////////////////////////////////////////////////////

void setupFourthOrder100Hz(void)
{
    fourthOrder100Hz[AX_FILTER].inputTm1 = 0.0f;
    fourthOrder100Hz[AX_FILTER].inputTm2 = 0.0f;
    fourthOrder100Hz[AX_FILTER].inputTm3 = 0.0f;
    fourthOrder100Hz[AX_FILTER].inputTm4 = 0.0f;

    fourthOrder100Hz[AX_FILTER].outputTm1 = 0.0f;
    fourthOrder100Hz[AX_FILTER].outputTm2 = 0.0f;
    fourthOrder100Hz[AX_FILTER].outputTm3 = 0.0f;
    fourthOrder100Hz[AX_FILTER].outputTm4 = 0.0f;

    fourthOrder100Hz[AY_FILTER].inputTm1 = 0.0f;
    fourthOrder100Hz[AY_FILTER].inputTm2 = 0.0f;
    fourthOrder100Hz[AY_FILTER].inputTm3 = 0.0f;
    fourthOrder100Hz[AY_FILTER].inputTm4 = 0.0f;

    fourthOrder100Hz[AY_FILTER].outputTm1 = 0.0f;
    fourthOrder100Hz[AY_FILTER].outputTm2 = 0.0f;
    fourthOrder100Hz[AY_FILTER].outputTm3 = 0.0f;
    fourthOrder100Hz[AY_FILTER].outputTm4 = 0.0f;

    fourthOrder100Hz[AZ_FILTER].inputTm1 = 16384.0f;
    fourthOrder100Hz[AZ_FILTER].inputTm2 = 16384.0f;
    fourthOrder100Hz[AZ_FILTER].inputTm3 = 16384.0f;
    fourthOrder100Hz[AZ_FILTER].inputTm4 = 16384.0f;

    fourthOrder100Hz[AZ_FILTER].outputTm1 = 16384.0f;
    fourthOrder100Hz[AZ_FILTER].outputTm2 = 16384.0f;
    fourthOrder100Hz[AZ_FILTER].outputTm3 = 16384.0f;
    fourthOrder100Hz[AZ_FILTER].outputTm4 = 16384.0f;
   
    fourthOrder100Hz[GX_FILTER].inputTm1 = 0.0f;
    fourthOrder100Hz[GX_FILTER].inputTm2 = 0.0f;
    fourthOrder100Hz[GX_FILTER].inputTm3 = 0.0f;
    fourthOrder100Hz[GX_FILTER].inputTm4 = 0.0f;

    fourthOrder100Hz[GX_FILTER].outputTm1 = 0.0f;
    fourthOrder100Hz[GX_FILTER].outputTm2 = 0.0f;
    fourthOrder100Hz[GX_FILTER].outputTm3 = 0.0f;
    fourthOrder100Hz[GX_FILTER].outputTm4 = 0.0f;
 
    fourthOrder100Hz[GY_FILTER].inputTm1 = 0.0f;
    fourthOrder100Hz[GY_FILTER].inputTm2 = 0.0f;
    fourthOrder100Hz[GY_FILTER].inputTm3 = 0.0f;
    fourthOrder100Hz[GY_FILTER].inputTm4 = 0.0f;

    fourthOrder100Hz[GY_FILTER].outputTm1 = 0.0f;
    fourthOrder100Hz[GY_FILTER].outputTm2 = 0.0f;
    fourthOrder100Hz[GY_FILTER].outputTm3 = 0.0f;
    fourthOrder100Hz[GY_FILTER].outputTm4 = 0.0f;
  
    fourthOrder100Hz[GZ_FILTER].inputTm1 = 0.0f;
    fourthOrder100Hz[GZ_FILTER].inputTm2 = 0.0f;
    fourthOrder100Hz[GZ_FILTER].inputTm3 = 0.0f;
    fourthOrder100Hz[GZ_FILTER].inputTm4 = 0.0f;

    fourthOrder100Hz[GZ_FILTER].outputTm1 = 0.0f;
    fourthOrder100Hz[GZ_FILTER].outputTm2 = 0.0f;
    fourthOrder100Hz[GZ_FILTER].outputTm3 = 0.0f;
    fourthOrder100Hz[GZ_FILTER].outputTm4 = 0.0f;
    
    
    
    fifthOrder100Hz[AX_FILTER].inputTm1 = 0.0f;
    fifthOrder100Hz[AX_FILTER].inputTm2 = 0.0f;
    fifthOrder100Hz[AX_FILTER].inputTm3 = 0.0f;
    fifthOrder100Hz[AX_FILTER].inputTm4 = 0.0f;
    fifthOrder100Hz[AX_FILTER].inputTm5 = 0.0f;

    fifthOrder100Hz[AX_FILTER].outputTm1 = 0.0f;
    fifthOrder100Hz[AX_FILTER].outputTm2 = 0.0f;
    fifthOrder100Hz[AX_FILTER].outputTm3 = 0.0f;
    fifthOrder100Hz[AX_FILTER].outputTm4 = 0.0f;
    fifthOrder100Hz[AX_FILTER].outputTm5 = 0.0f;


    fifthOrder100Hz[AY_FILTER].inputTm1 = 0.0f;
    fifthOrder100Hz[AY_FILTER].inputTm2 = 0.0f;
    fifthOrder100Hz[AY_FILTER].inputTm3 = 0.0f;
    fifthOrder100Hz[AY_FILTER].inputTm4 = 0.0f;
    fifthOrder100Hz[AY_FILTER].inputTm5 = 0.0f;

    fifthOrder100Hz[AY_FILTER].outputTm1 = 0.0f;
    fifthOrder100Hz[AY_FILTER].outputTm2 = 0.0f;
    fifthOrder100Hz[AY_FILTER].outputTm3 = 0.0f;
    fifthOrder100Hz[AY_FILTER].outputTm4 = 0.0f;
    fifthOrder100Hz[AY_FILTER].outputTm5 = 0.0f;

    fifthOrder100Hz[AZ_FILTER].inputTm1 = 0.0f;
    fifthOrder100Hz[AZ_FILTER].inputTm2 = 0.0f;
    fifthOrder100Hz[AZ_FILTER].inputTm3 = 0.0f;
    fifthOrder100Hz[AZ_FILTER].inputTm4 = 0.0f;
    fifthOrder100Hz[AZ_FILTER].inputTm5 = 0.0f;

    fifthOrder100Hz[AZ_FILTER].outputTm1 = 0.0f;
    fifthOrder100Hz[AZ_FILTER].outputTm2 = 0.0f;
    fifthOrder100Hz[AZ_FILTER].outputTm3 = 0.0f;
    fifthOrder100Hz[AZ_FILTER].outputTm4 = 0.0f;   
    fifthOrder100Hz[AZ_FILTER].outputTm5 = 0.0f;
  
    fifthOrder100Hz[GX_FILTER].inputTm1 = 0.0f;
    fifthOrder100Hz[GX_FILTER].inputTm2 = 0.0f;
    fifthOrder100Hz[GX_FILTER].inputTm3 = 0.0f;
    fifthOrder100Hz[GX_FILTER].inputTm4 = 0.0f;
    fifthOrder100Hz[GX_FILTER].inputTm5 = 0.0f;


    fifthOrder100Hz[GX_FILTER].outputTm1 = 0.0f;
    fifthOrder100Hz[GX_FILTER].outputTm2 = 0.0f;
    fifthOrder100Hz[GX_FILTER].outputTm3 = 0.0f;
    fifthOrder100Hz[GX_FILTER].outputTm4 = 0.0f;
    fifthOrder100Hz[GX_FILTER].outputTm5 = 0.0f;
 
    fifthOrder100Hz[GY_FILTER].inputTm1 = 0.0f;
    fifthOrder100Hz[GY_FILTER].inputTm2 = 0.0f;
    fifthOrder100Hz[GY_FILTER].inputTm3 = 0.0f;
    fifthOrder100Hz[GY_FILTER].inputTm4 = 0.0f;
    fifthOrder100Hz[GY_FILTER].inputTm5 = 0.0f;

    fifthOrder100Hz[GY_FILTER].outputTm1 = 0.0f;
    fifthOrder100Hz[GY_FILTER].outputTm2 = 0.0f;
    fifthOrder100Hz[GY_FILTER].outputTm3 = 0.0f;
    fifthOrder100Hz[GY_FILTER].outputTm4 = 0.0f;
    fifthOrder100Hz[GY_FILTER].outputTm5 = 0.0f;
   
    fifthOrder100Hz[GZ_FILTER].inputTm1 = 0.0f;
    fifthOrder100Hz[GZ_FILTER].inputTm2 = 0.0f;
    fifthOrder100Hz[GZ_FILTER].inputTm3 = 0.0f;
    fifthOrder100Hz[GZ_FILTER].inputTm4 = 0.0f;
    fifthOrder100Hz[GZ_FILTER].inputTm5 = 0.0f;

    fifthOrder100Hz[GZ_FILTER].outputTm1 = 0.0f;
    fifthOrder100Hz[GZ_FILTER].outputTm2 = 0.0f;
    fifthOrder100Hz[GZ_FILTER].outputTm3 = 0.0f;
    fifthOrder100Hz[GZ_FILTER].outputTm4 = 0.0f;
    fifthOrder100Hz[GZ_FILTER].outputTm5 = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////
//  4th Order Low Pass Filter for 200 Hz Data
///////////////////////////////////////////////////////////////////////////////

#define B0_200HZ  7.999845533961459e-007
#define B1_200HZ  3.199938213584584e-006
#define B2_200HZ  4.799907320376875e-006
#define B3_200HZ  3.199938213584584e-006
#define B4_200HZ  7.999845533961459e-007
  
#define A1_200HZ -3.843521068870213
#define A2_200HZ  5.543304153983421
#define A3_200HZ -3.555449460935613
#define A4_200HZ  0.855679175575259

float computeFourthOrder200Hz(float currentInput, fourthOrderData_t * filterParameters)
{
    // cheby2(4,60,12.5/100)
//#define B0_200HZ  0.001139392787073f
//#define B1_200HZ -0.003386240693441f
//#define B2_200HZ  0.004665482032666f
//#define B3_200HZ -0.003386240693441f
//#define B4_200HZ  0.001139392787073f
//
//#define A1_200HZ -3.692341608388116f
//#define A2_200HZ  5.123502002652351f
//#define A3_200HZ -3.165946995349404f
//#define A4_200HZ  0.734958387305099f
  

  
  //#define B0_100HZ  7.999845533961459e-007//???10hz,?????????(?????(???)??????+—0.05m2s??,??????,????????????
//#define B1_100HZ  3.199938213584584e-006
//#define B2_100HZ  4.799907320376875e-006
//#define B3_100HZ  3.199938213584584e-006
//#define B4_100HZ  7.999845533961459e-007
//
//#define A1_100HZ  -3.843521068870213
//#define A2_100HZ  5.543304153983421
//#define A3_100HZ  -3.555449460935613
//#define A4_100HZ  0.855679175575259
  
	float output;
  
    output = B0_200HZ * currentInput + B1_200HZ * filterParameters->inputTm1 + B2_200HZ * filterParameters->inputTm2 + B3_200HZ * filterParameters->inputTm3 + B4_200HZ * filterParameters->inputTm4 - A1_200HZ * filterParameters->outputTm1 - A2_200HZ * filterParameters->outputTm2 - A3_200HZ * filterParameters->outputTm3 - A4_200HZ * filterParameters->outputTm4;

    filterParameters->inputTm4 = filterParameters->inputTm3;
    filterParameters->inputTm3 = filterParameters->inputTm2;
    filterParameters->inputTm2 = filterParameters->inputTm1;
    filterParameters->inputTm1 = currentInput;

    filterParameters->outputTm4 = filterParameters->outputTm3;
    filterParameters->outputTm3 = filterParameters->outputTm2;
    filterParameters->outputTm2 = filterParameters->outputTm1;
    filterParameters->outputTm1 = output;

    return output;
}

///////////////////////////////////////////////////////////////////////////////

void setupFourthOrder200Hz(void)
{
    fourthOrder200Hz[AX_FILTER].inputTm1 = 0.0f;
    fourthOrder200Hz[AX_FILTER].inputTm2 = 0.0f;
    fourthOrder200Hz[AX_FILTER].inputTm3 = 0.0f;
    fourthOrder200Hz[AX_FILTER].inputTm4 = 0.0f;

    fourthOrder200Hz[AX_FILTER].outputTm1 = 0.0f;
    fourthOrder200Hz[AX_FILTER].outputTm2 = 0.0f;
    fourthOrder200Hz[AX_FILTER].outputTm3 = 0.0f;
    fourthOrder200Hz[AX_FILTER].outputTm4 = 0.0f;

    /////////////////////////////////////

    fourthOrder200Hz[AY_FILTER].inputTm1 = 0.0f;
    fourthOrder200Hz[AY_FILTER].inputTm2 = 0.0f;
    fourthOrder200Hz[AY_FILTER].inputTm3 = 0.0f;
    fourthOrder200Hz[AY_FILTER].inputTm4 = 0.0f;

    fourthOrder200Hz[AY_FILTER].outputTm1 = 0.0f;
    fourthOrder200Hz[AY_FILTER].outputTm2 = 0.0f;
    fourthOrder200Hz[AY_FILTER].outputTm3 = 0.0f;
    fourthOrder200Hz[AY_FILTER].outputTm4 = 0.0f;

    /////////////////////////////////////

    fourthOrder200Hz[AZ_FILTER].inputTm1 = -9.8065f;
    fourthOrder200Hz[AZ_FILTER].inputTm2 = -9.8065f;
    fourthOrder200Hz[AZ_FILTER].inputTm3 = -9.8065f;
    fourthOrder200Hz[AZ_FILTER].inputTm4 = -9.8065f;

    fourthOrder200Hz[AZ_FILTER].outputTm1 = -9.8065f;
    fourthOrder200Hz[AZ_FILTER].outputTm2 = -9.8065f;
    fourthOrder200Hz[AZ_FILTER].outputTm3 = -9.8065f;
    fourthOrder200Hz[AZ_FILTER].outputTm4 = -9.8065f;
}

///////////////////////////////////////////////////////////////////////////////