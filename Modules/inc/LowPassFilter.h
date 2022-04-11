#ifndef __LOWPASSFILTER_H
#define __LOWPASSFILTER_H

#define AX_FILTER 0
#define AY_FILTER 1
#define AZ_FILTER 2

#define GX_FILTER 3
#define GY_FILTER 4
#define GZ_FILTER 5

#define ALT_FILTER 6

typedef struct {
    float inputTm1, inputTm2, inputTm3, inputTm4;
    float outputTm1, outputTm2, outputTm3, outputTm4;
} fourthOrderData_t;

typedef struct {
    float inputTm1,  inputTm2,  inputTm3,  inputTm4,  inputTm5;
    float outputTm1, outputTm2, outputTm3, outputTm4, outputTm5;
} fifthOrderData_t;

typedef struct {
    float inputTm1,  inputTm2,  inputTm3;
    float outputTm1, outputTm2, outputTm3;
} secondOrderData_t;

typedef struct {
    double inputTm1,  inputTm2,  inputTm3,  inputTm4,  inputTm5 , inputTm6;
    double outputTm1, outputTm2, outputTm3, outputTm4, outputTm5 ,outputTm6;
} sixthOrderData_t;

typedef struct {
    double inputTm1,  inputTm2,  inputTm3,  inputTm4,  inputTm5 , inputTm6,inputTm7 , inputTm8;
    double outputTm1, outputTm2, outputTm3, outputTm4, outputTm5 ,outputTm6,outputTm7 ,outputTm8;
} eighthOrderData_t;
///////////////////////////////////////////////////////////////////////////////
//  4th Order Low Pass Filter for 100 Hz Data
///////////////////////////////////////////////////////////////////////////////

extern fourthOrderData_t fourthOrder100Hz[15];
extern fifthOrderData_t  fifthOrder100Hz[7];
extern secondOrderData_t secondOrder100Hz[7];
extern sixthOrderData_t    sixthOrder100Hz[7];
extern eighthOrderData_t  eighthOrder100Hz[7];

///////////////////////////////////////////////////////////////////////////////

float computeFourthOrder100Hz(float currentInput, fourthOrderData_t * filterParameters);

float computeFifthOrder100Hz(float currentInput,  fifthOrderData_t * filterParameters);

float computeSecondOrder100Hz(float currentInput,  secondOrderData_t * filterParameters);//50hz

float computeSecondOrder100HzH(float currentInput, secondOrderData_t * filterParameters);
float computeFourthOrder100Hz2(float currentInput, fourthOrderData_t * filterParameters);

float computeSixthOrder100Hz(float currentInput, sixthOrderData_t * filterParameters);
float computeEighthOrder100Hz(float currentInput, eighthOrderData_t * filterParameters);


///////////////////////////////////////////////////////////////////////////////

void setupFourthOrder100Hz(void);

///////////////////////////////////////////////////////////////////////////////
//  4th Order Low Pass Filter for 200 Hz Data
///////////////////////////////////////////////////////////////////////////////

extern fourthOrderData_t fourthOrder200Hz[3];

///////////////////////////////////////////////////////////////////////////////

float computeFourthOrder200Hz(float currentInput, fourthOrderData_t * filterParameters);

///////////////////////////////////////////////////////////////////////////////

void setupFourthOrder200Hz(void);

///////////////////////////////////////////////////////////////////////////////
#endif
