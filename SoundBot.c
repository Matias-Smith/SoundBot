
#include "driverlib.h"
#include <stdint.h>
#include "../inc/SysTick.h"
#include "../inc/Clock.h"

//Iterators
uint32_t Size;
uint32_t I;
uint16_t c;
uint16_t cc;
uint8_t start;
uint8_t Ldegree, Rdegree;
uint16_t noUTurn;

//Analyzing Samples (Digital Band Pass Filter)
int32_t INPUT_P6_0[1024];
int32_t INPUT_P6_1[1024];
float Real_INPUT_P6_0[1024];
float Real_INPUT_P6_1[1024];
float x1[1024];
float x2[1024];
float HighPass1[1024];
float HighPass2[1024];
float HighLowPass1[1024];
float HighLowPass2[1024];
float alpha;
float alpha2;

//Filtered Signal Processing (Determining Amplitude of Waves)
float max1, max2;
float min1, min2;
float Amplitude1, Amplitude2;
float avgAmplitude1, avgAmplitude2;
float avgAmpOverWave1, avgAmpOverWave2;

//Path Efficiency
float previousAvgAmp1, previousAvgAmp2;
int confidence;
uint8_t UTurn;

//Lines 47-48: Duration Motor is on for Each Value of Rdegree/Ldegree which is the Index
int howMuchLeft[6] = {3, 8, 11, 20, 25, 0};
int howMuchRight[6] = {3, 8, 11, 20, 25, 0};

//Speed
int boost;
int howMuchBoost;

uint8_t DIRECTION;

#define FORWARD    1
#define BACKWARD   2
#define LEFT       3
#define RIGHT      4
#define STOP       5
#define ROTATE_180 6
uint8_t MODE;

#define SAMPLING_MODE    1
#define RUNNING_MODE     2

#define PERIOD   100

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source 24MHz
        TIMER_A_CLOCKSOURCE_DIVIDER_12,         // SMCLK/12 = 1MHz Timer clock
        PERIOD,                                 // a period of 100 timer clocks => 10 KHz Frequency
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};


/* Application Defines */
#define TIMER_PERIOD 15000  // 10 ms PWM Period
#define DUTY_CYCLE1 0
#define DUTY_CYCLE2 0



/* Timer_A UpDown Configuration Parameter */
Timer_A_UpDownModeConfig upDownConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock SOurce
        TIMER_A_CLOCKSOURCE_DIVIDER_4,          // SMCLK/1 = 1.5MHz
        TIMER_PERIOD,                           // 15000 period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value

};

/* Timer_A Compare Configuration Parameter  (PWM3) */
Timer_A_CompareModeConfig compareConfig_PWM3 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_3,          // Use CCR3
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output but
        DUTY_CYCLE1
};

/* Timer_A Compare Configuration Parameter (PWM4) */
Timer_A_CompareModeConfig compareConfig_PWM4 =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_4,          // Use CCR4
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output but
        DUTY_CYCLE2
};

void TimerA2_Init(void);
void PWM_Init12(void);
void PWM_Init12(void);
void PWM_duty1(uint16_t duty1, Timer_A_CompareModeConfig* data);
void PWM_duty2(uint16_t duty1, Timer_A_CompareModeConfig* data);
void MotorInit(void);
void motor_forward(uint16_t leftDuty, uint16_t rightDuty);
void motor_right(uint16_t leftDuty, uint16_t rightDuty);
void motor_left(uint16_t leftDuty, uint16_t rightDuty);
void motor_backward(uint16_t leftDuty, uint16_t rightDuty);
void motor_stop(void);
void ADC_Ch14Ch15_Init(void);

int main(void)
{
     //Iterators
     Size=1000;
     I=Size-1;

     //Variables Used For Path Efficiency
     previousAvgAmp1 = 0;
     previousAvgAmp2 = 0;
     start = 0;
     confidence = 0;
     noUTurn = 0;
     UTurn = 0;
     Ldegree = 4;
     Rdegree = 4;

     //Variables to Increase Speed
     howMuchBoost = 45;
     boost = 0;


    // Set Microcontroller Clock = 48 MHz
    Clock_Init48MHz();

    PWM_Init12();

    // Systick Configuration
    SysTick_Init();

    // Motor Configuration
    MotorInit();





    // Port 5 Configuration: make P6.4 out
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);



   // Setup ADC for Channel A14 and A15
    ADC_Ch14Ch15_Init();

    // Timer A2 Configuration
    TimerA2_Init();

    DIRECTION  = FORWARD;
    MODE  = SAMPLING_MODE;
    //Keep the Program Iterating Infinitely
    while (1)
    {
    }

}

void TA2_0_IRQHandler(void)
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN4);
    GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN3);

    // IN SAMPLING MODE
     if(MODE == SAMPLING_MODE)
     {
           ADC14_toggleConversionTrigger(); // ask ADC to get data

           while(ADC14_isBusy()){};
           //Collect Samples From Memory
           INPUT_P6_0[I] = ADC14_getResult(ADC_MEM1);
           Real_INPUT_P6_0[I] = (INPUT_P6_0[I] * 3.3) / 16384;
           INPUT_P6_1[I] = ADC14_getResult(ADC_MEM0);
           Real_INPUT_P6_1[I] = (INPUT_P6_1[I] * 3.3) / 16384;


           if(I == 0)
           {
               I = Size-1;
               MODE= RUNNING_MODE;


            /* MAKE DIRECTION DECISION BASED SAMPLING RESULTS HERE */

               /* Digital filters for ADC data */

               for (c = 0; c < 1000; c++) {
                   x1[c] = Real_INPUT_P6_0[c];
                   x2[c] = Real_INPUT_P6_1[c];
               }

               //Band-Pass Filter
               alpha = 0.88837; //cutoff frequency of 200Hz (high pass)
               alpha2 = 0.65337;  //cutoff frequency of 3000Hz (Low pass)
               for (c = 1; c < 1000; c++) {
                     //High-Pass Filter First
                     HighPass1[c] = alpha * HighPass1[c-1] + alpha * (x1[c] - x1[c-1]);            // y1 is Real_INPUT_P4_6 after a high pass filter
                     HighPass2[c] = alpha * HighPass2[c-1] + alpha * (x2[c] - x2[c-1]);
                     //Then Low-Pass Filter
                     HighLowPass1[c] = HighLowPass1[c-1] + alpha2 * (HighPass1[c] - HighLowPass1[c-1]);
                     HighLowPass2[c] = HighLowPass2[c-1] + alpha2 * (HighPass2[c] - HighLowPass2[c-1]);

               }

               /* Lines 246-268: Compute Amplitude of the Waves */
               avgAmplitude1=0.0;
               avgAmplitude2=0.0;

               for (c = 0; c < 9; c++)
               {
                     max1=0.0;
                     max2=0.0;
                     min1=0.0;
                     min2=0.0;
                     for (cc = 100*c + 100; cc < 200 + 100*c; cc++)
                     {
                         if (HighLowPass2[cc] > max2) max2 = HighLowPass2[cc];
                         if (HighLowPass1[cc] > max1) max1 = HighLowPass1[cc];
                         if (HighLowPass2[cc] < min2) min2 = HighLowPass2[cc];
                         if (HighLowPass1[cc] < min1) min1 = HighLowPass1[cc];
                     }
                     Amplitude1 = max1 - min1;
                     Amplitude2 = max2 - min2;
                     avgAmplitude1 = avgAmplitude1+Amplitude1;
                     avgAmplitude2 = avgAmplitude2+Amplitude2;
               }
               avgAmpOverWave1 = avgAmplitude1/10;
               avgAmpOverWave2 = avgAmplitude2/10;

             //Line 273: Check if there is a significant sound
             //Lines 273-347: Determining Direction if there is Sound
               if(avgAmpOverWave1 > 0.5 | avgAmpOverWave2 > 0.5)
               {
                   //Lines 275-280: Upon Detecting a Sound, Wait 0.5 Seconds Before Processing Sound Waves (Minimize Initial Error)
                   if (start < 1)
                   {
                       start++;
                       SysTick_Wait10ms(50);
                   }
                   else
                   {

                       //Lines 285-290: Determine if Robot Should be Rotated 180 Degrees
                       if ((DIRECTION == FORWARD) & (previousAvgAmp1 < avgAmpOverWave1) & (previousAvgAmp2 < avgAmpOverWave2)) UTurn = 0;
                       if ((DIRECTION == FORWARD) & (previousAvgAmp1 > avgAmpOverWave1) & (previousAvgAmp2 > avgAmpOverWave2) & (confidence < 3))
                       {   //If the Robot has Consistently traveled the wrong direction Twice, We Execute 180 Degree Rotation
                           if (UTurn == 2) DIRECTION = ROTATE_180;
                           else UTurn++;
                       }

                       //Lines 293-303: Left Turn Calculations
                       if (DIRECTION == ROTATE_180) UTurn = 0;
                       else if (avgAmpOverWave2 > avgAmpOverWave1 + 0.02)
                       {
                           //Lines 296-301: Determine How Much To Turn Left Based on Amplitude Difference
                           if (avgAmpOverWave2 > avgAmpOverWave1 + 0.3) Ldegree = 4;
                           else if (avgAmpOverWave2 > avgAmpOverWave1 + 0.2) Ldegree = 3;
                           else if (avgAmpOverWave2 > avgAmpOverWave1 + 0.12) Ldegree = 2;
                           else if (avgAmpOverWave2 > avgAmpOverWave1 + 0.05) Ldegree = 1;
                           else if (avgAmpOverWave2 > avgAmpOverWave1 + 0.02) Ldegree = 0;
                           DIRECTION = LEFT;
                       }
                       //Lines 305-314: Right Turn Calculations
                       else if (avgAmpOverWave1 > avgAmpOverWave2 + 0.02)
                       {
                           //Lines 309-313: Determine How Much To Turn Right Based on Amplitude Difference
                           if (avgAmpOverWave1 > avgAmpOverWave2 + 0.3) Rdegree = 4;
                           else if (avgAmpOverWave1 > avgAmpOverWave2 + 0.2) Rdegree = 3;
                           else if (avgAmpOverWave1 > avgAmpOverWave2 + 0.12) Rdegree = 2;
                           else if (avgAmpOverWave1 > avgAmpOverWave2 + 0.05) Rdegree = 1;
                           else if (avgAmpOverWave1 > avgAmpOverWave2 + 0.02) Rdegree = 0;
                           DIRECTION = RIGHT;
                       }
                       //Lines 316-328: Traveling Forward
                       else
                       {
                           howMuchBoost = 40;
                           DIRECTION = FORWARD;
                           confidence++;

                           //Lines 323-327: Speed up the Motors if the Robot Already made numerous Decisions and is "Confident" it is headed in the Correct Direction
                           if (confidence >= 2)
                           {
                               boost = 7000;
                               howMuchBoost = 80;
                           }
                       }

                       //If First Sample Indicates the Microphones are the Same Distance From Microphone, Robot Must be Facing Directly Away From Source
                       //Lines 332-338: Determine if Initial Instruction Should be U-Turn
                       if ((Ldegree < 2 | Rdegree < 2 | DIRECTION == FORWARD) & (noUTurn == 0)) //Ldegree < 1 | Rdegree < 1 |
                       {
                           DIRECTION = ROTATE_180;
                           confidence = confidence++;

                       }
                       noUTurn = 30;

                       //Lines 341-345: Prevent Robot From Getting Stuck Turning Back and Forth in an Infinite Cycle
                       if ((abs(Ldegree - Rdegree) < 2) & (DIRECTION != FORWARD))
                       {
                           if((DIRECTION == LEFT) & (Ldegree > 0)) Ldegree--;
                           else if ((DIRECTION == RIGHT) & (Rdegree > 0)) Rdegree--;
                       }
                   }
               }
               //Lines 349-355: Executes if no Significant Sound is Detected, Recollects Samples.
               else
               {
                   DIRECTION = STOP;
                   confidence = 0;
                   start = 0;
                   if (noUTurn > 0 ) noUTurn--;
               }

               //Lines 358-359: Prevent Over Rotation
               if (DIRECTION == LEFT & avgAmpOverWave1 < previousAvgAmp1) DIRECTION = STOP;
               if (DIRECTION == RIGHT & avgAmpOverWave2 < previousAvgAmp2) DIRECTION = STOP;

               //Keep track of Previous Wave Amplitudes (Used in Various Algorithms)
               previousAvgAmp1 = avgAmpOverWave1;
               previousAvgAmp2 = avgAmpOverWave2;

           }
           //Lines 366-370: 1000 Samples have not Been Collected Yet
           else
           {
                 I--;
           }
     } //End of Sampling Mode Block

     // IN RUNNING MODE
     if(MODE == RUNNING_MODE)
     {

        if(DIRECTION  == FORWARD)
        {
            //Include Speed Boost If There is Some
            motor_forward(7000+boost,7000+boost);
            SysTick_Wait10ms(howMuchBoost);

            motor_stop();
            SysTick_Wait10ms(40);
            }

        else if (DIRECTION  == BACKWARD)
        {
            motor_backward(5000,5000);
            SysTick_Wait10ms(100);
        }
        else if (DIRECTION  == LEFT)
        {
            motor_left(6000,6000);
            SysTick_Wait10ms(howMuchLeft[Ldegree]);

        }
        else if (DIRECTION  == RIGHT)
        {
            motor_right(6000,6000);
            SysTick_Wait10ms(howMuchRight[Rdegree]);
        }
        else if (DIRECTION  == STOP)
        {
            motor_stop();
        }
        else if (DIRECTION  == ROTATE_180)
        {
            motor_right(5000,5000);
            SysTick_Wait10ms(80);
            DIRECTION = STOP;
        }
        MODE = SAMPLING_MODE;
        motor_stop();

     }
     //Clear Interrupt to Start Cycle Again
     Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

void TimerA2_Init(void){
/* Configuring Timer_A1 for Up Mode */
   Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);

   /* Enabling interrupts and starting the timer */
   Interrupt_enableSleepOnIsrExit();
   Interrupt_enableInterrupt(INT_TA2_0);
   Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
   /* Enabling MASTER interrupts */
   Interrupt_setPriority(INT_TA2_0, 0x20);
   Interrupt_enableMaster();

}


void PWM_Init12(void){

        /* Setting P2.6 and P2.7 and peripheral outputs for CCR */
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6 + GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

        /* Configuring Timer_A1 for UpDown Mode and starting */
        Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);
        Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);

        /* Initialize compare registers to generate PWM1 */
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM3);

        /* Initialize compare registers to generate PWM2 */
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM4);
}


void PWM_duty1(uint16_t duty1, Timer_A_CompareModeConfig* data)  // function definition
{
  if(duty1 >= TIMER_PERIOD) return; // bad input
  data->compareValue = duty1; // access a struct member through a pointer using the -> operator
  /* Initialize compare registers to generate PWM1 */
  Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM3);
}

void PWM_duty2(uint16_t duty2, Timer_A_CompareModeConfig* data)  // function definition
{
  if(duty2 >= TIMER_PERIOD) return; // bad input
  data->compareValue = duty2; // access a struct member through a pointer using the -> operator
  /* Initialize compare registers to generate PWM2 */
  Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM4);
}


void MotorInit(void){

    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4|GPIO_PIN5); // choose P5.4 and P5.5 as outputs
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN7); // choose P3.6 and P3.7 as outputs

}

void motor_forward(uint16_t leftDuty, uint16_t rightDuty){

    GPIO_setOutputLowOnPin( GPIO_PORT_P5, GPIO_PIN4|GPIO_PIN5); // choose P5.4 and P5.5 Low
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN7); // choose P3.6 and P3.7 High
    PWM_duty1(rightDuty, &compareConfig_PWM3);
    PWM_duty2(leftDuty,  &compareConfig_PWM4);

}

void motor_right(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN7); // choose P5.4 and P5.5 Low
    PWM_duty1(rightDuty, &compareConfig_PWM3);
    PWM_duty2(leftDuty,  &compareConfig_PWM4);
}

void motor_left(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN7); // choose P5.4 and P5.5 Low
    PWM_duty1(rightDuty, &compareConfig_PWM3);
    PWM_duty2(leftDuty,  &compareConfig_PWM4);

}

void motor_backward(uint16_t leftDuty, uint16_t rightDuty){
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4|GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN7); // choose P5.4 and P5.5 Low
    PWM_duty1(rightDuty, &compareConfig_PWM3);
    PWM_duty2(leftDuty,  &compareConfig_PWM4);
}


void motor_stop(void){

    PWM_duty1(0, &compareConfig_PWM3); //sets the duty cycles on the motors to 0
    PWM_duty2(0, &compareConfig_PWM4);

}


void ADC_Ch14Ch15_Init(void){

        /* Initializing ADC (MCLK/1/1) */
        ADC14_enableModule();
        ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,0);

        /* Configuring GPIOs for Analog In */

        //GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6,GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

        /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A14 - A15) */
        ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);

        ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
        ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false);
        /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
         *  is complete and enabling conversions */
        ADC14_disableInterrupt(ADC_INT1);

        /* Enabling Interrupts */
        Interrupt_disableInterrupt(INT_ADC14);
    //  Interrupt_enableMaster();

        /* Setting up the sample timer to automatically step through the sequence
         * convert.
         */
        ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

        /* Triggering the start of the sample */
        ADC14_enableConversion();

}

///////////////////////////////////////END/////////////////////////////////////////////

