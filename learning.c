void LED_Init(void){

    // Port 2 Configuration: Datasheet Page 141

    P2->SEL0 &= ~0x07;    // 0x07 = 0x00000111 and ~0x07 = 0x11111000
    P2->SEL1 &= ~0x07;    // 1) configure P2.2-P2.0 as GPIO
    P2->DIR |= 0x07;      // 2) make P2.2-P2.0 outputs
    P2->DS |= 0x07;       // 3) activate increased drive strength
    P2->OUT &= ~0x07;     //    all LEDs off
}

/* 
The SEL &= will force zeros, and in this case forces zeros in the last 3 with ~0x07. 
The DIR sets outputs, and using the |= 0x07, it forces 1s in the last 3 positions, which correspond to out LEDs. 
*/

int main(void){

    LED_Init();

    // RED LED = P2.0 ; GREEN LED = P2.1 ; BLUE LED = P2.2

    P2->OUT |= 0x01; // P2->OUT = P2->OUT or 0x00000001  => P2.0 = 1

    P2->OUT |= 0x02;
    P2->OUT |= 0x04;  
}

/* 
P2->OUT |= 0x0n turns the related LED on. 
To pick the LED, use the number that makes the binary mask in the right position. 
Ex. P2->OUT |=0x02 represents P2.1. 0x04 = 2.2 etc.
*/

int main(void)
{
    // Port 2 Configuration for Toggling LED
    LED_Init();


    // RED LED = P2.0 ; GREEN LED = P2.1 ; BLUE LED = P2.2
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
    // Assignment: Keep the red LED on, Turn ON GREEN and BLUE LED

}

/*
GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1) will set the output to high at P2.1. 
Pick the right port and pin number to set outputs.
*/

void LED_Init(void){

    // Port 2 Configuration: make P2.2-P2.0 out
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,PIN_ALL16);
}

/*
The LED_Init() function simply sets output pins, at P2.0, 2.1, 2.2 and sets all the LEDs to off.
*/

while(1){

        SysTick_Wait10ms(100); // wait 1 second

        // RED LED = P2.0 ; GREEN LED = P2.1 ; BLUE LED = P2.2
        // Example: Toggle  RED LED
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
        SysTick_Wait10ms(100);
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
      
}

/*
GPIO_toggleOutputOnPin simply sets the output to the other option, so if it was HIGH, it sets it to LOW. 
So every second, the code will turn on/off the red LED. 
Between each toggle of the green LED, there are 2 delay functions, 
so it only toggles every 2s. Between every toggle of red, 
there is only one delay function, so it toggles every second.
*/

if (status & ADC_INT1)
    {
        INPUT_P4_6 = ADC14_getResult(ADC_MEM1);
        Real_INPUT_P4_6 = (INPUT_P4_6 * 3.3) / 16384;
        INPUT_P4_7 = ADC14_getResult(ADC_MEM0)>>2;
        Real_INPUT_P4_7 = (INPUT_P4_7 * 3.3) / 4095;
    }

/*
The main addition here is to add the >>2 in the calculation for input 4.7. 
This converts the 14 bit result into a 12 bit one. 
And in the calculation for Real_INPUT_P4_7, the division is by 4095, since the input 4.7 is smaller by 2 bits.
*/

void TA2_0_IRQHandler(void)
{
    i++;
    if (i==9){
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
        i=0;
    }
    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

/*
This addition simply adds a counter in the i variable, which increments every time the handler is called. 
And since it’s called every 0.1s, after ten calls or 1s, the pin is toggled.
*/

#define TIMER_PERIOD   50 //changes frequency of calls to 10kHz
int main(void)
{

    Size1=1000; //sets size of array for 4.6 to be 1000
    I1=Size1-1; 
    Size2=10; // sets size of array for 4.7 to be 10
    I2=Size2-1;

    Clock_Init48MHz();


    // Port 5 Configuration: make P6.4 out
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN6);
    // Setup ADC for Channel A6 and A7
    ADC_Ch67_Init();

    // Timer A1 Configuration
    TimerA2_Init();

    while (1)
    {

    }
}
void TA2_0_IRQHandler(void)
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN5);
    GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN4);
    ADC14_toggleConversionTrigger();    //sample ADC once per timer interrupt
    i++;
    while(ADC14_isBusy()){};
    if (i==4){
        INPUT_P4_6[I1] = ADC14_getResult(ADC_MEM1);
        Real_INPUT_P4_6[I1] = (INPUT_P4_6[I1] * 3.3) / 16384;
        GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN4);
        i=0;
    }
    INPUT_P4_7[I2] = ADC14_getResult(ADC_MEM0);
    Real_INPUT_P4_7[I2] = (INPUT_P4_7[I2] * 3.3) / 16384;
    if(I1 == 0)
    {
        I1 = Size1-1;
    }
    else
    {
        I1--;
    }
    if(I2 == 0)
    {
        I2 = Size2-1;
    }
    else
    {
        I2--;
    }
    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

/*
In the define timer, changing it to 50 doubles the frequency from the original of 5kHz to 10kHz. 
The main addition in the handler is the counter for i again. 
Since this handler is running at 10kHz, only updating 4.6 every 5 triggers means it’s running at 2kHz.
*/
