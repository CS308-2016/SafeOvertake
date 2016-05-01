#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"


#define PWM_FREQUENCY 55
int longPress1(){
	int b=0;
	while(b<10000){
		if(!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00){
			return 0;
		}
		SysCtlDelay(2000);
		b++;
	}
	return 1;
}
int longPress2(){
	int b=0;
	while(b<10000){
		if(!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00){
			return 0;
		}
		if(!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00){
			return 0;
		}
		SysCtlDelay(2000);
		b++;
	}
	return 1;
}


int main(void)
{
	volatile uint32_t ui32Load;
		volatile uint32_t ui32PWMClock;
		volatile uint8_t ui8AdjustMax = 244;
		volatile uint8_t ui8AdjustMin = 10;
		volatile uint8_t ui8AdjustR = ui8AdjustMax;
		volatile uint8_t ui8AdjustB = ui8AdjustMin;
		volatile uint8_t ui8AdjustG = ui8AdjustMin;
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
		SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
		GPIOPinConfigure(GPIO_PF1_M1PWM5);
		GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
		GPIOPinConfigure(GPIO_PF2_M1PWM6);
		GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
		GPIOPinConfigure(GPIO_PF3_M1PWM7);
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
		HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
		HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
		GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
		GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		ui32PWMClock = SysCtlClockGet() / 64;
		ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
		PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
		PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ui32Load);
		PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
		PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ui8AdjustR * ui32Load / 1000);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui8AdjustB * ui32Load / 1000);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui8AdjustG * ui32Load / 1000);
		PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
		PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
		PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
		PWMGenEnable(PWM1_BASE, PWM_GEN_2);
		PWMGenEnable(PWM1_BASE, PWM_GEN_3);

	/*
	while(1)
	{

		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
		{
			ui8AdjustG--;
			if (ui8AdjustG < 10)
			{
				ui8AdjustG = 10;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui8AdjustG * ui32Load / 1000);
		}

		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
		{
			ui8AdjustG++;
			if (ui8AdjustG > 254)
			{
				ui8AdjustG = 254;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui8AdjustG * ui32Load / 1000);
		}

		SysCtlDelay(100000);
	}*/


	int delay = 10000;
	int a=1;
	while(1){
		//auto mode

		while(1){
			if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00 && GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00){
				break;
			}

			else if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00){
				delay = delay-50;
				if(delay<5000){
					delay=5000;
				}
			}
			else if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00){
				delay = delay+50;
				if(delay>100000){
					delay = 100000;
				}
			}

			if(a==1){
				ui8AdjustR--;
				ui8AdjustG++;
				if(ui8AdjustG==ui8AdjustMax)a=2;
			}
			else if(a==2){
				ui8AdjustG--;
				ui8AdjustB++;
				if(ui8AdjustB==ui8AdjustMax)a=3;
			}
			else if(a==3){
				ui8AdjustB--;
				ui8AdjustR++;
				if(ui8AdjustR==ui8AdjustMax)a=1;
			}
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ui8AdjustR * ui32Load / 1000);
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui8AdjustB * ui32Load / 1000);
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui8AdjustG * ui32Load / 1000);
			SysCtlDelay(delay);
		}
		//manual mode

		int msk = 0;
		int mode = 0;
		while(1){
			if(mode==1){
				if( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
				{
					ui8AdjustR--;
					if (ui8AdjustR < 10)
					{
						ui8AdjustR = 10;
					}
					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ui8AdjustR * ui32Load / 1000);
				}

				if( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
				{
					ui8AdjustR++;
					if (ui8AdjustR > 254)
					{
						ui8AdjustR = 254;
					}
					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, ui8AdjustR * ui32Load / 1000);
				}
				SysCtlDelay(100000);
			}


			if(mode==2){
				if( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
				{
					ui8AdjustB--;
					if (ui8AdjustB < 10)
					{
						ui8AdjustB = 10;
					}
					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui8AdjustB * ui32Load / 1000);
				}

				if( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
				{
					ui8AdjustB++;
					if (ui8AdjustB > 254)
					{
						ui8AdjustB = 254;
					}
					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui8AdjustB * ui32Load / 1000);
				}
				SysCtlDelay(100000);
			}


			if(mode==3){
				if( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
				{
					ui8AdjustG--;
					if (ui8AdjustG < 10)
					{
						ui8AdjustG = 10;
					}
					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui8AdjustG * ui32Load / 1000);
				}

				if( GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
				{
					ui8AdjustG++;
					if (ui8AdjustG > 254)
					{
						ui8AdjustG = 254;
					}
					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui8AdjustG * ui32Load / 1000);
				}
				SysCtlDelay(100000);
			}


			if(longPress2()){
				//Mode 3
				mode = 3;
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 10 * ui32Load / 1000);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 10 * ui32Load / 1000);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 254 * ui32Load / 1000);
				continue;
			}
			/*
			else if(longPress1()){
				int i=0;

					if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00){
						int found = 0;
						while(msk>100){
							msk++;
							if(!GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00){
								if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00){
									mode = 2;
									PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 10 * ui32Load / 1000);
									PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 254 * ui32Load / 1000);
									PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 10 * ui32Load / 1000);
									found = 1;
								}
							}
						}
						if(found==1)continue;
						mode = 1;
						PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 254 * ui32Load / 1000);
						PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 10 * ui32Load / 1000);
						PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 10 * ui32Load / 1000);
					}


			}*/
		}
	}

}
