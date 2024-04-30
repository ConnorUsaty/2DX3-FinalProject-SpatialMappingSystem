/*  

Connor Usaty
usatyc
400409624

Assigned Bus Speed: 24MHz
Assigned Measurement Status LED: PF4 (D3)
Assigned Additional  Status LED: PF0 (D4)

2DX3 - Final Project - Keil .c Code for motor control, ToF control and data collection/transmission to PC

To be used in combination with FinalProject.py to create a 3D spatial reconstruction of area scanned

*/

#include <stdint.h>
#include <stdio.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up


/// *************** DATA STRUCTURES **************** ///

struct MeasurementPair {
	// Data structure to hold measuredDistance and RangeStatus of each ToF scan for UART transmission
    uint16_t measuredDistance;
		uint8_t measuredRangeStatus;
};

/// ************** GLOBAL VARIABLES *************** ///

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C follower peripheral
int status = 0;

/// ************** FUNCTION PROTOTYPES *************** ///

void getDistanceMeasurement(int j, struct MeasurementPair *measuredData); // function prototype, forward declaration
void goHome(int i); // function prototype, forward declaration
void unspin(); // function prototype, forward declaration

/// ****************** Initializations and Function definitions ************* ///

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
		GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
		GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
		//GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
		GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

// Onboard Buttons -> PJ0 and PJ1
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ0 and PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ0 and PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//� Configure PJ0 and PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											//��Disable analog functionality on PJ0 and PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;													//	Enable weak pull up resistor on PJ0 and PJ1
}

// Stepper motor 
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        								// configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;     								// disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;        								// enable digital I/O on Port H pins (PH0-PH3)
																									// configure Port H as GPIO
  GPIO_PORTH_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port H	pins (PH0-PH3)	
	return;
}

// output; PM0 -> For bus speed demo
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};      // Allow time for clock to stabilize
		
	GPIO_PORTM_DIR_R |= 0x01;       								      		// Enable PM0 as output
  GPIO_PORTM_DEN_R |= 0x01;																	// Enable PM0 as digital pin
	return;
}


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}

	
void spin() {
	// Does one full 360 degree rotation CW and collects 32 distance measurements (Every 11.25 degrees)
	
	uint32_t delay = 2; //fastest speed = 2ms
	int j = 0;	// iteration variable for data transmission
	struct MeasurementPair measuredData[32]; // Array of 32 measurementPair's which store the Distance and RangeStatus of each measurement
	
	for (int i = 0; i < 512; ++i) { // 512 * 4 = 2048, overall 1 full 360 degree rotation.
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait1ms(delay);											
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(delay);
		
		// 512iterations = 360deg; 360 / 32 = 11.25deg, 512 / 32 = 16
		if(i%16 == 0){ // The motor stops 32 times (or every 11.25degrees)
			getDistanceMeasurement(j, measuredData);		  // gather data
			j++;
		}

		// check if PJ0 pressed -> voids the scan and returns to home where scan can be restarted
		if ((GPIO_PORTJ_DATA_R & 0x01) == 0x00) {
			goHome(i); // puts motor back to 0deg spot
			return; // returns to main
		}
	}
	
	// All scans have happened send over the MeasurementPair's
	for (int i = 0; i < 32; ++i) {
		sprintf(printf_buffer," %u, %u\r\n", measuredData[i].measuredDistance, measuredData[i].measuredRangeStatus); // Sends the measurement to UART
		UART_printf(printf_buffer);
		FlashLED4(1); // blink LED4 (PF0) Assigned Additonal Measurement LED to show UART transmission
	}

	unspin();							// spins 360deg CCW to untangle wires
}


void getDistanceMeasurement(int j, struct MeasurementPair *measuredData) {
	// Gets distance measurement at current angle and sends to PC over UART
	
		GPIO_PORTF_DATA_R |= 0x10; // Turn on PF4 while measurement happens
		
	// initialize variables
	  uint16_t Distance;
		uint8_t RangeStatus;
		uint8_t dataReady = 0;
	
		status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging
		
		for (int i = 0; i < 5; ++i)
		/*
		for some reason theres a delay in gathering distance data, so to fix it I just made it so that the sensor
		gathers distance data 5 times and takes the distance measurement of the 5th iteration so that we know its accurate
		and not some accidental delayed distance that was acquired. 
		*/
		{
			//5 wait until the ToF sensor's data is ready
			while (dataReady == 0){
			status = VL53L1X_CheckForDataReady(dev, &dataReady);
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
	  
			//7 read the data values from ToF sensor
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);					//7 The Measured Distance value

			SysTick_Wait1ms(5);
		}
		// store the result of scan in array that will be sent to PC later over UART
		measuredData[j].measuredDistance = Distance;
		measuredData[j].measuredRangeStatus = RangeStatus;
				
		VL53L1X_StopRanging(dev);
		
		GPIO_PORTF_DATA_R &= 0xEF; // Turn off PF4 now that measurement is done
}


void unspin() {
	// Does one full 360 degree rotation CCW to untangle the wires
	
	uint32_t delay = 2; //fastest speed = 2ms
	
	for(int i=0; i < 512; ++i) { // 512 * 4 = 2048, overall 1 full 360 degree rotation.
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(delay);	
		GPIO_PORTH_DATA_R = 0b00001100;		
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait1ms(delay);			
	}
}


void goHome(int i) {
	// CCW rotation until back at 0 degree spot
	
	uint32_t delay = 2; //fastest speed = 2ms
	
	for(; i >= 0; --i) { // returns motor back to home by undoing all steps that have been done
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(delay);	
		GPIO_PORTH_DATA_R = 0b00001100;		
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait1ms(delay);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait1ms(delay);			
	}
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

int main(void) {
	uint8_t sensorState=0;

	//initialize
	PLL_Init();																			// Set system bus speed to 24MHz
	SysTick_Init();																	// Initialize SysTick for 24MHz bus speed
	onboardLEDs_Init();															// Initialize onboard LEDs to inidcate system status
	I2C_Init();																			// Initialize I2C for communication with ToF sensor
	UART_Init();																		// Initialize UART for communication with PC
	PortJ_Init();																		// Initialize Port J for button input
	PortH_Init();																		// Initialize Port H for stepper motor
	PortM_Init();																		// Initialize Port M for bus speed demo

	// 1 Wait for device ToF booted
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
	}
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
	/* 2 Initialize the sensor with the default setting  */
	status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	
	// poll for button presses
	while(1) {
		
		// checks for button press at PJ1
		if ((GPIO_PORTJ_DATA_R & 0x02) == 0x00) {
			spin();
		}
		
		// checks for button press at PJ0
		else if ((GPIO_PORTJ_DATA_R & 0x01) == 0x00) {
			UART_printf("Measurements done\n"); // sent to PC so it knows scans are done and to visualize now
			break; // breaks while loop since measurements are done and ends program
		}
		
	/*
		BUS SPEED DEMO
	SysTick_Wait10ms(10);
	GPIO_PORTM_DATA_R ^= 0x01; // Toggle PM0 to show clock speed -> Show that up time is 10ms
		*/
	}
	
	
}

