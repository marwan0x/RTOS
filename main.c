#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "TM4C123.h"

#define mainSW_INTURRUPT_PortF ((IRQn_Type)30)
#define mainSW_INTURRUPT_PortB ((IRQn_Type)1)
void PortA_Init();

void PortB_Init(void);

void PortF_Init();

//define a Semaphore handle
xSemaphoreHandle portF0;
xSemaphoreHandle portF4;
//semaphores for portB interrupt
xSemaphoreHandle portB0;
xSemaphoreHandle portB1;
xSemaphoreHandle portB2;
xSemaphoreHandle portB3;
xSemaphoreHandle portB6;
xSemaphoreHandle lockMutex;




void motorUp()
{
	GPIOA->DATA &= ~(0x60);
		
	GPIOA->DATA |= 0x20;
	GPIOF->DATA &= ~(0x0E);
	GPIOF->DATA |= 0x02;
		
	

}
void motorDown()
{
	GPIOA->DATA &= ~(0x60);
	GPIOA->DATA |= 0x40;
	GPIOF->DATA &= ~(0x0E);
	GPIOF->DATA |= 0x04;
		

}
void motorOff()
{
	GPIOA->DATA &= ~(0x60);
	GPIOF->DATA &= ~(0x0E);

}

void passengerUp(){
		xSemaphoreTake(portF0,0);
		uint32_t flag = 1;
		TickType_t xLastAwakeTime = 0;
		TickType_t Time;
	
	for(;;)
	{
			if(xSemaphoreTake(portF0,portMAX_DELAY)==pdTRUE)
			{
				if(xSemaphoreTake(lockMutex,0) == pdTRUE)
				{
				if(flag %2 == 0){
					Time = xTaskGetTickCount() - xLastAwakeTime;
					if(Time >= pdMS_TO_TICKS(250))
					{
						motorOff();
					}
				}else{
							xLastAwakeTime = xTaskGetTickCount();
							motorUp();


				}
					flag++;
					xSemaphoreGive(lockMutex);
				}
				
			}
	}
}
void passengerDown()
{
		xSemaphoreTake(portF4,0);
		uint32_t flag = 1;
		TickType_t xLastAwakeTime = 0;
		TickType_t Time;
	
	
	for(;;)
	{
		if(xSemaphoreTake(portF4,portMAX_DELAY)==pdTRUE)
		{
				if(xSemaphoreTake(lockMutex,0) == pdTRUE)
				{
					if(flag %2 == 0){
					Time = xTaskGetTickCount() - xLastAwakeTime;
					if(Time >= pdMS_TO_TICKS(250))
					{
						motorOff();
					}
				}else{
							xLastAwakeTime = xTaskGetTickCount();
							motorDown();


				}
					flag++;
					xSemaphoreGive(lockMutex);

				}		
		}
	}
}
void driverUp()
{
		xSemaphoreTake(portB0,0);
		uint32_t flag = 1;
		TickType_t xLastAwakeTime = 0;
		TickType_t Time;
	

	for(;;)
	{
			if(xSemaphoreTake(portB0,portMAX_DELAY)==pdTRUE)
			{
				
				if(flag %2 == 0){
					Time = xTaskGetTickCount() - xLastAwakeTime;
					if(Time >= pdMS_TO_TICKS(250))
					{
						motorOff();
					}
				}else if (flag %2 != 0){
							xLastAwakeTime = xTaskGetTickCount();
							motorUp();



				}
					flag++;
}
					
			

	}
}
void driverDown()
{
		xSemaphoreTake(portB1,0);
		uint32_t flag = 1;
		TickType_t xLastAwakeTime = 0;
		TickType_t Time;
	
	for(;;)
	{
			if(xSemaphoreTake(portB1,portMAX_DELAY)==pdTRUE)
			{
				if(flag %2 == 0){
					Time = xTaskGetTickCount() - xLastAwakeTime;
					if(Time >= pdMS_TO_TICKS(250))
					{
						motorOff();
					}
				}else{
							xLastAwakeTime = xTaskGetTickCount();
							motorDown();


				}
					flag++;
				}
	}
}
void limitSwitch()
{
		xSemaphoreTake(portB2,0);

	for(;;)
	{
			if((xSemaphoreTake(portB2,portMAX_DELAY)==pdTRUE) )
			{				
			//if an interrupt came from the limit switches
			//Turn off the motor
			motorOff();
}
	}}

	void lockSwitch()
	{
		uint32_t flag =0;
        xSemaphoreTake(portB3,0);
        for(;;)
        {
            if(xSemaphoreTake(portB3,portMAX_DELAY)==pdTRUE){
                if(flag %2 ==0){
                    xSemaphoreTake(lockMutex,0);
                }else{
                    xSemaphoreGive(lockMutex);
                }
                flag++;

            }
						
        }
		
	}
void emergency()
{
	xSemaphoreTake(portB6,0);

	for(;;)
	{
			if((xSemaphoreTake(portB6,portMAX_DELAY)==pdTRUE) )
			{				
			//if an interrupt came from the emergency button
			//Turn off the motor
			motorOff();
}
	}
}
void Initial_State(){

	for(;;){
		vTaskDelay(500);
	}

}
                         /*main function*/
/*------------------------------------------------------------------------*/
int main( void )
 {
		PortA_Init();
    PortF_Init();
	  PortB_Init();


		__ASM("CPSIE i");
	 
			xTaskCreate(passengerUp, "PassengerUp", 140, NULL, 2, NULL );
			xTaskCreate(passengerDown, "PassengerDown", 140, NULL, 2, NULL);
			xTaskCreate(driverUp, "driverUp", 140, NULL, 3, NULL );
			xTaskCreate(driverDown, "driverDown", 140, NULL, 3, NULL);
			xTaskCreate(limitSwitch, "limit", 140, NULL, 3, NULL);
			xTaskCreate(lockSwitch, "lock", 140, NULL, 4, NULL);
			xTaskCreate(emergency, "emergency", 140, NULL, 4, NULL);
	 
	 
			xTaskCreate( Initial_State, "Initial", 140, NULL, 1, NULL );
			
			vTaskStartScheduler();
		

    for( ;; );
}

/*------------------------------------------------------------------------*/
//Initialize the hardware of Port-F
void PortF_Init(void){ 
	vSemaphoreCreateBinary(portF0);
	vSemaphoreCreateBinary(portF4);
  SYSCTL->RCGCGPIO |= 0x00000020;    // 1) F clock
  GPIOF->LOCK = 0x4C4F434B;  				 // 2) unlock PortF PF0  
  GPIOF->CR = 0x1F;          				 // allow changes to PF4-0       
  GPIOF->AMSEL= 0x00;       				 // 3) disable analog function
  GPIOF->PCTL = 0x00000000;  				 // 4) GPIO clear bit PCTL  
  GPIOF->DIR = 0x0E;         				 // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIOF->AFSEL = 0x00;      				 // 6) no alternate function
  GPIOF->PUR = 0x11;       				   // enable pullup resistors on PF4,PF0       
  GPIOF->DEN = 0x1F;       				   // 7) enable digital pins PF4-PF0
	GPIOF->DATA = 0x00;
	
	// Setup the interrupt on PortF
	GPIOF->ICR = 0x11;     // Clear any Previous Interrupt 
	GPIOF->IM |=0x11;      // Unmask the interrupts for PF0 and PF4
	GPIOF->IS |= 0x00;     // Make bits PF0 and PF4 level sensitive
	GPIOF->IEV &= ~0x00;   // Sense on Low Level
	GPIOF->IBE |= 0x11;
	NVIC_SetPriority(mainSW_INTURRUPT_PortF,5);

	NVIC_EnableIRQ(mainSW_INTURRUPT_PortF);        // Enable the Interrupt for PortF in NVIC
}
//Initialize the hardware of Port-A
void PortA_Init(void)
{
	/*port-A is used only for the motor so we only need to initialize 2 pins as output*/
	SYSCTL->RCGCGPIO |= 0x00000001;   
  GPIOA->LOCK = 0x4C4F434B;  			
  GPIOA->CR = 0x60;          			      
  GPIOA->AMSEL= 0x00;       				 
  GPIOA->PCTL = 0x00000000;  				 
  GPIOA->DIR = 0x60; 
  GPIOA->AFSEL = 0x00;      			
  GPIOA->DEN = 0x60;       				  
	GPIOA->DATA = 0x00;
	
}
//Initialize the hardware of Port-B

void PortB_Init(void)
{
	/*port-B utilizes most of the buttons needed for the operation of the program so all the pins are used as input pins*/
	vSemaphoreCreateBinary(portB0);
	vSemaphoreCreateBinary(portB1);
	vSemaphoreCreateBinary(portB2);
	vSemaphoreCreateBinary(portB3);
	vSemaphoreCreateBinary(portB6);

	lockMutex = xSemaphoreCreateMutex();
	SYSCTL->RCGCGPIO |= 0x00000002;  
  GPIOB->LOCK = 0x4C4F434B;  				  
  GPIOB->CR = 0x7F;          				 
  GPIOB->AMSEL= 0x00;       				 
  GPIOB->PCTL = 0x00000000;  				 
  GPIOB->DIR =0x00;         				    
  GPIOB->AFSEL = 0x00;      				 
  GPIOB->PUR = 0x6F;       				        
  GPIOB->DEN = 0x7F;
	GPIOB->DATA = 0x00;
	
	GPIOB->ICR = 0x5F; 
	GPIOB->IM |=0x5F;     
	GPIOB->IS |= 0x4C;     
	GPIOB->IEV &= ~0x4C;  
	GPIOB->IBE |= 0x13;
	NVIC_SetPriority(mainSW_INTURRUPT_PortB,5);
	NVIC_EnableIRQ(mainSW_INTURRUPT_PortB);        
}

/*------------------------------------------------------------------------*/
//Port-F handler
void GPIOF_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if((GPIOF->RIS & (0x01))== 1)
	{		
		for(uint32_t j =0; j<= 300000; j++){}

		xSemaphoreGiveFromISR(portF0,&xHigherPriorityTaskWoken);
	}
	else if((GPIOF->RIS & (0x10))== 0x10)
	{
		for(uint32_t j =0; j<= 300000; j++){}

		xSemaphoreGiveFromISR(portF4,&xHigherPriorityTaskWoken);
	}
	
	GPIOF->ICR = 0x11;        // clear the interrupt flag of PORTF
  i= GPIOF->ICR ;           // Reading the register to force the flag to be cleared


	
}
// Port-B handler
 void GPIOB_Handler(void){

	uint32_t i;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if((GPIOB->RIS & (0x01))== 0x01)
	{		
		for(uint32_t j =0; j<= 300000; j++){}
		xSemaphoreGiveFromISR(portB0,&xHigherPriorityTaskWoken);
	}
	else if((GPIOB->RIS & (0x02))== 0x02)
	{
		for(uint32_t j =0; j<= 300000; j++){}
		xSemaphoreGiveFromISR(portB1,&xHigherPriorityTaskWoken);
	}

	else if((GPIOB->RIS & (0x08))== 0x08 || (GPIOB->RIS & (0x04))== 0x04)
	{
		xSemaphoreGiveFromISR(portB2,&xHigherPriorityTaskWoken);

	}
	else if((GPIOB->RIS & (0x10))== 0x10)
	{		
		xSemaphoreGiveFromISR(portB3,&xHigherPriorityTaskWoken);
	}
	else if((GPIOB->RIS & (0x40))== 0x40)
	{		
		xSemaphoreGiveFromISR(portB6,&xHigherPriorityTaskWoken);
	}
	
	GPIOB->ICR = 0x1F;       
  i= GPIOB->ICR ;         

	
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}