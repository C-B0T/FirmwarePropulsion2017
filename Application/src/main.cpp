/**
 * @file    main.cpp
 * @author  Kevin WYSOCKI
 * @date    8 nov. 2016
 * @brief   Main
 */
#include <stdio.h>
#include <stdlib.h>

#include "../../STM32_Driver/inc/stm32f4xx.h"
#include "common.h"
#include "FreeRTOS.h"
#include "task.h"

#include "HAL.hpp"
#include "BrushlessMotor.hpp"
#include "Odometry.hpp"
#include "VelocityControl.hpp"
#include "PositionControl.hpp"
#include "ProfileGenerator.hpp"
#include "TrajectoryPlanning.hpp"
#include "../../STM32_Driver/inc/stm32f4xx_it.h"

#include "CommunicationHandler.hpp"
#include "Controller.hpp"


using namespace HAL;
using namespace Utils;
using namespace Location;
using namespace MotionControl;
using namespace Communication;



extern "C" void hard_fault_handler_c(unsigned int * hardfault_args)
{
      unsigned int stacked_r0;
      unsigned int stacked_r1;
      unsigned int stacked_r2;
      unsigned int stacked_r3;
      unsigned int stacked_r12;
      unsigned int stacked_lr;
      unsigned int stacked_pc;
      unsigned int stacked_psr;

      stacked_r0 = ((unsigned long) hardfault_args[0]);
      stacked_r1 = ((unsigned long) hardfault_args[1]);
      stacked_r2 = ((unsigned long) hardfault_args[2]);
      stacked_r3 = ((unsigned long) hardfault_args[3]);

      stacked_r12 = ((unsigned long) hardfault_args[4]);
      stacked_lr = ((unsigned long) hardfault_args[5]);
      stacked_pc = ((unsigned long) hardfault_args[6]);
      stacked_psr = ((unsigned long) hardfault_args[7]);

//      printf ("\r\n\r\n[Hard fault handler - all numbers in hex]\r\n");
//      printf ("R0 = %x\r\n", stacked_r0);
//      printf ("R1 = %x\r\n", stacked_r1);
//      printf ("R2 = %x\r\n", stacked_r2);
//      printf ("R3 = %x\r\n", stacked_r3);
//      printf ("R12 = %x\r\n", stacked_r12);
//      printf ("LR [R14] = %x  subroutine call return address\r\n", stacked_lr);
//      printf ("PC [R15] = %x  program counter\r\n", stacked_pc);
//      printf ("PSR = %x\r\n", stacked_psr);
//      printf ("BFAR = %x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
//      printf ("CFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
//      printf ("HFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
//      printf ("DFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
//      printf ("AFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
//      printf ("SCB_SHCSR = %x\r\n", SCB->SHCSR);

      while (1);
}
void HardFault_Handler(void)
{
__ASM(".extern hard_fault_handler_c");
__ASM("TST LR, #4");
__ASM("ITE EQ");
__ASM("MRSEQ R0, MSP");
__ASM("MRSNE R0, PSP");
__ASM("B hard_fault_handler_c");
}

void BusFault_Handler(void)
{
    while(1);
}
void MemManage_Handler(void)
{
    while(1);
}
void WWDG_IRQHandler(void)
{
    while(1);
}
void UsageFault_Handler(void)
{
    while(1);
}


TaskHandle_t xHandleTraces;

/**
 * @brief Initialize hardware
 */
static void HardwareInit (void)
{
    // Ensure all priority bits are assigned as preemption priority bits.
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

    // Enable all GPIO clock
    RCC_AHB1PeriphClockCmd((RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC |
                            RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
                            RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI),
                            ENABLE);

    // Enable Timer clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5,
                           ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,
                           ENABLE);

    // Enable USART Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // Enable I2C Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_I2C2 | RCC_APB1Periph_I2C3, ENABLE);

}

/**
 * @brief Main task handler
 */
void TASKHANDLER_Test (void * obj)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

//    CommunicationHandler * comHandler = CommunicationHandler::GetInstance();
    Controller * controller = Controller::GetInstance();

    // Get instances
//    GPIO *led1 = GPIO::GetInstance(GPIO::GPIO6);

    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

//        led1->Toggle();

    }
}


void TASKHANDLER_TestEncoder (void * obj)
{
	Encoder* leftEncoder = Encoder::GetInstance(Encoder::ENCODER0);
	Encoder* rightEncoder = Encoder::GetInstance(Encoder::ENCODER1);

	int32_t leftValue = 0, rightValue = 0;

	while(1)
	{
		leftValue = leftEncoder->GetRelativeValue();
		rightValue = rightEncoder->GetRelativeValue();

		printf("%d;%d\r\n", leftValue, rightValue);

		vTaskDelay(100);
	}
}

void TASKHANDLER_TestBrushlessMotor (void * obj)
{
	BrushlessMotorDriver* leftMotor = BrushlessMotorDriver::GetInstance(BrushlessMotorDriver::DRIVER0);
	BrushlessMotorDriver* rightMotor = BrushlessMotorDriver::GetInstance(BrushlessMotorDriver::DRIVER1);

	float32_t speed = 0.0f;

	leftMotor->SetSpeed(speed);
	rightMotor->SetSpeed(speed);

	leftMotor->SetDirection(BrushlessMotorDriver::FORWARD);
	rightMotor->SetDirection(BrushlessMotorDriver::FORWARD);

	leftMotor->Move();
	rightMotor->Move();

	while(1)
	{
		speed += 0.1;

		leftMotor->SetSpeed(speed);
		rightMotor->SetSpeed(speed);

		if(speed >= 1.0)
		{
			leftMotor->Brake();
			rightMotor->Brake();

			speed = 0.0f;

			if(leftMotor->GetDirection() == BrushlessMotorDriver::FORWARD)
			{
				leftMotor->SetDirection(BrushlessMotorDriver::REVERSE);
				rightMotor->SetDirection(BrushlessMotorDriver::REVERSE);
			}
			else
			{
				leftMotor->SetDirection(BrushlessMotorDriver::FORWARD);
				rightMotor->SetDirection(BrushlessMotorDriver::FORWARD);
			}

			vTaskDelay(1000);
		}

		vTaskDelay(200);
	}
}

/**
 * @brief Main
 */
int main(void)
{
    HardwareInit();

    // Start (Led init and set up led1)
//    GPIO *led1 = GPIO::GetInstance(GPIO::GPIO6);
//    led1->Set(GPIO::State::Low);

    // Serial init
//    Serial *serial0 = Serial::GetInstance(Serial::SERIAL0);


    // Welcome
//    printf("\r\n\r\nS/0 CarteProp Firmware V0.1 (" __DATE__ " - " __TIME__ ")\r\n");

    // Create Test task
    xTaskCreate(&TASKHANDLER_TestEncoder,
                "Test Task",
                128,
                NULL,
                3,
                NULL);


    vTaskStartScheduler();

    assert(0 == 1);

    return 0;
}

/**
 * @brief Assertion failed callback
 * @param file : File name where assertion occured
 * @param line : Line where assertion occured
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    printf(ASSERT_FAILED_MESSSAGE, file, line);

    while(1)
    {

    }
}

/**
 * @brief FreeRTOS Tick Hook
 */
void vApplicationTickHook(void)
{
    // Do something
}

/**
 * @brief FreeRTOS Idle Hook
 */
void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}

/**
 * @brief FreeRTOS Malloc Failed Hook
 */
void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */

    taskDISABLE_INTERRUPTS();
    printf("ERROR | Safe malloc failed !\n");
    for( ;; );
}

/**
 * @brief FreeRTOS Stack Overflow Hook
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    printf("ERROR | %s Task Stack Overflowed !\n", pcTaskName);
    for( ;; );
}
