/**
 * @file    VelocityControl.cpp
 * @author  Jeremy ROULLAND
 * @date    20 dec. 2016
 * @brief   VelocityControl class
 *
 *
 */

#include "VelocityControl.hpp"
#include "common.h"

#include <stdio.h>

using namespace HAL;
using namespace Utils;
using namespace MotionControl;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define ANGULAR_VELOCITY_PID_KP     (0.8f)
#define ANGULAR_VELOCITY_PID_KI     (0.5f)
#define ANGULAR_VELOCITY_PID_KD     (0.0f)

#define LINEAR_VELOCITY_PID_KP      (4.5f)
#define LINEAR_VELOCITY_PID_KI      (2.891845f)
#define LINEAR_VELOCITY_PID_KD      (0.0f)

#define VC_MOTOR_LEFT               (BrushlessMotorDriver::ID::DRIVER0)
#define VC_MOTOR_RIGHT              (BrushlessMotorDriver::ID::DRIVER1)

#define VC_TASK_STACK_SIZE          (256u)
#define VC_TASK_PRIORITY            (configMAX_PRIORITIES-3)

#define VC_TASK_PERIOD_MS           (5u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::VelocityControl* _velocityControl = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static VC_DEF _getDefStructure (enum VelocityControl::ID id)
{
    VC_DEF def;

    assert(id < VelocityControl::VELOCITY_MAX);

    switch(id)
    {
        case VelocityControl::ANGULAR:
            def.Motors.ID_left        =    VC_MOTOR_LEFT;
            def.Motors.ID_right       =    VC_MOTOR_RIGHT;
            def.PID_Angular.kp        =    ANGULAR_VELOCITY_PID_KP;
            def.PID_Angular.ki        =    ANGULAR_VELOCITY_PID_KI;
            def.PID_Angular.kd        =    ANGULAR_VELOCITY_PID_KD;
            break;

        case VelocityControl::LINEAR:
            def.Motors.ID_left       =    VC_MOTOR_LEFT;
            def.Motors.ID_right      =    VC_MOTOR_RIGHT;
            def.PID_Linear.kp        =    LINEAR_VELOCITY_PID_KP;
            def.PID_Linear.ki        =    LINEAR_VELOCITY_PID_KI;
            def.PID_Linear.kd        =    LINEAR_VELOCITY_PID_KD;
            break;

        default:
            break;
    }

    return def;
}

/*----------------------------------------------------------------------------*/
/* Class Implementation                                                       */
/*----------------------------------------------------------------------------*/

namespace MotionControl
{
    VelocityControl* VelocityControl::GetInstance(bool standalone)
    {
        // If VelocityControl instance already exists
        if(_velocityControl != NULL)
        {
            return _velocityControl;
        }
        else
        {
            _velocityControl = new VelocityControl(standalone);
            return _velocityControl;
        }
    }

    /**
     * @brief  VelocityControl constructor
     */
    VelocityControl::VelocityControl(bool standalone)
    {
        this->name = "VelocityControl";
        this->taskHandle = NULL;

        // Init Angular velocity control
        this->def = _getDefStructure(VelocityControl::ANGULAR);
        this->pid_angular = PID(this->def.PID_Angular.kp,
                                this->def.PID_Angular.ki,
                                this->def.PID_Angular.kd,
                                VC_TASK_PERIOD_MS/1000.0);

        // Init Linear velocity control
        this->def = _getDefStructure(VelocityControl::LINEAR);
        this->pid_linear  = PID(this->def.PID_Linear.kp,
                                this->def.PID_Linear.ki,
                                this->def.PID_Linear.kd,
                                VC_TASK_PERIOD_MS/1000.0);


        this->odometry = Odometry::GetInstance();

        this->leftMotor  = BrushlessMotorDriver::GetInstance(this->def.Motors.ID_left);
        this->rightMotor = BrushlessMotorDriver::GetInstance(this->def.Motors.ID_right);

        this->angularVelocity = 0.0f;
        this->linearVelocity  = 0.0f;

        this->angularSpeed = 0.0f;
        this->linearSpeed  = 0.0f;

        this->leftSpeed  = 0.0f;
        this->rightSpeed = 0.0f;

        this->enable = true;
        this->stop   = false;

        if(standalone)
        {
            // Create task
            xTaskCreate((TaskFunction_t)(&VelocityControl::taskHandler),
                        this->name.c_str(),
                        VC_TASK_STACK_SIZE,
                        NULL,
                        VC_TASK_PRIORITY,
                        NULL);
        }

        this->xMutex = NULL;
        this->xMutex = xSemaphoreCreateMutex();
    }

    /**
     * @brief  VelocityControl compute
     */
    void VelocityControl::Compute(float32_t period)
    {
        float32_t currentAngularVelocity = 0.0;
        float32_t currentLinearVelocity  = 0.0;

        float32_t speed_angular = 0.0;
        float32_t speed_linear  = 0.0;
        float32_t speed_left  = 0.0;
        float32_t speed_right = 0.0;

        if(this->enable == true)
        {
            // Get current velocities
            currentAngularVelocity = odometry->GetAngularVelocity();
            currentLinearVelocity  = odometry->GetLinearVelocity();

			// Set setpoint velocities
        	if( xSemaphoreTake( this->xMutex, ( TickType_t ) 4 ) == pdTRUE )
        	{
				this->pid_angular.SetSetpoint(this->angularVelocity);
				this->pid_linear.SetSetpoint(this->linearVelocity);
				xSemaphoreGive(this->xMutex);
			}

            // Compute PID
            speed_angular = this->pid_angular.Get(currentAngularVelocity);
            speed_linear  = this->pid_linear.Get(currentLinearVelocity);

            // Angular/Linear to Left/Right
            speed_left  = speed_linear - speed_angular;
            speed_right = speed_linear + speed_angular;

            this->angularSpeed  = speed_angular;
            this->linearSpeed   = speed_linear;

            // Set speed to Motors
            this->leftSpeed  = -speed_left;
            this->rightSpeed =  speed_right;

            if(this->stop != true)
            {
                leftMotor->SetMotorSpeed(this->leftSpeed);
                rightMotor->SetMotorSpeed(this->rightSpeed);
            }
        }
    }

    void VelocityControl::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(VC_TASK_PERIOD_MS);

        VelocityControl* instance = _velocityControl;
        TickType_t prevTick = 0u,  tick = 0u;

        float32_t period = 0.0f;

        // 1. Initialise periodical task
        xLastWakeTime = xTaskGetTickCount();

        // 2. Get tick count
        prevTick = xTaskGetTickCount();

        while(1)
        {
            // 2. Wait until period elapse
            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            // 3. Get tick
            tick = xTaskGetTickCount();

            period = static_cast<float32_t>(tick) -
                     static_cast<float32_t>(prevTick);

            //4. Compute velocity (VelocityControl)
            instance->Compute(period);

            // 5. Set previous tick
            prevTick = tick;
        }
    }
}
