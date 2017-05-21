/**
 * @file    PositionControl.cpp
 * @author  Jeremy ROULLAND
 * @date    25 dec. 2016
 * @brief   PositionControl class
 *
 *
 */

#include "PositionControl.hpp"
#include "common.h"

#include <stdio.h>

using namespace HAL;
using namespace Utils;
using namespace MotionControl;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define ANGULAR_POSITION_PID_KP     (31.2f)
#define ANGULAR_POSITION_PID_KI     (0.0f)
#define ANGULAR_POSITION_PID_KD     (0.0f)

#define LINEAR_POSITION_PID_KP      (42.75f)
#define LINEAR_POSITION_PID_KI      (0.0f)
#define LINEAR_POSITION_PID_KD      (0.0f)

#define PC_ANGULAR_VELOCITY         (VelocityControl::ID::ANGULAR)
#define PC_LINEAR_VELOCITY          (VelocityControl::ID::ANGULAR)

#define PC_TASK_STACK_SIZE          (512u)
#define PC_TASK_PRIORITY            (configMAX_PRIORITIES-4)

#define PC_TASK_PERIOD_MS           (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::PositionControl* _positionControl = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static PC_DEF _getDefStructure (enum PositionControl::ID id)
{
    PC_DEF def;

    assert(id < PositionControl::POSITION_MAX);

    switch(id)
    {
        case PositionControl::ANGULAR:
            def.Velocity.ID_Angular      =    PC_ANGULAR_VELOCITY;
            def.Velocity.ID_Linear       =    PC_LINEAR_VELOCITY;
            def.PID_Angular.kp           =    ANGULAR_POSITION_PID_KP;
            def.PID_Angular.ki           =    ANGULAR_POSITION_PID_KI;
            def.PID_Angular.kd           =    ANGULAR_POSITION_PID_KD;
            break;

        case PositionControl::LINEAR:
            def.Velocity.ID_Angular      =    PC_ANGULAR_VELOCITY;
            def.Velocity.ID_Linear       =    PC_LINEAR_VELOCITY;
            def.PID_Linear.kp            =    LINEAR_POSITION_PID_KP;
            def.PID_Linear.ki            =    LINEAR_POSITION_PID_KI;
            def.PID_Linear.kd            =    LINEAR_POSITION_PID_KD;
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

    PositionControl* PositionControl::GetInstance(bool standalone)
    {
        // If PositionControl instance already exists
        if(_positionControl != NULL)
        {
            return _positionControl;
        }
        else
        {
            _positionControl = new PositionControl(standalone);
            return _positionControl;
        }
    }

    /**
     * @brief  PositionControl constructor
     */
    PositionControl::PositionControl(bool standalone)
    {
        float32_t currentAngularPosition = 0.0;
        float32_t currentLinearPosition  = 0.0;

        this->name = "PositionControl";
        this->taskHandle = NULL;

        // Init Angular velocity control
        this->def = _getDefStructure(PositionControl::ANGULAR);
        this->pid_angular = PID(this->def.PID_Angular.kp,
                                this->def.PID_Angular.ki,
                                this->def.PID_Angular.kd,
                                PC_TASK_PERIOD_MS/1000.0);

        // Init Linear velocity control
        this->def = _getDefStructure(PositionControl::LINEAR);
        this->pid_linear  = PID(this->def.PID_Linear.kp,
                                this->def.PID_Linear.ki,
                                this->def.PID_Linear.kd,
                                PC_TASK_PERIOD_MS/1000.0);

        this->odometry = Odometry::GetInstance();

        this->velocityControl  = VelocityControl::GetInstance();


        // Get current positions
        currentAngularPosition = odometry->GetAngularPosition();
        currentLinearPosition  = odometry->GetLinearPosition();

        this->angularPosition = currentAngularPosition;
        this->linearPosition  = currentLinearPosition;

        this->angularVelocity = 0.0f;
        this->linearVelocity  = 0.0f;

        this->enable = true;

        if(standalone)
        {
            // Create task
            xTaskCreate((TaskFunction_t)(&PositionControl::taskHandler),
                        this->name.c_str(),
                        PC_TASK_STACK_SIZE,
                        NULL,
                        PC_TASK_PRIORITY,
                        NULL);
        }

        this->xMutex = NULL;
        this->xMutex = xSemaphoreCreateMutex();
    }

    /**
     * @brief  PositionControl compute
     */
    void PositionControl::Compute(float32_t period)
    {
        float32_t currentAngularPosition = 0.0;
        float32_t currentLinearPosition  = 0.0;

        float32_t angularVelocity = 0.0;
        float32_t linearVelocity  = 0.0;

        if(this->enable == true)
        {
            // Get current positions
            currentAngularPosition = odometry->GetAngularPosition();
            currentLinearPosition  = odometry->GetLinearPosition();

            if(xSemaphoreTake( this->xMutex, ( TickType_t ) 4 ) == pdTRUE)
            {
				this->pid_angular.SetSetpoint(this->angularPosition);
				this->pid_linear.SetSetpoint(this->linearPosition);
				xSemaphoreGive(this->xMutex);
            }

            // Compute PID
            angularVelocity = this->pid_angular.Get(currentAngularPosition);
            linearVelocity  = this->pid_linear.Get(currentLinearPosition);

            // Set Velocities requiered to velocity controller
            this->angularVelocity = angularVelocity;
            this->linearVelocity  = linearVelocity;

            velocityControl->SetAngularVelocity(this->angularVelocity);
            velocityControl->SetLinearVelocity(this->linearVelocity);
        }
    }

    void PositionControl::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(PC_TASK_PERIOD_MS);

        PositionControl* instance = _positionControl;
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

    float32_t PositionControl::getTime()
    {
        float32_t time = 0.0;

        time = static_cast<float32_t>(xTaskGetTickCount());
        time /= 1000.0;

        return  time;
    }
}
