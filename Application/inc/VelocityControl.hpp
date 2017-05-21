/**
 * @file    VelocityControl.hpp
 * @author  Jeremy ROULLAND
 * @date    20 dec. 2016
 * @brief   VelocityControl class
 */

#ifndef INC_VELOCITYCONTROL_HPP_
#define INC_VELOCITYCONTROL_HPP_

#include "HAL.hpp"
#include "Utils.hpp"
#include "Odometry.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/
typedef struct
{
    // Motor
    struct
    {
        HAL::BrushlessMotorDriver::ID ID_left;
        HAL::BrushlessMotorDriver::ID ID_right;
    }Motors;

    // PID
    struct vc_pid
    {
        float32_t    kp;
        float32_t    ki;
        float32_t    kd;
    }PID_Angular;

    struct vc_pid PID_Linear;

}VC_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @namespace MotionControl
 */
namespace MotionControl
{
    /**
     * @class VelocityControl
     * @brief Velocity control class
     *
     * HOWTO :
     * -
     *
     */
    class VelocityControl
    {
    public:

        /**
         * @brief Velocity Identifier List
         */
        enum ID
        {
            ANGULAR = 0,    //!< ANGULAR
            LINEAR,         //!< LINEAR
            VELOCITY_MAX    //!< VELOCITY_MAX
        };

        /**
         * @brief Get instance method
         * @return VelocityLoop instance
         * @return BrushlessMotor instance
         */
        static VelocityControl* GetInstance(bool standalone = true);

        /**
         * @brief Return instance name
         */
        std::string Name()
        {
            return this->name;
        }

        /**
         * @brief Set angular velocity setpoint
         */
        void SetAngularVelocity(float32_t velocity)
        {
        	if( xSemaphoreTake( this->xMutex, ( TickType_t ) 4 ) == pdTRUE )
        	{
				this->angularVelocity = velocity;
				xSemaphoreGive(this->xMutex);
        	}
        }

        /**
         * @brief Get angular velocity setpoint
         */
        float32_t GetAngularVelocity()
        {
        	float32_t r = 0.0;
        	if( xSemaphoreTake( this->xMutex, ( TickType_t ) 4 ) == pdTRUE )
        	{
				r = this->angularVelocity;
				xSemaphoreGive(this->xMutex);
        	}
            return r;
        }

        /**
         * @brief Set linear velocity setpoint
         */
        void SetLinearVelocity(float32_t velocity)
        {
            this->linearVelocity = velocity;
        }

        /**
         * @brief Get linear velocity setpoint
         */
        float32_t GetLinearVelocity()
        {
            return this->linearVelocity;
        }

        /**
         * @brief Get angular speed
         */
        float32_t GetAngularSpeed()
        {
            return this->angularSpeed;
        }

        /**
         * @brief Get linear speed
         */
        float32_t GetLinearSpeed()
        {
            return this->linearSpeed;
        }

        /**
         * @brief Get left speed
         */
        float32_t GetLeftSpeed()
        {
            return this->leftSpeed;
        }

        /**
         * @brief Get right speed
         */
        float32_t GetRightSpeed()
        {
            return this->rightSpeed;
        }

        /**
         * @brief Set Angular Kp
         */
        void SetAngularKp(float32_t Kp)
        {
            this->def.PID_Angular.kp = Kp;
            this->pid_angular.SetKp(Kp);
        }

        /**
         * @brief Set Angular Ki
         */
        void SetAngularKi(float32_t Ki)
        {
            this->def.PID_Angular.ki = Ki;
            this->pid_angular.SetKi(Ki);
        }

        /**
         * @brief Set Linear Kp
         */
        void SetLinearKp(float32_t Kp)
        {
            this->def.PID_Linear.kp = Kp;
            this->pid_linear.SetKp(Kp);
        }

        /**
         * @brief Set Linear Ki
         */
        void SetLinearKi(float32_t Ki)
        {
            this->def.PID_Linear.ki = Ki;
            this->pid_linear.SetKi(Ki);
        }


        void Brake()
        {
            leftMotor->Brake();
            rightMotor->Brake();
        }

        void Stop()
        {
            this->stop = true;
            leftMotor->Brake();
            rightMotor->Brake();
        }

        void Start()
        {
            pid_angular.Reset();
            pid_linear.Reset();

            this->stop = false;
            leftMotor->Move();
            rightMotor->Move();
        }

        void Enable()
        {
            if(this->enable == false)
            {
                //pid_angular.Reset();
                //pid_linear.Reset();

                leftMotor->Move();
                rightMotor->Move();

                this->enable = true;
            }
        }

        void Disable()
        {
            if(this->enable == true)
            {
                leftMotor->Freewheel();
                rightMotor->Freewheel();

                this->enable = false;
            }
        }

        /**
         * @brief Compute robot velocity
         */
        void Compute(float32_t period);

    protected:

        /**
         * @protected
         * @brief Private constructor
         */
        VelocityControl(bool standalone);

        /**
         * @protected
         * @brief Instance name
         */
        std::string name;

        /**
         * @protected
         * @brief angular velocity PID controller
         */
        Utils::PID    pid_angular;

        /**
         * @protected
         * @brief linear velocity PID controller
         */
        Utils::PID    pid_linear;

        /**
         * @protected
         * @brief Coef definitions
         */
        VC_DEF def;

        /**
         * @protected
         * @brief Odometry instance
         */
        Location::Odometry* odometry;

        /**
         * @protected
         * @brief Left motor brushless driver instance
         */
        HAL::BrushlessMotorDriver* leftMotor;

        /**
         * @protected
         * @brief Right motor brushless driver instance
         */
        HAL::BrushlessMotorDriver* rightMotor;

        /**
         * @protected
         * @brief Angular velocity setpoint
         */
        float32_t angularVelocity;

        /**
         * @protected
         * @brief Linear velocity setpoint
         */
        float32_t linearVelocity;

        /**
         * @protected
         * @brief angular speed
         */
        float32_t angularSpeed;

        /**
         * @protected
         * @brief right speed
         */
        float32_t linearSpeed;

        /**
         * @protected
         * @brief left speed
         */
        float32_t leftSpeed;

        /**
         * @protected
         * @brief right speed
         */
        float32_t rightSpeed;


        /**
         * @protected
         * @brief enable/disable
         */
        bool enable;

        /**
         * @protected
         * @brief emergency stop
         */
        bool stop;

        /**
         * @protected
         * @brief OS Task handle
         *
         * Used by speed control loop
         */
        TaskHandle_t taskHandle;

        /**
         * @protected
         * @brief Mutex on variable data
         */
        SemaphoreHandle_t xMutex;

        /**
         * @protected
         * @brief Speed control loop task handler
         * @param obj : Always NULL
         */
        void taskHandler (void* obj);
    };
}

#endif /* INC_VELOCITYCONTROL_HPP_ */
