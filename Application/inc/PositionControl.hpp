/**
 * @file    PositionControl.hpp
 * @author  Jeremy ROULLAND
 * @date    24 dec. 2016
 * @brief   PositionControl class
 */

#ifndef INC_POSITIONCONTROL_HPP_
#define INC_POSITIONCONTROL_HPP_

#include "HAL.hpp"
#include "Utils.hpp"
#include "Odometry.hpp"
#include "VelocityControl.hpp"

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
    // VelocityControlers
    struct
    {
        MotionControl::VelocityControl::ID ID_Angular;
        MotionControl::VelocityControl::ID ID_Linear;
    }Velocity;

    // PID
    struct pc_pid
    {
        float32_t    kp;
        float32_t    ki;
        float32_t    kd;
    }PID_Angular;

    struct pc_pid PID_Linear;

}PC_DEF;

/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @namespace MotionControl
 */
namespace MotionControl
{
    /**
     * @class PositionControl
     * @brief Velocity control class
     *
     * HOWTO :
     * -
     *
     */
    class PositionControl
    {
    public:

        /**
         * @brief Position Identifier List
         */
        enum ID
        {
            ANGULAR = 0,    //!< ANGULAR
            LINEAR,         //!< LINEAR
            POSITION_MAX    //!< POISTION_MAX
        };

        /**
         * @brief Get instance method
         * @return Position loop instance
         */
        static PositionControl* GetInstance(bool standalone = true);

        /**
         * @brief Return instance name
         */
        std::string Name()
        {
            return this->name;
        }

        /**
         * @brief Set angular position setpoint
         */
        void SetAngularPosition(float32_t position)
        {
        	if( xSemaphoreTake( this->xMutex, ( TickType_t ) 4 ) == pdTRUE )
        	{
				// Set angular position order
				this->angularPosition = position;

				xSemaphoreGive(this->xMutex);
        	}

            // Disable brake emergency
            /*if(this->stop == true)
            {
                this->velocityControl->Start();
                this->stop = false;
            }*/
        }

        /**
         * @brief Get angular position setpoint
         */
        float32_t GetAngularPosition()
        {
        	float32_t r = 0.0;
        	if( xSemaphoreTake( this->xMutex, ( TickType_t ) 4 ) == pdTRUE )
        	{
				r = this->angularPosition;

				xSemaphoreGive(this->xMutex);
			}
            return r;
        }

        /**
         * @brief Set linear position setpoint
         */
        void SetLinearPosition(float32_t position)
        {
            // Set linear position order
            this->linearPosition = position;

            // Disable brake emergency
            /*if(this->stop == true)
            {
                this->velocityControl->Start();
                this->stop = false;
            }*/
        }

        /**
         * @brief Get linear position setpoint
         */
        float32_t GetLinearPosition()
        {
            return this->linearPosition;
        }

        /**
         * @brief Get angular velocity requiered
         */
        float32_t GetAngularVelocity()
        {
            return this->angularVelocity;
        }

        /**
         * @brief Get linear velocity requiered
         */
        float32_t GetLinearVelocity()
        {
            return this->linearVelocity;
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
         * @brief Set Angular Kd
         */
        void SetAngularKd(float32_t Kd)
        {
            this->def.PID_Angular.kd = Kd;
            this->pid_angular.SetKd(Kd);
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

        /**
         * @brief Set Linear Kd
         */
        void SetLinearKd(float32_t Kd)
        {
            this->def.PID_Linear.kd = Kd;
            this->pid_linear.SetKd(Kd);
        }

        /**
         * @brief Enable
         */
        void Enable()
        {
            if(this->enable == false)
            {
                this->enable = true;
            }
        }

        /**
         * @brief Disable
         */
        void Disable()
        {
            if(this->enable == true)
            {
                this->enable = false;
            }
        }

        /**
         * @brief Stop
         */
        void Stop()
        {
            this->velocityControl->Brake();
            this->stop = true;
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
        PositionControl(bool standalone);

        /**
         * @protected
         * @brief get time in seconds
         */
        float32_t getTime();

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
        PC_DEF def;

        /**
         * @protected
         * @brief Odometry instance
         */
        Location::Odometry* odometry;

        /**
         * @protected
         * @brief VelocityControl instance
         */
        VelocityControl* velocityControl;

        /**
         * @protected
         * @brief Angular position setpoint
         */
        float32_t angularPosition;

        /**
         * @protected
         * @brief Linear position setpoint
         */
        float32_t linearPosition;

        /**
         * @protected
         * @brief angular velocity required
         */
        float32_t angularVelocity;

        /**
         * @protected
         * @brief linear velocity required
         */
        float32_t linearVelocity;

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
         * Used by position control loop
         */
        TaskHandle_t taskHandle;


        /**
         * @protected
         * @brief Mutex on variable data
         */
        SemaphoreHandle_t xMutex;

        /**
         * @protected
         * @brief Position control loop task handler
         * @param obj : Always NULL
         */
        void taskHandler (void* obj);
    };
}

#endif /* INC_POSITION */
