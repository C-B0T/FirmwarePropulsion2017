/**
 * @file    ProfileControl.cpp
 * @author  Jeremy ROULLAND
 * @date    14 jan. 2017
 * @brief   ProfileControl class
 *
 *
 */

#include "ProfileControl.hpp"
#include "common.h"

#include <stdio.h>

using namespace MotionControl;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

//#define ANGULAR_VEL_MAX             (1.57f)
#define ANGULAR_VEL_MAX             (3.14f)
//#define ANGULAR_ACC_MAX             (3.14f)
#define ANGULAR_ACC_MAX             (6.28f)
#define ANGULAR_PROFILE             (MotionProfile::PROFILE::POLY5)

//#define LINEAR_VEL_MAX              (0.2f)
#define LINEAR_VEL_MAX              (0.4f)
#define LINEAR_ACC_MAX              (1.0f)
#define LINEAR_PROFILE              (MotionProfile::PROFILE::POLY5)

#define PfC_TASK_STACK_SIZE         (512u)
#define PfC_TASK_PRIORITY           (configMAX_PRIORITIES-5)

#define PfC_TASK_PERIOD_MS          (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::ProfileControl* _profileControl = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static PfC_DEF _getDefStructure (enum ProfileControl::ID id)
{
    PfC_DEF def;

    assert(id < ProfileControl::PROFILE_MAX);

    switch(id)
    {
        case ProfileControl::ANGULAR:
            def.Profile_Angular.maxVel   =    ANGULAR_VEL_MAX;
            def.Profile_Angular.maxAcc   =    ANGULAR_ACC_MAX;
            def.Profile_Angular.profile  =    ANGULAR_PROFILE;
            break;

        case ProfileControl::LINEAR:
            def.Profile_Linear.maxVel    =    LINEAR_VEL_MAX;
            def.Profile_Linear.maxAcc    =    LINEAR_ACC_MAX;
            def.Profile_Linear.profile   =    LINEAR_PROFILE;
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
    ProfileControl* ProfileControl::GetInstance(bool standalone)
    {
        // If PositionControl instance already exists
        if(_profileControl != NULL)
        {
            return _profileControl;
        }
        else
        {
            _profileControl = new ProfileControl(standalone);
            return _profileControl;
        }
    }

    /**
     * @brief  PositionControl constructor
     */
    ProfileControl::ProfileControl(bool standalone)
    {
        float32_t currentAngularPosition = 0.0;
        float32_t currentLinearPosition  = 0.0;

        this->name = "ProfileControl";
        this->taskHandle = NULL;

        this->Finished = false;
        this->linearFinished = false;
        this->Near = false;

        // Init Angular motion profile generator
        this->def = _getDefStructure(ProfileControl::ANGULAR);
        this->angularProfile = MotionProfile(this->def.Profile_Angular.maxVel,
                                             this->def.Profile_Angular.maxAcc,
                                             this->def.Profile_Angular.profile);

        // Init Linear motion profile generator
        this->def = _getDefStructure(ProfileControl::LINEAR);
        this->linearProfile = MotionProfile(this->def.Profile_Linear.maxVel,
                                            this->def.Profile_Linear.maxAcc,
                                            this->def.Profile_Linear.profile);


        // Get instances
        this->odometry = Odometry::GetInstance();
        this->positionControl = PositionControl::GetInstance();

        // Get current positions
        currentAngularPosition = odometry->GetAngularPosition();
        currentLinearPosition  = odometry->GetLinearPosition();

        // Initial values
        this->angularPosition = currentAngularPosition;
        this->linearPosition  = currentLinearPosition;
        this->angularPositionProfiled = currentAngularPosition;
        this->linearPositionProfiled  = currentLinearPosition;
        this->setpointAngularPosition = currentAngularPosition;
        this->setpointLinearPosition  = currentLinearPosition;

        this->angularProfile.SetSetPoint(currentAngularPosition,
                                         currentAngularPosition,
                                         this->getTime());
        this->linearProfile.SetSetPoint(currentLinearPosition,
                                        currentLinearPosition,
                                        this->getTime());


        this->angularPhaseProfile = Zero;
        this->linearPhaseProfile  = Zero;

        this->angularLastPosition = true;
        this->linearLastPosition  = true;

        this->last = true;


        if(standalone)
        {
            // Create task
            xTaskCreate((TaskFunction_t)(&ProfileControl::taskHandler),
                        this->name.c_str(),
                        PfC_TASK_STACK_SIZE,
                        NULL,
                        PfC_TASK_PRIORITY,
                        NULL);
        }
    }

    /**
     * @brief  PositionControl compute
     */
    void ProfileControl::Compute(float32_t period)
    {
        float32_t currentAngularPosition = 0.0;
        float32_t currentLinearPosition  = 0.0;

        float32_t time = getTime();

        // Get current positions
        currentAngularPosition = odometry->GetAngularPosition();
        currentLinearPosition  = odometry->GetLinearPosition();

        // Generate profile
        this->Generate(period);


        positionControl->SetAngularPosition(this->angularPositionProfiled);
        positionControl->SetLinearPosition(this->linearPositionProfiled);
    }

    /**
     * @brief  PositionControl generate
     */
    void ProfileControl::Generate(float32_t period)
    {
        float32_t currentAngularPosition = 0.0;
        float32_t currentLinearPosition  = 0.0;

        float32_t angularPositionProfiled = 0.0;
        float32_t linearPositionProfiled  = 0.0;

        float32_t time = getTime();

        float32_t d = 0.0;

        // Get current positions
        currentAngularPosition = odometry->GetAngularPosition();
        currentLinearPosition  = odometry->GetLinearPosition();

        // #1 : Update setpoint with new order
        // Angular
        if(this->angularPosition != this->setpointAngularPosition)
        {
        	if( (this->angularPhaseProfile != Acc) && (this->angularPhaseProfile != Dec) && (this->angularPhaseProfile != AccDec) )
        	{
				// Start profiling of the angular position
				this->setpointAngularPosition = this->angularPosition;

				this->angularPhaseProfile = AccDec;
				this->angularProfile.SetProfile(MotionProfile::POLY5);

				this->angularProfile.SetSetPoint(this->setpointAngularPosition, currentAngularPosition, time);
        	}
        }
        // Linear
        if(this->linearPosition != this->setpointLinearPosition)
        {
        	if( (this->linearPhaseProfile != Acc) && (this->linearPhaseProfile != Dec) && (this->linearPhaseProfile != AccDec) )
        	{
				// Start profiling of the linear position
				this->setpointLinearPosition = this->linearPosition;

				d = this->linearProfile.GetMinDist(MotionProfile::POLY5_P1);

    			if(this->linearPhaseProfile == Zero)
    			{
					if(abs(this->setpointLinearPosition - currentLinearPosition) <= 2.0*d)
					{
						this->linearPhaseProfile = AccDec;
						this->linearProfile.SetProfile(MotionProfile::POLY5);

						this->linearProfile.SetSetPoint(this->setpointLinearPosition, currentLinearPosition, time);
					}
					else
					{
							this->linearPhaseProfile = Acc;
							this->linearProfile.SetProfile(MotionProfile::POLY5_P1);
							this->linearProfile.SetSetPoint(this->linearPositionProfiled+d, currentLinearPosition, time);
					}
    			}
        	}
        }

        // #2 : Compute Profile
        angularPositionProfiled = this->angularProfile.Get(time);
        linearPositionProfiled  = this->linearProfile.Get(time);

        // Set Positions profiled required to positions controller
        this->angularPositionProfiled = angularPositionProfiled;
        this->linearPositionProfiled  = linearPositionProfiled;

        // #3 : Profile to Profile
        switch (this->angularPhaseProfile)
        {
			case Zero:
				break;

			case Acc:
				break;

			case ConstVel:
				break;

			case Dec:
				break;

			case AccDec:
				this->setpointAngularPosition = this->angularPosition;
				this->angularProfile.SetPoint(this->setpointAngularPosition);
				if(this->angularProfile.isFinished())
				{
					this->angularPhaseProfile = Zero;
				}
				break;

			default:
				break;
        }

        switch (this->linearPhaseProfile)
        {
			case Zero:
				break;

			case Acc:
				if(this->linearProfile.isFinished())
				{
					this->linearPhaseProfile = ConstVel;
					this->linearProfile.SetProfile(MotionProfile::LINEAR);

					d = this->linearProfile.GetMinDist(MotionProfile::POLY5_P2);

					// Start profiling of the linear position
					this->linearProfile.SetSetPoint(this->setpointLinearPosition-d, this->linearPositionProfiled, time);
				}
				break;

			case ConstVel:
				if(this->linearProfile.isFinished())
				{
					if(this->last)
					{
						d = this->linearProfile.GetMinDist(MotionProfile::POLY5_P2);

						if(abs(this->setpointLinearPosition - this->linearPositionProfiled) <= d)
						{
							this->linearPhaseProfile = Dec;
							this->linearProfile.SetProfile(MotionProfile::POLY5_P2);
							this->linearProfile.SetSetPoint(this->setpointLinearPosition, this->linearPositionProfiled, time);
						}
						else
						{
							this->linearPhaseProfile = ConstVel;
							this->linearProfile.SetProfile(MotionProfile::LINEAR);
							this->linearProfile.SetSetPoint(this->setpointLinearPosition-d, this->linearPositionProfiled, time);
						}
					}
					else
					{
						this->linearPhaseProfile = ConstVel;
						this->linearProfile.SetProfile(MotionProfile::LINEAR);

						d = 1.0;

						this->linearProfile.SetSetPoint(this->linearPositionProfiled+d, this->linearPositionProfiled, time);
					}
				}
				break;

			case Dec:
				if(this->linearProfile.isFinished())
				{
					this->linearPhaseProfile = Zero;
				}
				break;

			case AccDec:
				if(this->linearProfile.isFinished())
				{
					this->linearPhaseProfile = Zero;
				}
				break;

			default:
				break;
        }

        this->Finished = this->angularProfile.isFinished() && this->linearProfile.isFinished();
    }

    void ProfileControl::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(PfC_TASK_PERIOD_MS);

        ProfileControl* instance = _profileControl;
        TickType_t prevTick = 0u,  tick = 0u;

        float32_t period = 0.0f;
#include "GPIO.hpp"
        HAL::GPIO *led1 = HAL::GPIO::GetInstance(HAL::GPIO::GPIO6);

        // 1. Initialize periodical task
        xLastWakeTime = xTaskGetTickCount();

        // 2. Get tick count
        prevTick = xTaskGetTickCount();

        while(1)
        {
            // 2. Wait until period elapse
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            led1->Set(HAL::GPIO::Low);

            // 3. Get tick
            tick = xTaskGetTickCount();

            period = static_cast<float32_t>(tick) -
                     static_cast<float32_t>(prevTick);

            //4. Compute velocity (VelocityControl)
            instance->Compute(period);

            // 5. Set previous tick
            prevTick = tick;
            led1->Set(HAL::GPIO::High);
        }
    }

    float32_t ProfileControl::getTime()
    {
        float32_t time = 0.0;

        time = static_cast<float32_t>(xTaskGetTickCount());
        time /= 1000.0;

        return  time;
    }


    float32_t ProfileControl::abs(float32_t val)
    {
        if(val < 0.0)
            val = -val;
        return val;
    }
}
