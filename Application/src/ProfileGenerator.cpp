/**
 * @file    ProfileControl.cpp
 * @author  Jeremy ROULLAND
 * @date    5 feb. 2017
 * @brief   ProfileGenerator class
 */

#include "ProfileGenerator.hpp"
#include "common.h"

#include <stdio.h>

using namespace MotionControl;
using namespace Location;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

//#define ANGULAR_VEL_MAX             (1.57f)
//#define ANGULAR_VEL_MAX             (3.14f)
#define ANGULAR_VEL_MAX             (12.0f)
//#define ANGULAR_ACC_MAX             (3.14f)
#define ANGULAR_ACC_MAX             (18.0f)
#define ANGULAR_PROFILE             (MotionProfile::PROFILE::POLY5)

//#define LINEAR_VEL_MAX              (0.2f)
#define LINEAR_VEL_MAX              (0.4f)
#define LINEAR_ACC_MAX              (1.0f)
#define LINEAR_PROFILE              (MotionProfile::PROFILE::POLY5)

#define PG_TASK_STACK_SIZE          (512u)
#define PG_TASK_PRIORITY            (configMAX_PRIORITIES-5)

#define PG_TASK_PERIOD_MS           (10u)

/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static MotionControl::ProfileGenerator* _profileGenerator = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

static PG_DEF _getDefStructure (enum ProfileGenerator::ID id)
{
    PG_DEF def;

    assert(id < ProfileGenerator::PROFILE_MAX);

    switch(id)
    {
        case ProfileGenerator::ANGULAR:
            def.Profile_Angular.maxVel   =    ANGULAR_VEL_MAX;
            def.Profile_Angular.maxAcc   =    ANGULAR_ACC_MAX;
            def.Profile_Angular.profile  =    ANGULAR_PROFILE;
            break;

        case ProfileGenerator::LINEAR:
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
    ProfileGenerator* ProfileGenerator::GetInstance(bool standalone)
    {
        // If PositionControl instance already exists
        if(_profileGenerator != NULL)
        {
            return _profileGenerator;
        }
        else
        {
            _profileGenerator = new ProfileGenerator(standalone);
            return _profileGenerator;
        }
    }

    /**
     * @brief  Profile generator constructor
     */
    ProfileGenerator::ProfileGenerator(bool standalone)
    {
        float32_t currentAngularPosition = 0.0;
        float32_t currentLinearPosition  = 0.0;

        this->name = "ProfileGenerator";
        this->taskHandle = NULL;

        this->Finished = false;

        // Init Angular motion profile generator
        this->def = _getDefStructure(ProfileGenerator::ANGULAR);
        this->angularProfile = MotionProfile(this->def.Profile_Angular.maxVel,
                                             this->def.Profile_Angular.maxAcc,
                                             this->def.Profile_Angular.profile);

        // Init Linear motion profile generator
        this->def = _getDefStructure(ProfileGenerator::LINEAR);
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

        this->angularProfile.SetSetPoint(currentAngularPosition,
                                         currentAngularPosition,
                                         this->getTime());
        this->linearProfile.SetSetPoint(currentLinearPosition,
                                        currentLinearPosition,
                                        this->getTime());

        this->angularPhaseProfile = Zero;
        this->linearPhaseProfile  = Zero;

        if(standalone)
        {
            // Create task
            xTaskCreate((TaskFunction_t)(&ProfileGenerator::taskHandler),
                        this->name.c_str(),
                        PG_TASK_STACK_SIZE,
                        NULL,
                        PG_TASK_PRIORITY,
                        NULL);
        }
    }

    /**
     * @brief Start angular position setpoint
     */
    void ProfileGenerator::StartAngularPosition(float32_t position)
    {
        float32_t currentAngularPosition = 0.0;
        float32_t time = 0.0;

        // Set angular position order
        this->angularPosition = position;

        // Get current position and time
        currentAngularPosition = odometry->GetAngularPosition();
        time = getTime();

        // Set profile type
        this->angularPhaseProfile = AccDec;
        this->angularProfile.SetProfile(MotionProfile::POLY5);

        // Start profile
        this->angularProfile.SetSetPoint(this->angularPosition, currentAngularPosition, time);
    }

    /**
     * @brief start linear position setpoint
     */
    void ProfileGenerator::StartLinearPosition(float32_t position)
    {
        float32_t currentLinearPosition  = 0.0;
        float32_t time = 0.0;
        float32_t d = 0.0;

        // Set linear position order
        this->linearPosition = position;

        // Get current position and time
        currentLinearPosition  = odometry->GetLinearPosition();
        time = getTime();

        // Calculate min distance
        d = this->linearProfile.GetMinDist(MotionProfile::POLY5_P1);

        // Choose profile type
        if(abs(this->linearPosition - currentLinearPosition) <= 2.0*d)
        {
            // Set profile type
            this->linearPhaseProfile = AccDec;
            this->linearProfile.SetProfile(MotionProfile::POLY5);

            // Start profile
            this->linearProfile.SetSetPoint(this->linearPosition, currentLinearPosition, time);
        }
        else
        {
            // Set profile type
            this->linearPhaseProfile = Acc;
            this->linearProfile.SetProfile(MotionProfile::POLY5_P1);

            // Start profile
            this->linearProfile.SetSetPoint(this->linearPositionProfiled+d, currentLinearPosition, time);
        }
    }


    /**
     * @brief  PositionControl compute
     */
    void ProfileGenerator::Compute(float32_t period)
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
    void ProfileGenerator::Generate(float32_t period)
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

        // #1 : Update setpoint
        //this->angularProfile.SetPoint(this->angularPosition);
        //this->linearProfile.SetPoint(this->linearPosition);

        // #2 : Compute Profile
        angularPositionProfiled = this->angularProfile.Get(time);
        linearPositionProfiled  = this->linearProfile.Get(time);

        // Set Positions profiled required to positions controller
        this->angularPositionProfiled = angularPositionProfiled;
        this->linearPositionProfiled  = linearPositionProfiled;

        // #3 : Profile to Profile
        // Angular profile
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
            	this->angularProfile.SetPoint(this->angularPosition);
                if(this->angularProfile.isFinished())
                {
                    this->angularPhaseProfile = Zero;
                }
                break;

            default:
                break;
        }

        // Linear profile
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
                    this->linearProfile.SetSetPoint(this->linearPosition-d, this->linearPositionProfiled, time);
                }
                break;

            case ConstVel:
                d = this->linearProfile.GetMinDist(MotionProfile::POLY5_P2);
                if(abs(this->linearPosition - this->linearPositionProfiled) <= d)
                {
                    this->linearPhaseProfile = Dec;
                    this->linearProfile.SetProfile(MotionProfile::POLY5_P2);
                    this->linearProfile.SetSetPoint(this->linearPosition, this->linearPositionProfiled, time);
                }
                else
                {
                    if(this->linearProfile.isFinished())
                    {
                        this->linearPhaseProfile = ConstVel;
                        this->linearProfile.SetProfile(MotionProfile::LINEAR);
                        this->linearProfile.SetSetPoint(this->linearPositionProfiled+1.0, this->linearPositionProfiled, time);
                    }
                }
                break;

            case Dec:
            	this->linearProfile.SetPoint(this->linearPosition);
                if(this->linearProfile.isFinished())
                {
                    this->linearPhaseProfile = Zero;
                }
                break;

            case AccDec:
            	this->linearProfile.SetPoint(this->linearPosition);
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

    void ProfileGenerator::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(PG_TASK_PERIOD_MS);

        ProfileGenerator* instance = _profileGenerator;
        TickType_t prevTick = 0u,  tick = 0u;

        float32_t period = 0.0f;

        // 1. Initialize periodical task
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

            //4. Compute profile (ProfileGenerator)
            instance->Compute(period);

            // 5. Set previous tick
            prevTick = tick;
        }
    }

    float32_t ProfileGenerator::getTime()
    {
        float32_t time = 0.0;

        time = static_cast<float32_t>(xTaskGetTickCount());
        time /= 1000.0;

        return  time;
    }


    float32_t ProfileGenerator::abs(float32_t val)
    {
        if(val < 0.0)
            val = -val;
        return val;
    }
}
