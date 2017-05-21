/**
 * @file    ProfileControl.hpp
 * @author  Jeremy ROULLAND
 * @date    14 jan. 2017
 * @brief   ProfileControl class
 */

#ifndef INC_PROFILECONTROL_HPP_
#define INC_PROFILECONTROL_HPP_

#include "HAL.hpp"
#include "Utils.hpp"
#include "Odometry.hpp"
#include "PositionControl.hpp"
#include "MotionProfile.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

using namespace MotionControl;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/
typedef struct
{
    // VelocityControlers
    struct
    {
        PositionControl::ID ID_Angular;
        PositionControl::ID ID_Linear;
    }Position;

    // Profile generator
    struct pc_profile
    {
        float32_t    maxVel;
        float32_t    maxAcc;
        enum MotionProfile::PROFILE profile;
    }Profile_Angular;

    struct pc_profile Profile_Linear;
}PfC_DEF;


/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @namespace MotionControl
 */
namespace MotionControl
{
    /**
     * @class ProfileControl
     * @brief Profile control class
     *
     * HOWTO :
     * -
     *
     */
    class ProfileControl
    {
    public:

        /**
         * @brief Profile Identifier List
         */
        enum ID
        {
            ANGULAR = 0,    //!< ANGULAR
            LINEAR,         //!< LINEAR
            PROFILE_MAX     //!< PROFILE_MAX
        };

        /**
         * @brief Profile Phase List
         */
        enum Phase
        {
            Zero = 0,    //!< Initial state
            Acc,	     //!< Acc
            ConstVel,    //!< ConstVel
            Dec,         //!< Dec
            AccDec,      //!< AccDec
            Phase_MAX    //!< Phase_max
        };

        /**
         * @brief Get instance method
         * @return Profile loop instance
         */
        static ProfileControl* GetInstance(bool standalone = true);

        /**
         * @brief Return instance name
         */
        std::string Name()
        {
            return this->name;
        }

        /**
         * @brief Get linear position setpoint
         */
        float32_t GetLinearPosition()
        {
            return this->linearPosition;
        }

        /**
         * @brief Set linear position setpoint
         */
        void SetLinearPosition(float32_t position)
        {
            // Set linear position order
            this->linearPosition = position;
        }

        /**
         * @brief Get angular position setpoint
         */
        float32_t GetAngularPosition()
        {
            return this->angularPosition;
        }

        /**
         * @brief Set angular position setpoint
         */
        void SetAngularPosition(float32_t position)
        {
            // Set angular position order
            this->angularPosition = position;
        }

        /**
         * @brief Set end position setpoint
         */
        void SetEnd(bool last)
        {
        	this->last = last;
        }

        /**
         * @brief Set angular position setpoint
         */
        void SetAngularLastPosition(bool last)
        {
        	this->angularLastPosition = last;
        }

        /**
         * @brief Set angular position setpoint
         */
        void SetLinearLastPosition(bool last)
        {
        	this->linearLastPosition = last;
        }

        /**
         * @brief Set Angular maximum velocity
         */
        void SetAngularMaxVel(float32_t maxVel)
        {
            this->def.Profile_Angular.maxVel = maxVel;
            this->angularProfile.SetVelMax(maxVel);
        }

        /**
         * @brief Set Angular maximum acceleration
         */
        void SetAngularMaxAcc(float32_t maxAcc)
        {
            this->def.Profile_Angular.maxAcc = maxAcc;
            this->angularProfile.SetAccMax(maxAcc);
        }

        /**
         * @brief Set Angular profile
         */
        void SetAngularProfile(enum MotionProfile::PROFILE profile)
        {
            this->def.Profile_Angular.profile = profile;
            this->angularProfile.SetProfile(profile);
        }

        /**
         * @brief Set Linear maximum velocity
         */
        void SetLinearMaxVel(float32_t maxVel)
        {
            this->def.Profile_Linear.maxVel = maxVel;
            this->linearProfile.SetVelMax(maxVel);
        }

        /**
         * @brief Set Linear maximum acceleration
         */
        void SetLinearMaxAcc(float32_t maxAcc)
        {
            this->def.Profile_Linear.maxAcc = maxAcc;
            this->linearProfile.SetAccMax(maxAcc);
        }

        /**
         * @brief Set Linear profile
         */
        void SetLinearProfile(enum MotionProfile::PROFILE profile)
        {
            this->def.Profile_Linear.profile = profile;
            this->linearProfile.SetProfile(profile);
        }

        /**
         * @brief Compute profile control
         */
        void Compute(float32_t period);

        /**
         * @brief Generate profile
         */
        void Generate(float32_t period);

        /**
         * @brief is angular and linear positioning finished
         */
        bool isPositioningFinished()
        {
        	return this->Finished;
        }

        /**
         * @brief is Near
         */
        bool isLinearPositioningNear()
        {
        	return this->Near;
        }

        /**
         * @brief is angular positioning finished
         */
        bool isAngularPositioningFinished()
        {
        	return this->angularProfile.isFinished();
        }

        /**
         * @brief is linear positioning finished
         */
        bool isLinearPositioningFinished()
        {
        	return this->linearProfile.isFinished();
        }

        /**
         * @brief get phase profile
         */
        uint32_t GetAngularPhase()
        {
        	return (uint32_t)this->angularPhaseProfile;
        }

        /**
         * @brief get phase profile
         */
        uint32_t GetLinearPhase()
        {
        	return (uint32_t)this->linearPhaseProfile;
        }

    protected:

        /**
         * @protected
         * @brief Private constructor
         */
        ProfileControl(bool standalone);

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
         * @brief angular profile generator
         */
        MotionProfile angularProfile;

        /**
         * @protected
         * @brief linear profile generator
         */
        MotionProfile linearProfile;

        /**
         * @protected
         * @brief Coef definitions
         */
        PfC_DEF def;

        /**
         * @protected
         * @brief Odometry instance
         */
        Location::Odometry* odometry;

        /**
         * @protected
         * @brief PositionControl instance
         */
        PositionControl* positionControl;

        /**
         * @protected
         * @brief Profile finished
         */
        bool Finished;

        /**
         * @protected
         * @brief Profile linear finished
         */
        bool linearFinished;

        /**
         * @protected
         * @brief Profile near
         */
        bool Near;

        /**
         * @protected
         * @brief Angular position required
         */
        float32_t angularPosition;

        /**
         * @protected
         * @brief Linear position required
         */
        float32_t linearPosition;

        /**
         * @protected
         * @brief This is the last position. End must be precise.
         */
        bool last;

        /**
         * @protected
         * @brief Angular last position
         */
        bool angularLastPosition;

        /**
         * @protected
         * @brief Linear last required
         */
        bool linearLastPosition;


        /**
         * @protected
         * @brief Angular position setpoint
         */
        float32_t setpointAngularPosition;

        /**
         * @protected
         * @brief Linear position setpoint
         */
        float32_t setpointLinearPosition;

        /**
         * @protected
         * @brief Angular position profiled
         */
        float32_t angularPositionProfiled;

        /**
         * @protected
         * @brief Linear position profiled
         */
        float32_t linearPositionProfiled;

        /**
         * @protected
         * @brief Angular phase of the profile
         */
        enum Phase angularPhaseProfile;

        /**
         * @protected
         * @brief Linear phase of the profile
         */
        enum Phase linearPhaseProfile;

        /**
         * @protected
         * @brief OS Task handle
         *
         * Used by position control loop
         */
        TaskHandle_t taskHandle;

        /**
         * @protected
         * @brief Position control loop task handler
         * @param obj : Always NULL
         */
        void taskHandler (void* obj);

        /**
         * @protected
         * @brief get absolute value
         */
        float32_t abs(float32_t val);

    };
}
#endif /* INC_PROFILECONTROL_HPP_ */
