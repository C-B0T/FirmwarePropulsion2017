/**
 * @file    MotionProfile.hpp
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Motion profile (Trapezoidal, S-Curve, ...)
 */

#ifndef INC_MOTIONPROFILE_HPP_
#define INC_MOTIONPROFILE_HPP_

#include "common.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

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
     * @brief Provides a motion profile generator
     *
     * HOWTO :
     * -
     *
     */
    class MotionProfile
    {
    public:

        /**
         * @brief Profile type List
         */
        enum PROFILE
        {
            NONE = 0,
            LINEAR,
            TRIANGLE,
            TRAPEZ,
            SCURVE,
            POLY3,
            POLY5,
            POLY5_P1,
            POLY5_P2,
            AUTO,
            MPROFILE_MAX
        };

        /**
         * @brief MotionProfile Mode List
         */
        enum MODE
        {
            MODE_MANUAL,
            MODE_AUTO,
            MODE_MODEMAX
        };

        /**
         * @brief Constructor
         */
        MotionProfile(float32_t maxVel = 1.0, float32_t maxAcc = 1.0, enum MotionProfile::PROFILE profile = POLY5);

        /**
         * @brief Destructor
         */
        ~MotionProfile();

        /**
         * @brief Set setpoint
         */
         void SetPoint(float32_t point);

        /**
         * @brief Set setpoint
         */
         void SetSetPoint(float32_t point, float32_t currentPoint, float32_t currentTime);

        /**
         * @brief set maximum velocity
         */
        void SetVelMax(float32_t maxVel)
        {
            this->maxVel = maxVel;
        }

        /**
         * @brief set maximum acceleration
         */
        void SetAccMax(float32_t maxAcc)
        {
            this->maxAcc = maxAcc;
        }

        /**
         * @brief set profile used
         */
         void SetProfile(enum MotionProfile::PROFILE profile)
        {
            this->profile = profile;
        }

         /**
          * @brief set mode used
          */
          void SetMode(enum MotionProfile::MODE mode)
         {
             this->mode = mode;
         }

        /**
        * @brief get tf used
        */
        float32_t GetTf()
        {
            return this->tf;
        }

        /**
         * @brief get setpoint profiled
         */
        float32_t Get(float32_t time);

        /**
         * @brief get setpoint profiled in tf
         */
        float32_t Get(float32_t time, float32_t tf);

        /**
         * @brief get min time
         */
        float32_t GetMinTime(enum MotionProfile::PROFILE profile)
        {
        	return this->calculateMinTime(profile);
        }

        /**
         * @brief get setpoint distance
         */
        float32_t GetMinDist(enum MotionProfile::PROFILE profile)
        {
        	return this->calculateMinDist(profile);
        }

        /**
         * @brief is profile finished
         */
        bool isFinished();

    protected:
        /**
         * @protected
         * @brief finished flag
         */
        bool finished;

        /**
         * @protected
         * @brief start time
         */
        float32_t startTime;

        /**
         * @protected
         * @brief start point
         */
        float32_t startPoint;

        /**
         * @protected
         * @brief calculate the minimum time
         */
        float32_t calculateMinTime(enum MotionProfile::PROFILE profile = AUTO);

        /**
         * @protected
         * @brief calculate the minimum distance
         */
        float32_t calculateMinDist(enum MotionProfile::PROFILE profile = AUTO);

        /**
         * @protected
         * @brief calculate profile
         */
        float32_t calculateProfile(float32_t t);

        /**
         * @protected
         * @brief calculate without profile
         */
        float32_t calculateNoneProfile(float32_t t);

        /**
         * @protected
         * @brief calculate a linear profile
         */
        float32_t calculateLinearProfile(float32_t t);

        /**
         * @protected
         * @brief calculate a triangle profile
         */
        float32_t calculateTriangleProfile(float32_t t);

        /**
         * @protected
         * @brief calculate a custom trapezoidal profile
         */
        float32_t calculateCustomTrapezoidalProfile(float32_t t);

        /**
         * @protected
         * @brief calculate a standard trapezoidal profile
         */
        float32_t calculateTrapezoidalProfile(float32_t t);

        /**
         * @protected
         * @brief calculate a standard trapezoidal profile
         */
        float32_t calculateSCurveProfile(float32_t t);

        /**
         * @protected
         * @brief calculate a cubic profile
         */
        float32_t calculatePolynomial3Profile(float32_t t);

        /**
         * @protected
         * @brief calculate a quintic profile
         */
        float32_t calculatePolynomial5Profile(float32_t t);

        /**
         * @protected
         * @brief calculate quintic profile phase 1 only
         */
        float32_t calculatePolynomial5Phase1Profile(float32_t t);

        /**
         * @protected
         * @brief calculate quintic profile phase 2 only
         */
        float32_t calculatePolynomial5Phase2Profile(float32_t t);

        /**
         * @protected
         * @brief calculate the optimised profile
         */
        float32_t calculateAutoProfile(float32_t t);

        /**
         * @protected
         * @brief get absolute value from a float32_t
         */
        float32_t abs(float32_t val);

        /**
         * @protected
         * @brief profile type selected
         */
        enum PROFILE profile;

        /**
         * @protected
         * @brief Motion Profile mode
         */
        enum MODE mode;

        /**
         * @protected
         * @brief set point
         */
        float32_t setPoint;

        /**
         * @protected
         * @brief maximum velocity
         */
        float32_t maxVel;

        /**
         * @protected
         * @brief maximum acceleration
         */
        float32_t maxAcc;

        /**
         * @protected
         * @brief tf used by manual mode
         */
        float32_t tf;
    };
}

#endif /* INC_MOTIONPROFILE_HPP_ */
