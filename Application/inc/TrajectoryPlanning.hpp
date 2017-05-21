/**
 * @file    TrajectoryPlanning.hpp
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Trajectory planning (Linear, Curve, ...)
 */


#ifndef INC_TRAJECTORYPLANNING_H_
#define INC_TRAJECTORYPLANNING_H_

#include "common.h"

#include "Odometry.hpp"
#include "MotionControl.hpp"
//#include "ProfileControl.hpp"
#include "ProfileGenerator.hpp"

using namespace Location;
using namespace MotionControl;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define _PI_        3.14159265358979323846
#define _2_PI_      6.28318530717958647692  // 2*PI

/*----------------------------------------------------------------------------*/
/* Class declaration                                                          */
/*----------------------------------------------------------------------------*/

/**
 * @namespace MotionControl
 */
namespace MotionControl
{
    /**
     * @class TrajectoryPlanning
     * @brief Provides a trajectory generator
     *
     * HOWTO :
     * -
     *
     */
    class TrajectoryPlanning
    {
    public:
        TrajectoryPlanning();
        ~TrajectoryPlanning();

        //Orders:
        void goLinear(float32_t linear);     // linear in meters
        void goAngular(float32_t angular);   // angular in radian
        void stop();
        void gotoXY(float32_t X, float32_t Y);   // X,Y in meters
        void pushXY(float32_t X[], float32_t Y[], uint32_t n);   // X,Y in meters
        int32_t stallX(int32_t stallMode);       // stallMode allow to choose side to side contact (upTable to backBot, upTable to frontBot, downTable to backBot, downTable to frontBot)
        int32_t stallY(int32_t stallMode);       // stallMode allow to choose side to side contact (leftTable to backBot, leftTable to frontBot, rightTable to backBot, rightTable to frontBot)
        // others orders...

        float32_t update();
        float32_t getSuggestedLinearPosition();
        float32_t getSuggestedAngularPosition();

        uint32_t GetStep()
        {
        	return (uint32_t)this->step;
        }

    protected:
        enum state_t {FREE=0, LINEAR, ANGULAR, STOP, KEEP, LINEARPLAN, CURVEPLAN, STALLX, STALLY, DRAWPLAN};

        /**
         * @protected
         * @brief get absolute value from a float32_t
         */
        float32_t abs(float32_t val);

        /**
         * @protected
         * @brief get current time in second
         */
        float32_t getTime();

        void calculateFree();
        void calculateMove();
        void calculateGoLinear();
        void calculateGoAngular();
        void calculateStop();
        void calculateKeepPosition();
        void calculateDrawPlan();
        void calculateLinearPlan();
        void calculateCurvePlan();
        void calculateStallX(int32_t mode);
        void calculateStallY(int32_t mode);

        void updateXYtoLA(uint32_t n);

        int32_t state;
        int32_t step;

        float32_t linearSetPoint;
        float32_t linearNextSetPoint;
        float32_t angularSetPoint;

        int32_t stallMode;

        float32_t startTime;
        float32_t startLinearPosition;  //
        float32_t startAngularPosition; //

        float32_t endLinearPosition;     // Linear Position Target
        float32_t endAngularPosition;    // Linear Angular Target

        float32_t linearSign;
        float32_t angularSign;

        float32_t suggestedLinearPosition;
        float32_t suggestedAngularPosition;

        float32_t X[10];
        float32_t Y[10];
        uint32_t  XYn;

        Odometry *odometry;
        PositionControl *position;
        VelocityControl *velocity;
        ProfileGenerator *profile;
    };
}

#endif /* INC_TRAJECTORYPLANNING */
