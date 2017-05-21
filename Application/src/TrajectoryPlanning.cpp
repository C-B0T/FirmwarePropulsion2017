/**
 * @file    TrajectoryPlanning.cpp
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Motion profile (Trapezoidal, S-Curve, ...)
 */

#include "TrajectoryPlanning.hpp"

#include <math.h>

using namespace MotionControl;

namespace MotionControl
{
    TrajectoryPlanning::TrajectoryPlanning()
    {
        this->state = FREE;
        this->step = 0;

        this->stallMode = 0;

        this->X[0] = 0.0;
        this->Y[0] = 0.0;
        this->XYn  = 0.0;

        this->linearSetPoint = 0.0;
        this->linearNextSetPoint = 0.0;
        this->angularSetPoint = 0.0;

        this->odometry = Odometry::GetInstance();
        this->position = PositionControl::GetInstance();
        this->velocity = VelocityControl::GetInstance();
        this->profile  = ProfileGenerator::GetInstance();
    }

    TrajectoryPlanning::~TrajectoryPlanning()
    {
    }

    void TrajectoryPlanning::goLinear(float32_t linear) // linear in meters
    {
        this->linearSetPoint = linear;

        this->state = LINEAR;
        this->step  = 1;
    }

    void TrajectoryPlanning::goAngular(float32_t angular) // angular in radian
    {
        this->angularSetPoint = angular;

        this->state = ANGULAR;
        this->step  = 1;
    }

    void TrajectoryPlanning::stop()
    {
        this->position->Stop();

        //TODO: To prepare S-curve deceleration (to discuss)

        this->state = STOP;
        this->step  = 1;
    }

    void TrajectoryPlanning::gotoXY(float32_t X, float32_t Y)
    {
        robot_t r;

        this->odometry->GetRobot(&r);

        float32_t Xm = static_cast<float32_t>(r.Xmm) / 1000.0;
        float32_t Ym = static_cast<float32_t>(r.Ymm) / 1000.0;
        float32_t Lm = static_cast<float32_t>(r.Lmm) / 1000.0;

        float32_t XX = pow((X - Xm), 2);
        float32_t YY = pow((Y - Ym), 2);

        float32_t dX = X - Xm;   // meters
        float32_t dY = Y - Ym;   // meters

        this->linearSetPoint  = Lm + sqrtl(XX + YY); // meters
        this->angularSetPoint = atan2f(dY,dX);  // radians

        /* Faster path */
        while( (this->angularSetPoint - r.O) > _PI_)
            this->angularSetPoint -= _2_PI_;
        while( (this->angularSetPoint - r.O) < -_PI_)
            this->angularSetPoint += _2_PI_;

        this->state = LINEARPLAN;
        this->step  = 1;
    }

    void TrajectoryPlanning::pushXY(float32_t X[], float32_t Y[], uint32_t n)
    {
        uint32_t i;

        assert(n<10);

        for(i=0 ; i<n ; i++)
        {
            this->X[i] = X[i];
            this->Y[i] = Y[i];
        }

        this->XYn = n;

        this->state = DRAWPLAN;
        this->step  = 1;
    }

    int32_t TrajectoryPlanning::stallX(int32_t stallMode)
    {
        // TODO:Check the stallMode coherence (Ex1: if ur on the left side of the table don't exe rightTable side to side Mode)
        //                                    (Ex2: if ur on the up side of the table don't exe downTable side to side Mode)

        struct robot r;
        odometry->GetRobot(&r);

        startLinearPosition = r.L;
        startAngularPosition = r.O;

        endLinearPosition = startLinearPosition - 0.0;
        endAngularPosition = 0.0;

        state = STALLX;
        step = 1;

        return 0;
    }

    int32_t TrajectoryPlanning::stallY(int32_t stallMode)
    {
        // TODO:Check the stallMode coherence (Ex1: if ur on the left side of the table don't exe rightTable side to side Mode)
        //                                    (Ex2: if ur on the up side of the table don't exe downTable side to side Mode)

        struct robot r;
        odometry->GetRobot(&r);

        startLinearPosition = r.L;
        startAngularPosition = r.O;

        endLinearPosition = startLinearPosition - 0.0;
        endAngularPosition = _PI_/2.0;

        state = STALLY;
        step = 1;

        return 0;
    }


    float32_t TrajectoryPlanning::update()
    {
        switch(state)
        {
            // Simple mouvements
            case LINEAR:
                calculateGoLinear();
                break;

            case ANGULAR:
                calculateGoAngular();
                break;

            case STOP:
                calculateStop();
                break;

            case KEEP:
                calculateKeepPosition();
                break;

            // semi-complex mouvements
            case LINEARPLAN:
                calculateLinearPlan();
                break;

            case DRAWPLAN:
                calculateDrawPlan();
                break;

            // complex mouvements
            case CURVEPLAN:
                calculateCurvePlan();
                break;

            // special mouvements
            case STALLX:
                calculateStallX(1);
                break;

            case STALLY:
                calculateStallY(1);
                break;

            default:
                break;
        }

        if(this->state == FREE)
        {
            calculateFree();
            return 1;
        }
        else
        {
            calculateMove();
            return 0;
        }
    }

    void TrajectoryPlanning::calculateFree()
    {
        this->position->Disable();
        this->velocity->Disable();
        this->position->Stop();
    }

    void TrajectoryPlanning::calculateMove()
    {
        this->position->Enable();
        this->velocity->Enable();
    }

    void TrajectoryPlanning::calculateGoLinear()
    {
        switch (step)
        {
            case 1:    // Set order
                this->profile->StartLinearPosition(this->linearSetPoint);
                this->profile->StartAngularPosition(odometry->GetAngularPosition());
                this->step = 2;
                break;

            case 2:    // Check is arrived
                if(this->profile->isPositioningFinished())
                {
                    this->step = 3;
                    this->state = FREE;
                }
                break;

            default:
                break;
        }
    }

    void TrajectoryPlanning::calculateGoAngular()
    {
        switch (step)
        {
            case 1:    // Set order
                this->profile->StartLinearPosition(odometry->GetLinearPosition());
                this->profile->StartAngularPosition(this->angularSetPoint);
                this->step = 2;
                break;

            case 2:    // Check is arrived
                if(this->profile->isPositioningFinished())
                {
                    this->step = 3;
                    this->state = FREE;
                }
                break;

            default:
                break;
        }
    }

    void TrajectoryPlanning::calculateStop()
    {
        //TODO: calculate stop (brake)
    }

    void TrajectoryPlanning::calculateKeepPosition()
    {
        //TODO: calculate keep position
    }

    void TrajectoryPlanning::calculateLinearPlan()
    {
        switch (step)
        {
            case 1:    // Start Angular Position
                break;

            case 2:
                if(this->profile->isPositioningFinished())
                {
                    step = 3;
                }
                break;

            case 3:    // Start Linear Position
                break;

            case 4:
                if(this->profile->isPositioningFinished())
                {
                    step = 5;
                    this->state = FREE;
                }
                break;

            default:
                break;
        }

        switch (step)
        {
            case 1:
                this->profile->StartLinearPosition(odometry->GetLinearPosition());
                this->profile->StartAngularPosition(this->angularSetPoint);
                step = 2;
                break;

            case 2:
                break;

            case 3:
                this->profile->StartLinearPosition(this->linearSetPoint);
                this->profile->StartAngularPosition(odometry->GetAngularPosition());
                step = 4;
                break;

            case 4:
                break;

            default:
                break;
        }

    }

    void TrajectoryPlanning::updateXYtoLA(uint32_t n)
    {
        float32_t Xm, Ym, Lm;
        float32_t XX, YY;
        float32_t dX, dY;
        robot_t r;
        uint32_t i;

        this->odometry->GetRobot(&r);

        Xm = static_cast<float32_t>(r.Xmm) / 1000.0;
        Ym = static_cast<float32_t>(r.Ymm) / 1000.0;
        Lm = static_cast<float32_t>(r.Lmm) / 1000.0;

        XX = pow((this->X[n] - Xm), 2);
        YY = pow((this->Y[n] - Ym), 2);

        dX = this->X[n] - Xm;   // meters
        dY = this->Y[n] - Ym;   // meters

        this->linearSetPoint  = Lm + sqrtl(XX + YY); // meters
        this->angularSetPoint = atan2f(dY,dX);  // radians

        /* Faster path */
        while( (this->angularSetPoint - r.O) > _PI_)
            this->angularSetPoint -= _2_PI_;
        while( (this->angularSetPoint - r.O) < -_PI_)
            this->angularSetPoint += _2_PI_;

        this->linearNextSetPoint  = this->linearSetPoint;

        if(this->XYn >= 2)
        {
    for(i=n ; i < (this->XYn-1) ; i++)
    {
    XX = pow((this->X[i] - this->X[i-1]), 2);
    YY = pow((this->Y[i] - this->Y[i-1]), 2);

    this->linearSetPoint  += sqrtl(XX + YY);
    }
        }
    }

    void TrajectoryPlanning::calculateDrawPlan()
    {
        static uint32_t n = 0;

        float32_t currentLinearPosition = 0.0;

        switch (step)
        {
            case 1:    // Start Angular Position
                n = 0;
                step = 2;
                this->updateXYtoLA(n);
                this->profile->StartLinearPosition(this->linearSetPoint);
                this->profile->StartAngularPosition(this->angularSetPoint);
                /* no break */
                //break;

            case 2:
                this->updateXYtoLA(n);
                currentLinearPosition = odometry->GetLinearPosition();
                if(abs(this->linearNextSetPoint - currentLinearPosition) <= 0.1)
                {
                    n++;
                    step = 2;
                    if(n >= this->XYn-1)
                    {
                        step = 3;
                    }
                    this->updateXYtoLA(n);
                    this->profile->StartAngularPosition(this->angularSetPoint);
                }
                break;

            case 3:    /* Last coordinate */
                this->updateXYtoLA(n);
                currentLinearPosition = odometry->GetLinearPosition();

                if(abs(this->linearSetPoint - currentLinearPosition) <= 0.1)
                {
                    step = 4;
                }
                break;

            case 4:    /* Finishing last coordinate */
                this->updateXYtoLA(n);
                currentLinearPosition = odometry->GetLinearPosition();
                if(this->profile->isPositioningFinished() || (abs(this->linearSetPoint - currentLinearPosition) <= 0.01))
                {
                    step = 5;
                    this->state = FREE;
                }
                break;

            default:
                break;
        }

        switch (step)
        {
            case 1:
                break;

            case 2:
                this->profile->SetLinearPosition(this->linearSetPoint);
                this->profile->SetAngularPosition(this->angularSetPoint);
                break;

            case 3:
                this->profile->SetLinearPosition(this->linearSetPoint);
                this->profile->SetAngularPosition(this->angularSetPoint);
                break;

            case 4:
                this->profile->SetLinearPosition(this->linearSetPoint);
                this->profile->SetAngularPosition(this->angularSetPoint);
                break;

            default:
                break;
        }

    }

    void TrajectoryPlanning::calculateCurvePlan()
    {
        //TODO: Curve Plan : Need both MotionProfile synchronized
    }


    void TrajectoryPlanning::calculateStallX(int32_t mode)
    {
        float32_t time = getTime();
        time -= startTime;

        float32_t profile = 0.0;

        //TODO:Add stallMode gestion
        bool jackBackLeft  = true;
        bool jackBackRight = true;

        switch (step)
        {
            case 1:
                if(this->profile->isPositioningFinished())
                    step = 2;
                break;

            case 2:
                if(jackBackLeft && jackBackRight)
                    step = 3;
                break;

            case 3:
                // X axis and Angular are calibrated
                break;

            default:
                break;
        }

        switch (step)
        {
            case 1: // Rotate to 0 rad
                this->position->SetAngularPosition(0.0);
                break;

            case 2:
                //TODO:Disable Angular asserv.
                //TODO:Back and wait both jacks
                break;

            case 3:
                //TODO:Modify X et O value in function of the mechanic
                odometry->SetXO(0.0, 0.0);
                break;

            default:
                break;
        }

    }

    void TrajectoryPlanning::calculateStallY(int32_t mode)
    {
        float32_t time = getTime();
        time -= startTime;

        float32_t profile = 0.0;

        //TODO:Add stallMode gestion
        bool jackBackLeft  = true;
        bool jackBackRight = true;

        switch (step)
        {
            case 1:
                if(this->profile->isPositioningFinished())
                    step = 2;
                break;

            case 2:
                if(jackBackLeft && jackBackRight)
                    step = 3;
                break;

            case 3:
                // Y axis and Angular are calibrated
                break;

            default:
                break;
        }

        switch (step)
        {
            case 1: // Rotate to pi/2 rad
                this->position->SetAngularPosition(_PI_/2.0);
                break;

            case 2:
                //TODO:Disable Angular asserv.
                //TODO:Back and wait both jacks
                break;

            case 3:
                //TODO:Modify Y value in function of the mechanic
                odometry->SetYO(0.0, _PI_/2.0);
                break;

            default:
                break;
        }

    }


    float32_t TrajectoryPlanning::getTime()
    {
        float32_t time = 0.0;

        time = static_cast<float32_t>(xTaskGetTickCount());
        time /= 1000.0;

        return  time;
    }

    float32_t TrajectoryPlanning::abs(float32_t val)
    {
        if(val < 0.0)
            val = -val;
        return val;
    }
}
