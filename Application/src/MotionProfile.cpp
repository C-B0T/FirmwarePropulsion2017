/**
 * @file    MotionProfile.cpp
 * @author  Jeremy ROULLAND
 * @date    12 nov. 2016
 * @brief   Motion profile (Trapezoidal, S-Curve, ...)
 */

#include "MotionProfile.hpp"
#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace MotionControl;

namespace MotionControl
{

    MotionProfile::MotionProfile(float32_t maxVel, float32_t maxAcc, enum PROFILE profile)
    {
        assert(profile < PROFILE::MPROFILE_MAX);

        this->profile = profile;

        this->mode = MODE_AUTO;

        this->startTime = 0.0;

        this->setPoint   = 0.0;
        this->startPoint = 0.0;

        this->maxVel = maxVel;
        this->maxAcc = maxAcc;

        this->finished = false;
    }

    MotionProfile::~MotionProfile()
    {
    }

    bool MotionProfile::isFinished()
    {
        return this->finished;
    }

    void MotionProfile::SetPoint(float32_t point)
    {
    	this->setPoint = point - this->startPoint;
    }

    void MotionProfile::SetSetPoint(float32_t point, float32_t currentPoint, float32_t currentTime)
    {
        this->startTime = currentTime;
        this->startPoint = currentPoint;

        this->setPoint = point - this->startPoint;

        this->finished = false;
    }

    float32_t MotionProfile::Get(float32_t time)
    {
        float32_t r = 0.0;
        float32_t tf = 0.0;

		tf = calculateMinTime();

        r = this->Get(time, tf);

        return r;
    }

    float32_t MotionProfile::Get(float32_t time, float32_t tf)
    {
        float32_t r = 0.0;
        float32_t t = 0.0;

        t = time - this->startTime;
        assert(t >= 0.0);

        this->tf = tf;
        t /= tf;

        r = this->calculateProfile(t);

        return r;
    }

    float32_t MotionProfile::calculateProfile(float32_t t)
    {
        float32_t r = 0.0;

        switch (this->profile)
        {
            case NONE:
                r = this->calculateNoneProfile(t);
                break;

            case LINEAR:
                r = this->calculateLinearProfile(t);
                break;

            case TRIANGLE:
                r = this->calculateTriangleProfile(t);
                break;

            case TRAPEZ:
                r = this->calculateTrapezoidalProfile(t);
                break;

            case SCURVE:
                r = this->calculateSCurveProfile(t);
                break;

            case POLY3:
                r = this->calculatePolynomial3Profile(t);
                break;

            case POLY5:
                r = this->calculatePolynomial5Profile(t);
                break;

            case POLY5_P1:
                r = this->calculatePolynomial5Phase1Profile(t);
                break;

            case POLY5_P2:
                r = this->calculatePolynomial5Phase2Profile(t);
                break;

            case AUTO:
                r = this->calculateAutoProfile(t);
                break;

            default:
                r = 1.0;
                break;
        }

        r += this->startPoint;

        return r;
    }

    float32_t MotionProfile::calculateMinTime(enum MotionProfile::PROFILE profile)
    {
        float32_t tf = 0.0, tfVel = 0.0, tfAcc = 0.0;

        if(profile == AUTO)
        	profile = this->profile;

        switch (profile)
        {
            case LINEAR:
                tfVel = abs(this->setPoint) / this->maxVel;
                tfAcc = 0.0;
                break;

            case TRIANGLE:
                tfVel = 2.0 * abs(this->setPoint) / this->maxVel;
                tfAcc = 2.0 * sqrt( abs(this->setPoint) / this->maxAcc );
                break;

            case TRAPEZ:
                tfVel = this->maxVel / this->maxAcc + abs(this->setPoint) / this->maxVel;
                tfAcc = tfVel;
                break;

            case SCURVE:
                tfVel = 1.0;
                tfAcc = 1.0;
                break;

            case POLY3:
                tfVel = (3.0 * abs(this->setPoint)) / (2.0 * this->maxVel);
                tfAcc = sqrt( (6.0 * abs(this->setPoint)) / this->maxAcc );
                break;

            case POLY5:
                tfVel = (15.0 * abs(this->setPoint)) / (8.0 * this->maxVel);
                tfAcc = sqrt( (10.0 * abs(this->setPoint)) / (sqrt(3) * this->maxAcc) );
                break;

            case POLY5_P1:
            case POLY5_P2:
                tfVel = (15.0 * abs(this->setPoint)) / (8.0 * this->maxVel);
                tfAcc = sqrt( (10.0 * abs(this->setPoint)) / (2.0 * sqrt(3) * this->maxAcc) );
                break;

            case AUTO:
                tfVel = (15.0 * abs(this->setPoint)) / (8.0 * this->maxVel);
                tfAcc = sqrt( (10.0 * abs(this->setPoint)) / (2.0 * sqrt(3) * this->maxAcc) );
                break;

            default:
                tfVel = 0.0;
                tfAcc = 0.0;
                break;
        }

        tf = (tfVel >= tfAcc) ? tfVel : tfAcc;

        return tf;
    }

    float32_t MotionProfile::calculateMinDist(enum MotionProfile::PROFILE profile)
    {
    	float32_t d = 0.0;

        if(profile == AUTO)
            profile = this->profile;

        switch (profile)
        {
            case POLY5_P1:
            case POLY5_P2:
                d = (64.0*sqrt(3.0)/135.0)*(pow(this->maxVel,2)/this->maxAcc);
                break;
            default:
                break;
        }
    	return d;
    }

    float32_t MotionProfile::calculateNoneProfile(float32_t t)
    {
        float32_t s = 0.0;

        s = 1.0;

        this->finished = true;

        s *= this->setPoint;

        return s;
    }

    float32_t MotionProfile::calculateLinearProfile(float32_t t)
    {
        float32_t s = 0.0;

        if(t <= 1.0)
            s = t;
        else
            s = 1.0;

        if(t >= 1.0)
            this->finished = true;
        else
            this->finished = false;

        s *= this->setPoint;

        return s;
    }

    float32_t MotionProfile::calculateTriangleProfile(float32_t t)
    {
        float32_t s = 0.0;

        if(t <= 0.5)                                    // [0.0 to 0.5]
            s = 2.0*pow(t,2);
        else if(t <= 1.0)                               // ]0.5 to 1.0]
            s = -1.0 + 4.0*t - 2.0*pow(t,2);
        else                                            // ]1.0 to Inf[
            s = 1.0;

        if(t >= 1.0)
            this->finished = true;
        else
            this->finished = false;

        s *= this->setPoint;

        return s;
    }

    float32_t MotionProfile::calculateTrapezoidalProfile(float32_t t)
    {
        float32_t s = 0.0;

        if(!(abs(this->setPoint) > (pow(this->maxVel, 2) / this->maxAcc)))
        {
            // Force Triangle profile because we cant do trapez
            this->profile = TRIANGLE;
            s = calculateTriangleProfile(t);
            this->profile = TRAPEZ;
        }
        else
        {
            s = calculateCustomTrapezoidalProfile(t);
        }

        return s;
    }

    float32_t MotionProfile::calculateCustomTrapezoidalProfile(float32_t t)
    {
        float32_t s = 0.0;
        float32_t tf = 0.0;
        float32_t t1 = 0.0, t2 = 0.0;

        float32_t S1 = 0.0, S2 = 0.0, S3 = 0.0;
        float32_t T1 = 0.0, T2 = 0.0, T3 = 0.0;

        static float32_t st1 = 0.0, st2 = 0.0;

        /* Reverse tf calcul */
        tf = calculateMinTime();
        t *= tf;

        T1 = this->maxVel / this->maxAcc;
        S1 = pow(this->maxVel,2) / (2.0 * this->maxAcc);

        T3 = this->maxVel / this->maxAcc;
        S3 = pow(this->maxVel,2) / (2.0 * this->maxAcc);

        S2 = this->setPoint - pow(this->maxVel,2) / this->maxAcc;
        T2 = this->setPoint / this->maxVel - this->maxVel / this->maxAcc;

        tf = this->setPoint / this->maxVel + this->maxVel / this->maxAcc;
        t1 = T1;
        t2 = T1 + T2;

        if(t <= t1)
            s = 0.5 * this->maxAcc * pow(t,2);
        else if(t <= t2)
            s = this->maxVel * (t-t1) + st1;
        else if(t <= tf)
            s = -0.5 * this->maxAcc * pow(t-t2,2) + this->maxVel * (t-t2) + st2;
        else
            s = this->setPoint;

        if(t < t1)
            st1 = s;
        else if(t < t2)
            st2 = s;

        if(t >= tf)
            this->finished = true;
        else
            this->finished = false;

        return s;
    }

    float32_t MotionProfile::calculateSCurveProfile(float32_t t)
    {
        float32_t s = 0.0;

        /*@todo: S-Curve Algorihm */
        s = 1.0;

        if(t >= 1.0)
            this->finished = true;
        else
            this->finished = false;

        s *= this->setPoint;

        return s;
    }

    float32_t MotionProfile::calculatePolynomial3Profile(float32_t t)
    {
        float32_t s = 0.0;

        if(t <= 1.0)
            s = 3.0*pow(t,2)-2.0*pow(t,3);
        else
            s = 1.0;

        if(t >= 1.0)
            this->finished = true;
        else
            this->finished = false;

        s *= this->setPoint;

        return s;
    }

    float32_t MotionProfile::calculatePolynomial5Profile(float32_t t)
    {
        float32_t s = 0.0;

        if(t <= 1.0)
            s = 10.0*pow(t,3)-15.0*pow(t,4)+6.0*pow(t,5);
        else
            s = 1.0;

        if(t >= 1.0)
            this->finished = true;
        else
            this->finished = false;

        s *= this->setPoint;

        return s;
    }

    float32_t MotionProfile::calculatePolynomial5Phase1Profile(float32_t t)
    {
        float32_t s = 0.0;

        if(t <= 1.0)
            s = 5.0/2.0*pow(t,3)-15.0/8.0*pow(t,4)+3.0/8.0*pow(t,5);
        else
        {
            s = 1.0;
        }

        if(t >= 1.0)
            this->finished = true;
        else
            this->finished = false;

        s *= this->setPoint;

        return s;
    }

    float32_t MotionProfile::calculatePolynomial5Phase2Profile(float32_t t)
    {
        float32_t s = 0.0;

        if(t <= 1.0)
            s = 15.0/8.0*t-5.0/4.0*pow(t,3)+3.0/8.0*pow(t,5);
        else
        {
            s = 1.0;
        }

        if(t >= 1.0)
            this->finished = true;
        else
            this->finished = false;

        s *= this->setPoint;

        return s;
    }

    float32_t MotionProfile::calculateAutoProfile(float32_t t)
    {
        float32_t s = 0.0;

        if(t <= 1.0)
            s = 10.0/4.0*pow(t,3)-15.0/8.0*pow(t,4)+6.0/16.0*pow(t,5);
        else
        {
            s = 1.875 * t - 0.875;
        }

        if(t >= 2.0)
            this->finished = true;
        else
            this->finished = false;

        s *= this->setPoint;

        return s;
    }

    float32_t MotionProfile::abs(float32_t val)
    {
        if(val < 0.0)
            val = -val;
        return val;
    }

}
