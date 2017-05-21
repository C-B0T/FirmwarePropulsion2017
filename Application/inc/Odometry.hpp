/**
 * @file    Odometry.hpp
 * @author  Jeremy ROULLAND
 * @date    3 dec. 2016
 * @brief   Odometry is use to estimate location
 */


#ifndef INC_ODOMETRY_HPP_
#define INC_ODOMETRY_HPP_

#include "HAL.hpp"

#include <math.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

typedef struct
{
    // Meca
    struct
    {
        float32_t   wd;     /* Wheel diameter */
        float32_t   wc;     /* Wheel correction */
        int32_t     er;     /* Encoder resolution */
        float32_t   adw;    /* Axial distance between wheels */

        float32_t   tbm;    /* Tick by mm (ER/(_PI_*WD)) */
        float32_t   adwt;   /* Tick by mm (ER/(_PI_*WD)) */
    }MECA;
}MECA_DEF;

typedef struct robot
{
    float32_t X;
    float32_t Y;
    float32_t O;
    float32_t L;

    float32_t AngularVelocity;
    float32_t LinearVelocity;

    float32_t LeftVelocity;
    float32_t RightVelocity;

    int32_t Xmm;
    int32_t Ymm;
    float32_t Odeg;
    int32_t Lmm;
} robot_t;


/*----------------------------------------------------------------------------*/
/* Class declaration	                                                      */
/*----------------------------------------------------------------------------*/

/**
 * @namespace Location
 */
namespace Location
{
    /**
     * @class Odometry
     * @brief Provides a singletron Odometry class
     *
     * HOWTO :
     * - Get Odometry instance with Location::Odometry::GetInstance()
     * - On init, Call Init() to init values
     * - Call periodically Compute() to compute the robot location
     * - Call GetRobot() to get current location
     */
    class Odometry
    {
    public:

        /**
        * @brief Get instance method
        * @return Odometry
        *  instance
        */
        static Odometry* GetInstance (bool standalone = true);

        /**
         * @brief Return instance name
         */
        std::string Name()
        {
            return this->name;
        }

        /**
         * @brief Init coordinates
         * @param X : X cartesian coordinate (X plane)
         * @param Y : Y cartesian coordinate (Y plane)
         * @param O : O polar coordinate (pole)
         * @param L : L polar coordinate (axis)
         */
         void Init(float32_t X, float32_t Y, float32_t O, float32_t L);

        /**
         * @brief Get current location
         * @param r : robot struct
         */
         void GetRobot(robot_t * r);

        /**
         * @brief Get Angular Position (S.I normalized)
         */
         float32_t GetAngularPosition();

        /**
         * @brief Get Linear Position (S.I normalized)
         */
         float32_t GetLinearPosition();

        /**
         * @brief Get Angular Velocity
         * @param period : Velocity period required
         */
         float32_t GetAngularVelocity(float32_t period = 1000.0);

        /**
         * @brief Get Linear Velocity
         * @param period : Velocity period required
         */
         float32_t GetLinearVelocity(float32_t period = 1000.0);

        /**
         * @brief Get Left Velocity
         * @param period : Velocity period required
         */
         float32_t GetLeftVelocity(float32_t period = 1000.0);

        /**
         * @brief Get Right Velocity
         * @param period : Velocity period required
         */
         float32_t GetRightVelocity(float32_t period = 1000.0);

         /**
          * @brief Set coordinate X and O during stall
          * @param X : X cartesian coordinate (X plane)
          * @param O : O polar coordinate (pole)
          */
         void SetXO(float32_t X, float32_t O);

        /**
         * @brief Set coordinate Y and O during stall
         * @param Y : Y cartesian coordinate (Y plane)
         * @param O : O polar coordinate (pole)
         */
         void SetYO(float32_t Y, float32_t O);

        /**
         * @brief Compute robot location (Should be called periodically)
         */
         void Compute(float32_t period);

    protected:
        /**
         * @brief Odometry default constructor
         */
        Odometry(bool standalone);

        /**
         * @brief Odometry constructor
         * @param X : X cartesian coordinate (X plane)
         * @param Y : Y cartesian coordinate (Y plane)
         * @param O : O polar coordinate (pole)
         * @param L : L polar coordinate (axis)
         */
        Odometry(float32_t X, float32_t Y, float32_t O, float32_t L);

        /**
         * @brief Odometry default destructor
         */
        ~Odometry();

        /**
         * @protected
         * @brief Instance name
         */
        std::string name;

        /**
         * @protected
         * @brief data of robot
         */
        robot_t robot;

        /**
         * @protected
         * @brief Left wheel encoder
         */
        HAL::Encoder* leftEncoder;

        /**
         * @protected
         * @brief Right wheel encoder
         */
        HAL::Encoder* rightEncoder;

        /**
         * @protected
         * @brief Meca definitions
         */
        MECA_DEF def;

        /**
         * @protected
         * @brief OS Task handle
         *
         * Used by speed control loop
         */
        TaskHandle_t taskHandle;

        /**
         * @protected
         * @brief Odometry loop task handler
         * @param obj : Always NULL
         */
        void taskHandler (void* obj);

    };
}

#endif /* INC_ODOMETRY_HPP_ */
