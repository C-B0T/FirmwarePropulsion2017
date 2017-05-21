/**
 * @file    Odometry.cpp
 * @author  Jeremy ROULLAND
 * @date    3 dec. 2016
 * @brief   Odometry is use to estimate location
 */

#include "Odometry.hpp"
#include "common.h"


using namespace HAL;

/*----------------------------------------------------------------------------*/
/* Definitions                                                                */
/*----------------------------------------------------------------------------*/

#define L_ENCODER_ID    (Encoder::ENCODER0)
#define R_ENCODER_ID    (Encoder::ENCODER1)

#define WD          47.7            // Wheel diameter
#define WC          1.0             // Wheel correction (Difference between the (referent) right and left wheel)
#define ER          4096            // Encoder resolution
//#define ADW         254.0           // Axial distance between wheels
#define ADW         252.68          // Axial distance between wheels

#define _PI_        3.14159265358979323846
#define _2_PI_      6.28318530717958647692  // 2*PI

#define TICK_BY_MM  27.3332765997653373295  // (ER/(_PI_*WD))
//#define ADW_TICK    6942.65225634039568170  // (ADW * TICK_BY_MM)
#define ADW_TICK    6906.57233123  // (ADW * TICK_BY_MM)



#define ODO_TASK_STACK_SIZE     (256u)
#define ODO_TASK_PRIORITY       (configMAX_PRIORITIES-2)

#define ODO_LOOP_PERIOD_MS      (5u) // 5ms Odometry loop


/*----------------------------------------------------------------------------*/
/* Private Members                                                            */
/*----------------------------------------------------------------------------*/

static Location::Odometry* _odometry = NULL;

/*----------------------------------------------------------------------------*/
/* Private Functions                                                          */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Class Implementation                                                       */
/*----------------------------------------------------------------------------*/

namespace Location
{
    Odometry* Odometry::GetInstance(bool standalone)
    {
        if(_odometry != NULL)
        {
            return _odometry;
        }
        else
        {
            _odometry = new Odometry(standalone);
            return _odometry;
        }
    }

    Odometry::Odometry(bool standalone)
    {
        this->name = "ODOMETRY";
        this->taskHandle = NULL;

        // Init members
        this->robot.X = 0.0;
        this->robot.Y = 0.0;
        this->robot.O = 0.0;
        this->robot.L = 0.0;

        this->robot.Xmm  = 0;
        this->robot.Ymm  = 0;
        this->robot.Odeg = 0.0;
        this->robot.Lmm  = 0;

        this->robot.AngularVelocity = 0.0;
        this->robot.LinearVelocity  = 0.0;

        this->robot.LeftVelocity  = 0.0;
        this->robot.RightVelocity = 0.0;

        // Init encoders
        this->leftEncoder  = Encoder::GetInstance(L_ENCODER_ID);
        this->rightEncoder = Encoder::GetInstance(R_ENCODER_ID);

        if(standalone == true)
        {
            // Create task
            xTaskCreate((TaskFunction_t)(&Odometry::taskHandler),
            this->name.c_str(),
            ODO_TASK_STACK_SIZE,
            NULL,
            ODO_TASK_PRIORITY,
            NULL);
        }
    }


    Odometry::Odometry(float32_t X, float32_t Y, float32_t O, float32_t L) : Odometry(false)
    {
        this->Init(X, Y, O, L);
    }


    Odometry::~Odometry()
    {

    }

    void Odometry::Init(float32_t X, float32_t Y, float32_t O, float32_t L)
    {
        this->robot.X = X;  // tick
        this->robot.Y = Y;  // tick
        this->robot.O = O;  // radian
        this->robot.L = L;  // tick

        this->robot.Xmm  = static_cast<int32_t>(X / TICK_BY_MM);
        this->robot.Ymm  = static_cast<int32_t>(Y / TICK_BY_MM);
        this->robot.Odeg = static_cast<float32_t>((180.0 * O) / _PI_);
        this->robot.Lmm  = static_cast<int32_t>(L / TICK_BY_MM);
    }

    void Odometry::GetRobot(robot_t *r)
    {
        r->X = this->robot.X;
        r->Y = this->robot.Y;
        r->O = this->robot.O;
        r->L = this->robot.L;

        r->AngularVelocity = this->robot.AngularVelocity;
        r->LinearVelocity  = this->robot.LinearVelocity;

        r->LeftVelocity  = this->robot.LeftVelocity;
        r->RightVelocity = this->robot.RightVelocity;

        r->Xmm  = this->robot.Xmm;
        r->Ymm  = this->robot.Ymm;
        r->Odeg = this->robot.Odeg;
        r->Lmm  = this->robot.Lmm;
    }

     /**
     * @brief Get Angular Position
     * @return : Angular position in radian (S.I Unit)
     */
     float32_t Odometry::GetAngularPosition()
     {
         return this->robot.O;
     }

     /**
     * @brief Get Linear Position
     * @return : Linear position in meter (S.I Unit)
     */
     float32_t Odometry::GetLinearPosition()
     {
         return this->robot.L / (TICK_BY_MM * 1000.0);
     }

     /**
     * @brief Get Angular Velocity
     * @param period : Velocity period required
     * @return : Angular Velocity in rad/s (S.I Units)
     */
     float32_t Odometry::GetAngularVelocity(float32_t period)
     {
         float32_t odo_period = ODO_LOOP_PERIOD_MS;

         return (this->robot.AngularVelocity / odo_period) * period;
     }

    /**
     * @brief Get Linear Velocity
     * @param period : Velocity period required
     * @return : Linear Velocity in m/s (S.I Units)
     */
     float32_t Odometry::GetLinearVelocity(float32_t period)
     {
         float32_t odo_period = ODO_LOOP_PERIOD_MS;

         return ((this->robot.LinearVelocity / odo_period) * period) / (TICK_BY_MM * 1000.0);
     }

    /**
     * @brief Get Left Velocity
     * @param period : Velocity period required
     * @return : Left Velocity in m/s (S.I Units)
     */
     float32_t Odometry::GetLeftVelocity(float32_t period)
     {
         float32_t odo_period = ODO_LOOP_PERIOD_MS;

         return ((this->robot.LeftVelocity / odo_period) * period) / (TICK_BY_MM * 1000.0);
     }

    /**
     * @brief Get Right Velocity
     * @param period : Velocity period required
     * @return : Left Velocity in m/s (S.I Units)
     */
     float32_t Odometry::GetRightVelocity(float32_t period)
     {
         float32_t odo_period = ODO_LOOP_PERIOD_MS;

         return ((this->robot.RightVelocity / odo_period) * period) / (TICK_BY_MM * 1000.0);
     }

    void Odometry::SetXO(float32_t X, float32_t O)
    {
        this->robot.X = X;
        this->robot.O = O;

        this->robot.Xmm = X / TICK_BY_MM;
        this->robot.Odeg = (180.0 * O) / _PI_;
    }


    void Odometry::SetYO(float32_t Y, float32_t O)
    {
        this->robot.Y = Y;
        this->robot.O = O;

        this->robot.Ymm = Y / TICK_BY_MM;
        this->robot.Odeg = (180.0 * O) / _PI_;
    }


    void Odometry::Compute(float32_t period)
    {
        int32_t dl = 0;
        int32_t dr = 0;

        float32_t dX = 0.0;
        float32_t dY = 0.0;

        float32_t dO = 0.0;
        float32_t dL = 0.0;

        float32_t dlf = 0.0;
        float32_t drf = 0.0;

        dl = +  leftEncoder->GetRelativeValue();
        dr = - rightEncoder->GetRelativeValue();

        dlf = static_cast<float32_t>(dl) * WC;
        drf = static_cast<float32_t>(dr);

        dO =  drf - dlf;
        dL = (drf + dlf) / 2.0;

        this->robot.O += (dO / ADW_TICK);
        this->robot.L += dL;

        this->robot.AngularVelocity = (dO / ADW_TICK);
        this->robot.LinearVelocity  = dL;

        while(this->robot.O > _2_PI_)
            this->robot.O -= _2_PI_;
        while(this->robot.O < -_2_PI_)
            this->robot.O += _2_PI_;

        dX = cos(this->robot.O) * dL;
        dY = sin(this->robot.O) * dL;

        this->robot.X += dX;
        this->robot.Y += dY;

        this->robot.LeftVelocity  = dl;
        this->robot.RightVelocity = dr;

        this->robot.Xmm  = static_cast<int32_t>(this->robot.X / TICK_BY_MM);
        this->robot.Ymm  = static_cast<int32_t>(this->robot.Y / TICK_BY_MM);
        this->robot.Odeg = static_cast<float32_t>((180.0 * this->robot.O) / _PI_);
        this->robot.Lmm  = static_cast<int32_t>(this->robot.L / TICK_BY_MM);
    }

    void Odometry::taskHandler(void* obj)
    {
        TickType_t xLastWakeTime;
        TickType_t xFrequency = pdMS_TO_TICKS(ODO_LOOP_PERIOD_MS);

        Odometry* instance = _odometry;
        TickType_t prevTick = 0u,  tick = 0u;

        float32_t period = 0.0f;

        // 1. Initialise periodical task
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

            //4. Compute location (Odometry)
            instance->Compute(period);

            // 5. Set previous tick
            prevTick = tick;
        }
    }

}
