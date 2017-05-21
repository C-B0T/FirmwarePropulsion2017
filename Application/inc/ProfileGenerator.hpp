/**
 * @file    ProfileControl.hpp
 * @author  Jeremy ROULLAND
 * @date    5 feb. 2017
 * @brief   ProfileGenerator class
 */

 #ifndef INC_PROFILEGENERATOR_HPP_
 #define INC_PROFILEGENERATOR_HPP_

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
     struct pg_profile
     {
         float32_t    maxVel;
         float32_t    maxAcc;
         enum MotionProfile::PROFILE profile;
     }Profile_Angular;

     struct pg_profile Profile_Linear;
 }PG_DEF;


 /*----------------------------------------------------------------------------*/
 /* Class declaration                                                          */
 /*----------------------------------------------------------------------------*/

 /**
  * @namespace MotionControl
  */
 namespace MotionControl
 {
     /**
      * @class ProfileGenerator
      * @brief Profile generator class
      *
      * HOWTO :
      * -
      *
      */
     class ProfileGenerator
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
         static ProfileGenerator* GetInstance(bool standalone = true);

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
          * @brief start linear position setpoint
          */
         void StartLinearPosition(float32_t position);

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
          * @brief Start angular position setpoint
          */
         void StartAngularPosition(float32_t position);

         /**
          * @brief Set angular position setpoint
          */
         void SetAngularPosition(float32_t position)
         {
             // Set angular position order
             this->angularPosition = position;
         }

         /**
          * @brief Compute profile control
          */
         void Compute(float32_t period);

         /**
          * @brief Generate profile control
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
         ProfileGenerator(bool standalone);

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
         PG_DEF def;

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
#endif /* INC_PROFILEGENERATOR_HPP_ */
