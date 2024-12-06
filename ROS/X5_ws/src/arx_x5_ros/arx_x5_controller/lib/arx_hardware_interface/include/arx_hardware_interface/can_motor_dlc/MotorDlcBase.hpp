#pragma once

#include "arx_hardware_interface/typedef/HybridJointTypeDef.hpp"
#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

namespace arx
{
    namespace hw_interface
    {
        class MotorDlcBase
        {
        public:
            virtual ~MotorDlcBase() {}

            virtual CanFrame packMotorMsg(HybridJointCmd *command) = 0;
            virtual CanFrame packMotorMsg(double k_p, double k_d, double position, double velocity, double torque) = 0;

            virtual void CanAnalyze(CanFrame *frame) = 0;

            virtual CanFrame packEnableMotor() = 0;
            virtual CanFrame packDisableMotor() = 0;

            virtual HybridJointStatus GetMotorMsg() = 0;
            virtual void ExchangeMotorMsg() = 0;
            
        private:

        };
    }
}