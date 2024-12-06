#pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

#include "arx_hardware_interface/can_motor_dlc/MotorDlcBase.hpp"

#include "arx_hardware_interface/typedef/HybridJointTypeDef.hpp"

#include <stdio.h>

#include <cmath>


namespace arx
{
    namespace hw_interface
    {
        class MotorType2 : public MotorDlcBase
        {
        public:
            MotorType2(int motor_id) : motor_id_(motor_id) {};

            CanFrame packMotorMsg(HybridJointCmd *command);
            CanFrame packMotorMsg(double k_p, double k_d, double position, double velocity, double torque);

            CanFrame packEnableMotor();
            CanFrame packDisableMotor();

            void CanAnalyze(CanFrame *frame) override; // 尝试接收电机数据

            HybridJointStatus GetMotorMsg();
            void ExchangeMotorMsg();

        private:
            double restrictBound(double num, double max, double min);
            
            double position_;
            double velocity_;
            double current_;

            double position_exchange_;
            double velocity_exchange_;
            double current_exchange_;

            CanFrame *frame_;
            int motor_id_;
        };
    }
}
