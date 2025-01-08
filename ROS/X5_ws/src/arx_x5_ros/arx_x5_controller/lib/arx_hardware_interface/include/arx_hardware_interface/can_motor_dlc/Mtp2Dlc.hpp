#pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

#include "arx_hardware_interface/can_motor_dlc/MotorDlcBase.hpp"

#include "arx_hardware_interface/typedef/HybridJointTypeDef.hpp"

#include <stdio.h>

#include <cmath>

#include <chrono>

namespace arx
{
    namespace hw_interface
    {
        class MotorType2 : public MotorDlcBase
        {
        public:
            MotorType2(int motor_id) : motor_id_(motor_id) {
                last_update_time_exchange_ = std::chrono::system_clock::now();
            };
            CanFrame packMotorMsg(HybridJointCmd *command);
            CanFrame packMotorMsg(double k_p, double k_d, double position, double velocity, double torque);

            CanFrame packEnableMotor();
            CanFrame packDisableMotor();

            void CanAnalyze(CanFrame *frame) override; // 尝试接收电机数据

            HybridJointStatus GetMotorMsg();
            bool online() override
            {
              std::chrono::time_point now = std::chrono::system_clock::now();
              auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_time_);
              if (duration.count() > 100000)
                online_ = false;
              return online_;
            }
            void ExchangeMotorMsg();

        private:
            double restrictBound(double num, double max, double min);
            
            double position_;
            double velocity_;
            double current_;
            bool online_{true};
            std::chrono::system_clock::time_point last_update_time_;

            double position_exchange_;
            double velocity_exchange_;
            double current_exchange_;
            std::chrono::system_clock::time_point last_update_time_exchange_;

            CanFrame *frame_;
            int motor_id_;
        };
    }
}
