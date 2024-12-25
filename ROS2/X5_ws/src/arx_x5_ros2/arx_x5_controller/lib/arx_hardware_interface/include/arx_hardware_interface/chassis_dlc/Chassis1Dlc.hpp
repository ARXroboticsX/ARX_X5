#pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

namespace arx
{
    namespace hw_interface
    {
        class ChassisType1
        {
        public:
            CanFrame packChassisCmd1(int v_x, int v_y, int v_z, int mode);
            CanFrame packChassisCmd2(int wheel1_vel, int wheel2_vel, int wheel3_vel);
            void unpackChassisFeedback(CanFrame& frame);

        private:
            double wheel1_vel_, wheel2_vel_, wheel3_vel_;
        };
    }
}
