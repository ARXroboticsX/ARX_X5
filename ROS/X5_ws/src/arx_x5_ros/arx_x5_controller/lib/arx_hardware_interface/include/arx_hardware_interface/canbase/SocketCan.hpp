 #pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"
#include "arx_hardware_interface/typedef/HybridJointTypeDef.hpp"

// #include "CANAdapter.h"
#include <stdbool.h>
#include <net/if.h>
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>

#include <linux/can.h>

#include <sys/select.h>
#include <sys/socket.h>
// SIOCGIFINDEX
#include <sys/ioctl.h>

#include <mutex>
#include <memory>
#include <functional>
#include <string.h>

#include <atomic>

namespace arx
{
    namespace hw_interface
    {
        /// @brief 继承CanBase并使用SocketCan进行数据通信
        class SocketCan : public CanBase
        {
        public:
            SocketCan();
            ~SocketCan();

            bool Init(const char *interface) override;
            bool ExchangeData(CanFrame *frame) override;
            bool WriteData();
            bool ReadData();
            bool IsOpen();

            void setCallBackFunction(const std::function<void(CanFrame *frame)> func);
            void setGetMsgContentFunction(const std::function<void()> func);

            void LoadMutexMsg();

            /// @brief ----
            void Close();

        private:
            class impl;
            std::unique_ptr<impl> pimpl;
        };
    }
}