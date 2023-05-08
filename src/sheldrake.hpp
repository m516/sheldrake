#pragma once

#include <drake/systems/controllers/pid_controller.h>
#include "serial/serial.h"
#include <memory>



namespace sheldrake{
    class Sheldrake{
        public:
        Sheldrake(std::shared_ptr<serial::Serial> const &serial): com_type{Serial}, comserial(serial){}

        /// @brief Deploys a PID controller to an Arduino
        /// @tparam T the type of PidController
        /// @param controller 
        template<typename T>
        void deploy(const drake::systems::controllers::PidController<T> &controller);

        private:
        enum _Com_type { Serial };
        const _Com_type com_type;
        const std::shared_ptr<serial::Serial> comserial;
    };
}