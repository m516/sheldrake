#pragma once

#include <drake/systems/controllers/pid_controller.h>
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "serial/serial.h"
#include <memory>



namespace sheldrake{

    /// @brief Eggs are the hidden serialized form of a controller, designed to be 
    /// shipped through any byte stream. (compare to a Python pickle.)
    /// @details A Sheldrake protects its eggs. The internals of an Egg are not exposed 
    /// to library users. Sheldrake's only purpose is to translate and interpret 
    /// control policies, so any manual work with serialized control policy data is 
    /// counterintuitive to the library's purpose.
    class Egg;

    /// @brief The Sheldrake is an abstract class that provides a common interface for 
    /// serializing control policies. Subclasses of Sheldrake are responsible for
    /// transmitting Eggs over some interface.
    class Sheldrake{
        public:
        /// @brief Deploys a valid controller to an Arduino
        /// @tparam T the type of controller to deploy, one of the following:
        /// - drake::systems::controllers::PIDController
        /// - drake::systemssystems::AffineSystem<number type> (TODO
        /// @param controller 
        template<typename T>
        void deploy(const T &controller);

        protected:
        /// @brief To be implemented by every Sheldrake subclass. Serializes 
        /// an Egg and transfers it over some interface.
        /// @param egg 
        virtual void _deploy_egg(const Egg &egg) = 0;
    };


    /// @brief A Sheldrake that sends Eggs over a serial port.
    /// It's best practice to use the factory function create_Sheldrake() to
    /// create a Sheldrake_Serial instance, and to use it like a normal Sheldrake instance.
    /// If you need to use the Sheldrake_Serial constructor directly, that's an 
    /// issue with the library, so feel free to submit the issue on GitHub.
    class Sheldrake_Serial: public Sheldrake{
        public:

        /// @brief Constructs a Sheldrake_Serial instance.
        /// @param serial 
        Sheldrake_Serial(std::shared_ptr<serial::Serial> const &serial):serial(serial){}

        protected:
        /// @brief Serializes an Egg and sends it over the serial port.
        /// @param egg 
        virtual void _deploy_egg(const Egg &egg);

        /// @brief The serial port to send Eggs over.
        const std::shared_ptr<serial::Serial> serial;
    };


    // Creates a new Sheldrake instance that can send control policies over a serial port.
    std::unique_ptr<Sheldrake> create_Sheldrake(std::shared_ptr<serial::Serial> const &serial){
        return std::make_unique<Sheldrake_Serial>(serial);
    }
}