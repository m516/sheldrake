#pragma once



#ifdef USING_DRAKE_LIBRARY
#include <drake/systems/controllers/pid_controller.h>
#endif

#include "TinyControlPolicies.hpp"
#include "TCP_states.hpp"
#include "Serializer.hpp"



namespace sheldrake{

    struct ByteStream{
        /// @brief A pointer to a function that returns true if the stream is open.
        bool (*is_open)() = nullptr;
        /// @brief A pointer to a function that writes data over some interface.
        void (*write)(const uint8_t data) = nullptr;
    };

    template<typename Model_Type, tmm::Size num_observable_states>
    class Host{
        public:
        /// @brief A bytestream that can be used to communicate with a physical controller (a Peripheral).
        ByteStream& stream;
        /// @brief The permenant ID of this controller/peripheral pair
        const uint8_t id;

        /// @brief  The model of the controller.
        Model_Type model;
        /// @brief The most recent measurements of the observable states.
        const tcp::States<num_observable_states>& observable_states = state_deserializer.object;

        /// @brief Construct a new Host object
        /// @param stream Create a custom sheldrake::Bytestream object by implementing its two functions.
        /// @param id The permenant, ID of this controller/peripheral, unique to the system.
        /// @note There can only be 256 unique Hosts/Peripherals that can be transmitted though a single ByteStream.
        Host(ByteStream& stream, uint8_t id): stream(stream),id(id),model_serializer(id),state_deserializer(id){};
        
        /// @brief If a peripheral is publishing its state, that state can be read 
        /// with this function. Every time a byte is read from some byte stream, 
        /// pass that new byte through this function. 
        /// @param incoming_byte The byte that was received from the peripheral.
        /// @return true if the state was successfully read.
        bool read_observable_state(uint8_t incoming_byte){
            return state_deserializer.deserialize(incoming_byte);
        }

        /// @brief Sends the model over the ByteStream to a peripheral.
        void send_model(){
            model_serializer.serialize(model,stream.write);
        }
        
        private:
        /// @brief A serializer for the model.
        serializer::Serializer<Model_Type> model_serializer;
        /// @brief A deserializer for the observed states.
        serializer::Deserializer<tcp::States<num_observable_states>> state_deserializer;
    };

    template<typename Model_Type, tmm::Size num_observable_states>
    class Peripheral{
        public:
        /// @brief A bytestream that can be used to communicate with a Host.
        ByteStream& stream;
        /// @brief The permenant ID of this controller/peripheral pair
        const uint8_t id;

        /// @brief  The model of the controller.
        Model_Type& model = model_deserializer.object;
        /// @brief The most recent measurements of the observable states.
        const tcp::States<num_observable_states> observable_states;

        /// @brief Construct a new Host object
        /// @param stream Create a custom sheldrake::Bytestream object by implementing its two functions.
        /// @param id The permenant, ID of this controller/peripheral, unique to the system.
        /// @note There can only be 256 unique Hosts/Peripherals that can be transmitted though a single ByteStream.
        Peripheral(ByteStream& stream, uint8_t id): stream(stream),id(id),model_deserializer(id),state_serializer(id){};
        
        /// @brief If a peripheral is publishing its state, that state can be read 
        /// with this function. Every time a byte is read from some byte stream, 
        /// pass that new byte through this function. 
        /// @param incoming_byte The byte that was received from the peripheral.
        /// @return true if the state was successfully read.
        bool read_model(uint8_t incoming_byte){
            return model_deserializer.deserialize(incoming_byte);
        }

        /// @brief Sends the model over the ByteStream to a peripheral.
        void send_observable_state(){
            state_serializer.serialize(observable_states,stream,ByteStream::write);
        }
        
        private:
        /// @brief A deserializer for the model.
        serializer::Deserializer<Model_Type> model_deserializer;
        /// @brief A serializer for the observed states.
        serializer::Serializer<tcp::States<num_observable_states>> state_serializer;
    
    };


    #ifdef USING_DRAKE_LIBRARY

    /// @brief Casts a Drake PID controller to a TCP PID controller.
    /// @tparam Drake_Type The datatype of each element in the Drake PID controller.
    /// @tparam Model_Type The datatype of each element in the TCP PID controller.
    /// @tparam num_pid_controllers The number of parallel PID controllers in the TCP PID controller.
    /// @param model The Drake PID controller to cast.
    /// @return A TCP PID controller.
    template<tmm::Size num_pid_controllers, typename Drake_Type = float, typename Model_Type = float>
    tcp::Controller_PID<num_pid_controllers,Model_Type>
    to_tcp(drake::systems::controllers::PidController<Drake_Type> &model){
        /*
        Make a cast
                            FROM
        drake::systems::controllers::PidController<Drake_Type>
                             TO
        tcp::Controller_PID<num_pid_controllers,Model_Type>
        */
        
        tcp::Controller_PID<num_pid_controllers,Model_Type> casted_model;
        auto Kp = model.get_Kp_vector();
        if(Kp.size() < num_pid_controllers){
            throw std::runtime_error("Kp vector must match the number of gains in the PID controller.");
        }
        for(int i = 0; i < num_pid_controllers; i++){
            casted_model.Kp[i][0] = Kp[i];
        }
        return casted_model;
    }

    /// @brief This is a shortcut to create a Host for TCP PID controllers given a Drake PIDController. 
    /// It may be useful for transmitting PID controller parameters over an arbitrary connection to embedded systems
    /// or other systems that can't employ the Drake library and/or Eigen, since the TinyControlPolicy is more portable.
    /// @tparam Drake_Type The datatype of each element in the Drake PID controller.
    /// @tparam Model_Type The datatype of each element in the TCP PID controller. (the model being transmitted)
    /// @tparam num_pid_controllers the number of PID controllers in the TCP PID controller. It must be less than or equal to the number of gains in the Drake PID controller.
    /// @tparam num_observable_states The number of states that the peripheral can sense/observe.
    /// @param stream a bytestream that can be used to communicate with a peripheral.
    /// @param id The permenant ID of this controller/peripheral, unique to the system.
    /// @param model The Drake PID controller to cast.
    /// @return A Host for TCP PID controllers, initially set to the closest approximation of `model` after casting the data from `Drake_Type` to `Model_Type`.
    template<tmm::Size num_pid_controllers, tmm::Size num_observable_states, typename Drake_Type = float, typename Model_Type = float>
    Host<tcp::Controller_PID<num_pid_controllers,Model_Type>, num_observable_states> 
    create(ByteStream& stream, uint8_t id, drake::systems::controllers::PidController<Drake_Type> &model){
        // Make a host with the new casted model
        Host<tcp::Controller_PID<num_pid_controllers,Model_Type>, num_observable_states> host(stream, id);
        host.model = to_tcp<num_pid_controllers,Drake_Type,Model_Type>(model);
        return host;
    }

    #endif

} // namespace sheldrake