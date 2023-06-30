// /*
// The Sheldrake library has an internal, unified representation for serializing control policies, called "Eggs" (i.e. Sheldrakes produce Eggs).

// This implementation code handles converting Drake control policies to Eggs, serializing those eggs, and sending serialized data to an embedded system.

// Implementation is split into three sections:
//  - Egg type def
//     - Defining the Egg type
//     - Utilities on Eggs
//  - Building eggs
//     - generate_egg_<insert_control_policy_here>: a function that converts a Drake control policy to an Egg. Current list:
//         - generate_egg_PID
//  - Deploying eggs
//     - deploy_<insert_communication_protocol_here>: a function that sends an Egg to an embedded system via a certain communication protocol. Current list:
//         - deploy_serial


// The Sheldrake object is a clean wrapper for these utilities, so library users don't have to worry about Eggs (i.e. the Sheldrake protects its eggs).
// */



// #include "sheldrake.hpp"
// #include <bit>
// #include <exception>



// // Include stdfloat if available to enforce IEEE 754-compliance (since most embedded systems also follow IEEE 754)
// #if __cplusplus >= 202300L
//     #define USE_STDFLOAT 1
// #endif
// // Not sure if this will work. Compiler-specific alternative:
// // #ifdef __clang__
// //     #if __clang_major__ >= 16
// //         #define USE_STDFLOAT 1
// //     #endif
// // #else
// //     #if __GNUC__ >= 13
// //         #define USE_STDFLOAT 1
// //     #endif
// // #endif
// #ifdef USE_STDFLOAT
//     #include <stdfloat>
// #endif


// namespace sheldrake{

// // Define the egg_floating_point_type
// #ifdef USE_STDFLOAT
// using egg_floating_point_type=std::float32_t;
// #else
// using egg_floating_point_type=float;
// #endif
// using egg_floating_vector_type=std::vector<egg_floating_point_type>;
// using egg_body_type=std::vector<uint8_t>;







// /*
//   ______                     _                                  _           __ 
//  |  ____|                   | |                                | |         / _|
//  | |__      __ _    __ _    | |_   _   _   _ __     ___      __| |   ___  | |_ 
//  |  __|    / _` |  / _` |   | __| | | | | | '_ \   / _ \    / _` |  / _ \ |  _|
//  | |____  | (_| | | (_| |   | |_  | |_| | | |_) | |  __/   | (_| | |  __/ | |  
//  |______|  \__, |  \__, |    \__|  \__, | | .__/   \___|    \__,_|  \___| |_|  
//             __/ |   __/ |           __/ | | |                                  
//            |___/   |___/           |___/  |_|                                  
// */
// /// @brief The structure for passing control policy objects to embedded systems
// struct Egg{

//     // The start of an egg, easy to parse
//     const char prefix='s';

//     // The type of control policy
//     enum ControlPolicyType{
//         INVALID                = 0,
//         PID                    = 1,
//         GAIN                   = 2,
//         LINEAR_TIME_INVARIANT  = 3
//     };
//     enum ControlPolicyType type;

//     // The length of the body
//     uint8_t body_length;

//     // The meat and potatoes of this egg (hungry yet?)
//     egg_body_type body;

//     // A checksum of all the elements in "body", equal to the sum of all the elements of "body"
//     uint16_t checksum = 0;

//     // The end of the egg
//     const char suffix='d';
// };


// namespace eggs{


// /// @brief Ensures an egg is valid
// /// @param egg 
// void validate(Egg &egg){
//     using namespace std;

//     // Type
//     if(egg.type==egg.INVALID) throw runtime_error("Egg type hasn't been defined");

//     // Body length
//     if(egg.body.size()>255) throw runtime_error("Egg size too large (exceeds maximum of 255 bytes)");
//     egg.body_length = egg.body.size();

//     // Checksum
//     egg.checksum = 0;
//     for(auto i: egg.body) egg.checksum += i;
// }


// std::vector<uint8_t> serialize(const Egg &egg_arg){

//     // Make a mutable validated copy of the egg
//     Egg egg = egg_arg;
//     validate(egg);

//     std::vector<uint8_t> body;

//     body.push_back(egg.prefix);
//     body.push_back(egg.type);
//     body.push_back(egg.body_length);
//     body.insert(body.end(), egg.body.begin(), egg.body.end());
//     body.push_back(egg.checksum);
//     body.push_back(egg.checksum >> 8);
//     body.push_back(egg.suffix);

//     return body;
// }

// void append_float(egg_body_type &body, egg_floating_point_type value){
//     const int n = sizeof(egg_floating_point_type);
//     union {
//         egg_floating_point_type float_variable;
//         uint8_t temp_array[n]; 
//     } u;
//     u.float_variable = value;
//     for(int i = 0; i < n; i++){
//         body.push_back(u.temp_array[i]); 
//     }
// }


















// /*

// -----------------------------------Building Eggs-------------------------------------------

//   ____            _   _       _   _                     ______                       
//  |  _ \          (_) | |     | | (_)                   |  ____|                      
//  | |_) |  _   _   _  | |   __| |  _   _ __     __ _    | |__      __ _    __ _   ___ 
//  |  _ <  | | | | | | | |  / _` | | | | '_ \   / _` |   |  __|    / _` |  / _` | / __|
//  | |_) | | |_| | | | | | | (_| | | | | | | | | (_| |   | |____  | (_| | | (_| | \__ \
//  |____/   \__,_| |_| |_|  \__,_| |_| |_| |_|  \__, |   |______|  \__, |  \__, | |___/
//                                                __/ |              __/ |   __/ |      
//                                               |___/              |___/   |___/       


// */

// // PID controller
// template<typename T> 
// Egg generate_egg(const drake::systems::controllers::PidController<T> &controller){
//     /*
//     PID eggs follow this structure:
//         - uint8_t  n    (not necessary because Eggs also contain their size)
//         - float[]  K_p  (an n-dimensional vector)
//         - float[]  K_i  (an n-dimensional vector)
//         - float[]  K_d  (an n-dimensional vector)
//     */ 
//     using namespace std;

//     Egg egg;

//     // Sanity checks
//     if(controller.get_Kd_vector().size()>255) throw runtime_error("Large PID controllers (>255 dimensions) are not supported yet");
//     uint8_t size = controller.get_Kd_vector().size();

//     // Set the egg type to "PID"
//     egg.type = 1; // 1 is the unique code for PID controllers

//     // Dump PID data
//     egg.body.push_back(size);                                        // n
//     for(auto i: controller.get_Kp_vector) append_float(egg.body, i); // K_p
//     for(auto i: controller.get_Ki_vector) append_float(egg.body, i); // K_i
//     for(auto i: controller.get_Kd_vector) append_float(egg.body, i); // K_d

//     // validate() can handle the rest. If an exception is thrown here, Sheldrake has a problem.
//     eggs::validate(egg);

//     return egg;
// }




// // Affine systems (TODO)
// template<typename T> 
// Egg generate_egg(const drake::systems::AffineSystem<T> &controller){
//     // Affine systems follow this structure:
//     //     X_dot = A * X + B * U
//     //     Y     = C * X + D * U
//     // Where X is an n-dimensional vector, U is an m-dimensional vector, and Y is a p-dimensional vector.

//     /*
//     Affine eggs follow this structure:
//         - uint8_t  n
//         - uint8_t  m
//         - uint8_t  p
//         - float[]  A  (an n*n-dimensional vector)
//         - float[]  B  (an n*m-dimensional vector)
//         - float[]  C  (a p*n-dimensional vector)
//         - float[]  D  (a p*m-dimensional vector)
//     */ 
//     using namespace std;

//     Egg egg;

//     // TODO stub

//     return egg;
// } // end generate_egg_PID


// /// @brief Generates an Egg from a Drake controller
// /// @tparam T one of the Drake controller types listed in the documentation for deploy()
// /// @param t 
// template<typename T>
// void _ensure_valid_controller_type(const T& t){
//     constexpr bool can_make_egg = requires(const T& t) {
//         generate_egg(t);
//     };
//     static_assert(can_make_egg, "This type of controller cannot be converted to an Egg");
// } // end _ensure_valid_controller_type


// } // end namespace eggs























// /*

// ----------------------------------Deploying Eggs----------------------------------------------


//   _____                   _                   _                     ______                       
//  |  __ \                 | |                 (_)                   |  ____|                      
//  | |  | |   ___   _ __   | |   ___    _   _   _   _ __     __ _    | |__      __ _    __ _   ___ 
//  | |  | |  / _ \ | '_ \  | |  / _ \  | | | | | | | '_ \   / _` |   |  __|    / _` |  / _` | / __|
//  | |__| | |  __/ | |_) | | | | (_) | | |_| | | | | | | | | (_| |   | |____  | (_| | | (_| | \__ \
//  |_____/   \___| | .__/  |_|  \___/   \__, | |_| |_| |_|  \__, |   |______|  \__, |  \__, | |___/
//                  | |                   __/ |               __/ |              __/ |   __/ |      
//                  |_|                  |___/               |___/              |___/   |___/       



// */



// void Sheldrake_Serial::_deploy_egg(const Egg &egg){
   
//     using namespace std;

//     // If float32_t is not defined, check if the egg_floating_point_type
//     #ifndef USE_STDFLOAT
//         static_assert(numeric_limits<egg_floating_point_type>::is_iec559, "Unsupported OS/Architecture/Compiler: Can't find IEEE 754-compliant floating point numbers");
//     #endif

//     // Ensure this is a little-endian system (for now. Something to look into when someone creates a new GitHub issue with this error)
//     static_assert(std::endian::native == std::endian::little, "Eggs are little-endian, but this library is not being compiled for a little-endian system, and endianness conversion is not supported yet.");


//     // Ensure this is a little-endian system for now
//     static_assert(sizeof(egg_floating_point_type) == 4, "Eggs require 32-bit floating-point numbers, but egg_floating_point_type is not 4 bytes.");

//     // Ensure this is still an open connection
//     if(!serial->isOpen()) throw runtime_error("Attempted to use a serial port while calling deploy_serial, but the serial port was not open");

//     // Deploy the egg
    
//     std::vector<uint8_t> serialized_egg = eggs::serialize(egg);
//     serial->write(serialized_egg);

// } // end Sheldrake_Serial::_deploy_egg










// /*
//   _____            _       _   _                    __                  _                 
//  |  __ \          | |     | | (_)                  / _|                (_)                
//  | |__) |  _   _  | |__   | |  _    ___   ______  | |_    __ _    ___   _   _ __     __ _ 
//  |  ___/  | | | | | '_ \  | | | |  / __| |______| |  _|  / _` |  / __| | | | '_ \   / _` |
//  | |      | |_| | | |_) | | | | | | (__           | |   | (_| | | (__  | | | | | | | (_| |
//  |_|       \__,_| |_.__/  |_| |_|  \___|          |_|    \__,_|  \___| |_| |_| |_|  \__, |
//                                                                                      __/ |
//                                                                                     |___/ 
// */

// template<typename T>
// void Sheldrake::deploy(const T &controller)
// {

//     // Ensure this is a valid controller type
//     _ensure_valid_controller_type(controller);

//     // Create an Egg from this controller
//     Egg egg = eggs::generate_egg(controller);

//     // Now deploy the egg
//     _deploy_egg(egg);

// } // end Sheldrake::deploy








// } // end namespace sheldrake