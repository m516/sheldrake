
# Sheldrake
Sheldrake (“duck” in Early Modern English) creates a *shell* of *Drake* for embedded systems. Specifically, it is a C++ toolbox that aims to interactively apply control policies written and simulated with [Drake](https://drake.mit.edu/) to embedded systems.


Using custom software written for the embedded system using the Sheldrake Interpreter library: 
* Sensor data can be connected to the SI inputs (the variable `inputs` in any `cpl::Controller` class)
* The control policy is generated using the `update()` function of `cpl::Controller` class, and the results are in its `outputs` field.
* (A template is availalble [here](https://github.com/m516/sheldrake-arduino-template)).

The Sheldrake library depoloys Drake control policies to embedded systems as follows:
* The control policy is serialized.
* The serialized data is sent to an Arduino or other embedded system through a [Serial](https://github.com/wjwwood/serial) connection.
* The serialized data is interpreted through a Sheldrake Interpreter on the embedded system.

This is currently a proof-of-concept and only applies PID and LQR models at this point. It is tested Ubuntu 22.04 with the latest stable version of Drake.

## Supported Configuration:
* Operating System: Ubuntu 22.04
* Minimum C++ Version: 20 (gcc 12+)
* Little-endian (most x86-64/aarch64 CPUs)

## Getting Started
Install Drake and other necessary dependencies.

Navigate to the root folder of your local Sheldrake clone and run:
```bash
sh configure.sh
```
Building the project and all examples is similar to most other CMake projects.
```bash
mkdir -p build
cd build
cmake ..
make
```

An early prototype of a Sheldrake-enabled LQR controller on an inverted pendulum can be found [here](https://github.com/m516/sheldrake-arduino-template/).
