
# Sheldrake
Sheldrake (“duck” in Early Modern English) creates a *shell* of *Drake* for embedded systems. Specifically, it is a C++ toolbox that aims to interactively apply control policies written and simulated with [Drake](https://drake.mit.edu/) to embedded systems.

This is currently a proof-of-concept and only applies PID and LQR models at this point. It is tested Ubuntu 22.04 with the latest stable version of Drake

## Supported Configuration:
* Operating System: Ubuntu 22.04
* Minimum C++ Version: 20 (gcc 12+)

## Getting Started
Install Drake and other necessary dependencies
```bash
sh configure.sh
```

## How to use it