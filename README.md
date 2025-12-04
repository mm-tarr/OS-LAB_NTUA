# Operating Systems Lab - Lab 2: Lunix TNG

This repository contains the implementation for Lab 2 of the Operating Systems course. The project focuses on developing the **Lunix:TNG** driver, a Linux kernel module for handling sensor data from a wireless sensor network.

> **Note:** Most of the codebase is provided by the course instructors. The original author(s) of each file are credited in the header comments of the respective source files.

## Project Structure

The project is organized as follows:

### 1. Core Implementation (Root Directory)
The main files for the Lunix:TNG driver and helper tools are located in the root directory:
- **Kernel Module Code**:
  - `lunix-chrdev.c`: Character device driver implementation.
  - `lunix-ldisc.c`: Line discipline for the serial interface.
  - `lunix-protocol.c`: Protocol handling for sensor data.
  - `lunix-sensors.c`: Management of sensor state and buffers.
- **Helper Tools**:
  - `lunix-attach.c`: A userspace utility to attach the line discipline to a serial port.
  - `mk-lunix-devs.sh`: A script to create the necessary device nodes in `/dev`.

### 2. `lunix-monitor/`
A command-line interface (CLI) tool that displays real-time data from the sensors.

## Building

To build the kernel module , navigate to the root directory of the repo and run `make`:

```bash
make
```

This will generate the `lunix.ko` kernel module and the `lunix-attach` executable.

To build the monitor tool, navigate to the `lunix-monitor` directory and run `gcc -o lunix-monitor lunix-monitor.c -lncurses`

```bash
gcc -o lunix-monitor lunix-monitor.c -lncurses
```

## Usage

Follow these steps to set up and run the Lunix:TNG driver:

### 1. Create Device Nodes
Before loading the module, you need to create the character device nodes for the sensors. Run the provided script with root privileges:

```bash
sudo ./mk-lunix-devs.sh
```
This creates nodes like `/dev/lunix0-temp`, `/dev/lunix0-batt`, etc.

### 2. Load the Module
Insert the compiled kernel module into the kernel:

```bash
sudo insmod lunix.ko
```

### 3. Attach the Line Discipline
To start receiving data from the sensors, you need to attach the Lunix line discipline to the serial port where the base station is connected (e.g., `/dev/ttyS0`).
> **Serial Port Connection**: In the context of this exercise the data from the sensors are recieved with the use of a TCP socket that is attached to the Serial Port of linux.
```bash
./lunix-attach /dev/ttyS0
```
> **Note:** If there is access to only one terminal is recommeended to attach this in the background with `&`

Once attached, the driver will begin processing incoming sensor data, which can then be read from the `/dev/lunix*` nodes or viewed using the `lunix-monitor` tool.

##  Disclaimer
These codes are intended for educational purposes and reference.
