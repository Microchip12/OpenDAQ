
# OpenDAQ

OpenDAQ is an open-source FPGA-based data acquisition platform for characterizing experimental and novel integrated circuits (ICs). It combines high-speed differential capture hardware with an embedded Linux environment, enabling flexible data acquisition, DMA transfer, DDR storage, and on-device analysis in a low-cost system designed for research and education.

The system has been successfully used to capture data from novel experimental ICs for characterization and validation purposes.

Below is the system-level block diagram.

<img width="1947" height="1098" alt="image" src="https://github.com/user-attachments/assets/e433c2e8-fc89-4e8a-a4f8-6855eae42dab" />

OpenDAQ utilizes the Xilinx Zynq SoC, which integrates FPGA fabric with ARM processor cores. The design has been thoroughly tested and validated on the Digilent ZedBoard, though the architecture can be ported to other FPGA platforms.

The ARM processors run an embedded Linux system responsible for data management, file generation, and system supervision. In FPGA-only platforms, this role could alternatively be implemented using a soft processor core.

------------------------------------------------------------

# Architecture Overview

OpenDAQ uses a hardware–software co-design approach. High-speed signal capture occurs in FPGA hardware while system control and data management are handled by Linux.

## Hardware Data Path

The FPGA fabric performs deterministic high-speed signal acquisition.

1. Experimental Chip
   The device under test produces high-speed differential digital outputs.

2. Differential to Single-Ended Buffers
   Signals are converted into logic levels compatible with the FPGA.

3. SERDES (Deserializer)
   Incoming high-speed serial data streams are converted into parallel digital data.

4. FIFO Buffers
   FIFOs perform clock-domain crossing and temporary buffering to ensure reliable data flow.

5. DMA Engine
   Data is transferred efficiently from FPGA fabric into system memory.

6. DRAM Storage
   Captured samples are stored in external DDR memory for later retrieval.

This capture chain enables continuous, deterministic acquisition at high data rates.

------------------------------------------------------------

## Software Data Path

Once data is stored in DRAM, the embedded Linux environment handles extraction and storage.

1. Linux Driver
   Interfaces with the FPGA hardware and exposes capture buffers to the operating system.

2. Userspace Application
   Reads data from memory and organizes it into structured output.

3. Output File Generation
   Captured samples are written to disk.

4. External Computer
   Files can be transferred to a host system for analysis using MATLAB, Python, or other tools.

------------------------------------------------------------

# Performance

OpenDAQ has been validated through high-speed multi-channel data capture experiments.

Input Frequency: 250 MHz  
Channels: 3 simultaneous capture channels  
Capture Size: 32 MB per channel  
Total Data Captured: 96 MB

The system demonstrated continuous PRBS capture across all three channels without data loss, validating the end-to-end acquisition pipeline from FPGA front-end through Linux-based storage.

------------------------------------------------------------

# Output Format

Captured data is exported as CSV files.

Each column corresponds to a separate DMA capture channel, allowing simultaneous multi-channel acquisition.

These files can be analyzed using common tools such as:

- Python
- MATLAB
- Excel
- Signal processing environments

------------------------------------------------------------

# Supported Hardware

Currently validated hardware:

- Digilent ZedBoard
- Xilinx Zynq-7000 SoC

The architecture relies on standard FPGA constructs such as SERDES, FIFO buffers, and DMA engines, making the design portable to other Xilinx FPGA platforms with relatively minor modifications.

------------------------------------------------------------

# Running OpenDAQ

Running the system does NOT require installing Vivado or PetaLinux.

A prebuilt system image is provided in the Releases section of the repository.

Steps:

1. Download the .wic image from Releases
2. Flash the image to an SD card
3. Insert the SD card into the ZedBoard
4. Power on the board

The embedded Linux system will boot automatically and the OpenDAQ environment will be ready to run.

------------------------------------------------------------

# Development and Rebuilding

Users who wish to modify the system can rebuild the software environment.

The PetaLinux Board Support Package (BSP) used to generate the Linux system is located in:

/petalinux

Rebuilding the system requires:

- Xilinx Vivado
- PetaLinux

These tools are only required if you intend to modify the hardware or operating system. Running the system using the provided .wic image does not require any additional software installation.

------------------------------------------------------------

# Documentation

Additional documentation is available in the documentation folder.

Included documents:

Engineering Design Document (EDD)
Detailed description of the architecture and design implementation.

User Manual
Setup instructions and operational guidance.
