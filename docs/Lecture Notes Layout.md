# Lecture Notes Layout

## Licensing
## Acknowledgements
## Preamble
## Table of Contents
## Table of Examples
## Table of Figures
## Table of Equations
## List of Acronyms
---
## Learning Material Setup
 - [X] Autodesk Fusion360
 - [ ] [Optional] Fritzing
 - [ ] [Optional] DrawIO
 - [X] Arduino IDE
 - [X] [Optional] VS Code
 - [ ] [Optional] Git and GitHub
 - [X] Arduino Kit Introduction
	 - [X] Microcontroller
	 - [X] MAX7219 LED Module
	 - [X] Real Time Clock Module
	 - [X] Matrix Keyboard
	 - [X] PIR Motion Sensor
	 - [X] IR receiver
	 - [X] LCD screen
     - [X] Joystick module
	 - [X] Ultrasonic distance sensor
	 - [X] Sound receiver module
	 - [X] ULN2003 motor driver board
	 - [X] DHT-11 Sensor module
	 - [X] Water level sensor
     - [X] 7-Segment display
---
## Digital Electronics
### Introduction to Binary
 - [X] What is Binary?
     - [X] Example 1.1 Arduino Blink
 - [X] Binary Arithmetic
 - [X] Binary Conversions
     - [X] Octal
     - [X] Hexadecimal
### Introduction to Logic Gates
 - [X] Truth Tables
 - [X] Basic Logical Operations
     - [X] Example: Basic Gates 1
     - [X] Example: Basic Gates 2
 - [X] Combination Logical Operations
 - [X] Advanced Logical Operations
     - [X] Example: Advanced Gates
     - [ ] Exercise: Arduino and Logic Gates
### Boolean Algebra
 - [X] Logical Optimization
     - [X] Karnaugh maps
### Multiplexing
 - [ ] Demultiplexing
### Registers
 - [ ] Register Basics
 - [ ] Shift register
 - [ ] Example 1.4 Arduino Shift Register
### Analog-to-Digital Conversion
 - [ ] ADC Resolution
 - [ ] Example 1.5 Analog Voltage Measurement
 - [ ] Digital-to-Analog Conversion
### State Machines
 - [ ] Example 1.6 Arduino RGB LED State Machine
### TTL Digital Communications
 - [ ] Universal Asynchronous Receive and Transmit (UART)
 - [ ] RS232
 - [ ] Inter-Integrated Circuit (I2C)
 - [ ] Serial Peripheral Interface (SPI)
### Differential Digital Communications
 - [ ] RS485/Ethernet
 - [ ] Universal Serial Bus (USB)
 - [ ] Controller Area Network Bus (CANbus)
### Wireless Communications
 - [ ] Basic Principle
     - [ ] Wide Area Network (WAN)
     - [ ] Star topology
     - [ ] Mesh topology
 - [ ] Long Range (LoRa) Radios
     - [ ] LoRaWAN
 - [ ] Sigfox
 - [ ] Wireless Fidelity (WiFi)
 - [ ] BlueTooth
     - [ ] BlueTooth Low Energy
 - [ ] Zigbee
 - [ ] Cellular
 - [ ] Satellite Communications (SATCOM)
---
## Electrical Schematics
### Schematic Basics
 - [ ] Frame and Layout
 - [ ] Component symbols
     - [ ] Power symbols
         - [ ] Power source rail
         - [ ] Power sink rail (ground)
         - [ ] DC sources (battery)
         - [ ] AC sources
     - [ ] Basic components
         - [ ] Resistors
             - [ ] Potentiometer
             - [ ] Thermistor
             - [ ] Photoresistor
         - [ ] Capacitors
             - [ ] Ceramic
             - [ ] Electrolytic
         - [ ] Inductors
         - [ ] Diodes
             - [ ] Light Emitting Diodes
             - [ ] Zener diodes
         - [ ] Crystal oscillators
         - [ ] Switches
             - [ ] Buttons
             - [ ] Relays
         - [ ] Transistors
             - [ ] BJT
             - [ ] MOSFETS (P-Type, N-Type)
         - [ ] Operational amplifiers
         - [ ] External connectors
         - [ ] Common Examples
     - [ ] Integrated circuits
         - [ ] Example: Voltage Regulator
 - [ ] Placing Components
	 - [ ] Consideration: Component package and board footprint
 - [ ]  Connecting Signals
     - [ ] Aside: Bundling Signals Using Busses
 - [ ] Layers
 - [ ] Submodules
 - [ ] Electronics Rule Check (ERC)
---
## Printed Circuit Boards
### PCB Basics
 - [ ] Board Shape
     - [ ] Aside: Using sketches to derive board shape
 - [ ] Laying Out Components
     - [ ] Consideration: Grouping components of relied functionality
     - [ ] Aside: Keepout zones, restricted zones
 - [ ] Routing Traces
     - [ ] Aside: Two-Layer versus Four-Layer
     - [ ] Aside: Trace lengths, matching, thicknesses
 - [ ] Vias
     - [ ] Silkscreen
 - [ ] Generating Copper Planes
     - [ ] Design Rules Check (DRC)
 - [ ] Download Component Library
	 - [ ] Component Selection
		 - [ ] Aside: Component searching and filtering in catalog websites like DigiKey
	 - [ ] Locating a Library Source
	 - [ ] Installation in Fusion360
### Creating A Custom Library
 - [ ] Guided example with the Xsense MTi-300 AHRS
 - [ ] Component Selection and Datasheet
     - [ ] Creating a Library
     - [ ] Creating a Symbol
 - [ ] Creating a Footprint
 - [ ] Creating a 3D Package
     - [ ] Aside: Find user-generated 3D models on sites like GrabCAD
### Guided Example: Thetis Instrumentation Package
 - [ ] Generating requirements
 - [ ] Making schematic
 - [ ] Making the PCB shape
 - [ ] Placing components
 - [ ] Routing the PCB
 - [ ] Checking and Finalizing

## Software Basics and Architecture
### Arduino Basics
### Primitive Programming Datatypes
 - [ ] Arrays
### Logic Flow
 - [ ] If/Else Statement
 - [ ] Switch/Case Statement
### Loops
 - [ ] For Loops
 - [ ] While Loops
### Classes and Objects
 - [ ] Classes
 - [ ] Structs
 - [ ] Example 3.1 Using Arduino Objects
### Functions
### Interrupt Service Routines
### Guided Example: EVE Launchsonde Instrumentation Package
 - [ ] Generating Requirements
 - [ ] Visualizing Logic Flow
 - [ ] Breaking it Down into Code Blocks
 - [ ] Writing Each Code Block
 - [ ] Wrapping it All Together
### Guided Example: Thetis Instrumentation Package
 - [ ] Generating Requirements
 - [ ] Visualizing Logic Flow
 - [ ] Breaking it Down into Code Blocks
 - [ ] Writing Each Code Block
 - [ ] Wrapping it All Together
---
## Data Acquisition and Processing
### Background
- [ ] Statistics
     - [ ] Standard deviation
     - [ ] Variance
     - [ ] Distribution
     - [ ] Probability Density Functions
- [ ] Time-series Domain
     - [ ] Continuous sampling
     - [ ] Burst sampling
- [ ] Nyquist frequency
- [ ] Frequency Domain
     - [ ] Fast Fourier Transform
     - [ ] Spectrum plots
     - [ ] Bode Plots
     - [ ] Gain and phase
### Guided example: Convert water level time series into Wave Spectra
### Digital Filtering
- [ ] Kalman Filter
- [ ] Mahony Filter
- [ ] Butterworth Filter
### Guided Example: Converting Inertial Measurement Unit Readings to Orientation
- [ ] Traditional Method
- [ ] With a Kalman Filter
- [ ] With a Mahony Filter
### Guided Example: Removing Gravity from Accelerometer Signals
- [ ] Background: Quaternions and Euler angles
- [ ] Rotating the gravity vector from the world frame to body frame
- [ ] Filtering with a Kalman filter
- [ ] Embedded (Real-Time) Method
- [ ] Post-Processing
### Guided Example: Smoothing Accelerometer Signals
---
## Appendix
 - [ ] Syllabus
	
 - [ ] Demonstration: YSI Castaway
 - [ ] Demonstration: HOBO Waterlogger
 - [ ] Demonstration: Project EVE Launchsonde
 - [ ] Demonstration: Lowell and Thetis Instrumentation
 - [ ] Demonstration: Sidescan SONAR
 - [ ] Demonstration: Magnetometer
 - [ ] Demonstration: Soldering workshop

 - [ ] Exam: Arduino Programming Midterm

 - [X] Homework 1: Arduino RGB LED Cycler
 - [X] Homework 2: Digital Inputs and Interrupts
 - [X] Homework 3: 4-Digit 7-Segment Display Counter
 - [ ] Homework 4: Accelerometer and LCD
	
 - [ ] Project: Undergraduate Project Overview and Requirements
 - [ ] Project: Graduate Project Overview and Requirements
	
 - [ ] Supplement: Using a Raspberry Pi to Program Arduino Over ICSP
---
## Bibliography