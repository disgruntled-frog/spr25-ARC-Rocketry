
### Definition of Done

**Tasks are to be marked as done only when these criteria are finalized:**
- Parts and tasks are properly documented with a what, who, when, and why
- New tasks derived from the task at hand are finalized as well (any dependent task is also complete before task can be considered done)

- [ ] **Components & Passives** due: 1/6/25
	- [ ] GPS + Antenna
	- [ ] IMU
	- [ ] Barometric Pressure Sensor
	- [ ] NOR Flash Chip
	- [ ] Servo
	- [ ] Radio Module
	- [ ] Power Distribution
	- [ ] Battery
		- [ ] Connector
	- [ ] USB-C Connector
	- [ ] JTAG (SWD)
	- [ ] Indicator Lights
		- [ ] Potential Breakout Board
	- [ ] External Crystal Oscillator
- [ ] **Schematic** due: 1/13/25
	- [ ] Communication
		- [ ] I2C Bus
		- [ ] SPI Bus
		- [ ] UART Bus
		- [ ] SWD
		- [ ] USB
	- [ ] Power / Ground
		- [ ] Radio
		- [ ] 3v3 Components
		- [ ] 5v Components
- [ ] **PCB Design** due: 1/27/25
	- [ ] Layer Stack-up
	- [ ] Power
	- [ ] Ground
	- [ ] Communication
		- [ ] I2C
		- [ ] SPI
		- [ ] UART
		- [ ] USB-C
		- [ ] SWD
	- [ ] **IMPEDANCE MATCHING BOARD REVIEW**
		- [ ] Saturn PCB design calculator
	- [ ] Test vias
	- [ ] Stitching vias
	- [ ] Ensure analog signal integrity
		- [ ] Seperate digital, high-speed digital (USB), and analog (GPS?)

### Software Roadmap & Deadlines
- [ ] Blinking Indicator LEDs
- [ ] Communication between MCU & Sensors
	- [ ] I2C Bus
	- [ ] SPI Bus
	- [ ] UART Bus
	- [ ] SWD
- [ ] Communication between MCU & PC
- [ ] **(As time allows)** 
	- [ ] Communication via Radio Datalink to Base station
		- [ ] If we are to implement buttons and commands, we will need 2-way communication
	- [ ] Base station UI 
		- [ ] Processing telemetry data and display on live station
- [ ] Data Logging
	- [ ] Writing data to NOR/NAND Flash Chip (Recently looking more into NAND for $ and storage)
	- [ ] Reading data through USB-C to personal PC (consider using [[USB VCP]])
	- [ ] (optional) Reading data to base station via Radio Datalink
- [ ] State Detection
	- [ ] IMU will send interrupt for ignition, and for ballistic trajectory via interrupt pin
		- 
- [ ] Airbrake Control Loop
	- [ ] PID Controller loop
	- [ ] Function to calculate the drag / deceleration of the rocket compared with its velocity and extrusion and feed that into the control system
	- [ ] At Burnout, extrude to set amount and then adjust based off of that
	- [ ] Maybe add in something where as you approach 1000' it gets more aggressive with its error correction

### **TESTING & VALIDATION**

This section is incredibly important, but I have no idea how to expand upon this yet at the moment, anyone who wants to add in their own tests, simulations, or validation, please do.

- [ ] Software in the loop
	- [ ] Simulating sensor data and seeing what the uC does as a response
- [ ] Simulink / Matlab Simulation
	- [ ] Test Control Systems
	- [ ] Can be set up in a way to support future iterations with alternative active control systems such as, but not limited to: Reaction wheels, Control Surfaces, Thrust Vector Control, Canards, etc.
