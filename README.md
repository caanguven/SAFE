# SAFE

Soft robots can adapt to dynamic environments where traditional rigid robots fall short. Existing soft quadruped robots lack effective remote control capabilities and often struggle with maintaining stable locomotion over long distances, limiting their practical application in real-world scenarios. This research addresses the challenges by improving the controllability and reliability of soft quadruped robots for use in search and rescue operations.

I developed the Soft Adaptive Field Explorer (SAFE), a tetherless soft quadruped robot equipped with onboard control interfaces and integrated sensors. A user-controlled web-based interface allows operators to interact with and control SAFE remotely. The integration of sensors such as cameras, inertial measurement units, and absolute rotary encoders optimizes gait performance through closed-loop feedback control. A comparative analysis of soft, rigid, and semi-soft leg materials evaluates their impact on stability, efficiency, and adaptability, identifying the optimal balance for diverse terrains.

This research demonstrates that incorporating the web-based interface and onboard sensor feedback improves the robot’s stability and navigational accuracy. By improving the adaptability and controllability of soft quadruped robots, this work bridges the gap between current soft robot capabilities and potential requirements for use in disaster response applications.

## Project Overview

The Advanced Quadruped project focuses on developing a robust and adaptable quadruped robot, SAFE, designed for challenging environments. The project includes the following key components:

- **Motor Control**: Utilizing Raspberry Pi and motor controllers to achieve precise movement and control of the quadruped's legs.
- **PID Control**: Implementing PID controllers to maintain stable locomotion and optimize gait performance.
- **Sensor Integration**: Incorporating various sensors such as cameras, IMUs, and rotary encoders to provide real-time feedback and enhance control accuracy.
- **Web-Based Interface**: Developing a user-friendly web interface for remote control and monitoring of the robot.



## Project Structure

The project is organized into the following directories and files:

### Directories

- **data_collect/**: Includes code used to collect data while operating the robot.
- **development/**: Includes an interface used at developemnt. By running the main.py you can access the main interface at port 5000. Using this interface we can access to various functionalies of SAFE. This is only for development purposes. Some codes might not work as expected.  
- **motor_controller_dev/**: Includes the directores that includes the motor controller such as square follow, April Tag detectino.
- **web_based_drive/**: Main teleoperated controller of the SAFE. Running main.py should start a fully functional Flask server that will provide IMU Camera and encoder data to user. This is the most optimzated way to operate SAFE. 
- **others/**: Contains additional scripts and files.


## Installation

To run this project, you need to have the following dependencies installed:

- `RPi.GPIO`
- `Adafruit_GPIO`
- `Adafruit_MCP3008`
- `solidpython`

You can install the required Python packages using pip. 


## Usage

To operate the SAFE robot, follow these steps:

1. **Setup**: Ensure all hardware components are properly connected and powered on. Verify that the Raspberry Pi is connected to the same network as your control device.
2. **Start the Server**: Navigate to the `web_based_drive` directory and run `main.py` to start the Flask server.
    ```sh
    cd /path/to/web_based_drive
    python main.py
    ```
3. **Access the Interface**: Open a web browser and navigate to `http://<Raspberry_Pi_IP>:5000` to access the web-based control interface.
4. **Control the Robot**: Use the interface to control the robot's movements, monitor sensor data, and adjust settings as needed.

## Contributing

Contributions to the SAFE project are welcome. If you have suggestions for improvements or new features, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bugfix.
3. Commit your changes and push the branch to your fork.
4. Submit a pull request with a detailed description of your changes.
