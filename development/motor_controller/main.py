import RPi.GPIO as GPIO
from motor import Motor
from pid_controller import PIDController
from spike_filter import SpikeFilter
from adc_reader import ADCReader
from sawtooth_wave_generator import SawtoothWaveGenerator
from motor_controller import MotorController

import threading
import time
import argparse  # Import argparse module

def run_motor_controller(motor_controller, stop_event, direction):
    motor_controller.control_loop(stop_event, direction)

def main():
    try:
        # Set up argument parser
        parser = argparse.ArgumentParser(description='Motor Control Program')
        parser.add_argument('direction', choices=['forward', 'reverse'], help='Direction to move the motor')
        parser.add_argument('gait_type', nargs='?', default='default', help='Type of gait to use')
        args = parser.parse_args()


        # Extract the direction
        direction = args.direction
        gait_type = args.gait_type

        # Set up GPIO
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering

        # Create ADCReader instance
        adc_reader = ADCReader(spi_port=0, spi_device=0)

        # Shared configuration
        config = {
            'dead_zone_start': 330,
            'offset': 5,
            'num_samples_for_average': 5,
            'slowdown_threshold': 20,
            'max_control_change': 5,  
            'max_degrees_per_second': 60  
        }

        # PID constants (tune as necessary)
        Kp = 0.1
        Ki = 0.01
        Kd = 0.1

        # Define motors and their configurations
        motors_info = [
            {
                'id': 1,
                'name': 'Motor 1',
                'in1': 7,
                'in2': 26,
                'spd': 18,
                'adc_channel': 0,
            },
            {
                'id': 2,
                'name': 'Motor 2',
                'in1': 29,
                'in2': 22,
                'spd': 31,
                'adc_channel': 1,
            },
            {
                'id': 3,
                'name': 'Motor 3',
                'in1': 11,
                'in2': 32,
                'spd': 33,
                'adc_channel': 2,
            },
            {
                'id': 4,
                'name': 'Motor 4',
                'in1': 12,
                'in2': 13,
                'spd': 35,
                'adc_channel': 3,
            },
        ]


        # Create a list to hold motor controllers
        motor_controllers = []
        stop_event = threading.Event()

        # For each motor, create the necessary instances
        for motor_info in motors_info:
            # Create Motor instance
            motor = Motor(motor_info['in1'], motor_info['in2'], motor_info['spd'])

            # Create PIDController instance
            pid_controller = PIDController(Kp, Ki, Kd)

            # Create SpikeFilter instance
            spike_filter = SpikeFilter()

            # Initialize initial_angle based on gait_type
            if gait_type == 'trot':
                if motor_info['id'] in [1, 4]:  # Diagonal pair 1
                    initial_angle = 0
                elif motor_info['id'] in [2, 3]:  # Diagonal pair 2
                    initial_angle = 180
            elif gait_type == 'gallop':
                # Implement logic for gallop gait
                pass
            elif gait_type == 'pace':
                # Implement logic for pace gait
                pass
            else:
                # For default gait, initialize from potentiometer
                initial_pot_value = adc_reader.read_channel(motor_info['adc_channel'])
                initial_angle = initial_pot_value * (360 / 1023)

            # Ensure initial_angle is within [0, 360)
            initial_angle = initial_angle % 360

            # Create SawtoothWaveGenerator instance with direction
            sawtooth_generator = SawtoothWaveGenerator(
                period=1000,
                amplitude=360,
                initial_angle=initial_angle,
                direction=direction  # Pass the direction here
            )

            # Create MotorController instance
            motor_controller = MotorController(
                motor=motor,
                pid_controller=pid_controller,
                adc_reader=adc_reader,
                channel=motor_info['adc_channel'],
                spike_filter=spike_filter,
                sawtooth_generator=sawtooth_generator,
                config=config,
                name=motor_info['name']  # Pass the name for logging
            )

            # Add to the list of motor controllers
            motor_controllers.append(motor_controller)


        # Create and start threads for each motor controller
        threads = []
        for mc in motor_controllers:
            t = threading.Thread(target=run_motor_controller, args=(mc, stop_event, direction))
            t.start()
            threads.append(t)

        # Wait for KeyboardInterrupt to stop
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program stopped by user")
        stop_event.set()  # Signal all threads to stop
        for t in threads:
            t.join()  # Wait for all threads to finish
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()
