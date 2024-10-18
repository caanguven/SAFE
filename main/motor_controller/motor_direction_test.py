import RPi.GPIO as GPIO
import time
import sys

class Motor:
    def __init__(self, in1, in2, spd, name="Motor"):
        self.IN1 = in1
        self.IN2 = in2
        self.SPD = spd  # Speed pin, not used in direction testing
        self.name = name
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        # Initialize all motors to stop
        self.stop()

    def forward(self):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        print(f"{self.name} set to FORWARD")

    def reverse(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        print(f"{self.name} set to REVERSE")

    def stop(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        print(f"{self.name} STOPPED")

# GPIO Setup
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering

# Define Motors
motors = {
    1: Motor(in1=7, in2=26, spd=18, name="Motor 1"),
    2: Motor(in1=29, in2=22, spd=31, name="Motor 2"),
    3: Motor(in1=11, in2=32, spd=33, name="Motor 3"),
    4: Motor(in1=12, in2=13, spd=35, name="Motor 4"),
}

def set_motor_direction(motor_number, direction, duration=None):
    if motor_number not in motors:
        print(f"Invalid motor number: {motor_number}")
        return

    motor = motors[motor_number]

    if direction == 'forward':
        motor.forward()
    elif direction == 'reverse':
        motor.reverse()
    elif direction == 'stop':
        motor.stop()
    else:
        print(f"Invalid direction: {direction}")
        return

    if duration:
        time.sleep(duration)
        motor.stop()

def print_menu():
    print("\n=== Motor Test Menu ===")
    print("1. Motor 1")
    print("2. Motor 2")
    print("3. Motor 3")
    print("4. Motor 4")
    print("5. All Motors")
    print("6. Exit")

def print_direction_menu():
    print("\nSelect Direction:")
    print("1. Forward")
    print("2. Reverse")
    print("3. Stop")
    print("4. Back to Main Menu")

def main():
    try:
        while True:
            print_menu()
            choice = input("Select a motor to test (1-6): ")

            if choice in ['1', '2', '3', '4']:
                motor_num = int(choice)
                while True:
                    print_direction_menu()
                    dir_choice = input("Select direction (1-4): ")

                    if dir_choice == '1':
                        set_motor_direction(motor_num, 'forward')
                    elif dir_choice == '2':
                        set_motor_direction(motor_num, 'reverse')
                    elif dir_choice == '3':
                        set_motor_direction(motor_num, 'stop')
                    elif dir_choice == '4':
                        break
                    else:
                        print("Invalid choice. Please select again.")

            elif choice == '5':
                while True:
                    print_direction_menu()
                    dir_choice = input("Select direction for ALL motors (1-4): ")

                    if dir_choice == '1':
                        for m in motors.values():
                            m.forward()
                    elif dir_choice == '2':
                        for m in motors.values():
                            m.reverse()
                    elif dir_choice == '3':
                        for m in motors.values():
                            m.stop()
                    elif dir_choice == '4':
                        break
                    else:
                        print("Invalid choice. Please select again.")

            elif choice == '6':
                print("Exiting Motor Test.")
                break
            else:
                print("Invalid choice. Please select again.")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")

    finally:
        # Stop all motors and clean up GPIO
        for m in motors.values():
            m.stop()
        GPIO.cleanup()
        print("GPIO cleaned up. Program terminated.")

if __name__ == "__main__":
    main()
