import numpy as np
import time
from src.IMU import IMU
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State
from spotmicro.HardwareInterface import HardwareInterface
from spotmicro.Config import Configuration
from spotmicro.Kinematics import four_legs_inverse_kinematics


def convert_to_spot_leg_coords(foot_locations):

    spot = [[0, 0, 0, 0], [60, 60, 60, 60], [220, 220, 220, 220]]
    spot_locations = spot - foot_locations
    return spot_locations


def main(use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    z_positions = np.empty(1)
    # Wait until the activate button has been pressed
    while True:
        # print("Waiting for L1 to activate robot.")
        # while True:
        #     command = joystick_interface.get_command(state)
        #     joystick_interface.set_color(config.ps4_deactivated_color)
        #     if command.activate_event == 1:
        #         break
        #     time.sleep(0.1)
        # print("Robot activated.")
        # joystick_interface.set_color(config.ps4_color)

        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = joystick_interface.get_command(state)
            if command.activate_event == 1:
                print("Deactivating Robot")
                break

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            controller.run(state, command)
            print(f'{state.ticks}: ################# Controller #################')
            print(f'{state.foot_locations}, {controller.gait_controller.contacts(state.ticks)}')
            print(f'{state.ticks}: ################# Spot #################')
            spot_foot_locations = convert_to_spot_leg_coords(state.foot_locations)
            x = spot_foot_locations[2, 0]
            z_positions = np.append(z_positions, x)
            print(f'{spot_foot_locations}')

            # Update the pwm widths going to the servos
            # hardware_interface.set_actuator_postions(state.joint_angles)

if __name__ == "__main__":
    float_formatter = "{:.2f}".format
    np.set_printoptions(formatter={'float_kind': float_formatter})
    main()
