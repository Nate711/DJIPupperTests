import msgpack
import serial
import numpy as np
import glob
import platform


def pack_serialized(bytes_array, start=0x00):
    return bytes([start, len(bytes_array)]) + bytes_array


def pack_dict(dict, start=0x00):
    raw = msgpack.packb(dict, use_single_float=True)
    full_array = pack_serialized(raw, start)
    return full_array


if platform.system() == "Windows":
    serial_port = input(
        "Looks like you're on Windows, please enter"
        " the COM port for the connected Teensy (eg COM4): "
    )

else:
    serial_ports = glob.glob("/dev/tty.usbmodem*")
    if len(serial_ports) < 1:
        raise ValueError("No Teensy detected or multiple serial devices connected.")
    serial_port = serial_ports[0]

with serial.Serial(serial_port, timeout=0.2) as ser:
    try:
        # Use the zero command if you'd like to set the current position as the zero point for all motors
        print("Setting current motor positions as the zero positions.")
        ser.write(pack_dict({"zero": True}))

        # Set maximum motor current and activate motors
        print("Setting max current to 4A")
        ser.write(pack_dict({"max_current": 4.0}))
        print("Activating all motors")
        ser.write(pack_dict({"activations": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}))

        # Use these commands to test angle position control mode
        print("Setting position Kp to 8 A/rad")
        ser.write(pack_dict({"kp": 8.0}))
        print("Setting position kd to 0.02 A/rad/s")
        ser.write(pack_dict({"kd": 0.02}))
        print("Setting position setpoint to 0 rad")
        ser.write(
            pack_dict(
                {"pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
            )
        )

        # Use these commands to test cartesian position control mode
        # ser.write(pack_dict({"cart_kp": [0.0, 0.0, 0.0]}))
        # ser.write(pack_dict({"cart_kd": [4.0, 4.0, 4.0]}))
        # ser.write(
        #     pack_dict({"cart_pos": [0.1, -0.07, 0.0, 0.1, 0.07, 0.0, -0.1, -0.07, 0.0, -0.1, 0.07, 0.0]})
        # )

        # Use this command to test the feed forward force control
        # IN AMPS! # Rough ratio is 10A = 2.5Nm -> 0.25Nm/A.  It thinks this is N, going to Nm. So
        # If I want weight/4 = 4.4N per leg, then command 17.6 ?????? Seems like a lot
        # ser.write(pack_dict({"ff_force": [0.0, 0.0, -17.6, 0.0, 0.0, -17.6, 0.0, 0.0, -17.6, 0.0, 0.0, -17.6]}))

        ser.flush()
        print("Beginning to read debug messages.")
        while True:
            # Robot must be sending non-msgpack debug messages for this line to work
            print(ser.readline().decode(), end="")

            # However robot must be sending msgdpack debug messages for the --log flag to work
            # with run_djipupper.py

    except Exception as e:
        print("Exception encountered: ", e)
        print("Breaking loop. Deactivating motors")
        ser.write(pack_dict({"activations": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}))
        ser.write(pack_dict({"idle": True}))
