import msgpack
import serial
import numpy as np
import glob


def pack_serialized(bytes_array, start=0x00):
    return bytes([start, len(bytes_array)]) + bytes_array


def pack_dict(dict, start=0x00):
    raw = msgpack.packb(dict, use_single_float=True)
    full_array = pack_serialized(raw, start)
    return full_array

# Pretty sure this glob pattern only works on mac, otherwise you'll need to change it to correctly find the Teensy.
serial_port = glob.glob("/dev/tty.usbmodem*")[0]
with serial.Serial(serial_port, timeout=0.2) as ser:
    try:
        # Use the zero command if you'd like to set the current position as the zero point for all motors
        # ser.write(pack_dict({"zero": False}))
        
        # Set maximum motor current and activate motors
        ser.write(pack_dict({"max_current": 4.0}))
        ser.write(pack_dict({"activations": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}))

        # Use these commands to test angle position control mode
        ser.write(pack_dict({"kp": 8.0}))
        ser.write(pack_dict({"kd": 0.02}))
        ser.write(
            pack_dict({"pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})
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
        while True:
            # Robot must be sending non-msgpack debug messages for this to work
            print(ser.readline().decode(), end="")

            # However robot must be sending msgdpack debug messages for the --log flag to work
            # with run_djipupper.py
    except:
        print("Breaking loop.")
        ser.write(pack_dict({"activations": [0,0,0,0,0,0,0,0,0,0,0,0]}))
        ser.write(pack_dict({"idle":True}))
