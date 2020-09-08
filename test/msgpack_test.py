import msgpack
import serial
import numpy as np


def pack_serialized(bytes_array, start=0x00):
    return bytes([start, len(bytes_array)]) + bytes_array


def pack_dict(dict, start=0x00):
    raw = msgpack.packb(dict, use_single_float=True)#use_bin_type=True)
    # print(raw)
    full_array = pack_serialized(raw, start)
    return full_array

with serial.Serial("/dev/tty.usbmodem78075901", timeout=0.2) as ser:
    try:
        # ser.write(pack_dict({"zero": False}))
        # ser.write(pack_dict({"kp": 12.0}))
        # ser.write(pack_dict({"kd": 0.005}))
        ser.write(pack_dict({"max_current": 6.0}))
        ser.write(pack_dict({"cart_kp": [0.0, 0.0, 0.0]}))
        ser.write(pack_dict({"cart_kd": [4.0, 4.0, 4.0]}))
        ser.write(pack_dict({"activations": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}))
        # # ser.write(pack_dict({"activations": [0,0,0,0,0,0,0,0,0,0,0,0]}))
        # ser.write(
        #     pack_dict({"pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})
        # )
        ser.write(
            pack_dict({"cart_pos": [0.1, -0.07, 0.0, 0.1, 0.07, 0.0, -0.1, -0.07, 0.0, -0.1, 0.07, 0.0]})
        )
        # IN AMPS! # Rough ratio is 10A = 2.5Nm -> 0.25Nm/A.  It thinks this is N, going to Nm. So
        # If I want weight/4 = 4.4N per leg, then command 17.6 ?????? Seems like a lot
        #  
        # ser.write(pack_dict({"ff_force": [0.0, 0.0, -17.6, 0.0, 0.0, -17.6, 0.0, 0.0, -17.6, 0.0, 0.0, -17.6]}))

        ser.flush()
        while True:
            print(ser.readline().decode(), end="")
    except:
        print("Breaking loop.")
        ser.write(pack_dict({"activations": [0,0,0,0,0,0,0,0,0,0,0,0]}))
        ser.write(pack_dict({"idle":True}))
