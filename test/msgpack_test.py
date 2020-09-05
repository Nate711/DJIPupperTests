import msgpack
import serial
import numpy as np


def pack_serialized(bytes_array, start=0x00):
    return bytes([start, len(bytes_array)]) + bytes_array


def pack_dict(dict, start=0x00):
    raw = msgpack.packb(dict, use_single_float=True)#use_bin_type=True)
    # print(raw)
    full_array = pack_serialized(raw, start)
    print(full_array)
    return full_array

with serial.Serial("/dev/tty.usbmodem78075901", timeout=0.2) as ser:
    try:
        ser.write(pack_dict({"zero": True}))
        # ser.write(pack_dict({"kp": 12.0}))
        # ser.write(pack_dict({"kd": 0.005}))
        # ser.write(pack_dict({"max_current": 0.5}))
        ser.write(pack_dict({"cart_kp": [1.0, 2.0, 3.0]}))
        ser.write(pack_dict({"cart_kd": [0.1, 0.2, 0.3]}))
        ser.write(pack_dict({"activations": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}))
        # # ser.write(pack_dict({"activations": [0,0,0,0,0,0,0,0,0,0,0,0]}))
        # ser.write(
        #     pack_dict({"pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})
        # )
        ser.write(
            pack_dict({"cart_pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})
        )

        ser.flush()
        while True:
            print(ser.readline().decode(), end="")
    except:
        print("Breaking loop.")
        ser.write(pack_dict({"activations": [0,0,0,0,0,0,0,0,0,0,0,0]}))
        ser.write(pack_dict({"idle":True}))
