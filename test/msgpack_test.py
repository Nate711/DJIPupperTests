import msgpack
import serial
import numpy as np


def pack_serialized(bytes_array, start=b"<", stop=b">"):
    return start + bytes_array + stop


def pack_dict(dict, start=b"<", stop=b">"):
    raw = msgpack.packb(dict, use_bin_type=True)
    return pack_serialized(raw, start, stop)

with serial.Serial("/dev/tty.usbmodem71393001", timeout=0.2) as ser:
    try:
        ser.write(pack_dict({"zero": True}))
        ser.write(pack_dict({"kp": 12.0}))
        ser.write(pack_dict({"kd": 0.005}))
        ser.write(pack_dict({"max_current": 0.5}))
        # ser.write(pack_dict({"activations": [0,0,0,0,0,0,1,1,1,1,1,1]}))
        ser.write(pack_dict({"activations": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}))
        # ser.write(pack_dict({"activations": [0,0,0,0,0,0,0,0,0,0,0,0]}))
        ser.write(
            pack_dict({"pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]})
        )

        ser.flush()
        count = 0
        while True:
            print(ser.readline().decode(), end="")
    except:
        print("Breaking loop.")
        ser.write(pack_dict({"activations": [0,0,0,0,0,0,0,0,0,0,0,0]}))
        ser.write(pack_dict({"idle":True}))
