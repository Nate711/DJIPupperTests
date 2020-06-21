import msgpack
import serial
import numpy as np

def pack_serialized(bytes_array, start=b'<', stop = b'>'):
    return start + bytes_array + stop

def pack_dict(dict, start=b'<', stop=b'>'):
    raw = msgpack.packb(dict, use_bin_type=True)
    return pack_serialized(raw, start, stop)

pos_command = pack_dict({"pos":[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]})
print(pos_command)

with serial.Serial("/dev/tty.usbmodem71393001", timeout=0.2) as ser:
    ser.write(pos_command)
    ser.write(pack_dict({"kp":2.0}))
    ser.write(pack_dict({"kd":0.001}))
    ser.flush()

    while True:
        print(ser.readline().decode(),end="")