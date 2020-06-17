import msgpack
import serial

# o = msgpack.packb({}, use_bin_type=True)
# o = msgpack.packb({"g":[1.1,2.1]}, use_bin_type=True)
o = msgpack.packb({"p":[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]}, use_bin_type=True)
print(o)

with serial.Serial("/dev/tty.usbmodem71393001", timeout=0.2) as ser:
    ser.write(b'<')
    ser.write(o)
    ser.write(b'\n')
    ser.flush()

    while True:
        print(ser.readline())