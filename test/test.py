import serial
import struct
import sys

with serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=0.1) as ser:
    try:
        count = 0
        while True:
            data = ser.read(1)
            if len(data) == 0:
                continue
            if data[0] == 0x55:
                data = ser.read(1)
                if len(data) == 0:
                    continue
                if data[0] == 0xaa:
                    data = ser.read(36)
                    if len(data) == 0:
                        continue
                    lng = len(data)
                    if lng == 36:
                        count += 1
                        if count == 250:
                            [
                                acc_x, acc_y, acc_z,
                                gyro_p, gyro_q, gyro_r,
                                mag_x, mag_y, mag_z,
                            ] = struct.unpack(">fffffffff", data)
                            print(
                                # "\n"
                                f"acc: ({acc_x:.4f}, {acc_y:.4f}, {acc_z:.4f}), "
                                f"gyro: ({gyro_p:.4f}, {gyro_q:.4f}, {gyro_r:.4f}), "
                                f"mag: ({mag_x:.4f}, {mag_y:.4f}, {mag_z:.4f})"
                            )
                            # sys.stdout.flush()
                            count = 0
                        # else:
                        #     print('+', end='')
                        #     sys.stdout.flush()
                else:
                    continue
            else:
                continue
    except KeyboardInterrupt:
        pass
