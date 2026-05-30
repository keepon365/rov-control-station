import struct

def pack_downlink_command(y_force=0.0, x_force=0.0, z_force=0.0, yaw_torque=0.0, arm_angle=0):

    data = struct.pack('<ffffB', y_force, x_force, z_force, yaw_torque, arm_angle)
    frame_type = 0x49
    checksum = frame_type
    for b in data:
        checksum ^= b
    packet = b'\xFA\xAF' + bytes([frame_type]) + data + bytes([checksum]) + b'\xFB\xBF'
    return packet

def parse_uplink_frame(data):

    frame_type = data[2]
    if frame_type != 0x53:
        return None
    (temp,) = struct.unpack('<f', data[3:7])
    water_leak = data[7]
    (current,) = struct.unpack('<f', data[8:12])
    checksum = frame_type
    for b in data[3:12]:
        checksum ^= b
    if checksum != data[12]:
        return None
    return temp, water_leak, current