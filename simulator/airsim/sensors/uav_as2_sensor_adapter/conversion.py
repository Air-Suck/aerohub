"""Coordinate and payload conversions independent of ROS."""
import math
import struct


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def extrinsic(config, optical=False):
    # AirSim body FRD -> ROS body FLU. Camera optical: right/down/forward.
    r, p, y = (math.radians(float(config.get(k, 0))) / 2
               for k in ('Roll', 'Pitch', 'Yaw'))
    q = multiply(multiply((0, 0, math.sin(y), math.cos(y)),
                          (0, math.sin(p), 0, math.cos(p))),
                 (math.sin(r), 0, 0, math.cos(r)))
    q = (q[0], -q[1], -q[2], q[3])
    if optical:
        q = multiply(q, (-0.5, 0.5, -0.5, 0.5))
    return (float(config.get('X', 0)), -float(config.get('Y', 0)),
            -float(config.get('Z', 0))), q


def imu_payload(response):
    """Convert AirSim FRD IMU samples to ROS FLU coordinates."""
    orientation = response['orientation']
    angular = response['angular_velocity']
    acceleration = response['linear_acceleration']
    values = [
        orientation[key] for key in ('x_val', 'y_val', 'z_val', 'w_val')
    ] + [angular[key] for key in ('x_val', 'y_val', 'z_val')] + [
        acceleration[key] for key in ('x_val', 'y_val', 'z_val')
    ]
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('non-finite IMU sample')
    quaternion = (float(values[0]), -float(values[1]), -float(values[2]),
                  float(values[3]))
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 0.0:
        raise ValueError('invalid IMU orientation')
    quaternion = tuple(value / norm for value in quaternion)
    angular_flu = (float(values[4]), -float(values[5]), -float(values[6]))
    acceleration_flu = (float(values[7]), -float(values[8]), -float(values[9]))
    return quaternion, angular_flu, acceleration_flu


def image_payload(response, image_type, color_order='rgb'):
    width, height = int(response['width']), int(response['height'])
    if width <= 0 or height <= 0:
        raise ValueError('empty image: ' + str(response.get('message', '')))
    if image_type in (1, 2):
        values = response['image_data_float']
        if len(values) != width * height:
            raise ValueError('invalid depth length')
        return '32FC1', width * 4, struct.pack('<%sf' % len(values), *values)
    data = bytes(response['image_data_uint8'])
    if len(data) != width * height * 3:
        raise ValueError('expected uncompressed three-channel RGB')
    if color_order == 'bgr':
        normalized = bytearray(data)
        normalized[0::3], normalized[2::3] = data[2::3], data[0::3]
        data = bytes(normalized)
    elif color_order != 'rgb':
        raise ValueError('color_order must be rgb or bgr')
    return 'rgb8', width * 3, data


def lidar_payload(response, data_frame):
    points = response['point_cloud']
    if len(points) % 3:
        raise ValueError('invalid XYZ point array')
    pose = response['pose']
    p, q = pose['position'], pose['orientation']
    origin = [p[k] for k in ('x_val', 'y_val', 'z_val')]
    rotation = tuple(q[k] for k in ('x_val', 'y_val', 'z_val', 'w_val'))
    inverse = (-rotation[0], -rotation[1], -rotation[2], rotation[3])
    output = bytearray()
    for i in range(0, len(points), 3):
        x, y, z = points[i:i+3]
        if data_frame == 'VehicleInertialFrame':
            v = (x-origin[0], y-origin[1], z-origin[2], 0)
            x, y, z, _ = multiply(multiply(inverse, v), rotation)
        elif data_frame != 'SensorLocalFrame':
            raise ValueError('unsupported LiDAR DataFrame')
        output.extend(struct.pack('<fff', x, -y, -z))
    return bytes(output)
