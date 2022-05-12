#!/usr/bin/env python3
import math
import time
from ahrs.filters import Madgwick
import numpy as np

import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from imu_driver import mpu9250
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

# TODO: calibration and loading custom config

G_mps2 = 9.80665  # 1 G in m/s^2
DEG2RAD = math.pi / 180
NUM_MESURMENTS = 1000

class IMUNotFound(Exception):
    pass


class IMUNode(DTROS):
    def __init__(self):
        super(IMUNode, self).__init__(
            node_name="imu_node",
            node_type=NodeType.DRIVER,
        )
        # get parameters
        self._veh = rospy.get_param('~veh')
        i2c_bus = rospy.get_param("~i2c_bus")
        i2c_address = rospy.get_param("~i2c_address")
        #i2c_address = 0x68
        polling_hz = rospy.get_param("~polling_hz")

        self._ang_vel_offset = DTParam(
            "~ang_vel_offset",
            param_type=ParamType.LIST,
            help="Angular velocity offsets",  # TODO
        )
        self._accel_offset = DTParam(
            "~accel_offset",
            param_type=ParamType.LIST,
            help="Acceleration offset",
        )

        trial_msg = lambda msg: "At I2C <bus{}, addr{}>: {}".format(
            i2c_bus, i2c_address, msg)
        try:
            self._imu = mpu9250(i2c_bus, i2c_address)
            _ = self._imu.accel
            _ = self._imu.gyro
            self.loginfo(trial_msg("Found IMU"))
            self.Q = np.array([1.0, 0.0, 0.0, 0.0])
            self.madgwick = Madgwick()
        except IOError:
            self.logerr(trial_msg("IMU sensor not correctly detected"))
            raise IMUNotFound()

        self.offset = self.calibrate()
        #self._ang_vel_offset.value = self.offset['ang_vel_offset']
        #self._accel_offset.value = self.offset['accel_offset']

        self.pub = rospy.Publisher('~imu_data', Imu, queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / polling_hz),
            self.publish_data,
        )

    def calibrate(self):
        """
        Calibrates the imu sensor data.
        """
        gyro_totals=[0, 0, 0]
        acc_totals=[0, 0, 0]

        #input('Place the Duckiebot on a level surface, and leave it perfecly still. Press enter when ready. ')
        print("Calibatring")
        for n in range(0, NUM_MESURMENTS):
            a=self._imu.accel
            g=self._imu.gyro

            for i in range(0, 3):
                gyro_totals[i] += g[i] * DEG2RAD
                acc_totals[i] += a[i] * G_mps2

            if n % 10 == 0:
                self.logdebug('{:4}/{:4}'.format(n + 1, NUM_MESURMENTS))
            rospy.sleep(0.01)

        gyro_avg = list(map(lambda x: x / NUM_MESURMENTS, gyro_totals))
        acc_avg = list(map(lambda x: x / NUM_MESURMENTS, acc_totals))
        acc_avg[2] -= G_mps2

        self.logdebug('Calibration done')

        return {
            'ang_vel_offset': gyro_avg,
            'accel_offset': acc_avg
        }

    def calc3(self, const, data_zip_offset):
        """
        The size of data_zip_offset should be 3
        Calculate: data[i] * const - offset[i]
        Returns a Vector3() storing in (x, y, z) the (0, 1, 2)-th calculated num
        """
        values = [data * const - offset for data, offset in data_zip_offset]
        ret = Vector3()
        ret.x, ret.y, ret.z = values
        return ret

    def calc_angular_velocity(self, gyro_data):
        #return self.calc3(DEG2RAD, zip(gyro_data, self._ang_vel_offset.value))
        return self.calc3(DEG2RAD, zip(gyro_data, self.offset['ang_vel_offset']))

    def calc_linear_acceleration(self, acc_data):
        #return self.calc3(G_mps2, zip(acc_data, self._accel_offset.value))
        return self.calc3(G_mps2, zip(acc_data, self.offset['accel_offset']))

    def calc_orientation(self, gyro_data, acc_data):
        #mag = tuple(ti/1000000 for ti in mag_data)
        acc_data_ar = np.array([acc_data.x, acc_data.y, acc_data.z])
        gyro_data_ar = np.array([gyro_data.x, gyro_data.y, gyro_data.z])

        #t = self.madgwick.updateMARG(self.Q, acc=acc_data_ar, gyr=gyro_data_ar, mag=np.asarray(mag))
        t = self.madgwick.updateIMU(self.Q, acc=acc_data_ar, gyr=gyro_data_ar)
        self.Q = t

        ret = Quaternion()
        ret.x, ret.y, ret.z, ret.w = t
        return ret

    def publish_data(self, event):
        try:
            acc_data = self._imu.accel
            self.logdebug('Accel: {:.3f} {:.3f} {:.3f} mg'.format(*acc_data))
            gyro_data = self._imu.gyro
            self.logdebug('Gyro: {:.3f} {:.3f} {:.3f} dps'.format(*gyro_data))

            try:
                mag = self._imu.mag
                self.logdebug('Magnet: {:.3f} {:.3f} {:.3f} mT'.format(*mag))
                temp = self._imu.temp
                self.logdebug('Temperature: {:.3f} C'.format(temp))
            except:
                self.logdebug("Didn't manage to read magnet/temp")

            msg = Imu()
            msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = "/odom"  # TODO
            msg.angular_velocity = self.calc_angular_velocity(gyro_data)
            msg.linear_acceleration = self.calc_linear_acceleration(acc_data)

            #msg.orientation = self.calc_orientation(gyro_data, acc_data, mag)
            msg.orientation = self.calc_orientation(msg.angular_velocity, msg.linear_acceleration)

            # TODO
            for i in range(0, 9):
                msg.angular_velocity_covariance[i] = 0
                msg.linear_acceleration_covariance[i] = 0
                msg.orientation_covariance[i] = -1

            self.pub.publish(msg)

        except IOError as e:
            self.logwarn(f"I/O error: {e}")


if __name__ == '__main__':
    try:
        node = IMUNode()
        rospy.spin()
    except IMUNotFound as e:
        # Err already logged in node
        # Don't kill the entire interface container
        pass
