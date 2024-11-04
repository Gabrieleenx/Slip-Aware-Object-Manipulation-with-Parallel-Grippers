import numpy as np
import rospy
import tf


class FramePose(object):
    """Class for describing a frame or transformation"""
    def __init__(self, position=np.zeros(3), quaternion=np.zeros(4)):
        """:param position: np.array([x,y,z]) position [m]
        :param quaternion: np.array([x,y,z,w]) orientation [unit quaternion]"""
        self.position = np.copy(position)
        self.quaternion = np.copy(quaternion)
            
        self.tempPosition = np.zeros(3)
        self.tempQuaternion = np.zeros(4)

    def getQuaternion(self):
        """returns the quaternion"""
        return np.copy(self.quaternion)

    def getPosition(self):
        """returns the position"""
        return np.copy(self.position[0:3])

    def setPosition(self, position):
        """updates the position
        :param position: np.array([x,y,z]) [m]
        """
        self.position = np.copy(position) 

    def setQuaternion(self, quaternion):
        """updates the orientation
       :param quaternion: np.array([x,y,z,w]) orientation [unit quaternion]"""
        self.quaternion = np.copy(quaternion)


class TfBroadcastFrames(object):
    """class for adding new frames to the tf tree and used internally for control"""
    def __init__(self, ft_sensor_1, vel_sensor_1, ft_sensor_2, vel_sensor_2, middle_frame):
        """:param gripperRight: class instance of FramePose describing the local transformation from yumi_link_7_r
        to yumi_grip_r
         :param gripperLeft: class instance of FramePose describing the local transformation from yumi_link_7_l
        to yumi_grip_l
        :param yumiToWorld: class instance of FramePose describing the local transformation from yumi_base_link
        to world"""
        self.tfbrodcaster = tf.TransformBroadcaster()
        self.postion = 0.0
        
        self.ft_sensor_1 = ft_sensor_1
        self.vel_sensor_1 = vel_sensor_1

        self.ft_sensor_2 = ft_sensor_2
        self.vel_sensor_2 =  vel_sensor_2

        self.middle_frame = middle_frame
        
    def updateGripper(self, finger_position):
        self.position = finger_position/1000 # mm to m
        vel_sensor_offset = abs(self.vel_sensor_1.getPosition()[2])
        new_z = self.position/2 + vel_sensor_offset
        
        pos_2 = self.ft_sensor_2.getPosition()
        pos_2[2] = new_z
        self.ft_sensor_2.setPosition(pos_2)

        pos_1 = self.ft_sensor_1.getPosition()
        pos_1[2] = -new_z
        self.ft_sensor_1.setPosition(pos_1)
        
        self.tfBroadcast()

    def tfBroadcast(self):
        """Sends out to the tf tree"""
        self.tfbrodcaster.sendTransform(tuple([0.0,0.0,0.0]),
                                        tuple([0.0,0.0,0.0,1.0]),
                                        rospy.Time.now(), "gipper_base_link", "tool0")
        
        self.tfbrodcaster.sendTransform(tuple(self.middle_frame.getPosition()),
                                        tuple(self.middle_frame.getQuaternion()),
                                        rospy.Time.now(), "gipper_middle_frame", "gipper_base_link")

        self.tfbrodcaster.sendTransform(tuple(self.ft_sensor_1.getPosition()),
                                        tuple(self.ft_sensor_1.getQuaternion()),
                                        rospy.Time.now(), "ft_sensor_1", "gipper_middle_frame")
        
        self.tfbrodcaster.sendTransform(tuple(self.ft_sensor_2.getPosition()),
                                        tuple(self.ft_sensor_2.getQuaternion()),
                                        rospy.Time.now(), "ft_sensor_2", "gipper_middle_frame")
        
        self.tfbrodcaster.sendTransform(tuple(self.vel_sensor_1.getPosition()),
                                        tuple(self.vel_sensor_1.getQuaternion()),
                                        rospy.Time.now(), "vel_sensor_1", "ft_sensor_1")
        
        self.tfbrodcaster.sendTransform(tuple(self.vel_sensor_2.getPosition()),
                                        tuple(self.vel_sensor_2.getQuaternion()),
                                        rospy.Time.now(), "vel_sensor_2", "ft_sensor_2")
        

