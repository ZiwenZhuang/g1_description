import rclpy
from rclpy.node import Node
from unitree_hg.msg import (
    LowState,
    # MotorState,
    IMUState,
    LowCmd,
    # MotorCmd,
)

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

class RobotCfgs:
    class G1_29Dof_TorsoBase:
        NUM_DOF = 29
        NUM_ACTIONS = 29
        joint_names = [  # NOTE: order matters. This list is the order in real robot.
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ]
        joint_signs = [1] * 29  # All joints are in the same direction as the real robot.
        joint_signs[12] = -1; joint_signs[13] = -1; joint_signs[14] = -1  # waist joints are reversed
        """ please check this value from
            https://support.unitree.com/home/zh/G1_developer/basic_services_interface
            https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description
        """


class UnitreeROS2hgJointState(Node):
    def __init__(self):
        super().__init__('unitree_ros2_joint_state')
        
        self.joint_state = JointState()
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.robot_class = RobotCfgs.G1_29Dof_TorsoBase # TODO: make this configurable from ros params

        self.low_state_subscriber = self.create_subscription(
            LowState,
            '/low_state',
            self.low_state_callback,
            10,
        )
        self.imu_state_subscriber = self.create_subscription(
            IMUState,
            '/secandary_imu',
            self.imu_state_callback,
            10,
        )

        # publish fake states if the parameter is set
        self.declare_parameter('publish_fake_states', False)
        self.publish_fake_states = self.get_parameter('publish_fake_states').get_parameter_value().bool_value
        if self.publish_fake_states:
            self.get_logger().info("Publishing fake states", once=True)
            self.fake_joint_state_publisher = self.create_publisher(
                LowState,
                '/low_state',
                10,
            )
            self.fake_imu_state_publisher = self.create_publisher(
                IMUState,
                '/secandary_imu',
                10,
            )
            self.create_timer(0.002, self.publish_fake_states_callback) # 500Hz

    def low_state_callback(self, msg: LowState):
        self.get_logger().info("low_state revieved", once=True)
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.header.frame_id = "torso_link"
        self.joint_state.name = []
        self.joint_state.position = []
        self.joint_state.velocity = []
        self.joint_state.effort = []
        for i, joint_name in enumerate(self.robot_class.joint_names):
            self.joint_state.name.append(joint_name)
            self.joint_state.position.append(msg.motor_state[i].q * self.robot_class.joint_signs[i])
            self.joint_state.velocity.append(msg.motor_state[i].dq * self.robot_class.joint_signs[i])
            self.joint_state.effort.append(msg.motor_state[i].tau_est * self.robot_class.joint_signs[i])
        self.joint_state_publisher.publish(self.joint_state)

    def imu_state_callback(self, msg: IMUState):
        self.get_logger().info("imu_state revieved", once=True)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "world"
        tf_msg.child_frame_id = "torso_link"
        tf_msg.transform.translation.x = 0.
        tf_msg.transform.translation.y = 0.
        tf_msg.transform.translation.z = 0.
        tf_msg.transform.rotation.x = msg.quaternion[1].item()
        tf_msg.transform.rotation.y = msg.quaternion[2].item()
        tf_msg.transform.rotation.z = msg.quaternion[3].item()
        tf_msg.transform.rotation.w = msg.quaternion[0].item()
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_fake_states_callback(self):
        fake_low_state = LowState()
        fake_imu_state = IMUState()
        # Fill fake_low_state and fake_imu_state with fake data
        fake_imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.fake_joint_state_publisher.publish(fake_low_state)
        self.fake_imu_state_publisher.publish(fake_imu_state)

def main():
    rclpy.init()
    joint_state_node = UnitreeROS2hgJointState()
    rclpy.spin(joint_state_node)
    joint_state_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
