#!/usr/bin/env python3
import rclpy
import matplotlib.pyplot as plt

from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState


buffered_joint_positions: dict[str, list[float]] = dict()
buffered_time_stamps = []


#########################################################################
#
#   This node is supposed to run in a separate terminal after
#   launching the `inspect_ur20.launch.py` file.
#
#   Initially, the node will remain idle, waiting for a messaage
#   to be published in `/animate_cmd`. Then, it will start to record 
#   the incoming messages published at `/joint_states`.
#
#   By throwing a Keyboard Interrupt (Ctrl+C) in this node's shell, 
#   the main program will display a 2D visual plot of the each joint's 
#   position w.r.t time while the robot arm was animating.
#
#########################################################################
class JointStatePlotter(Node):

    def __init__(self):
        super().__init__('joint_configuration_plotter')
        
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_listener_callback,
            10
        )
        self.animate_command_subscription = self.create_subscription(
            Empty,
            'animate_cmd',
            self.animate_cmd_listener_callback,
            10
        )
        # prevent unused variable warning
        self.joint_state_subscription
        self.animate_command_subscription

        self.start_time = 0

        self.initialised = False

        self.heard_animate_cmd = False


    def animate_cmd_listener_callback(self, msg: Empty):
        msg
        # Received a message on `/animate_cmd`. The robot will start moving
        self.heard_animate_cmd = True
    

    def joint_state_listener_callback(self, msg: JointState):
        global buffered_joint_positions
        global buffered_time_stamps

        if not self.heard_animate_cmd:
            # The robot has not started yet. No reason to record its joints' positions.
            return

        if not self.initialised:
            self.start_time, _ = self.get_clock().now().seconds_nanoseconds()
            self.initialised = True

        self.get_logger().info(f"I heard:\n>{msg.name}\n{msg.position}")

        if (len(buffered_joint_positions) > 1000):
            self.get_logger().warn(
                "Joint state positions buffer reached max capacity. Further data will not be recorded"
            )
            return
        
        for name in msg.name:
            # Initialize every joint's measurement sequence.
            if name not in buffered_joint_positions.keys():
                buffered_joint_positions.update({name : []})

        for name, position in zip(msg.name, msg.position, strict=True):
            # Add the new measurements for every joint (vertical axis).
            buffered_joint_positions[name].append(position)

        # Add the new time stampt (horizontal axis).
        buffered_time_stamps.append(msg.header.stamp.sec - self.start_time)


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_subscriber = JointStatePlotter()
        rclpy.spin(minimal_subscriber)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\njoint_configuration_plotter finished successfully")

    for joint_name in buffered_joint_positions:
        # Plot each line
        plt.plot(buffered_time_stamps, buffered_joint_positions[joint_name], label=f'{joint_name}')

    # Adding labels and title
    plt.xlabel('time')
    plt.ylabel('positions')
    plt.title('UR20 Joint Positions Plot')
    plt.legend()

    # Display the plot
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()