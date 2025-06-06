from yasmin import StateMachine
from final_pkg.states.patrol import Patrol
from final_pkg.states.rotate_and_detect import RotateDetect
from final_pkg.states.approach_target import ApproachTarget
from final_pkg.states.report_target import ReportTarget
import rclpy
from rclpy.node import Node
from yasmin_viewer import YasminViewerPub


class MainNode(Node):
    def __init__(self):
        super().__init__('sm_main')

        sm = StateMachine(outcomes=['EXIT'])

        sm.add_state('PATROL', Patrol(self), transitions={
            'reached': 'ROTATE_DETECT',
            'finished': 'EXIT'
        })

        sm.add_state('ROTATE_DETECT', RotateDetect(self), transitions={
            'found': 'APPROACH_TARGET',
            'next': 'PATROL'
        })

        sm.add_state('APPROACH_TARGET', ApproachTarget(self), transitions={
            'reached': 'REPORT_TARGET'
        })

        sm.add_state('REPORT_TARGET', ReportTarget(self), transitions={
            'done': 'ROTATE_DETECT'
        })

        YasminViewerPub('sm_main', sm)
        sm()

def shutdown(node: Node):
    """Shutdown function
    Stop TurtleBot3 when terminating

    Args:
        node (Node): Node object
    """
    node.get_logger().info("Follow State Cleanup!!")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    pub.publish(Twist())
    node.get_clock().sleep_for(Duration(nanoseconds=100))
    node.destroy_publisher(pub)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = MainNode()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
