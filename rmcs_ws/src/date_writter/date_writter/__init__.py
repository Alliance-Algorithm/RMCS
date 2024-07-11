import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

class Subscriber(Node):
    armor_angle = 0
    pnp_position = 0
    pnp_rotatiin = 0
    
    def __init__(self,name):
        super().__init__(name)
        self.create_subscription(Float64,"/armor/angle",self.ArmorCallback,20)
        self.create_subscription(Pose,"/armor_pose",self.PnpCallbaclk,20)
        self.get_logger().info("data_writter subscriber init")
    
    def ArmorCallback(self,msg=Float64):
        self.armor_angle = msg.data

    def PnpCallbaclk(self, msg =Pose):
        self.pnp_position = msg.position
        self.pnp_rotatiin = msg.orientation
        print(msg.position)


def main(args=None):
    rclpy.init(args = args)
    node = Subscriber("writter")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()