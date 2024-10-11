import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Pose2D
from tf_transformations import euler_from_quaternion

class OdomToPose(Node):
    def __init__(self):
        super().__init__('init_odom_node')
        self.declare_parameter('input_topic', 'gz_pose')
        self.declare_parameter('output_topic', 'rosbot_pose')
        self.input_topic_ = self.get_parameter('input_topic').value
        self.output_topic_ = self.get_parameter('output_topic').value
        self.get_logger().info('Subscribing to %s pose topic'%self.input_topic_)
        
        self.get_logger().info('Publishing to %s pose topic'%self.output_topic_)
        self.pose_pub_ = self.create_publisher(Pose2D, self.output_topic_, 10)
        self.pose_sub_ = self.create_subscription(Pose,
                                                  self.input_topic_,
                                                  self.processPose, 10)
        

  
        self.get_logger().info('Subscribing to %s odom topic'%self.input_topic_)
        
        self.get_logger().info('Publishing to %s odometry topic'%self.output_topic_)
    
     
        
    def processPose(self, msg):
        pose2d = Pose2D()
        pose2d.x = msg.position.x
        pose2d.y = msg.position.y

        quat = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        pose2d.theta = yaw

        self.pose_pub_.publish(pose2d);

def main(args=None):
    rclpy.init(args=args)
    pose_node = OdomToPose()
    rclpy.spin(pose_node)
    rclpy.shutdown()
    
if __name__=="__main__":
    main()
    
