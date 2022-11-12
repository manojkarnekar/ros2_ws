from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration



goal_poses = []


def nav2_pose(t_x, t_y, o_z, o_w):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = t_x
    goal_pose.pose.position.y = t_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = o_z
    goal_pose.pose.orientation.w = o_w
    goal_poses.append(goal_pose)


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    nav_start = navigator.get_clock().now()
    # nav2_pose(1.0, 0.0, 0.0, 1.0)
    # navigator.goToPose(goal_poses)
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
            
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()
    

    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    
    navigator.lifecycleShutdown()
    exit(0)


if __name__ == '__main__':
    main()