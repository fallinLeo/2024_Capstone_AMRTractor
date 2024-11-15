
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult, PublisherSubscriber
import rclpy
from rclpy.duration import Duration
import time 
import threading
from std_msgs.msg import Bool,Int32

# 쓰레기통 앞 좌표
front_trash = [17.644,0.22942]
clubroom_front = [15.928,0.032135]
credit_point = [28.539,6.30] #7.2155 엘베봤을때z = 0.0148, w = 0.9998
#강의실방향 봤을때 z = -0.7296 w = 0.684



trailer_goal_state = [11.831,-0.14858]
# z :1, w = 0.0

charge_pose = [4.45531,-0.3803]
# z: -0.668 , w : 0.743

# Shelf positions for picking / pickingup file
shelf_positions = {
    'shelf_A': [-3.829, -7.604],
    'shelf_B': [-3.791, -3.287],
    'shelf_C': [-3.791, 1.254],
    'shelf_D': [-3.24, 5.861]}



# Shipping destination for picked products
# shipping_destinations = {
#     'recycling': [-0.205, 7.403],
#     'pallet_jack7': [28.073, 0.8],
#     'conveyer_432': [6.217, 2.153],
#     'frieght_bay_3': [-6.349, 9.147]}


def main():

    rclpy.init()
    logger = rclpy.logging.get_logger('my_logger')

    navigator = BasicNavigator()

    navigator.waitUntilNav2Active()


    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = clubroom_front[0]
    initial_pose.pose.position.y = clubroom_front[1]
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)


    talk_with_arduino = PublisherSubscriber()
    docking_pub = Bool()
    docking_pub.data = False

    #목적지 설정 (PoseStamped로 전달)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = credit_point[0]
    goal_pose.pose.position.y = credit_point[1]
    goal_pose.pose.orientation.z = -0.6958
    goal_pose.pose.orientation.w = 0.7198

    path1 = navigator.getPath(initial_pose, goal_pose, planner_id="GridBased")

    # navigator.goToPose(goal_pose)
    navigator.followPath(path1)
    result = None
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Remaining distance: {feedback}")
    
    result = navigator.getResult()
    time.sleep(2)    
    

    while rclpy.ok():
        rclpy.spin_once(talk_with_arduino)

        if result == TaskResult.SUCCEEDED:
            docking_pub.data = True  # 주행 성공 시 True 값 퍼블리시
            talk_with_arduino.publish_docking.publish(docking_pub)
            print("Navigation succeeded, docking started")
            break
        else:
            print("Navigation failed or canceled")

        task = talk_with_arduino.received
        logger.info(f'docking_state : {docking_pub}, received :{task.data}, result : {result}')

        if task.data==2 : break #trailer_start topic 받아서 트렉-트레일러 주행시작


    # talk_with_arduino.destroy_node()
    logger.info('Gripper success')
    print('Waiting for 5 seconds before moving to the next shelf location...')
    logger.info('Waiting for 3 seconds')
    time.sleep(3)

#############위까지 3차발표였음 ###

    trailer_goal_station = PoseStamped()
    trailer_goal_station.header.frame_id = 'map'
    trailer_goal_station.header.stamp = navigator.get_clock().now().to_msg()
    trailer_goal_station.pose.position.x = trailer_goal_state[0]
    trailer_goal_station.pose.position.y = trailer_goal_state[1]
    trailer_goal_station.pose.orientation.z = 1.0
    trailer_goal_station.pose.orientation.w = 0.0

    # navigator.goToPose(shipping_destination)
    path2 = navigator.getPath(goal_pose, trailer_goal_station, planner_id="SmacPlanner")
    navigator.followPath(path2)

    result = None
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Remaining distance: {feedback}")
    
    result2 = navigator.getResult()
    time.sleep(2)    
    

    while rclpy.ok():
        rclpy.spin_once(talk_with_arduino)

        if result2 == TaskResult.SUCCEEDED:
            grip_connect = Int32() #그리퍼 여는곳
            grip_connect.data = 3
            talk_with_arduino.trailer_pub.publish(grip_connect)
            print('Route complete! Restarting...')
            pass
        elif result2 == TaskResult.CANCELED:
            print('Security route was canceled, exiting.')
            time.sleep(3)
        elif result2 == TaskResult.FAILED:
            print('Security route failed! Restarting from other side...')
            time.sleep(3)


    
    
    
    #차징스테이션 도착 pose 찾기
    charge_station = PoseStamped()
    charge_station.header.frame_id = 'map'
    charge_station.header.stamp = navigator.get_clock().now().to_msg()
    charge_station.pose.position.x = charge_state[0]
    charge_station.pose.position.y = charge_state[1]
    charge_station.pose.orientation.z = 0.0 #방향 어떻게 도착할지
    charge_station.pose.orientation.w = 1.0

    # navigator.goToPose(shipping_destination)
    path3 = navigator.getPath(trailer_goal_station,charge_station, planner_id="GridBased")
    navigator.followPath(path3)

    exit(0)



    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()

