
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult, PublisherSubscriber, StartNode
import rclpy
from rclpy.duration import Duration
import time 
import threading
from std_msgs.msg import Bool,Int32

# 쓰레기통 앞 좌표
front_trash = [17.644,0.22942]
clubroom_front = [15.928,0.032135] #real club front
clubroom_front2 = [27.882,0.0371] #test start point
credit_point = [28.1152,6.35] #7.2155 엘베봤을때z = 0.0148, w = 0.9998
#강의실방향 봤을때 z = -0.7296 w = 0.684



trailer_goal_state = [11.831,-0.14858]
# z :1, w = 0.0

charge_pose = [4.98,0.2783]
# z: 0.6858 , w : 0.73

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
    talk_with_arduino = PublisherSubscriber()
    docking_pub = Bool()
    docking_pub.data = False
    charging_pub = Bool()
    charging_pub.data = False

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = clubroom_front2[0]
    initial_pose.pose.position.y = clubroom_front2[1]
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0
    initial_pose.pose.orientation.z = 0.7013  #test orientation 
    initial_pose.pose.orientation.w = 0.7127
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    ###adding ui
    # start_navigation = StartNode()

    # while rclpy.ok():
    #     rclpy.spin_once(start_navigation)
    #     start_call = start_navigation.start_received 
    #     logger.info(f'received :{start_call.data}')

    #     if start_call.data == True: break
    #     else:
    #         print("Navigation don't received start call")      
    ##aadding 


    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = credit_point[0]
    goal_pose.pose.position.y = credit_point[1]
    goal_pose.pose.orientation.z = -0.6958
    goal_pose.pose.orientation.w = 0.7198

    #recovery node adding##################
    result = None
    path1 = navigator.getPath(initial_pose, goal_pose, planner_id="GridBased")

    max_retries = 3  # 최대 재시도 횟수
    retries = 0

    while retries < max_retries:
        # path1 = navigator.getPath(initial_pose, goal_pose, planner_id="GridBased")

        
        if path1 is not None:
            logger.info("Path planning succeeded.")
                # 첫 번째 시도에서는 followPath 사용
            if retries == 0: 
                navigator.followPath(path1)
                # navigator.followWaypoints(goal_poses)
            else:
                # 재시도 시 goToPose 사용
                navigator.goToPose(goal_pose)

            #navigation complete and waiting
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                # if feedback:
                #     print(f"Remaining distance: {feedback}")

            result = navigator.getResult()
            

            if result == TaskResult.SUCCEEDED:
                logger.info(f'result : {result}')
                # print("goal_pose succeeded")
                time.sleep(2)
                break  # 내비게이션 성공
            else:
                logger.warning("Navigation failed. Retrying...")
                navigator.backup(backup_dist=0.5, backup_speed=0.15)
                retries += 1
                while not navigator.isTaskComplete():
                    feedback=navigator.getFeedback()
                    logger.error("Backup failed. Aborting navigation.")
                time.sleep(2)  # 재시도 간 대기 시간
                # navigator.lifecycleShutdown()
                # exit(0)

                

        else:
            logger.warning(f"Path planning failed, attempt {retries + 1}. Retrying...")
            retries += 1
            time.sleep(1)  # 재시도 간 대기 시간 
        
    if retries >= max_retries:
        logger.error("Maximum retries reached. Navigation failed.")
        #recovery node done#####################


    start_time = time.time()

    while rclpy.ok():
        rclpy.spin_once(talk_with_arduino)
        time.sleep(0.1)

        if result == TaskResult.SUCCEEDED:
            docking_pub.data = True  # 주행 성공 시 True 값 퍼블리시
            charging_pub.data = False
            talk_with_arduino.publish_docking.publish(docking_pub)
            talk_with_arduino.publish_charging.publish(charging_pub)
            
            print("Navigation succeeded, docking started")
        else:
            print("Navigation failed or canceled")

        task = talk_with_arduino.received
        # task.data = talk_with_arduino.listener_callback()
        logger.info(f'received :{task.data}')

        elapsed_time = time.time() - start_time

        if task.data or elapsed_time >25:
            talk_with_arduino.get_logger().info('Received True, breaking loop.')
            break #trailer_start topic 받아서 트렉-트레일러 주행시작
    
    #카메라 끄기 위해
    docking_pub.data = False
    talk_with_arduino.publish_docking.publish(docking_pub)
    



    # talk_with_arduino.destroy_node()
    logger.info('Gripper success')
    print('Waiting for 5 seconds before moving to the next shelf location...')
    logger.info('Waiting for 3 seconds')
    time.sleep(3)
    

#############위까지 3차발표였음 ###

    trailer_goal_station = PoseStamped()
    trailer_goal_station.header.frame_id = 'map'
    trailer_goal_station.header.stamp = navigator.get_clock().now().to_msg()
    # trailer_goal_station.pose.position.x = trailer_goal_state[0] #real
    # trailer_goal_station.pose.position.y = trailer_goal_state[1]
    # trailer_goal_station.pose.orientation.z = 1.0
    # trailer_goal_station.pose.orientation.w = 0.0

    trailer_goal_station.pose.position.x = clubroom_front[0] #test
    trailer_goal_station.pose.position.y = clubroom_front[1]
    trailer_goal_station.pose.orientation.z = 1.0
    trailer_goal_station.pose.orientation.w = 0.0

    # navigator.goToPose(shipping_destination)
    path2 = navigator.getPath(goal_pose, trailer_goal_station, planner_id="SmacPlanner")
    navigator.followPath(path2)

    result2 = None
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        result2 = navigator.getResult()
        if feedback:
            remaining_distance = feedback.distance_to_goal
            print(f"Remaining distance: {remaining_distance} meters")
            if remaining_distance > 0.1:  # 목표까지의 거리가 0.1m 이상이면 계속 대기
                continue
            
    time.sleep(2)
    
    start_time = time.time()

    while rclpy.ok():
        rclpy.spin_once(talk_with_arduino)
        time.sleep(0.1)

        if result2 == TaskResult.SUCCEEDED:
            grip_connect = Int32() #그리퍼 여는곳
            grip_connect.data = 3
            talk_with_arduino.trailer_pub.publish(grip_connect)
            print('Route complete! Restarting...')
            time.sleep(3)
        elif result2 == TaskResult.CANCELED:
            print('Security route was canceled, exiting.')
            time.sleep(3)
        elif result2 == TaskResult.FAILED:
            print('Security route failed! Restarting from other side...')
            time.sleep(3)
        
        elapsed_time = time.time() - start_time
        
        task = talk_with_arduino.received
        logger.info(f'docking_state : {docking_pub}, received :{task.data}, result : {result}')

        if task.data==False or elapsed_time > 15: break #trailer_start topic 받아서 트렉-트레일러 주행시작

    
    
    
    #차징스테이션 도착 pose 찾기
    charge_station = PoseStamped()
    charge_station.header.frame_id = 'map'
    charge_station.header.stamp = navigator.get_clock().now().to_msg()
    charge_station.pose.position.x = charge_pose[0]
    charge_station.pose.position.y = charge_pose[1]
    charge_station.pose.orientation.z = 0.668 #방향 어떻게 도착할지
    charge_station.pose.orientation.w = 0.73

    # navigator.goToPose(shipping_destination)
    path3 = navigator.getPath(trailer_goal_station,charge_station, planner_id="GridBased")
    navigator.followPath(path3)

    result = None
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # if feedback:
        #     print(f"Remaining distance: {feedback}")
    
    result3 = navigator.getResult()
    #카메라 키기 위해
    docking_pub.data = True
    talk_with_arduino.publish_docking.publish(docking_pub)
    charging_pub.data = True  # 주행 성공 시 True 값 퍼블리시
    talk_with_arduino.publish_charging.publish(charging_pub)
    time.sleep(2)
    
    while rclpy.ok():
        rclpy.spin_once(talk_with_arduino)
        if result3 == TaskResult.SUCCEEDED:
            charging_pub.data = True  # 주행 성공 시 True 값 퍼블리시
            talk_with_arduino.publish_charging.publish(charging_pub)
            print("Navigation succeeded, charging started")
            break
        else:
            print("Navigation failed or canceled")
            break




    exit(0)

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()

