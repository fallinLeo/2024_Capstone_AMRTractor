# hybrid_astar  
Hybrid-A\*를 사용한 global planner   
## 실행방법  
### 패키지 Dependencies  
```
sudo apt install libompl-dev  
```
### Hybrid-A\* 단독실행(in image)  
* 시뮬레이션 없이 path planning 노드와 rviz 실행:  
```
roslaunch hybrid_astar manual.launch  
```
rviz에서 2D Pose Estimate를 설정하여 출발지점설정, 2D Nav Goal을 설정하여 도착지점 설정  
__2D Nav Goal 토픽명 : /move_base_simple/goal__  
#### map 변경 방법  
maps/map.yaml에서 map의 이미지 파일을 바꿀 수 있다.  
단 constants.h의 cellSize와 map.yaml의 resolution을 항상 맞춰야한다.  

### Hybrid-A\* 노드실행(in simulation)  
__2D Nav Goal 토픽명 : /move_base_simple/goal__  
* 시뮬레이션에서 path planning노드와 rivz 실행:  
```
roslaunch sp_gazebo r_001_docking_perception.gazebo.launch  
roslaunch hybrid_astar simul.launch  
```
* 시뮬레이션에서 path planning노드, rivz, path tracker노드 실행:  
```
roslaunch sp_gazebo r_001_docking_perception.gazebo.launch  
roslaunch hybrid_astar simul_move.launch  
```

### Hybrid-A\* 플러그인실행(in simulation)  
__2D Nav Goal 토픽명 : /R_001/move_base_simple/goal__  
* move base 런치파일 수정  
sp_navigation/launch/rid_navi.simul.launch  
```
    <!-- <param name="base_global_planner" value="global_planner/GlobalPlanner" /> -->
    <param name="base_global_planner" value="HybridAStar/GlobalHAPlanner" />
    <param name="planner_frequency" value="0.3" />
    <param name="planner_patience" value="30" />
```
* 시뮬레이션 실행  
```
roslaunch sp_gazebo r_001_docking_perception.gazebo.launch  
```

### rqt dynamic configuration   
rqt dynamic configuration에서 대부분의 설정을 수정할 수 있다.  
다음 planning부터 바뀐 설정이 적용된다.  
<img src = "./img/rqt.jpg" width="50%" height="100%">   

