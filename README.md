这是基于ego_planner实现的自动遍历探索，通过在rviz界面通过鼠标画出需要探索的区域，无人机自动实现这一区域的遍历探索。  
1. 按照下面步骤启动仿真：  
roslaunch mav_simulator tmux_gazebo_sim.launch  
roslaunch mav_simulator tmux_spawn.launch  
roslaunch mav_simulator tmux_control.launch   
roslaunch explore_scanner explore_scan.launch   
2. 首先在rviz菜单栏点击Polygon Draw Tool，然后使用鼠标左键在rviz交互区域点击需要探索的区域，使用鼠标右键确认完成，此时会将最后点击的位置与最初点击的位置连接，形成封闭的探索区域，并且自动启动基于ego_planner_swarm的遍历探索.
