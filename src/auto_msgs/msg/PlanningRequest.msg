std_msgs/Header header
geometry_msgs/PoseStamped start       # 起点位置
geometry_msgs/PoseStamped goal        # 目标位置
string planner_type                   # 规划器类型 "astar" 或 "hybrid_astar"
bool consider_kinematic               # 是否考虑运动学约束