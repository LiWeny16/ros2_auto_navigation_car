std_msgs/Header header
uint32 width                    # 地图宽度
uint32 height                   # 地图高度
float64 resolution              # 每个格子的实际大小（米/格）
geometry_msgs/Pose origin       # 地图原点在世界坐标系中的位置
int8[] data                     # 地图数据，-1表示未知，0表示空闲，100表示占用