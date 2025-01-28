# pcd_rotator
This package is used to rotate point cloud data.
## build
```bash
colcon build --symlink-install
```
## run
```bash
ros2 launch pcd_rotator pcd_rotator.launch.py
```
## parameter

* config_file_path: config/params.yaml

```yaml
pcd_rotator:
  ros__parameters:
    # 输入PCD文件的路径
    input_path: "/home/hy/wust_nav/src/point_lio/PCD/ul25.pcd"
    # 旋转处理后输出PCD文件的路径
    output_path: "output.pcd"
    # 滚转轴旋转角度（弧度），此处设置为π，实现180度旋转
    roll: 3.1415926 
    # 俯仰轴旋转角度（弧度），此处设置为0，无俯仰旋转
    pitch: 0.0 
    # 偏航轴旋转角度（弧度），此处设置为0，无偏航旋转
    yaw: 0.0   
    # 调试模式开关，false表示关闭调试模式
    debug: false
```
