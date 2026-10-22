ip设置：
本地ip：192.168.1.102
机械臂ip：192.168.1.202
雷达ip：192.168.1.200
路由器ip：192.168.1.1
路由器管理员密码和账号：[REDACTED: obtain from site configuration]
wifi密码：[REDACTED: obtain from site configuration]

（出厂已设置完成）

升降柱：
运行后会自动下降初始化（注意周围设备缠绕）
roslaunch lifting_ctrl start_motor.launch
绝对位置向上升100mm
rosservice call /lifter_1/LiftingMotorService "val: 100
mode: 0"
量程（0-800mm）

1、建图：
使能地盘
rosrun ranger_bringup setup_can2usb.bash (端口can0)
打开雷达
roslaunch agilexpro open_lidar.launch
启动建图
roslaunch agilexpro gmapping.launch
保存地图
roscd agilexpro/maps
rosrun map_server map_saver -f map

2、机械臂部分：
启动机械臂
roslaunch agx_xarm_bringup setup_arm.launch
（设置initial状态）
启动相机
roslaunch agx_xarm_bringup open_camera.launch
(观察识别码距离0.9m)
启动夹爪
roslaunch agx_xarm_bringup open_gripper.launch
如果夹爪关闭状态，执行下面
（ rosrun agx_xarm_pick control_gripper  输入1,回车，q退出）
启动控制夹爪任务节点
roslaunch agx_xarm_bringup agx_xarm_bring.launch

单独测试（不运行roslaunch agx_xarm_bringup agx_xarm_bring.launch）
(抓：rosrun agx_xarm_pick demo_pick
放：rosrun agx_xarm_pick demo_place)

3、导航部分：
打开雷达
roslaunch agilexpro open_lidar.launch
启动导航
roslaunch agilexpro navigation_4wd.launch

4、录制A（抓），B（放）点导航点：
（使用技巧：2D Nav Goal标点先导航到目标位置，再录制点位）
roscd agx_xarm_pick/config
rosrun agx_xarm_pick record
根据提示，按下回车记录两个点的位置。

5、demo测试：
运行状态机
cd ~/agilex_ws/src/core/agx_xarm_smach/script
python3 smach_demo.py
运行测试节点
roslaunch agx_xarm_pick test.launch


录屏软件：
sudo apt install simplescreenrecorder
播放软件：
sudo apt install mpv

播放操作视频使用mpv软件（出厂已安装）
