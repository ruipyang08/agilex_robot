我们需要解决的问题总结如下：

1. 仓库命名不统一并且发布之后依然在变化
2. 机器人协议支持混乱，客户很难分清楚应该用哪个版本的SDK和ROS包
3. 机器人仿真的支持不完善且缺乏更新和维护
4. 机器人对串口的支持不完整
5. 代码仓库管理和维护的分工不明确，WestonRobot和AgileX两边存在代码冗余、更新不同步的问题

针对以上问题，提出以下解决办法：

1. 统一代码仓库命名，并尽量减少仓库的数量（问题1）
   * a. ugv_sdk: C++ interface for all robots, async_port将合并进入ugv_sdk以方便用户下载编译
   * b. <robot-name>_ros: ROS driver package (xxx_base、xxx_bringup 、xxx_description、xxx_msgs)
   * c. <robot-name>_nav: ROS navigation support package
   * d. 每个ROS仓库按照支持的ROS版本提供分支, 如melodic, foxy
2. 所有代码仓库默认支持V2.0协议，V1.0协议的支持通过改变配置文件而不是切换分支来实现（问题2）
   * a. 松灵提供表格列出哪些车型支持V1.0协议，哪些支持V2.0协议（例如协议版本与车子编号的对应关系） <--- AgileX 
   * b. 所有协议相关的实现细节应该封装在ugv_sdk, ugv_sdk应同时支持V1.0协议和V2.0协议 <--- 需要WestonRobot进一步评估如何实现
   * c. ROS包通过更改launch file参数来切换不同的协议版本
3. 导航ROS包默认支持Webots，松灵根据客户需求增加Gazebo的支持（问题3）
4. 松灵提供表格列出对哪些车型同时支持CAN和串口，哪些车型仅支持CAN（问题4） <--- AgileX 
5. 代码仓库管理
   * a. WestonRobot的所有代码托管在：https://github.com/westonrobot
   * b. AgileX的所有代码托管在：https://github.com/agilexrobotics
   * c. WestonRobot主导ugv_sdk的更新和维护，并提供ROS包的样板工程（如scout_ros)，AgileX参照样板工程主导ROS包的更新和维护
   * d. "主导"的含义：代码的更改发生在主导方维护的repo里，例如WestonRobot主导ugv_sdk，AgileX依然可以贡献代码，AgileX新的功能代码应在对应分支里实现（如feature-ranger_support)，然后提交到westonrobot/ugv_sdk的master分支，代码merge之后，AgileX再把agilexrobotics/ugv_sdk的master分支与westonrobot/ugv_sdk的master分支同步。类似的，除了ROS样板工程，WestonRobot应把功能更改提交merge request到AgileX主导的代码仓库里，并在merge之后完成同步。