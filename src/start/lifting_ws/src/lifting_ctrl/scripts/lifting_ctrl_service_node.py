#!/usr/bin/env python3
# 升降柱控制服务端节点
# 同时将接收到的串口数据，publish出去
# 开了一个线程用来pub数据
import rospy
import _thread
import time
import json
import os

from lifting_motor_ctrl import C_LiftingMotorCtrl

from bt_task_msgs.srv  import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse 

from bt_task_msgs.msg import LiftMotorMsg

class C_ROS_Server:
    def __init__(self,) -> None:
        '''
        初始化节点
        '''
        rospy.init_node('lifting_ctrl_server_node', anonymous=True)
        # 外部参数,判断控制哪一个电机编号
        self.motor_id = 1
        if rospy.has_param('~motor_id'):
            self.motor_id = rospy.get_param("~motor_id")
            rospy.loginfo("%s is %d", rospy.resolve_name('~motor_id'), self.motor_id)
        else: rospy.loginfo("未找到") 
        # 电机控制类实例化 
        self.input_callback_time = time.time()
        self.ctrl = C_LiftingMotorCtrl(self.motor_id)
        self.ctrl.MotorConfigInit()
        lift_port = self.ctrl.GetLiftPortName()
        self.add_to_param_list('/liftPort',lift_port)
        # 设置阻塞式回调函数超时时间，默认为300s，即5分钟
        if('responseTimeout' in self.ctrl.GetMotorJsonConfig().keys()):
            self.response_timeout = self.ctrl.GetMotorJsonConfig()["responseTimeout"]
        else: self.response_timeout = 60
        # 设置回调函数模式，如果是1则是立刻返回，0为阻塞式回调
        if('callBackMode' in self.ctrl.GetMotorJsonConfig().keys()):
            self.callBackMode = self.ctrl.GetMotorJsonConfig()["callBackMode"]
        else: self.callBackMode = 1
        if('topicHz' in self.ctrl.GetMotorJsonConfig().keys()):
            self.topicHz = self.ctrl.GetMotorJsonConfig()["topicHz"]
        else: self.topicHz = -1
        # 设定执行器末端速度，单位mm/s
        self.liftTargetSpd = abs((self.ctrl.GetMotorMaxSpd() * 10000) / (self.ctrl.reductionRatio * 60))
        if('initSpd' in self.ctrl.GetMotorJsonConfig().keys()):
            self.initSpd = self.ctrl.GetMotorJsonConfig()["initSpd"]
        else: self.initSpd = -100
        # 初始化位置
        if('initPos' in self.ctrl.GetMotorJsonConfig().keys()):
            self.__initPos = (self.ctrl.GetMotorJsonConfig()["initPos"])
        else: self.__initPos = 0
        # 读取电机数据
        self.motor_states = self.ctrl.ReadMotorData()
        self.target_speed = 0
        # 初始阶段为初始化模式
        if('initMode' in self.ctrl.GetMotorJsonConfig().keys()):
            self.mode = (self.ctrl.GetMotorJsonConfig()["initMode"])
        else: self.mode = 1
        self.fps_error=0
        self.back_height = 0
        if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
            self.back_height = self.motor_states.back_lift_height
        # init_state 保持上游语义 = "限位初始化流程已跑完"（上游在 mode != 1 时置 True），
        # 服务端各 mode 分支的 `and self.init_state` 判据依赖它，**不能**拿它兼做坐标系列定，
        # 否则坐标系一不可信，急停(-2)/下行速度(-4) 也会被一起拒掉（实测确认）。
        # 坐标系列定是独立的 self.frame_valid，只在发布 initState 时合成给下游看。
        self.init_state:bool = False
        if(self.mode != 1):
            self.init_state = True
        self.motor_msgs = LiftMotorMsg()
        self.motor_pub = rospy.Publisher('LiftMotorStatePub', LiftMotorMsg, queue_size=1)
        if(self.callBackMode == 0):
            self.motor_srv = rospy.Service('LiftingMotorService', LiftMotorSrv, self.SververCallbackBlock)
        elif(self.callBackMode == 1):
            self.motor_srv = rospy.Service('LiftingMotorService', LiftMotorSrv, self.SververCallbackImmediately)
        self.first_up_flag = False
        self.first_down_flag = False
        self.previous_dir = 0
        self.command_send = 0
        self.offset = 0
        self.overload_flag:bool = False
        self.callLock = self.last_callLock = True
        self.timeoutFlag:bool = False
        self.print_flag_init = True
        # [agx-fix 2026-09-29] 升降坐标系标定门禁，详见 CheckFrameAtStartup 的注释
        self.frame_check = bool(rospy.get_param("~frame_check", True))
        self.frame_tolerance = int(rospy.get_param("~frame_tolerance", 20))
        self.frame_file = rospy.get_param("~frame_file", self.DefaultFrameFile())
        self.frame_valid = False
        self.last_saved_height = None
        self.last_save_time = 0.0
        self.CheckFrameAtStartup()
        # [agx-fix 2026-09-29] 原代码这里取的是 self.ctrl.GetTargetHeight()，
        # 那是 C_LiftingMotorCtrl.__init__ 里"配置初始化之前"那次读的值，读失败时会
        # 退化成 0，再被 LiftMovePos 下发成一条"去 0 位"的指令。改为用配置初始化之后
        # 这次读到的真实高度，并且只在坐标系可信时才下发。
        if(self.frame_valid):
            self.target_height = self.back_height
            self.ctrl.LiftMovePos(self.target_height)
        else:
            self.target_height = self.back_height

    def add_to_param_list(self, param_name, new_element):
        # 检查参数是否存在
        if not rospy.has_param(param_name):
            # 如果参数不存在，则创建一个新的空列表参数
            rospy.set_param(param_name, [])

        # 获取参数值
        param_list = rospy.get_param(param_name)

        # 检查新元素是否已存在于列表中
        if new_element not in param_list:
            # 如果新元素不存在于列表中，则添加到列表中
            param_list.append(new_element)

            # 更新参数值
            rospy.set_param(param_name, param_list)
            rospy.loginfo("Added {} to param list {}".format(new_element, param_name))
        else:
            rospy.loginfo("{} already exists in param list {}".format(new_element, param_name))

    # ------------------------------------------------------------------
    # [agx-fix 2026-09-29] 升降坐标系标定门禁
    #
    # 背景（2026-09-29 在本机实测确认）：
    #   830ABS 驱动器的位置基准是"下降压下限位 + 清零"建立的，整机断电重启后这个
    #   基准会丢失，驱动器位置寄存器回到 ≈0（实测 backPos=-206），但柱子物理上并不
    #   在最低点（下限位开关 downLimit=False 可佐证）。原配置 initMode=0 关掉了开机
    #   自动初始化，于是节点拿着一份失真的原点开机，后果有三：
    #     1. lift_joint_bridge 把假 0 喂给 MoveIt 的 lift_joint，TF/碰撞检查全错；
    #     2. 控制台显示"已到达 0mm"，给人假安全感；
    #     3. "升到 600mm" 会让物理柱子从当前位置再上 600mm，撞机械硬限位。
    #
    # 做法：把"坐标系是否可信"显式建模成 init_state。
    #   - 可信：行为与原来完全一致；
    #   - 不可信：拒绝位置/上行指令，50Hz 定位回路不下发位移，initState=False 让
    #     下游（lift_joint_bridge、控制台）自动降级；初始化(1)/急停(-2)/清故障(-3)/
    #     位置清零(-6)/下行速度 仍然放行，下行由下限位开关兜底。
    #   - 初始化跑完 = 坐标系重建，重新开放并落盘。
    # 关闭方式（不需要重编译）：~frame_check:=false（不推荐，只是绕过检查）
    # ------------------------------------------------------------------
    def DefaultFrameFile(self) -> str:
        ros_home = os.environ.get("ROS_HOME", os.path.expanduser("~/.ros"))
        return os.path.join(ros_home, "lift_frame_m{}.json".format(self.motor_id))

    def LoadFrameFile(self):
        '''
        读取上次记录的停机高度，返回 int；没有记录或记录损坏返回 None
        '''
        try:
            with open(self.frame_file, "r", encoding="utf-8") as f:
                return int(json.load(f)["height_mm"])
        except Exception:
            return None

    def SaveFrameFile(self, height, force=False) -> None:
        '''
        记录当前可信高度，供下次上电比对。默认 2 秒节流 + 只在变化时写。
        '''
        if(not force and not self.frame_valid):
            return
        now = time.time()
        if(not force):
            if((now - self.last_save_time) < 2.0):
                return
            if(self.last_saved_height == int(height)):
                self.last_save_time = now
                return
        try:
            os.makedirs(os.path.dirname(self.frame_file), exist_ok=True)
            tmp = self.frame_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"height_mm": int(height),
                           "motor_id": self.motor_id,
                           "saved_at": int(now)}, f)
            os.replace(tmp, self.frame_file)
            self.last_saved_height = int(height)
            self.last_save_time = now
        except Exception as e:
            rospy.logwarn("升降柱 %d 高度写入 %s 失败: %s", self.motor_id, self.frame_file, e)

    def CheckFrameAtStartup(self) -> None:
        '''
        上电时判断升降坐标系是否仍然可信，结果只写 self.frame_valid。
        注意不要碰 self.init_state：那是上游的「限位初始化已跑完」语义，
        服务端各 mode 分支的 `and self.init_state` 判据依赖它。
        '''
        if(self.mode == 1):
            # 配置成开机自动初始化，等初始化跑完再建零
            rospy.loginfo("升降柱 %d: initMode=1，开机会自动下降找下限位重建坐标系", self.motor_id)
            self.frame_valid = False
            return
        if(not isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg)):
            self.frame_valid = False
            rospy.logerr("升降柱 %d: 上电首次读取电机数据失败，坐标系未知，拒绝一切位置指令", self.motor_id)
            return
        if(not self.frame_check):
            self.frame_valid = True
            rospy.logwarn("升降柱 %d: frame_check=false，已跳过坐标系一致性检查（不推荐）", self.motor_id)
            return
        saved = self.LoadFrameFile()
        if(saved is None):
            # 没有历史记录可比对，只能先相信驱动器；本次运行结束落盘后，下次就能比对了
            self.frame_valid = True
            rospy.logwarn("升降柱 %d: 找不到 %s，无法校验坐标系，暂按驱动器读数 %.0fmm 信任。"
                          "若曾在非零高度断电，请执行 'lift init' 重新建立坐标系",
                          self.motor_id, self.frame_file, self.back_height)
            return
        diff = abs(self.back_height - saved)
        if(diff <= self.frame_tolerance):
            self.frame_valid = True
            rospy.loginfo("升降柱 %d: 坐标系一致性通过（记录 %.0fmm / 实读 %.0fmm，差 %.0fmm）",
                          self.motor_id, saved, self.back_height, diff)
        else:
            self.frame_valid = False
            rospy.logerr("================================================================")
            rospy.logerr("升降柱 %d 坐标系未标定，已拒绝位置指令！", self.motor_id)
            rospy.logerr("  上次记录高度 : %.0f mm", saved)
            rospy.logerr("  本次驱动读数 : %.0f mm  (差 %.0f mm > 容差 %d mm)",
                         self.back_height, diff, self.frame_tolerance)
            rospy.logerr("  830ABS 的位置基准在上电后已丢失，此读数不可信。")
            rospy.logerr("  恢复办法: 执行 'lift init'，下降到下限位重新建立坐标系（真实运动）。")
            rospy.logerr("  在此之前 MoveIt 使用上次已知高度，控制台升降柱一行不可拖动。")
            rospy.logerr("================================================================")

    def FrameGateAllows(self, req) -> bool:
        '''
        坐标系未标定时的指令门禁。
        放行：初始化(1)、急停(-2)、清故障(-3)、位置清零(-6)。
        拒绝：位置模式(0) 和**所有**速度模式(-4/-7，含下行)。
        [agx-fix 2026-09-29] 原本下行是放行的，想法是"下限位开关能兜底"，实测证伪：
        ModeCtrl 的 -4 分支每拍无条件重发 LiftMoveSpd，**没有到位/超时退出条件**，
        一条 `mode=-4 val=-20` 会让柱子一直下行到撞机械限位（实测降到 -141mm 才被
        急停掐住，高度读数为负 = 软件行程下限也没拦住）。速度模式和位置模式一样是
        "会一直跑到撞限位"的指令，坐标系不可信时一律拒绝。
        '''
        if(self.frame_valid or self.mode == 1):
            return True
        if(req.mode == 0 or req.mode in (-4, -7)):
            rospy.logerr("升降柱 %d 坐标系未标定，已拒绝 mode=%d val=%d。"
                         "请先执行 'lift init' 下降到下限位重新建立坐标系。",
                         self.motor_id, req.mode, req.val)
            return False
        return True
    
    def JudgeFirstUpOrDown(self):
        '''
        判断电机是否切换状态，比如上一次是上升，当前要发送下降，那么判断当前是上升后的第一次下降，反之亦然
        '''
        dir = self.JudgeMotorDirection()
        if(self.command_send == 1):
            self.command_send = 0
            if(dir == 1 and self.previous_dir == -1):
                self.first_up_flag = True
                self.first_down_flag = False
            elif(dir == -1 and self.previous_dir == 1): 
                self.first_up_flag = False
                self.first_down_flag = True
            elif(dir == 1 and self.previous_dir == 1):
                self.first_up_flag = False
            elif(dir == -1 and self.previous_dir == -1):
                self.first_down_flag = False
        if(dir != 0):
            self.previous_dir = dir
        if(self.first_up_flag): 
            self.offset = 3
        if(self.first_down_flag):
            self.offset = -3
        
    def PublishMotorMsgs(self) -> None:
        '''
        将要pub出去的数据赋值
        '''
        if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
            self.motor_msgs.header.stamp = rospy.Time.now()
            # 当前话题电机id
            self.motor_msgs.motorId = self.motor_id
            # 初始化状态
            # [agx-fix 2026-09-29] 对下游合成「坐标系是否可信」：
            # initState=False 时 lift_joint_bridge 停发/回放、控制台锁滑条。
            # 上游原本只发 init_state（限位初始化已跑完），断电后它恒为 True，
            # 于是下游无法察觉位置基准已丢失。
            self.motor_msgs.initState = self.init_state and self.frame_valid
            # print("---------------")
            # 电机控制模式
            self.motor_msgs.ctrlMode = self.motor_states.ctrl_mode
            # 电机反馈电压
            self.motor_msgs.backVol = self.motor_states.back_vol
            # 电机反馈电流
            self.motor_msgs.backCurrent = self.motor_states.back_current
            # 电机状态位
            self.motor_msgs.stateBit = self.motor_states.state_bit
                # 电机是否运行，0-停机，1-启动
            self.motor_msgs.runState = (self.motor_msgs.stateBit&0x01)
                # 过流
            self.motor_msgs.overflowI = (bool)(self.motor_msgs.stateBit&0x02)
                # 过压
            self.motor_msgs.overflowV = (bool)(self.motor_msgs.stateBit&0x04)
                # 编码器故障
            self.motor_msgs.encodeErr = (bool)(self.motor_msgs.stateBit&0x8)
                # 位置偏差过大
            self.motor_msgs.posBiasOver = (bool)(self.motor_msgs.stateBit&0x10)
                # 欠压
            self.motor_msgs.underVol = (bool)(self.motor_msgs.stateBit&0x20)
                # 过载标志
            self.motor_msgs.overLoad = (bool)(self.motor_msgs.stateBit&0x40)
            # 外部控制标志 = 0-pc ； 1-外部 PLC
            self.motor_msgs.externalCtrlMode = (bool)(self.motor_msgs.stateBit&0x80)
            # 电机转速
            self.motor_msgs.motorSpeed = self.motor_states.back_speed
            # 升降柱转速abs(round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60))
            self.motor_msgs.backSpeed = round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60)) # mm/s
            # 电机定位完成
            self.motor_msgs.locationComplete = self.motor_states.location_complete
            # 电机反馈位置
            self.motor_msgs.backPos = self.motor_states.back_pos
            # 升降柱当前高度
            self.motor_msgs.backHeight = self.motor_states.back_lift_height
            # 升降柱目标高度
            self.motor_msgs.targetHeight = self.ctrl.GetTargetHeight()
            # 电机目标位置
            self.motor_msgs.targetPos = self.ctrl.GetTargetPos()
            # 电机上限位
            self.motor_msgs.upLimit = (self.motor_states.upLimit)
            # 电机下限位
            self.motor_msgs.downLimit = (self.motor_states.downLimit)
            self.motor_msgs.upLimitHeight = self.ctrl.GetUpLimitHeight()
            self.motor_msgs.downLimitHeight = self.ctrl.GetDownLimitHeight()
            # 电机是否到达了目标位置，误差小于1认为到达了
            if(abs(self.target_height - self.motor_states.back_lift_height)<=1): 
                self.motor_msgs.reachTargetPos = True
            else: self.motor_msgs.reachTargetPos = False
        self.motor_msgs.stopFlag = self.ctrl.GetStopFlag()
        self.motor_msgs.fpsError = self.fps_error
            # 发布
        try:
            self.motor_pub.publish(self.motor_msgs)
        except Exception as e:
            print("发布失败",e)
            pass
    
    def JudgeMotorDirection(self):
        '''
        判断电机方向,1上升，-1下降，0不动
        '''
        # 正向，向上升
        if(self.target_height-self.back_height > 0):
            return 1
        # 向下降
        elif(self.target_height-self.back_height < 0):
            return -1
        # 不动
        else:
            return 0 
    def JudgeMotorDirectionWithSpeed(self):
        '''
        判断电机方向，1正方向，-1负方向，0不动
        '''
        if(self.target_speed > 0):
            # print("电机正方向速度运动")
            return 1
        elif(self.target_speed < 0):
            # print("电机负方向速度运动")
            return -1
        # 不动
        elif(self.target_speed == 0):
            return 0 
        else: return 2
    # 回调函数
    def SververCallbackImmediately(self,req):
        '''
        830ABS 服务端回调函数,立刻反馈数值，不等待电机到达位置就反馈
        '''
        self.input_callback_time = time.time()
        self.command_send = 1
        
        self.mode = req.mode
        resp = -1
        self.timeoutFlag = False
        init_state_lock = True
        # [agx-fix 2026-09-29] 升降坐标系未标定门禁：回滚 self.mode，避免 ModeCtrl
        # 按被拒绝的指令去驱动电机；直接返回 -14（与"未初始化"共用同一个码）。
        if(not self.FrameGateAllows(req)):
            self.mode = 0
            self.last_callLock = self.callLock
            rospy.logerr("服务端的应答为-14（升降坐标系未标定）")
            return LiftMotorSrvResponse(-14)
        if(self.callLock == self.last_callLock):
            self.callLock = not self.last_callLock
            # 初始化模式
            if(req.mode == 1):
                if(init_state_lock):
                    self.init_state = False
                    init_state_lock = False
                if(self.init_state): resp = 1
                else: resp = -1
            # 位置模式
            elif(req.mode == 0 and self.init_state):
                self.target_height = self.ctrl.SetTargetHeight(req.val)
                rospy.loginfo("target height: %d", self.target_height)
                # 电机是否到达了目标位置，误差小于1认为到达了，反馈1为到达，反馈-1为未到达
                if(abs(self.target_height - self.back_height) < 2): 
                    resp = 1
                else: resp = -1
            # 急停
            elif(req.mode == -2 and self.init_state):
                if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                    if(self.motor_states.back_speed == 0):
                        resp = 1
                else: resp = -1
            # -3 指令用来初始化电机，并清空错误帧率
            elif(req.mode == -3):
                resp = 1
                # else: resp = -1
            # 恒定速度
            elif(req.mode == -4 and self.init_state):
                # rospy.loginfo("the max speed is: ",self.liftTargetSpd)
                self.ctrl.SetTargetHeight(self.back_height)
                self.target_speed = req.val
                rospy.loginfo("----------操作电机 %d 目标速度为%d",self.motor_id, self.target_speed)
                if(self.target_speed > abs(self.liftTargetSpd)): self.target_speed=abs(self.liftTargetSpd)
                if(self.target_speed < -abs(self.liftTargetSpd)): self.target_speed=-abs(self.liftTargetSpd)
                rospy.loginfo("----------操作电机 %d 最终发送速度为%d",self.motor_id, self.target_speed)
                if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                    rospy.loginfo(self.motor_states.back_speed)
                    if(abs(round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60)) - self.target_speed)<20):
                        resp = 1
                else: resp = -1
            elif(req.mode == -5 and self.init_state):
                self.target_speed = req.val
                if(self.target_speed > self.liftTargetSpd): self.target_speed=self.liftTargetSpd
                if(self.target_speed < -self.liftTargetSpd): self.target_speed=-self.liftTargetSpd
                self.target_speed = round((self.target_speed * self.ctrl.reductionRatio * 60)/10000)
                self.ctrl.MotorSetMaxSpd(self.target_speed)
                resp = 1
            elif(req.mode == -6):
                if(self.target_speed ==0 ):
                    self.ctrl.MotorClearPosZero()
                else:
                    resp = -15
                if(abs(self.motor_states.back_pos) <= 1):
                    resp = 1
                else:
                    resp = -1
            # 恒定速度
            elif(req.mode == -7 and self.init_state):
                # rospy.loginfo("the max speed is: ",self.liftTargetSpd)
                self.ctrl.SetTargetHeight(self.back_height)
                self.target_speed = req.val
                max_spd = self.ctrl.GetMotorMaxSpd()
                rospy.loginfo("----------操作电机 %d 目标速度为%d",self.motor_id, self.target_speed)
                if(self.target_speed > max_spd): self.target_speed=max_spd
                if(self.target_speed < -max_spd): self.target_speed=-max_spd
                rospy.loginfo("----------操作电机 %d 最终发送速度为%d",self.motor_id, self.target_speed)
                if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                    rospy.loginfo(self.motor_states.back_speed)
                    if(abs(self.motor_states.back_speed - self.target_speed)<20):
                        resp = 1
                else: resp = -1
            else:
                resp = 0
                if(not self.init_state):
                    resp = -14
            
            if(resp == -1):
                if(self.init_state == True):
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        if(self.motor_states.upLimit):
                            if(self.mode == 0 and self.JudgeMotorDirection()==-1):
                                resp = -1
                            elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==-1):
                                resp = -1
                            else: resp = -2
                        elif(self.motor_states.downLimit):
                            if(self.mode == 0 and self.JudgeMotorDirection()==1):
                                resp = -1
                            elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==1):
                                resp = -1
                            else: resp = 2
                else: 
                    resp = -1
            # 如果错误帧率达到4以上
            if(self.fps_error > 4):
                resp = -9
            
            if(time.time()  - self.input_callback_time > self.response_timeout):
                self.mode == -2
                self.timeoutFlag = True 
                rospy.logerr("lifting_motor timeout!!!!")
                resp = -12
                if(not self.init_state):
                    resp = -13

        self.last_callLock = self.callLock
        rospy.loginfo("resquest的mode为%s", self.mode)
        rospy.loginfo("resquest的val为%d", req.val)
        rospy.loginfo("服务端的应答为%s", resp)
        return LiftMotorSrvResponse(resp)

    def SververCallbackBlock(self,req):
        '''
        服务端回调函数，阻塞式，等待电机到达目标位置才会反馈
        '''
        self.input_callback_time = time.time()
        self.command_send = 1
        
        self.mode = req.mode
        resp = -1
        self.timeoutFlag = False
        init_state_lock = True
        # [agx-fix 2026-09-29] 升降坐标系未标定门禁：回滚 self.mode，避免 ModeCtrl
        # 按被拒绝的指令去驱动电机；直接返回 -14（与"未初始化"共用同一个码）。
        if(not self.FrameGateAllows(req)):
            self.mode = 0
            self.last_callLock = self.callLock
            rospy.logerr("服务端的应答为-14（升降坐标系未标定）")
            return LiftMotorSrvResponse(-14)
        if(self.callLock == self.last_callLock):
            while resp == -1:
                self.callLock = not self.last_callLock
                # 初始化模式
                if(req.mode == 1):
                    if(init_state_lock):
                        self.init_state = False
                        init_state_lock = False
                    if(self.init_state): resp = 1
                    else: resp = -1
                # 位置模式
                elif(req.mode == 0 and self.init_state):
                    self.target_height = self.ctrl.SetTargetHeight(req.val)
                    rospy.loginfo("target height: %d", self.target_height)
                    # 电机是否到达了目标位置，误差小于1认为到达了，反馈1为到达，反馈-1为未到达
                    if(abs(self.target_height - self.back_height) < 2): 
                        resp = 1
                    else: resp = -1
                # 急停
                elif(req.mode == -2 and self.init_state):
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        if(self.motor_states.back_speed == 0):
                            resp = 1
                    else: resp = -1
                # -3 指令用来初始化电机，并清空错误帧率
                elif(req.mode == -3):
                    resp = 1
                    # else: resp = -1
                # 恒定速度
                elif(req.mode == -4 and self.init_state):
                    # rospy.loginfo("the max speed is: ",self.liftTargetSpd)
                    self.ctrl.SetTargetHeight(self.back_height)
                    self.target_speed = req.val
                    rospy.loginfo("----------操作电机 %d 目标速度为%d",self.motor_id, self.target_speed)
                    if(self.target_speed > abs(self.liftTargetSpd)): self.target_speed=abs(self.liftTargetSpd)
                    if(self.target_speed < -abs(self.liftTargetSpd)): self.target_speed=-abs(self.liftTargetSpd)
                    rospy.loginfo("----------操作电机 %d 最终发送速度为%d",self.motor_id, self.target_speed)
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        rospy.loginfo(self.motor_states.back_speed)
                        if(abs(round((self.motor_states.back_speed*10000) / (self.ctrl.reductionRatio * 60)) - self.target_speed)<5):
                            resp = 1
                    else: resp = -1
                elif(req.mode == -5 and self.init_state):
                    self.target_speed = req.val
                    if(self.target_speed > self.liftTargetSpd): self.target_speed=self.liftTargetSpd
                    if(self.target_speed < -self.liftTargetSpd): self.target_speed=-self.liftTargetSpd
                    self.target_speed = round((self.target_speed * self.ctrl.reductionRatio * 60)/10000)
                    self.ctrl.MotorSetMaxSpd(self.target_speed)
                    resp = 1
                elif(req.mode == -6):
                    if(self.target_speed ==0 ):
                        self.ctrl.MotorClearPosZero()
                    else:
                        resp = -15
                    if(abs(self.motor_states.back_pos) <= 1):
                        resp = 1
                    else:
                        resp = -1
                # 恒定速度
                elif(req.mode == -7 and self.init_state):
                    # rospy.loginfo("the max speed is: ",self.liftTargetSpd)
                    self.ctrl.SetTargetHeight(self.back_height)
                    self.target_speed = req.val
                    max_spd = self.ctrl.GetMotorMaxSpd()
                    rospy.loginfo("----------操作电机 %d 目标速度为%d",self.motor_id, self.target_speed)
                    if(self.target_speed > max_spd): self.target_speed=max_spd
                    if(self.target_speed < -max_spd): self.target_speed=-max_spd
                    rospy.loginfo("----------操作电机 %d 最终发送速度为%d",self.motor_id, self.target_speed)
                    if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                        rospy.loginfo(self.motor_states.back_speed)
                        if(abs(self.motor_states.back_speed - self.target_speed)<5):
                            resp = 1
                    else: resp = -1
                else:
                    resp = 0
                    if(not self.init_state):
                        resp = -14
                
                if(resp == -1):
                    if(self.init_state == True):
                        if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                            if(self.motor_states.upLimit):
                                if(self.mode == 0 and self.JudgeMotorDirection()==-1):
                                    resp = -1
                                elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==-1):
                                    resp = -1
                                else: resp = -2
                            elif(self.motor_states.downLimit):
                                if(self.mode == 0 and self.JudgeMotorDirection()==1):
                                    resp = -1
                                elif(self.mode == -4 and self.JudgeMotorDirectionWithSpeed()==1):
                                    resp = -1
                                else: resp = 2
                    else: 
                        resp = -1
                # 如果错误帧率达到4以上
                if(self.fps_error > 4):
                    resp = -9
                
                if(time.time()  - self.input_callback_time > self.response_timeout):
                    self.mode == -2
                    self.timeoutFlag = True 
                    rospy.logerr("lifting_motor timeout!!!!")
                    resp = -12
                    if(not self.init_state):
                        resp = -13
                    break

                time.sleep(0.5)
        self.last_callLock = self.callLock
        rospy.loginfo("resquest的mode为%s", self.mode)
        rospy.loginfo("resquest的val为%d", req.val)
        rospy.loginfo("服务端的应答为%s", resp)
        return LiftMotorSrvResponse(resp)
    
    def ShowLog(self):
        rospy.loginfo("--------------")
        rospy.loginfo("模式：%d", self.mode)
        rospy.loginfo("--------------")

    def ModeCtrl(self):
        '''
        电机模式控制
        '''
        # if(self.timeoutFlag):
        #     if(self.init_state == False):
        #         rospy.loginfo(self.motor_id," 号电机初始化超时,直接将当前位置作为初始化位置")
        #         self.init_state = True
        #         self.ctrl.LiftTimeoutInit(True, self.motor_states)
        #     self.mode == -2
        # 判断初始化标志位，进行电机初始化
        # if(not self.init_state): self.mode = 1
        # print("current_mode: ",self.mode)
        
        if(self.mode == 1 and self.fps_error <= 3):
            # print("初始化ing...速度为：",self.initSpd)
            self.ctrl.SetStopFlag(False)
            if(not self.overload_flag):
                if(self.print_flag_init):
                    rospy.loginfo("开始限位初始化...")
            self.init_state = self.ctrl.LiftLimitInit(self.initSpd, self.motor_states)
            # print(self.motor_msgs.overLoad, self.motor_msgs.backCurrent, self.init_state)
            if(self.init_state): 
                self.mode = 0xF1
                self.target_height = self.__initPos
                # [agx-fix 2026-09-29] 压到下限位并清零 = 升降坐标系重建成功，
                # 重新开放位置指令，并把新基准立刻落盘
                self.frame_valid = True
                rospy.loginfo("初始化完成,over（升降坐标系已重建，可以接受位置指令）")
                self.SaveFrameFile(self.back_height, force=True)
                time.sleep(1)
            else:
                if(self.print_flag_init):
                    rospy.logwarn("未完成初始化...")
                if(self.motor_msgs.overLoad or self.motor_msgs.backCurrent > self.ctrl.GetOverflowILimit()):
                    self.overload_flag = True
                    rospy.logwarn("检测到电机过载，可能是电机卡住或者下限位失效，在当前位置进行初始化")
                    self.init_state = True
                    self.ctrl.LiftTimeoutInit(True, self.motor_states)
                    self.mode = 0xF2
                if(self.timeoutFlag):
                    print(self.motor_id," 号电机初始化超时,直接将当前位置作为初始化位置")
                    # rclpy.node.get_logger().info(self.motor_id," 号电机初始化超时,直接将当前位置作为初始化位置")
                    self.init_state = True
                    self.ctrl.LiftTimeoutInit(True, self.motor_states)
                    self.mode = 0xF2
            self.print_flag_init = False
            # [agx-fix 2026-09-29] 超时/过载初始化 = 把"当前位置"当成 0 点。坐标系
            # 自洽所以放行，但 0~600mm 已经不再对应真实机械行程（说明下限位没触发，
            # 要么开关坏了要么下方有东西卡住），必须让人知道。
            if(self.mode == 0xF2):
                self.frame_valid = True
                rospy.logerr("=" * 62)
                rospy.logerr("升降柱 %d 是在当前位置软清零的（超时/过载初始化）！", self.motor_id)
                rospy.logerr("  当前 %.0fmm 被定义为 0 点，0~%dmm 不再对应真实机械行程。",
                             self.back_height, self.ctrl.GetUpLimitHeight())
                rospy.logerr("  机械上下限位开关仍然有效（已修复其保护逻辑），但数值仅供参考。")
                rospy.logerr("  请检查下限位开关与柱下方是否有障碍物。")
                rospy.logerr("=" * 62)
                    # print("--------",self.motor_msgs.overLoad, self.motor_msgs.backCurrent, self.init_state)
        if(self.mode == 0xF1):
            self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=False)
            # 运动电机
            self.ctrl.LiftMovePos(self.target_height)
            # self.mode = 0xF2
        if(self.mode == 0xF2):
            self.ctrl.LiftMoveSpd(0)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=False)
        if(self.mode == -2):# -2 指令用来紧急停止电机
            self.target_speed = 0
            self.ctrl.LiftMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=True)
        if(self.mode == -3):# -3 指令用来初始化电机，并清空错误帧率
            # self.ctrl.MotorInit(self.motor_id)
            self.ctrl.MotorClearBug()
            self.fps_error = 0
        if(self.mode == -4 and self.fps_error <= 1):# -4指令用来运动电机指定速度
            self.ctrl.SetTargetHeight(self.back_height)
            self.ctrl.LiftMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=False)
        if(self.mode == -5 and self.fps_error <= 1):# -4指令用来运动i电机指定速度
            self.ctrl.MotorSetMaxSpd(self.target_speed)
        if(self.mode == -6 and self.fps_error <= 1):# -4指令用来运动i电机指定速度
            self.ctrl.MotorClearPosZero()
        if(self.mode == -7 and self.fps_error <= 1):# -7指令用来运动i电机指定速度
            self.ctrl.SetTargetHeight(self.back_height)
            self.ctrl.MotorMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=False)
        if(self.mode == 0 and self.fps_error <= 1):
            # [agx-fix 2026-09-29] 坐标系未标定时，50Hz 定位回路绝不能下发位移：
            # 此时 self.target_height 是一个不可信的原点，一旦驱动器读数漂移就会
            # 悄悄把柱子往那个假位置拉。这里改成 force 急停，只保持不动。
            if(not self.frame_valid):
                self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=True)
            else:
                self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=False)
                # 运动电机
                if(not self.motor_msgs.reachTargetPos):
                    self.ctrl.LiftMovePos(self.target_height)
        # 过流保护
        if(self.ctrl.OverflowIProtect()):
            self.ctrl.MotorStop(self.JudgeMotorDirection(), self.motor_states, force=True)
            rospy.logwarn(f"'电机电流超过'{self.ctrl.GetOverflowILimit()},'A并且反馈速度为0，判定是卡住'")
            self.target_speed = 0
            self.ctrl.LiftMoveSpd(self.target_speed)
            self.ctrl.MotorStop(self.JudgeMotorDirectionWithSpeed(), self.motor_states, force=True)
            self.ctrl.MotorClearBug()
            # raise("电机电流超过11A并且反馈速度为0，判定是卡住")
    # 更新电机信息
    def UpdateDataAndCtrlThread(self, state) -> None:
        success_fps = 0
        while state == 1:
            self.motor_states = self.ctrl.ReadMotorData() #读取电机状态消息
            # ## 返回的字符串消息如果非空，publish电机消息并赋值当前升降柱高度
            # print(self.motor_states)
            if isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg):
                self.back_height = self.motor_states.back_lift_height
                success_fps = success_fps + 1
                if(success_fps > 5):  self.fps_error = 0
            # ## 如果反馈消息为空，计算错误帧率
            else: 
                success_fps = 0
                self.fps_error = self.fps_error + 1
            # 发布消息
            if(self.topicHz <= 0):
                self.PublishMotorMsgs()
            # [agx-fix 2026-09-29] 停稳时把当前高度落盘，供下次上电比对。
            # 只在可信 + 静止时写，运动中不写，避免把运动途中的瞬时值当基准。
            if(self.frame_valid and isinstance(self.motor_states, C_LiftingMotorCtrl.motor_msg)):
                if(self.motor_states.back_speed == 0):
                    self.SaveFrameFile(self.back_height)
            # ## 模式控制
            self.ModeCtrl()
            # time.sleep(0.02)
    
    # 发布电机各种信息线程
    def PubThread(self, state) -> None:
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # 发布消息
            self.PublishMotorMsgs()
            rate.sleep()
    
    def Main(self, state) -> None:
        '''
        主函数
        '''
        # 创建线程
        _thread.start_new_thread(self.UpdateDataAndCtrlThread, (state,))
        if(self.topicHz > 0):
            _thread.start_new_thread(self.PubThread, (state,))
        rospy.spin()


if __name__ == "__main__":
    # 实例化
    server = C_ROS_Server()
    # 调用服务端函数
    server.Main(1)
