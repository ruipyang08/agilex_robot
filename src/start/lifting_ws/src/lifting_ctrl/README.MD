# 升降柱电机控制功能包

## 830ABS,绝对值

lifting_ctrl_service_node.py 为开启服务控制电机节点`lifting_ctrl_server_node`，服务名为`/LiftingMotorService`

### ⚠ 断电后位置基准会丢失（2026-09-29 在 AgileX Ranger Mini + 830ABS 上实测确认）

**830ABS 的绝对位置基准是靠「下降压下限位 + 清零（0x4B/0x4C）」建立的，整机电断电重启后
这个基准会丢失，驱动器多圈位置回到 ≈0。** 这台机器上实测：在 lift 高度非 0 时整机电断电，
重启后 `backPos=-206` / `backHeight=0`，但 `downLimit=False`——柱子物理上还在行程中间。

所以**每次整机电断电后，开机第一步必须执行 `lift init`（mode=1）** 下降到下限位重建基准，
否则软件会拿着一份偏移的原点，「升到 600mm」会让物理柱子从当前位置再上 600mm 撞机械硬限位。

> 注意：json 里的 `initMode` 保持 `0`（不自动初始化）。改成 1 会让节点**开机时无人值守地
> 自动下降**，而且驱动在超时/过载时会走 `LiftTimeoutInit` 在**当前位置**软清零，把
> 「下限位坏了」这个硬件故障悄悄变成一个看似正常的任意零点。安全上应该由人显式执行 `lift init`。

**本仓库新增的坐标系列定门禁**（`[agx-fix 2026-09-29]`，上游未包含）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `~frame_check` | `true` | 关掉 = 不比对直接信任驱动器（不推荐，只是绕过检查） |
| `~frame_tolerance` | `20` | 上电比对容差，单位 mm |
| `~frame_file` | `~/.ros/lift_frame_m<id>.json` | 可信高度落盘位置 |

行为：把可信高度落盘 → 上电时和驱动器读数比对 → 差 > 容差判为「坐标系未标定」→
拒绝位置模式(`mode=0`)和**所有**速度模式(`-4`/`-7`，**含下行**)，返回 `-14`；50Hz 定位回路改为
force 急停不下发位移；对下游发布 `initState=False` 让 `lift_joint_bridge` 和控制台自动降级。
放行 `mode=1` 初始化 / `-2` 急停 / `-3` 清故障 / `-6` 清零。

> **为什么下行也拒绝**：实测证伪了「下行可以靠下限位开关兜底」的假设——`ModeCtrl` 的 `-4` 分支
> 每拍无条件重发 `LiftMoveSpd`，**没有到位/超时退出条件**，一条 `mode=-4 val=-20` 会让柱子
> 一直下行到撞机械限位（实测降到 -141mm 才被急停掐住，高度读数为负 = 软件行程下限也没拦住）。
> 速度模式和位置模式一样是「会一直跑到撞限位」的指令。

初始化跑完 = 基准重建，重新开放并立刻落盘。

同时修了 `MotorStop()` 的限位保护失效：原实现 `elif upLimit: SetStopFlag(True)` 会被紧随其后
的 `if back_lift_height >= 上限 ... else SetStopFlag(False)` 无条件覆盖，导致机械限位开关在方向判断
分支里从未生效。

## 850pro,增量式

lifting_ctrl_service_node_850pro.py 为开启服务控制电机节点`lifting_ctrl_850pro_server_node`，服务名为`/LiftingMotorService`

2025-3-6：很多客户发现电机只能初始化一次，之后就无效了，是因为速度控制下的限位被驱动器限制了，需要使用电机win上位机，然后设定can通讯组号为1，取消速度模式下限位，就可以恢复正常

## 启动升降柱控制节点

注意，_motor_id参数必需，参数为1,2,3,4中一个，跟升降柱的电机驱动器ID相关，无默认值

```bash
rosrun lifting_ctrl lifting_ctrl_service_node.py _motor_id:=1
rosrun lifting_ctrl lifting_ctrl_service_node_850pro.py _motor_id:=1
```

也可以指定启动的namespace:

```bash
rosrun lifting_ctrl lifting_ctrl_service_node.py _motor_id:=1 __ns:=lifter_1
rosrun lifting_ctrl lifting_ctrl_service_node_850pro.py _motor_id:=1 __ns:=lifter_1
```

也有自带的launch

使用默认参数,ns=lifter_1,motor_id=1

```bash
# 830abs
roslaunch lifting_ctrl start_motor.launch
# 850pro
roslaunch lifting_ctrl start_850pro_motor.launch
```

这是使用自定义参数

```bash
roslaunch lifting_ctrl start_motor.launch motor_id_:=1 lifter_ns:=lifter_1
roslaunch lifting_ctrl start_850pro_motor.launch motor_id_:=1 lifter_ns:=lifter_1
```

## 电机控制json参数文件

```json
{
    "portName": "/dev/lifter_1",//串口名字
    "initMode":0,//开始上电时的模式,0为位置控制,1为上电开始初始化,建议测试好功能后,将电机装到模组上面并测试好下限位正常再去改为1
    "baudRate": 38400 ,//串口波特率
   "timeout":0.02,//串口读取一帧数据的超时时间，数值越小读取的数据帧率越快，但是越可能出现丢帧，经测试，830abs为0.02,850pro为0.008最宜
    "dataFormat":"hex",//是否16进制转换
    "reductionRatio":12500,//执行器末端执行1mm，电机运行多少脉冲(1圈10000脉冲)，单位脉冲数
    "responseTimeout":60,//应答超时时间，单位s
    "motorSpd":2000,//电机最大速度设定，单位rpm
    "motorTime":0,//电机加减速时间(梯形曲线)，单位 1 为 65ms，假设设定为10,则加减速时间为650ms
    "callBackMode":1,//回调函数模式，0为阻塞应答，1为立刻应答，其它值无效
    "upLimitVal":600,//电机正方向运行最大值，单位mm
    "downLimitVal":5,//电机负方向运行最小值，单位mm
    "initSpd":-800,//初始化时电机运行速度，单位rpm
    "initPos":0,//初始化后执行器要到达的位置，单位mm
    "motorStallCurrent":15//电机堵转电流限幅,单位A
}
```

## service的数据类型如下

```C
int64 val 
int64 mode
---
int64 state
```

`val` 为要发送的目标值，与mode有关：

- mode为0，为位置控制模式，val值表示要运动的升降柱高度[0,700];
- mode为-4.为速度控制模式，val值表示要控制电机运动的目标速度，其中，速度为正表示升降柱向上，速度为负表示升降柱向下[-1000,1000] (单位：rpm);

`mode` 为控制模式，其中：

- 1为初始化电机，当电机识别到下限位时，结束初始化，如果下限位的线坏了，有过流保护，会直接退出该程序;
  
  注意：运行程序默认初始化，如果下限位出问题，电机转到最底下堵转电流会到11A以上，我写了电流保护，可以在堵转位置初始化为0位置，然后会执行到设定位置(也就是json中initPos这个参数)
- 0为控制电机运动，位置控制模式，控制升降柱指定高度;
- -2为控制电机急停;
- -3为清除错误并初始化电机(只是将电机的一些必须的变量赋值，比如另当前电机位置赋值给目标电机位置等等，实际没有电机操作);
- -4为控制电机恒定速度运动，其中，速度为正表示升降柱向上，速度为负表示升降柱向下,单位是mm/s;
- -5为设定位置模式下电机运动速度，单位rpm，需为正整数，范围为0~[ config里设定的电机最大速度 ];
- -6为清零位置指令
- -7为控制电机以多少RPM运行,需为正整数，范围为0~[ config里设定的电机最大速度 ]
- ...后期还可以添加

初始化：

程序第一次执行时，会读取对应id电机的config的initMode参数,如果为1,即默认初始化，初始化过程中屏蔽其它模式指令，只有初始化成功后，发送其他模式才会生效，如果想要知道是否初始化完成，发送mode值为1,查看反馈即可，1为初始化成功，-1为未完成

## state 为response，应答值

### 回调函数的选择

- SververCallbackImmediately 为立刻应答，即call一次服务端立刻response;
- SververCallbackBlock 为阻塞应答，即call一次服务端，直到完成发送的指令才会response。

### res反馈值的含义

- -14 初始化过程中没有将初始化标志位置为True，发送任何指令都会反馈-14，也就是初始化未成功(注意，如果初始化没有成功，之后发送的任何指令都会反馈-14);
- 1 & -1 (初始化成功为前提)
  - 初始化电机模式下(mode=1)，1表示初始化成功，-1表示初始化未完成(也就是初始化标志位被置为True);
  - 位置模式下(mode=0)，1表示到达了目标位置(就是目标位置与当前位置差值小于2mm)，-1表示没有到达(同上反之亦得);
  - 急停模式下(mode=-2)，1表示已经停止(也就是电机反馈速度为0rpm)，-1表示电机还没有停止;
  - 速度模式下(mode=-2)，1表示已经到达设定速度(其实是电机的rpm转化为末端执行器的mm/s速度后，设定速度与反馈速度差值小于5,就返回1,但是因为单位转换了，所以会很快返回res)，-1表示没有到达目标速度;
  - 设定位置模式速度(mode=-5)，1表示已经设定完毕，-1表示没有设定成功
- 0 如果发送的模式不是上述数字，同时初始化成功，反馈值为0;
- 2 & -2 (初始化成功)
  - 当上面的指令反馈为-1,也就是还没有满足指令成功的条件和或者指令执行失败，判断当前电机运行方向和当前指令模式：如果当前触碰了上限位and当前处于0或者-4模式and电机运行方向为负方向，跳过，否则反馈-2，表示触碰了上限位;如果当前触碰了下限位and当前处于0或者-4模式and电机运行方向为正方向，跳过，否则反馈2，表示触碰了下限位;
  - 注意，上下限位并不一定是所谓的上下，只用来表示电机运行方向，电机向负方向运行限制就是下限位，电机正方向运行限制就是上限位;
- -12 & -13 如果执行指令超过设定的responseTimeout应答超时时间，会自动将mode置为-2也就是急停模式，并且反馈-12，如果没有完成初始化还执行指令超时，则反馈-13(这个-13貌似不会反馈>_<);
- -9,如果电机断电同时程序还在运行，fps_error变量会累加，该值大于1时，反馈-9,说明电机离线(只有给service发消息读取反馈才行，非主动)
- -15执行电机位置清零指令,如果执行清零指令的时候电机速度不为0,反馈该值

## topic

这个node文件还会pub topic类型为`/LiftMotorStatePub`的消息，具体如下

```C
Header header       //数据帧header
uint8 motorId       //电机id
bool initState      //初始化状态

uint8 ctrlMode      //控制模式
uint8 backVol       //反馈电压
float64 backCurrent //反馈电流

uint8 stateBit      //状态位，用来表示电机状态，8位数据每位表示一种状态，下面连续的8个bool类型数据就是这个数值的具体表示
bool runState       //状态位，运行状态，False停机，True启动
bool overflowI      //状态位，过流状态,False则没有,True则触发,下面都是一样的
bool overflowV      //状态位，过压状态
bool encodeErr      //状态位，编码器故障
bool posBiasOver    //状态位，位置偏差过大
bool underVol       //状态位，欠压状态
bool overLoad       //状态位，过载标志
bool externalCtrlMode//状态位，外部控制标志，False-PC控制模式，True-外部PLC控制模式

int16 motorSpeed    //电机反馈速度，rpm
int16 backSpeed     //通过电机速度解算升降柱速度，单位：mm/s
uint8 locationComplete//目标位置，实际是电机内部驱动器的反馈值，不是你发送的
int32 backPos       //反馈位置,电机编码器数值,单位是脉冲数，1圈10000脉冲
int32 backHeight    //反馈升降柱高度，这个是用上面那个反馈位置解算得到(四舍五入),mm
int64 targetHeight  //目标高度，这个是你给service发送height的值(四舍五入),mm
int64 targetPos     //目标位置，这个是目标高度经过解算后得到的目标位置值,mm
bool upLimit        //上限位,0/1
bool downLimit      //下限位,0/1
int64 upLimitHeight //从json中读取到的上限位数值
int64 downLimitHeight//从json中读取到的下限位数值

bool reachTargetPos //是否到达目标位置，这个值是用反馈高度和目标高度相减是否大于1判断得到，大于1就False，小于1就True
int64 fpsError      //帧率错误，即电机离线计数值，发送-3的mode值后会软件清零
```

## 外层code含义

| 代码 |                    含义                  |
|:---:|:------------:|
|13000 - 1| 升降柱控制服务-->发送的指令执行成功|
|13001 - -1| 升降柱控制服务-->发送的指令执行失败|
|13002 - 2| 升降柱控制服务-->下限位触发，优先级比较高，会覆盖前两个|
|13003 - -2| 升降柱控制服务-->上限位触发，同上|
|13004 - -9| 升降柱控制服务-->电机离线，串口收发出现问题，优先级更高，会覆盖其他所有反馈|
|13005| 升降柱控制服务-->开启发布电机msg|
|13006| 升降柱控制服务-->关闭发布电机msg，但是topic名会保留，实际已经取消了发送|
|13007 - -13| 升降柱控制服务-->电机执行指令超时并且初始化失败|
|13008 - -14| 升降柱控制服务-->电机初始化失败|
|13010| 升降柱控制服务-->发送的指令不是设定好的几个指令码|
