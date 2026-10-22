# 设置串口别名
## 查看ch110 USB口的设备ID号
```
$lsusb
```
## 新建udev规则，在/etc/udev/rule.d目录下新建文件IMU.rules。内容如下：
```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="设备号", ATTRS{idProduct}=="设备号", MODE:="0666", GROUP:="dialout",  SYMLINK+="ch110"
```
## 重新加载规则
```
$sudo service udev restart
$sudo service udev reload
```
note：重新拔插usb口之后，查看/dev目录下有没有ch110。
# 启动imu
## 无rviz启动
```
$roslaunch serial_imu  imu_msg.launcg
```

## 有rviz启动
```
$roslaunch serial_imu  imu_rviz.launcg
```

## 查看imu原始数据
```
$rostopic echo /imu/date_raw
```