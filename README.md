# ROBOT_DOG

该程序使用4个线程，对电机进行控制，需要系统安装实时补丁，电机控制频率1khz

---

## 6月18日：

- 使用沁恒上位机修改usb to 485的设备号，修改为`ttyMotorA`,`ttyMotorB`,`ttyMotorC`,`ttyMotorD`。支持任意usb端口插入。

### 首先运行：
```bash
for dev in /dev/ttyACM*; do
echo -n "$dev → "
udevadm info --query=property --name=$dev | grep -E '^(ID_SERIAL_SHORT|ATTRS{serial})='
done
```
### 将会得到类似输出：
```bash
/dev/ttyACM0 → ID_SERIAL_SHORT=MOTO_A003
/dev/ttyACM1 → ID_SERIAL_SHORT=MOTO_A001
/dev/ttyACM2 → ID_SERIAL_SHORT=MOTO_A004
/dev/ttyACM3 → ID_SERIAL_SHORT=MOTO_A002
```
### 使用Vim新建规则：
```bash
sudo vi /etc/udev/rules.d/99-ch343p.rules
```
### 写入规则：
```bash
# MOTO_A001
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ENV{ID_SERIAL_SHORT}=="MOTO_A001", \
SYMLINK+="ttyMotorA", MODE="0666"
# MOTO_A002
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ENV{ID_SERIAL_SHORT}=="MOTO_A002", \
SYMLINK+="ttyMotorB", MODE="0666"
# MOTO_A003
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ENV{ID_SERIAL_SHORT}=="MOTO_A003", \
SYMLINK+="ttyMotorC", MODE="0666"
# MOTO_A004
SUBSYSTEM=="tty", KERNEL=="ttyACM*", ENV{ID_SERIAL_SHORT}=="MOTO_A004", \
SYMLINK+="ttyMotorD", MODE="0666"
```
### 更新规则：
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```


## 6月19日：

- 完成所有机器狗电机零点标定，完成机器狗站立demo。机器狗保护机制待添加，电机初始化漏洞待完善，imu待添加...

## 6月20日：

- 增加机器狗起立状态机，修改线程的初始化，修复电机的初始化漏洞。机器狗保护机制待添加，imu待添加

## 6月23日：

- 完成机器狗保护函数，imu读取完成，imu数据更新率稳定200hz，符合官方手册。整体逻辑无误，代码框架待整理

## 6月24日：

- 修复imu间断丢包，imu数据获取率必须低于200hz，上位机线程延时必须高于5ms，否则与imu时间冲突后，导致间断性丢失部分数据。
- 对imu获取数据函数引入列表，方便后续根据强化学习调整imu获取的数据
- lcm通信待完成，强化学习代码待学习

## 7月13日

- 完成强化学习部署框架，使用cuda加速C++推理，使用libpytorch库，需要手动去官方下载
- 增加rl状态机
- 修复imu坐标系反转问题
- 修复底层电机控制线程中的锁冲突问题
- 修复rl启动时没有预热导致的力矩突变
- 新增力矩保护限制，机身侧翻转限制，电机位置限制
- 待解决：rl算法运行一段时间后机身高度开始自动缓慢下降，排除力矩保护，疑似算法逻辑中目标位置可能没有成功更新，或者观测网络的滤波导致了rl的网络的问题