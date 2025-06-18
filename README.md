该程序使用4个线程，对电机进行控制，需要系统安装实时补丁，电机控制频率1khz

# 6月18日：
使用沁恒上位机修改usb to 485的设备号，修改为ttyMotorA,ttyMotorB,ttyMotorC,ttyMotorD。支持任意usb端口插入。
## 首先运行：
for dev in /dev/ttyACM*; do
echo -n "$dev → "
udevadm info --query=property --name=$dev | grep -E '^(ID_SERIAL_SHORT|ATTRS{serial})='
done

## 将会得到类似输出：
/dev/ttyACM0 → ID_SERIAL_SHORT=MOTO_A003
/dev/ttyACM1 → ID_SERIAL_SHORT=MOTO_A001
/dev/ttyACM2 → ID_SERIAL_SHORT=MOTO_A004
/dev/ttyACM3 → ID_SERIAL_SHORT=MOTO_A002

## 使用Vim新建规则：
sudo vi /etc/udev/rules.d/99-ch343p.rules

## 写入规则：
'''
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
'''
更新规则：
sudo udevadm control --reload-rules
sudo udevadm trigger
