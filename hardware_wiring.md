# 硬件核验清单

## 已知硬件

- 飞控：SpeedyBee F405 V4；真实任务必须确认已刷 ArduCopter。
- 机载电脑：Raspberry Pi 5。
- FPV：CADDX 蜗牛 Nano AR0234、TBS Unify 5G8 Pro32 HV；这套链路默认给飞手看，不等于树莓派视频采集。
- 定位/遥控/数传：MATEK GPS、BETAFPV ELRS Nano、SiK V3；GPS 在 5m 场地不能代替室内相对定位。
- 测距：MT-06；必须补充协议、接线、单位、时间戳、方向和量程验证。单点测距不能提供随机障碍物左右占据栅格。
- 投放：舵机顺序释放；必须配置每个仓位的输出通道、开闭 PWM、颜色装载反馈和释放反馈。

## 禁止直接假定

- 不把 Betaflight 当作 ArduCopter。
- 不把普通 FPV 图传当作 Pi 的 `cv2.VideoCapture` 输入。
- 不把 GPS fix 当作室内厘米级位置闭环。
- 不把 MT-06 的一个距离数当成左右无碰撞空间。
- 不把 MAVLink `COMMAND_ACK` 当成货物已落地。

## 接线/参数必须由台架核实

在 `mission_points.json` 中把 `commissioning` 对应项改为 `true` 之前，记录：飞控固件版本、真实 UART/波特率、MAVLink 心跳、`LOCAL_POSITION_NED` 来源、下视距离消息方向、相机采集节点、舵机输出和 ArduPilot failsafe。原项目的 UART3/UART6 和 SERVO5 只能作为待核对假设。
