# 夹爪串口改造——电控审核清单

状态：**源码待审核，尚未编译到正式固件、尚未烧录，硬件未生效。**

## 审核范围

只审核以下文件中的夹爪相关增量：

- `application/hand/hand_task_interface.h`
- `application/hand/hand_task_interface.c`
- `components/modules/Custom_contrllor/arm_serial_protocol.h`
- `components/modules/Custom_contrllor/arm_serial_protocol.c`
- `components/modules/Custom_contrllor/arm_visual_control.h`
- `components/modules/Custom_contrllor/arm_visual_control.c`
- `components/modules/Custom_contrllor/visual_trajectory_h7_adapter.c`
- `components/modules/Custom_contrllor/tests/test_arm_visual_claw.c`（仅 PC 测试，不加入固件）

本次没有修改 J1～J6 的 `MAP_K/MAP_D`、关节限位、轨迹插值、电机输出、
遥控器档位、急停、掉线、堵转逻辑。上述文件中若存在此前未提交的六轴改动，
不属于本次夹爪改造，电控应按原流程单独审核。

## 串口协议表

| 方向 | TYPE | Payload | 含义 |
|---|---:|---|---|
| 视觉→电控 | `0x06 CLAW_COMMAND` | `action:u8` | `1=OPEN, 2=CLOSE, 3=STOP` |
| 电控→视觉 | `0x83 CLAW_RESULT` | `result:u8` | `0=完成但未验证, 1=中断, 2=超时, 3=故障` |
| 电控→视觉 | `0x84 CLAW_STATE` | `state:u8, flags:u8` | `0=UNKNOWN,1=OPENING,2=OPEN,3=CLOSING,4=CLOSED,5=FAULT`; `flags.bit0=verified`，当前固定为 0 |

`CLAW_STATE` 状态变化立即发送，并以 100 ms 周期发送。上电为 `UNKNOWN`，
不自动动作。夹爪动作使用独立 500 ms 心跳超时和 2000 ms 动作超时。

## 任务边界

- 串口任务只调用 `hand_claw_request()` 投递请求，不直接操作 GPIO。
- `hand_claw_poll()` 仍在 HandTask 1 ms 循环内独占 PE13/PE9，保持原 700 ms 脉冲。
- OPEN/CLOSE 在 `HAND_MODE_RC2_CTRL` 下拒绝；STOP 始终可以投递。
- 动作中 STOP/通信超时同时拉低 PE13、PE9，并把开环估计设为 UNKNOWN。
- OPEN/CLOSED 是程序执行估计，不代表夹住水果，也不代表机械端点被传感器验证。

## PC 测试（不会访问 GPIO）

测试使用假时钟与假夹爪钩子验证 OPEN、CLOSE、STOP、重复命令、2 s 超时、
500 ms 通信中断、状态变化、10 Hz 周期帧、ACK 和结果序号。测试文件不加入
Keil 工程，不影响固件任务调度。

电控审核后还必须：使用正式工程编译、检查告警、烧录，再在电机不上使能的条件下
示波器检查 PE13/PE9 互斥及 700 ms 脉冲，之后才能进行真机夹爪验收。
