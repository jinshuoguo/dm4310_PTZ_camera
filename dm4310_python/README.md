# DM4310 云台相机

基于双轴 DM-J4310 电机、USB2FDCAN、相机视觉、手柄与局域网手机端的云台相机控制项目。

当前设备在调试工具中显示为 `COM10-USB2FDCAN`，应使用 USB2FDCAN SDK，
而不是旧版 COM 串口封装。

## 安装

依赖和官方 Windows DLL 已放入当前目录。请使用 Conda base 的 Python 3.13，
不要使用指向 Python 3.15 alpha 的 `py` 启动器。

运行前必须完全关闭 DM_Tools 和 USB2CAN 图形工具，确保 COM10 没有被占用。

## USB2FDCAN 首次低速测试

1. 完全关闭 DMTool 和 USB2CAN 图形程序。
2. 固定或悬空电机。
3. 在 `(base)` 终端运行：

```powershell
python dm4310_usb2fdcan.py
```

默认参数：

- Slave ID：0x01
- Master ID：0x11
- CAN：经典 CAN 1 Mbps
- 速度：0.5 rad/s
- 时间：5 秒
- 缓启动、缓停止时间：各 1.5 秒（S 曲线）
- MIT 参数：KP=0、KD=1、Torque=0

程序结束、报错或按 `Ctrl+C` 时都会发送失能命令。

根据当前调试工具截图，实际 Slave ID 是 `0x01`，Master ID 是 `0x11`；
新脚本也以这两个值为默认值。

修改速度和运行时间：

```powershell
python dm4310_usb2fdcan.py --speed -0.5 --duration 3
```

用 `--ramp-time` 调整启动和停止的平滑时间，例如各用 2 秒：

```powershell
python dm4310_usb2fdcan.py --speed 0.5 --duration 8 --ramp-time 2
```

自动正反转（每个方向运行 5 秒，换向前零速停顿 0.5 秒）：

```powershell
python dm4310_usb2fdcan.py --speed 0.5 --duration 5 --ramp-time 1.5 --bidirectional
```

可用 `--pause-time 1` 将换向停顿改为 1 秒。`--duration` 表示每个方向
各自的运行时间，因此上述正反转流程总时长约为 10.5 秒。

## 缓慢移动到零位

确认从当前位置到 `0 rad` 的运动路径没有机械干涉，然后运行：

```powershell
python dm4310_home.py --duration 20 --max-speed 0.3 --max-acceleration 0.3 --min-speed 0.12 --kp 2 --kd 1 --capture-ki 1 --integral-torque-limit 0.35 --tolerance 0.005
```

程序采用两阶段归零：远离零点时根据剩余制动距离计算目标速度，同时通过
`--min-speed` 克服静摩擦；进入默认 `0.2 rad` 捕获区后，速度先降到零，
位置 KP 再用 1 秒从零平滑增加到设定值。进入零点容差并低速停稳后才算
归零成功。默认实测速度超过 `2 rad/s` 会立即停止并失能，位置误差连续
3 秒没有改善也会停止。

捕获阶段还会使用带限幅的积分前馈力矩消除减速器静摩擦或恒定负载造成的
稳态误差。默认补偿最多 `0.35 N·m`，运行输出中的 `Tff` 是当前补偿值。
默认归零误差要求为 `0.005 rad`，需要连续稳定 0.5 秒，并在零位继续保持
2 秒后才检查最终误差。

它不会改写编码器零点；失能后也不会继续提供保持力矩。`--duration` 是允许
的最大归零时间，超时会直接失能。MIT 位置反馈在 ±12.5 rad 处会回绕，脚本
不会再把首次反馈直接作为绝对位置指令，以免跨量程时突然高速回转。

## 网页实时角度控制

完全关闭 DMTool，然后在 `(base)` 终端运行：

```powershell
python dm4310_web.py
```

当前机构的前端方向映射默认为反向（`--direction -1`），使画面正方向与实际
机构正方向一致。如果更换安装方向后需要恢复电机原始坐标，可运行
`python dm4310_web.py --direction 1`。

程序会打开 `http://127.0.0.1:8765/`。操作顺序：

1. 点击“连接设备”，此时电机仍保持失能。
2. 点击“安全归零”，等待状态变成 `TRACKING`。
3. 拖动中央角度盘，或使用 ±45°/±90°/回零快捷按钮。
4. 结束时点击红色“立即失能”，再关闭终端。

浏览器只发送目标角，本地 Python 控制线程以 100 Hz 生成限速、限加速度的
MIT 位置轨迹。目标范围限制为 ±3 rad，实测速度超过 2 rad/s、跟随误差超过
0.8 rad、反馈中断或前端心跳丢失 1 秒都会自动失能。服务默认只监听本机，
不要使用 `--host 0.0.0.0` 暴露到局域网。

## 键盘左右转动控制

关闭网页服务、DMTool 和 USB2CAN 工具，在前台终端运行：

```powershell
python dm4310_keyboard.py
```

- 按住 `←`：向左转动。
- 按住 `→`：向右转动。
- 每按一次 `↑`：速度增加 `0.5 rad/s`，最高 `5 rad/s`。
- 每按一次 `↓`：速度减少 `0.5 rad/s`，最低 `0.5 rad/s`。
- 松开方向键：平滑减速到零。
- 按住空格：制动。
- 按 `Esc` 或 `Ctrl+C`：停止并失能。

默认速度 `2.5 rad/s`，按下和松开方向键的速度指令分别在 `0.1 s` 内完成
加速与减速（等效 `25 rad/s²`），并以脚本启动位置为中心设置 ±3 rad 软限位。
可用 `--speed 1 --ramp-time 0.2 --position-limit 1.5` 降低响应和范围。
当前机构方向默认 `--direction -1`，若方向不符可改为 `--direction 1`。

## 两台电机同步键盘控制

第二台电机使用 CAN ID `0x02`、Master ID `0x12` 时运行：

```powershell
python dm4310_dual_keyboard.py
```

左右键只控制电机1，上下键只控制电机2；两台可以同时按键、独立运动。
空格同时制动，任意一台反馈中断、状态异常或超速时，脚本都会停止并失能
两台。若第二台机械安装方向相反，可运行：

```powershell
python dm4310_dual_keyboard.py --direction-1 -1 --direction-2 1
```

两台电机共享 CAN_H/CAN_L 总线并使用不同 ID；24V 电源应并联供电。总线
只在物理两端设置 120Ω 终端电阻，不要在每个中间节点重复增加终端电阻。

## Xbox 左摇杆控制两台电机

连接 Xbox 手柄并关闭 DMTool、网页服务等占用 USB2FDCAN 的程序，然后运行：

```powershell
python dm4310_dual_gamepad.py
```

左摇杆横轴控制电机1、纵轴控制电机2，偏转方向决定旋转方向，偏转幅度线性
决定速度。纵轴满行程默认对应 `5 rad/s`，横轴为其 60%，即 `3 rad/s`；
摇杆回中后在约 `0.25 s` 内停止，默认圆形死区为 10%。主轴锁定会在明显
上下推动时忽略少量左右偏移，减少误触；接近斜向推动时仍可同时控制两台。
按住 `A` 立即制动；按 `B`、`Back`、键盘 `Esc` 或 `Ctrl+C` 会停止并
失能两台电机。手柄运行中断开也会触发失能。

按一下 `X` 会记录两台电机相对脚本启动位置的当前位置；之后按一下 `Y`，
两台电机会以默认最高 `1.5 rad/s` 自动返回记录位置。回位达到 ±0.02 rad
并稳定后结束。回位过程中拨动左摇杆或按 `A` 会立刻取消回位并允许人工
接管。可使用 `--return-speed 0.8` 降低回位速度，或使用
`--return-tolerance 0.01` 提高回位精度。

按一下手柄十字键 `↓` 开启人脸跟随，再按一次关闭。首次开启时会按名称打开
`icspring camera` 外置相机和预览窗口，不会再误用笔记本的
`Integrated Camera`。人脸检测使用 MediaPipe BlazeFace Short Range，
并以双眼和鼻尖三个关键点的中心作为云台目标，取代了原先容易受光照、侧脸
影响的 Haar 分类器；画面中绿色框标出当前跟随的人脸和置信度。默认最大
跟随速度 `1.5 rad/s`、加速度 `4 rad/s²`，人脸靠近画面中心 8% 的区域内
不移动。人脸丢失超过 0.5 秒会减速停止，不会自行搜索。跟随期间拨动左摇杆
或按 `A` 会立即退出跟随并交回手动控制。当前机构的人脸水平跟随方向默认
为 `--face-direction-x -1`，摇杆的左右方向不受这个参数影响。

按一下手柄十字键 `↑` 开启手部跟随，再按一次关闭。手部跟随与人脸跟随
共用 `icspring camera`，两种模式不能同时运行。程序使用 MediaPipe 的 21
个手部关键点，并用掌根和四个指根的平均中心控制云台，因此手指张合不会
明显改变跟随目标。默认最大速度 `1.5 rad/s`、加速度 `4 rad/s²`、画面
中心死区 10%；手部丢失超过 0.5 秒会减速停止。拨动摇杆或按 `A` 会立即
退出手部跟随。

调整手部跟随速度、方向或灵敏度：

```powershell
python dm4310_dual_gamepad.py --hand-speed 1 --hand-gain 2 --hand-direction-x 1 --hand-direction-y -1
```

如果更换了其他型号的摄像头、跟随方向相反或不需要预览窗口，可运行：

```powershell
python dm4310_dual_gamepad.py --camera-name "新相机名称" --face-direction-x -1 --face-direction-y -1 --no-camera-preview
```

人脸与手部检测依赖 `mediapipe`、`opencv-contrib-python`，安装全部依赖可运行：

```powershell
python -m pip install -r requirements.txt
```

按一下手柄十字键 `←` 开始录制两台电机的位置—时间轨迹，再按一次 `←`
结束录制。随后按十字键 `→`，设备会先以安全速度返回录制起点，再按记录的
时间顺序复现刚才的双电机动作。默认每 0.02 秒采样一次，最长录制 30 秒；
复现速度限制为 `3 rad/s`、加速度为 `6 rad/s²`。复现期间拨动左摇杆或按
`A` 会立即取消并交回人工控制。轨迹使用自适应时间轴：误差超过 0.3 rad
开始自动减慢，达到 0.6 rad 时暂停轨迹时间并等待电机追上；只有误差超过
1.2 rad 的硬保护阈值才会停止并失能。因此录制动作快于复现速度限制时，
动作会自动放慢而不会直接报错。自动
返回录制起点时，两台电机都进入 ±0.05 rad 范围并停稳后开始复现；这个
容差可通过 `--replay-start-tolerance` 调整。

调整最长录制时间或降低复现速度：

```powershell
python dm4310_dual_gamepad.py --record-max-duration 60 --replay-speed 1.5 --replay-acceleration 3
```

降低最高速度、增大死区或改变摇杆响应曲线：

```powershell
python dm4310_dual_gamepad.py --max-speed 3 --x-speed-scale 0.5 --deadzone 0.15 --response-exponent 1.5
```

`response-exponent=1` 为线性；大于 1 时中心区域更细腻。可通过
`--axis-lock-ratio 0` 关闭主轴锁定。两台电机仍分别受到启动位置 ±3 rad
软限位、反馈超时和超速保护。当前手柄控制的两轴方向默认均为 `1`；如果
以后更改电机安装方向，可分别使用 `--direction-1 -1` 或
`--direction-2 -1` 反转对应电机。

## 局域网手机遥控两台电机

先让手机和电脑连接同一个 Wi-Fi，完全关闭 DMTool 和其他
会占用 USB2FDCAN 的脚本，然后在电脑上运行：

```powershell
python dm4310_mobile.py
```

终端会输出一个带随机控制码的局域网地址，例如：

```text
http://192.168.1.23:8766/?token=xxxxxxxx
```

用手机浏览器打开这个完整地址。如果 Windows 弹出防火墙询问，
只允许“专用网络”即可。页面中先点“连接设备”，将摇杆回中后
长按“使能” 1 秒；摇杆左右控制电机1，上下控制电机2，幅度
决定速度。页面上可实时调整最高速度和加减速时间。
手机端专为横屏操作设计：左侧是摇杆和两电机遥测，中间是实时画面，
右侧是跟随、记录/复现、运动参数和安全按钮。竖屏时页面会提示旋转手机。

手机页面也包含与 Xbox 手柄相同的扩展功能：

- `CAM` 开启 `icspring camera` 局域网画面回传；默认约 15 FPS。
- `人脸跟随` / `手部跟随` 分别对应手柄十字键 `↓` / `↑`，点击后
  会自动打开回传画面并显示检测框、关键点与画面偏差。
- `记录位置` / `返回位置` 对应手柄 `X` / `Y`。
- `动作录制` 点一次开始、再点一次停止；`动作复现` 会先自动
  返回录制起点，再按记录时间轴复现，对应手柄十字键 `←` / `→`。
- `取消自动` 会停止跟随、回位或复现；在任何自动模式中拨动
  手机摇杆也会立即切回手动控制。

视频只在手机打开回传窗口时传输；关闭窗口后会断开手机端视频流以
节省带宽，但正在进行的人脸/手部检测不会中断。需要降低网络负载时：

```powershell
python dm4310_mobile.py --stream-fps 10 --jpeg-quality 60
```

默认仍使用已验证的两组 ID：电机1 `0x01/0x11`，电机2
`0x02/0x12`，经典 CAN 1 Mbps。最高速度默认 `3 rad/s`，水平轴是
该值的 60%，两轴都受启动位置 ±3 rad 软限位保护。手机停止发送
摇杆数据 0.25 秒后会先减速到零；断联超过 1.2 秒会自动失能
两台电机。手机页面上的“急停 / 失能”也可随时同时失能两台。

需要改端口、方向或初始速度时：

```powershell
python dm4310_mobile.py --port 9000 --max-speed 2 --direction-1 -1 --direction-2 1
```

如果手机打不开地址，依次检查：电脑网络类型是“专用网络”、
手机没有使用蜂窝网络/VPN，以及路由器没有开启客户端隔离。不要将这个
服务映射到公网。

## 拖动电机2示教电机1

让电机2以低力矩助力模式手动拖动，电机1跟随其相对角度：

```powershell
python dm4310_teach_follow.py
```

脚本启动时两台电机的位置分别作为相对零点，不会改写编码器零位。电机2
默认先安全失能，再以零位置/速度刚度使能，仅输出最高 `0.10 N·m` 的低速
顺向摩擦补偿；补偿随速度增加而衰减，在 0.6 rad/s 时归零，降低松手自转
风险。电机1使用限速速度闭环跟随。默认比例 1:1、最高速度 2.5 rad/s、
相对软限位 ±3 rad。

如果需要反向跟随或降低速度：

```powershell
python dm4310_teach_follow.py --ratio -1 --max-speed 1.5
```

调整助力或恢复完全失能拖动：

```powershell
python dm4310_teach_follow.py --assist-torque 0.15
python dm4310_teach_follow.py --assist-torque 0
```

运行期间按 `Esc` 或 `Ctrl+C` 会失能两台；任一反馈异常、电机超速或电机2
状态与所选助力模式不符也会立即停止。

`dm4310_run.py` 是旧版串口 USB2CAN 的兼容脚本，不适用于当前 USB2FDCAN。
