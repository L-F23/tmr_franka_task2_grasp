# TMR Franka Task 2 — Red Strip Detection

从移动操作机器人主摄 ZED-M 的实时 JPEG 中检测桌面红色条状标签。

正式入口会先松开左夹爪、将 Spine 调到 `0.600 m`、仅复位左臂并验证误差，然后记录实测状态。初始化不向右臂发送任何命令；任一步失败都会中止启动。

## 安装

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

机器人本机通常已经安装 NumPy 和 OpenCV，也可以直接使用 `/usr/bin/python3`。

## 一行命令

```bash
cd /home/aup/tmr_franka_task2_grasp && /usr/bin/python3 start_project.py --annotated-output outputs/red_strip.jpg
```

初始化目标位于 `config/initial_pose.json`，每次成功启动的实测结果写入并覆盖仓库跟踪文件 `config/latest_initial_state.json`。正式运行必须使用 `start_project.py`；`detect_red_strip.py` 仅保留用于不控制机器人的离线调试。每次初始态或项目代码更新经确认后都应提交并推送到远端仓库。

默认读取：

```text
http://172.16.0.50:18082/tmr_zed_latest.jpg
```

输出 JSON 包含：目标中心像素坐标、归一化坐标、四角坐标、长轴方向、像素长度/宽度、面积和置信度。退出码为 `0` 表示检测成功，`2` 表示没有找到目标，`3` 表示相机画面没有更新。

离线图片：

```bash
/usr/bin/python3 detect_red_strip.py --image frame.jpg --all --annotated-output outputs/result.jpg
```

桌面区域可按现场视角调整：

```bash
/usr/bin/python3 detect_red_strip.py --roi-top 0.40 --roi-bottom 0.95
```

## 测试

```bash
/usr/bin/python3 -m pytest -q
```

检测采用 HSV 红色双区间、形态学去噪和旋转矩形几何约束，能处理红色色相在 HSV 0/179 边界两侧的情况。HTTP 输入会检查 `Last-Modified`，不会把重复下载的旧 JPEG 当作新帧。

## 黑色底座与灰色导热垫横向对准

只观察并输出决策：

```bash
/usr/bin/python3 align_to_thermal_pad.py
```

允许实际横移：

```bash
/usr/bin/python3 align_to_thermal_pad.py --execute
```

程序优先使用左腕画面闭环居中；初始姿态的现场标定为腕部图像上/下分别对应机器人左/右，因此目标在画面上方时左移、在下方时右移。腕部未见目标时用主摄水平位置提供搜索方向。每步横移默认仅 `0.02 m`。当前 Task 2 按现场要求关闭双雷达碰撞门，只使用新鲜里程计、静止状态、控制租约、命令订阅者和超时进行运动约束；每次退出仍连续发送零速。

底盘运动通过 SSH 在 `tmr-user@172.16.0.50` 本机执行，固定使用与底盘控制器一致的隔离 ROS Domain 97；机械臂不接收运动命令。

## 导热垫末端抓取 FK/IK

按固定顺序执行“左臂初始位复位 → 底盘视觉居中 → 连续静止确认 → D405 原始深度配准 → 手眼坐标变换 → FK/IK”：

```bash
cd /home/aup/tmr_franka_task2_grasp && source /home/aup/tmr_env.sh && /usr/bin/python3 run_thermal_pad_pipeline.py --execute
```

流程的最后阶段仅解算和校验，不闭合夹爪、不执行抓取轨迹；结果写入 `config/latest_thermal_pad_ik.json`，标注图写入 `outputs/thermal_pad_ik.jpg`。`config/thermal_pad_pick.json` 中的 `kinematics.avoid_collisions` 当前固定为 `false`，因此不会调用 MoveIt 场景碰撞门。

`thermal_pad_ik.py` 会同时要求：导热垫中心落在左腕图像 Y 方向 ±35 px、底盘速度连续 1 秒低于阈值、左臂处于记录的初始关节位、RGB/深度时间差不超过 0.1 秒、7 帧深度中至少 5 帧在三维空间一致、手眼标定与 FK 一致、所有 IK 点连续且关节步长受限。任一条件不满足即以零动作退出。靠近机器人且向下搭的一端按当前初始姿态标定为导热垫长轴的图像 `+X` 端，参数集中在 `config/thermal_pad_pick.json`。

入口在复位前会以 `--state-only` 运行 `bootstrap_left_runtime.py`：确认左臂硬件、错误恢复和两路状态广播可用，然后由原生低速 PTP 动作复位。正式入口不会启用阻抗控制器，避免 FCI 重连后控制器读到空目标或过期目标。

FK/IK 请求会读取 Spine 实测高度（当前为 `0.600 m`），并显式组合“整机基座 → 左臂安装座 → FCI 实测末端”的坐标链；禁止把 FCI 的左臂局部位姿直接与 MoveIt 整机坐标比较。实测末端相对 `link8` 的法兰偏移保存在 `config/thermal_pad_pick.json`。

### 导热垫抓取、提起与脱离动作设计

FK/IK 规划器还会生成完整但默认禁止实机执行的动作序列：在检测到的夹取末端高度，把夹爪伸出/指尖朝向轴保持为地面 `+X`，把两指开合轴保持为地面 `+Z`；因此夹爪整体水平且两个指一上一下。夹爪张开后沿 `+X` 前进到目标并闭合；以该姿态沿 `+Z` 上提 `0.12 m`，沿 `+X` 向远端移动 `0.12 m`；第一段到此保持。第二段经独立授权后先沿 `-Z` 下降 `0.22 m`，再保持同一水平姿态沿 `-Z/-X` 斜向下降和内缩。斜向移动完全结束后先松开夹爪，随后才把夹爪朝向轴从 `+X` 向地面 `-Z` 小角度旋转，并同时沿 `-X` 继续内缩，形成“倒铲斗”脱离动作。旋转和位置使用同步插值，每个中间点均请求 IK 并检查状态有效性。

坐标和参数位于 `config/thermal_pad_pick.json` 的 `motion_sequence`。所有方向和高度先在与地面平行的整机 `base` 参考系中表达；FCI 返回的肩部局部末端位姿必须通过“整机根 → Spine → 左臂安装座”变换后才能使用，严禁直接在 `left_fr3v2_link0` 肩部坐标中加减这些位移。这里的“与此前夹取末端同高”使用变换后的地面参考 Z；第一段上提 `0.12 m`、远移 `0.12 m` 和第二段下降 `0.22 m` 都是地面轴上的相对位移，因此不依赖地面参考系原点具体落在哪里。

用户明确指定的第一段上提/远移 `0.12/0.12 m` 和第二段下降 `0.22 m` 已固定；开放前进距离、斜向下降/内缩距离以及倒铲斗角度尚无现场尺寸依据，目前仅为保守设计初值，并标记 `parameters_calibrated: false`。当前倒铲斗角度暂定 `15°`：它改变的是夹爪朝向，从水平向下俯转，不是向 `+X` 做位置修正。在完成 TCP、桌面 PlanningScene、夹持视觉验证和小步现场标定前，任何执行器都必须拒绝运行该放置/脱离序列。

完整动作按第 6 步结束位置拆成两个独立段。第一段 `pick_lift_and_far_transfer` 完成夹取、上提 `0.12 m` 和向远端移动 `0.12 m`，在 `carry_far_12cm` 保持姿态并退出；第二段 `lower_place_and_release` 只有在重新确认第一段终点、物体仍被可靠夹持并获得单独授权后，才从下降 `0.22 m` 开始。禁止第一段完成后自动串行进入第二段。

## 2 m 底盘接近与导热片完整流程

### 快速启动（实机服务已正常）

推荐从机器人主机 `.100` 使用统一入口。默认只准备运行环境，不移动底盘、机械臂或夹爪：

```bash
cd /home/aup/tmr_franka_task2_grasp && source /home/aup/tmr_env.sh && /usr/bin/python3 -u quick_start.py
```

只做只读检查：

```bash
/usr/bin/python3 -u quick_start.py --check-only
```

确认机器人处于本任务指定起点、夹爪状态正确、现场无人且急停可触达后，一行启动完整流程：

```bash
cd /home/aup/tmr_franka_task2_grasp && source /home/aup/tmr_env.sh && /usr/bin/python3 -u quick_start.py --execute
```

快速入口与所有其他运动入口共用同一把单实例锁，先检查核心 ROS 服务；随后并行恢复左臂的“仅状态”运行态和底盘隔离运行栈，最后要求主摄与左腕帧序号持续递增。健康服务会直接复用，不会重启；只允许重启无硬件所有权的三相机 HTTP 桥。左臂仅在硬件不是 `active` 或状态流异常时执行一次有界恢复，顺序固定为“停活动控制器 → ErrorRecovery → 激活硬件 → 激活状态广播器”。若 FR3、Robotiq、Spine、D405 或 IK 等核心驱动缺失，入口不会猜测或启动第二实例，而是阻塞并要求先运行参考项目的冷启动助手：

```powershell
powershell -ExecutionPolicy Bypass -File .\grasp\scripts\start_tmr_system.ps1
```

`--execute` 会把刚生成、有效期仅 20 秒且明确标记“未发送运动”的准备记录交给完整流程，避免重复初始化；此后每个底盘短步仍重新检查里程计、双雷达和控制租约。FCI 实时环始终留在机器人本机，底盘 Humble Domain 97 与机械臂 Jazzy Domain 0 不做高频 DDS 混接。

完整流程入口为：

```bash
cd /home/aup/tmr_franka_task2_grasp && source /home/aup/tmr_env.sh && /usr/bin/python3 -u run_full_thermal_pad_cycle.py --execute
```

若底盘已经位于黑色底座抓取参考点、左臂也已经处于标定的抓取预备位，可跳过服务启动、2 m 运输、黑底座搜索、桌边校准和进入预备位的动作，从预备位一行运行到释放后复位：

```bash
cd /home/aup/tmr_franka_task2_grasp && source /home/aup/tmr_env.sh && /usr/bin/python3 -u run_from_pregrasp_to_finish.py --execute
```

该入口不会自动启动或重启任何机器人服务。它先只读检查核心 ROS 服务、主摄和左腕实时帧，再用实测关节及 FK 末端位姿核对标定预备位；任一条件不满足都在第一条运动命令前退出。通过后依次前进 `2.5 cm` 到抓取位、视觉左右精对准、闭爪、垂直上提 `12 cm`、红垫粗定位及左腕 Y 闭环、前移/下降各 `12 cm`、后退倾转释放、上抬脱离并回到左臂初始位。运行记录位于 `config/latest_pregrasp_to_finish.json`；默认不带 `--execute` 只打印计划。

Task 2 的标准初始位置定义为左臂复位位姿。若从该标准初始位置开始，使用下面的主入口；它在健康检查后先复位左臂，立即将左臂送到抓取预备位并用关节数据与 FK 复核，然后接续上述抓取、运输、放置和最终复位流程：

```bash
cd /home/aup/tmr_franka_task2_grasp && source /home/aup/tmr_env.sh && /usr/bin/python3 -u run_task2_from_initial.py --execute
```

这个入口同样不会启动机器人服务，也不会执行 2 m 初始底盘运输、黑底座粗搜索或桌边校准。底盘必须已经位于黑底座抓取参考位置；不带 `--execute` 时只输出动作顺序。

入口先把左臂恢复到运输安全初始位，并通过底盘主机的 `19_ensure_navigation_stack.sh` 自动恢复隔离任务栈、关闭冲突的旧控制栈，然后用一条连续里程计闭环轨迹将底盘向右移动 `2.0 m`，中途不再分段停走。Task 2 自己的底盘运动器通过 SSH 标准输入在 `.50` 本机运行，不读取或覆盖 task3 文件；整段轨迹持续监控新鲜里程计、底盘速度控制租约和命令订阅者，异常时发送零速并终止。之后依次执行黑色底座/灰色导热片粗定位、桌边前后位置判定和抓取位姿到达。闭爪前必须运行 `pregrasp_lateral_alignment.py`：左腕可见目标时按“导热片在夹爪上方则底盘右移、下方则左移”校准；目标在腕部画面外时使用主摄彩色板布局判断搜索方向。只有左腕连续两帧确认对准且记录未过期，`stage1_close_gripper.py` 才允许闭爪。随后垂直上提 `0.12 m`、定位红色垫片、前移 `0.12 m` 并下降 `0.12 m`。

task3 仓库只作为行为模板和底层冷启动服务来源，不由本项目写入。借鉴的契约包括：任务级互斥锁、ROS Domain 隔离、运动阶段原子检查点、严格动作结果验证和异常后的零速退出。

最终释放使用 `stage5_release_diagonal.py`：先下降 `0.008 m`，随后在后退 `0.11 m` 的同时按 ease-out 曲线继续下降——前段下降幅度较大，后段逐步减小，总累计下降仍不超过 `0.07 m`；并从后退开始逐渐把末端朝地面倾转至 `90°`。后退达到 `0.06 m` 时开始张开夹爪，机械臂继续完成剩余后退，随后上抬脱离并回到初始位。默认下降曲线幂次为 `2.0`，可用 `--descent-ease-power` 在 `(1, 4]` 内调整。流程记录写入 `config/latest_full_thermal_pad_cycle.json`。不带 `--execute` 时只输出阶段计划，不发送任何运动命令。
