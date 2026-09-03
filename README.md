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

程序优先使用左腕画面闭环居中；初始姿态的现场标定为腕部图像上/下分别对应机器人左/右，因此目标在画面上方时左移、在下方时右移。腕部未见目标时用主摄水平位置提供搜索方向。每步横移默认仅 `0.02 m`，双雷达任一路无新鲜数据时禁止运动并以零运动退出。

现场明确允许在双雷达离线时仅使用 50 Hz 里程计低速短步闭环，可显式运行：

```bash
/usr/bin/python3 align_to_thermal_pad.py --execute --allow-odom-only
```

底盘运动通过 SSH 在 `tmr-user@172.16.0.50` 本机执行，固定使用与底盘控制器一致的 ROS Domain 0；机械臂不接收运动命令。

## 导热垫末端抓取 FK/IK

按固定顺序执行“左臂初始位复位 → 底盘视觉居中 → 连续静止确认 → D405 原始深度配准 → 手眼坐标变换 → FK/IK → MoveIt 碰撞检查”：

```bash
cd /home/aup/tmr_franka_task2_grasp && source /home/aup/tmr_env.sh && /usr/bin/python3 run_thermal_pad_pipeline.py --execute
```

双雷达确实离线且现场明确允许仅依赖里程计时，必须额外显式添加 `--allow-odom-only`。流程的最后阶段仅解算和校验，不闭合夹爪、不执行抓取轨迹；结果写入 `config/latest_thermal_pad_ik.json`，标注图写入 `outputs/thermal_pad_ik.jpg`。

`thermal_pad_ik.py` 会同时要求：导热垫中心落在左腕图像 Y 方向 ±35 px、底盘速度连续 1 秒低于阈值、左臂处于记录的初始关节位、RGB/深度时间差不超过 0.1 秒、7 帧深度中至少 5 帧在三维空间一致、手眼标定与 FK 一致、所有 IK 点连续且无碰撞。任一条件不满足即以零动作退出。靠近机器人且向下搭的一端按当前初始姿态标定为导热垫长轴的图像 `+X` 端，参数集中在 `config/thermal_pad_pick.json`。

入口在复位前会以 `--state-only` 运行 `bootstrap_left_runtime.py`：确认左臂硬件、错误恢复和两路状态广播可用，然后由原生低速 PTP 动作复位。正式入口不会启用阻抗控制器，避免 FCI 重连后控制器读到空目标或过期目标。

FK/IK 请求会读取 Spine 实测高度（当前为 `0.600 m`），并显式组合“整机基座 → 左臂安装座 → FCI 实测末端”的坐标链；禁止把 FCI 的左臂局部位姿直接与 MoveIt 整机坐标比较。实测末端相对 `link8` 的法兰偏移保存在 `config/thermal_pad_pick.json`。

### 导热垫抓取、提起与脱离动作设计

FK/IK 规划器还会生成完整但默认禁止实机执行的动作序列：在检测到的夹取末端高度保持腕部法兰水平、夹爪竖直；夹爪张开后沿机器人基座 `+X` 前进到目标并闭合；沿 `+Z` 上提 `0.22 m`，沿 `+X` 向远端移动 `0.12 m`，再下降 `0.02 m`；随后沿 `-Z/-X` 斜向下降和内缩，松开夹爪，最后一边向前倾斜一边继续沿 `-X` 内缩，使导热垫脱离。最终倾斜段的位置和姿态使用同步插值，每个中间点均请求 IK 并检查状态有效性。

坐标和参数位于 `config/thermal_pad_pick.json` 的 `motion_sequence`。所有方向和高度先在与地面平行的整机 `base` 参考系中表达；FCI 返回的肩部局部末端位姿必须通过“整机根 → Spine → 左臂安装座”变换后才能使用，严禁直接在 `left_fr3v2_link0` 肩部坐标中加减这些位移。这里的“与此前夹取末端同高”使用变换后的地面参考 Z；`0.22/0.12/0.02 m` 是地面轴上的相对位移，因此不依赖地面参考系原点具体落在哪里。

用户明确指定的 `0.22/0.12/0.02 m` 已固定；开放前进距离、斜向下降/内缩距离以及最终倾角尚无现场尺寸依据，目前仅为保守设计初值，并标记 `parameters_calibrated: false`。在完成 TCP、桌面 PlanningScene、夹持视觉验证和小步现场标定前，任何执行器都必须拒绝运行该放置/脱离序列。

完整动作按第 6 步结束位置拆成两个独立段。第一段 `pick_lift_and_far_transfer` 完成夹取、上提 `0.22 m` 和向远端移动 `0.12 m`，在 `carry_far_12cm` 保持姿态并退出；第二段 `lower_place_and_release` 只有在重新确认第一段终点、物体仍被可靠夹持并获得单独授权后，才从下降 `0.02 m` 开始。禁止第一段完成后自动串行进入第二段。
