# BR 主控电机接口与机械臂控制

CAN2 为**左臂 larm**，CAN3 为**右臂 rarm**。左右臂运动链均包含根部、大臂、小臂、腕部、末端旋转。左根部为 M2006，右根部为 RS02；不经过 CANA/CANB 小板。

## 模式定义

支持 MIT 的 RS00、RS05、RS02、J4310 采用以下顺序，POSITION 和 MIT 是不同模式：

```c
typedef enum
{
    J4310_MODE_STOP = 0U, /* 停止输出。 */
    J4310_MODE_DAMPING,   /* 速度阻尼，不锁定角度。 */
    J4310_MODE_SPEED,     /* 速度闭环。 */
    J4310_MODE_POSITION,  /* 位置外环嵌套速度内环。 */
    J4310_MODE_MIT        /* 原生 MIT，独立 tau 与角度。 */
} j4310_mode_t;
```

RS 电机使用对应的 `RS_MODE_*` 名称，模式顺序相同。M2006、M3508 的前四种模式也是 STOP、DAMPING、SPEED、POSITION，额外保留 CURRENT 电流模式；DJI 不支持 MIT，不添加假的 MIT 枚举。

POSITION 的处理路径为 `目标角度 → 位置 PID → 目标速度 → 速度 PID → 力矩/电流`。托管 RS/DM 在主控计算这两个环，并用 KP=KD=0 的运控帧传递力矩；这只是传输格式，应用模式仍是 POSITION。独立 RS00/RS05/J4310 驱动对象的 POSITION 同样实现位置—速度双环。M2006/M3508 保留原有双环、轨迹规划和电流输出。

MIT 不经过上述位置 PID；它发送输出轴角度、宏定义 KP/KD 和显式力矩前馈。SPEED 使用速度闭环；DAMPING 仅提供速度阻尼。托管机械臂某关节 STOP 或反馈失效时会停止对应整臂。

## 电机对象命名与默认总线

| 对象 | 总线 | 作用 / 默认 ID |
|---|---|---|
| `rs00_larm[0]` / `[1]` | CAN2 | 左大臂 / 小臂，RS ID2 / ID3 |
| `rs00_rarm[0]` / `[1]` | CAN3 | 右大臂 / 小臂，RS ID2 / ID3 |
| `j4310_larm` / `j4310_rarm` | CAN2 / CAN3 | 左 / 右腕部，DM ID4 |
| `rs05_larm` / `rs05_rarm` | CAN2 / CAN3 | 左 / 右末端旋转，RS ID5 |
| `m2006_larm` | CAN2 | 左根部，默认 DJI ID1 |
| `rs02_rarm` | CAN3 | 右根部，RS ID1 |
| `m2006_aux[0][i]` | CAN2 | 非根部 M2006 辅助电机 |
| `m2006_aux[1][0..1]` | CAN3 | 两台 M2006，DJI ID1 / ID2 |
| `m3508_aux[0]` | CAN2 | M3508，DJI ID5，0x1FF 第一个电流槽 |

`m2006_larm`、`rs02_rarm` 都是真实独立结构体，可直接取地址。RS02 托管对象共享 RS 状态布局，但组帧与解析使用 RS02 量程。

两臂从根部到末端的电机 ID 固定为 **1、2、3、4、5**。左根部按 CAN2 四台 M2006 中的一台处理，`BR_LEFT_ROOT_M2006_INDEX` 默认为 0。对应 `m2006_aux[0][0]` 是保留槽，不再代表另一台电机，不能重复控制。如果根部实际是第五台 M2006，需要重新规划 DJI ID/分组。其余三台才是辅助机构。各总线内 DJI ID 不能重复。

## SetTarget 调用

包含 `upper.h`。普通模式用三个参数；MIT 用四个参数：**电机对象、MIT、前馈 tau、输出轴角度**。

```c
// POSITION：角度单位度，主控/原 DJI 驱动执行位置—速度双环。
RS00_SetTarget(&rs00_larm[0], RS_MODE_POSITION, -45.0f);
RS00_SetTarget(&rs00_rarm[1], RS_MODE_POSITION, 30.0f);
J4310_SetTarget(&j4310_larm, J4310_MODE_POSITION, 20.0f);
M2006_SetTarget(&m2006_larm, M2006_MODE_POSITION, 15.0f);
M3508_SetTarget(&m3508_aux[0], M3508_MODE_POSITION, 30.0f);

// SPEED 单位 rad/s；DAMPING 的目标填 0。
RS05_SetTarget(&rs05_larm, RS_MODE_SPEED, 0.2f);
RS05_SetTarget(&rs05_rarm, RS_MODE_DAMPING, 0.0f);
J4310_SetTarget(&j4310_rarm, J4310_MODE_STOP, 0.0f);

// MIT：tau 单位 Nm，最后一个参数是电机真实输出轴角度，单位度。
J4310_SetTarget(&j4310_larm, J4310_MODE_MIT, 0.10f, 30.0f);
RS00_SetTarget(&rs00_larm[0], RS_MODE_MIT, 0.20f, -45.0f);
RS05_SetTarget(&rs05_rarm, RS_MODE_MIT, 0.05f, 20.0f);
RS02_SetTarget(&rs02_rarm, RS_MODE_MIT, 0.15f, 30.0f);
```

不要在最后一个参数后添加逗号。C 头文件用参数个数宏选择固定签名函数，不使用运行时 `va_arg`；MIT 少传角度或普通模式误传第四参数会拒绝提交，不越界读取参数。

普通托管 POSITION 使用机械关节角；MIT 使用真实电机输出轴坐标：程序按方向、零位、外部传动比换算到模型关节角，发送时再还原。显式 `tau` 已是电机输出轴 Nm，不再重复乘方向/传动比；会应用配置中的前馈限幅和变化率。KP/KD、MIT 默认速度零值、双环增益与限幅均由宏配置。

托管对象的 `SetTarget` 只提交命令，实际 CAN 帧由唯一发送任务生成。不要再对这些对象额外调用旧 `Service/Send/Stop/SetMitTarget` 接口绕过管理；停止使用上述 SetTarget STOP。

公共使能是 `g_motor_control_enable`。机械臂保持型目标由 `Arm_Task` 周期刷新，辅助 DJI 命令需周期刷新，否则超过 100 ms 归零。初始模式为 POSITION、目标为 0 度；开启总使能前设置所需目标。组目标为 `Arm_larm`、`Arm_rarm`，含 `.Root`；组接口和遥测数组顺序仍为**旋转、腕部、小臂、大臂、根部**。`ARM_LARM_INDEX=0`、`ARM_RARM_INDEX=1`。

## 参数与重力

托管机械臂配置在 `User/Mechanism/Upper/arm_config.h`：

- `BR_LEFT_CAN/BR_RIGHT_CAN`：CAN2 / CAN3。
- `BR_*_KP/KD`：各关节原生 MIT 增益。
- `BR_POSITION_PID_*`、`BR_SPEED_PID_*`：POSITION 外位置环与内速度环，和 MIT 增益分开。
- `BR_*ZERO*`、`BR_*DIRECTION`、`BR_*JOINT_PER*`：坐标与传动比。
- 前馈限幅、变化率、角度/速度限幅、周期等均为宏。
- 独立 RS/DM 驱动对象的默认双环/MIT 宏在各自头文件；DJI 原双环宏在 `m2006.h/m3508.h`。

重力使用五关节实测姿态计算 `tau_hold = ∂U/∂q`，其中 `U=-Σ m_i*g·c_i(q)`。根部姿态参与所有下游质心变换，不把根部当辅助电机。模型包含固定安装旋转、连杆质量/质心、关节轴、可选已知末端负载。

默认 `BR_MIT_ADD_MODEL_GRAVITY=0`：MIT 只使用调用者显式传入的 tau，防止调用者传入重力 tau 后再重复叠加。若希望内部自动叠加已标定模型，改为 1，此时调用者的 tau 是额外力矩；传 0 可只用模型重力。POSITION/SPEED/DAMPING 的模型前馈与对应闭环分别计算后合成。STOP 清零。

左根部 M2006 仍用原双环。只有起始绝对角、模型及有效输出轴 Nm/A 已标定时才增加重力电流；没有用新的电流 PD 替换原双环。其软件上电零点不自动等于 CAD 零点，相关宏为 `BR_LEFT_ROOT_START_RAD`、`BR_LEFT_ROOT_POSE_CALIBRATED`、`BR_LEFT_ROOT_NM_PER_A`。

**实际 BR 质量/质心标定仍未完成**：原始 BR 目录有 420 个 Creo 文件，指定 BR 转换目录为空。本机 J-Link 只读探测尚未获得有效质量属性导出，没有使用 TR 模型或合成测试数据替代 BR 参数。`BR_LEFT_MODEL_CALIBRATED/BR_RIGHT_MODEL_CALIBRATED` 默认为 0，模型前馈为 0；用户显式 MIT tau 仍可按接口传入。

模型宏顺序为根部、大臂、小臂、腕部、末端旋转。每个运动连杆需要 kg/m/rad 单位的 `{origin[3], fixed_rotation[3][3], axis[3], mass_kg, com_m[3]}`，固定零件只能计入所属连杆一次；左右安装方向需分别核对。未接入动态机身倾斜反馈。必须取得并验证真实参数后再启用 calibrated 标志。

## 验证

运行 `tests/run.ps1`。测试覆盖五种模式枚举顺序、三/四参数接口分派、普通 POSITION 真正嵌套两个 PID、MIT 不进入外部双环、显式 tau 不重复变换/叠加、左右总线、独立驱动行为、DJI 原双环、根部掉线停臂、超时和发送失败重试。重力另有 200 姿态 × 5 关节共 1000 项势能中心差分检查；质量/几何为测试数据。

Keil 工程为 `MDK-ARM/H723VGT6.uvprojx`，使用 ARM Compiler 6.24。未烧录、未进行实机调参。CANA_Board/CANB_Board 未修改。
