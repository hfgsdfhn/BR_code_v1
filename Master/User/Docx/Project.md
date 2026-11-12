# BR 主控电机接口与机械臂控制

CAN2 ??*左臂 larm**，CAN3 ??*右臂 rarm**。左右臂运动链均包含根部、大臂、小臂、腕部、末端旋转。左根部??M2006，右根部??RS01；不经过 CANA/CANB 小板??

## 模式定义

支持 MIT ??RS00、RS05、RS01、J4310 采用以下顺序，POSITION ??MIT 是不同模式：

```c
typedef enum
{
    J4310_MODE_STOP = 0U,
    J4310_MODE_DAMPING,
    J4310_MODE_SPEED,
    J4310_MODE_POSITION,
    J4310_MODE_MIT
} j4310_mode_t;
```

RS 电机使用对应??`RS_MODE_*` 名称，模式顺序相同。M2006、M3508 的前四种模式也是 STOP、DAMPING、SPEED、POSITION，额外保??CURRENT 电流模式；DJI 不支??MIT，不添加假的 MIT 枚举??

POSITION 的处理路径为 `目标角度 ??位置 PID ??目标速度 ??速度 PID ??力矩/电流`。托??RS/DM 在主控计算这两个环，并用 KP=KD=0 的运控帧传递力矩；这只是传输格式，应用模式仍是 POSITION。独??RS00/RS05/J4310 驱动对象??POSITION 同样实现位置—速度双环。M2006/M3508 保留原有双环、轨迹规划和电流输出??

MIT 不经过上述位??PID；它发送输出轴角度、宏定义 KP/KD 和显式力矩前馈。SPEED 使用速度闭环；DAMPING 仅提供速度阻尼。托管机械臂某关??STOP 或反馈失效时会停止对应整臂??

## 电机对象命名与默认总线

| 对象 | 总线 | 作用 / 默认 ID |
|---|---|---|
| `rs00_larm[0]` / `[1]` | CAN2 | 左大??/ 小臂，RS ID2 / ID3 |
| `rs00_rarm[0]` / `[1]` | CAN3 | 右大??/ 小臂，RS ID2 / ID3 |
| `j4310_larm` / `j4310_rarm` | CAN2 / CAN3 | ??/ 右腕部，DM ID4 |
| `rs05_larm` / `rs05_rarm` | CAN2 / CAN3 | ??/ 右末端旋转，RS ID5 |
| `m2006_larm` | CAN2 | 左根部，默认 DJI ID1 |
| `rs01_rarm` | CAN3 | 右根部，RS ID1 |
| `m2006_ucup[i-1]` | CAN2 | 非根??M2006 辅助电机 |
| `m2006_dcup[0..1]` | CAN3 | 两台 M2006，DJI ID1 / ID2 |
| `m3508_dcup[0]` | CAN2 | M3508，DJI ID5??x1FF 第一个电流槽 |

`m2006_larm`、`rs01_rarm` 都是真实独立结构体，可直接取地址。RS01 托管对象共享 RS 状态布局，但组帧与解析使??RS01 量程??

两臂从根部到末端的电??ID 固定??**1????????**。左根部??CAN2 四台 M2006 中的一台处理，`BR_LEFT_ROOT_M2006_INDEX` 默认??0。对??`m2006_larm` 是保留槽，不再代表另一台电机，不能重复控制。如果根部实际是第五??M2006，需要重新规??DJI ID/分组。其余三台才是辅助机构。各总线??DJI ID 不能重复??

## SetTarget 调用

包含 `upper.h`。普通模式用三个参数；MIT 用四个参数：**电机对象、MIT、前??tau、输出轴角度**??

```c

RS00_SetTarget(&rs00_larm[0], RS_MODE_POSITION, -45.0f);
RS00_SetTarget(&rs00_rarm[1], RS_MODE_POSITION, 30.0f);
J4310_SetTarget(&j4310_larm, J4310_MODE_POSITION, 20.0f);
M2006_SetTarget(&m2006_larm, M2006_MODE_POSITION, 15.0f);
M3508_SetTarget(&m3508_dcup[0], M3508_MODE_POSITION, 30.0f);


RS05_SetTarget(&rs05_larm, RS_MODE_SPEED, 0.2f);
RS05_SetTarget(&rs05_rarm, RS_MODE_DAMPING, 0.0f);
J4310_SetTarget(&j4310_rarm, J4310_MODE_STOP, 0.0f);


J4310_SetTarget(&j4310_larm, J4310_MODE_MIT, 0.10f, 30.0f);
RS00_SetTarget(&rs00_larm[0], RS_MODE_MIT, 0.20f, -45.0f);
RS05_SetTarget(&rs05_rarm, RS_MODE_MIT, 0.05f, 20.0f);
RS01_SetTarget(&rs01_rarm, RS_MODE_MIT, 0.15f, 30.0f);
```

不要在最后一个参数后添加逗号。C 头文件用参数个数宏选择固定签名函数，不使用运行??`va_arg`；MIT 少传角度或普通模式误传第四参数会拒绝提交，不越界读取参数??

普通托??POSITION 使用机械关节角；MIT 使用真实电机输出轴坐标：程序按方向、零位、外部传动比换算到模型关节角，发送时再还原。显??`tau` 已是电机输出??Nm，不再重复乘方向/传动比；会应用配置中的前馈限幅和变化率。KP/KD、MIT 默认速度零值、双环增益与限幅均由宏配置??

托管对象??`SetTarget` 只提交命令，实际 CAN 帧由唯一发送任务生成。不要再对这些对象额外调用旧 `Service/Send/Stop/SetMitTarget` 接口绕过管理；停止使用上??SetTarget STOP??

机械臂使能使用 `upper_state.enable`。机械臂目标统一使用角度（度）；辅助 DJI 命令需周期刷新，否则超时 100 ms 归零。初始模式为 POSITION、目标为 0 度；使能前设置所需目标。组目标??`Arm_larm`、`Arm_rarm`，含 `.Root`；组接口和遥测数组顺序仍??*旋转、腕部、小臂、大臂、根??*。`ARM_LARM_INDEX=0`、`ARM_RARM_INDEX=1`??

## 参数与重??

托管机械臂配置在 `User/Mechanism/Upper/arm_config.h`??

- `BR_LEFT_CAN/BR_RIGHT_CAN`：CAN2 / CAN3??
- `BR_*_KP/KD`：各关节原生 MIT 增益??
- `BR_POSITION_PID_*`、`BR_SPEED_PID_*`：POSITION 外位置环与内速度环，??MIT 增益分开??
- `BR_*ZERO*`、`BR_*DIRECTION`、`BR_*JOINT_PER*`：坐标与传动比??
- 前馈限幅、变化率、角??速度限幅、周期等均为宏??
- 独立 RS/DM 驱动对象的默认双??MIT 宏在各自头文件；DJI 原双环宏??`m2006.h/m3508.h`??

重力使用五关节实测姿态计??`tau_hold = ?U/?q`，其??`U=-Σ m_i*g·c_i(q)`。根部姿态参与所有下游质心变换，不把根部当辅助电机。模型包含固定安装旋转、连杆质??质心、关节轴、可选已知末端负载??

默认 `BR_MIT_ADD_MODEL_GRAVITY=0`：MIT 只使用调用者显式传入的 tau，防止调用者传入重??tau 后再重复叠加。若希望内部自动叠加已标定模型，改为 1，此时调用者的 tau 是额外力矩；??0 可只用模型重力。POSITION/SPEED/DAMPING 的模型前馈与对应闭环分别计算后合成。STOP 清零??

左根??M2006 仍用原双环。只有起始绝对角、模型及有效输出??Nm/A 已标定时才增加重力电流；没有用新的电??PD 替换原双环。其软件上电零点不自动等??CAD 零点，相关宏??`BR_LEFT_ROOT_START_RAD`、`BR_LEFT_ROOT_POSE_CALIBRATED`、`BR_LEFT_ROOT_NM_PER_A`??

**实际 BR 质量/质心标定仍未完成**：原??BR 目录??420 ??Creo 文件，指??BR 转换目录为空。本??J-Link 只读探测尚未获得有效质量属性导出，没有使用 TR 模型或合成测试数据替??BR 参数。`BR_LEFT_MODEL_CALIBRATED/BR_RIGHT_MODEL_CALIBRATED` 默认??0，模型前馈为 0；用户显??MIT tau 仍可按接口传入??

模型宏顺序为根部、大臂、小臂、腕部、末端旋转。每个运动连杆需??kg/m/rad 单位??`{origin[3], fixed_rotation[3][3], axis[3], mass_kg, com_m[3]}`，固定零件只能计入所属连杆一次；左右安装方向需分别核对。未接入动态机身倾斜反馈。必须取得并验证真实参数后再启用 calibrated 标志??

## 验证

运行 `tests/run.ps1`。测试覆盖五种模式枚举顺序、三/四参数接口分派、普??POSITION 真正嵌套两个 PID、MIT 不进入外部双环、显??tau 不重复变??叠加、左右总线、独立驱动行为、DJI 原双环、根部掉线停臂、超时和发送失败重试。重力另??200 姿??× 5 关节??1000 项势能中心差分检查；质量/几何为测试数据??

Keil 工程??`MDK-ARM/H723VGT6.uvprojx`，使??ARM Compiler 6.24。未烧录、未进行实机调参。CANA_Board/CANB_Board 未修改??
