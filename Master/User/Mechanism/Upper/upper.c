#include "upper.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "my_main.h"
#include <math.h>
#include <string.h>
extern osMutexId_t br_mutex;
extern float br_target[2][4];
extern float br_root_target[2];
extern float br_mit_tau[2][ARM_JOINT_COUNT];
extern rs_mode_t br_joint_mode[2][ARM_JOINT_COUNT];
extern float br_joint_velocity[2][ARM_JOINT_COUNT];
extern uint32_t br_command_tick[2];
extern bool br_command_enable[2], br_grip[2];
typedef struct
{
    FDCAN_HandleTypeDef *bus;
    uint8_t id, dm, stage, state, fault;
    bool seen, reference_valid, stop_pending;
    uint32_t tick, retry;
    float p, v, t, temperature, reference, ff;
    float pmax, vmax, tmax, kp, kd, direction, zero, ratio, ff_limit;
    rs_mode_t command_mode;
    float command_velocity, requested_mit_tau;
    rs_mode_t previous_mode;
    pid_t position_pid, speed_pid; /* POSITION: angle -> speed -> joint torque. */
} direct_motor_t;
static direct_motor_t arm[2][4], rs02;
static br_gravity_model_t models[2] = {{.link = BR_LEFT_LINKS,
                                        .gravity_base = BR_LEFT_GRAVITY_BASE,
                                        .payload_mass_kg = BR_PAYLOAD_MASS_KG,
                                        .payload_com_m = BR_PAYLOAD_COM_M,
                                        .calibrated = BR_LEFT_MODEL_CALIBRATED},
                                       {.link = BR_RIGHT_LINKS,
                                        .gravity_base = BR_RIGHT_GRAVITY_BASE,
                                        .payload_mass_kg = BR_PAYLOAD_MASS_KG,
                                        .payload_com_m = BR_PAYLOAD_COM_M,
                                        .calibrated = BR_RIGHT_MODEL_CALIBRATED}};
m2006_t m2006_larm; /* CAN2 左臂根部，真实独立电机对象。 */
m2006_t m2006_aux[2][4];
m3508_t m3508_aux[1];
#define AUX2006(a, j) (((a) == 0 && (j) == BR_LEFT_ROOT_M2006_INDEX) ? &m2006_larm : &m2006_aux[a][j])
#define aux3508 m3508_aux[0]
/* CAN2 左臂；CAN3 右臂。RS00 下标 0=大臂，1=小臂。 */
rs00_t rs00_larm[2], rs00_rarm[2];
rs05_t rs05_larm, rs05_rarm;    /* 左右臂末端旋转电机。 */
j4310_t j4310_larm, j4310_rarm; /* 左右臂腕部电机。 */
rs02_t rs02_rarm;               /* CAN3 右臂根部。 */
static rs00_t *const rs00_by_arm[2] = {rs00_larm, rs00_rarm};
static rs05_t *const rs05_by_arm[2] = {&rs05_larm, &rs05_rarm};
static j4310_t *const j4310_by_arm[2] = {&j4310_larm, &j4310_rarm};
static void managed_target(void *motor, int mode, float target, float position_deg);

typedef struct
{
    float target;
    uint32_t command_tick, feedback_tick;
    uint8_t mode;
    bool seen, valid;
} aux_command_t;
static aux_command_t cmd2006[2][4], cmd3508;
static uint32_t last_service;
static float left_root_ff_a;
static const unsigned counts[2] = {4, 2};
static float clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}
static uint16_t enc(float x, float max, unsigned bits)
{
    return (uint16_t)((clamp(x, -max, max) + max) * ((1UL << bits) - 1) / (2 * max));
}
static uint16_t gain(float x, float max, unsigned bits)
{
    return (uint16_t)(clamp(x, 0, max) * ((1UL << bits) - 1) / max);
}
static float dec(unsigned x, float max, unsigned bits)
{
    return x * (2 * max) / ((1UL << bits) - 1) - max;
}
static void be16(uint8_t *p, uint16_t n)
{
    p[0] = (uint8_t)(n >> 8);
    p[1] = (uint8_t)n;
}
static bool special(direct_motor_t *m, unsigned code)
{
    can_frame_t f = {0};
    f.hfdcan = m->bus;
    f.length = 8;
    if (m->dm)
    {
        f.id_type = FDCAN_STANDARD_ID;
        f.id = m->id;
        memset(f.data, 255, 8);
        f.data[7] = (uint8_t)code;
    }
    else
    {
        f.id_type = FDCAN_EXTENDED_ID;
        f.id = (code << 24) | (BR_RS_HOST_ID << 8) | m->id;
    }
    return BspCan_Send(&f);
}
static bool mode_mit(direct_motor_t *m)
{
    can_frame_t f = {0};
    f.hfdcan = m->bus;
    f.length = 8;
    f.id_type = FDCAN_EXTENDED_ID;
    f.id = (0x12UL << 24) | (BR_RS_HOST_ID << 8) | m->id;
    f.data[0] = 0x05;
    f.data[1] = 0x70; /* run_mode 0, uint8 at data[4] */
    return BspCan_Send(&f);
}
static bool motion(direct_motor_t *m, float joint_target, float tau)
{
    can_frame_t f = {0};
    uint16_t p, v, k, d, t;
    float target = m->zero + m->direction * joint_target / m->ratio;
    float kp = (m->command_mode == RS_MODE_MIT ? m->kp : 0.0f) * m->ratio * m->ratio;
    float kd =
        (m->command_mode == RS_MODE_MIT || m->command_mode == RS_MODE_DAMPING ? m->kd : 0.0f) * m->ratio * m->ratio;
    float velocity = 0.0f; /* SPEED/POSITION use host loops; MIT target velocity defaults to zero. */
    if (!isfinite(target) || !isfinite(tau) || fabsf(target) > m->pmax)
        return false;
    f.hfdcan = m->bus;
    f.length = 8;
    p = enc(target, m->pmax, 16);
    if (m->dm)
    {
        f.id_type = FDCAN_STANDARD_ID;
        f.id = m->id;
        v = enc(velocity, m->vmax, 12);
        k = gain(kp, BR_DM_KP_MAX, 12);
        d = gain(kd, BR_DM_KD_MAX, 12);
        t = enc(tau, m->tmax, 12);
        be16(f.data, p);
        f.data[2] = (uint8_t)(v >> 4);
        f.data[3] = (uint8_t)((v << 4) | (k >> 8));
        f.data[4] = (uint8_t)k;
        f.data[5] = (uint8_t)(d >> 4);
        f.data[6] = (uint8_t)((d << 4) | (t >> 8));
        f.data[7] = (uint8_t)t;
    }
    else
    {
        f.id_type = FDCAN_EXTENDED_ID;
        f.id = (1UL << 24) | ((uint32_t)enc(tau, m->tmax, 16) << 8) | m->id;
        v = enc(velocity, m->vmax, 16);
        k = gain(kp, BR_RS_KP_MAX, 16);
        d = gain(kd, BR_RS_KD_MAX, 16);
        be16(f.data, p);
        be16(f.data + 2, v);
        be16(f.data + 4, k);
        be16(f.data + 6, d);
    }
    return BspCan_Send(&f);
}
static bool feedback(direct_motor_t *m, const can_frame_t *f, uint32_t now)
{
    unsigned p, v, t;
    if (f->hfdcan != m->bus || f->length != 8)
        return false;
    if (m->dm)
    {
        if (f->id_type != FDCAN_STANDARD_ID || f->id != BR_DM_FEEDBACK_ID || (f->data[0] & 15) != m->id)
            return false;
        m->state = f->data[0] >> 4;
        m->fault = m->state > 1 ? m->state : 0;
        p = ((unsigned)f->data[1] << 8) | f->data[2];
        v = ((unsigned)f->data[3] << 4) | (f->data[4] >> 4);
        t = ((f->data[4] & 15) << 8) | f->data[5];
        m->v = dec(v, m->vmax, 12);
        m->t = dec(t, m->tmax, 12);
        m->temperature = f->data[6];
    }
    else
    {
        /* Status/fault bits are variable; do not compare the entire extended ID. */
        if (f->id_type != FDCAN_EXTENDED_ID || ((f->id >> 24) & 31) != 2 || ((f->id >> 8) & 255) != m->id ||
            (f->id & 255) != BR_RS_HOST_ID)
            return false;
        m->fault = (f->id >> 16) & 63;
        m->state = (f->id >> 22) & 3;
        p = ((unsigned)f->data[0] << 8) | f->data[1];
        v = ((unsigned)f->data[2] << 8) | f->data[3];
        t = ((unsigned)f->data[4] << 8) | f->data[5];
        m->v = dec(v, m->vmax, 16);
        m->t = dec(t, m->tmax, 16);
        m->temperature = (((unsigned)f->data[6] << 8) | f->data[7]) * 0.1f;
    }
    m->p = m->direction * (dec(p, m->pmax, 16) - m->zero) * m->ratio;
    m->v *= m->direction * m->ratio;
    m->t *= m->direction / m->ratio;
    m->seen = true;
    m->tick = now;
    return true;
}
static bool fresh(const direct_motor_t *m, uint32_t now)
{
    return m->seen && now - m->tick <= BR_FEEDBACK_TIMEOUT_MS && !m->fault;
}
static void disable(direct_motor_t *m, uint32_t now)
{
    m->reference_valid = false;
    m->ff = 0;
    PID_Reset(&m->position_pid);
    PID_Reset(&m->speed_pid);
    if (m->stage || m->stop_pending || now - m->retry >= BR_ENABLE_RETRY_MS)
    {
        m->stop_pending = true;
        if (special(m, m->dm ? 0xFD : 4))
        {
            m->stop_pending = false;
            m->stage = 0;
            m->retry = now;
        }
    }
}
static bool prepare(direct_motor_t *m, uint32_t now)
{
    if (m->stop_pending)
    {
        disable(m, now);
        return false;
    }
    if (m->fault && m->seen && now - m->tick <= BR_FEEDBACK_TIMEOUT_MS)
    {
        disable(m, now);
        return false;
    }
    if (m->stage == 3 && !fresh(m, now) && m->reference_valid)
    {
        disable(m, now);
        return false;
    }
    if (m->stage == 0)
    {
        /* Always stop before changing run_mode. */
        if (special(m, m->dm ? 0xFD : 4))
            m->stage = 1;
        return false;
    }
    if (m->stage == 1)
    {
        if (m->dm || mode_mit(m))
            m->stage = 2;
        return false;
    }
    if (m->stage == 2 || !fresh(m, now) || m->state != (m->dm ? 1 : 2))
    {
        if (m->stage == 2 || now - m->retry >= BR_ENABLE_RETRY_MS)
        {
            if (special(m, m->dm ? 0xFC : 3))
            {
                m->stage = 3;
                m->retry = now;
                m->seen = false;
            }
        }
        m->reference_valid = false;
        return false;
    }
    return true;
}
static void init_cascade(direct_motor_t *m)
{
    PID_Init(&m->position_pid, BR_POSITION_PID_KP, BR_POSITION_PID_KI, BR_POSITION_PID_KD, BR_POSITION_SLEW_RAD_S,
             BR_POSITION_PID_I_LIMIT);
    PID_Init(&m->speed_pid, BR_SPEED_PID_KP, BR_SPEED_PID_KI, BR_SPEED_PID_KD, BR_CASCADE_TORQUE_LIMIT_NM,
             BR_SPEED_PID_I_LIMIT);
}
static void control(direct_motor_t *m, float target, float gravity_joint_tau, float dt)
{
    float step = BR_POSITION_SLEW_RAD_S * dt, loop_tau = 0;
    float ff = m->direction * m->ratio * gravity_joint_tau;
    if (m->previous_mode != m->command_mode)
    {
        PID_Reset(&m->position_pid);
        PID_Reset(&m->speed_pid);
        m->reference_valid = false;
        m->previous_mode = m->command_mode;
        m->ff = 0; /* Do not carry explicit MIT torque into another mode. */
    }
    if (!m->reference_valid)
    {
        m->reference = m->p;
        m->reference_valid = true;
    }
    if (m->command_mode == RS_MODE_POSITION || m->command_mode == RS_MODE_MIT)
        m->reference += clamp(target - m->reference, -step, step);
    else
        m->reference = m->p;
    if (m->command_mode == RS_MODE_POSITION)
    {
        float speed_ref = PID_Calc(&m->position_pid, m->reference, m->p, dt);
        loop_tau = PID_Calc(&m->speed_pid, speed_ref, m->v, dt);
    }
    else if (m->command_mode == RS_MODE_SPEED)
    {
        loop_tau = PID_Calc(&m->speed_pid, m->command_velocity, m->v, dt);
    }
    else if (m->command_mode == RS_MODE_MIT)
    {
        /* Explicit tau is already motor output-shaft Nm: no duplicate direction/ratio conversion. */
        ff = m->requested_mit_tau + (BR_MIT_ADD_MODEL_GRAVITY ? ff : 0.0f);
    }
    ff = clamp(ff, -m->ff_limit, m->ff_limit);
    m->ff += clamp(ff - m->ff, -BR_FF_SLEW_NM_S * dt, BR_FF_SLEW_NM_S * dt);
    if (!motion(m, m->reference, clamp(m->ff + m->direction * m->ratio * loop_tau, -m->tmax, m->tmax)))
    {
        m->stage = 0;
        m->reference_valid = false;
        m->ff = 0;
        PID_Reset(&m->position_pid);
        PID_Reset(&m->speed_pid);
    }
}
void Upper_DirectInit(void)
{
    const uint8_t ids[4] = {BR_SHOULDER_ID, BR_ELBOW_ID, BR_WRIST_ID, BR_ROTATION_ID};
    const float kp[4] = {BR_SHOULDER_KP, BR_ELBOW_KP, BR_WRIST_KP, BR_ROTATION_KP};
    const float kd[4] = {BR_SHOULDER_KD, BR_ELBOW_KD, BR_WRIST_KD, BR_ROTATION_KD};
    const float direction[2][4] = {BR_LEFT_DIRECTION, BR_RIGHT_DIRECTION};
    const float zero[2][4] = {BR_LEFT_ZERO_RAD, BR_RIGHT_ZERO_RAD};
    const float ratio[2][4] = {BR_LEFT_JOINT_PER_MOTOR, BR_RIGHT_JOINT_PER_MOTOR};
    const float limits[4] = BR_FF_LIMIT_NM;
    const int dj_dir[2][4] = {BR_CAN2_M2006_DIRECTIONS, BR_CAN3_M2006_DIRECTIONS};
    const uint8_t dj_ids[2][4] = {BR_CAN2_M2006_IDS, BR_CAN3_M2006_IDS};
    FDCAN_HandleTypeDef *bus[2] = {BR_LEFT_CAN, BR_RIGHT_CAN};
    unsigned a, j;
    if (bus[0] == bus[1] || (bus[0] != &hfdcan2 && bus[0] != &hfdcan3) || (bus[1] != &hfdcan2 && bus[1] != &hfdcan3) ||
        BR_LEFT_ROOT_M2006_INDEX >= 4U || BR_LEFT_ROOT_JOINT_PER_OUTPUT <= 0 || BR_RIGHT_ROOT_JOINT_PER_MOTOR <= 0 ||
        BR_LEFT_ROOT_NM_PER_A < 0 || BR_CAN2_M3508_ID < 5U || BR_CAN2_M3508_ID > 8U || BR_WRIST_ID == 0U ||
        BR_WRIST_ID > 15U || BR_SHOULDER_ID == BR_ELBOW_ID || BR_SHOULDER_ID == BR_ROTATION_ID ||
        BR_ELBOW_ID == BR_ROTATION_ID || BR_ROOT_RS02_ID == BR_SHOULDER_ID || BR_ROOT_RS02_ID == BR_ELBOW_ID ||
        BR_ROOT_RS02_ID == BR_ROTATION_ID)
        ERROR_HANDLER(ERROR_CODE_CAN_INIT);
    for (a = 0; a < 2; a++)
        for (j = 0; j < counts[a]; j++)
        {
            unsigned k;
            if (dj_ids[a][j] < 1U || dj_ids[a][j] > 4U)
                ERROR_HANDLER(ERROR_CODE_CAN_INIT);
            for (k = 0; k < j; k++)
                if (dj_ids[a][j] == dj_ids[a][k])
                    ERROR_HANDLER(ERROR_CODE_CAN_INIT);
        }
    memset(m2006_aux, 0, sizeof(m2006_aux));
    memset(arm, 0, sizeof(arm));
    memset(rs00_larm, 0, sizeof(rs00_larm));
    memset(rs00_rarm, 0, sizeof(rs00_rarm));
    memset(&rs05_larm, 0, sizeof(rs05_larm));
    memset(&rs05_rarm, 0, sizeof(rs05_rarm));
    memset(&j4310_larm, 0, sizeof(j4310_larm));
    memset(&j4310_rarm, 0, sizeof(j4310_rarm));
    memset(&rs02_rarm, 0, sizeof(rs02_rarm));
    memset(cmd2006, 0, sizeof(cmd2006));
    memset(&cmd3508, 0, sizeof(cmd3508));
    for (a = 0; a < 2; a++)
    {
        for (j = 0; j < 4; j++)
        {
            direct_motor_t *m = &arm[a][j];
            m->command_mode = RS_MODE_POSITION;
            m->bus = bus[a];
            m->id = ids[j];
            m->dm = j == 2;
            m->kp = kp[j];
            m->kd = kd[j];
            m->direction = direction[a][j];
            m->zero = zero[a][j];
            m->ratio = ratio[a][j];
            m->pmax = j == 2 ? BR_DM_POSITION_MAX : BR_RS_POSITION_MAX;
            m->vmax = j == 2 ? BR_DM_SPEED_MAX : (j == 3 ? BR_RS05_SPEED_MAX : BR_RS00_SPEED_MAX);
            m->tmax = j == 2 ? BR_DM_TORQUE_MAX : (j == 3 ? BR_RS05_TORQUE_MAX : BR_RS00_TORQUE_MAX);
            m->ff_limit = limits[j];
            init_cascade(m);
            if (!isfinite(m->ratio) || m->ratio <= 0 || fabsf(m->direction) != 1 || !isfinite(m->zero) ||
                !isfinite(m->kp) || !isfinite(m->kd) || m->kp < 0 || m->kd < 0 ||
                m->kp * m->ratio * m->ratio > BR_RS_KP_MAX || m->kd * m->ratio * m->ratio > BR_RS_KD_MAX ||
                !isfinite(m->ff_limit) || m->ff_limit < 0 || m->ff_limit > m->tmax)
                ERROR_HANDLER(ERROR_CODE_CAN_INIT);
        }
        for (j = 0; j < counts[a]; j++)
            if (M2006_Init(AUX2006(a, j), a ? &hfdcan3 : &hfdcan2, dj_ids[a][j], dj_dir[a][j]) != HAL_OK)
                ERROR_HANDLER(ERROR_CODE_M2006_INIT);
    }
    if (M3508_Init(&aux3508, &hfdcan2, BR_CAN2_M3508_ID, BR_CAN2_M3508_DIRECTION) != HAL_OK)
        ERROR_HANDLER(ERROR_CODE_M3508_INIT);
    memset(&rs02, 0, sizeof(rs02));
    rs02.bus = &hfdcan3;
    rs02.id = BR_ROOT_RS02_ID;
    rs02.pmax = BR_RS_POSITION_MAX;
    rs02.vmax = BR_RS02_SPEED_MAX;
    rs02.tmax = BR_RS02_TORQUE_MAX;
    rs02.kp = BR_ROOT_RS02_KP;
    rs02.kd = BR_ROOT_RS02_KD;
    rs02.direction = BR_ROOT_RS02_DIRECTION;
    rs02.ratio = BR_RIGHT_ROOT_JOINT_PER_MOTOR;
    rs02.zero = BR_RIGHT_ROOT_ZERO_RAD;
    rs02.ff_limit = BR_ROOT_FF_LIMIT_NM;
    init_cascade(&rs02);
    rs02.command_mode = RS_MODE_POSITION;
    for (a = 0; a < 2; a++)
    {
        for (j = 0; j < 2; j++)
        {
            rs00_by_arm[a][j].target_sink = managed_target;
            rs00_by_arm[a][j].hcan = bus[a];
            rs00_by_arm[a][j].id = ids[j];
        }
        rs05_by_arm[a]->target_sink = managed_target;
        rs05_by_arm[a]->hcan = bus[a];
        rs05_by_arm[a]->id = BR_ROTATION_ID;
        j4310_by_arm[a]->target_sink = managed_target;
        j4310_by_arm[a]->motor_id = BR_WRIST_ID;
        for (j = 0; j < counts[a]; j++)
            AUX2006(a, j)->target_sink = managed_target;
    }
    rs02_rarm.target_sink = managed_target;
    rs02_rarm.hcan = &hfdcan3;
    rs02_rarm.id = BR_ROOT_RS02_ID;
    m2006_aux[0][BR_LEFT_ROOT_M2006_INDEX].target_sink = managed_target; /* 保留槽拒绝直接输出。 */
    aux3508.target_sink = managed_target;
    left_root_ff_a = 0;
    last_service = HAL_GetTick();
}
static bool aux_live(const aux_command_t *c, uint32_t now)
{
    return g_motor_control_enable == 1U && c->valid && c->seen && now - c->command_tick <= BR_COMMAND_TIMEOUT_MS &&
           now - c->feedback_tick <= BR_FEEDBACK_TIMEOUT_MS;
}
/* Public setters never transmit; the sole bus owner computes each current once. */
bool Upper_SetM2006(uint8_t bus, uint8_t index, m2006_mode_t mode, float target)
{
    aux_command_t *c;
    unsigned a;
    if ((bus != 2 && bus != 3) || mode > M2006_MODE_CURRENT || mode < M2006_MODE_STOP || !isfinite(target) || !br_mutex)
        return false;
    a = bus - 2;
    if (index >= counts[a] || (bus == 2 && index == BR_LEFT_ROOT_M2006_INDEX))
        return false;
    osMutexAcquire(br_mutex, osWaitForever);
    c = &cmd2006[a][index];
    c->target = target;
    c->mode = (uint8_t)mode;
    c->command_tick = HAL_GetTick();
    c->valid = true;
    osMutexRelease(br_mutex);
    return true;
}
bool Upper_SetM3508(m3508_mode_t mode, float target)
{
    if (mode > M3508_MODE_CURRENT || mode < M3508_MODE_STOP || !isfinite(target) || !br_mutex)
        return false;
    osMutexAcquire(br_mutex, osWaitForever);
    cmd3508.target = target;
    cmd3508.mode = (uint8_t)mode;
    cmd3508.command_tick = HAL_GetTick();
    cmd3508.valid = true;
    osMutexRelease(br_mutex);
    return true;
}
/* 兼容角度接口：RS02 为 CAN3 右臂根部，共用该臂使能与反馈看门狗。 */
bool Upper_SetRS02Degrees(float degrees, bool enable)
{
    if (!enable)
        return Upper_SetJointModeTarget(1, ARM_JOINT_ROOT, RS_MODE_STOP, 0.0f);
    return Upper_SetJointDegrees(ARM2_BOARD_INDEX, ARM_JOINT_ROOT, degrees);
}
void Upper_DirectFeedback(upper_t *u, const can_frame_t *f)
{
    uint32_t now = HAL_GetTick();
    unsigned a, j;
    for (a = 0; a < 2; a++)
        for (j = 0; j < 4; j++)
            if (feedback(&arm[a][j], f, now))
            {
                unsigned k = 3 - j;
                direct_motor_t *m = &arm[a][j];
                u->board[a].joint_mrad[k] = (int16_t)clamp(m->p * 1000, -32768, 32767);
                u->board[a].speed_centi_rad_s[k] = (int16_t)clamp(m->v * 100, -32768, 32767);
                u->board[a].torque_centi_nm[k] = (int16_t)clamp(m->t * 100, -32768, 32767);
                u->board[a].temperature_c[k] = (int16_t)m->temperature;
                u->board[a].fault[k] = m->fault;
                u->board[a].last_tick = now;
                u->board[a].frame_count++;
                if (j < 2)
                {
                    rs00_t *view = &rs00_by_arm[a][j];
                    view->position = m->p;
                    view->velocity = m->v;
                    view->torque = m->t;
                    view->feedback_valid = true;
                    view->last_feedback_tick = now;
                    view->fault = m->fault;
                }
                else if (j == 3)
                {
                    rs05_by_arm[a]->position = m->p;
                    rs05_by_arm[a]->velocity = m->v;
                    rs05_by_arm[a]->torque = m->t;
                    rs05_by_arm[a]->feedback_valid = true;
                    rs05_by_arm[a]->last_feedback_tick = now;
                    rs05_by_arm[a]->fault = m->fault;
                }
                else
                {
                    j4310_by_arm[a]->feedback.position_rad = m->p;
                    j4310_by_arm[a]->feedback.speed_rad_s = m->v;
                    j4310_by_arm[a]->feedback.torque_nm = m->t;
                    j4310_by_arm[a]->feedback.state = m->state;
                    j4310_by_arm[a]->feedback.received = true;
                }
                return;
            }
    if (feedback(&rs02, f, now))
    {
        rs02_rarm.position = rs02.p;
        rs02_rarm.velocity = rs02.v;
        rs02_rarm.torque = rs02.t;
        rs02_rarm.feedback_valid = true;
        rs02_rarm.last_feedback_tick = now;
        rs02_rarm.fault = rs02.fault;
        u->board[1].last_tick = now;
        u->board[1].frame_count++;
        return;
    }
    if (f->id_type != FDCAN_STANDARD_ID)
        return;
    for (a = 0; a < 2; a++)
        for (j = 0; j < counts[a]; j++)
            if (M2006_OnFrame(AUX2006(a, j), f))
            {
                cmd2006[a][j].seen = true;
                cmd2006[a][j].feedback_tick = now;
                return;
            }
    if (M3508_OnFrame(&aux3508, f))
    {
        cmd3508.seen = true;
        cmd3508.feedback_tick = now;
    }
}
static bool root_fresh(unsigned a, uint32_t now)
{
    const aux_command_t *c = &cmd2006[0][BR_LEFT_ROOT_M2006_INDEX];
    return a ? fresh(&rs02, now) : c->seen && now - c->feedback_tick <= BR_FEEDBACK_TIMEOUT_MS;
}
static float root_position(unsigned a)
{
    return a ? rs02.p : BR_LEFT_ROOT_START_RAD + m2006_larm.position_rad * BR_LEFT_ROOT_JOINT_PER_OUTPUT;
}
static int16_t left_root_current(bool enabled, float tau, float dt)
{
    m2006_t *m = &m2006_larm;
    float ff = 0, raw;
    if (!enabled)
    {
        M2006_SetTargetDeferred(m, M2006_MODE_STOP, 0);
        left_root_ff_a = 0;
        return 0;
    }
    /* Preserve the driver's trajectory and cascaded position/speed PID. */
    if (br_joint_mode[0][ARM_JOINT_ROOT] == RS_MODE_POSITION)
        M2006_SetTargetDeferred(m, M2006_MODE_POSITION,
                                (br_root_target[0] - BR_LEFT_ROOT_START_RAD) / BR_LEFT_ROOT_JOINT_PER_OUTPUT *
                                    ARM_RAD_TO_DEG);
    else if (br_joint_mode[0][ARM_JOINT_ROOT] == RS_MODE_SPEED)
        M2006_SetTargetDeferred(m, M2006_MODE_SPEED,
                                br_joint_velocity[0][ARM_JOINT_ROOT] / BR_LEFT_ROOT_JOINT_PER_OUTPUT);
    else
        M2006_SetTargetDeferred(m, M2006_MODE_DAMPING, 0.0f);
    if (BR_LEFT_ROOT_NM_PER_A > 0.0f)
        ff = tau * BR_LEFT_ROOT_JOINT_PER_OUTPUT / (BR_LEFT_ROOT_NM_PER_A > 0 ? BR_LEFT_ROOT_NM_PER_A : 1.0f);
    ff = clamp(ff, -BR_ROOT_FF_CURRENT_LIMIT_A, BR_ROOT_FF_CURRENT_LIMIT_A);
    left_root_ff_a += clamp(ff - left_root_ff_a, -BR_ROOT_FF_SLEW_A_S * dt, BR_ROOT_FF_SLEW_A_S * dt);
    raw = M2006_Calc(m, dt) + m->direction * left_root_ff_a / M2006_CURRENT_SCALE;
    return (int16_t)clamp(raw, -M2006_CURRENT_RAW_LIMIT, M2006_CURRENT_RAW_LIMIT);
}
void Upper_DirectService(uint32_t now)
{
    unsigned a, j;
    bool arm_ready[2] = {false, false};
    float root_tau[2] = {0, 0};
    float dt = (now - last_service) * 0.001f;
    last_service = now;
    dt = clamp(dt, 0, 0.02f);
    for (a = 0; a < 2; a++)
    {
        bool enabled = g_motor_control_enable == 1U && upper_state.enable && br_command_enable[a] &&
                       now - br_command_tick[a] <= BR_COMMAND_TIMEOUT_MS;
        bool online = root_fresh(a, now), ready = online;
        for (j = 0; j < ARM_JOINT_COUNT; j++)
            if (br_joint_mode[a][j] == RS_MODE_STOP)
                enabled = false;
        float q[BR_ARM_DOF], tau[BR_ARM_DOF] = {0};
        q[0] = root_position(a);
        rs02.requested_mit_tau = br_mit_tau[1][ARM_JOINT_ROOT];
        rs02.command_mode = br_joint_mode[1][ARM_JOINT_ROOT];
        rs02.command_velocity = br_joint_velocity[1][ARM_JOINT_ROOT];
        if (a == 1)
        {
            if (!enabled)
            {
                disable(&rs02, now);
                ready = false;
            }
            else if (!prepare(&rs02, now))
                ready = false;
        }
        for (j = 0; j < 4; j++)
        {
            direct_motor_t *m = &arm[a][j];
            m->requested_mit_tau = br_mit_tau[a][3 - j];
            m->command_mode = br_joint_mode[a][3 - j];
            m->command_velocity = br_joint_velocity[a][3 - j];
            q[j + 1] = m->p;
            if (!fresh(m, now))
                online = false;
            if (!enabled)
            {
                disable(m, now);
                ready = false;
            }
            else if (!prepare(m, now))
                ready = false;
        }
        upper_state.board[a].online = online;
        /* M2006 position is relative to startup. Never treat that as a known CAD zero. */
        upper_state.board[a].gravity_ready =
            online && (a || BR_LEFT_ROOT_POSE_CALIBRATED) && BR_GravityEvaluate(&models[a], q, br_grip[a], tau, NULL);
        if (ready && enabled)
        {
            for (j = 0; j < 4; j++)
                control(&arm[a][j], br_target[a][j], tau[j + 1], dt);
            for (j = 0; j < 4; j++)
                if (!arm[a][j].reference_valid)
                    ready = false;
            if (a && ready)
            {
                control(&rs02, br_root_target[a], tau[0], dt);
                ready = rs02.reference_valid;
            }
        }
        else
            ready = false;
        if (!ready)
        {
            for (j = 0; j < 4; j++)
                if (arm[a][j].reference_valid)
                    disable(&arm[a][j], now);
            if (a && rs02.reference_valid)
                disable(&rs02, now);
        }
        arm_ready[a] = ready;
        root_tau[a] = ready ? tau[0] : 0;
        upper_state.board[a].joint_mrad[4] = (int16_t)clamp(q[0] * 1000, -32768, 32767);
        upper_state.board[a].fault[4] = a ? rs02.fault : 0;
        upper_state.board[a].speed_centi_rad_s[4] =
            (int16_t)clamp(100 * (a ? rs02.v : m2006_larm.speed_rad_s * BR_LEFT_ROOT_JOINT_PER_OUTPUT), -32768, 32767);
        upper_state.board[a].torque_centi_nm[4] =
            (int16_t)clamp(100 * (a ? rs02.t
                                    : m2006_larm.current_raw * M2006_CURRENT_SCALE * BR_LEFT_ROOT_NM_PER_A /
                                          BR_LEFT_ROOT_JOINT_PER_OUTPUT),
                           -32768, 32767);
        for (j = 0; j < 4; j++)
            upper_state.board[a].gravity_tau_nm[3 - j] = arm[a][j].ff;
    }
    for (a = 0; a < 2; a++)
    {
        int16_t current[8] = {0};
        can_frame_t f;
        bool live;
        for (j = 0; j < counts[a]; j++)
        {
            aux_command_t *c = &cmd2006[a][j];
            m2006_t *m = AUX2006(a, j);
            if (!a && j == BR_LEFT_ROOT_M2006_INDEX)
            {
                current[m->id - 1] = left_root_current(arm_ready[0], root_tau[0], dt);
                continue;
            }
            live = aux_live(c, now);
            M2006_SetTargetDeferred(m, live ? (m2006_mode_t)c->mode : M2006_MODE_STOP, live ? c->target : 0);
            current[m->id - 1] = live ? M2006_Calc(m, dt) : 0;
        }
        if (a == 0)
        {
            live = aux_live(&cmd3508, now);
            M3508_SetTargetDeferred(&aux3508, live ? (m3508_mode_t)cmd3508.mode : M3508_MODE_STOP,
                                    live ? cmd3508.target : 0);
            current[aux3508.id - 1] = live ? M3508_Calc(&aux3508, dt) : 0;
        }
        M2006_BuildGroupFrame(1, current, &f);
        f.hfdcan = a ? &hfdcan3 : &hfdcan2;
        (void)BspCan_Send(&f);
        if (!a)
        {
            M3508_BuildGroupFrame(5, current + 4, &f);
            f.hfdcan = &hfdcan2;
            (void)BspCan_Send(&f);
        }
    }
    upper_state.board[0].root_ff_current_a = left_root_ff_a;
    upper_state.board[0].gravity_tau_nm[4] = left_root_ff_a * BR_LEFT_ROOT_NM_PER_A / BR_LEFT_ROOT_JOINT_PER_OUTPUT;
    upper_state.board[1].gravity_tau_nm[4] = rs02.ff;
    for (a = 0; a < 2; a++)
    {
        for (j = 0; j < 2; j++)
        {
            rs00_by_arm[a][j].mode = br_joint_mode[a][3 - j];
            rs00_by_arm[a][j].target = br_target[a][j];
            rs00_by_arm[a][j].tau_ff_nm = arm[a][j].ff;
            rs00_by_arm[a][j].feedback_valid = fresh(&arm[a][j], now);
        }
        rs05_by_arm[a]->mode = br_joint_mode[a][ARM_JOINT_ROTATION];
        rs05_by_arm[a]->target = br_target[a][3];
        rs05_by_arm[a]->tau_ff_nm = arm[a][3].ff;
        rs05_by_arm[a]->feedback_valid = fresh(&arm[a][3], now);
        switch (br_joint_mode[a][ARM_JOINT_WRIST])
        {
        case RS_MODE_MIT:
            j4310_by_arm[a]->mode = J4310_MODE_MIT;
            break;
        case RS_MODE_POSITION:
            j4310_by_arm[a]->mode = J4310_MODE_POSITION;
            break;
        case RS_MODE_SPEED:
            j4310_by_arm[a]->mode = J4310_MODE_SPEED;
            break;
        case RS_MODE_DAMPING:
            j4310_by_arm[a]->mode = J4310_MODE_DAMPING;
            break;
        default:
            j4310_by_arm[a]->mode = J4310_MODE_STOP;
            break;
        }
        j4310_by_arm[a]->target_position_rad = br_target[a][2];
        j4310_by_arm[a]->target_torque_nm = arm[a][2].ff;
        j4310_by_arm[a]->feedback.received = fresh(&arm[a][2], now);
    }
    rs02_rarm.mode = br_joint_mode[1][ARM_JOINT_ROOT];
    rs02_rarm.target = br_root_target[1];
    rs02_rarm.tau_ff_nm = rs02.ff;
    rs02_rarm.feedback_valid = fresh(&rs02, now);
}

void RS02_SetTargetBasic(rs02_t *motor, rs_mode_t mode, float target)
{
    if (mode == RS_MODE_MIT)
        return; /* MIT 必须同时提供 tau 和角度。 */
    if (motor && motor->target_sink)
        motor->target_sink(motor, mode, target, 0.0f);
}
void RS02_SetTargetMIT(rs02_t *motor, rs_mode_t mode, float tau_nm, float degrees)
{
    if (motor && motor->target_sink && mode == RS_MODE_MIT && isfinite(tau_nm) && isfinite(degrees))
        motor->target_sink(motor, mode, tau_nm, degrees);
}
/* RS/DM 模式序号一致；MIT 角度为真实电机输出轴角度。 */
static void route_arm_target(unsigned a, arm_joint_t joint, rs_mode_t mode, float target, float degrees)
{
    if (mode == RS_MODE_MIT)
        (void)Upper_SetJointMIT(a, joint, target, degrees);
    else
        (void)Upper_SetJointModeTarget(a, joint, mode, target);
}
static void managed_target(void *motor, int mode, float target, float position_deg)
{
    unsigned a, j;
    if (!isfinite(target))
        return;
    for (a = 0; a < 2; a++)
    {
        for (j = 0; j < 2; j++)
            if (motor == &rs00_by_arm[a][j])
            {
                route_arm_target(a, j ? ARM_JOINT_ELBOW : ARM_JOINT_SHOULDER, (rs_mode_t)mode, target, position_deg);
                return;
            }
        if (motor == rs05_by_arm[a])
        {
            route_arm_target(a, ARM_JOINT_ROTATION, (rs_mode_t)mode, target, position_deg);
            return;
        }
        if (motor == j4310_by_arm[a])
        {
            rs_mode_t mapped;
            switch ((j4310_mode_t)mode)
            {
            case J4310_MODE_MIT:
                mapped = RS_MODE_MIT;
                break;
            case J4310_MODE_POSITION:
                mapped = RS_MODE_POSITION;
                break;
            case J4310_MODE_SPEED:
                mapped = RS_MODE_SPEED;
                break;
            case J4310_MODE_DAMPING:
                mapped = RS_MODE_DAMPING;
                break;
            case J4310_MODE_STOP:
                mapped = RS_MODE_STOP;
                break;
            default:
                return;
            }
            route_arm_target(a, ARM_JOINT_WRIST, mapped, target, position_deg);
            return;
        }
        for (j = 0; j < counts[a]; j++)
            if (motor == AUX2006(a, j))
            {
                if (!a && j == BR_LEFT_ROOT_M2006_INDEX)
                {
                    rs_mode_t mapped;
                    switch ((m2006_mode_t)mode)
                    {
                    case M2006_MODE_POSITION:
                        mapped = RS_MODE_POSITION;
                        break;
                    case M2006_MODE_SPEED:
                        mapped = RS_MODE_SPEED;
                        break;
                    case M2006_MODE_DAMPING:
                        mapped = RS_MODE_DAMPING;
                        break;
                    case M2006_MODE_STOP:
                        mapped = RS_MODE_STOP;
                        break;
                    default:
                        return; /* Root torque is supplied by the gravity model, not a raw current target. */
                    }
                    (void)Upper_SetJointModeTarget(0, ARM_JOINT_ROOT, mapped, target);
                }
                else
                    (void)Upper_SetM2006(a + 2, j, (m2006_mode_t)mode, target);
                return;
            }
    }
    if (motor == &rs02_rarm)
        route_arm_target(1, ARM_JOINT_ROOT, (rs_mode_t)mode, target, position_deg);
    else if (motor == &aux3508)
        (void)Upper_SetM3508((m3508_mode_t)mode, target);
}
