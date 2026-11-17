/*
 * @file upper.c
 * @brief upper濠电姷鏁告慨鐑姐€傞挊澹╋綁宕ㄩ弶鎴濈€銈呯箰閻楀棝鎮為崹顐犱簻闁圭儤鍨甸弳鐐烘煟濠垫劒閭柡灞剧〒閳ь剨绲婚崝宀勫礉閿曞倹鐓涚€光偓鐎ｎ剛袦婵犵鍓濋幃鍌涗繆閻戣棄唯妞ゆ棁宕电壕濠氭⒒閸屾瑦绁版俊妞煎姂濮婅棄顓兼径濠勶紮濠电娀娼ч鍥х暤娓氣偓閹鏁愭惔鈩冪亶闂佺粯鎸堕崕鐢稿箖濡ゅ懏鏅查幖瀛樼箘閻╁骸顪冮妶蹇涙闁绘挴鈧剚娼栨繛宸簻瀹告繂鈹戦悩鎻掝仾闁搞倕鍊荤槐鎾存媴閽樺鏁块梺绋款儐閸旀宕氶幒妤婃晣闁绘劕鐏氬▓鏇㈡⒑闁偛鑻晶鎾煕閳规儳浜炬俊鐐€栧濠氬磻閹惧绡€闁逞屽墴閺屽棗顓奸崨顖ょ幢闂備胶绮Λ渚€宕戦幇顒傛殼濞撴埃鍋撻柡灞剧洴婵＄兘顢欓悡搴交闁?
 */

#include "upper.h"
#include "cup_config.h"
#include "cup_task.h"
#include "arm_task.h"
#include "my_main.h"
#include <math.h>
#include <string.h>

extern volatile float arm_target[2][ARM_JOINT_COUNT];
extern volatile float br_mit_tau[2][ARM_JOINT_COUNT];
extern float br_joint_velocity[2][ARM_JOINT_COUNT];
extern bool br_grip[2];
extern volatile uint8_t arm_joint_zero_calibration[2][ARM_JOINT_COUNT];
static uint8_t zero_calibration_previous[2][ARM_JOINT_COUNT];

__ALIGNED(32) volatile float arm_feedback_angle[2][ARM_JOINT_COUNT];
volatile float arm_end_position_world[2][3];
upper_t upper_state;
Arm_Pos Arm_larm, Arm_rarm;
static uint32_t last_service_tick;
static volatile bool arm_overtemp_latched[2];

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垽姊虹粙鎸庢拱濞撴碍顨婂畷顖炲醇閵夛腹鎷绘繛杈剧到閹诧繝骞嗛崼銉︾厽婵°倐鍋撶紒缁樏悾鐑藉即閵忕姈褍顭块懜鐢点€掔紒瀣箰椤啴濡堕崱妤€衼闂傚倸瀚€氭澘鐣烽崹顐ょ瘈闁搞儯鍔庨崢閬嶆煟韫囨洖浠︾€规洘蓱缁旂喎螣閸忕厧鏋戦梺瑙勵問閸犳牠鍩㈤崼銉︾厸閻忕偠顕ч埀顒佺箓閻ｇ兘顢曢敃鈧粈瀣亜閹邦喖鏋庡ù婊堢畺閹綊宕堕宥嗏挅闂佺粯鍨堕敋妞ゆ洟浜堕弻鈩冨緞鐎ｎ亞浠撮梺鍝ュ亼閸旀垵顫忓ú顏勭闁绘劖褰冩俊褔姊洪崨濠傚闁哄懏绮撻、鏃堟偄閸忓皷鎷绘繛杈剧导鐠€锕傛倿閸撗呯＜閻犲洩灏欐晶锔锯偓瑙勬礃濞茬喖寮婚崱妤婂悑閹兼番鍨哄▍鏃€淇婇悙顏勨偓鏍涙担瑙勫弿闁靛牆顦壕鍧楁煟閺傚灝鎮戦柣鎾寸洴閺屾盯骞囬埡浣割瀳闂侀€炲苯澧紓宥咃躬楠炲啯銈ｉ崘鈺冨姸閻庡箍鍎卞Λ娑㈠储閻㈢數纾藉ù锝堫嚃濞堟洜绱掔€ｎ偄鐏ラ摶鐐淬亜閹惧崬鐏╃紓浣叉櫅闇夐柣妯烘▕閸庢劙鏌ｉ幘瀵告创闁哄本鐩、鏇㈡偐閼碱兛鎮ｉ梻浣告啞閺屻劑顢栨径鎰摕闁挎繂妫欓崕鐔搞亜閺冨洤浜瑰ù鐘虫倐濮?*/
bool apply_arm(uint8_t arm, uint8_t joint_index, float target_degrees)
{
    if (joint_index < 1U || joint_index > ARM_JOINT_COUNT)
    {
        return false;
    }
/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝庝簽閻も偓濠电偠鎻徊浠嬪箟閳ユ枼鏋嶆俊銈呭暟绾句粙鏌涚仦鎹愬闁逞屽墯閹倸鐣烽娑橆嚤闁哄鍨归崫妤冪磽娴ｅ憡婀版い鎺斿皟r_SetJointDegrees闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姎缂佺姾娅ｇ槐鎾存媴閻熸澘顫嶅銈冨妼閿曨亜顕ｆ繝姘伋闁归绀侀幃鎴︽⒑閸涘﹣绶遍柛鐘愁殜楠炲啫鈻庨幘绮规嫼闂備緡鍋嗛崑娑㈡嚐椤栨稒娅犳い鏇楀亾闁哄本绋戦～婵嬵敆婢跺﹤澹庨柣搴ゎ潐濞叉牜绱炴繝鍐х箚闁归棿绀佸敮濡炪倖鏌ㄩ崥瀣枍婵犲洦鈷掑〒姘ｅ亾婵炰匠鍥佸洭顢橀姀鐘碉紮濠德板€曢崯浼存儗濞嗗繆鏀介柣妯诲絻閺嗘洜绱掔拠鍙夘棡闁靛洤瀚板浠嬪Ω閿濆嫮鎹曢梻浣侯焾椤戝棝骞愰幖浣瑰仼闁绘垼妫勯悙濠囨煃閸濆嫸鏀婚柡浣靛€濆?*/
    return Upper_SetJointDegrees(arm,
                                 (arm_joint_t)(joint_index - 1U),
                                 target_degrees);
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝庝簽閻も偓濠电偠鎻徊浠嬪箟閳ユ枼鏋嶆俊銈呭暟绾句粙鏌涚仦鎹愬闁逞屽墯閹倸鐣烽弶娆炬僵閻犻缚娅ｉ悰銉╂⒑閸濆嫮鈻夐柛妯垮亹缁顫濋鐘殿啎闂佺懓鐡ㄩ悷銉╂倶椤忓牊鐓曢幖绮光偓鎰佸妷婵烇絽娲ら敃顏堛€佸☉姗嗘僵妞ゆ帊绀侀埀顒夊灣缁辨挻鎷呴悷鏉款潔濡炪們鍔岄敃顏勵嚕婵犳艾鐏抽柟棰佺閹垿姊洪崨濠佺繁闁哥姵宀搁幃锟犲箣閻樼數锛濋梺绋挎湰濮樸劌鐡梻浣侯攰濞呮洟宕濆Δ鈧嵄闁圭増婢樼猾宥夋煕椤愵偄浜濋柡鍛櫊濮婃椽鎳濋柇锔界秷濠电姰鍨洪敃銏ゅ箖閻㈠壊鏁嶉柣鎰綑閳ь剙鐏氱换娑㈠醇濠靛牅铏庨梺鍝勵儐閹瑰洭寮诲☉姘ｅ亾閿濆骸浜濈€规洖鐭傞弻锛勪沪閸撗€妲堝銈庡亝缁诲牓骞冮埄鍐╁劅闁挎繂瀚幗瀣⒒閸屾瑦绁版い鏇熺墵瀹曞綊鏌嗗鍡楀壆濡炪倖鐗滈崑娑氱不濮樿埖鐓曢柡鍥ュ妼閳ь剛濞€閹垽鎮滃Ο铏瑰酱闂傚倸顭崑鎺楀储閸撗勬殰婵炴垯鍨洪埛鎺懨归敐鍛暈闁哥喓鍋ら幃浠嬵敍濠婂啯鐏佺紓浣介哺閻燂妇鈧絻鍋愰埀顒佺⊕钃辨繛?*/
bool apply_cup(uint8_t cup, uint8_t index, float target_degrees)
{
    if (cup >= 2U || index >= 3U)
    {
        return false;
    }
    suction_target[cup][index] = target_degrees;
    return true;
}

volatile float br_shoulder_kp = 10.0f;
volatile float br_shoulder_kd = 1.0f;
volatile float br_elbow_kp = 10.0f;
volatile float br_elbow_kd = 1.0f;
volatile float br_wrist_kp = 10.0f;
volatile float br_wrist_kd = 1.0f;
volatile float br_rotation_kp = 10.0f;
volatile float br_rotation_kd = 1.0f;
volatile float br_root_rs01_kp = 10.0f;
volatile float br_root_rs01_kd = 1.0f;
volatile float br_root_m2006_position_kp = 94.247779608f;
volatile float br_root_m2006_position_ki = 52.359877560f;
volatile float br_root_m2006_position_kd = 0.523598776f;
volatile float br_root_m2006_speed_kp = 3.342253805f;
volatile float br_root_m2006_speed_ki = 2.387324146f;
volatile float br_root_m2006_speed_kd = 0.0f;
volatile uint8_t g_rs_recovery_motion_release = 0U;

volatile uint8_t br_manual_tau_enable = 0U;
volatile float br_shoulder_tau_nm = 0.0f;
volatile float br_elbow_tau_nm = 0.0f;
volatile float br_wrist_tau_nm = 0.0f;
volatile float br_rotation_tau_nm = 0.0f;



/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娆撴煠閹帒鍔ょ€规洜顨廵r_ZeroCalibrationRequest闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
static void Upper_ZeroCalibrationRequest(unsigned arm, unsigned joint)
{
    uint8_t active = arm_joint_zero_calibration[arm][joint] != 0U;
    if (active && !zero_calibration_previous[arm][joint])
    {
        if (joint == ARM_JOINT_ROOT)
        {
            if (arm == ARM_RARM_INDEX) rs01_rarm.set_zero_position_request = 1U;
        }
        else if (joint == ARM_JOINT_SHOULDER || joint == ARM_JOINT_ELBOW)
            (arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm)[joint == ARM_JOINT_SHOULDER ? 0 : 1].set_zero_position_request = 1U;
        else if (joint == ARM_JOINT_WRIST)
            (arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm)->set_zero_position_request = 1U;
        else if (joint == ARM_JOINT_ROTATION)
            (arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm)->set_zero_position_request = 1U;
    }
    zero_calibration_previous[arm][joint] = active;
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娆撴煠閹帒鍔ょ€规洜顨廵r_ZeroCalibrationComplete闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
static void Upper_ZeroCalibrationComplete(unsigned arm, unsigned joint)
{
    bool at_zero = false;
    bool pending = false;
    if (joint == ARM_JOINT_ROOT)
    {
        if (arm == ARM_RARM_INDEX) { pending = rs01_rarm.set_zero_position_request != 0U; at_zero = rs01_rarm.feedback_valid && fabsf(rs01_rarm.single_position) <= RS01_ZERO_POSITION_TOLERANCE_RAD; }
    }
    else if (joint == ARM_JOINT_SHOULDER || joint == ARM_JOINT_ELBOW)
    {
        rs00_t *m = (arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm) + (joint == ARM_JOINT_SHOULDER ? 0 : 1);
        pending = m->set_zero_position_request != 0U; at_zero = m->feedback_valid && fabsf(m->single_position) <= RS00_ZERO_POSITION_TOLERANCE_RAD;
    }
    else if (joint == ARM_JOINT_WRIST)
    {
        j4310_t *m = arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm;
        pending = m->set_zero_position_request != 0U; at_zero = m->feedback.received && fabsf(m->feedback.position_rad) <= J4310_ZERO_POSITION_TOLERANCE_RAD;
    }
    else if (joint == ARM_JOINT_ROTATION)
    {
        rs05_t *m = arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm;
        pending = m->set_zero_position_request != 0U; at_zero = m->feedback_valid && fabsf(m->single_position) <= RS05_ZERO_POSITION_TOLERANCE_RAD;
    }
    if (arm_joint_zero_calibration[arm][joint] != 0U && !pending && at_zero)
    {
        arm_joint_zero_calibration[arm][joint] = 0U;
        zero_calibration_previous[arm][joint] = 0U;
    }
}
/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娆撴煟閿濆懐鐒炬繛鍏肩煒mpf闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娑樏归敐鍛暈闁轰焦螕emetry闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
static void telemetry(upper_board_t *board, unsigned joint, float p, float v, float t, float temp, unsigned fault,
                      uint32_t now)
{
    board->joint_mrad[joint] = (int16_t)clampf(p * 1000.0f, -32768.0f, 32767.0f);
    board->speed_centi_rad_s[joint] = (int16_t)clampf(v * 100.0f, -32768.0f, 32767.0f);
    board->torque_centi_nm[joint] = (int16_t)clampf(t * 100.0f, -32768.0f, 32767.0f);
    board->temperature_c[joint] = (int16_t)clampf(temp, -32768.0f, 32767.0f);
    board->fault[joint] = (uint16_t)fault;
    board->last_tick = now;
    board->frame_count++;
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娆撴煠閹帒鍔ょ€规洜顨廵r_ConfigurePidGains闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
static void Upper_ConfigurePidGains(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娆撴煠閹帒鍔ょ€规洜顨廵r_ConfigureM2006Gains闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
static void Upper_ConfigureM2006Gains(void)
{
    Upper_ConfigurePidGains(&m2006_dcup[0].position_pid, BR_M2006_CUP0_POSITION_KP, BR_M2006_CUP0_POSITION_KI, BR_M2006_CUP0_POSITION_KD);
    Upper_ConfigurePidGains(&m2006_dcup[0].speed_pid, BR_M2006_CUP0_SPEED_KP, BR_M2006_CUP0_SPEED_KI, BR_M2006_CUP0_SPEED_KD);
    Upper_ConfigurePidGains(&m2006_ucup[0].position_pid, BR_M2006_CUP0_POSITION_KP, BR_M2006_CUP0_POSITION_KI, BR_M2006_CUP0_POSITION_KD);
    Upper_ConfigurePidGains(&m2006_ucup[0].speed_pid, BR_M2006_CUP0_SPEED_KP, BR_M2006_CUP0_SPEED_KI, BR_M2006_CUP0_SPEED_KD);
    Upper_ConfigurePidGains(&m2006_dcup[1].position_pid, BR_M2006_CUP1_POSITION_KP, BR_M2006_CUP1_POSITION_KI, BR_M2006_CUP1_POSITION_KD);
    Upper_ConfigurePidGains(&m2006_dcup[1].speed_pid, BR_M2006_CUP1_SPEED_KP, BR_M2006_CUP1_SPEED_KI, BR_M2006_CUP1_SPEED_KD);
    Upper_ConfigurePidGains(&m2006_ucup[1].position_pid, BR_M2006_CUP1_POSITION_KP, BR_M2006_CUP1_POSITION_KI, BR_M2006_CUP1_POSITION_KD);
    Upper_ConfigurePidGains(&m2006_ucup[1].speed_pid, BR_M2006_CUP1_SPEED_KP, BR_M2006_CUP1_SPEED_KI, BR_M2006_CUP1_SPEED_KD);
    Upper_ConfigurePidGains(&m2006_larm.position_pid, br_root_m2006_position_kp, br_root_m2006_position_ki, br_root_m2006_position_kd);
    Upper_ConfigurePidGains(&m2006_larm.speed_pid, br_root_m2006_speed_kp, br_root_m2006_speed_ki, br_root_m2006_speed_kd);
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娆撴煠閹帒鍔ょ€规洜顨廵r_BindMotors闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
void Upper_BindMotors(upper_t *upper)
{
    if (upper == NULL)
        return;

    m2006_larm.output_ratio = 14.0f / 45.0f;
    rs01_rarm.output_ratio = RS01_ROOT_OUTPUT_RATIO;


    m2006_larm.trajectory_time_s = BR_ROOT_M2006_TRAJECTORY_TIME_S;
    rs01_rarm.trajectory_time_s = BR_ROOT_RS01_TRAJECTORY_TIME_S;
    rs00_larm[0].trajectory_time_s = BR_BIG_ARM_RS00_TRAJECTORY_TIME_S;
    rs00_rarm[0].trajectory_time_s = BR_BIG_ARM_RS00_TRAJECTORY_TIME_S;
    rs00_larm[1].trajectory_time_s = BR_SMALL_ARM_RS00_TRAJECTORY_TIME_S;
    rs00_rarm[1].trajectory_time_s = BR_SMALL_ARM_RS00_TRAJECTORY_TIME_S;
    j4310_larm.trajectory_time_s = BR_J4310_TRAJECTORY_TIME_S;
    j4310_rarm.trajectory_time_s = BR_J4310_TRAJECTORY_TIME_S;
    rs05_larm.trajectory_time_s = BR_RS05_TRAJECTORY_TIME_S;
    rs05_rarm.trajectory_time_s = BR_RS05_TRAJECTORY_TIME_S;
    m3508_dcup[0].trajectory_time_s = BR_AUX_M3508_TRAJECTORY_TIME_S;
    m2006_dcup[0].trajectory_time_s = BR_M2006_CUP0_TRAJECTORY_TIME_S;
    m2006_ucup[0].trajectory_time_s = BR_M2006_CUP0_TRAJECTORY_TIME_S;
    m2006_dcup[0].trajectory_enabled = BR_M2006_CUP0_USE_QUINTIC != 0U;
    m2006_ucup[0].trajectory_enabled = BR_M2006_CUP0_USE_QUINTIC != 0U;
    m2006_dcup[1].trajectory_time_s = BR_M2006_CUP1_TRAJECTORY_TIME_S;
    m2006_ucup[1].trajectory_time_s = BR_M2006_CUP1_TRAJECTORY_TIME_S;
    m2006_dcup[1].trajectory_enabled = BR_M2006_CUP1_USE_QUINTIC != 0U;
    m2006_ucup[1].trajectory_enabled = BR_M2006_CUP1_USE_QUINTIC != 0U;
    Upper_ConfigureM2006Gains();
    upper->arm[0] = (arm_group_t){&rs05_larm, &j4310_larm, &rs00_larm[1], &rs00_larm[0], &m2006_larm, NULL};
    upper->arm[1] = (arm_group_t){&rs05_rarm, &j4310_rarm, &rs00_rarm[1], &rs00_rarm[0], NULL, &rs01_rarm};
    last_service_tick = 0U;
    memset(zero_calibration_previous, 0, sizeof(zero_calibration_previous));
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺咃綁姊虹紒妯哄婵☆垰锕ゅ嵄濠电姵纰嶉埛鎺懨归敐鍜佹綗闁逞屽厴閸嬫捇姊虹粙娆惧剱闁圭懓娲獮鍐偡閹冲﹥妞介、鏃堝幢濞嗘垹宕堕梻浣筋嚙鐎涒晝鍠婂澶嬪仭闁冲搫鎳庣粻鏍偓鐟板婢瑰寮告惔銏㈢闁糕剝锚閻忊晠鎷戦柆宥嗏拻濞达絿鐡旈崵娆愮箾閹绢噮妫戠紒顔硷功缁﹥淇婂绁宊OnFrame闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劑寮撮姀鈾€鎷洪梺鍛婄箓鐎氼參藟閻樼粯鐓曢柣妯虹－婢х敻鏌ｅ☉鍗炴珝妤犵偛顑夐弫鍌滅磼濡粯鐝ㄩ梻鍌欒兌绾爼宕滃┑瀣仭鐟滄棁妫熷銈嗘磵閸嬫挻鎱ㄦ繝鍐┿仢鐎规洦鍋婂畷鐔碱敆娴ｇ懓鈧兘姊绘担鍓插悢闁哄鐏濋～鍥倵濞堝灝鏋ょ紒顕呭灣閹广垹鈹戠€ｎ亞锛涢梺浼欑到閼活垶寮稿☉銏＄厱闁靛ň鏅濋悾娲煙閻撳海绉烘い銏℃礋閿濈偤顢橀悢绯曟灆闂傚倸鍊搁崐鐑芥嚄閸撲礁鍨濇い鏍亹閳ь剨绠撳畷濂稿Ψ閵夛附袣闂備礁鎼粙渚€宕㈡總鍛婂€块柛顭戝亖娴滄粓鏌熺€涙绠栭柛瀣ㄥ劜娣囧﹪宕滆閸樻挳鏌?*/
void Upper_OnFrame(upper_t *upper, const can_frame_t *frame)
{
    uint32_t now = HAL_GetTick();
    unsigned a;
    if (upper == NULL || frame == NULL)
        return;
    for (a = 0U; a < 2U; a++)
    {
        rs00_t *rs00 = a == 0U ? rs00_larm : rs00_rarm;
        rs05_t *rs05 = a == 0U ? &rs05_larm : &rs05_rarm;
        j4310_t *j4310 = a == 0U ? &j4310_larm : &j4310_rarm;
        if (RS00_OnFrame(&rs00[0], frame)) { arm_feedback_angle[a][ARM_JOINT_SHOULDER] = rs00[0].position * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_SHOULDER, rs00[0].position, rs00[0].velocity, rs00[0].torque, rs00[0].temperature_c, rs00[0].fault, now); return; }
        if (RS00_OnFrame(&rs00[1], frame)) { arm_feedback_angle[a][ARM_JOINT_ELBOW] = rs00[1].position * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_ELBOW, rs00[1].position, rs00[1].velocity, rs00[1].torque, rs00[1].temperature_c, rs00[1].fault, now); return; }
        if (J4310_OnFrame(j4310, frame)) { arm_feedback_angle[a][ARM_JOINT_WRIST] = j4310->feedback.position_rad * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_WRIST, j4310->feedback.position_rad, j4310->feedback.speed_rad_s, j4310->feedback.torque_nm, j4310->feedback.mos_temperature, j4310->feedback.state, now); return; }
        if (RS05_OnFrame(rs05, frame)) { arm_feedback_angle[a][ARM_JOINT_ROTATION] = rs05->position * ARM_RAD_TO_DEG; telemetry(&upper->board[a], ARM_JOINT_ROTATION, rs05->position, rs05->velocity, rs05->torque, rs05->temperature_c, rs05->fault, now); return; }
    }
    if (M2006_OnFrame(&m2006_larm, frame))
    {
        arm_feedback_angle[ARM_LARM_INDEX][ARM_JOINT_ROOT] = m2006_larm.position_rad * ARM_RAD_TO_DEG;
        telemetry(&upper->board[0], ARM_JOINT_ROOT, m2006_larm.position_rad, m2006_larm.speed_rad_s,
                  m2006_larm.current_raw * M2006_CURRENT_SCALE * BR_LEFT_ROOT_NM_PER_A, 0, 0, now);
        return;
    }
    if (RS01_OnFrame(&rs01_rarm, frame))
    {
        arm_feedback_angle[ARM_RARM_INDEX][ARM_JOINT_ROOT] = rs01_rarm.position * ARM_RAD_TO_DEG;
        telemetry(&upper->board[1], ARM_JOINT_ROOT, rs01_rarm.position, rs01_rarm.velocity, rs01_rarm.torque, 0,
                  rs01_rarm.fault, now);
    }
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娆撴煠閹帒鍔ょ€规洜顨廵r_ToRsMode闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
static rs_mode_t Upper_ToRsMode(arm_mode_t mode)
{
    switch (mode)
    {
    case ARM_MODE_DAMPING: return RS_MODE_DAMPING;
    case ARM_MODE_SPEED: return RS_MODE_SPEED;
    case ARM_MODE_POSITION: return RS_MODE_POSITION;
    case ARM_MODE_MIT: return RS_MODE_MIT;
    case ARM_MODE_STOP:
    default: return RS_MODE_STOP;
    }
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娆撴煠閹帒鍔ょ€规洜顨廵r_ToJ4310Mode闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
static j4310_mode_t Upper_ToJ4310Mode(arm_mode_t mode)
{
    switch (mode)
    {
    case ARM_MODE_DAMPING: return J4310_MODE_DAMPING;
    case ARM_MODE_SPEED: return J4310_MODE_SPEED;
    case ARM_MODE_POSITION: return J4310_MODE_POSITION;
    case ARM_MODE_MIT: return J4310_MODE_MIT;
    case ARM_MODE_STOP:
    default: return J4310_MODE_STOP;
    }
}

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺呫垺绻濋姀锝嗙【闁荤啿鏅犻幃銏ゅ礂鐏忔牗瀚奸梻浣哄帶椤洟宕愰弴鐔侯浄婵犲﹤鐗婇悡娆撴煟閿濆懐鐒鹃柡鈧悶绶噛_arm_motor闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姕闁诲繑鎸剧槐鎾诲磼濮樺崬顣哄銈庡幘閸忔ê顕ｇ拠娴嬫婵犲﹤鎳愰弶鎼佹煟鎼搭垳绉靛ù婊勭矋閹便劍鎯旈妸锔规嫼闂佽壈顫夊姗€宕ヨぐ鎺撶厱闁绘ê纾晶顏呫亜閺囶亞鎮奸柍褜鍓涢弫鍝ユ兜閸洖纾婚柟鐐灱濡插牊绻涢崨鐗堢彧闁搞劌娼￠獮鍐晸閻樿尙鍔﹀銈嗗笒鐎氼參鍩涢幋锔界厽闁瑰瓨姊瑰▍鍡樼箾閸涱厽澶勯柕鍥у椤㈡﹢鎮欓棃娑掑徍闁诲氦顫夊ú鈺冪礊閳ь剚銇勯敃鈧紞濠囧蓟閻旂⒈鏁婇悷娆忓閻濇岸姊虹涵鍛彧缂侇喖閰ｅ畷鐗堢節閸嬬晫鍠愬顏堝级濞嗙偓顥″┑鐘垫暩閸嬬娀骞撻鍡楃筏闁秆勩仠閳ь剙鍊圭粋鎺斺偓锝庡亝濞?*/
static bool Upper_ArmTemperatureSafe(unsigned arm)
{
    rs00_t *rs00 = arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm;
    rs05_t *rs05 = arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm;
    j4310_t *j4310 = arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm;
    rs01_t *rs01 = &rs01_rarm;
    if (!j4310->feedback.received) return true;
    if (rs00[0].temperature_c >= 80.0f || rs00[1].temperature_c >= 80.0f ||
        rs05->temperature_c >= 80.0f ||
        j4310->feedback.mos_temperature >= 80U || j4310->feedback.rotor_temperature >= 80U)
        return false;
    return arm == ARM_LARM_INDEX || rs01->temperature_c < 80.0f;
}

static bool Upper_ArmTemperatureOverLimit(unsigned arm)
{
    rs00_t *rs00 = arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm;
    rs05_t *rs05 = arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm;
    j4310_t *j4310 = arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm;
    rs01_t *rs01 = &rs01_rarm;
    if (!j4310->feedback.received) return false;
    return rs00[0].temperature_c > 80.0f || rs00[1].temperature_c > 80.0f ||
           rs05->temperature_c > 80.0f ||
           j4310->feedback.mos_temperature > 80U || j4310->feedback.rotor_temperature > 80U ||
           (arm == ARM_RARM_INDEX && rs01->temperature_c > 80.0f);
}
static void Upper_ApplyRecoveryState(unsigned arm, bool active)
{
    rs00_t *rs00 = arm == ARM_LARM_INDEX ? rs00_larm : rs00_rarm;
    rs05_t *rs05 = arm == ARM_LARM_INDEX ? &rs05_larm : &rs05_rarm;
    j4310_t *j4310 = arm == ARM_LARM_INDEX ? &j4310_larm : &j4310_rarm;
    rs00[0].recovery_trajectory = active;
    rs00[1].recovery_trajectory = active;
    rs05->recovery_trajectory = active;
    j4310->recovery_trajectory = active;
    if (arm == ARM_LARM_INDEX) m2006_larm.recovery_trajectory = active;
    else rs01_rarm.recovery_trajectory = active;
    if (active)
    {
        for (unsigned joint = 0U; joint < ARM_JOINT_COUNT; joint++)
        {
            /* Keep enabled arm joints in MIT position control during recovery.
             * The left root M2006 is converted to native position mode in
             * apply_arm_motor(), because that driver has no MIT interface. */
            arm_mode[arm][joint] = ARM_MODE_MIT;
            arm_target[arm][joint] = 0.0f;
        }
    }
}

bool Upper_IsArmOvertemperature(uint8_t arm)
{
    return arm < 2U && arm_overtemp_latched[arm];
}
static void apply_arm_motor(void *motor, unsigned arm, arm_joint_t joint, float tau)
{
    arm_mode_t mode = arm_mode[arm][joint];
    float target = arm_target[arm][joint];

    if (joint == ARM_JOINT_ROOT)
    {
        tau = mode == ARM_MODE_MIT ? br_mit_tau[arm][joint] : 0.0f;
        if (arm == ARM_LARM_INDEX)
        {
            m2006_t *root_motor = (m2006_t *)motor;
            m2006_t *root = (m2006_t *)motor;
            (void)mode;
            (void)tau;
            M2006_SetTargetDeferred(root_motor, M2006_MODE_POSITION,
                                    (target * ARM_DEG_TO_RAD - BR_LEFT_ROOT_START_RAD) /
                                        BR_LEFT_ROOT_JOINT_PER_OUTPUT * ARM_RAD_TO_DEG);
        }
        else if (arm == ARM_RARM_INDEX)
        {
            rs01_t *root_motor = (rs01_t *)motor;
            if (mode == ARM_MODE_MIT)
                RS01_SetTargetMIT(root_motor, RS_MODE_MIT, tau, target);
            else
                RS01_SetTargetBasic(root_motor, Upper_ToRsMode(mode), target);
        }
        return;
    }

    if (joint == ARM_JOINT_SHOULDER || joint == ARM_JOINT_ELBOW)
    {
        rs00_t *m = (rs00_t *)motor;
        m->tau_ff_nm = tau;
        if (mode == ARM_MODE_MIT) RS00_SetTargetMIT(m, RS_MODE_MIT, tau, target);

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝庝簽閻も偓濠电偠鎻徊浠嬪箟閳ユ枼鏋嶆俊銈呭暟绾句粙鏌涚仦鎹愬闁逞屽墯閹倸鐣烽娑橆嚤闁哄鍨归崫妤呮⒑?0_SetTargetBasic闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姎缂佺姾娅ｇ槐鎾存媴閻熸澘顫嶅銈冨妼閿曨亜顕ｆ繝姘伋闁归绀侀幃鎴︽⒑閸涘﹣绶遍柛鐘愁殜楠炲啫鈻庨幘绮规嫼闂備緡鍋嗛崑娑㈡嚐椤栨稒娅犳い鏇楀亾闁哄本绋戦～婵嬵敆婢跺﹤澹庨柣搴ゎ潐濞叉牜绱炴繝鍐х箚闁归棿绀佸敮濡炪倖鏌ㄩ崥瀣枍婵犲洦鈷掑〒姘ｅ亾婵炰匠鍥佸洭顢橀姀鐘碉紮濠德板€曢崯浼存儗濞嗗繆鏀介柣妯诲絻閺嗘洜绱掔拠鍙夘棡闁靛洤瀚板浠嬪Ω閿濆嫮鎹曢梻浣侯焾椤戝棝骞愰幖浣瑰仼闁绘垼妫勯悙濠囨煃閸濆嫸鏀婚柡浣靛€濆?*/
        else RS00_SetTargetBasic(m, Upper_ToRsMode(mode), target);
    }
    else if (joint == ARM_JOINT_WRIST)
    {
        j4310_t *m = (j4310_t *)motor;
        if (mode == ARM_MODE_MIT) J4310_SetTargetMIT(m, J4310_MODE_MIT, tau, target);

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝庝簽閻も偓濠电偠鎻徊浠嬪箟閳ユ枼鏋嶆俊銈呭暟绾句粙鏌涚仦鎹愬闁逞屽墯閹倸鐣烽娑橆嚤閻庢稒蓱濞?310_SetTargetBasic闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姎缂佺姾娅ｇ槐鎾存媴閻熸澘顫嶅銈冨妼閿曨亜顕ｆ繝姘伋闁归绀侀幃鎴︽⒑閸涘﹣绶遍柛鐘愁殜楠炲啫鈻庨幘绮规嫼闂備緡鍋嗛崑娑㈡嚐椤栨稒娅犳い鏇楀亾闁哄本绋戦～婵嬵敆婢跺﹤澹庨柣搴ゎ潐濞叉牜绱炴繝鍐х箚闁归棿绀佸敮濡炪倖鏌ㄩ崥瀣枍婵犲洦鈷掑〒姘ｅ亾婵炰匠鍥佸洭顢橀姀鐘碉紮濠德板€曢崯浼存儗濞嗗繆鏀介柣妯诲絻閺嗘洜绱掔拠鍙夘棡闁靛洤瀚板浠嬪Ω閿濆嫮鎹曢梻浣侯焾椤戝棝骞愰幖浣瑰仼闁绘垼妫勯悙濠囨煃閸濆嫸鏀婚柡浣靛€濆?*/
        else J4310_SetTargetBasic(m, Upper_ToJ4310Mode(mode), target);
    }
    else if (joint == ARM_JOINT_ROTATION)
    {
        rs05_t *m = (rs05_t *)motor;
        m->tau_ff_nm = tau;
        if (mode == ARM_MODE_MIT) RS05_SetTargetMIT(m, RS_MODE_MIT, tau, target);

/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝庝簽閻も偓濠电偠鎻徊浠嬪箟閳ユ枼鏋嶆俊銈呭暟绾句粙鏌涚仦鎹愬闁逞屽墯閹倸鐣烽娑橆嚤闁哄鍨归崫妤呮⒑?5_SetTargetBasic闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姎缂佺姾娅ｇ槐鎾存媴閻熸澘顫嶅銈冨妼閿曨亜顕ｆ繝姘伋闁归绀侀幃鎴︽⒑閸涘﹣绶遍柛鐘愁殜楠炲啫鈻庨幘绮规嫼闂備緡鍋嗛崑娑㈡嚐椤栨稒娅犳い鏇楀亾闁哄本绋戦～婵嬵敆婢跺﹤澹庨柣搴ゎ潐濞叉牜绱炴繝鍐х箚闁归棿绀佸敮濡炪倖鏌ㄩ崥瀣枍婵犲洦鈷掑〒姘ｅ亾婵炰匠鍥佸洭顢橀姀鐘碉紮濠德板€曢崯浼存儗濞嗗繆鏀介柣妯诲絻閺嗘洜绱掔拠鍙夘棡闁靛洤瀚板浠嬪Ω閿濆嫮鎹曢梻浣侯焾椤戝棝骞愰幖浣瑰仼闁绘垼妫勯悙濠囨煃閸濆嫸鏀婚柡浣靛€濆?*/
        else RS05_SetTargetBasic(m, Upper_ToRsMode(mode), target);
    }
}
/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鍨鹃幇浣圭稁婵犵數濮甸懝鍓х玻濡ゅ懏鐓涢柛銉ｅ劚閻忊晝绱掗埀顒勫磼濞戞瑥寮垮┑锛勫仩椤曆勭妤ｅ啯鈷戦弶鐐村椤︼妇绱掓径瀣唉闁挎繄鍋涢埞鎴犫偓锝呯仛閺咃綁姊虹紒妯哄闁哄懏绮庨埀顒€鐏氶悧鐘差潖濞差亝鍤掗柕鍫濇噸濞岊亞绱撴担鍓插剱闁搞劌娼￠妴浣肝旈崨顔煎墻闂佹眹鍎冲Λ娓瀇Service闂傚倸鍊峰ù鍥敋瑜嶉湁闁绘垼妫勯弸渚€鏌熼梻瀵割槮缂佺姷濞€閺岀喖鎮欓鈧崝璺衡攽椤旇棄鈻曢柡宀嬬磿娴狅妇鎷犻幓鎺戭潥闂備胶顭堟鍝ョ矓瑜版帒钃熸繛鎴欏灩閸楁娊鏌曟繛鍨姍缂併劌銈稿娲閳哄啰校闂佹悶鍔岄悥鐓庮嚕婵犳碍鏅搁柣妯荤叀閸炲爼姊洪棃娑氱濠殿喚鏁婚幃妤€煤椤忓應鎷洪梺鑽ゅ枑婢瑰棝鎮鹃銏＄厱闁靛鍔嶉ˉ澶愭煟閿濆懎妲婚柍瑙勫灩閳ь剨缍嗘禍鐐村鐎ｎ喗鐓熼幖绮瑰墲鐠愨€斥攽椤旇姤灏﹂柟顔炬暬椤㈡稑顭ㄩ崨顖ょ闯闂備胶顭堥張顒勬偡瑜旇棟闁挎洖鍊归悡鐔煎箳閹惰棄绀夌€广儱顦粻鐐烘煏婵炵偓娅呮い顐㈡嚇閺屽秹鍩℃担鍛婃濠殿喗锕粻鎾愁潖妤﹁￥浜归柟鐑樺灣閸犲﹪姊洪幖鐐插闁靛牏顭堥锝夊Ω閿旂虎娴勯柣搴秵閸嬪棝宕㈤挊澶嗘斀闁宠棄妫楅悘鐘绘煟韫囨梻绠為柟顕嗙節楠炲洭鎮ч崼銏犲笚闁荤喐绮嶇划鎾崇暦濡も偓閳藉濮€閳锯偓閹稿啴鎮楅獮鍨姎妞わ缚鍗冲?*/
void Upper_Service(uint32_t now) {
  float dt =
      last_service_tick == 0U ? 0.002f : (now - last_service_tick) * 0.001f;
  unsigned a;
  if (dt <= 0.0f || dt > 0.02f)
    dt = 0.002f;
  last_service_tick = now;
  configure_motor_gains();
  for (a = 0U; a < 2U; a++) {
    if (ARM_OVERTEMP_PROTECTION_ENABLED != 0U) {
      if (arm_overtemp_latched[a] && Upper_ArmTemperatureSafe(a))
        arm_overtemp_latched[a] = false;
      else if (!arm_overtemp_latched[a] && Upper_ArmTemperatureOverLimit(a))
        arm_overtemp_latched[a] = true;
    } else {
      arm_overtemp_latched[a] = false;
    }
    Upper_ApplyRecoveryState(a, arm_overtemp_latched[a]);
  }
  Upper_ConfigureM2006Gains();
  for (a = 0U; a < 2U; a++)
    for (unsigned zero_joint = 0U; zero_joint < ARM_JOINT_COUNT; zero_joint++)
      Upper_ZeroCalibrationRequest(a, zero_joint);
  for (a = 0U; a < 2U; a++) {
    bool enabled = upper_state.enable != 0U;
    float q[BR_ARM_DOF] = {0}, q_deg[BR_ARM_DOF] = {0}, tau[BR_ARM_DOF] = {0};
    rs00_t *rs00 = a == 0U ? rs00_larm : rs00_rarm;
    rs05_t *rs05 = a == 0U ? &rs05_larm : &rs05_rarm;
    j4310_t *j4310 = a == 0U ? &j4310_larm : &j4310_rarm;
    bool online = rs00[0].feedback_valid && rs00[1].feedback_valid &&
                  rs05->feedback_valid && j4310->feedback.received;
    /* Gravity geometry follows positive arm_target command angles. */
    for (unsigned joint = 0U; joint < ARM_JOINT_COUNT; ++joint)
      q_deg[joint] = arm_target[a][joint];
    for (unsigned joint = 0U; joint < BR_ARM_DOF; joint++)
      q[joint] = q_deg[joint] * ARM_DEG_TO_RAD;
    upper_state.board[a].online = online;
    br_gravity_models[a].payload_mass_kg = 0.174f;
    {
      const bool payload_attached =
          (a == ARM_LARM_INDEX) ? (gpio_pd5 != 0U) : (gpio_pd6 != 0U);
      upper_state.board[a].gravity_ready =
          online && BR_GravityEvaluate(&br_gravity_models[a], q,
                                       payload_attached, tau, NULL);
    }
    float end_position_m[3];
    if (!BR_ForwardKinematics(&br_gravity_models[a], q, end_position_m)) {
      arm_end_position_world[a][0] = 0.0f;
      arm_end_position_world[a][1] = 0.0f;
      arm_end_position_world[a][2] = 0.0f;
    } else {
      arm_end_position_world[a][0] = end_position_m[0] * 100.0f;
      arm_end_position_world[a][1] = end_position_m[1] * 100.0f;
      arm_end_position_world[a][2] = end_position_m[2] * 100.0f;
    }
    if (!upper_state.board[a].gravity_ready)
      memset(tau, 0, sizeof(tau));
    if (br_manual_tau_enable == 0U)
    {
      /* Automatic mode: expose the raw gravity torques for debugging. */
      br_shoulder_tau_nm = tau[ARM_JOINT_SHOULDER];
      br_elbow_tau_nm = tau[ARM_JOINT_ELBOW];
      br_wrist_tau_nm = tau[ARM_JOINT_WRIST];
    }
    if (br_manual_tau_enable != 0U) {
      tau[1] = br_shoulder_tau_nm;
      tau[2] = br_elbow_tau_nm;
      tau[3] = br_wrist_tau_nm;
      tau[4] = br_rotation_tau_nm;
    }
    if (!enabled) {
      RS00_SetTargetBasic(&rs00[0], RS_MODE_STOP, 0);
      RS00_SetTargetBasic(&rs00[1], RS_MODE_STOP, 0);
      RS05_SetTargetBasic(rs05, RS_MODE_STOP, 0);
      J4310_SetTargetBasic(j4310, J4310_MODE_STOP, 0);
      if (a == 1U)
        RS01_SetTargetBasic(&rs01_rarm, RS_MODE_STOP, 0);
      else
        M2006_SetTargetDeferred(&m2006_larm, M2006_MODE_STOP, 0);
    } else {
      apply_arm_motor(a == ARM_LARM_INDEX ? (void *)&m2006_larm
                                          : (void *)&rs01_rarm,
                      a, ARM_JOINT_ROOT, 0.0f);
      apply_arm_motor(&rs00[0], a, ARM_JOINT_SHOULDER, tau[1]);
      apply_arm_motor(&rs00[1], a, ARM_JOINT_ELBOW, tau[2]);
      apply_arm_motor(j4310, a, ARM_JOINT_WRIST, tau[3]);
      apply_arm_motor(rs05, a, ARM_JOINT_ROTATION, tau[4]);
    }
    RS00_Service(&rs00[0]);
    RS00_Service(&rs00[1]);
    RS05_Service(rs05);
    J4310_Service(j4310, tau[3], now);
    if (a == 1U)
      RS01_Service(&rs01_rarm);
    memcpy((void *)upper_state.board[a].gravity_tau_nm, tau, sizeof(tau));
    for (unsigned zero_joint = 0U; zero_joint < ARM_JOINT_COUNT; zero_joint++)
      Upper_ZeroCalibrationComplete(a, zero_joint);
  }
}
