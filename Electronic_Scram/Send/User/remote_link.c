/*
 * @file remote_link.c
 * @brief remote_link婵犵數濮烽。钘壩ｉ崨鏉戝瀭妞ゅ繐鐗嗛悞鍨亜閹烘垵鏆為柣婵愪邯閺屾稓鈧絻鍔岄崝锕傛煛瀹€瀣М濠碘剝鎮傛俊鐑藉Ψ椤旇崵纾婚梻鍌欐祰濡椼劑姊藉澶婄９婵犻潧顑囧畵渚€鎮楅敐搴℃灍闁绘挶鍎甸幃妤呮晲鎼存繄鐩庡銈忛檮閻撯€愁潖濞差亜宸濆┑鐘插閸ゅ倻绱撴担钘夎敿闁稿鍔楅崚鎺楊敇閻愬灚娈曢梺閫炲苯澧撮柛鈹惧亾濡炪倖宸婚崑鎾寸節閳ь剟鏌嗗鍛紱闂佺粯妫侀崑鎰暤娓氣偓閺屾盯濡烽鐓庮潻閺? */

#include "remote_link.h"
#include "usart.h"

static uint8_t sw_bits;
static uint8_t sw_last;
static uint8_t sw_prev;
static uint8_t sw_candidate;
static uint8_t sw_sample_count;
static uint8_t sw_initialized;
static uint32_t last_tx_tick;
static uint32_t buzzer_until;
static uint8_t link_ready;
static uint8_t tx_frame[3];

/* 闂傚倸鍊风粈渚€骞夐垾鎰佹綎濠电姵鑹剧粣妤呮煛閸ャ儱鐏╃紒鈧崒娑欏弿婵＄偠顕ф禍楣冩⒑鏉炴壆顦︾紒澶嬫尦閿濈偛鈹戠€ｅ灚鏅㈤梺閫炲苯澧撮柟顔斤耿婵＄兘濡烽姀鈩冩澑婵＄偑鍊栧濠氬煕閸儱纾块煫鍥ㄧ⊕閸婄敻鏌ｉ姀鐘典粵缂佸墎灏僲oteLink_Init闂傚倷娴囬褍霉閻戣棄鏋侀柟闂寸缁犵娀鏌熼悙顒€鍔跺┑顔藉▕閺岋紕浠︾拠鎻掑闂佺楠哥粔褰掑蓟濞戙垹鍗抽柕濞垮劙閼割亜鈹戦埥鍡楃仩婵☆偅绻堥獮鍐喆閸曨剙顎撻悗鐟板濠㈡﹢鎮甸悢鍏尖拺闂侇偆鍋涢懟顖炲储閸濄儳纾奸柍褜鍓氶幏鍛村礈閹绘帗顓挎俊鐐€栫敮鎺斺偓姘煎墴瀵憡绗熼埀顒勫蓟閳╁啫绶為悗锝庝憾閸ゅ鏌ｆ惔銏㈩暡闁绘挴鈧剚鍤曢柛顐ｆ礀閻撴盯鏌涚仦鎹愬妞ゎ偅鍨甸—鍐Χ閸愩劎浠鹃梺鎼炲妼濞硷繝鐛崘顔肩厸闁告侗鍠栧▓婵堢磼閸撗冾暭闁挎艾霉濠婂嫬鍔ら棁澶愭煥濠靛棙绁╅柡鍡╁灦閺?*/
void RemoteLink_Init(void)
{
    sw_bits = 0U; sw_last = 0xFFU; sw_prev = 0U; sw_candidate = 0U; sw_sample_count = 0U; sw_initialized = 0U;
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    link_ready = 1U;
}

/* 闂傚倸鍊风粈渚€骞夐垾鎰佹綎濠电姵鑹剧粣妤呮煛閸ャ儱鐏╃紒鈧崒娑欏弿婵＄偠顕ф禍楣冩⒑鏉炴壆顦︾紒澶嬫尦閿濈偛鈹戦崱娆愭闂佹悶鍎崝搴ㄥ焵椤掑寮柡宀嬬秮閹晠宕ｆ径濠備憾闂備焦鎮堕崝宀€绱炴繝鍥х疇闁绘柨鍚嬮崑鍕煟閹炬娊顎楃€规挸妫濋弻锝嗘償閵忊懇濮囬柦鍐憾閺岋綁骞橀姘RemoteLink_PollSwitches闂傚倷娴囬褍霉閻戣棄鏋侀柟闂寸缁犵娀鏌熼悙顒€鍔跺┑顔藉▕閺岋紕浠︾拠鎻掑闂佺楠哥粔褰掑蓟濞戙垹鍗抽柕濞垮劜閻忔挾绱撻崒姘卞妞ゎ厾鍏樺璇测槈濠婂懐鏉搁柣搴秵娴滄粍鎱ㄦ惔銊︹拺缁绢厼鎳忛懙褰掓煕濡や礁鈻曟鐐插暣閸╋繝宕ㄩ鍏肩杺缂備礁澧芥晶妤呮晪婵炲瓨绮嶉崕鎶芥箒闂佹寧绻傚ù鍌炲汲椤栫偞鐓?*/
void RemoteLink_PollSwitches(void)
{
    uint8_t sample;

    sample = 0U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) sample |= 1U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) sample |= 2U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) sample |= 4U;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) sample |= 8U;

    if (sw_initialized == 0U) {
        sw_prev = sample;
        sw_candidate = sample;
        sw_sample_count = 1U;
        sw_bits = sample;
        sw_initialized = 1U;
        return;
    }

    if (sample != sw_candidate) {
        sw_candidate = sample;
        sw_sample_count = 1U;
    } else if (sw_sample_count < 3U) {
        sw_sample_count++;
    }

    if (sw_sample_count >= 3U && sw_candidate != sw_prev) {
        sw_prev = sw_candidate;
        sw_bits = sw_prev;
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        buzzer_until = HAL_GetTick() + 100U;
    } else {
        sw_bits = sw_prev;
    }
}
/* 闂傚倸鍊风粈渚€骞夐垾鎰佹綎濠电姵鑹剧粣妤呮煛閸ャ儱鐏╃紒鈧崒娑欏弿婵＄偠顕ф禍楣冩⒑鏉炴壆顦︾紒澶嬫尦閿濈偛鈹戦崱娆愭闂佹悶鍎崝搴ㄥ焵椤掑寮柡宀嬬秮閹晠宕ｆ径濠備憾闂備焦鎮堕崝宀€绱炴繝鍥х疇闁绘柨鍚嬮崑鍕煟閹炬娊顎楃€规挸妫濋弻锝嗘償閵忊懇濮囬柦鍐憾閺岋綁骞橀姘RemoteLink_Process闂傚倷娴囬褍霉閻戣棄鏋侀柟闂寸缁犵娀鏌熼悙顒€鍔跺┑顔藉▕閺岋紕浠︾拠鎻掑闂佺楠哥粔褰掑蓟濞戙垹鍗抽柕濞垮劜閻忔挾绱撻崒姘卞妞ゎ厾鍏樺璇测槈濠婂懐鏉搁柣搴秵娴滄粍鎱ㄦ惔銊︹拺缁绢厼鎳忛懙褰掓煕濡や礁鈻曟鐐插暣閸╋繝宕ㄩ鍏肩杺缂備礁澧芥晶妤呮晪婵炲瓨绮嶉崕鎶芥箒闂佹寧绻傚ù鍌炲汲椤栫偞鐓?*/
void RemoteLink_Process(void)
{
    uint32_t now;
    if (link_ready == 0U) return;
    now = HAL_GetTick();
    if (buzzer_until != 0U && (int32_t)(now - buzzer_until) >= 0) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        buzzer_until = 0U;
    }
    if ((sw_bits == sw_last) && ((uint32_t)(now - last_tx_tick) < 100U)) return;
    if (huart1.gState != HAL_UART_STATE_READY) return;
    tx_frame[0] = 0xA5U; tx_frame[1] = 0x5AU; tx_frame[2] = (uint8_t)(sw_bits & 0x0FU);
    if (HAL_UART_Transmit_DMA(&huart1, tx_frame, sizeof(tx_frame)) == HAL_OK) {
        sw_last = sw_bits; last_tx_tick = now;
    }
}

/* 闂傚倸鍊风粈渚€骞夐垾鎰佹綎濠电姵鑹剧粣妤呮煛閸ャ儱鐏╃紒鈧崒娑欏弿婵＄偠顕ф禍楣冩⒑鏉炴壆顦︾紒澶嬫尦閿濈偛鈹戠€ｅ灚鏅㈤梺閫炲苯澧扮€垫澘锕ょ叅妞ゅ繐鎳夐幏鍝勵渻閵堝棗濮х紒缁樼箓鍗辨い銈囧箑oteLink_ForwardRawFrame闂傚倷娴囬褍霉閻戣棄鏋侀柟闂寸缁犵娀鏌熼悙顒€鍔跺┑顔藉▕閺岋紕浠︾拠鎻掑闂佺楠哥粔褰掑蓟濞戙垹鍗抽柕濞垮劚椤亜鈹戦悙鑼闁绘牕銈稿濠氭偄閻戞ê鐝伴梺鐐藉劥濞呮洟鎮樺澶嬧拺闁硅偐鍋涙俊鎼佹煕閺冣偓閸ㄥ潡鎮伴鈧獮鎺楀即閵娤€鈺呮⒒娴ｈ櫣銆婇柡鍛矒閵嗗啯绻濋崶鈺佺ウ闂佸湱澧楀姗€鎮欐繝鍥ㄧ叆闁哄啫娲﹂ˉ澶屾偖濮椻偓濮?*/
void RemoteLink_ForwardRawFrame(void) {}
