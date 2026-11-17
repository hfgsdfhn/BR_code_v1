/*
 * @file remote_link.c
 * @brief remote_link濠碘槅鍨埀顒冩珪閸嬨儵鏌ｉ妸銉ヮ伀闁轰降鍊濋獮瀣煥鐎ｎ亜顦查梺鍛婄懕缁茶偐绮畝鈧埀顒€婀遍崑鐔肩嵁閸ヮ剙违? */

#include "remote_link.h"
#include "usart.h"
#include <string.h>

static remote_port_t ports[REMOTE_PORT_COUNT];
static uint8_t lora_frame_state;

static void RemoteLink_ApplyLoraStatus(uint8_t status)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_5,
                      (status & (1U << 3)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,
                      (status & (1U << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,
                      (status & (1U << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void RemoteLink_ProcessLoraByte(uint8_t byte)
{
    switch (lora_frame_state)
    {
    case 0U:
        if (byte == 0xA5U) lora_frame_state = 1U;
        break;
    case 1U:
        if (byte == 0x5AU) lora_frame_state = 2U;
        else lora_frame_state = (byte == 0xA5U) ? 1U : 0U;
        break;
    default:
        RemoteLink_ApplyLoraStatus((uint8_t)(byte & 0x0FU));
        lora_frame_state = 0U;
        break;
    }
}

static remote_port_t *RemoteLink_FindPort(UART_HandleTypeDef *uart)
{
    uint32_t index;
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) {
        if (ports[index].uart == uart) return &ports[index];
    }
    return NULL;
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋鐐差槻闂佽　鍋撻悹娲細鐎氭瑩鎮峰▎娆戠ɑ闁诲海绁瞖moteLink_StartReceive闁诲海鏁搁幊鎾惰姳閺屻儲鍎嶉柛鏇ㄥ枤缂堝鏌涜箛瀣姕闁哄棛鍠栭獮鎴︻敋閸℃瑧顦梺鍝勬搐閻°劑寮绘繝鍋界喖鍨鹃搹顐淮闂佺粯顭堥崺鏍焵椤戝潡妾烽柍?*/
static HAL_StatusTypeDef RemoteLink_StartReceive(remote_port_t *port)
{
    if (port->uart->hdmarx == NULL || port->uart->hdmatx == NULL) return HAL_ERROR;

    if (port->uart->hdmarx->Init.Mode != DMA_CIRCULAR ||
        port->uart->hdmatx->Init.Mode != DMA_NORMAL) return HAL_ERROR;
/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋鐐差槻闂佽　鍋撻悹娲細鐎氭瑥顭跨捄鍝勵伀闁诡喖锕ュ蹇涘箻閸愭彃姹查梺杞拌兌婢ф鐣垫笟鈧俊?*/
    return HAL_UART_Receive_DMA(port->uart, port->rx_buffer, REMOTE_RX_SIZE);
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋鐐差槻闂佽　鍋撻悹娲細鐎氭瑩鎮峰▎娆戠ɑ闁诲海绁瞖moteLink_UpdateReceive闁诲海鏁搁幊鎾惰姳閺屻儲鍎嶉柛鏇ㄥ枤缂堝鏌涜箛瀣姕闁哄棛鍠栭獮鎴︻敋閸℃瑧顦梺鍝勬搐閻°劑寮绘繝鍋界喖鍨鹃搹顐淮闂佺粯顭堥崺鏍焵椤戝潡妾烽柍?*/
static uint32_t RemoteLink_UpdateReceive(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t produced;
    uint16_t position;
    uint16_t count;
    __disable_irq();
    position = (uint16_t)((REMOTE_RX_SIZE - __HAL_DMA_GET_COUNTER(port->uart->hdmarx)) % REMOTE_RX_SIZE);
    count = (uint16_t)((position + REMOTE_RX_SIZE - port->rx_position) % REMOTE_RX_SIZE);
    port->rx_position = position;
    port->rx_produced += count;
    produced = port->rx_produced;
    __DMB();
    __set_PRIMASK(primask);
    return produced;
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋澶嬓熼梺鑽ゅ仜閹藉坏moteLink_Recover闁诲海鏁搁幊鎾惰姳閺屻儲鍎嶉柛鏇ㄥ枤缂堝鏌涜箛瀣姎闁艰崵鍠栧瀵糕偓娑櫳戦悡鈧梺鐑╂櫇椤牓宕ｈ閹晠鎳滅喊妯轰壕濞达絿顣介崑?*/
static void RemoteLink_Recover(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_UART_Abort(port->uart) != HAL_OK) {
        __set_PRIMASK(primask);
        return;
    }
    port->dropped_bytes += port->tx_write - port->tx_read;
    port->dropped_bytes += port->rx_produced - port->rx_consumed;
    port->rx_produced = port->rx_consumed = 0U;
    port->rx_position = 0U;
    port->tx_write = port->tx_read = 0U;
    port->tx_active_size = 0U;
    port->tx_complete = 0U;
    port->restart_requested = 0U;
    if (RemoteLink_StartReceive(port) != HAL_OK) port->restart_requested = 1U;
    __set_PRIMASK(primask);
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋鈧徊娲⒑椤愶繝妲moteLink_TransmitLocked闁诲海鏁搁幊鎾惰姳閺屻儲鍎嶉柛鏇ㄥ墯濞堝爼鏌熺拠鈥虫灍闁搞劊鍔戦獮鎺曨槻闁糕晜顨婇獮鎰板炊閿旇棄袘闂?*/
static void RemoteLink_TransmitLocked(remote_port_t *port)
{
    uint32_t available;
    uint16_t offset;
    uint16_t length;
    HAL_StatusTypeDef status;
    if (port->restart_requested != 0U) return;
    if (port->tx_complete != 0U) {
        port->tx_read += port->tx_active_size;
        port->tx_active_size = 0U;
        port->tx_complete = 0U;
    }
    if (port->tx_active_size != 0U) return;
    available = port->tx_write - port->tx_read;
    if (available == 0U) return;
    offset = (uint16_t)(port->tx_read % REMOTE_TX_SIZE);
    length = (uint16_t)(REMOTE_TX_SIZE - offset);
    if (available < length) length = (uint16_t)available;
    port->tx_active_size = length;

    status = HAL_UART_Transmit_DMA(port->uart, &port->tx_buffer[offset], length);
    if (status != HAL_OK) {
        port->tx_active_size = 0U;
        if (status != HAL_BUSY) port->restart_requested = 1U;
    }
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋鈧徊娲⒑椤愶繝妲moteLink_Transmit闁诲海鏁搁幊鎾惰姳閺屻儲鍎嶉柛鏇ㄥ墯濞堝爼鏌熺拠鈥虫灍闁搞劊鍔戦獮鎺曨槻闁糕晜顨婇獮鎰板炊閿旇棄袘闂?*/
static void RemoteLink_Transmit(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    RemoteLink_TransmitLocked(port);
    __set_PRIMASK(primask);
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋鈧徊娲⒑椤愶繝妲moteLink_Forward闁诲海鏁搁幊鎾惰姳閺屻儲鍎嶉柛鏇ㄥ墯濞堝爼鏌熺拠鈥虫灍闁搞劊鍔戦獮鎺曨槻闁糕晜顨婇獮鎰板炊閿旇棄袘闂?*/
static void RemoteLink_Forward(remote_port_t *source, remote_port_t *target)
{
    uint32_t produced;
    uint32_t available;
    uint32_t space;
    uint32_t count;
    uint32_t index;
    uint8_t byte;
    if (source->restart_requested != 0U || target->restart_requested != 0U) return;
    produced = RemoteLink_UpdateReceive(source);
    __DMB();
    available = produced - source->rx_consumed;
    if (available > REMOTE_RX_SIZE) {
        source->rx_overruns++;
        source->dropped_bytes += available;
        source->rx_consumed = produced;
        return;
    }
    if (source == &ports[REMOTE_LORA_PORT]) {
        uint32_t parse_start = source->lora_parsed;
        if (produced - parse_start > REMOTE_RX_SIZE)
            parse_start = produced - REMOTE_RX_SIZE;
        while (parse_start < produced) {
            RemoteLink_ProcessLoraByte(source->rx_buffer[parse_start % REMOTE_RX_SIZE]);
            parse_start++;
        }
        source->lora_parsed = produced;
    }    space = REMOTE_TX_SIZE - (target->tx_write - target->tx_read);
    count = (available < space) ? available : space;
    for (index = 0U; index < count; index++) {
        byte = source->rx_buffer[(source->rx_consumed + index) % REMOTE_RX_SIZE];
        target->tx_buffer[(target->tx_write + index) % REMOTE_TX_SIZE] = byte;
    }
    __DMB();
    produced = RemoteLink_UpdateReceive(source);
    if (produced - source->rx_consumed > REMOTE_RX_SIZE) {
        source->rx_overruns++;
        source->dropped_bytes += produced - source->rx_consumed;
        source->rx_consumed = produced;
        return;
    }
    if (source->restart_requested != 0U || target->restart_requested != 0U) return;
    target->tx_write += count;
    source->rx_consumed += count;
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋鈧悘銉モ攽椤旂⒈鍎忛悗鐢垫緪emoteLink_Init闁诲海鏁搁幊鎾惰姳閺屻儲鍎嶉柛鏇ㄥ亽濡棝鎮规担宄板珟闁逞屽厸缁€浣割嚕椤掑嫬鏋侀柡澶嬪鐎氭煡寮堕埡鍌氬妞ゃ垺鍨块幃鈺呮嚋绾版ê浜惧ù锝囶暯閸?*/
void RemoteLink_Init(void)
{
    uint32_t index;
    memset(ports, 0, sizeof(ports));
    lora_frame_state = 0U;
    ports[0].uart = &huart1;
    ports[1].uart = &huart3;
    HAL_Delay(10U);
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) {
        if (RemoteLink_StartReceive(&ports[index]) != HAL_OK) {
            ports[index].restart_requested = 1U;
        }
    }
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濆畷鍥ㄧ€柣鐘叉储閸ㄥ綊鎳熼悢鐓庡嵆閻庢稒蓱閻撯偓RemoteLink_Process闁诲海鏁搁幊鎾惰姳閺屻儲鍎嶉柛鏇ㄥ枤缂堝鏌涜箛瀣姎闁硅渹鍗抽幃鈺呮嚋绾版ê浜惧ù锝囶暯閸?*/
void RemoteLink_Process(void)
{
    uint32_t index;
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) {
        if (ports[index].restart_requested != 0U) RemoteLink_Recover(&ports[index]);
        RemoteLink_Transmit(&ports[index]);
    }

    RemoteLink_Forward(&ports[REMOTE_LORA_PORT], &ports[REMOTE_HOST_PORT]);
    RemoteLink_Forward(&ports[REMOTE_HOST_PORT], &ports[REMOTE_LORA_PORT]);
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) RemoteLink_Transmit(&ports[index]);
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋澶嬓熼梺?UART 缂備礁鏈钘壩涙禒瀣闁靛闄勯弳顏劽瑰鍐惧剮婵炲棎鍨介悰顕€鎳滈崹顐ゅ嚱闂佸搫鍊绘晶妤冩暜閹绢喖缁╅柛鎾茶兌婢跺嫰鏌涢幇顓炵祷閻忓浚鍨舵俊?*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    remote_port_t *port = RemoteLink_FindPort(uart);
    (void)size;
    if (port != NULL && port->restart_requested == 0U) {
        (void)RemoteLink_UpdateReceive(port);
    }
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋澶嬓熼梺?UART 闂佸憡鐟﹂崹鍧楀焵椤戣法顦﹂柣锝庡墴楠炲骞囬鍡欐婵炲濮惧▔娑㈡嚐閻斿吋鐓傚┑鐘插暞閺夊綊鏌涘▎鎰伌闁逞屽厸閼宠埖鎱ㄩ幖浣哥畱濞达絿顣介崑?*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    remote_port_t *port = RemoteLink_FindPort(uart);
    if (port != NULL) {
        port->tx_complete = 1U;
        RemoteLink_Transmit(port);
    }
}

/* 闂佸憡姊婚崰鏇㈠礂濮椻偓閺佸秴顫濋澶嬓熼梺?UART 闂備焦瀵ч悷銊╊敋閵堝棛顩查悗锝傛櫆椤愯姤顨ラ悙鏉戝姸妞ゆ洦鍓欐晥闁稿瞼鍋為幊鐘绘偣娓氼垰鐏″ù鐙€鍠楀鍕吋閸愶絽浜?*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    remote_port_t *port = RemoteLink_FindPort(uart);
    if (port != NULL) {
        port->uart_errors++;
        port->restart_requested = 1U;
    }
}

