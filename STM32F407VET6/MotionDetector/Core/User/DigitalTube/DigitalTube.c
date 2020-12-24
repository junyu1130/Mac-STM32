//
// Created by Y on 2020/10/28.
//

#include "DigitalTube.h"

void Write595(uint8_t sel, uint8_t num, uint8_t bdot)//共阴数码管. '0'~7'， '8'~'F'
{
    static const uint8_t TAB[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
                                    0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};
    // 74HC138输出高电平，关数码管显示
    HAL_GPIO_WritePin(DigitalTube_A3_GPIO_Port, DigitalTube_A3_Pin, GPIO_PIN_RESET);
    uint8_t dat = TAB[num & 0x0F] | (bdot ? 0x80 : 0x00);
    // 74HC595锁存8段数据
    for (uint8_t i = 0; i < 8; ++i) {
        HAL_GPIO_WritePin(DigitalTube_SCK_GPIO_Port, DigitalTube_SCK_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DigitalTube_SER_GPIO_Port, DigitalTube_SER_Pin, (dat & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        dat <<= 1;
        HAL_GPIO_WritePin(DigitalTube_SCK_GPIO_Port, DigitalTube_SCK_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DigitalTube_DISLK_GPIO_Port, DigitalTube_DISLK_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DigitalTube_DISLK_GPIO_Port, DigitalTube_DISLK_Pin, GPIO_PIN_SET);
        // 74HC138译码，sel (0~3)对应4位片选信号输出
        HAL_GPIO_WritePin(DigitalTube_A0_GPIO_Port, DigitalTube_A0_Pin, (sel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DigitalTube_A1_GPIO_Port, DigitalTube_A1_Pin, (sel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DigitalTube_A2_GPIO_Port, DigitalTube_A2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DigitalTube_A3_GPIO_Port, DigitalTube_A3_Pin, GPIO_PIN_SET);
    }
}
