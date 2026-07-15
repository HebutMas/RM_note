# 【SBUS】一文看懂SBUS协议

> **来源**：CSDN 博客 @kk1kk2kk3kk4
> **原文链接**：https://blog.csdn.net/kk1kk2kk3kk4/article/details/135316071
> **发布时间**：2024-01-04（最后修改：2025-11-11）
> **转载自**：@郭老二 https://blog.csdn.net/u010168781/article/details/129380783
> **存档时间**：2026-07-15（防止原文 404 丢失）

---

## 1 简介

S.BUS是一个串行通信协议，S.BUS是FUTABA提出的舵机控制总线。

S.bus使用RS232C串口的硬件协议作为自己的硬件运行基础。
- 使用TTL电平，即3.3V
- 使用负逻辑，即低电平为"1"，高电平为"0"
- 波特率：100000（100k），注意：不兼容波特率115200

## 2 硬件电路

硬件取反电路如下，实际上就是一个很简单的三极管电路。Sbus的信号从基极输入，从集电极输出。基极输入 '0'，集电极上拉输出 '1'；基极输入 '1'，三极管导通，输出被拉低为 '0'，实现了反向。

或者使用 74HC04 等反相器芯片。

## 3 协议格式

协议帧很简洁，一帧包括25字节数据：

```
首部（1字节）+ 数据（22字节）+ 标志位（1字节）+ 结束符（1字节）
```

- **首部**：起始字节 = `0000 1111b`（0x0F）
- **数据**：22 字节的数据，分别代表16个通道的数据，也即是每个通道的值用了 11 位来表示，22×8/16 = 11
  - 每个通道的取值范围为 0~2047，低位在前、高位在后
- **标志位**：1字节，高四位从高到低依次表示：
  - bit7：CH17数字通道
  - bit6：CH16数字通道
  - bit5：帧丢失（Frame lost）
  - bit4：安全保护（Failsafe）：失控保护激活位（0x10）判断飞机是否失控
  - bit3~bit0：低四位不用
- **结束符**：0x00

## 4 协议解析

### 4.1 解析方法

将数据解析为通道的方法：

### 4.2 示例一

```c
void Sbus_Data_Count(uint8_t *buf)
{
    CH[0]  = ((int16_t)buf[2] >> 0  | ((int16_t)buf[3] << 8))                 & 0x07FF;
    CH[1]  = ((int16_t)buf[3] >> 3  | ((int16_t)buf[4] << 5))                 & 0x07FF;
    CH[2]  = ((int16_t)buf[4] >> 6  | ((int16_t)buf[5] << 2)  | (int16_t)buf[6] << 10) & 0x07FF;
    CH[3]  = ((int16_t)buf[6] >> 1  | ((int16_t)buf[7] << 7))                 & 0x07FF;
    CH[4]  = ((int16_t)buf[7] >> 4  | ((int16_t)buf[8] << 4))                 & 0x07FF;
    CH[5]  = ((int16_t)buf[8] >> 7  | ((int16_t)buf[9] << 1)  | (int16_t)buf[10] << 9) & 0x07FF;
    CH[6]  = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6))                & 0x07FF;
    CH[7]  = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3))                & 0x07FF;
    CH[8]  = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8))                & 0x07FF;
    CH[9]  = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5))                & 0x07FF;
    CH[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2) | (int16_t)buf[17] << 10) & 0x07FF;
    CH[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7))                & 0x07FF;
    CH[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4))                & 0x07FF;
    CH[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1) | (int16_t)buf[21] << 9) & 0x07FF;
    CH[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6))                & 0x07FF;
    CH[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3))                & 0x07FF;
}
```

> **注意**：上述代码中 buf 的索引从 1 开始（buf[1] = 0x0F 帧头），所以 buf[2] 对应帧的第 2 个字节（data[1]）。

### 4.3 示例二

示例二是一个完整的 STM32 HAL 库代码。

#### 头文件

```c
// sbus.h
#ifndef SBUS_H
#define SBUS_H

#include "sys.h"

#define USART_BUF_SIZE      8       // HAL库USART接收Buffer大小
#define SBUS_DATA_SIZE      25      // 25字节

#define SBUS_PIN        GPIO_PIN_2 | GPIO_PIN_3         // PA2--TX, PA3--RX
#define SBUS_GPIO       GPIOA
#define SBUS_ENCLK()    __HAL_RCC_GPIOA_CLK_ENABLE();   \
                        __HAL_RCC_USART2_CLK_ENABLE();  // 使能GPIOA时钟 // 使能USART2时钟

struct SBUS_t{
    uint8_t head;                   // 1字节首部
    uint16_t ch[16];                // 16个字节数据
    uint8_t flag;                   // 1字节标志位
    uint8_t end;                    // 1字节结束
};

void SBUS_Init(void);
void SbusParseTask(void *arg);

#endif
```

#### 源文件

```c
// sbus.c
#include "sbus.h"
#include "delay.h"

uint8_t usart_buf[USART_BUF_SIZE];
uint8_t sbus_rx_head = 0;       // 发现起始字节 0x0F
uint8_t sbus_rx_sta = 0;        // sbus 接收状态，0：未完成，1：已完成一帧接收
uint8_t sbus_rx_index;          // 接收字节计数
uint8_t sbus_rx_buf[SBUS_DATA_SIZE];  // 接收sbus数据缓冲区
struct SBUS_t sbus;             // SBUS 结构体实例化
UART_HandleTypeDef UART2_Handler;    // 串口2配置句柄

void SBUS_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    // 时钟使能
    SBUS_ENCLK();

    // 串口初始化配置
    // 波特率100kbps，8位数据，偶校验(even)，2位停止位，无流控。
    UART2_Handler.Instance = USART2;
    UART2_Handler.Init.BaudRate = 100000;
    UART2_Handler.Init.WordLength = UART_WORDLENGTH_8B;
    UART2_Handler.Init.StopBits = UART_STOPBITS_2;
    UART2_Handler.Init.Parity = UART_PARITY_EVEN;
    UART2_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART2_Handler.Init.Mode = UART_MODE_TX_RX;

    // 引脚配置
    GPIO_Initure.Pin = SBUS_PIN;            // PA2--TX, PA3--RX
    GPIO_Initure.Mode = GPIO_MODE_AF_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_HIGH;
    GPIO_Initure.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_Initure);

    // 中断配置
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 4);
    HAL_UART_Init(&UART2_Handler);          // HAL_UART_Init()会使能UART2

    HAL_UART_Receive_IT(&UART2_Handler, (uint8_t *)usart_buf, USART_BUF_SIZE);
    // 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
}

/* USART2 中断服务函数 */
/* 实现对S.BUS协议缓存，头部为 0x0F,结尾为 0x00, 中间22Bytes16通道数据，1Byte标志符 */
void USART2_IRQHandler(void)
{
    uint8_t chr;
    if ((__HAL_UART_GET_FLAG(&UART2_Handler, UART_FLAG_RXNE) != RESET))
    {
        HAL_UART_Receive(&UART2_Handler, &chr, 1, 1000);  // 接收一个字符
        if (sbus_rx_sta == 0)  // 接收未完成
        {
            if ((chr == 0x0F) || sbus_rx_head)  // 找到首字节或已经找到首字节
            {
                sbus_rx_head = 1;
                if (sbus_rx_index < SBUS_DATA_SIZE)  // 未接收到25个字符
                {
                    sbus_rx_buf[sbus_rx_index] = chr;
                    sbus_rx_index++;
                }
                else  // 接收到25个字符了
                {
                    sbus_rx_sta = 1;       // 接收完成
                    sbus_rx_head = 0;
                    sbus_rx_index = 0;
                }
            }
        }
    }
    HAL_UART_IRQHandler(&UART2_Handler);
}

/* 对SBUS协议数据进行解析 */
void SbusParseTask(void *arg)
{
    while (1)
    {
        if (sbus_rx_sta == 1)  // 接收完一帧
        {
            NVIC_DisableIRQ(USART2_IRQn);  // 关闭中断，防止读写混乱

            sbus.head = sbus_rx_buf[0];    // 首部
            sbus.flag = sbus_rx_buf[23];   // 标志符
            sbus.end = sbus_rx_buf[24];    // 结尾

            sbus.ch[0]  = ((sbus_rx_buf[2]  << 8)  + (sbus_rx_buf[1]))       & 0x07ff;
            sbus.ch[1]  = ((sbus_rx_buf[3]  << 5)  + (sbus_rx_buf[2]  >> 3)) & 0x07ff;
            sbus.ch[2]  = ((sbus_rx_buf[5]  << 10) + (sbus_rx_buf[4]  << 2)  + (sbus_rx_buf[3]  >> 6)) & 0x07ff;
            sbus.ch[3]  = ((sbus_rx_buf[6]  << 7)  + (sbus_rx_buf[5]  >> 1)) & 0x07ff;
            sbus.ch[4]  = ((sbus_rx_buf[7]  << 4)  + (sbus_rx_buf[6]  >> 4)) & 0x07ff;
            sbus.ch[5]  = ((sbus_rx_buf[9]  << 9)  + (sbus_rx_buf[8]  << 1)  + (sbus_rx_buf[7]  >> 7)) & 0x07ff;
            sbus.ch[6]  = ((sbus_rx_buf[10] << 6)  + (sbus_rx_buf[9]  >> 2)) & 0x07ff;
            sbus.ch[7]  = ((sbus_rx_buf[11] << 3)  + (sbus_rx_buf[10] >> 5)) & 0x07ff;
            sbus.ch[8]  = ((sbus_rx_buf[13] << 8)  + (sbus_rx_buf[12]))      & 0x07ff;
            sbus.ch[9]  = ((sbus_rx_buf[14] << 5)  + (sbus_rx_buf[13] >> 3)) & 0x07ff;
            sbus.ch[10] = ((sbus_rx_buf[16] << 10) + (sbus_rx_buf[15] << 2)  + (sbus_rx_buf[14] >> 6)) & 0x07ff;
            sbus.ch[11] = ((sbus_rx_buf[17] << 7)  + (sbus_rx_buf[16] >> 1)) & 0x07ff;
            sbus.ch[12] = ((sbus_rx_buf[18] << 4)  + (sbus_rx_buf[17] >> 4)) & 0x07ff;
            sbus.ch[13] = ((sbus_rx_buf[20] << 9)  + (sbus_rx_buf[19] << 1)  + (sbus_rx_buf[18] >> 7)) & 0x07ff;
            sbus.ch[14] = ((sbus_rx_buf[21] << 6)  + (sbus_rx_buf[20] >> 2)) & 0x07ff;
            sbus.ch[15] = ((sbus_rx_buf[22] << 3)  + (sbus_rx_buf[21] >> 5)) & 0x07ff;

            NVIC_EnableIRQ(USART2_IRQn);
            sbus_rx_sta = 0;
        }
        else
        {
            delay_ms(500);
        }
    }
}
```
