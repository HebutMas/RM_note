# CMakeLists.txt — bsp 硬件抽象层

## 文件位置

`mas_embedded_threadx/board/bsp/CMakeLists.txt`

## 作用

被 [[02_code_twin/board/dji_c/CMakeLists-txt]] 通过 `add_subdirectory(../../board/bsp ...)` 加载。把 CubeMX 生成的 HAL 调用封装成更高级的接口（`BSP_CAN_Send()`、`BSP_UART_Send()` 等），上层模块不直接碰 HAL。

---

## 原始代码

```cmake
add_library(bsp OBJECT
    bsp_init.c
    DWT/bsp_dwt.c
    UART/bsp_uart.c
    SPI/bsp_spi.c
    I2C/bsp_i2c.c
    FLASH/bsp_flash.c
    GPIO/bsp_gpio.c
    PWM/bsp_pwm.c
    USB/usbd_cdc_acm_user.c
    LED/bsp_led.c
    BEEP/bsp_beep.c
    CAN/bsp_can.c
    CAN/bsp_can_task.c
)

target_compile_options(bsp PRIVATE -O2)

target_include_directories(bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/DWT
    ${CMAKE_CURRENT_SOURCE_DIR}/UART
    ${CMAKE_CURRENT_SOURCE_DIR}/SPI
    ${CMAKE_CURRENT_SOURCE_DIR}/I2C
    ${CMAKE_CURRENT_SOURCE_DIR}/FLASH
    ${CMAKE_CURRENT_SOURCE_DIR}/GPIO
    ${CMAKE_CURRENT_SOURCE_DIR}/PWM
    ${CMAKE_CURRENT_SOURCE_DIR}/USB
    ${CMAKE_CURRENT_SOURCE_DIR}/LED
    ${CMAKE_CURRENT_SOURCE_DIR}/BEEP
    ${CMAKE_CURRENT_SOURCE_DIR}/CAN
)

target_link_libraries(bsp PUBLIC stm32cubemx azrtos::threadx utils cherryusb)
```

---

## 与 utils 的差异

结构和 [[02_code_twin/utils/CMakeLists-txt]] 完全一致，都是简单模式 OBJECT 库。差异点：

| | utils | bsp |
|---|---|---|
| 链接依赖 | `stm32cubemx azrtos::threadx` | `stm32cubemx azrtos::threadx utils cherryusb` |
| 多链接的原因 | — | USB 子目录用到了 CherryUSB 的 CDC ACM 接口；bsp 封装了 HAL 调用，依赖 utils 的日志输出 |

每个子目录对应一种外设：CAN、UART、SPI、I2C、PWM、DWT（精确延时）、FLASH、GPIO、USB（CDC 虚拟串口）、LED、BEEP。
