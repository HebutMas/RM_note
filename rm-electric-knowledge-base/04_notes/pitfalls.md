# 踩坑记录

> 持续追加。格式：`### 日期 - 问题`，记录问题现象、原因、解决方法。
>
> **日志系统使用**：如需了解 ulog 日志系统的 API 用法和 Ozone Terminal 查看方式，见 [[../../02_code_twin/utils/ulog]]。

---

### 2026-07-18 - AI 辅助 Flash PGSERR 调试

**问题现象**：BMI088 标定时调用 `BSP_FLASH_Write_Buffer` 写入扇区 11 失败，`FLASH->SR = 0xC0`（PGSERR + PGPERR），HAL 返回 `HAL_ERROR`。但调度器启动后的测试线程里同样的写操作却能成功。

**调试过程**：

1. **让 AI 读懂代码**：把 `bsp_flash.c`、`module_bmi088_cali.c`、`stm32f4xx_it.c`、`main.c` 的初始化流程全部喂给 AI，让它理解完整的调用链 `main → tx_application_define → Robot_Init → MODULE_Init → Module_BMI088_init → calibrate → BSP_FLASH_Write_Buffer`。

2. **让 AI 联网搜索**：AI 搜到了 ST 官方 LAT1210（DMA 中断残留往地址 0 写导致 PGSERR）、硬汉论坛（IWDG 未初始化就 Refresh）、Stack Overflow（栈溢出野指针写 Flash）等多个同类案例。虽然最终根因不是这些，但帮我们排除了几个方向。

3. **插桩二分定位**：让 AI 在 `main.c` 的每个 `MX_xxx_Init` 后面、`robot_init.c` 的 BSP/MODULE/APP Init 之间插入 `FLASH->SR` 检查日志，全速跑看日志。发现进入 `Robot_Init` 时 SR 还是 CLEAN，但到 BMI088 校准调 Flash 时已经脏了——说明是在 BSP_Init 内部某个操作弄脏的。

4. **关键转折**：AI 提出在 `BSP_FLASH_Erase_Sector` 和 `BSP_FLASH_Write_Buffer` 入口加 `__HAL_FLASH_CLEAR_FLAG` 清除残留标志。这不是“头疼医头”——HAL 库自己的 `FLASH_WaitForLastOperation` 里也有类似的清除逻辑。加上之后 Flash 操作立即成功。

5. **注意 SR 位映射**：AI 一开始把 FLASH_SR 的位映射写错了（PGSERR 误标为 bit1），导致 `SR=0xC0` 被误判为 EOP。后来查 STM32F407 头文件发现 PGSERR=bit7、PGPERR=bit6，修正后 `0xC0 = PGSERR + PGPERR` 才对上。

**根因**：BSP_Init 阶段某个 MX_Init（疑似 ADC DMA 或 SPI DMA 中断）把 FLASH->SR 弄脏了，脏标志一直残留到 BMI088 标定时。由于日志系统在 main 阶段未初始化，无法精确定位是哪个 Init。但清标志后问题解决。

**经验**：

- AI 辅助调试的有效模式：**AI 读代码 + 联网搜案例 + 生成插桩代码 + 分析日志**，人类负责烧录跑和决策方向
- `FLASH->SR` 的错误标志是黏着的，上次报错后不清除会一直影响后续操作。Flash 操作前进门清标志是标准防御性编程
- main 阶段日志系统没初始化时，用 `printf` 或直接读寄存器值在 Ozone 里看
- 逐行调试会干扰 Flash 控制器（J-Link 读 Flash 干扰时序），全速跑更可靠
- STM32F407 FLASH_SR 位定义：EOP=bit0, OPERR=bit1, WRPERR=bit4, PGAERR=bit5, PGPERR=bit6, PGSERR=bit7, BSY=bit16

---

<!-- 在此上方追加新记录 -->
