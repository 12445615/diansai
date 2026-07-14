# MSPM0 电机控制工程

## 2026-07-15 调试记录

### 硬件和换算参数

- 编码器分辨率：`500 PPR`
- 减速比：`1:20`
- 编码器计数方式：A、B 两相上升沿计数，按 `2 倍频` 计算
- 输出轴一圈计数：`500 x 20 x 2 = 20000`
- 编码器测速周期：`10 ms`
- PID 控制周期：`10 ms`
- 目标转速 `30 rpm` 对应：`100 个计数/10 ms`

### 今天遇到的问题

1. 第一次下载程序后编码器可以计数，再次下载同一程序后可能不能计数；物理复位无效时，完全断电再上电可以恢复。
2. 固定调用 `Set_Pwm1(4000)`、`Set_Pwm2(-4000)` 时可以测速，但从零开始由 PID 缓慢增加 PWM 时，实际速度长时间为零。
3. PID 最初输出约为 20，低于电机能够克服静摩擦并开始转动的 PWM，因此电机轴没有旋转，也不会产生编码器脉冲。
4. 目标速度、编码器反馈和定时器曾分别使用 5 ms 与 10 ms，导致目标值与实际值无法直接比较。
5. 用手转动车轮时，`Encoder_IRQ_Count` 不增加；PWM 驱动电机后，中断计数和实际速度可以增加。
6. PWM、目标速度和实际速度使用不同单位，曾将它们直接比较：PWM 是驱动比较值，目标速度和实际速度是每个采样周期内的编码器计数。

### 今天完成的修改

1. 编码器换算统一使用 `500 PPR`、`1:20` 减速比和 `2 倍频`。
2. 编码器测速和 PID 周期统一为 `10 ms`，目标速度换算同步改为 10 ms。
3. 转速换算和 PID 中间计算使用 `float`，脉冲计数、PWM 和状态标志继续使用整数，原有调用方式保持不变。
4. 启动时使用：

   ```c
   Set_Pwm1(0);
   Set_Pwm2(0);
   ```

   `Set_Pwm1(0)` 和 `Set_Pwm2(0)` 会清零 PWM 比较寄存器并停止输出。
5. PWM 死区暂定为 `600`：

   - 输入为 0 时保持完全停止。
   - 非零 PWM 的绝对值小于 600 时，实际输出提升到 600。
   - 绝对值大于或等于 600 时保持原值，例如 1000 仍然输出 1000。

6. 增加编码器诊断变量：

   - `Encoder_Raw_Pins`：编码器 GPIO 原始电平。
   - `Encoder_IRQ_Count`：GPIO 中断进入次数。
   - `Encoder_Timer_Count`：测速定时器进入次数。

7. CCS Debug 工程已完成构建和链接，生成 `Debug/sanshihaodaima.out`。

### 已经确认的结论

- PWM 驱动电机时，`Encoder_IRQ_Count` 和 `Reality_Velocity1` 能够变化，说明编码器信号、GPIO 中断和软件计数链路能够工作。
- 死区只解释“PID 输出太小时电机没有起转，因此没有速度反馈”，不能解释手转车轮时为什么没有脉冲。
- 编码器 GPIO 中断并不依赖 `Set_Pwm` 开启，`Set_Pwm` 只控制电机方向和 PWM 输出。
- 目标速度和实际速度应使用相同的“计数/10 ms”单位进行 PID 比较，不能直接拿它们与 PWM 数值比较。

### 仍未解决或尚未完全验证

1. **重复下载后偶尔不能计数的根本原因尚未完全确定。** 已修改启动状态和 PWM 清零逻辑，但仍需连续多次执行“下载、复位、断电上电”测试，确认问题是否彻底消失。
2. **手转车轮无法测速的原因尚未完全确定。** 目前更可能是 `1:20` 减速箱难以从输出轴反向带动电机后端编码器，也需要排除编码器在 PWM 为零时供电不正确的问题。
3. **死区 600 是当前试验值，不是最终标定值。** 左右电机的真实启动死区可能不同，需要分别测量。
4. **PID 参数尚未完成整定。** 死区补偿解决起步问题后，还需要观察超调、振荡、稳态误差以及左右轮速度差。
5. 工程仍有少量 Unicode 空格和 OLED 字符指针类型编译警告，当前不影响生成固件，但后续应清理。

### 下一步验证方法

1. 保持 `Set_Pwm1(0)`，测量编码器 VCC 对 GND 的电压，确认电机停止时编码器仍稳定供电。
2. 不转车轮，直接转动电机后端编码器轴或磁铁，观察 `Encoder_Raw_Pins` 和 `Encoder_IRQ_Count`：

   - 原始引脚变化且中断增加：编码器电气链路正常，车轮手转问题属于机械传动。
   - 原始引脚变化但中断不增加：继续检查 GPIO 中断配置。
   - 原始引脚完全不变化：检查编码器供电、公共地、接线和引脚映射。

3. 连续多次下载相同程序，每次记录下载后、物理 RESET 后和完全断电后的三个诊断计数。
4. 同时记录 `Target_Velocity1`、`Reality_Velocity1` 和实际 PWM，分别标定左右电机的启动死区并重新整定 PID。

---

## 原始 TI 工程说明

## Example Summary

Empty project using DriverLib.
This example shows a basic empty project using DriverLib with just main file
and SysConfig initialization.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SYSCTL |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA20 | DEBUGSS | SWCLK | N/A | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | N/A | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0G3507 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873).

## Example Usage

Compile, load and run the example.
