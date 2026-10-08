# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Micromouse robot firmware for an STM32F411CEU6 (Cortex-M4F, UFQFPN48) on a custom board, built with STM32CubeIDE 1.19 / CubeMX 6.15 (FW_F4 V1.28.3, HAL drivers). The code is at the hardware bring-up stage: IR sensors, encoders, IMU, motor PWM and USB telemetry are each wired up and tested on their own. Of the cascaded PID named in the README, only the inner per-wheel velocity loop exists so far (`Motor_Speed_Control`). It is off by default (`motor_pid_enable = 0`). Comments are in Vietnamese, and strings sent over USB are Vietnamese without diacritics.

Application code is split by subsystem. Each module's header is in `Core/Inc/`:

| File | Contains |
|---|---|
| `main.c` | CubeMX init plus `USER CODE` blocks: `CDC_Print`/`CDC_WaitHost`, the start-up order, the TIM9 dispatcher `HAL_TIM_PeriodElapsedCallback`, and the `while(1)` loop |
| `main.h` (`USER CODE` EC/EFP) | `CONTROL_DT` (shared 2 ms tick) and the `CDC_Print`/`CDC_WaitHost`/`UART_Print`/`UART_WaitStart` prototypes |
| `pid.c` / `pid.h` | Motor driver, encoders, velocity PID, the outer motion loop (`Move_Straight`/`Turn_Angle`, encoder distance plus gyro heading), `PID_Test`, `Rectangle_Test`, `Straight_Test`. All tuning and pin/sign config `#define`s live in `pid.h` |
| `mpu6500.c` / `mpu6500.h` | **MPU6500 only**, no MPU6050/9250 code paths, I2C address `MPU6500_ADDR`. IMU init (including a `WHO_AM_I` read into `mpu_whoami`: 0x70 MPU6500, or 0x71 for the MPU9250 die actually fitted) and calibration, DMA read tick, `HAL_I2C_MemRxCpltCallback`, `HAL_I2C_ErrorCallback` (counts I2C errors), angle math, `MPU6500_Test` |
| `adc.c` / `adc.h` | IR ADC+DMA start, emitters, `adc_value[]` |

Sections marked "(chưa có code)" are planned and still empty: wall distance and wall detection in `adc`. IR-based heading correction for maze runs doesn't exist yet either. Each module has its own `static char msg[200]` for printing; there is no shared global `msg`. Do **not** enable CubeMX's "Generate peripheral initialization as a pair of .c/.h files per peripheral", because it would generate its own `Core/Src/adc.c` and clobber this one.

## Build, flash, debug

There are no tests or linters. The build is the CubeIDE managed build. Its makefiles are generated into `Debug/` (and `Release/`), which are gitignored. `make` and the GCC 13.3 toolchain come bundled inside CubeIDE and are not on PATH. To build from Git Bash at the repo root:

```bash
TC=/c/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins
export PATH="$TC/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin:$TC/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845/tools/bin:$PATH"
make -C Debug -j8 all      # -> Debug/version1.elf (+ .map, .list, size report)
make -C Debug clean
```

`Debug/**/subdir.mk` lists only the sources that existed the last time the IDE generated it. After you add or remove a `.c` file, change `.cproject` options, or start from a fresh clone, regenerate the makefiles: build once in the IDE, or close the IDE (it holds a workspace lock) and run:

```bat
C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\headless-build.bat -data C:\Users\ADMIN\STM32CubeIDE\workspace_1.19.0 -build version1/Debug
```

On a workspace where the project isn't imported yet, add `-import <repo path>`. The IDE compiles only these source folders: `Core`, `Core/Startup`, `Drivers`, `Middlewares`, `USB_DEVICE`.

Flash over ST-Link SWD:

```bash
"$TC/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107/tools/bin/STM32_Programmer_CLI.exe" -c port=SWD -w Debug/version1.elf -v -rst
```

For interactive debugging, use the IDE launch config `version1 Debug.launch` (ST-Link GDB server, SWD).

Compiler settings to know about:
- Debug builds with `-O0 -g3` and Release with `-Os`.
- Hard-float `fpv4-sp-d16`.
- `-std=gnu11 -Wall`.
- newlib-nano with `-u _printf_float`, so `%f` in `snprintf` works.

## CubeMX code generation rules

- **`version1.ioc` is the source of truth** for pins, clocks, peripherals, DMA and NVIC. Make peripheral changes there and regenerate. Don't hand-edit `MX_*_Init()`, `stm32f4xx_hal_msp.c`, the IRQ handler bodies in `stm32f4xx_it.c`, or `USB_DEVICE/Target/`. Never edit the `.ioc` text by hand.
- Regeneration keeps **only** code inside `/* USER CODE BEGIN x */ … /* USER CODE END x */` blocks. Everything else in `Core/` and `USB_DEVICE/` gets overwritten. The `.ioc` also sets `DeletePrevious=true`, so generated files that are no longer needed get deleted. The module files (`pid`, `mpu6500`, `adc`) are not touched by CubeMX. In `main.c` the user code is spread across these blocks:
  - `PFP`: the USB print helpers
  - `2`: the start-up order `ADC_IR_Start()` → `Motor_Init()` → `HAL_TIM_Base_Start_IT(&htim9)`, with TIM9 last so the ISR never sees an uninitialized module
  - `WHILE`, before `while(1)`: `ADC_IR_EmittersOn()`, then `MPU6500_Setup()` (init plus about 1.2 s of calibration, which then sets `imu_ready`)
  - `4`: the TIM9 callback, which only calls `Motor_Speed_Tick()` and then `MPU6500_Tick()`
- New `.c` files under `Core/Src` are compiled automatically, but the IDE only sees files created outside it after a **Refresh (F5)** of the project. A missing module shows up as `undefined reference` at link time. For CLI builds, `Debug/Core/Src/subdir.mk` and `Debug/objects.list` must also list the file; an IDE or headless build regenerates both. The same applies when CubeMX enables a new HAL module. For example, USART1 needed `stm32f4xx_hal_uart.c` added by hand to `Debug/Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk` and `objects.list` before the CLI build would link.
- `Drivers/` and `Middlewares/` are vendor code copied in by CubeMX. Don't modify them.
- Put new `#include`s in `USER CODE BEGIN Includes`. That block currently has `<math.h>` and `"usbd_cdc_if.h"`; `<stdio.h>`/`<string.h>` come in indirectly via `usb_device.h` → `usbd_def.h` → `usbd_conf.h`. The Debug build has zero warnings, so keep it that way.

## Runtime architecture

Clocks: 25 MHz HSE → PLL → 60 MHz SYSCLK. APB1 runs at 30 MHz, and all timer clocks run at 60 MHz. USB takes 48 MHz from PLLQ.

| Function | Peripheral | Pins | Notes |
|---|---|---|---|
| IR wall sensors ×4 | ADC1 IN4–IN7 → DMA2_Stream0 (circular) | PA4–PA7 | Hardware-triggered by TIM4 CC4 at ~5 kHz. `adc_value[0..3]` = front-left, left, right, front-right |
| IR emitters | GPIO | PA8, PA9, PA10 (4th unknown) | Currently just switched on at boot |
| Left motor | IN/IN driver (DRV8833-style). Both inputs are PWM on the same timer: IN1 = TIM1 CH2N, IN2 = TIM1 CH3N (complementary-only outputs, which are non-inverted) | IN1 PB0, IN2 PB1 | 20 kHz, ARR 2999, CCR preload on |
| Right motor | Same, IN1 = TIM5 CH4, IN2 = TIM5 CH3 | IN1 PA3, IN2 PA2 | 20 kHz, ARR 2999, CCR preload on |
| Left encoder | TIM3 encoder mode (TI1) | PB4/PB5 | |
| Right encoder | TIM2 encoder mode (TI1) | PA0/PA1 | ARR 65535, so it wraps as 16-bit even though TIM2 is 32-bit |
| IMU (MPU6500) | I2C1 at 300 kHz, RX on DMA1_Stream0 | PB6/PB7 | 7-bit address 0x68. The macro is named `MPU6050_ADDR` |
| Control tick | TIM9 update IRQ | — | ~500 Hz (2 ms) |
| Telemetry | USB OTG FS, CDC virtual COM port | PA11/PA12 | Sent with `CDC_Print()` |
| Bluetooth (HC-05) | USART1, currently **9600** 8N1 to match the HC-05 default; no DMA/IRQ | TX PA15, RX PB3 (AF7) | `UART_Print()` (blocking; its timeout scales with length/baud, because a fixed 100 ms cut lines at 9600) and `UART_WaitStart()` (waits for any received byte) in `main.c`. At 9600 a log line takes about 60 ms, so `LOG_UART_PERIOD_MS` = 20 is effectively 60. Switching the HC-05 to 115200 (`AT+UART`) and CubeMX to match restores 20 ms. PA9/PA10 stay IR emitters |
| MPU status LED | GPIO | PB15 | Driven 0 when `MPU6500_Setup()` returns HAL_OK; 1 on init failure or a disturbed calibration (HAL_BUSY) |

**Control-loop data flow (interrupt-driven; `while(1)` is empty):**
1. Every 2 ms, TIM9 fires `HAL_TIM_PeriodElapsedCallback` in `main.c`.
2. It calls `Motor_Speed_Tick()` (`pid.c`). That function reads `TIM3->CNT` and `TIM2->CNT` and computes wrap-safe deltas (`(int16_t)(uint16_t curr - prev)`).
3. It then calls `Motor_Speed_Control(delta_left, delta_right)`, the velocity loop:
   - It converts the deltas to mm/s and low-pass filters them (`speed_*_mms`).
   - It integrates the **unfiltered** speed × dt into `dist_left_mm`/`dist_right_mm`, which equals counts × `MM_PER_COUNT` with no filter lag. It also dead-reckons `odo_x_mm`/`odo_y_mm` from the centre distance and `Robot_Heading()`.
   - It calls `Motion_Update()`, the outer loop. When a motion is active, this writes `target_speed_*` (see "Outer motion loop" below).
   - It ramps `ref_speed_*` toward `target_speed_*` at no more than `MAX_ACCEL_MMS2`.
   - It runs one `PID_TypeDef` per wheel against `ref_speed_*` (mm/s). The output is the feedforward term `Ks·sign(ref) + Kf·ref + Ka·dref/dt` plus the PID terms. `dref/dt` is taken from the ramp, and `prev_setpoint` is reset to 0 in `PID_Reset`. The integrator only accumulates while the output isn't saturated in the direction the error pushes (conditional anti-windup).
   - It writes the outputs through `Motor_SetPWM(left, right)`, which takes a signed duty of ±`PWM_MAX`.
4. Next the callback calls `MPU6500_Tick()` (`mpu6500.c`). Once `imu_ready` is set, that starts `HAL_I2C_Mem_Read_DMA`, which reads 14 bytes from register 0x3B into the static `imu_rx_buffer`. The data comes back through DMA1_Stream0 IRQ → `HAL_DMA_IRQHandler` → `I2C_DMAXferCplt`.
5. When the read finishes, `HAL_I2C_MemRxCpltCallback` calls `MPU6500_Process_Data(dt)`, where `dt = imu_ticks × CONTROL_DT` (see the IMU gotchas). That function does three things:
   - It updates pitch and roll with a complementary filter whose time constant is `ANGLE_COMP_TAU_S` = 0.5 s; α is computed from the real `dt`.
   - It detects standstill (`imu_still`): both filtered wheel speeds from `pid.c` below `STILL_SPEED_MMS`, and |Gz| below `STILL_GZ_DPS`, held for `STILL_HOLD_S`.
     - `STILL_GZ_DPS` is 5, not 1. It has to exceed any possible offset error, or tracking can never start.
     - This happened on hardware: a boot calibration done while the robot was being set down was off by 2.68 °/s. With threshold 1 the robot never counted as still and never self-corrected. The heading PID then held the wrong heading, and the robot turned about 12° per metre.
     - With the wheels still, the robot can't rotate on the floor, so the wider threshold is safe.
   - It updates yaw by integrating the gyro alone. While still, yaw is frozen and `gyro_z_offset` low-pass tracks the raw Gz (`GYRO_BIAS_TAU_S` = 1 s). While moving, every sample is integrated with **no deadband**.

   The outer motion loop runs in the TIM9 path (step 3), not in this callback, so it keeps running even if I2C reads fail. It uses yaw from the previous sample, which is at most about 2.6 ms old.

Application code sets speeds through two functions:
- `Motor_SetSpeed(left_mms, right_mms)` clamps the targets to ±`MAX_SPEED_MMS` and turns the loop on. The wheels speed up or slow down gradually to the new value, and `(0, 0)` brings them to a smooth stop.
- `Motor_Stop()` turns the PID off and brakes both motors immediately.

Both cancel any active motion: they set `motion_mode` to IDLE **before** touching the targets, so the ISR can't overwrite them afterwards. Don't write `target_speed_*` or `motor_pid_enable` directly; only `Motion_Update` inside `pid.c` does.

**Outer motion loop.** `Move_Straight(mm)`, `Turn_Angle(deg)` (positive is left), `Turn_Left` and `Turn_Right` block the main thread until the robot has arrived and stood still for `MOTION_SETTLE_MS`. They return 0 on any of these:
- abort: heading error above `MOTION_ABORT_DEG`, which usually means a wrong `YAW_SIGN` or a collision;
- timeout after `MOTION_TIMEOUT_MS` plus twice the segment's time at maximum speed, so long distances aren't cut off;
- motors locked or IMU not ready.

How it works:
- The heading target is **absolute**: it starts at `Motion_Begin()` and each `Turn_Angle` adds to it. A turn that ends 0.5° off is corrected by the next straight's heading hold, so errors don't accumulate around a path.
- **Straight:** v = min(`MOVE_SPEED_MMS`, √(2·`MOVE_DECEL_MMS2`·remaining)). Heading hold is a PID that applies ±ω·`TRACK_WIDTH_MM`/2 to the two wheels, where ω = `HEADING_KP`·err + `HEADING_KI`·∫err − `HEADING_KD`·`yaw_rate_dps`, clamped.
  - The integral is clamped to `HEADING_I_MAX_DPS` and reset at each `Motion_Start`.
  - The D term uses the gyro rate directly, with no numeric differentiation.
  - Simulated with the left wheel 1% smaller: P-only left 0.41° of steady error and 3.7 mm/m of lateral drift; the PID cuts this to 0.09° and 0.8 mm.
  - Gains are runtime variables `heading_kp/ki/kd`, initialized from `HEADING_KP/KI/KD` = 12/6/0.4. History: 6/8/0.1, then 10/15/0.2.
  - The step test (simulated) showed Ki = 15 overshoots 13–15% on ±5° steps with about 0.8 s settling. 12/6/0.4 gives at most 7% overshoot, about 0.25 s settling, and still holds against a 1% wheel mismatch.
  - `Heading_PID_Test` can set the gains over Bluetooth without reflashing.
  - Straight mode adds `heading_step_deg` (0 except during that test) to the target.
- **Turn in place:** the same √ profile in degrees, using `TURN_SPEED_DPS` and `TURN_DECEL_DPS2`.
- Once within `MOVE_TOL_MM` or `TURN_TOL_DEG`, the targets are exactly 0, so `Wheel_PID` brakes.
- `Robot_Heading()` = `YAW_SCALE · yaw_angle`, and is not wrapped. Yaw direction is fixed at the source: `YAW_SIGN` in `mpu6500.h` is −1 on this robot, which was checked on hardware because the gyro Z was reversed. `yaw_angle` itself increases on a left (CCW) turn, so prints and tests agree with the motion code. `YAW_SCALE` calibrates gyro sensitivity (±3% tolerance per the datasheet): new = old × (true total rotation / commanded).
- `TRACK_WIDTH_MM` is 76.5 mm, measured on the robot. It only scales turn speed and correction gain; the final angles come from the gyro.

`Rectangle_Test(length, width)` drives 4 sides with a right turn after each, then `CDC_WaitHost()` and prints a per-segment report: target vs achieved, final heading error, max deviation or overshoot, time, and result. It ends with the odometry end position.

The blocking calls are split into `Motion_Start` and `Motion_Wait(timeout, hook)`. The hook runs repeatedly in the main thread while the ISR drives the robot.

`Straight_Log_UART(dist_mm)` waits for any byte over USART1, then drives straight while printing `LOG,` CSV lines over Bluetooth every `LOG_UART_PERIOD_MS` (20 ms) through that hook. Each line has:
- t and distance;
- wheel speeds;
- yaw from the gyro vs `enc_heading_deg` (heading from encoders only, (dR−dL)/W);
- the gyro vs encoder turn rate, both averaged over the same interval, because an instantaneous encoder rate is far too quantized;
- heading error and PID output.

The summary flags left/right wheel diameter mismatch: the encoder heading drifts while the gyro holds straight.

`Heading_PID_Test(dist_mm, print_raw)` tests the heading PID.
- **Input:** it reads a UART line (`UART_ReadLine`, which skips empty lines). "Kp Ki Kd" sets new gains, parsed with `strtof` because newlib-nano has no `_scanf_float`; any other text keeps the current gains.
- **Run:** it re-zeros the gyro, then drives straight (≥ about 1050 mm). A main-thread hook steps the heading target +5°, 0, −5°, 0 (`HP_*`, starting at 0.5 s, every 1 s).
- **Recording:** a `HeadRec` (10 bytes) every 2 ms, holding ref, gyro heading, encoder heading, rate and PID output.
- **Report:** a baseline hold RMS, then per step: rise time to 90%, overshoot %, settle into ±0.5°, steady error and RMS, w_max; then gyro vs encoder rotation, and tuning hints. `HP,` raw lines every 10 ms when `print_raw` is set.
- **Memory:** its buffer shares a `rec_buf` union with `Straight_IMU_Test` (54 KB).

`Straight_IMU_Test(dist_mm, dump_raw)` is the vibration and interference test. It runs after a UART key press and nothing is sent while the robot drives.
- **Recording:** `Motor_Speed_Control` stores one 18-byte `ImuRec` every 2 ms into a 54 KB RAM buffer (`IMU_REC_MAX` = 3000, always allocated). Each record holds:
  - the latest raw gyro/accel from `imu_raw[]`;
  - a fresh-sample flag via `imu_sample_count`;
  - encoder deltas;
  - the heading PID output.
- **Phases:**
  1. 0.5 s still and silent;
  2. 0.5 s still while spamming UART, which checks for UART/Bluetooth coupling into the encoders or IMU;
  3. the straight run;
  4. 0.3 s after the stop.
- **Report:**
  - per-phase std of every axis, plus a "cruise" row (the middle 60% of the run);
  - encoder pulses while still (must be 0);
  - Goertzel top-3 peaks from 0.5 to 100 Hz for Gz, Gx, Gy, Az and Ax, next to the expected wheel frequency;
  - `RAW,` CSV lines when `dump_raw` is set (about 2 minutes at 9600).
- **Interrupt timing:** `Motor_Speed_Tick` measures its own period and execution time with the DWT cycle counter (`TickStat`, enabled in `Motor_Init`). This test and `Straight_Log_UART` both print it, which shows whether main-thread UART printing delays the control loop. It shouldn't: TIM9 is priority 0 and preempts the blocking `HAL_UART_Transmit`, and the FPU context is stacked automatically.
- **Encoder input filter is off** (`IC1Filter`/`IC2Filter` = 0 on TIM2/TIM3), and PB3 (UART RX) is the pin next to PB4/PB5 (left encoder). If the test shows pulses while still with UART active, set the filter to 15 in CubeMX. Encoder edges are at least about 300 µs apart, so the roughly 4 µs filter is safe.

`Straight_Test(dist_mm)` is the single-straight version (negative reverses). It reports:
- centre and per-wheel encoder distance;
- final and maximum heading error;
- odometry along/lateral offset in the start frame;
- the `WHEEL_DIAMETER_L`/`_R` correction formula. It is the right test for calibrating wheel diameter.

A full-robot host simulation was used to check this. It isn't in the repo, but it can be rebuilt: compile the real `pid.c` and `mpu6500.c` against a mock `main.h` that provides TIM/GPIO structs, plus the motor model, encoders, and a gyro with DLPF lag and noise. Results:
- 300×200 closes to about 0.5 mm, with turns about 0.5° over.
- It is robust to ±20% `TRACK_WIDTH_MM` error.
- A wrong gyro sign aborts after 30°.
- A 2% gyro gain error gives a 23 mm true error with no self-visible error; one `YAW_SCALE` correction brings it to 1 mm.

`Motor_SetPWM` drives an IN1/IN2 bridge with slow decay (drive/brake) in both directions. On this board (IN1, IN2) = (0,1) drives forward, (1,0) reverse and (1,1) brakes:
- Duty ≥ 0: IN2 is held at 100% and IN1 is low for the duty fraction.
- Duty < 0: IN1 is held at 100% and IN2 is low for the duty fraction.
- Duty 0 brakes.
- The 100% pin is written first, so the intermediate state during a direction change is always brake.

The old scheme, where IN1 was a GPIO and the off phase coasted when driving forward, was measured with `PID_Test`. Forward then needed 1000–2000 PWM just to start the wheel and about twice the PWM of reverse for the same speed. Don't go back to it.

`motor_hw_ok` comes from `Motor_PinsConfigured()`, which checks that PB0/PA3 are in timer AF mode. If they aren't, `Motor_Write` forces both CCRs to 0 (coast) and `PID_Test` refuses to run. This matters because with the pins still configured as GPIO, IN1 stays low while IN2 goes to 100%, which drives the robot forward at full speed.

To fix the wheel direction, flip `MOTOR_*_SIGN`; to fix the encoder direction, flip `ENC_*_SIGN`. Both live in `pid.h`. Positive duty must produce positive measured speed, or the loop runs away. When `motor_pid_enable` is 0, the loop resets the PIDs and leaves the outputs alone, so `Motor_SetPWM()` can be called directly from `main` for open-loop tests.

The `print_LED()`, `print_mpu6500()` and `print_SpeedPID()` helpers are debug output, meant to be called by hand from the main loop. `print_SpeedPID()` only manages about 50 lines/s, which is too slow to show a 2 ms loop's dynamics.

To tune gains, use `PID_Test(print_raw)` instead. It works like this:
- It runs the speed sequence in `pid_test_speeds[]` (0→200→300→150→−150→0 mm/s), holding each step for `PID_TEST_STAGE_LEN` × 2 ms.
- Meanwhile the TIM9 ISR records one sample into `pid_log[]` every 2 ms.
- After the run, `PID_Test_Report()` prints one line per step and wheel with:
  - rise time to 90%
  - overshoot %
  - settling time into ±`PID_SETTLE_BAND`
  - mean error over the last quarter of the step
  - maximum ref-vs-measured lag
  - peak |PWM|, flagged `BAO HOA` (saturated) if it hits `PWM_MAX`
- `print_raw = 1` also dumps `STEP,t_ms,refL,measL,pwmL,refR,measR,pwmR` lines.

`MPU6500_Test(temp_seconds)` (mpu6500.c, config `MPU_TEST_*` in `mpu6500.h`) measures IMU noise under three conditions. Wheels must be lifted, and the robot must be still from power-on.
- How it measures: each condition is 1000 samples. `HAL_I2C_MemRxCpltCallback` accumulates int64 sum/sumsq/min/max per raw channel while `imu_stat_active` is set; the main thread computes mean and std afterwards. No sample buffer is needed.
- The three conditions:
  1. PWM off (brake, no switching edges);
  2. open-loop PWM `MPU_TEST_PWM_LOW` = 150, which isolates EMI. At 300 the lifted wheels already turned at 30–50 mm/s;
  3. wheels at `MPU_TEST_SPEED` under PID, which adds vibration.
- Per condition it reports:
  - temperature;
  - residual Gz bias (`GzTB`, the mean minus the offset in use at the end of the measurement);
  - gyro and accel noise;
  - yaw drift per minute;
  - % of samples flagged still (`DYen%`; conditions 1–2 should be ~100, condition 3 should be 0);
  - received vs expected samples;
  - I2C errors plus rejected DMA reads.
- It then logs 1-second averages for `temp_seconds` (≤ `MPU_TEMP_LOG_MAX` = 600) and least-squares fits the gyro bias against temperature in °/s per °C. In that log, `gz` is relative to the start-up offset (`gyro_z_cal`) and `gz_off` is the tracked offset, which should follow it.
- Temperature uses the MPU6500 formula, raw/333.87 + 21. The header prints `WHO_AM_I`. 0x70 is MPU6500. 0x71 is an MPU9250 die, which the user's board actually reports; it has the same gyro, accel and temperature registers, so it is accepted.
- Like `PID_Test`, it runs first and then `CDC_WaitHost()` before printing.
- A host harness (`sim.c` with a mock `main.h`) drives `MPU6500_Tick`, the real callback and the blocking calibration reads against a simulated chip with known noise, bias shift and temperature slope. It reproduced the old deadband drift (1.5 °/min, matching hardware) and verified bias tracking, the 90° turn and the temperature coefficient.

Measured on hardware with the old filters (6 runs). Gyro DLPF 41 Hz, accel 41 Hz; Gz noise in °/s:

| Condition | Gz noise | Other |
|---|---|---|
| PWM off | 0.06 | — |
| PWM switching | 0.07–0.1 | No I2C errors, so EMI is small |
| Wheels at 200 mm/s | 0.36–0.42 | Mechanical vibration; accel 8–18 mg |

- The start-up offset was consistently off by +0.08–0.10 °/s.
- The 0.8 °/s deadband turned vibration plus bias into about 1.4 °/min of yaw drift.
- Temperature rose only about 1.2 °C over 6 minutes; the Gz bias moved about 0.01 °/s per °C.

Those numbers drove the current design: the deadband was replaced by standstill bias tracking, the accel DLPF was lowered, and the gyro DLPF was kept at 41 Hz.

The PID and report functions are self-contained. They can be compiled on the host against a simulated motor (MSYS2 gcc is at `/c/msys64/ucrt64/bin/gcc`):
- pull the typedefs out of `pid.h` with `sed`;
- pull `PID_Compute`, `PID_Reset`, `Wheel_PID`, `Ramp` and `PID_Test_Report` out of `pid.c` with `sed`.

Measured tuning data (30 mm wheels):
- Steady-state PWM in drive/brake mode, from 6 `PID_Test` runs:
  - left: about 832 @ 155, 1058 @ 200 and 1523 @ 297 mm/s
  - right: about 852 / 1070 / 1555 at the same speeds
  - reverse is symmetric
  - this is linear from 150 to 300 mm/s, giving `Ks` = 80/85 and `Kf` = 4.85/4.95
  - from rest, a wheel needs about 450–500 PWM before it starts turning (stiction)
- The motor time constant is about 35 ms. This was found by fitting the host simulation to the measured report, which it reproduces within about 10%. That gives `Ka` = Kf × 0.035 ≈ 0.17.
- The simulation (`pid_sil.c`, which pulls the real PID code out of `pid.c` and runs it against a motor model with stiction, the kinetic map above and τ) predicts:
  - ramp lag `Bam_max` drops from 60–100 to about 25–35 mm/s
  - settling to ±20 mm/s takes about 50–70 ms for the 200↔300 steps
  - this holds for τ between 25 and 50 ms
  - Real runs with `Ka` confirmed this: t_len 48–94 ms, `Bam_max` 20–31 mm/s, step errors within ±8 mm/s, repeatable across 4 runs.
- The report overstates two figures. "Vot_lo" of 11–17% and the scattered 0→200 `t_od` are encoder quantization, not control error. One extra count in a tick moves the filtered speed by 0.3 × 67 ≈ 20 mm/s, which is about the same as `PID_SETTLE_BAND` (20). These spikes (e.g. 222 at 200 mm/s, 317 at 300 mm/s) keep recurring long after the speed has settled.
- Raising Kp/Ki to 3/60 gave mixed results in the simulation, so they stay at 1.5/30 until real data says otherwise.
- `MAX_SPEED_MMS` = 350 is a leftover limit from the old forward drive. The linear map suggests about 450–500 mm/s with headroom left for Ka and the PID, but raise it only after testing at around 400.
- The ±12 mm/s jitter in `speed_*_mms` is encoder quantization (4–5 counts per tick, 67 mm/s per count), not real speed variation. Raising Kp mostly amplifies this jitter.

## Gotchas

- **NVIC priorities:** every peripheral IRQ is at priority 0 and SysTick is at 15. A `HAL_Delay()` or a HAL tick-based timeout inside any ISR or HAL callback therefore hangs. The following all rely on `HAL_GetTick`/`HAL_Delay`, so call them only from thread context:
  - `CDC_Print()` (20 ms timeout)
  - `CDC_WaitHost()`
  - the `print_*()` helpers

  The TIM9 and I2C callbacks must stay non-blocking: no `CDC_Print`, no blocking HAL I2C calls.
- **IMU timing:** these three must stay consistent:
  - TIM9 ticks every 2 ms and requests one IMU DMA read per tick.
  - The MPU sample rate is 500 Hz (`SMPLRT_DIV = 1`).
  - `HAL_I2C_MemRxCpltCallback` integrates with `dt = imu_ticks × CONTROL_DT`. `imu_ticks` counts the TIM9 ticks since the last processed sample, so a skipped or failed I2C read doesn't lose time.

  `dt` used to be a hard-coded `0.005f`, which made yaw read 2.5× too much.
- **MPU6500 filters** (macros in `mpu6500.h`):
  - Unlike the MPU6050, `CONFIG` (0x1A, `GYRO_DLPF_CFG` 3 ≈ 41 Hz, 5.9 ms delay) filters **only the gyro**.
  - The accelerometer has its own filter in `ACCEL_CONFIG2` (0x1D, `ACCEL_DLPF_CFG` 4 ≈ 20 Hz). Its default of roughly 460 Hz aliases motor vibration at the 500 Hz read rate.
  - The gyro stays at 41 Hz on purpose. Vibration noise is zero-mean and integrates away, while 20 Hz would nearly double the delay for a future rotation loop.
- **Gyro re-zero before every motion sequence:** `Motion_Begin()` calls `MPU6500_ZeroGyroZ()`. It averages 250 DMA samples of raw Gz (0.5 s) while the wheels are still and sets `gyro_z_offset` directly. If the wheels move or the noise exceeds `GYRO_CAL_MAX_STD_DPS`, it returns HAL_BUSY and keeps the old offset. Reports print the correction (`gyro_z_rezero_dps`). Callers must have the robot standing still; every test waits 1 s first.
- **Yaw has no deadband:** don't reintroduce one. It ate small real heading changes and rectified vibration into drift. Standstill handling depends on the encoder speeds (`speed_*_mms` from `pid.c`). If encoder jitter keeps `imu_still` at 0 (visible as a low `DYen%` in `MPU6500_Test`), bias tracking simply never runs.
- **IMU start-up order:** the ISR only starts DMA reads once `imu_ready` is set. That happens after `MPU6500_Init()` and the blocking `MPU6500_Calibrate()`, and only if init succeeded. Before this was added, the DMA reads collided with the blocking calibration reads on `hi2c1`. Don't add blocking I2C calls after `imu_ready` = 1.
  - Calibration waits 50 ms, discards `GYRO_CAL_DISCARD` samples, averages 500, and retries up to `GYRO_CAL_RETRY` times if any axis std exceeds `GYRO_CAL_MAX_STD_DPS`. The robot must be still; this takes about 1.3 s.
  - `MPU6500_Setup` returns HAL_BUSY when calibration never got a still reading (PB15 LED on). The IMU still runs, and the Z offset corrects itself at the next standstill.
- **Wheel geometry:** distance comes from the per-wheel *effective* diameters `WHEEL_DIAMETER_L`/`_R` and `ENCODER_CPR` (700 counts per wheel revolution), which give `MM_PER_COUNT_L`/`_R`. `WHEEL_DIAMETER` and `MM_PER_COUNT` are the averages and are only used for printing.
  - Values measured on 2026-10-09: 29.24/29.03 mm. The nominal 30 mm tires compress under the robot: 1 m by encoder was really about 970 mm. With the gyro holding heading, the encoders showed the right wheel about 0.72% smaller.
  - `Straight_Log_UART` prints suggested L/R values: (encoder heading − gyro heading)·W / distance, split evenly so the mean is kept. The mean needs a tape-measured distance.
  - `Kf`/`Ka` were rescaled by 30/D per wheel so the feedforward PWM is unchanged.
  - When the wheels physically change, also scale the PID gains and `MAX_SPEED_MMS` by old/new diameter, because the same PWM now produces a different speed in mm/s.
- **Hardware `Straight_IMU_Test` results (2026-10-09):**
  - UART/Bluetooth has no effect: 0 encoder pulses while transmitting, IMU noise ×0.9–1.1, TIM9 period 1995.9–2000.2 µs, `Motor_Speed_Tick` at most 38 µs (1.9% CPU).
  - Driving at 250 mm/s versus standing still, std ratios: Gz ×95–100 (about 6 °/s), Gx ×40, Gy ×155 (8.6 °/s, the robot pitches), Ax 51 mg, Az 22–28 mg.
  - Gz peaks at 2.75/5.5/8 Hz (1/2/3× wheel revolution) and 27.5 Hz (10× wheel, probably the motor or gear if the ratio is 10:1).
  - Rectification is negligible: the mean Gz while driving matches the mean at rest.
- **Robot shake at about 2.7 Hz (one wheel revolution at 250 mm/s) is mechanical.** The gyro shows a yaw spike once per revolution that the encoders don't. In simulation, 4× the heading gains only reduced it by about 10%, so check the wheels (run-out, tire seating, loose hub) rather than retuning.
- **Speed variables:** the only wheel speeds are `speed_*_mms` (mm/s, filtered, in `pid.c`). The old `speed_*_ms`/`print_Velocity()` and the stale `extern`s in `stm32f4xx_it.c` are gone. `stm32f4xx_it.c` contains only the CubeMX handlers that call into HAL.
- **USB printing must never block:** the robot runs on battery without a PC.
  - `CDC_Print()` returns immediately when USB isn't enumerated. It returns after at most 20 ms when the host isn't reading (COM port closed), then drops lines instantly until the host reads again.
  - Before this fix it retried forever. `CDC_Transmit_FS` also dereferences `pClassData` without a NULL check, so when unplugged it read garbage and returned BUSY. The result was that the robot froze at the first print (e.g. after one `PID_Test`).
  - `PID_Test` deliberately calls `CDC_WaitHost()` before dumping its report. That blocks, with the motors braked, until a terminal actually opens the port, so a floor run without a cable can be read out afterwards.
  - For anything else, never call `CDC_Transmit_FS` directly; use `CDC_Print`.
- **ADC start order:** `ADC_IR_Start()` starts the ADC DMA *before* the TIM4 trigger. In the other order, a trigger can land after ADON but before DMA is enabled. That causes an overrun (OVR), and since the ADC IRQ isn't enabled, DMA requests stay blocked and `adc_value[]` freezes. `adc_value` is sized `ADC_IR_CHANNELS` (4), which must match the CubeMX `NbrOfConversion`.
