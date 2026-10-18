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
| `adc.c` / `adc.h` | IR ADC+DMA start, emitters, `adc_value[]`, front and side sensor equalization (`IR_FL_ToRef`, `IR_L_ToRef`), filtered values for control (`IR_Tick`, `IR_*_Filt`) |
| `solving_maze.c` / `solving_maze.h` | On-robot flood-fill maze solver with continuous straights (see "Maze solver"). Fully non-blocking; driven by `mode_manager.c` |
| `flash_storage.c` / `flash_storage.h` | Maze map persistence in Flash sector 7 (`FlashMapData`, append log + CRC32, see "Flash map storage") |
| `mode_manager.c` / `mode_manager.h` | Mode Manager + Safety (see "Mode Manager"). 3 modes since 2026-10-15. The IR-left "virtual button" with the PB8 buzzer, MODE 1 (explore) / MODE 2 (fast run + return home), and pause/resume when the robot is picked up. `while(1)` only calls `ModeManager_Update()` |

`Simulator/mms-c/` is **host-side C, not firmware**. It is a vendored copy of github.com/mackorone/mms-c at commit `0342718` (2019-09-21), with its `.git` removed. It is the algorithm template for the mms Micromouse simulator.
- `API.c/h` (`API_wallFront()`, `API_moveForward()`, which returns 0 on crash, and so on) speaks the stdin/stdout protocol, so log through stderr only. The logger is named `log_msg` because `log` clashes with the C builtin.
- It replaced the earlier `Simulator/mms-cpp` on 2026-10-10. `Main.c` is a direct port of that `Main.cpp`; on 40 competition mazes, 10 of them with a mid-run reset, a harness found the two versions send identical command streams and logs.
- The IDE source entries are only Core/Drivers/Middlewares/USB_DEVICE, so it is never compiled into the firmware.
- Build: `gcc -O2 -static API.c Main.c -o a.exe`. Its `.gitignore` excludes `*.exe`.
- In mms on Windows, Run Command must be the **full path** to `a.exe`, because QProcess doesn't resolve relative to the algorithm directory. Watch for a pasted trailing newline, which caused "cannot find the file specified". mms keeps its config in `HKCU\Software\mackorone\mms\mouseAlgos`; the user's mms.exe is a prebuilt Qt6 build.

`Main.c` holds a **basic flood fill**. It explores start → centre and centre → start with unknown walls treated as open, re-flooding after every cell and showing distances with `setText`.
- **Exploration rounds:** after each return, it compares two shortest paths from start: one over visited cells only, one optimistic (unknown cells treated as open). If the visited-only path is longer, it does another exploration round, up to `MAX_ROUNDS` = 5. Once they are equal, the path is provably optimal.
- **Final run:** start → centre over visited cells only (`knownOnly`).
- **Reset:** keeps the wall map and restarts exploration.
- **Tested** with Python harnesses emulating the mms protocol:
  - 20 random mazes;
  - 60 real competition mazes from `Simulator/mazefiles/classic`, which is micromouseonline/mazefiles, gitignored.
  - Result: no crashes, and the final run equalled the true shortest path in 60/60, averaging 1.9 rounds (max 4).
- **Build** for mms with `gcc -O2 -static API.c Main.c -o a.exe`. Use `-static` so mms can run it without the MSYS2 DLLs.

`Simulator/mms/` (the simulator source, Qt6) is gitignored. Its MSYS2 build was attempted, but `qmake6` failed with "failed to parse default include paths" and the build was stopped at the user's request. The IDE source entries are only Core/Drivers/Middlewares/USB_DEVICE, so it is never compiled into the firmware. Build it with MSYS2 g++: `g++ API.cpp Main.cpp -o a.exe`; its `.gitignore` excludes `*.exe`.

Sections marked "(chưa có code)" are planned and still empty: IR → mm conversion and wall detection in `adc`. The wall logic in `pid.c` (see "Walls") works directly on equalized IR values. Each module has its own `static char msg[200]` for printing; there is no shared global `msg`. Do **not** enable CubeMX's "Generate peripheral initialization as a pair of .c/.h files per peripheral", because it would generate its own `Core/Src/adc.c` and clobber this one.

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

Flash over ST-Link SWD. Sector 7 holds the saved maze map and is outside the image, so a normal flash keeps it; a full chip erase wipes it:

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
  - `WHILE`: `ModeManager_Init()`, then `while(1) { ModeManager_Update(); }`. Never add `HAL_Delay` there.
  - `4`: the TIM9 callback calls `Motor_Speed_Tick()`, `MPU6500_Tick()`, then `Check_Robot_Picked_Up()`.
  - `PFP`: also `UART_TryRead()`, a non-blocking single-byte USART1 read that clears RXNE/ORE.
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
| Bluetooth (HC-05) | USART1, currently **9600** 8N1 to match the HC-05 default; no DMA/IRQ | TX PA15, RX PB3 (AF7) | `UART_Print()` (blocking; its timeout scales with length/baud, because a fixed 100 ms cut lines at 9600) and `UART_WaitStart()` (waits for any received byte) in `main.c`. At 9600 a ~90-character `print_mpu6500` line takes about 95 ms. Switching the HC-05 to 115200 (`AT+UART`) and CubeMX to match makes it 12× faster. PA9/PA10 stay IR emitters |
| MPU status LED | GPIO | PB14 | `main.c` (user code) writes 1 when `MPU6500_Setup()` returns HAL_OK, 0 otherwise |
| Buzzer | GPIO | PB8 | Configured in CubeMX (`GPIO_Output`). `BUZZER_ON`/`BUZZER_OFF` in `mode_manager.h`, assumed active-high and an *active* buzzer (DC on = sound) |
| Run LED | GPIO | PB15 | On while the robot sits on the floor and may move; off while it is lifted (see "Lift detection"). The polarity is `LED_RUN_ON`/`LED_RUN_OFF` in `mpu6500.h`, assumed active-high like PB14 |

**Control-loop data flow (interrupt-driven; `while(1)` only runs `ModeManager_Update()`):**
1. Every 2 ms, TIM9 fires `HAL_TIM_PeriodElapsedCallback` in `main.c`.
2. It calls `Motor_Speed_Tick()` (`pid.c`). That function reads `TIM3->CNT` and `TIM2->CNT` and computes wrap-safe deltas (`(int16_t)(uint16_t curr - prev)`).
3. It then calls `Motor_Speed_Control(delta_left, delta_right)`, the velocity loop:
   - It converts the deltas to mm/s and low-pass filters them (`speed_*_mms`).
   - It integrates the **unfiltered** speed × dt into `dist_left_mm`/`dist_right_mm`, which equals counts × `MM_PER_COUNT_L`/`_R` with no filter lag. It also dead-reckons `odo_x_mm`/`odo_y_mm` from the centre distance and `Robot_Heading()`.
   - It calls `IR_Tick()` (`adc.c`), which filters the four IR channels for the wall logic.
   - It calls `Motion_Update()`, the outer loop. When a motion is active, this writes `target_speed_*` (see "Outer motion loop" below).
   - It ramps `ref_speed_*` toward `target_speed_*` at no more than `MAX_ACCEL_MMS2`.
   - It runs one `PID_TypeDef` per wheel against `ref_speed_*` (mm/s). The output is the feedforward term `Ks·sign(ref) + Kf·ref + Ka·dref/dt` plus the PID terms. `dref/dt` is taken from the ramp, and `prev_setpoint` is reset to 0 in `PID_Reset`. The integrator only accumulates while the output isn't saturated in the direction the error pushes (conditional anti-windup).
   - It writes the outputs through `Motor_SetPWM(left, right)`, which takes a signed duty of ±`PWM_MAX`.
4. Next the callback calls `MPU6500_Tick()` (`mpu6500.c`). Once `imu_ready` is set, that starts `HAL_I2C_Mem_Read_DMA`, which reads 14 bytes from register 0x3B into the static `imu_rx_buffer`. The data comes back through DMA1_Stream0 IRQ → `HAL_DMA_IRQHandler` → `I2C_DMAXferCplt`.
5. When the read finishes, `HAL_I2C_MemRxCpltCallback` calls `MPU6500_Process_Data(dt)`, where `dt = imu_ticks × CONTROL_DT` (see the IMU gotchas). That function does three things:
   - It updates pitch and roll with a complementary filter whose time constant is `ANGLE_COMP_TAU_S` = 0.5 s; α is computed from the real `dt`.
   - It detects standstill (`imu_still`). Three conditions must hold continuously:
     - both filtered wheel speeds below `STILL_SPEED_MMS`;
     - Gz is **steady**: `gz_dev`, an EMA of |Gz − 0.1 s mean| with τ 0.25 s, is below `STILL_DEV_DPS` = 0.2 °/s;
     - |Gz| < `STILL_GZ_DPS` (1 °/s) for `STILL_HOLD_S` (0.25 s), or 1–5 °/s (`STILL_GZ_MAX_DPS`) for `STILL_HOLD_LONG_S` (2 s).

     The long tier lets a big offset error self-correct; a boot calibration once came out 2.68 °/s off. The steadiness check is essential. When the robot is rotated by hand with the motors braking, the wheels skid instead of rolling, so the encoders read "still". Without the check, the slow tail of each hand rotation counted as standstill: yaw froze and bias tracking absorbed the rotation. Hardware showed −5 to +2° after rotating back by hand, and simulation showed 22° after 6 asymmetric hand turns, versus 0.4° with the check. Bias tracking τ is `GYRO_BIAS_TAU_S` = 2 s.
   - It updates yaw by integrating the gyro alone. While still, yaw is frozen and `gyro_z_offset` low-pass tracks the raw Gz (`GYRO_BIAS_TAU_S` = 2 s). While moving, every sample is integrated with **no deadband**.

   The outer motion loop runs in the TIM9 path (step 3), not in this callback, so it keeps running even if I2C reads fail. It uses yaw from the previous sample, which is at most about 2.6 ms old.

Application code sets speeds through two functions:
- `Motor_SetSpeed(left_mms, right_mms)` clamps the targets to ±`MAX_SPEED_MMS` and turns the loop on. The wheels speed up or slow down gradually to the new value, and `(0, 0)` brings them to a smooth stop.
- `Motor_Stop()` turns the PID off and brakes both motors immediately.

Both cancel any active motion: they set `motion_mode` to IDLE **before** touching the targets, so the ISR can't overwrite them afterwards. Don't write `target_speed_*` or `motor_pid_enable` directly; only `Motion_Update` inside `pid.c` does.

**Outer motion loop.** `Move_Straight(mm, speed_mms)`, `Turn_Angle(deg)` (positive is left), `Turn_Left` and `Turn_Right` block the main thread until the robot has arrived and stood still for `MOTION_SETTLE_MS`. `speed_mms` is the straight's maximum speed, clamped to `MAX_SPEED_MMS`; `MOVE_SPEED_MMS` (250) is just the default used by `Rectangle_Test` and the examples. They return 0 on any of these:
- abort: heading error above `MOTION_ABORT_DEG`, which usually means a wrong `YAW_SIGN` or a collision;
- timeout after `MOTION_TIMEOUT_MS` plus twice the segment's time at maximum speed, so long distances aren't cut off;
- motors locked or IMU not ready.

A forward `Move_Straight` stopped by a front wall returns **2** once `Wall_Align` succeeds, or 0 with "dung truoc tuong nhung khong can vuong goc duoc" (see "Walls"). Callers that test `if (!ok)` keep working.

How it works:
- The heading target is **absolute**: it starts at `Motion_Begin()` and each `Turn_Angle` adds to it. A turn that ends 0.5° off is corrected by the next straight's heading hold, so errors don't accumulate around a path.
- **Straight:** v = min(`speed_mms`, √(2·`MOVE_DECEL_MMS2`·remaining)). Heading hold is a **PD** (no I, at the user's request) that applies ±ω·`TRACK_WIDTH_MM`/2 to the two wheels, where ω = `HEADING_KP`·err − `HEADING_KD`·rate_f, clamped to `HEADING_MAX_DPS`.
  - rate_f is `yaw_scale`·`yaw_rate_dps` low-pass filtered with `HEADING_D_TAU_S` (20 ms). The filter runs every tick, even when idle, so it is valid at the start of a segment. There is no numeric differentiation.
  - Gains are **20 / 0.5** (2026-10-10). They were chosen with the full-robot sim on the real code, with the motor fitted to the post-fix `PID_Test`:
    - A 5° yaw kick mid-run peaks at 3.1° instead of 4.3° (P 10), and is back within ±0.5° in 150 ms instead of 240 ms, with no overshoot.
    - A left wheel 1% larger gives 1.7 mm/m lateral drift instead of 3.3.
    - RMS PWM change per tick is 51 instead of 49, and 66 with 6 °/s gyro vibration.
    - 30/0.8 and 30/1.2 gain little more and add PWM jitter.
  - History: P-only 10 (2026-10-09), and before that 6/8/0.1, 10/15/0.2 and 12/6/0.4. D was dropped then because the robot jerked on hardware: about 6 °/s of gyro Z vibration, from the left-wheel tight spot that is now fixed, went unfiltered into the wheel targets every 2 ms. The velocity PID's `Ka`·dref/dt then amplified it into PWM spikes (simulated Kd 0.4 → 324, Kd 0 → 18). Keep D on the filtered rate.
  - **The PD only holds what the gyro reports.** Steady error from unequal wheels is (disturbance rate)/KP, about 0.1° for 1% mismatch, so a visible multi-degree drift while the gyro reads straight is a gyro problem, not a gain problem. The user reported about 5° left drift per 1 m in their `while(1)` loop (straight 1 m, `Turn_Angle(180)`, straight 1 m). The sim of that loop with gyro sensitivity 1.4% high gives 2.4° on the return leg and 5° after each full 360° of turning, accumulating, while P/D gains change nothing. Fix with `YAW_SCALE` (`Gyro_Spin_Calibrate`, still `YAW_SCALE` 1.0). Check with `Straight_Test` after a fresh `Motion_Begin`: if that single leg is straight, the drift comes from the turns.
- **Hold heading:** `Move_Straight(any, 0)` **or `Move_Straight(0, any)`** starts `MOTION_HOLD` and returns at once (non-blocking). The ISR holds `heading_target` until another motion command (`Move_Straight`/`Turn_Angle`/`Motor_SetSpeed`/`Motor_Stop`).
  - If HOLD is already running, the call does nothing, so it is safe to call it every iteration of `while(1)`. The user's test loop does exactly that with `Move_Straight(0.0f, 250.0f)`.
  - Distance 0 used to mean "arrived": the robot just braked and did not hold heading. The user reported this as a regression (2026-10-10).
  - HOLD also runs the front-wall logic (back-off, squaring, MPU reset); see "Walls".
  - Hand-rotated, the robot turns back the shortest way: the error is wrapped to ±180° by moving `heading_target` in 360° steps, so later absolute targets stay consistent.
  - There is no abort in this mode.
  - Stiction (about 470 PWM from rest) would make tiny PD corrections hunt. So it starts correcting only above `HOLD_START_DEG` (2°), turns at least `HOLD_MIN_DPS` (40 °/s) while heading back, caps at `HOLD_MAX_DPS` (180), and brakes inside `HOLD_TOL_DEG` (0.7°).
  - Sim with hand turns of 3–270°: back within 2° in 0.02–1 s, final error < 0.25°, no reversals afterwards, and 0 drift standing for 5 s. `imu_still` then engages, so bias tracking keeps running.
- **Turn in place:** the same √ profile in degrees, using `TURN_SPEED_DPS` and `TURN_DECEL_DPS2`.
- Once within `MOVE_TOL_MM` or `TURN_TOL_DEG`, the targets are exactly 0, so `Wheel_PID` brakes.
- `Robot_Heading()` = `yaw_scale · yaw_angle`, and is not wrapped. `yaw_scale` is a runtime variable initialized from `YAW_SCALE`. Yaw direction is fixed at the source: `YAW_SIGN` in `mpu6500.h` is −1 on this robot, which was checked on hardware because the gyro Z was reversed. `yaw_angle` itself increases on a left (CCW) turn, so prints and tests agree with the motion code. `YAW_SCALE` calibrates gyro sensitivity (±3% tolerance per the datasheet): new = old × (true total rotation / commanded).
- `TRACK_WIDTH_MM` is 76.0 mm. It is the distance between the wheel centres, from the user's 2026-10-10 measurement: inner wheel faces 72 mm apart and outer faces 80 mm, so each tire is 4 mm wide. It only scales turn speed and correction gain; the final angles come from the gyro.
- Robot geometry (2026-10-10): length 100 mm, axle midpoint 30 mm from one end (the user didn't say which). The farthest point is about 70 mm from the axle, so an in-place turn sweeps a radius of about 81 mm if the body is 80 mm wide at that end. That leaves about 3 mm against an 84 mm half-corridor (180 mm cell, 12 mm walls).
- Gyro position on the robot doesn't matter for yaw rate, because the robot is a rigid body. A **tilted** Z axis does matter: it reads cos(tilt) × the true rate.
  - At rest the accelerometer reads ax 0.16 / ay 0.15 g. If that is real tilt rather than offset, the tilt is about 10°, which means about 1.5% low and about 5° per 360°. That matches the user's "gyro off 5°" report.
  - To check: rotate the robot by hand while running `print_mpu6500()`. With tilt, gx/gy follow gz at about 15–18% of it; with a flat chip they stay at noise level.
  - Either way `YAW_SCALE` corrects the scale. Remounting the chip flat also removes pitch-to-yaw coupling.

**Front IR equalization** (`adc.c`, 2026-10-10): the two front sensors differ a lot.
- The data: 622 samples from `print_LED` while moving to and from a flat wall with both eyes at the same distance.
  - IRFL ≈ 2.4–2.8 × IRFR (far: about 480 vs 160).
  - IRFL **saturates at about 3850–3950** once IRFR exceeds about 1350–1500 (near the wall; 37% of the samples). IRFR peaked at 2764 unsaturated.
- IRFR is the reference scale. `IR_FL_ToRef()` maps raw IRFL through a 15-point piecewise-linear table, built from per-bin medians of the 384 unsaturated pairs. Below the table it is proportional; at `IR_FL_SAT` (3800) and above it clamps.
- After mapping the error is median 1.5% and 90% < 5.1%, checked by running the same C code on the log; before mapping the median gap was about 160%.
- **A refit from the 2026-10-10 19:20 log was reverted. Don't fit the table to a log without knowing the robot was square.**
  - The user had asked for denser knots at IRFR 1000–1200. A 22-knot refit forced `r` = 0 on that log (it read +0.016 on the old table).
  - The user then reported that the robot was about **5° right** during that log. All 13 approach passes show FL relatively brighter (old-table r +0.012…+0.022 at 1000–1200, +0.018…+0.035 at 700–900, varying between passes). That is consistent with a right (CW) rotation: the FL eye is closer.
  - The refit had baked that 5° into the table, so the session-1 table is back.
  - This gives the robot's real squaring sensitivity: g ≈ 0.016/5 ≈ **0.0032 per degree** at IR about 1100, roughly 4× smaller than the sim geometry assumed (0.014). **A 1% FL/FR mismatch is about 1.5° of squaring error on this robot.**
  - A valid fine table needs a log taken with the robot held truly square, for example sliding along a set square. Simpler: squaring always happens at about `WALL_TARGET_IR`, so calibrating `WALL_R_OFFSET` there (robot truly square, read `r:`) is enough.
- **Current table: session 3 (2026-10-10 20:17 log, 849 lines, user request for dense knots at IRFR 800–1200).**
  - **The IRFR scale changed.** At the same IRFL, IRFR read about 23% higher than in sessions 1–2: k = FR / f_old(FL) was 1.23–1.33 across 860–1730.
    - The side eyes against the front wall suggest it was FR that got brighter rather than FL dimmer. At the same IRL, FL matched session 2 while FR was +24%.
    - The cause (hardware or aim) was not confirmed by the user.
    - The old table therefore gave r ≈ −0.1, IRFL now saturates only at IRFR about 1750–1800, and every `WALL_*` IR threshold now sits about 10% farther from the wall physically.
  - **Two attitudes in the log.** Lines about 345–468 have FR/FL about 3.5% higher than the rest, with IRR higher too, which means the robot was turned more to the left. They were excluded (305 samples kept, from the start, middle and end).
  - Knots:
    - IRFL ≤ 1645 and IRFR 800/850/900: the session-1 table × 1.233, the scale factor at the lower edge, because there was no session-3 data there.
    - IRFR 950–1200: every 50, from session-3 data.
    - Above that, coarse: 1300, 1500 and 3772 → 1802. Above 1500 the attitude and the onset of IRFL compression scatter FR by up to 10% at the same FL. The range above 1200 is only used for back-off.
  - Result at IRFR 800–1200 (kept samples): r mean +0.0003, std 0.006. The excluded block reads about −0.009. `IR_FL_ToRef(≥3800)` = 1822, and the table is monotonic.
- **Session 4 (2026-10-14 15:27 log, 385 lines; user asked again for care at 800–1200). Only the range below 800 was changed.**
  - Two **stationary placements** match the current table: IRFR 1441 (23 samples) at r +0.003 and IRFR 1001 (42 samples) at r −0.001.
  - Every **hand-pushed pass** reads FL about 10% bright at 800–1500 in **both** directions. Approach and retreat agree, which rules out sensor lag and points to tilt. The pass-to-pass ratio also varies (FR/FL 0.385–0.435), and just before the final placement the ratio climbs 0.406 → 0.461 as the robot is squared. **Moving passes can't calibrate 800–1200**; only stationary, square placements can.
  - 800–900 are kept: within each pass the ratio is flat from about 750 to 1200, consistent with the table's ~0.463.
  - Below 800 the old extrapolation was really wrong: FLq was 15–40% above FR, growing with distance, because IRFL has a baseline of about 150 raw.
    - The new 130/175/250/350/450/550/650 knots (IRFL 426/524/672/861/1080/1281/1497) come from the passes. Tilt was removed using r_att = 0.043·√(IRFR/1000), fitted against the placements at 900–1500 (angular sensitivity scales as 1/d, i.e. √IR).
    - The far-range passes now read +0.02, which equals the modelled tilt.
    - No-wall FLq (raw 250–330) drops from about 104–137 to 76–100, which lowers the front mean the maze solver's `MAZE_FRONT_FAR_IR` sees.
  - The 650→800 junction is uncertain by about ±3% (it lies outside the squaring band).
  - Real C function: placements unchanged; the table is monotonic.
- **Session 5 (2026-10-14 16:03 log, 457 lines; user: the robot will square up at IRFR 1300–2000). Only IRFL ≥ 3357 (IRFR ≥ 1500) was changed.**
  - **IRFL compresses before it saturates.** Placements at IRFR 1595/1700/1738 read r +0.019/+0.030/+0.017 on the old straight segment 3357 → 3772 (→ 1802), which ran above the real convex curve. New knots: 3574 → 1595, 3775 → 1720 (the 1700 and 1738 placements merged), 3795 → 1840 (the "knee", about 5 IRFR per IRFL count, from passes). Those placements now read 0.000/+0.008/−0.007. Session 3's 3503 → 1563 placement went from +0.014 to 0.000, and its rotated-attitude block now reads a consistent −0.022…−0.035.
  - Below 1500 the table is kept: it matches the placements at 1294, 1345 and 1441 (sessions 3–4). The session-5 placement at 1354 (60 samples) reads +0.056 and contradicts them and session 5's own passes (+0.03), so it was treated as a rotated placement and not used.
  - **Limit:** above IRFR about 1720, IRFL is in the knee (3775–3800), and it saturates fully at about 3830 above about 1850. Where the knee sits varies about 8% between sessions: at IRFL 3771, session 3 read IRFR about 1860 and session 5 about 1720. **Square up at IRFR ≤ about 1650.** IRFR 1800–2000 can't be calibrated until the left channel's sensitivity is reduced in hardware.
  - Session-to-session scatter of stationary placements at the same IRFL is ±0.03 in r. That scatter, not the knot spacing, now limits accuracy. `IR_FL_ToRef(≥3800)` = 1870. Real C function on all 3 logs: monotonic, identical to the Python model.
- **The side table (IRL → IRR) was NOT refit from that log.** The user asked for it, but the log only contains a front wall. Facing a front wall, Lq/IRR is 0.60–0.65 at every distance, while side-wall data (IRL.log/IRR.log) gives 1.00. The side eyes see a front wall asymmetrically (right eye about 1.6× stronger; aim or mounting) but side walls equally. Fitting the side table to front-wall data would inflate left side-wall readings about 1.6× and break `WALL_SIDE_IR` avoidance. Re-measure the side pair with side walls (same 13-distance procedure) if needed.
- API: `IR_FL_ToRef(raw)` maps one IRFL reading; control code reads the filtered `IR_FL_Filt()`/`IR_FR_Filt()`. `print_LED` adds `FLq` (mapped IRFL), plus ` BH` when saturated, and `r:` (filtered (FLq − FR)/(FLq + FR), for `WALL_R_OFFSET`).
- While IRFL is saturated it carries no distance information, so the two sides can't be compared near the wall. That is exactly where front-wall alignment in a cell would happen: with the axle at the cell centre the robot front is about 14 mm from the wall. The fix is hardware: lower the left channel's sensitivity about 2.5×, via emitter or receiver resistor, then re-measure the table.
- The table must be re-measured after any emitter or receiver change.
- **Side pair** (`IRL.log`/`IRR.log`, same day): a side wall was placed at the same 13 distances for each eye (the user didn't give the actual distances).
  - Plateau detection gives 13 levels for IRL after merging sub-plateaus caused by hand adjustments.
  - IRR gave 14. Its 1094 and 1060 plateaus are only 3% apart, against an 8–17% step between distances, and a bump separates them, so they were treated as one distance.
  - Each table point is the median of the **last** stable sub-plateau at that distance. For IRL level 6 that means 621 rather than the earlier 572, which was recorded while something was near the right sensor.
- The side eyes already roughly match (IRR/IRL 0.91–1.09, max about 1270, no saturation). IRR is the reference, and `IR_L_ToRef()` uses a 13-point table through the shared `IR_Interp()`: proportional below the table, last slope extrapolated above it, clamped to 4095. API: `IR_L_ToRef(raw)`; control code reads the filtered `IR_L_Filt()`/`IR_R_Filt()`. `print_LED` adds `Lq`.
- The refactor of `IR_FL_ToRef` onto `IR_Interp` was checked identical for all 4096 inputs.
- Known gap: with no front wall, IRFL reads about 240–330 while IRFR reads about 20–60. The front table maps that to about 80–110, because its lowest point (486 → 164) still had a wall in view. An anchor around (290 → 30) would fix it, but the level depends on ambient light.

**Walls** (2026-10-10, user request: "keep IR < 1200 in the straight function; stop, square up to the wall, then reset the MPU"). Defines are the `WALL_*` block in `pid.h`. No IR → mm conversion exists yet, so every threshold is in equalized IR units (front on the IRFR scale, side on the IRR scale).
- **2026-10-14: the user set `WALL_TARGET_IR` to 1700 and `WALL_STOP_IR` to 2000** (previously 1100/1200), so squaring happens closer to the wall. The values in the notes below that predate this (1100/1200, sim results) refer to the old setting.
  - 1700 is where IRFL enters its knee before saturating (raw about 3775; see session 5). Squaring accuracy there depends on the steepest, least repeatable part of the FL table. If it squares off, measure `r` at true perpendicular at 1700 and set `WALL_R_OFFSET`, or lower the target to about 1600.
  - Sim re-check (`scratchpad/wall_sim2`: current sources, FL model from the session-5 table with the knee and saturation at about 3830, IR curve doubled so contact reads about 3400 like the real log). Approach at 250 mm/s from 150/500 mm, walls at −8/+12°, eyes toed out 10/in 40°, CLOSE start (IR 3100, back-off) and ENDWALL all returned 2. True heading after squaring was within ±0.6° and the frame error ≤ 0.07°. The 7-step HOLDWALL scenario passed for 5 seeds and both IR-curve scales. Peak IR while driving in was about 1690–1820. A 2% FL table error gives about 0.8° of frame error.
  - Clearance depends on the real nose-to-wall gap at IR 1700, which hasn't been measured. In the sim, with the gap at about 10 mm, the robot's own approach and squaring kept ≥ 7 mm. Hand rotations of 30–60° near the wall, and the robot's in-place return from them, swept the corners into the wall. Measure the gap on hardware; in-place 90°/180° turns at that spot need ≥ 11 mm.
- **Filtering:** `IR_Tick()` runs every 2 ms from `Motor_Speed_Control`. It takes the median of the last 3 samples, which removes spikes and dropouts, then applies an LPF with `IR_FILT_K` 0.3 (about 9 ms total lag). `IR_FL_Filt`/`IR_FR_Filt`/`IR_L_Filt`/`IR_R_Filt` return the mapped values.
- **Forward `Move_Straight`** with `wall_guard` = 1 (the default; set it to 0 to disable): `move_wall` is set only for dist > 0, because there are no rear sensors. In `MOTION_STRAIGHT` with v > 0:
  - *Front* = max(FL, FR), so the closer eye counts.
    - Above `WALL_SLOW_IR` (300) the speed is capped at √(Va² + (Vmax² − Va²)·frac). frac is linear in 1/√IR, which is distance if IR ∝ 1/d². Va = `WALL_APPROACH_MMS` (80), reached at the target.
    - The robot stops when front + max(rate, 0)·`WALL_STOP_LEAD_S` (0.04 s) ≥ `WALL_TARGET_IR` (1700). That sets `MOTION_WALL`; the ISR sets the targets to 0, and `Move_Straight` then calls `Wall_Align` and returns 2 or 0.
    - The straight also ends with `Wall_Align` (and returns 2 or 0) when it finished its distance normally with a **confirmed** wall ahead: both front eyes ≥ `WALL_PRESENT_IR`. The robot then creeps on to the target (user request, 2026-10-10). In the sim, a wall 30 mm past the goal (IR about 760) gave about 9 mm of extra travel and returned 2. At 50 mm (IR about 420) the result was 1 with no extra travel.
    - The IR rate lead covers the braking coast plus the filter lag for any IR-vs-mm slope. Without it the robot overshot to about 1150–1250.
  - *Side:* `head_err` used for steering gets +`WALL_SIDE_DEG_PER_IR`·(excess of R over `WALL_SIDE_IR`) − (same for L), clamped to ±`WALL_SIDE_MAX_DEG`. `heading_target` is untouched, and the abort and max-error statistics use the raw error.
- **Cell centring while driving straight** (2026-10-14, user request: "keep the IMU heading and centre in the cell; one side wall → use that side, both → use both"). `Wall_CenterOffset` in `pid.c`, `WALL_CENTER_*` in `pid.h`, runtime switch `wall_center` (default 1).
  - It runs in `MOTION_STRAIGHT` whenever v > 0, **independent of `wall_guard`**, so it also runs in the maze solver's non-blocking straights. Like side avoidance, it only adds `HEADING_KP`·offset to the steering command: `heading_target` and the absolute heading frame are unchanged.
  - The user measured the side eyes with the robot at the cell centre: IRL raw **240** (`WALL_CENTER_L_IR`, compared through `IR_L_ToRef`, about 248 Lq) and IRR **280**.
  - Lateral error uses the IR ∝ 1/d² law, which makes it linear in distance: eL = 1 − √(L0/L) and eR = √(R0/R) − 1, i.e. displacement / centre distance, positive = left. A side counts as a wall when its filtered reading is ≥ `WALL_SIDE_PRESENT_IR` = **170** in `print_LED` units (IRL raw, converted through `IR_L_ToRef`, and IRR); below 170 means no wall on that side (user setting, 2026-10-14). The no-wall baseline is about 70/40. With the centre at 240/280, a wall drops out once the robot is about 10–20 mm off-centre towards the other side; the near wall alone then keeps centring.
    - Both walls present: e = (eL + eR)/2. One wall: that side alone. Neither: gyro heading only. The filter state resets then.
  - The steering offset is −`WALL_CENTER_KP` (60)·e_f in degrees.
    - e_f is e low-passed with τ `WALL_CENTER_TAU_S` (20 ms).
    - The offset is clamped to ±`WALL_CENTER_MAX_DEG` (5°) and scaled by min(v/`WALL_CENTER_FULL_MMS` (150), 1).
    - It is off when the near front eye is > `WALL_CENTER_FRONT_IR` (600), because the side eyes see the front wall: with no side walls the 14/10 log shows IRR 40 → 90 at IRFR 1000.
  - It is P only. A gyro frame misaligned with the corridor by δ leaves a steady offset of about δ/(KP in rad) × the centre distance: 2° gives 1–4 mm depending on the IR law. Convergence is per distance travelled, so it is nearly speed-independent.
  - Sim (`scratchpad/wall_sim2`, `MODE=CENTER`): the real `pid.c`, a 168 mm corridor, side eyes scaled to read 240/280 at the centre. Variants tested:
    - 20 mm off: 3.0 mm left after 300 mm, 0.1 mm at 900 mm, no overshoot at 250 or 350 mm/s.
    - Eyes straight sideways or 45° diagonal, IR law offset 55.8/0/−20 mm (up to about 3× the effective gain), ADC noise ±10: all stable.
    - Without centring the same start hits the wall.
    - One wall only on either side, and a missing left-wall cell with no right wall: about 1° of heading blip and < 0.5 mm.
    - KP 40/60/80 were compared. 80 converges faster but reacts more at wall ends.
    - Front-wall APPROACH/ENDWALL/HOLDWALL results are unchanged.
  - **The maze solver uses the same threshold:** `MAZE_SIDE_WALL_IR` defaults to `WALL_SIDE_PRESENT_IR`. `HW_LeftWall`/`HW_RightWall` in `solving_maze.c` compare ≥ 170 the same way. It used to be 300 (Lq/IRR), above the 240/280 centre readings, so the solver would have seen no side walls.
- **Squaring lives in `MOTION_HOLD`** (`Hold_Update` in the TIM9 ISR, state `ws`). There is a single implementation; the first version had a separate blocking path that HOLD lacked. Each tick, in priority order:
  1. **Back-off:** if the near front eye is > `WALL_STOP_IR` (2000), the robot backs up at `WALL_CREEP_MMS` with heading hold until it is < `WALL_TARGET_IR`. Squaring then restarts.
  2. **`WS_TURN` fine turn:** at least `HOLD_MIN_DPS`.
     - It latches stopped once within `WALL_TURN_TOL_DEG` + |rate_f|·`WALL_TURN_COAST_S`, or once it has passed the target, and never turns back.
     - Stopping from the minimum speed coasts about 1°, so correcting would bounce forever.
     - It fails after `WALL_TURN_TIMEOUT_MS`.
     - After `WALL_SETTLE_MS` still, `heading_target` = actual heading and it measures. The gyro gives the true angle turned.
  3. **Normal hold:** the 2°/0.7° hysteresis. A hand rotation sends `ws` back to IDLE (except from FAIL), so the robot returns first and then squares again.
  4. **State machine** (when standing):
     - `WS_IDLE` waits for `WALL_SETTLE_MS` of stillness and needs a confirmed wall: **both** eyes ≥ `WALL_PRESENT_IR`, which is **600** at the user's request (it was 300). It then adjusts the distance first:
       - If the near eye is above target + `WALL_DIST_TOL_IR` (25), it backs off, because a probe turn raises one eye by about 4%.
       - If the near eye is below target − `WALL_DIST_START_IR` (60), it **creeps forward** (`ws_approach`, `WALL_CREEP_MMS`, heading hold). It stops when near + rate·`WALL_STOP_LEAD_S` ≥ target, when the wall is lost, or with FAIL after `WALL_APPROACH_MAX_MM`. It sets `ws_href` = `heading_target`. It re-probes g if the mean IR is more than 30% away from `wall_gain_ir`: g varies with distance, and with toed-in eyes it can even change sign.
     - `WS_MEAS` averages r = (FL − FR)/(FL + FR) − `WALL_R_OFFSET` over `WALL_MEAS_N` ticks; the robot must stay still. `Wall_Evaluate` then decides:
       - If g is unknown, it probe-turns `WALL_PROBE_DEG` (4°) and takes g = Δr/Δheading from the gyro, which also gives the sign. |g| < `WALL_MIN_GAIN` means fail.
       - Otherwise it updates g by secant after turns over 1° (same sign, ×0.5–2 only).
       - step = −r/g. It is done when |step| < `WALL_ALIGN_TOL_DEG` (0.5°), or < `WALL_ALIGN_ACCEPT_DEG` (1°) right after a turn that crossed perpendicular. Stiction makes the smallest in-place turn about 1.5°.
       - It fails after `WALL_ALIGN_ITERS` turns or beyond ±`WALL_ALIGN_MAX_DEG`.
     - `WS_ZERO` is a non-blocking copy of `MPU6500_ZeroGyroZ`: `GYRO_REZERO_SAMPLES` new `imu_raw[6]` samples, with the std and half-drift checks. It then calls `Wall_SetHeading`: `yaw_angle` = (ws_href + Robot_Heading − ws_hperp)/`yaw_scale` and `heading_target` = ws_href. This is written in the ISR at the same priority as the I2C callback, so no IRQ masking is needed. The wall normal becomes the held heading, and the sub-degree residual is kept.
     - `WS_DONE` re-squares if the filtered r implies more than `WALL_REALIGN_DEG`, if the mean IR moved more than 30%, or if the wall is lost (then IDLE).
     - `WS_FAIL` restores `heading_target` = ws_href and leaves the MPU untouched. It retries only after the wall disappears and comes back; otherwise a hand-return would retry the probe forever.
     - **Time limit** (2026-10-14, user report that alignment "hangs"). `ws_busy` counts ticks while there is a wall (or `ws_force`) and `ws` is neither DONE nor FAIL. That covers approach, back-off, waiting for stillness, probe/turns and the gyro re-zero. After `WALL_ALIGN_MAX_MS` (4000) it goes to FAIL:
       - from `WS_IDLE`, where no turn has changed `heading_target` yet, it only sets FAIL;
       - otherwise it calls `Wall_Fail`.

       Approach and back-off stop, but a > `WALL_STOP_IR` back-off still re-arms the next tick. The counter resets in `Wall_Reset`, at DONE/FAIL, and when the wall is gone.
       - Sim: normal alignments take 1–1.5 s; creeping about 30 mm to the wall first takes about 2.5–3 s. 2500 cut that case and 1500 also cut CLOSE, so 4000 leaves margin.
       - `MODE=WOBBLE` (front wall oscillating ±4 mm at 1 Hz, so squaring can never finish): before, the wheels kept chasing for the whole 12 s; now they stop after 4 s. A later `Wall_Align` with a still wall succeeds in 1.4 s.
- **`Wall_Align()`** (blocking, used after a wall stop): it sets `ws_force` (no wall means FAIL instead of waiting), starts HOLD, and waits for DONE, FAIL or `WALL_ALIGN_TIMEOUT_MS` (= `WALL_ALIGN_MAX_MS` + 500, a backstop; it was 8000). It then calls `Motor_SetSpeed(0,0)` (brake, IDLE), like the other blocking calls. On failure it sets `MOTION_WALL_FAIL` (unless LIFTED or NOT_READY) and `heading_target` = h_ref.
- **Accuracy is set by the FL→FR table, not by the turns.** In the sim each 1% FL/FR mismatch at the target distance gives about 0.35° of heading error (g ≈ 0.014/°), versus 0.02–0.08° with a perfect table. **On the real robot g ≈ 0.0032/°, so 1% is about 1.5°** (see the reverted refit above). Calibrate it by placing the robot truly square to a wall at about `WALL_TARGET_IR`, reading `r:` from `print_LED`, and putting that into `WALL_R_OFFSET`.
- **Sim** (`scratchpad/wall_sim`): real `pid.c` + `mpu6500.c` + `adc.c`; ray-cast walls; IR = C/(L + L0)²·cos(incidence), giving 1100 at 20 mm and 160 at 100 mm; FL raw from the inverse table; plus the motor and gyro models from the hold sim.
  - Every case returned 2 with the near-eye IR < 1200 while driving: 250 and 350 mm/s from 500 mm, walls tilted −8° to +12°, eyes toed out 10° or in 40° (sign flips), gyro gain +3%, starting 12 mm from the wall, and the user's `while(1)` loop with walls at both ends.
  - Heading frame after reset was 0.01–0.08° off, and the robot was physically within ±0.65° of perpendicular. Stop plus alignment took about 1–1.5 s, of which about 0.5 s is the gyro re-zero. Later walls reuse the stored g and skip the probe.
  - Toe-in of 25° makes g ≈ 0.0018/° < `WALL_MIN_GAIN`, so it fails cleanly.
  - No wall: returns 1. Side: driving 3° into a side wall peaks at IR ≈ 1235 with 2.5 mm clearance, versus a collision with `wall_guard` = 0.
  - `MODE=HOLDWALL` reproduces the user's `while(1) { Move_Straight(0, 250); }`:
    - placed 6° off a wall at 30 mm: squared to 0.8°, frame error 0.06°;
    - hand turns of +30 and −60: returned and re-squared, ≤ 0.1°;
    - wall pushed to 12 mm (IR 1600): backed off to about 1065 and re-squared;
    - wall rotated 5°: re-squared;
    - wall removed plus a hand turn of −45: returned to the held heading;
    - a new wall placed 3° off: squared.
  - The same scenario passes with eyes toed out 10° and toed in 40°. A 2% table mismatch gives about −0.8° of frame error, as expected.
  - **Geometry warning:** an in-place 180° turn sweeps about 81 mm from the axle against a 70 mm nose. With the sim's 20 mm target the corners grazed the wall, so check on hardware that the nose-to-wall gap at `WALL_TARGET_IR` is ≥ 11 mm, and lower the target if it isn't. The braking room between `WALL_SLOW_IR` and the target also needs about v²/(2·1500) mm: 20 mm at 250 mm/s, 50 mm at 400.

**Maze solver** (`solving_maze.c`, 2026-10-13, user request: "continuous motion, merge straight cells, stop only at the centre of a turn cell, keep mapping while moving, state machine, MISRA style").
- **Non-blocking straight API added to `pid.c`.** `Move_Straight_Begin(dist, speed)` uses the same `MOTION_STRAIGHT` as `Move_Straight` (gyro PD, √ decel profile, IR front stop, side avoidance) but returns immediately and does no end-of-straight `Wall_Align`. Companions:
  - `Move_Straight_SetTarget(dist)` changes the goal mid-run. Extending keeps full speed; shortening brakes to stop exactly at the new goal.
  - `Move_Straight_Traveled()` reads the distance with IRQs off.
  - `Motion_Status()` returns `MOTION_STATUS_RUNNING/DONE/WALL/FAIL`.
- **Layers.** The algorithm part (map, `Flood_Fill`, `Plan_BestDir`, `Plan_StraightCells`, phases) never touches hardware. Only the static `HW_*` wrappers call `IR_*_Filt`, the motion API, `Turn_Angle`, `Wall_Align`, `Motion_Begin`, `MPU6500_ZeroGyroZ`, `HAL_GetTick`/`HAL_Delay` and `UART_Print`.
  - Map: one byte per cell, wall bits plus known bits, edges written on both sides.
  - BFS uses a static queue: no recursion, no heap and no stdio.
  - Config defines are wrapped in `#ifndef` so a host test can override them.
- **State machine** (fully non-blocking since 2026-10-15). It has these states:
  - `MAZE_STATE_DECIDE_PATH`;
  - `MOVE_STRAIGHT`, using `Move_Straight_Begin`;
  - `TURN_LEFT`/`TURN_RIGHT`/`U_TURN`, using `Turn_Angle_Begin` + `Motion_Status`, with `MAZE_TURN_TIMEOUT_MS`;
  - `ALIGN`, using `Wall_Align_Begin`/`Poll` before a turn with a front wall;
  - `FINISHED`/`ERROR`;
  - `PAUSED`.

  Each motion is started once (`act_started`) and polled on later `Maze_Step()` calls.
  - A motion failure while `robot_lifted` → `PAUSED`, with the cell and heading kept; anything else → `ERROR`. `DECIDE_PATH` checks `robot_lifted` first, so it never reads sensors while lifted. Straight sensing is skipped while lifted.
  - `Maze_Pause()` rounds a running straight to the nearest cell centre: the same rounding as `Straight_Finish`, and consistent with what was sensed. It also finishes a pending align. `Maze_Resume()` goes back to `DECIDE_PATH`, which re-reads the front wall and re-plans from the saved cell.
  - Phases (enum values unchanged for the viewer; `RETURN` = 4 appended): SEARCH_GOAL (unknown walls treated as open), then SEARCH_START (both at `MAZE_SEARCH_SPEED_MMS` 300 mm/s, raised from 250 on 2026-10-14; sim at 300: front-wall stop peak IR about 1760–1790, centring unchanged). Exploration now ends in DONE at (0,0) (MODE 1). There is no automatic speed run any more: `Maze_StartSpeedRun()` (MODE 2, known walls only, 350 mm/s, refuses without a known path) → DONE at the goal; `Maze_StartReturn()` (U-turn, then known-only shortest path to (0,0) at `MAZE_RETURN_SPEED_MMS` 300) → DONE. A run that ends DONE at (0,0) first turns to face North, so it is ready for the next run.
  - `Maze_Init`/`Maze_StartSpeedRun` only call `Motion_SetOrigin()` (no gyro re-zero); the Mode Manager re-zeros the gyro beforehand, non-blocking. `MAZE_RUN_START_DELAY_MS`, `HW_Wait` and `HW_RezeroGyro` are gone.
  - On returning to start, if the known-only and optimistic shortest paths are equal the path is proven optimal; otherwise it runs another round, up to `MAZE_MAX_ROUNDS`.
- **Merging straight cells.**
  - `Plan_StraightCells` walks the flood-fill gradient while the best direction stays the current heading. Ties prefer straight, then right, left, back. The robot gets one goal at the centre of the last such cell.
  - While driving, when the axle is `MAZE_SENSE_BEFORE_MM` (90, the cell boundary) before the centre of cell k, it does three things without stopping:
    - reads cell k's side walls (diagonal side eyes look about 94 mm ahead of the axle) and the front wall about 114 mm away (`MAZE_FRONT_FAR_IR`);
    - re-runs flood fill;
    - re-plans from k, then extends or shortens the running goal.
  - A turn at k leaves 90 mm to brake (fine up to about 420 mm/s at `MOVE_DECEL_MMS2` 1000).
  - If the far reading misses a wall, the IR front stop (`MOTION_STATUS_WALL`) catches it. The cell is then rounded from the distance travelled and the wall written as known.
  - At rest the front wall is re-read near (`MAZE_FRONT_NEAR_IR`) and overwrites the map. Readings taken while moving only fill unknown edges.
  - Before a turn with a front wall it runs `Wall_Align` (`MAZE_ALIGN_ON_FRONT_WALL`, `MAZE_ALIGN_MIN_INTERVAL_MS`). That re-squares heading and also centres the robot along the cell, but only if `WALL_TARGET_IR` equals the IR reading at the cell centre.
- **Host sim** (`scratchpad/maze_sim`, written for the old blocking API; `scratchpad/mm_sim` is the current harness, see "Mode Manager"): the real `solving_maze.c` with mock headers, motion with a √ profile, 2000 mm/s² ramp and IR front stop, front IR from true wall distance, diagonal side IR, noise, and a nose-collision check. On 60 real 16×16 mazes:
  - 60/60 finished, 0 collisions, and the speed run equalled the true shortest path in every maze.
  - No stops mid-path. The only "stop then straight" events (21) are the deliberate stop on reaching the goal before the return.
  - The same holds with 15% IR noise, and with the far front reading disabled: the IR stop then caught 3–43 walls per maze.
  - Mean sim time was 455 s with alignment at every front wall, 375 s at a 5 s minimum interval, and 328 s without alignment. The sim assumes 0.3/0.5 s turns and a 1 s align.
  - Its explored cell counts match `Simulator/mms-c`.
- **Needs hardware calibration:** `MAZE_SIDE_WALL_IR`, `MAZE_FRONT_FAR_IR` (front-eye mean with the wall about 114 mm away), `MAZE_FRONT_NEAR_IR`, `MAZE_SENSE_BEFORE_MM` (about 40 if the side eyes look straight sideways), and `WALL_TARGET_IR` = the cell-centre front reading.
- Straights centre in the corridor from the side walls (`wall_center`, see "Cell centring while driving straight" under Walls) on top of the gyro PD and `WALL_SIDE_IR` avoidance.
- The CLI build needed `solving_maze` added by hand to `Debug/Core/Src/subdir.mk` and `objects.list`. In the IDE, use Refresh (F5).
- **Map to PC** (2026-10-13; since 2026-10-15 driven by the Mode Manager). The map is sent after each finished MODE 1 / return-home / error. In IDLE, any byte from the PC (`UART_TryRead`) re-sends it; this blocks about 1 s at 9600 baud, robot stopped. `UART_WaitStart` is no longer used by `main`.
  - Format (stdio-free, documented in `solving_maze.h`): `#MAZE_BEGIN n`, `#STATE`, `#ROBOT x y h`, `#STATS`, n× `#ROW y <hex bytes>` (the firmware cell bytes), and `#MAZE_END <sum of bytes mod 65536>`.
  - At 9600 baud it is about 800 bytes, about 1 s.
- **`maze/maze_viewer.py`** (host PC; Python 3 + tkinter; pyserial for `--port`; no matplotlib or Pillow).
  - `--port COMx` opens the HC-05 port (any pyserial URL also works, e.g. `socket://` for tests). It sends `M` every 3 s until a block with a valid checksum arrives (a bad block triggers a re-request), saves it to `maze/logs/maze_<time>.txt`, and draws it. A button re-requests.
  - `--file` redraws a saved log. It also parses `Maze_PrintMap` ASCII and mazefiles text. `--mazefile` draws a sample maze. `--list` lists COM ports. `--ps` exports PostScript. `--close-after N`.
  - The drawing shows known walls in red, unknown edges as grey dashes, unexplored cells in grey, start in green, the 2×2 goal in yellow, the robot as an orange arrow, and the BFS shortest path over known-open edges in blue (dashed optimistic path if none). An info panel shows state, phase, rounds, stats, % edges known, and known vs optimistic length.
  - The process is set DPI-aware and scales cell size by DPI.
  - Tested on real `Maze_SendMap`/`Maze_PrintMap` output from the host sim (`DUMP=<maze> [STOP_PHASE=1]`), and live through a fake robot on `socket://` that first sends a corrupted row: it was rejected, re-requested and then drawn.
  - The user's PC shows Bluetooth COM3/4/8/9. **COM8 is the HC-05 Outgoing port** (MAC 98:D3:51:F6:A1:FF); COM4/COM9 are Incoming and COM3 is a TWS headset.
  - `DEFAULT_PORT = "COM8"`, so running with no arguments uses it. `maze/run_viewer.bat` (double-click) runs `--port COM8 --baud 9600`.
  - **GUI app (rewritten 2026-10-13 after the user reported freezes on a momentary disconnect).**
    - Cause: serial writes ran on the Tk thread with no `write_timeout`. A pyserial write to a Bluetooth COM port whose link dropped blocks forever, and `close()` on a dead link can block too.
    - Now **all port I/O (open, write, read, close) runs in `SerialWorker`'s own thread.** The Tk thread only posts commands (`connect`, `disconnect`, `request`, `auto`, `quit`) to `cmd_q` and drains `evt_q` (`state`/`log`/`map`) every 50 ms.
    - Writes use `write_timeout=1.0`. A timeout or read error counts as a lost link: close, then retry every `RETRY_S` (3 s) if "Tự nối lại" is ticked. An open blocks in Windows for about 5–20 s ("semaphore timeout", WinError 121) when the HC-05 is off, out of range or connected elsewhere; commands queue and are applied after it returns.
    - Controls: port combobox (Bluetooth ports labelled Outgoing/Incoming from the MAC in hwid, ⟳ rescan), baud (default 9600), "Kết nối"/"Kết nối lại" (closes and reopens immediately), "Ngắt kết nối", "Xin bản đồ", "Tự nối lại", "Mở file...", "Lưu hình (.ps)", a coloured status dot and a log pane.
    - The app starts with no connection required. With no arguments it auto-connects to COM8 in the background; `--no-connect`, `--file`/`--mazefile` only draw.
    - Identical consecutive maps are saved only once.
  - **Python 3.13 pitfall:** don't subclass `threading.Thread` with private method names like `_handle`; 3.13 uses `Thread._handle` internally and the override broke the thread ("'_ThreadHandle' object is not callable"). `SerialWorker` therefore composes a Thread instead.
  - **Tests** (`scratchpad/viewer_test/test_app.py`, which measures the max Tk event-loop gap):
    - `hang://`, a pyserial handler whose write blocks 10 s and close 5 s, with reconnect and disconnect clicked meanwhile: max gap 32 ms.
    - A TCP fake robot appearing 5 s late, sending a corrupt block, then a good one, then dropping: max gap 85 ms; the map was drawn and the drop was detected and reconnected.
    - Real COM8 with the robot off: max gap 97 ms, clear "HC-05 không trả lời" message, retries.

**Mode Manager + Safety** (`mode_manager.c/.h`, 2026-10-15, user request: "Mode Manager and Safety System, buzzer on PB8, absolutely no `HAL_Delay`, everything as state machines on `HAL_GetTick()`").
- **Timing rule.** Every wait stores `t0 = HAL_GetTick()` on entry and checks `(now − t0) ≥ duration` on later `ModeManager_Update()` calls. The unsigned subtraction is wrap-safe. Nothing in the mode or maze path busy-waits any more:
  - turns, wall alignment and the start-of-run gyro re-zero all use start/poll APIs;
  - the only remaining blocking calls are UART prints and `Maze_SendMap` (about 1 s at 9600, no TX DMA/IRQ configured), and they run only while the robot is stopped.
- **Buzzer:** a non-blocking pattern player. A pattern is an array of ms durations alternating ON/OFF. On each step boundary `bz_t0 += step` (not `= now`), so loop jitter never accumulates. Patterns:
  - every ON pulse is `BUZZ_ON_MS` = 500 ms (user setting 2026-10-15; was 2 s for UI/goal and 150 ms for confirm/error);
  - per press: one 500 ms beep, always at least `UI_BEEP_GAP_MS` 300 after the previous one. Presses faster than the buzzer are queued (`ui_beeps_due`) and played in turn, so the number of beeps always equals the number of presses. (Restarting the pattern on each press, as before, lost beeps at ~4 presses/s: 4 presses gave 2 beeps.);
  - MODE 3 confirm: one `CONFIRM_LONG_BEEP_MS` 1500 ms beep (the user asked for "1 tiếng bíp dài");
  - goal: 500 / `GOAL_BEEP_GAP_MS` 500 / 500;
  - error ("tít tít tít"): 3 × 500 with 150 gaps, both for a rejected selection and for a run error.
- **Virtual button** = IR left raw (`adc_value[1]`, the `print_LED` IRL column).
  - A press is > `UI_IR_PRESS_RAW` 3000 continuously for `UI_DEBOUNCE_MS` 30.
  - It only counts if the hand was removed (< `UI_IR_RELEASE_RAW` 2000) since the last press, including after power-up. Holding the hand never counts twice.
- **Three modes** (2026-10-15 rework; `RunMode`). The map persists in Flash sector 7 (see "Flash map storage"):
  - **MODE_1, continue the old map:** the Flash map is imported into RAM by `Check_Mode` → `Maze_StartContinue` from (0,0) facing North (`MODE1_START_AT_SAVED_POS` 1 = from the saved cell and heading instead). All known walls are kept and flood fill only explores what is unknown. A map that is already `Maze_IsExplored()` still runs (goal and back, re-checking walls; "map da du: di kiem tra lai"). It no longer switches to MODE_2.
  - **MODE_2, fast run + return:** runs on the Flash map (imported by `Check_Mode`). Fast run → goal beeps 0.5 s ×2 → U-turn → shortest known path home → face North.
  - **MODE_3, new map:** `Maze_Init()` (RAM map cleared), then immediately saves an empty-map record (`FLASH_SAVE_NEW`) on top of the old one, so a power cut mid-run never resumes the old maze map. Then it explores.
- **UI: number of covers = mode number** (2026-10-15 rework, user request "che 1 lần = luôn luôn Mode 1, 2 lần = Mode 2, 3 lần = Mode 3"). There is no suggested mode and no automatic mode change any more; `Suggest_Mode`/`Mode_Next` are gone and `mm_mode` starts as `MODE_NONE`.
  - In IDLE, each press counts (`ui_count`) and beeps once. After the beeps end, 3 s (`UI_WINDOW_MS`) without a press → `Select_Mode(ui_count)` runs the **safety check while the robot is still stopped**:
    - 1 → MODE_1: `Flash_LoadMap` must succeed and `Maze_ImportMap` must accept the map, otherwise rejected.
    - 2 → MODE_2: additionally `FLASH_MAP_HAS_PATH` and `Maze_HasPath()` after import. "Map dở" (partial) means no known-only path to the goal yet, matching the earlier wording. A map that has a path but is not proven optimal (`FLASH_MAP_EXPLORED` clear) is accepted.
    - 3 → MODE_3: always valid → long confirm beep → run.
    - 0 or ≥ 4 → rejected, so an accidental extra cover can never wipe the map.
  - Rejected: UART "Che N lan: KHONG CHAY - <reason>", 3 error beeps, `STATE_ERROR` → IDLE. No save and no map send, and the robot never moves.
- **States** (`RobotState`):
  - `STATE_IDLE`: motors stopped (`Motor_Stop`); serves map requests.
  - `STATE_UI_BEEP` (press beeps playing, including queued ones) → `STATE_UI_WINDOW` (3 s count): as above.
  - `STATE_START` waits `MODE_START_DELAY_MS` 1 s, until the hand is off the left eye, `!robot_lifted` and the buzzer is idle (so MODE_3 starts after the long beep). It then re-zeros the gyro (`MPU6500_ZeroGyroZ_Start/Poll`, about 0.5 s) and calls `Start_Mode1/2/3`.
  - `STATE_EXPLORE` (MODE_1 and MODE_3):
    - maze `CHECKPOINT` (goal reached while exploring, robot braked) → `Save_Map(FLASH_SAVE_GOAL)` → `Maze_CheckpointDone()`;
    - `FINISHED` → `Save_Map(FLASH_SAVE_EXPLORED)`; send the map; IDLE;
    - `ERROR` → `Enter_Error(…, save)`.
  - `STATE_MODE2_RUN` → at the goal, `STATE_GOAL_BEEP` → `Maze_StartReturn()` → `STATE_RETURN_HOME` → at (0,0) facing North → IDLE.
  - `STATE_ERROR`: a run error saves `FLASH_SAVE_ERROR` if a run was active and sends the map. Both run errors and rejected selections then give 3 beeps and go to IDLE.
  - Moving states are EXPLORE/MODE2/RETURN. If `robot_lifted` or the maze reports `PAUSED` → `Enter_Paused()`:
    - `Maze_Pause()` and `Motor_Stop()`;
    - prints "TAM DUNG (nhac xe: …): dat lai o (x,y) mui huong …" over UART;
    - `Save_Map(FLASH_SAVE_LIFTED)`.
  - **Save points** (`Save_Map`): `Motor_Stop()` first; it exports the map, pose, phase, rounds and the HAS_PATH/EXPLORED flags, then prints "LUU FLASH (…) lan #n … -> OK". Saves happen only when stopped: at the goal and at the end of exploration, on a lift, on an error, and at the start of MODE 3. Never while driving.
  - **`STATE_PAUSED` → `STATE_RESUME`** needs 5 s (`PAUSE_STILL_MS`) of continuous stillness:
    - every IR channel within max(25, 8%) of the snapshot;
    - wheels rolled ≤ 1 mm;
    - `lift_acc_dev` ≤ 4 mg;
    - |yaw rate| ≤ 2 °/s.

    Any violation restarts the 5 s. At the end `!robot_lifted` (the IMU set-down, about 1 s calm) must also hold. RESUME calls `Maze_Resume()` and returns to the interrupted state. **The user must put the robot back at the printed cell centre and heading.**
- **`Check_Robot_Picked_Up()`** runs in the TIM9 ISR after `MPU6500_Tick()`. It counts 2 ms ticks; there is no `HAL_GetTick` in the ISR.
  1. **IMU** (primary, always on): `robot_lifted` from `Lift_Update`.
  2. **Encoder**, only in moving states: a wheel *faster* than its ramped reference by > `PICKUP_ENC_OVERSPEED_MMS` 120 for `PICKUP_ENC_MS` 30, when |ref| > 50, means it lost its load. Collisions and stuck wheels make it slower, so they don't trigger this.
  3. **IR**, only in moving states and with both wheels < 10 mm/s:
     - a strong wall (front ≥ 300 or side ≥ 170) was seen ≤ 300 ms before;
     - then all 4 eyes drop below the no-wall level (front < 130, side < 110) for 60 ms.

     It is not used while driving, because driving into an open area also drops all eyes.

  Secondary detections call `MPU6500_ForceLifted()`. That enters the same lifted state as the accelerometer (motion cancelled, motors locked, PB15 off); set-down is still confirmed by `Lift_Update`. The reason (`PICKUP_BY_*`) is printed. Both encoder and IR thresholds are untested on hardware; disable them with `PICKUP_ENC_ENABLE`/`PICKUP_IR_ENABLE`.
- **pid.c/mpu6500.c additions for this:**
  - `Motion_SetOrigin()` (= `Motion_Begin` without the gyro re-zero; `Motion_Begin` now calls it);
  - `Turn_Angle_Begin()` (`Turn_Angle` = Begin + `Motion_Wait`; a refused start now restores `heading_target`);
  - `Wall_Align_Begin()/Poll()` → `WALL_ALIGN_BUSY/OK/FAILED` (`Wall_Align` = Begin + loop);
  - `MPU6500_ZeroGyroZ_Start()/Poll()` → `GYRO_ZERO_RUNNING` or a HAL status (`MPU6500_ZeroGyroZ` = Start + loop);
  - `MPU6500_ForceLifted()`.

  The full-robot `wall_sim2` results (APPROACH/CLOSE/ENDWALL/HOLDWALL/CORRIDOR/CENTER) are identical before and after the refactor.
- **Sim** (`scratchpad/mm_sim`): the real `mode_manager.c` + `solving_maze.c` on real mazes, with mock motion, IMU and buzzer GPIO, a 1 ms loop, and `Check_Robot_Picked_Up` every 2 ms.
  - Buzzer (pre-rework measurement): 500 (UI) / 500-500-500 (goal). Window 3000 ms. Start 1500 ms after confirmation.
  - Full explore → MODE 2 → return flow correct on 29/30 competition mazes: speed run = true shortest path, back at (0,0) facing N, no crash. The 30th (`001-anomaly-test`, no path to centre) correctly ends in ERROR.
  - A lift mid-straight pauses instantly at the rounded cell, and the run resumes exactly 5.000 s after placement. A wheel roll of 3 mm or 50 ms of 8 mg vibration restarts the 5 s.
  - Encoder overspeed is detected in 30 ms, IR vanish while aligning in 60 ms. All runs then complete correctly.

**Flash map storage** (`flash_storage.c/.h`, 2026-10-15, user request: "sector 7, Magic_Word 0xA5A5B6B6, map + last X/Y, save when exploring reaches the goal and on errors, never continuously").
- **Linker:** `STM32F411CEUX_FLASH.ld` sets FLASH = 384K (sectors 0–6). Sector 7 (0x08060000–0x0807FFFF, 128 KB) is never part of the image:
  - flashing the `.elf` (CLI `-w`, IDE launch) erases only the program sectors and keeps the map;
  - "Full chip erase", `-e all` or an RDP regression wipes it;
  - code larger than 384 KB fails to link.
- **`FlashMapData`**, 280 B (word multiple, `_Static_assert`):
  - `magic_word` 0xA5A5B6B6, `version` 1, `record_size`, `sequence`;
  - `maze_map[256]` (byte x·16+y, same bits as RAM);
  - `last_x`/`last_y`/`last_heading`, `phase`, `rounds`, `reason` (`FLASH_SAVE_NEW/GOAL/EXPLORED/LIFTED/ERROR`), `flags` (`FLASH_MAP_HAS_PATH`/`EXPLORED`);
  - `checksum` (bitwise CRC-32 of everything before it).
- **Append log (wear leveling).** The sector holds 468 slots, written sequentially.
  - `Flash_SaveMap`: Unlock → (erase sector 7 only if no blank slot is left, or if `FLASH_ERASE_EVERY_SAVE` is 1) → program word by word into the next fully blank slot → Lock → flush the ART data cache (`DCEN` is on) → `memcmp` verify.
  - `Flash_LoadMap`: the first blank slot is found by first words. Then it walks back to the last record with a valid magic/version/size/CRC, usually 1 CRC.
  - A power loss mid-write leaves an invalid slot: load falls back to the previous record and the next save skips past it. Erase-before-write would lose both copies.
  - `Flash_EraseMap` wipes the sector.
- **Stalls:** the F411 has a single bank, so programming (a few ms per record) and erasing (about 1–2 s, only when full) stall instruction fetch and delay every IRQ, including the PID tick and SysTick. Callers therefore `Motor_Stop()` first and only save when the robot is stationary.
- `Maze_ExportMap`/`Maze_ImportMap`: the import rejects maps whose boundary is not known wall or whose two sides of an edge disagree. Also added: `Maze_StartContinue`, `Maze_IsExplored`, and `MAZE_STATE_CHECKPOINT` + `Maze_CheckpointDone`. `MAZE_STATE_CHECKPOINT` is appended (enum 10); `maze_viewer.py` knows it.
- **Sim** (`scratchpad/mm_sim/sim2.c`): the real `flash_storage.c` + `mode_manager.c` + `solving_maze.c` against a Flash mock (erase = 0xFF, programming only clears bits and fails on a non-blank word, optional power cut after N words), with power cycles (RAM cleared, Flash kept).
  - 1000 saves → 2 erases, 0 read/write errors.
  - A power cut mid-write → the previous record loads; the next save skips the broken slot.
  - Cover-count UI (scenario `ui`, empty Flash):
    - 1 cover → 1 beep, then rejected with 3 × 500 beeps, robot still;
    - 2 slow covers (2 s apart, the 2nd during the window) → 2 beeps, rejected;
    - 4 fast covers (250 ms period) → 4 beeps, rejected;
    - 3 fast covers → 3 beeps + 1500 ms → MODE 3 explores.
  - `full`: 3 covers → MODE 3 explored (flags HAS_PATH | EXPLORED) → power cycle → 2 covers → fast run = true shortest path, home → power cycle → 1 cover on the complete map → MODE 1 runs (goal and back) → erase Flash → 2 covers and 1 cover are both rejected.
  - `partial`: MODE 3 lifted after 10 cells → power cycle → 2 covers rejected ("map do") → 1 cover continues and finishes → 2 covers → correct fast run.
  - 120 competition mazes: 266/270 runs correct (`ui`/`full`/`partial` on the first 30, `full`/`partial` on the next 90), 0 Flash faults. The two failures are `001.txt` and `001-anomaly-test.txt` in `full`/`partial`. The harness's own BFS also finds no path to the goal there; the firmware correctly errors and rejects MODE 2.

**Lift detection** (2026-10-10, user request) is automatic. It runs in `Lift_Update()` at the end of `MPU6500_Process_Data`, in the I2C ISR, using the accelerometer only. The gyro is avoided because a fast turn in place would otherwise look like a tilt whenever the accel reference is misaligned. Thresholds are the `LIFT_*` defines in `mpu6500.h`.
- **Lifted** when either condition holds for `LIFT_CONFIRM_S` (20 ms):
  - tilt > `LIFT_TILT_DEG` (20°). Tilt is the angle between the accel vector, low-passed with `LIFT_TILT_TAU_S` (0.2 s), and `lift_ref`, the accel direction captured while resting on the floor.
  - |vertical velocity| > `LIFT_VZ_MS` (0.12 m/s). This is the leaky integral (τ `LIFT_VZ_TAU_S`) of the accel component along `lift_ref` minus its rest magnitude.
- On lift it sets `robot_lifted`, turns PB15 off and calls `Motion_OnLifted()`. That cancels the motion (the waiting function returns 0 with `MOTION_LIFTED`, "HUY: xe bi nhac len"), disables the PID and brakes.
- While lifted, three guards keep the motors off:
  - `Motor_SetPWM` forces 0;
  - `Motor_SetSpeed` calls `Motor_Stop` and returns;
  - `Motion_Start` refuses.

  As a safety net, `Motor_Speed_Control` re-runs `Motion_OnLifted` if the main thread re-enabled something.
- **Set down** needs every sample to be calm for `LIFT_PLACE_S` (1 s), restarting on any non-calm sample:
  - |gyro| < 2 °/s on all axes;
  - `gz_dev` < `STILL_DEV_DPS`;
  - accel vibration `lift_acc_dev` < `LIFT_PLACE_ACC_DEV_G` (4 mg; a hand always trembles more);
  - wheels still;
  - tilt < 10°.

  It then applies the same Gz std and half-drift checks as calibration.
- On success:
  - gyro offsets for all 3 axes are set from the window means;
  - `lift_ref` is refreshed;
  - `yaw_angle` = 0, so the initial angle is the angle at set-down;
  - PB15 turns on;
  - `Motion_OnPlaced()` sets `heading_target` and resets the odometry origin.
- Interrupted motions do not resume by themselves. The Mode Manager pauses an interrupted maze run and resumes it after the robot is put back and stays still for 5 s (see "Mode Manager + Safety").
- **Sim** (`scratchpad/lift_sim`): the real code, a 3-axis IMU with the board's (0.16, 0.15, −1.17) g rest reading modelled both as accel offsets and as a ~10° chip tilt, driving vibration, centripetal force at 35 mm from the axle, and hand tremor.
  - No false lift during driving at 350 mm/s, 180° turns, a hard brake from 350, hitting a wall at 300, hold mode with hand rotation on the floor, or a 4 s spin. Worst cases were tilt 6.7° and |vz| 0.054 m/s.
  - A lift is detected in 70–110 ms.
  - The LED comes on 1.5–2.0 s after release, yaw is 0, and the next straight follows the set-down heading.
  - Holding the robot still in hand (6 mg / 1 °/s tremor) stays "lifted".
  - Limitation: a perfectly level lift slower than about 5 cm/s (peak velocity under 0.12 m/s) isn't detected. Lower `LIFT_VZ_MS` only after checking the `vz` column while driving on hardware.

`Gyro_Spin_Calibrate(turns)` calibrates gyro sensitivity automatically. It runs after a UART key press and reports over UART. The front IR sensors use a wall as the reference:
1. It re-zeros the gyro, then spins left at a constant `SPIN_CAL_DPS` (150 °/s) for (turns + 1) × 360 + 180°, using `Motor_SetSpeed(-v, v)` directly, not the turn profile.
2. While spinning, the ISR records a `SpinRec` every `SPIN_CAL_DECIM` × 2 ms into a 36 KB buffer (`SPIN_CAL_MAX` = 3000). Each record holds the heading (old scale), dR − dL, and FL + FR from `adc_value`.
3. The polarity is auto-detected: the wall reading may rise or fall as it gets closer.
4. Each wall peak is located by the centroid of the smoothed signal above 70%. The first is found in [h0 + 90, h0 + 450); later ones are chained at +360 ± `SPIN_CAL_WINDOW_DEG`. Contrast must be at least `SPIN_CAL_MIN_CONTRAST`.
5. The scale factor is turns·360 / (gyro angle between the first and last peak). It is applied to `yaw_scale` immediately; the user must copy the printed value into `YAW_SCALE`. Effective track width = Δ(dR − dL) / (turns·2π).
6. The robot then turns back to its start orientation.

Inertia doesn't matter, because only peak positions taken during the steady spin are used. Simulated: a 3% gyro error is measured as 3.02%, track width 80.0 mm vs 80 true, and the final orientation is 0.2° off; inverted IR polarity also works; with no wall it fails cleanly and keeps the old scale.

`Rectangle_Test(length, width)` drives 4 sides with a right turn after each, then `CDC_WaitHost()` and prints a per-segment report: target vs achieved, final heading error, max deviation or overshoot, time, and result. It ends with the odometry end position.

The blocking calls are split into `Motion_Start` (non-blocking start) and `Motion_Wait(timeout)`.

Dead code removed on 2026-10-14 (user request; still in commit `e811747` on `main`):
- mpu6500: the MPU6050-style globals `Accel_*_RAW`/`Gyro_*_RAW`/`Ax..Gz`, blocking `MPU6500_Read_All`, `I2C_ScanAndPrint_CDC` and `MPU6500_YawWrapped`. `MPU6500_Init` now uses the `REG_*` defines instead of literals.
- adc: the raw getters `IR_FrontLeft`/`IR_FrontRight`/`IR_FrontLeftSaturated`/`IR_Left`/`IR_Right`. Only the `_Filt` versions were used.
- pid: `Robot_WaitPlaced` and the unused average defines `WHEEL_DIAMETER`/`MM_PER_COUNT`.
- The test and print tools were deliberately kept: `PID_Test`, `MPU6500_Test`, `Rectangle_Test`, `Straight_Test`, `Gyro_Spin_Calibrate`, `print_*`, `Maze_PrintMap`. So were the API pairs `Turn_Left`/`Turn_Right` and `Maze_GetPhase`/`Maze_GetStats`. With `--gc-sections` they cost no flash or RAM until main calls them.

Removed test tools (2026-10-09, they are in git commit `cbc3493` on branch `TEST_PID` if needed again):
- `Straight_Log_UART`: live gyro vs encoder log over Bluetooth;
- `Straight_IMU_Test`: 54 KB RAM recording of raw IMU and encoders, per-phase noise, Goertzel spectrum, DWT interrupt timing;
- `Heading_PID_Test`: ±5° heading steps, with gains received over Bluetooth via `UART_ReadLine`.

Their hardware results are recorded under Gotchas.

**Encoder input filter is off** (`IC1Filter`/`IC2Filter` = 0 on TIM2/TIM3), and PB3 (UART RX) is the pin next to PB4/PB5 (left encoder). The interference test measured 0 pulses while UART was active, but if encoder glitches ever show up, set the filter to 15 in CubeMX. Encoder edges are at least about 300 µs apart, so the roughly 4 µs filter is safe.

`Straight_Test(dist_mm, speed_mms)` is the single-straight version (negative reverses). It reports:
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

The `print_LED()`, `print_mpu6500()` and `print_SpeedPID()` helpers are debug output, meant to be called by hand from the main loop.

`print_mpu6500()` prints one CSV line over **USART1** (HC-05) with the IMU values *after* the project's filtering. It also prints a header line on the first call and every 50 lines. Columns: `MPU,t_ms,pitch,roll,yaw,gx,gy,gz,ax,ay,az,nhiet,dy,nghieng,vz,rung,nhac`. The last four are the lift detector's `lift_tilt_deg`, `lift_vz_ms`, `lift_acc_dev` (in mg) and `robot_lifted`, for tuning its thresholds. `yaw` is printed wrapped to [0, 360) by `MPU6500_Wrap360`; per the user, nothing above 360 is printed. `yaw_angle` itself is deliberately **never wrapped**: `Turn_Angle(3600)`, `Rectangle_Test`'s absolute targets and `Gyro_Spin_Calibrate` need a continuous angle, and wrapping would make the heading error jump about 360° at the 0/360 crossing. Use `MPU6500_Wrap360(yaw_angle)` for display or N/E/S/W.
- `pitch`/`roll` come from the complementary filter;
- `yaw` is held while still;
- `gx`/`gy` are offset-corrected;
- `gz` is `yaw_rate_dps`;
- accel is in g after the 20 Hz DLPF;
- `dy` is `imu_still`.

The values are snapshotted with interrupts briefly disabled. It has no internal delay; the caller paces it. At 9600 baud a line takes about 95 ms. `print_SpeedPID()` only manages about 50 lines/s, which is too slow to show a 2 ms loop's dynamics.

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
- **Kp/Ki are now 5/150 (2026-10-10).** The previous 1.5/30 gave a feedback bandwidth of only about 6 rad/s. Evidence from 8 `PID_Test` runs at 1.5/30:
  - The right wheel tracked in every run (RMS about 6.5 mm/s, which is pure quantization).
  - The **left wheel has a mechanical tight spot once per wheel revolution.** Successive slow episodes within a run are 86–90 mm apart (circumference 91.9 mm) and sit at the same wheel angle. Speed drops to 25–45% of the setpoint even though PWM is 500–1000 above feedforward, giving an RMS of 25–44 mm/s.
  - This matches the once-per-revolution yaw spike below.
- Why 5/150: Ki/Kp = 30 ≈ 1/τ, so the closed loop is roughly first order with τ ≈ 35 ms.
- Host SIL check (`scratchpad/pidlog/sim2.c`: real `pid.c` code, motor fitted to the log, a 70° tight spot of 800 PWM on the left wheel):
  - The left RMS falls from 34 to 21 mm/s, and the minimum speed rises from 23% to 63% of the setpoint.
  - The healthy wheel is unchanged, and stays stable for τ 25–50 ms with 2 extra ticks of delay.
  - The cost is PWM jitter from quantization rising from about 12 to about 43 per tick.
- Higher gains (8/250) only reach about 71%. The tight spot has to be fixed mechanically.
- **The user fixed it mechanically and the gains were verified on 2026-10-10.** In 24 `PID_Test` runs at 5/150:
  - Both wheels behave identically, with an RMS error of 6.1–6.6 mm/s at constant setpoints, which is only quantization. No sample was off by more than 35 mm/s.
  - Step errors stay within ±2.6 mm/s. `Bam_max` is 18–32 mm/s, settling for 200↔300 takes 44–50 ms, and t_len is 44–138 ms.
  - Remaining "Vot_lo" of 10–18% and left 0→200 `t_od` of up to 142 ms are single-count spikes, not real overshoot.
  - Steady PWM is left 1094 @ 200, 1615 @ 300, −841 @ −150 and right 1157 / 1683 / −897. The integrator covers the remaining 0–65 PWM of feedforward error.
  - Mean |ΔPWM| per tick is 11–57, with a maximum of about 104.
- `MAX_SPEED_MMS` = 350 is a leftover limit from the old forward drive. The linear map suggests about 450–500 mm/s with headroom left for Ka and the PID, but raise it only after testing at around 400.
- The ±12 mm/s jitter in `speed_*_mms` is encoder quantization (4–5 counts per tick, 67 mm/s per count), not real speed variation. Raising Kp amplifies it into PWM jitter, which the motor filters out. It doesn't improve tracking on a healthy wheel, but it does speed up the rejection of load disturbances.

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
- **Gyro re-zero before every motion sequence:** `Motion_Begin()` calls `MPU6500_ZeroGyroZ()`. It averages 250 DMA samples of raw Gz (0.5 s) while the wheels are still and sets `gyro_z_offset` directly. If the wheels move or the noise exceeds `GYRO_CAL_MAX_STD_DPS`, it returns HAL_BUSY and keeps the old offset. Callers must have the robot standing still; every test waits 1 s first.
- **Yaw has no deadband:** don't reintroduce one. It ate small real heading changes and rectified vibration into drift. Standstill handling depends on the encoder speeds (`speed_*_mms` from `pid.c`). If encoder jitter keeps `imu_still` at 0 (visible as a low `DYen%` in `MPU6500_Test`), bias tracking simply never runs.
- **IMU start-up order:** the ISR only starts DMA reads once `imu_ready` is set. That happens after `MPU6500_Init()` and the blocking `MPU6500_Calibrate()`, and only if init succeeded. Before this was added, the DMA reads collided with the blocking calibration reads on `hi2c1`. Don't add blocking I2C calls after `imu_ready` = 1.
  - Calibration waits 50 ms, discards `GYRO_CAL_DISCARD` samples, and averages 500. It retries up to `GYRO_CAL_RETRY` (5) times if any axis std exceeds `GYRO_CAL_MAX_STD_DPS`, or if the Gz mean of the first half differs from the second half by more than `GYRO_CAL_MAX_DRIFT_DPS` (0.1 °/s), which means the robot is being turned slowly while it is set down. `MPU6500_ZeroGyroZ` does the same half-check. The robot must be still; this takes about 1.5–1.7 s (HAL_Delay(2) waits 2–3 ms per sample).
  - **Time limit** (2026-10-14, user report that IMU calibration "hangs"):
    - It used to retry 5 times, about 8.5 s, when the robot was held or bumped. With a stuck I2C bus it could take minutes: 550 reads × 100 ms timeout × 5.
    - Now it stops after `GYRO_CAL_MAX_MS` (3000) in total. A retry starts only if another attempt, timed from the previous one, still fits. Each read uses `GYRO_CAL_I2C_TIMEOUT_MS` (5).
    - It returns HAL_TIMEOUT. Like HAL_BUSY, that means the robot starts as lifted and recalibrates itself once set down still.
    - Sim (`MODE=CAL`): still 1.7 s OK, bumped 1.7 s TIMEOUT (one attempt), stuck bus 3.0 s TIMEOUT.
  - `MPU6500_Setup` returns HAL_BUSY/HAL_TIMEOUT when calibration never got a still reading. The robot then starts as **lifted**: motors are locked and the PB15 LED stays off until it has been set down and is still. The IMU still runs, and the Z offset corrects itself at the next standstill.
- **Wheel geometry:** distance comes from the per-wheel *effective* diameters `WHEEL_DIAMETER_L`/`_R` and `ENCODER_CPR` (700 counts per wheel revolution), which give `MM_PER_COUNT_L`/`_R`.
  - Values measured on 2026-10-09: 29.24/29.03 mm. The nominal 30 mm tires compress under the robot: 1 m by encoder was really about 970 mm. With the gyro holding heading, the encoders showed the right wheel about 0.72% smaller.
  - The L/R values came from (encoder heading − gyro heading)·W / distance, split evenly so the mean is kept. The mean needs a tape-measured distance.
  - `Kf`/`Ka` were rescaled by 30/D per wheel so the feedforward PWM is unchanged.
  - When the wheels physically change, also scale the PID gains and `MAX_SPEED_MMS` by old/new diameter, because the same PWM now produces a different speed in mm/s.
- **Accelerometer on this board:** at rest on a flat floor it reads ax +0.16, ay +0.15, az −1.17 g, so |a| = 1.19 g instead of 1.00. The chip is mounted upside down (roll ≈ 173°, consistent with `YAW_SIGN` = −1) and has accel offsets of roughly 0.15–0.17 g. Pitch/roll therefore read about 7–8° off. They aren't used for control, so this is harmless for now. If pitch/roll are ever needed, do a 6-position accel offset calibration first.
- **Hardware results from the removed `Straight_IMU_Test` (2026-10-09):**
  - UART/Bluetooth has no effect: 0 encoder pulses while transmitting, IMU noise ×0.9–1.1, TIM9 period 1995.9–2000.2 µs, `Motor_Speed_Tick` at most 38 µs (1.9% CPU).
  - Driving at 250 mm/s versus standing still, std ratios: Gz ×95–100 (about 6 °/s), Gx ×40, Gy ×155 (8.6 °/s, the robot pitches), Ax 51 mg, Az 22–28 mg.
  - Gz peaks at 2.75/5.5/8 Hz (1/2/3× wheel revolution) and 27.5 Hz (10× wheel, probably the motor or gear if the ratio is 10:1).
  - Rectification is negligible: the mean Gz while driving matches the mean at rest.
    - Caveat: that run had the gyro heading loop closed, which forces the mean of (Gz − offset) to about 0. A bias that appears only while driving would therefore be hidden in it, showing up as a real physical turn instead.
    - On 2026-10-10 the user reported the robot ending about 5° right after a single 1 m straight. Not resolved yet. The candidates are (a) a gyro bias while driving, which gives an arc with the final body heading rotated, or (b) start misalignment, which gives a straight diagonal path with the body still parallel to its start. A forward-then-reverse run on the same absolute heading tells them apart: (b) retraces its path, (a) does not.
- **Robot shake at about 2.7 Hz (one wheel revolution at 250 mm/s) is mechanical.** The gyro shows a yaw spike once per revolution that the encoders don't. In simulation, 4× the heading gains only reduced it by about 10%, so check the wheels (run-out, tire seating, loose hub) rather than retuning. The 2026-10-10 `PID_Test` logs locate the problem on the **left** wheel: a tight spot once per revolution (see the tuning data).
- **Speed variables:** the only wheel speeds are `speed_*_mms` (mm/s, filtered, in `pid.c`). The old `speed_*_ms`/`print_Velocity()` and the stale `extern`s in `stm32f4xx_it.c` are gone. `stm32f4xx_it.c` contains only the CubeMX handlers that call into HAL.
- **USB printing must never block:** the robot runs on battery without a PC.
  - `CDC_Print()` returns immediately when USB isn't enumerated. It returns after at most 20 ms when the host isn't reading (COM port closed), then drops lines instantly until the host reads again.
  - Before this fix it retried forever. `CDC_Transmit_FS` also dereferences `pClassData` without a NULL check, so when unplugged it read garbage and returned BUSY. The result was that the robot froze at the first print (e.g. after one `PID_Test`).
  - `PID_Test` deliberately calls `CDC_WaitHost()` before dumping its report. That blocks, with the motors braked, until a terminal actually opens the port, so a floor run without a cable can be read out afterwards.
  - For anything else, never call `CDC_Transmit_FS` directly; use `CDC_Print`.
- **ADC start order:** `ADC_IR_Start()` starts the ADC DMA *before* the TIM4 trigger. In the other order, a trigger can land after ADON but before DMA is enabled. That causes an overrun (OVR), and since the ADC IRQ isn't enabled, DMA requests stay blocked and `adc_value[]` freezes. `adc_value` is sized `ADC_IR_CHANNELS` (4), which must match the CubeMX `NbrOfConversion`.
