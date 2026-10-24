# QUY TRÌNH NGHIỆP VỤ & MÁY TRẠNG THÁI (BUSINESS FLOWS & STATE MACHINES)

Tài liệu này đặc tả chi tiết các quy trình nghiệp vụ cốt lõi, trình tự khởi động an toàn, các máy trạng thái điều khiển thời gian thực và các cơ chế xử lý ngoại lệ trong firmware Robot Micromouse.

---

## 1. Trình tự khởi động an toàn của hệ thống (Safe Boot Sequence)

Để đảm bảo các bộ điều khiển phản hồi không bị lỗi tín hiệu ngõ vào khi các cảm biến chưa sẵn sàng, firmware tuân thủ một trình tự khởi tạo nghiêm ngặt trước khi cho phép bánh xe chuyển động:

```mermaid
flowchart TD
    A["Cấp nguồn vi điều khiển (Power-On Reset)"] --> B["1. HAL_Init() & SystemClock_Config() (60 MHz SYSCLK)"]
    B --> C["2. Khởi tạo ngoại vi MX_*_Init() (GPIO, DMA, ADC, I2C, Timers, UART, USB)"]
    C --> D["3. ADC_IR_Start(): Kích hoạt ADC DMA trước rồi mới bật TIM4 CC4 ~5kHz"]
    D --> E["4. Motor_Init(): Cấu hình PWM 20kHz, Encoder, đưa 2 động cơ vào trạng thái PHANH"]
    E --> F["5. HAL_TIM_Base_Start_IT(&htim9): Bật ngắt chu kỳ 2ms SAU CÙNG"]
    F --> G["6. ADC_IR_EmittersOn(): Bật LED phát hồng ngoại & Chờ ổn định quang 300ms"]
    G --> H["7. MPU6500_Setup(): Khởi tạo thanh ghi + Hiệu chỉnh Gyro tĩnh (~1.2s, max 3s)"]
    
    H -->|"Thành công (HAL_OK)"| I["Bật LED PB14 (IMU OK) & Cho phép TIM9 đọc I2C DMA (imu_ready = 1)"]
    H -->|"Thất bại / Bị rung (HAL_TIMEOUT/BUSY)"| J["Tắt LED PB14, Khởi động ở trạng thái LIFTED (Khóa động cơ)"]

    I --> K["8. ModeManager_Init(): Quét bản đồ Flash Sector 7, Đặt trạng thái STATE_IDLE"]
    J --> K
    K --> L["Vào vòng lặp chính while(1): Chạy ModeManager_Update() liên tục"]
```

> [!IMPORTANT]
> **Quy tắc bất biến về thứ tự khởi động:**
> 1. `ADC_IR_Start()` phải bật DMA trước khi bật Timer Trigger `TIM4`. Nếu bật Timer trước, xung kích có thể rơi vào lúc ADC đã bật nhưng DMA chưa kịp kích hoạt, dẫn đến cờ tràn `OVR` bị set, khóa vĩnh viễn luồng DMA của ADC.
> 2. `HAL_TIM_Base_Start_IT(&htim9)` phải được gọi sau khi PWM và Encoder đã khởi tạo xong, nhưng trước `MPU6500_Setup()`. Trong quá trình hiệu chỉnh tĩnh, ngắt TIM9 vẫn chạy nhưng cờ `imu_ready` vẫn bằng 0 nên chưa phát sinh đọc DMA I2C, tránh xung đột trên bus I2C.

---

## 2. Máy trạng thái Quản lý Chế độ (Mode Manager State Machine)

Module `mode_manager.c` điều phối toàn bộ hành vi mức cao của robot thông qua enum `RobotState`. Toàn bộ máy trạng thái hoạt động dựa trên việc lưu mốc thời gian `HAL_GetTick()` và so sánh thời lượng, tuyệt đối không sử dụng hàm chờ `HAL_Delay()`.

```mermaid
stateDiagram-v2
    [*] --> STATE_IDLE

    state STATE_IDLE {
        description: "Động cơ phanh, chờ che mắt IR trái, phục vụ yêu cầu bản đồ UART"
    }

    state STATE_UI_BEEP {
        description: "Còi phát 1 tiếng bíp (250ms) xác nhận lần che vừa nhận"
    }

    state STATE_UI_WINDOW {
        description: "Đếm cửa sổ chờ 3 giây xem người dùng có che tiếp không"
    }

    state STATE_START {
        description: "Chờ 0.5s bỏ tay + xe nằm trên sàn + đo lại offset Gyro Z (0.5s)"
    }

    state STATE_EXPLORE {
        description: "Khám phá mê cung: Chạy Mode 1 hoặc Mode 3"
    }

    state STATE_MODE2_RUN {
        description: "Bứt tốc chạy nhanh tới tâm theo bản đồ tối ưu"
    }

    state STATE_GOAL_BEEP {
        description: "Còi reo 2 nhịp mừng chạm đích"
    }

    state STATE_RETURN_HOME {
        description: "Quay đầu 180° và chạy đường ngắn nhất về ô (0,0)"
    }

    state STATE_PAUSED {
        description: "Tạm dừng khẩn cấp do bị nhấc / va chạm, lưu Flash, chờ đặt lại"
    }

    state STATE_RESUME {
        description: "Đọc lại tường trước, tính lại đường từ ô chốt và tiếp tục"
    }

    state STATE_ERROR {
        description: "Phát còi lỗi 'tít tít tít' (3 nhịp dồn dập) rồi về IDLE"
    }

    %% Transitions
    STATE_IDLE --> STATE_UI_BEEP: Che mắt IR trái lần 1 (ui_count = 1)
    
    STATE_UI_BEEP --> STATE_UI_BEEP: Che tiếp trong lúc còi đang kêu (ui_count++)
    STATE_UI_BEEP --> STATE_UI_WINDOW: Còi vừa tắt
    
    STATE_UI_WINDOW --> STATE_UI_BEEP: Che thêm trong cửa sổ 3s (ui_count++)
    STATE_UI_WINDOW --> STATE_START: Hết 3s & ui_count hợp lệ (1, 2, 3)
    STATE_UI_WINDOW --> STATE_ERROR: Hết 3s & ui_count không hợp lệ (>= 4 hoặc lỗi Flash)

    STATE_START --> STATE_EXPLORE: Start_Mode1() hoặc Start_Mode3()
    STATE_START --> STATE_MODE2_RUN: Start_Mode2()

    STATE_EXPLORE --> STATE_EXPLORE: Chạm đích (Checkpoint -> Lưu Flash -> Đi tiếp)
    STATE_EXPLORE --> STATE_IDLE: Dò xong, về tới (0,0) quay Bắc -> Lưu Flash & Gửi map
    STATE_EXPLORE --> STATE_PAUSED: Bị nhấc xe hoặc Maze báo PAUSED
    STATE_EXPLORE --> STATE_ERROR: Kẹt xe, mất đường, lệch hướng > 30°

    STATE_MODE2_RUN --> STATE_GOAL_BEEP: Tới đích tâm (4 ô trung tâm)
    STATE_MODE2_RUN --> STATE_PAUSED: Bị nhấc xe / va chạm
    STATE_MODE2_RUN --> STATE_ERROR: Lỗi chuyển động

    STATE_GOAL_BEEP --> STATE_RETURN_HOME: Hết còi -> Maze_StartReturn()
    
    STATE_RETURN_HOME --> STATE_IDLE: Về tới ô (0,0) quay Bắc -> Gửi map
    STATE_RETURN_HOME --> STATE_PAUSED: Bị nhấc xe / va chạm

    STATE_PAUSED --> STATE_RESUME: Đặt xuống sàn, đứng im liên tục 3s (PAUSE_STILL_MS)
    STATE_RESUME --> STATE_EXPLORE: Khôi phục về Explore nếu trước đó đang dò
    STATE_RESUME --> STATE_MODE2_RUN: Khôi phục về Mode 2 Run
    STATE_RESUME --> STATE_RETURN_HOME: Khôi phục về Return Home

    STATE_ERROR --> STATE_IDLE: Hết còi lỗi
```

---

### 2.1 Giao diện "Nút bấm ảo" & Quy tắc chọn Chế độ

Robot không sử dụng nút nhấn cơ học để tránh rung lắc cơ khí và hỏng hóc tiếp điểm:
- **Nguyên lý nút ảo:** Mắt thu hồng ngoại phía trước bên trái (`adc_value[1]`) đóng vai trò cảm biến tiệm cận.
- **Tiêu chuẩn nhận diện một lần che (`UI_Pressed`):**
  - Mức thô $IRL > \text{UI\_IR\_PRESS\_RAW} = 2000$ liên tục trong thời gian $\ge \text{UI\_DEBOUNCE\_MS} = 30\text{ ms}$.
  - Người dùng bắt buộc phải rút tay ra ($IRL < \text{UI\_IR\_RELEASE\_RAW} = 1500$) thì hệ thống mới tái kích hoạt nhận diện lần che kế tiếp (`ui_armed = true`). Cố tình giữ tay lâu không bị tính thành nhiều lần nhấn.
- **Phản hồi âm thanh còi Buzzer:**
  - Mỗi lần che hợp lệ: còi kêu 1 tiếng dài $250\text{ ms}$ (`BUZZ_ON_MS`). Nếu người dùng che nhanh liên tiếp, các nhịp còi sẽ được đưa vào hàng đợi `ui_beeps_due` và phát cách nhau tối thiểu $300\text{ ms}$ (`UI_BEEP_GAP_MS`) để người dùng đếm rõ ràng.
- **Cửa sổ chốt chế độ (`UI_WINDOW_MS = 3000 ms`):**
  - Sau khi tiếng còi cuối cùng kết thúc, hệ thống đếm lùi 3 giây. Nếu không có thêm lần che nào, hệ thống chốt số lần che $N = \text{ui\_count}$ và tiến hành **kiểm tra an toàn tĩnh (Safety Check)** trước khi cho phép chạy:

| Số lần che | Chế độ gán | Điều kiện an toàn bắt buộc | Hành vi khi đạt chuẩn | Hành vi khi vi phạm |
| :---: | :---: | :--- | :--- | :--- |
| **1 lần** | **MODE 1** (Dò tiếp bản đồ cũ) | Flash Sector 7 phải có bản đồ hợp lệ (`Flash_LoadMap == FLASH_OK`) và cấu trúc biên mê cung đúng chuẩn (`Maze_ImportMap == true`). | Nạp bản đồ Flash vào RAM, xuất phát từ (0,0) hoặc tọa độ đã lưu, flood-fill dò tiếp các ô chưa biết. | Còi báo lỗi 3 tiếng ("tít tít tít"), in UART: *"Flash trong, chua co map"*, không chạy, về `STATE_IDLE`. |
| **2 lần** | **MODE 2** (Chạy nhanh + Về) | Đạt điều kiện Mode 1 **VÀ** bản đồ phải có đường đi từ xuất phát đến đích chỉ qua các cạnh đã biết (`FLASH_MAP_HAS_PATH` hoặc `Maze_HasPath() == true`). | Bứt tốc chạy thẳng đến đích với tốc độ cao $500\text{ mm/s}$, reo còi, quay 180°, chạy về ô xuất phát. | Còi báo lỗi 3 tiếng, in UART: *"map do, chua co duong toi dich"*, không chạy, về `STATE_IDLE`. |
| **3 lần** | **MODE 3** (Dò mê cung mới) | Luôn luôn hợp lệ. | Phát 1 tiếng bíp dài $1000\text{ ms}$ (`CONFIRM_LONG_BEEP_MS`), xóa trắng bản đồ RAM, ghi đè 1 bản ghi mới vào Flash rồi bắt đầu dò. | Không bao giờ bị từ chối. |
| **$\ge 4$ lần** | Không hợp lệ | Không thỏa mãn điều kiện giao diện. | Ngăn ngừa việc che lố tay vô tình xóa mất bản đồ. | Còi báo lỗi 3 tiếng, in UART: *"che qua 3 lan"*, về `STATE_IDLE`. |

---

## 3. Quy trình Thuật toán Giải mê cung Chạy liên tục (Continuous Motion Flood-Fill)

Điểm đột phá của hệ thống là khả năng **di chuyển liên tục, gộp nhiều ô thẳng thành một chặng duy nhất, vừa chạy vừa quét cảm biến và điều chỉnh đích đến ngay trong khi đang chuyển động**:

```mermaid
flowchart TD
    A["Trạng thái MAZE_STATE_DECIDE_PATH (Xe ở tâm ô)"] --> B["1. Quét tường trước ở cự ly gần: TB(FL,FR) > 400 -> Ghi đè bản đồ"]
    B --> C{"2. Đã chạm ô đích của giai đoạn?"}
    
    C -->|"Đúng: SEARCH_GOAL chạm tâm"| D["Đổi sang SEARCH_START: quay về (0,0) tìm đường tối ưu"]
    C -->|"Đúng: SEARCH_START về (0,0)"| E{"Đường đã biết == Đường lạc quan? (Đã tối ưu?)"}
    C -->|"Chưa tới đích"| F["3. Chạy thuật toán Flood_Fill(phase, optimistic)"]

    E -->|"Đúng hoặc đã đủ 3 vòng"| G["Hoàn tất khám phá: MAZE_STATE_FINISHED"]
    E -->|"Chưa tối ưu (< 3 vòng)"| H["Bắt đầu vòng khám phá tiếp theo -> SEARCH_GOAL"]

    F --> I["4. Plan_BestDir(): Chọn hướng có khoảng cách nhỏ nhất (Ưu tiên: Thẳng > Phải > Trái > Quay đầu)"]
    I --> J{"Hướng tối ưu so với hướng hiện tại?"}

    J -->|"Cùng hướng (Đi thẳng)"| K["Plan_StraightCells(): Đếm số ô đi thẳng liên tiếp n"]
    J -->|"Lệch hướng (Cần rẽ hoặc quay đầu)"| L{"Ô hiện tại có tường trước không?"}

    K --> M["Bắt đầu đi thẳng không chặn: Move_Straight_Begin(n * 180 mm, speed)"]
    M --> N["Trạng thái MAZE_STATE_MOVE_STRAIGHT"]

    L -->|"Có tường trước"| O["Chạy Wall_Align_Begin(): Căn vuông góc tường trước"]
    L -->|"Không có tường trước"| P["Turn_Angle_Begin(±90° hoặc 180°)"]
    O --> P

    %% Continuous Motion Loop
    N --> Q["Đọc quãng đường đã đi: dist = Move_Straight_Traveled()"]
    Q --> R{"Trục bánh tới biên vào ô k? (dist >= k*180 - 90 mm)"}
    
    R -->|"Đúng: Chưa đọc ô k"| S["Straight_SenseCell(k): Đọc mắt bên (≥170) & Mắt trước (>120)"]
    S --> T["Ghi nhận các cạnh mới vào bản đồ RAM"]
    T --> U["Flood_Fill() lại ngay lập tức"]
    U --> V["Straight_Replan(k): Từ ô k còn đi thẳng được m ô nữa không?"]
    
    V -->|"Kéo dài thêm: m > đích cũ"| W["Move_Straight_SetTarget((k + m) * 180): Xe tiếp tục giữ tốc độ"]
    V -->|"Rút ngắn lại: m < đích cũ"| X["Move_Straight_SetTarget((k + m) * 180): Xe chủ động hãm tốc vừa đủ dừng"]
    
    R -->|"Chưa tới"| Y{"Kiểm tra Motion_Status()"}
    W --> Y
    X --> Y

    Y -->|"MOTION_STATUS_WALL (Gặp tường chắn IR)"| Z["Dừng sớm: Straight_Finish(làm tròn ô) & Xác nhận tường"]
    Y -->|"MOTION_STATUS_DONE (Tới đích chặng)"| AA["Straight_Finish(): Cập nhật rob_x, rob_y"]
    
    Z --> A
    AA --> A
```

---

## 4. Quy trình Tự căn Vuông góc Tường trước (`Wall_Align`)

Chạy hoàn toàn trong ngắt thời gian thực ở chế độ `MOTION_HOLD`. Giúp triệt tiêu sai số tích lũy của con quay Gyro và định tâm lại robot trong ô:

```mermaid
flowchart TD
    A["Bắt đầu Wall_Align_Begin()"] --> B["ws = WS_IDLE: Chờ xe đứng yên 100ms & Cả 2 mắt trước >= 600"]
    B --> C{"Kiểm tra cự ly tới tường"}
    
    C -->|"Mắt gần > 1725 (Quá gần)"| D["Lùi chậm 30 mm/s về target 1700"]
    C -->|"Mắt gần < 1640 (Còn xa)"| E["Bò chậm tiến 30 mm/s tới target 1700"]
    C -->|"Trong khoảng 1640..1725"| F["Chuyển sang ws = WS_MEAS"]

    D --> B
    E --> B

    F --> G["Lấy trung bình 25 mẫu sai lệch: r = (FL - FR)/(FL + FR) - WALL_R_OFFSET"]
    G --> H{"Độ nhạy g đã biết chưa?"}

    H -->|"Chưa biết (Lần đầu)"| I["Quay thử 4° (WALL_PROBE_DEG) -> g = delta_r / delta_heading"]
    H -->|"Đã biết"| J["Cập nhật g theo phương pháp cát tuyến (Secant method)"]

    I --> K["Tính góc quay chỉnh: step = -r / g"]
    J --> K

    K --> L{"|step| < 0.5° (WALL_ALIGN_TOL_DEG)?"}
    L -->|"Chưa đạt (và chưa quá 5 lần lặp)"| M["Quay chỉnh góc step (Tối thiểu 40°/s) -> Quay lại đo WS_MEAS"]
    M --> F

    L -->|"Đạt yêu cầu (Đã vuông góc)"| N["Chuyển sang ws = WS_ZERO"]
    
    N --> O["Lấy trung bình 250 mẫu con quay (0.5s) hiệu chỉnh lại offset Gyro Z"]
    O --> P["Wall_SetHeading(): Gán lại hệ tọa độ Gyro (Hướng vuông góc tường = Hướng đích)"]
    P --> Q["Hoàn tất: ws = WS_DONE -> Wall_Align_Poll() trả về WALL_ALIGN_OK"]

    %% Timeout guard
    B -.->|"Thời gian căn > 4000ms"| R["ws = WS_FAIL: Hủy căn, giữ nguyên mốc Gyro cũ, tránh treo"]
    F -.->|"Thời gian căn > 4000ms"| R
```

---

## 5. Quy trình Cua mượt 90 độ (Smooth 90° Curve Turn)

Thay vì dừng lại ở tâm ô rồi quay tại chỗ, robot có thể thực hiện khúc cua mượt 90 độ liên tục mà không làm gián đoạn gia tốc bánh:

```mermaid
flowchart LR
    subgraph PHASE_A["PHA A: TIẾP CẬN (Approach)"]
        A1["Trục bánh ở tâm ô trước góc"] --> A2["Đi thẳng, cân giữa ô"]
        A2 --> A3["Hãm theo căn bậc 2 về v_cua"]
        A3 --> A4["Đọc mắt bên phía rẽ ở cự ly 90mm"]
        A4 -->|"Phía rẽ TRỐNG"| A5["Vào đầu cung cua (Cách tâm góc cự ly D = 81.8mm)"]
        A4 -->|"Phía rẽ CÓ TƯỜNG"| A6["Hủy cua: Phanh dừng ở tâm ô góc"]
    end

    subgraph PHASE_B["PHA B: CUNG CUA (Arc L = 140mm)"]
        B1["Độ cong: k(s) = (pi/L) * sin²(pi*s/L)"] --> B2["Vận tốc góc mẫu: w = k(s) * v"]
        B2 --> B3["Phân rã bánh: v_L/R = v ± w*W/2"]
        B3 --> B4["PD góc: Sửa theo theta_ref(s) + Sửa lệch ngang Odometry"]
        B4 --> B5["Đến hết chiều dài cung L = 140mm"]
    end

    subgraph PHASE_C["PHA C: THOÁT CUA (Exit)"]
        C1["Góc hướng đạt đúng h0 ± 90°"] --> C2["Giữ v_cua tới khi lệch < 2° và lệch ngang < 3mm"]
        C2 --> C3["Tăng tốc lại theo gia tốc 1000 mm/s²"]
        C3 --> C4["Kích hoạt lại cân giữa ô và dừng trước tường nếu có"]
        C4 --> C5["Dừng chính xác tại tâm ô sau cua"]
    end

    PHASE_A --> PHASE_B
    PHASE_B --> PHASE_C
```

---

## 6. Quy trình Giám sát Sự cố & Bảo vệ An toàn (Safety System)

Hệ thống triển khai 4 tầng bảo vệ giám sát liên tục trong ngắt cứng TIM9 (chu kỳ 2 ms):

```mermaid
flowchart TD
    A["Ngắt TIM9 (Mỗi 2ms): Check_Robot_Picked_Up()"] --> B{"1. Tầng IMU (MPU6500)"}
    B -->|"Nghiêng >20° HOẶC |Vz| >0.12m/s HOẶC Lật >30°/s trong 200ms"| E["Phát hiện NHẤC XE (PICKUP_BY_IMU)"]
    B -->|"Độ giật Jerk >1.10g hoặc Side Jerk >0.73g kèm Gz Jerk >76°/s"| F["Phát hiện VA CHẠM (PICKUP_BY_CRASH)"]

    A --> C{"2. Tầng Encoder (Khi đang chạy MODE)"}
    C -->|"Bánh quay NHANH HƠN tốc độ đặt >120 mm/s liên tục 30ms"| G["Phát hiện MẤT TẢI BÁNH (PICKUP_BY_ENCODER)"]

    A --> D{"3. Tầng Quang học IR (Khi đang đứng yên)"}
    D -->|"Vừa thấy tường rõ mà cả 4 mắt cùng tụt về nền <130 trong 60ms"| H["Phát hiện NHẤC KHỎI MÊ CUNG (PICKUP_BY_IR)"]

    E --> I["Kích hoạt MPU6500_ForceLifted()"]
    F --> I
    G --> I
    H --> I

    I --> J["1. Khóa cứng động cơ: Motor_SetPWM(0, 0) (Phanh điện)"]
    J --> K["2. Tắt đèn LED báo chạy PB15"]
    J --> L["3. Hủy chuyển động đang chạy: Các hàm Move/Turn trả về 0 (MOTION_LIFTED)"]
    
    L --> M["ModeManager_Update() phát hiện robot_lifted != 0"]
    M --> N["Chuyển sang STATE_PAUSED:
    - Maze_Pause(): Chốt tọa độ ô gần nhất & Giữ nguyên bản đồ RAM
    - In UART: 'TAM DUNG: dat lai o (x,y) mui huong ...'
    - Lưu Flash Sector 7 (Lý do FLASH_SAVE_LIFTED)
    - Chụp mốc cảm biến & Bắt đầu đếm tĩnh Still_Restart()"]
```

---

## 7. Quy trình Tạm dừng & Khôi phục Chuyển động (Pause & Resume Flow)

Khi robot bị nhấc lên khỏi sàn hoặc va chạm vào tường, người dùng có thể nhấc robot lên, đặt lại vào đúng tâm ô và hướng nhìn được in ra trên cổng UART; robot sẽ tự động tiếp tục phần hành trình dở dang mà không cần cấu hình lại:

```mermaid
flowchart TD
    A["Robot đang ở STATE_PAUSED (Động cơ phanh, chờ đặt xuống)"] --> B["Người dùng đặt robot lại đúng ô (x,y) và hướng nhìn in trên UART"]
    B --> C["Hàm Still_Holding() kiểm tra điều kiện tĩnh liên tục:
    1. Bánh xe lăn <= 1.0 mm
    2. Cả 4 mắt IR không đổi quá max(25, 8% giá trị snapshot)
    3. Độ rung gia tốc lift_acc_dev <= 4 mg (LIFT_PLACE_ACC_DEV_G)
    4. Tốc độ quay |yaw_rate_dps| <= 2.0 °/s"]

    C -->|"Có bất kỳ biến động nào (Tay còn chạm, xe còn rung)"| D["Still_Restart(): Đếm lại từ đầu 3 giây"]
    C -->|"Giữ yên tĩnh liên tục đủ 3 giây (PAUSE_STILL_MS)"| E{"Gia tốc kế xác nhận đã đặt xuống? (robot_lifted == 0)"}
    
    D --> C
    E -->|"Chưa đặt / Tay cầm trên không"| C
    E -->|"Đã đặt vững chắc trên sàn"| F["Chuyển sang STATE_RESUME"]

    F --> G["1. Maze_Resume():
    - Đọc lại vách tường trước mặt
    - Chạy lại Flood_Fill() tính toán đường đi từ ô đã chốt"]
    G --> H["2. Bật lại đèn LED PB15"]
    G --> I["3. In thông báo UART: 'CHAY TIEP'"]
    G --> J["4. Khôi phục trạng thái hoạt động trước đó (STATE_EXPLORE hoặc STATE_MODE2_RUN)"]
```
