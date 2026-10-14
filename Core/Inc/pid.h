/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Động cơ (DRV8833 kiểu đẩy/phanh) + encoder + PID vận tốc 2 bánh + bài test PID
  ******************************************************************************
  */
#ifndef PID_H
#define PID_H

#include "main.h"

// ===== PID VẬN TỐC ĐỘNG CƠ =====  (CONTROL_DT = 2ms nằm trong main.h)
// Đường kính HIỆU DỤNG từng bánh (mm): bánh cao su bị nén dưới sức nặng xe nên lăn nhỏ hơn 30mm danh định.
// Đo ngày 09/10/2026 (6 lần đi thẳng 1m): xe báo 998.8mm mà thực tế ~970mm (thiếu 3cm)
//   -> trung bình 30 x 970/998.8 = 29.13; gyro giữ thẳng mà encoder báo quay trái ~5.4 độ/m
//   -> bánh phải nhỏ hơn bánh trái ~0.72%
// Chỉnh tiếp: đi thiếu/thừa -> nhân CẢ 2 với (quãng đường thật / quãng đường xe báo);
//   lệch 2 bánh (gyro giữ thẳng mà bánh phải đi nhiều hơn x%) -> trái nhân (1 + x/200), phải nhân (1 - x/200)
#define WHEEL_DIAMETER_L      29.24f
#define WHEEL_DIAMETER_R      29.03f
#define WHEEL_DIAMETER        (0.5f * (WHEEL_DIAMETER_L + WHEEL_DIAMETER_R))   // Trung bình (để in / tham khảo)
#define ENCODER_CPR           700.0f          // Số xung encoder / 1 vòng bánh
#define MM_PER_COUNT_L        (3.14159265f * WHEEL_DIAMETER_L / ENCODER_CPR)
#define MM_PER_COUNT_R        (3.14159265f * WHEEL_DIAMETER_R / ENCODER_CPR)
#define MM_PER_COUNT          (3.14159265f * WHEEL_DIAMETER / ENCODER_CPR)    // Trung bình
#define SPEED_LPF_ALPHA       0.3f            // Hệ số lọc thông thấp vận tốc đo (0..1, nhỏ = lọc mạnh)
#define PWM_MAX               2999            // = ARR của TIM1 và TIM5
#define MAX_SPEED_MMS         350.0f          // Giới hạn vận tốc đặt (mm/s). Đo được: 300 mm/s đã cần ~72% PWM
                                              // -> tối đa ~400 mm/s, chừa lại phần PWM cho PID tăng tốc/bù tải
// ===== TEST PID (PID_Test) =====
#define PID_TEST_STAGES       5               // Số bước thử, vận tốc từng bước ở mảng pid_test_speeds[]
#define PID_TEST_STAGE_LEN    200             // Số mẫu mỗi bước (200 x 2ms = 0.4s)
#define PID_LOG_SIZE          (PID_TEST_STAGES * PID_TEST_STAGE_LEN)  // 1000 mẫu x 12 byte = 12KB RAM
#define PID_SETTLE_BAND       20.0f           // Ngưỡng coi là đã ổn định (mm/s), lớn hơn nhiễu encoder ~±12 mm/s
#define MAX_ACCEL_MMS2        2000.0f         // Gia tốc tối đa khi đổi vận tốc đặt (mm/s^2)
// Driver kiểu IN1/IN2: GPIO nối IN1, chân PWM nối IN2
// Cả IN1 và IN2 đều là PWM trên cùng 1 timer -> mỗi chu kỳ là đẩy/phanh (slow decay) ở CẢ 2 chiều
// (CubeMX: PB0 = TIM1_CH2N, PA3 = TIM5_CH4)
#define PWM_PERIOD            (PWM_MAX + 1)   // CCR = PWM_PERIOD: chân ở mức cao 100%
#define MOTOR_L_IN1_CCR       (TIM1->CCR2)    // Trái: IN1 = PB0 (TIM1 CH2N)
#define MOTOR_L_IN2_CCR       (TIM1->CCR3)    //       IN2 = PB1 (TIM1 CH3N)
#define MOTOR_R_IN1_CCR       (TIM5->CCR4)    // Phải: IN1 = PA3 (TIM5 CH4)
#define MOTOR_R_IN2_CCR       (TIM5->CCR3)    //       IN2 = PA2 (TIM5 CH3)
// Mạch này: IN1 = 0, IN2 = PWM là chạy tới -> duty dương = chạy tới
// Đổi thành -1 nếu bánh quay lùi khi duty dương
#define MOTOR_L_SIGN          (1)
#define MOTOR_R_SIGN          (1)
// Đổi thành -1 nếu xe chạy tới mà vận tốc đo ra âm
#define ENC_L_SIGN            (1)
#define ENC_R_SIGN            (1)

// ===== VÒNG NGOÀI: ĐI THẲNG (ENCODER) + GIỮ HƯỚNG / QUAY (GYRO) =====
// Kích thước xe (đo 10/10/2026): mép trong 2 bánh cách nhau 72mm, mép ngoài 80mm (bánh rộng 4mm)
//   -> tâm 2 bánh cách nhau (72 + 80) / 2 = 76mm. Xe dài 100mm, trục bánh (điểm giữa 2 bánh) cách 1 đầu xe 30mm
//   -> điểm xa trục nhất cách 70mm theo chiều dọc; quay tại chỗ quét bán kính ~sqrt(70^2 + 40^2) = 81mm
//   (nếu thân rộng 80mm tới tận đầu xe; ô mê cung 180mm, tường 12mm -> nửa lối đi 84mm: còn ~3mm)
// TRACK_WIDTH chỉ đổi tốc độ quay -> chênh tốc 2 bánh (tốc độ quay / độ mạnh sửa hướng); góc cuối do GYRO quyết định,
// sai vài mm không làm lệch góc. Gyro đặt ở đâu trên xe cũng đo cùng 1 tốc độ quay (vật rắn)
#define TRACK_WIDTH_MM        76.0f           // Khoảng cách giữa tâm 2 bánh (mm)
                                              // (chỉ ảnh hưởng tốc độ quay/sửa hướng, góc cuối cùng do gyro quyết định)
// Hướng xe = YAW_SCALE * yaw_angle, dương = quay trái. Chiều của yaw chỉnh bằng YAW_SIGN trong mpu6500.h
// (sai dấu: giữ hướng bị ngược, xe lệch hướng > MOTION_ABORT_DEG là tự dừng và báo lỗi)
// Hiệu chỉnh độ nhạy gyro (MPU6500 sai số tới +-3%: 4 lần quay 90 có thể lệch thật tới ~10 độ dù xe tưởng đúng).
// Cách chỉnh: chạy Rectangle_Test, đo góc THẬT xe đã quay tổng cộng (4 lần quay phải = 360 nếu đúng)
//   YAW_SCALE mới = YAW_SCALE cũ x (góc thật / 360). Ví dụ xe về chỗ cũ mà mũi còn lệch trái 7 độ
//   (mới quay thật 353) -> YAW_SCALE = 1.0 x 353/360 = 0.981
// Hoặc TỰ ĐỘNG: Gyro_Spin_Calibrate(10) - xe tự quay 10 vòng, lấy tường làm mốc, tính và áp dụng ngay
// (biến yaw_scale), in số mới để chép vào đây (mất khi reset nếu không chép)
#define YAW_SCALE             1.0f
// Gyro_Spin_Calibrate: quay tại chỗ, mỗi lần mũi xe đối diện tường thì IR trước (FL+FR) đạt đỉnh = đúng 1 vòng
// thật. So góc gyro giữa đỉnh vòng đầu và vòng cuối với N x 360 độ -> độ nhạy gyro (quán tính không ảnh hưởng:
// chỉ lấy vị trí các đỉnh lúc đang quay đều, không dùng chỗ dừng)
#define SPIN_CAL_DPS          150.0f          // Tốc độ quay (độ/s)
#define SPIN_CAL_DECIM        5               // Ghi 1 mẫu / 5 chu kỳ 2ms (10ms ~ 1.5 độ ở 150 độ/s)
#define SPIN_CAL_MAX          3000            // Số mẫu tối đa (12 byte/mẫu = 36KB, đủ cho 10 vòng)
#define SPIN_CAL_MIN_CONTRAST 40.0f           // Đỉnh IR phải cao hơn nền ít nhất chừng này (số ADC) mới tin
#define SPIN_CAL_WINDOW_DEG   45.0f           // Tìm đỉnh vòng sau trong +-45 độ quanh chỗ dự kiến
#define MOVE_SPEED_MMS        250.0f          // Vận tốc đi thẳng mặc định (Rectangle_Test, ví dụ trong main)
#define MOVE_DECEL_MMS2       1000.0f         // Gia tốc hãm dùng để tính điểm giảm tốc (< MAX_ACCEL_MMS2 để bánh kịp bám)
#define MOVE_TOL_MM           2.0f            // Sai số quãng đường chấp nhận
// PD GIỮ HƯỚNG (gyro MPU6500): dùng khi đi thẳng và khi đứng giữ hướng (Move_Straight với tốc độ 0).
// Ra tốc độ sửa hướng w (độ/s) -> 2 bánh +-w*TRACK_WIDTH/2:
//   w = KP*lệch - KD*tốc độ quay (gyro, lọc thông thấp HEADING_D_TAU_S)
//   P kéo về hướng đích, D hãm lúc đang quay về -> không vọt lố. Không dùng I: lệch xác lập do 2 bánh
//   không đều = (tốc độ quay gây lệch) / KP, rất nhỏ (bánh lệch 1% @250mm/s: ~0.1 độ)
// Lịch sử: 09/10/2026 bỏ D vì xe giật: gyro Z rung ~6 độ/s (do chỗ kẹt bánh trái, nay đã sửa) đi thẳng
// vào vận tốc đặt mỗi 2ms, khâu Ka của PID vận tốc nhân thành PWM nhảy. Nay D dùng tốc độ quay ĐÃ LỌC.
// 10/10/2026 P 10 -> PD 20/0.5 (mô phỏng code thật, động cơ khớp PID_Test sau khi sửa bánh):
//   va lệch 5 độ giữa đường: đỉnh 4.3 -> 3.1 độ, về +-0.5 độ sau 240 -> 150 ms, không vọt lố;
//   bánh trái to hơn 1%: lệch ngang 3.3 -> 1.7 mm/m; PWM thay đổi mỗi 2ms 49 -> 51 (gyro rung 6 độ/s: 66)
//   Đứng giữ hướng, xoay tay 3..270 độ: về trong 0.02..1 s, lệch cuối < 0.25 độ, không rung tại chỗ
// LƯU Ý: PD chỉ giữ đúng hướng GYRO BÁO. Xe lệch đều mà gyro báo thẳng (vd lệch dần sau mỗi lần quay
//   180/360 độ) là sai ĐỘ NHẠY gyro -> chỉnh YAW_SCALE (Gyro_Spin_Calibrate), tăng KP/KD không sửa được
#define HEADING_KP            20.0f           // (độ/s) / độ lệch
#define HEADING_KD            0.5f            // (độ/s) / (độ/s)
#define HEADING_D_TAU_S       0.02f           // Lọc thông thấp tốc độ quay cho khâu D (s)
#define HEADING_MAX_DPS       90.0f           // Giới hạn tốc độ sửa hướng khi đi thẳng
// ĐỨNG GIỮ HƯỚNG (Move_Straight tốc độ 0 hoặc quãng đường 0): bị xoay (tay, va chạm) thì tự quay về hướng cũ.
// Bánh đứng yên cần ~470 PWM mới quay -> sửa từng chút sẽ rung tại chỗ. Nên: lệch > HOLD_START_DEG mới
// sửa, quay ít nhất HOLD_MIN_DPS, vào trong HOLD_TOL_DEG thì phanh đứng yên
#define HOLD_START_DEG        2.0f            // Lệch quá mức này mới bắt đầu quay về
#define HOLD_TOL_DEG          0.7f            // Về trong mức này thì phanh, ngừng sửa
#define HOLD_MIN_DPS          40.0f           // Tốc độ quay tối thiểu khi đang sửa
#define HOLD_MAX_DPS          180.0f          // Tốc độ quay tối đa khi quay về
#define TURN_SPEED_DPS        300.0f          // Tốc độ quay tối đa khi quay tại chỗ (độ/s)
#define TURN_DECEL_DPS2       1500.0f         // Gia tốc góc hãm dùng để tính điểm giảm tốc khi quay
#define TURN_TOL_DEG          1.0f            // Sai số góc quay chấp nhận
#define MOTION_STOP_MMS       10.0f           // 2 bánh chậm hơn mức này ...
#define MOTION_SETTLE_MS      60              // ... liên tục chừng này ms (và đã tới đích) thì coi là xong 1 đoạn
#define MOTION_ABORT_DEG      30.0f           // Lệch hướng vượt quá mức này (sai YAW_SIGN, bị va/kẹt) -> dừng
#define MOTION_TIMEOUT_MS     5000            // Mỗi đoạn tối đa: chừng này ms + 2 lần thời gian chạy hết đoạn ở tốc độ tối đa

// ===== TƯỜNG (IR, adc.h): DỪNG TRƯỚC TƯỜNG + TRÁNH TƯỜNG BÊN + TỰ CĂN VUÔNG GÓC =====
// Mọi giá trị IR là thang chuẩn ĐÃ LỌC (IR_*_Filt): trước = thang IRFR, bên = thang IRR. IR lớn = gần tường.
// ĐỨNG GIỮ HƯỚNG (Move_Straight tốc độ 0 / quãng đường 0) khi wall_guard = 1, chạy nền trong ngắt:
//   - mắt trước > WALL_STOP_IR: lùi chậm WALL_CREEP_MMS (vẫn giữ hướng) tới dưới WALL_TARGET_IR
//   - CÓ TƯỜNG = cả 2 mắt trước >= WALL_PRESENT_IR (600). Đứng yên WALL_SETTLE_MS: còn xa thì tiến chậm tới
//     ~WALL_TARGET_IR rồi dừng, tự căn vuông góc, đo lại offset gyro Z, ĐẶT LẠI HƯỚNG MPU (hướng vuông góc với tường
//     = hướng đang giữ). Bị xoay tay -> quay về rồi căn lại; tường bị xoay đi > WALL_REALIGN_DEG -> căn lại;
//     đưa tường mới tới trước mặt -> căn theo tường đó
// Move_Straight đi tới (dist > 0, tốc độ > 0) khi wall_guard = 1 (đi xong mà trước mặt có tường cũng tiến tới
// WALL_TARGET_IR rồi căn như dưới):
//   - mắt trước (lớn hơn của FL, FR) vượt WALL_SLOW_IR: giảm tốc dần, tới WALL_TARGET_IR còn WALL_APPROACH_MMS
//     (giảm theo 1/sqrt(IR) ~ khoảng cách nếu IR ~ 1/d^2 -> gia tốc hãm gần đều)
//   - mắt trước >= WALL_TARGET_IR: dừng -> Wall_Align() -> Move_Straight trả về 2
//   - mắt bên > WALL_SIDE_IR: lái lệch ra xa tường đó (tối đa WALL_SIDE_MAX_DEG), về dưới ngưỡng thì đi lại song song
//     (hướng đích không đổi -> hệ hướng tuyệt đối vẫn đúng)
// CẦN ĐỦ CHỖ HÃM: từ WALL_SLOW_IR tới WALL_TARGET_IR xe phải có ~v^2/(2*1500) mm (250 mm/s: ~20mm, 400: ~50mm).
//   Chưa đo IR theo mm -> chạy thử 250 mm/s trước; xe đâm/dừng quá gần thì giảm WALL_SLOW_IR (thấy tường sớm hơn)
// 14/10/2026 (người dùng chọn): căn ở 1700, lùi khi > 2000 (trước: 1100 / 1200). LƯU Ý: IRFR ~1700 là chỗ IRFL bắt đầu
// vào "gối" gần bão hòa (adc.c, lần đo 5: thô ~3775, > ~1850 bão hòa hẳn) -> độ chính xác căn tường phụ thuộc bảng ở
// đúng chỗ khó nhất; căn lệch thì đo r lúc vuông góc thật ở 1700 ghi vào WALL_R_OFFSET, hoặc hạ đích về ~1600
#define WALL_STOP_IR          2000.0f         // GIỚI HẠN: không đi gần tường trước hơn mức này
#define WALL_TARGET_IR        1700.0f         // Dừng / đứng cách tường trước ở mức này (< WALL_STOP_IR)
#define WALL_SLOW_IR          300.0f          // Bắt đầu giảm tốc. > mức không có tường (FR ~20-60, FL quy đổi ~80-110)
#define WALL_APPROACH_MMS     80.0f           // Vận tốc lúc tới WALL_TARGET_IR (phanh từ đây trôi ~2-3mm)
// Phanh SỚM theo tốc độ tăng của IR: dừng khi IR + (IR tăng / s) x WALL_STOP_LEAD_S >= WALL_TARGET_IR.
// Đoạn trôi khi phanh (mm) x độ dốc IR (/mm) = IR tăng thêm -> tự đúng với mọi độ dốc IR, không cần đổi IR ra mm
#define WALL_STOP_LEAD_S      0.04f           // ~ thời gian phanh hiệu dụng (~30ms) + trễ bộ lọc IR (~9ms)
#define WALL_RATE_TAU_S       0.01f           // Lọc thông thấp tốc độ tăng IR (s)
#define WALL_SIDE_IR          1200.0f         // Tường bên gần hơn mức này thì lái ra xa
#define WALL_SIDE_DEG_PER_IR  0.1f            // Lệch hướng ra xa tường (độ) cho mỗi đơn vị IR vượt WALL_SIDE_IR
#define WALL_SIDE_MAX_DEG     10.0f           // Lệch hướng tối đa khi tránh tường bên
// (mô phỏng: xe đi xiên 3 độ vào tường bên: 0.03 -> IR bên lên ~1305, 0.1 -> ~1235, không dao động; tắt -> đâm)
// CÂN GIỮA Ô khi đi thẳng tới (wall_center = 1), đồng thời vẫn giữ hướng gyro. Mắt bên lúc xe đứng GIỮA Ô (người dùng
// đo 14/10/2026): IRL thô 240, IRR 280. Lệch ngang ước lượng theo IR ~ 1/d^2 (đúng thứ nguyên khoảng cách):
//   tường trái: eL = 1 - sqrt(L0 / L), tường phải: eR = sqrt(R0 / R) - 1  (= lệch / khoảng cách tới tường, dương = lệch TRÁI)
//   2 bên có tường: e = (eL + eR) / 2; 1 bên: theo bên đó; không có tường: chỉ giữ hướng gyro
// Góc lái thêm -WALL_CENTER_KP x e (độ, kẹp WALL_CENTER_MAX_DEG) cộng vào lệnh lái của PD giữ hướng; hướng đích
// gyro KHÔNG đổi (hệ hướng tuyệt đối giữ nguyên). Lệch ngang hội tụ theo quãng đường đi, gần như không phụ thuộc tốc độ
#define WALL_CENTER_L_IR      240.0f          // IRL THÔ (cột IRL của print_LED) khi xe ở giữa ô
#define WALL_CENTER_R_IR      280.0f          // IRR khi xe ở giữa ô
// CÓ / KHÔNG CÓ TƯỜNG BÊN (người dùng đặt 14/10/2026): IRL thô hoặc IRR (số của print_LED, đã lọc) DƯỚI mức này = bên
// đó KHÔNG có tường, từ mức này trở lên = có tường. Dùng chung cho cân giữa ô và giải mê cung (MAZE_SIDE_WALL_IR).
// Giữa ô đọc 240 / 280, không tường ~40-80 -> xe lệch về phía bên kia ~10-20mm thì tường bên này coi như mất
#define WALL_SIDE_PRESENT_IR  170.0f
#define WALL_CENTER_KP        60.0f           // Độ lái thêm cho lệch tương đối 1.0 (lệch 5% khoảng cách -> 3 độ)
#define WALL_CENTER_MAX_DEG   5.0f            // Góc lái thêm tối đa
#define WALL_CENTER_TAU_S     0.02f           // Lọc thông thấp lệch ngang (s)
#define WALL_CENTER_FULL_MMS  150.0f          // Dưới tốc độ này góc lái thêm giảm dần theo tốc độ (đứng: 0)
#define WALL_CENTER_FRONT_IR  600.0f          // Tường trước gần hơn mức này thì thôi cân: mắt bên thấy cả tường trước
                                              // (log 14/10: không tường bên, IRR ~40 -> ~90 khi IRFR 1000, ~140 khi 1700)
// TỰ CĂN VUÔNG GÓC (trong ngắt, chế độ đứng giữ hướng; Wall_Align chờ nó xong): 2 mắt trước đã quy đổi cùng
// thang -> vuông góc khi FL = FR.
//   Lệch r = (FL - FR) / (FL + FR). Độ nhạy g = r thay đổi / độ quay và CHIỀU của nó phụ thuộc cách gắn mắt
//   (song song / chụm vào / xòe ra) -> KHÔNG đoán: lần căn đầu quay thử WALL_PROBE_DEG (gyro đo góc thật) để đo g,
//   rồi quay -r/g độ, đo lại, lặp tới khi còn lệch < WALL_ALIGN_TOL_DEG (g được cập nhật theo cát tuyến, nhớ cho lần sau)
// Độ chính xác bị giới hạn bởi bảng quy đổi FL -> FR: xe này g ~ 0.0032 / độ ở IR ~1100 (log 19:20: lệch phải ~5 độ
// <-> r +0.016) -> 2 mắt lệch 1% ~ căn sai ~1.5 độ. Đặt xe VUÔNG GÓC THẬT (áp ê-ke) ở IR ~1100, đọc cột r của
// print_LED rồi ghi vào WALL_R_OFFSET để bù (căn tường luôn diễn ra ở ~WALL_TARGET_IR nên chỉ cần đúng ở đó)
#define WALL_PRESENT_IR       600.0f          // CHỈ coi là có tường trước khi CẢ 2 mắt trước >= mức này
#define WALL_R_OFFSET         0.0f            // r lúc xe vuông góc thật (bù sai số bảng quy đổi)
#define WALL_DIST_TOL_IR      25.0f           // Trước khi căn: mắt trước cao hơn WALL_TARGET_IR quá mức này thì lùi về đích
                                              // (quay căn đưa 1 mắt lại gần thêm ~4%, vẫn dưới WALL_STOP_IR)
#define WALL_DIST_START_IR    60.0f           // Trước khi căn: mắt trước gần hơn thấp hơn WALL_TARGET_IR quá mức này
                                              // thì tiến tới đích (dừng sớm theo tốc độ tăng IR, WALL_STOP_LEAD_S)
#define WALL_CREEP_MMS        30.0f           // Vận tốc tiến / lùi khi chỉnh khoảng cách tới tường
#define WALL_APPROACH_MAX_MM  200.0f          // Tiến quá chừng này mà chưa tới đích (tường lùi theo?) -> thôi căn
#define WALL_SETTLE_MS        100             // Đứng yên liên tục chừng này ms mới đo IR để căn
#define WALL_REALIGN_DEG      2.0f            // Đã căn xong mà tường lệch đi quá mức này (ước lượng) -> căn lại
#define WALL_TURN_TIMEOUT_MS  1500            // 1 lần quay chỉnh quá lâu (kẹt bánh) -> thôi căn
// GIỚI HẠN THỜI GIAN 1 lần căn (người dùng yêu cầu 14/10/2026: căn bị "treo"): thấy tường mà sau chừng này ms vẫn chưa
// xong (tiến / lùi về đích, chờ đứng yên, quay thử, quay chỉnh, đo lại gyro) -> thôi căn (WS_FAIL): giữ hướng đang giữ,
// MPU không đổi. Đứng giữ hướng thì chỉ thử lại khi mất tường rồi có lại; Wall_Align trả về 0. Sim: căn bình thường
// 1-1.5s, phải bò tới tường ~30mm rồi căn ~2.5-3s (2500 đã cắt mất trường hợp này)
#define WALL_ALIGN_MAX_MS     4000
#define WALL_ALIGN_TIMEOUT_MS (WALL_ALIGN_MAX_MS + 500)   // Wall_Align chờ tối đa (dự phòng, ngắt tự thôi trước)
#define WALL_PROBE_DEG        4.0f            // Góc quay thử để đo độ nhạy g (chỉ lần đầu, hoặc sau 1 lần căn hỏng)
#define WALL_MIN_GAIN         0.002f          // |g| nhỏ hơn (r đổi < 0.2% / độ) -> không phải tường phẳng, không căn
#define WALL_ALIGN_TOL_DEG    0.5f            // Còn lệch (ước lượng) dưới mức này thì coi là vuông góc
#define WALL_ALIGN_ACCEPT_DEG 1.0f            // ... hoặc dưới mức này mà lần quay vừa rồi đã vượt qua (quay nhỏ hơn không được)
#define WALL_TURN_TOL_DEG     0.3f            // Sai số mỗi lần quay chỉnh (gyro)
#define WALL_TURN_COAST_S     0.03f           // Quay chỉnh: phanh sớm (tốc độ quay x thời gian này) bù đoạn trôi khi phanh
#define WALL_ALIGN_MAX_DEG    20.0f           // Không quay quá chừng này so với lúc bắt đầu căn
#define WALL_ALIGN_ITERS      5               // Số lần quay chỉnh tối đa
#define WALL_MEAS_N           25              // Đo IR: trung bình 25 mẫu cách nhau 2ms
#define WALL_REZERO_GYRO      1               // 1: vuông góc rồi thì đo lại offset gyro Z (0.5s, xe đứng yên) trước khi đặt hướng

// Bộ PID (dùng cho vòng vận tốc động cơ)
typedef struct {
    float Ks;            // Feedforward bù ma sát: PWM tối thiểu để bánh bắt đầu quay (cộng theo chiều)
    float Kf;            // Feedforward: PWM ước lượng cho 1 mm/s vận tốc đặt
    float Ka;            // Feedforward gia tốc: PWM cho 1 mm/s^2 (bù quán tính, = Kf * hằng số thời gian động cơ)
    float Kp;
    float Ki;
    float Kd;
    float integral;      // Tổng tích phân đã nhân Ki (đơn vị PWM)
    float prev_measure;  // Giá trị đo lần trước, dùng cho khâu D
    float prev_setpoint; // Vận tốc đặt lần trước, dùng tính gia tốc cho Ka
    float out_min;
    float out_max;
} PID_TypeDef;
// 1 mẫu ghi đáp ứng bước (mm/s và PWM, làm tròn thành số nguyên)
typedef struct {
    int16_t ref_l, meas_l, pwm_l;
    int16_t ref_r, meas_r, pwm_r;
} PIDLog_TypeDef;

extern PID_TypeDef pid_speed_left, pid_speed_right;
extern uint8_t motor_hw_ok;
extern volatile float target_speed_left, target_speed_right;
extern float ref_speed_left, ref_speed_right;
extern volatile uint8_t motor_pid_enable;
extern float speed_left_mms, speed_right_mms;
extern int32_t pwm_left, pwm_right;

// ===== KHỞI ĐỘNG (luồng chính) =====
void Motor_Init(void);                    // Bật PWM 4 kênh + encoder, kiểm tra chân, phanh 2 động cơ

// ===== ĐIỀU KHIỂN (luồng chính) =====
void Motor_SetSpeed(float left_mms, float right_mms);
void Motor_Stop(void);
void Motor_SetPWM(int32_t left, int32_t right);   // Duty trực tiếp (test tay khi PID tắt)

// ===== GỌI TRONG NGẮT =====
void Motor_Speed_Tick(void);              // Ngắt TIM9 mỗi 2ms: đọc encoder + chạy PID vận tốc

// ===== PID =====
float PID_Compute(PID_TypeDef *pid, float setpoint, float measure, float dt);
void PID_Reset(PID_TypeDef *pid, float measure);
float Ramp(float current, float target, float max_step);

// ===== VÒNG NGOÀI: ĐI ĐÚNG QUÃNG ĐƯỜNG + GIỮ HƯỚNG / QUAY BẰNG GYRO =====
// Các hàm dưới CHỜ tới khi xe tới đích và dừng hẳn (gọi ở luồng chính, không gọi trong ngắt).
// Trả về 1: xong, 0: bị hủy (lệch hướng quá MOTION_ABORT_DEG / quá MOTION_TIMEOUT_MS / IMU, động cơ chưa sẵn sàng)
extern float dist_left_mm, dist_right_mm; // Quãng đường cộng dồn từ vận tốc encoder (mm)
extern float odo_x_mm, odo_y_mm;          // Vị trí ước lượng (encoder + gyro), gốc = lúc gọi Motion_Begin()
void    Motion_Begin(void);               // Lấy vị trí + hướng hiện tại làm gốc (xe đứng yên)
// Đi thẳng dist_mm (âm = lùi) với tốc độ tối đa speed_mms (giới hạn MAX_SPEED_MMS), giữ hướng bằng gyro.
// speed_mms = 0 hoặc dist_mm = 0: ĐỨNG GIỮ HƯỚNG - không đi, KHÔNG CHỜ (trả về ngay), ngắt TIM9 giữ mãi tới lệnh
// chuyển động khác (Move_Straight đi / Turn_Angle / Motor_SetSpeed / Motor_Stop). Xoay tay lệch đi thì xe tự
// quay về hướng cũ (theo đường ngắn nhất). Hướng giữ = hướng đích hiện tại (Motion_Begin() trước = hướng lúc đó).
// Kèm tường trước: lùi khi IR > WALL_STOP_IR, thấy tường thì tự căn vuông góc + đặt lại hướng MPU (xem khối WALL_).
// Đang giữ rồi thì gọi lại không làm gì -> gọi lặp trong while(1) được.
// Đi tới mà gặp tường trước giữa đường, hoặc đi đủ quãng đường mà trước mặt có tường (cả 2 mắt >= WALL_PRESENT_IR)
// (wall_guard = 1): tiến / lùi tới WALL_TARGET_IR, tự căn vuông góc, đặt lại hướng MPU rồi trả về 2
// (căn không được: 0, kết quả "dung truoc tuong nhung khong can duoc"). Đi đủ quãng đường, không có tường: 1
uint8_t Move_Straight(float dist_mm, float speed_mms);
// TỰ CĂN THEO TƯỜNG TRƯỚC VÀ CHỜ XONG (xe đứng yên, mũi hướng vào tường, 2 mắt trước cùng thấy tường): chạy đúng chế
// độ đứng giữ hướng ở trên cho tới khi căn xong: lùi về WALL_TARGET_IR nếu quá gần, quay tới khi FL = FR, đo lại
// offset gyro Z, ĐẶT LẠI HƯỚNG: hướng vuông góc với tường = hướng đích lúc gọi (hướng xe được lệnh đi) -> hệ hướng
// tuyệt đối được sửa theo tường, lệnh quay sau đó tính từ hướng đúng. Xong thì phanh đứng yên.
// Trả về 1: xong, 0: không có tường / căn không được (hướng đích giữ nguyên, MPU không đổi)
uint8_t Wall_Align(void);
extern uint8_t wall_guard;                // 1 (mặc định): dừng trước tường + tránh tường bên + căn tường khi đứng, 0: tắt
extern uint8_t wall_center;               // 1 (mặc định): đi thẳng tới thì cân giữa ô theo tường bên (WALL_CENTER_), 0: tắt
// ĐI THẲNG KHÔNG CHỜ (giải mê cung chạy liên tục, solving_maze.c): như Move_Straight nhưng trả về ngay; đổi đích
// khi đang chạy (kéo dài: chạy tiếp không giảm tốc, rút ngắn: hãm vừa kịp dừng ở đích mới). Không tự căn tường
#define MOTION_STATUS_RUNNING 0U              // Đoạn đang chạy
#define MOTION_STATUS_DONE    1U              // Tới đích, đã đứng yên
#define MOTION_STATUS_WALL    2U              // Dừng sớm vì tường trước (IR >= WALL_TARGET_IR)
#define MOTION_STATUS_FAIL    3U              // Hủy: lệch hướng / nhấc xe / chưa sẵn sàng / bị lệnh khác cắt
uint8_t Move_Straight_Begin(float dist_mm, float speed_mms);   // 0: không chạy được
void    Move_Straight_SetTarget(float dist_mm);                // Quãng đường đích mới (từ lúc Begin)
float   Move_Straight_Traveled(void);                          // Quãng đường đã đi từ lúc Begin (mm)
uint8_t Motion_Status(void);                                   // MOTION_STATUS_*
uint8_t Turn_Angle(float deg);            // Quay tại chỗ: dương = trái, âm = phải
uint8_t Turn_Left(float deg);
uint8_t Turn_Right(float deg);
float   Robot_Heading(void);              // Hướng xe (độ), dương = trái, không quấn về +-180
// NHẤC / ĐẶT XE (tự động, xem mpu6500.h): nhấc lên thì mọi lệnh chạy bị hủy / từ chối (trả 0), động cơ phanh;
// đặt xuống nằm yên ~1s thì hướng lúc đặt = góc 0, LED PB15 sáng. Lệnh chạy dở KHÔNG tự chạy tiếp.
void    Robot_WaitPlaced(void);           // Chờ tới khi xe nằm trên sàn (đang trên sàn thì trả về ngay)
void    Motion_OnLifted(void);            // Gọi trong ngắt IMU khi phát hiện bị nhấc (không gọi từ main)
void    Motion_OnPlaced(void);            // Gọi trong ngắt IMU khi xe vừa đặt xuống (không gọi từ main)
extern float yaw_scale;                   // Độ nhạy gyro đang dùng (khởi đầu = YAW_SCALE, Gyro_Spin_Calibrate sửa)

// ===== TEST / DEBUG (luồng chính) =====
void PID_Test(uint8_t print_raw);
void print_SpeedPID(void);
// Đi hình chữ nhật: thẳng length -> quay phải 90 -> thẳng width -> quay phải ... (4 cạnh) về chỗ cũ, rồi in báo cáo
void Rectangle_Test(float length_mm, float width_mm);
// Đi thẳng dist_mm (âm = lùi) với tốc độ speed_mms, giữ hướng bằng gyro, rồi in báo cáo
// (quãng đường từng bánh, lệch hướng, lệch ngang)
void Straight_Test(float dist_mm, float speed_mms);
// TỰ HIỆU CHỈNH ĐỘ NHẠY GYRO: đặt xe cách tường 3-10cm, mũi hướng vào tường (hoặc trong 1 ô mê cung).
// Gửi 1 ký tự qua UART -> xe quay trái `turns` vòng (2..10), tìm đỉnh IR mỗi vòng, tính YAW_SCALE + TRACK_WIDTH
// hiệu dụng, ÁP DỤNG NGAY rồi tự quay về hướng ban đầu. In báo cáo qua UART (HC-05)
void Gyro_Spin_Calibrate(uint8_t turns);

#endif /* PID_H */
