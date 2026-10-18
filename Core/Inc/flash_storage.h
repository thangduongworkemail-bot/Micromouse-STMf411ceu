/**
  ******************************************************************************
  * @file    flash_storage.h
  * @brief   Lưu bản đồ mê cung vào Flash nội, SECTOR 7 (0x0806 0000 - 0x0807 FFFF, 128 KB) - còn sau khi tắt nguồn
  *
  * Linker (STM32F411CEUX_FLASH.ld) chỉ cho chương trình dùng sector 0-6 (384K) -> nạp code không xóa bản đồ.
  * Thao tác ghi: Unlock -> (Erase sector 7 nếu cần) -> Program từng word 32 bit -> Lock -> đọc lại so sánh.
  *
  * GHI NỐI TIẾP (wear leveling): sector chứa nhiều bản ghi FlashMapData liên tiếp. Mỗi lần lưu ghi bản mới vào ô trống
  * kế tiếp (sequence + 1), CHỈ XÓA sector khi đã đầy (~468 lần lưu mới xóa 1 lần). Đọc = bản hợp lệ có sequence lớn nhất.
  *   - Tuổi thọ: Flash chịu ~10 000 lần xóa / sector -> thực tế không bao giờ hết
  *   - Mất điện giữa lúc ghi: bản đang ghi sai checksum -> bỏ qua, vẫn còn bản trước (xóa-rồi-ghi thì mất cả 2)
  *   - Xóa sector mất ~1-2 s: hiếm khi xảy ra (chỉ khi đầy / Flash_EraseMap)
  *   FLASH_ERASE_EVERY_SAVE = 1: xóa sector trước MỌI lần ghi (đúng trình tự Unlock-Erase-Write-Lock, mỗi lần lưu ~1-2 s)
  *
  * CHỈ GỌI KHI XE ĐỨNG YÊN, ĐỘNG CƠ ĐÃ PHANH: STM32F411 chỉ có 1 bank Flash, lúc xóa / ghi CPU không đọc được lệnh từ
  * Flash -> mọi ngắt (PID 2ms, IMU, SysTick) bị hoãn tới khi xong (ghi 1 bản ~ vài ms, xóa sector ~1-2 s), timer PWM
  * vẫn xuất mức cũ. Chỉ gọi từ luồng chính (không gọi trong ngắt).
  ******************************************************************************
  */
#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "solving_maze.h"           /* MAZE_CELLS */

/* ===== VÙNG NHỚ ===== */
#define FLASH_MAP_SECTOR          7U            /* = FLASH_SECTOR_7 (HAL) */
#ifndef FLASH_MAP_ADDR
#define FLASH_MAP_ADDR            0x08060000UL  /* Địa chỉ bắt đầu sector 7 của STM32F411 */
#endif
#define FLASH_MAP_BYTES           (128UL * 1024UL)
#define FLASH_MAP_MAGIC           0xA5A5B6B6UL  /* Có bản đồ hợp lệ (Flash trống đọc ra 0xFFFFFFFF) */
#define FLASH_MAP_VERSION         1U            /* Đổi cấu trúc FlashMapData thì tăng -> bản cũ bị bỏ qua */
#ifndef FLASH_ERASE_EVERY_SAVE
#define FLASH_ERASE_EVERY_SAVE    0             /* 1: xóa sector trước mỗi lần ghi (chậm, mòn Flash nhanh hơn) */
#endif

/* Lý do lưu (FlashMapData.reason) */
#define FLASH_SAVE_NEW            0U            /* MODE 3 bắt đầu: bản đồ trống (đè bản đồ mê cung cũ) */
#define FLASH_SAVE_GOAL           1U            /* Đang dò map, chạm đích */
#define FLASH_SAVE_EXPLORED       2U            /* Dò map xong: về lại xuất phát, đường ngắn nhất đã chắc chắn */
#define FLASH_SAVE_LIFTED         3U            /* Bị nhấc xe giữa chừng */
#define FLASH_SAVE_ERROR          4U            /* Lỗi: kẹt bánh, lệch hướng, quá giờ, không có đường */

/* Cờ (FlashMapData.flags), tính lúc lưu */
#define FLASH_MAP_HAS_PATH        0x01U         /* Đã có đường xuất phát -> đích chỉ qua cạnh đã biết (chạy MODE 2 được) */
#define FLASH_MAP_EXPLORED        0x02U         /* Đường đó đã chắc chắn ngắn nhất (dò xong) */

/* Bản ghi lưu trong Flash. Kích thước bội số 4 (ghi theo word 32 bit) */
typedef struct
{
    uint32_t magic_word;                        /* FLASH_MAP_MAGIC */
    uint16_t version;                           /* FLASH_MAP_VERSION */
    uint16_t record_size;                       /* sizeof(FlashMapData) */
    uint32_t sequence;                          /* Số thứ tự lần lưu: lớn nhất = mới nhất */
    uint8_t  maze_map[MAZE_CELLS];              /* Bản đồ (byte ô x*16+y: bit 0..3 tường N E S W, 4..7 đã biết) */
    uint8_t  last_x;                            /* Ô cuối cùng của xe trước khi lưu (trạng thái dang dở) */
    uint8_t  last_y;
    uint8_t  last_heading;                      /* 0 Bắc 1 Đông 2 Nam 3 Tây */
    uint8_t  phase;                             /* MazePhase lúc lưu */
    uint8_t  rounds;                            /* Số vòng khám phá đã chạy */
    uint8_t  reason;                            /* FLASH_SAVE_* */
    uint8_t  flags;                             /* FLASH_MAP_* */
    uint8_t  reserved;
    uint32_t checksum;                          /* CRC32 của mọi byte phía trước (phát hiện ghi dở / hỏng) */
} FlashMapData;

typedef enum
{
    FLASH_OK = 0,
    FLASH_EMPTY,                                /* Không có bản ghi hợp lệ (Flash trống / sai magic / sai checksum) */
    FLASH_ERR_ERASE,
    FLASH_ERR_PROGRAM,
    FLASH_ERR_VERIFY                            /* Đọc lại khác dữ liệu vừa ghi */
} FlashResult;

/* ===== API (luồng chính, xe đứng yên + đã phanh) ===== */
FlashResult Flash_LoadMap(FlashMapData *out);   /* Chép bản hợp lệ mới nhất ra out. FLASH_EMPTY nếu không có */
FlashResult Flash_SaveMap(FlashMapData *rec);   /* Điền magic / version / sequence / checksum vào rec rồi ghi */
FlashResult Flash_EraseMap(void);               /* Xóa trắng sector 7 (~1-2 s) */
const char *Flash_ResultName(FlashResult r);    /* Tên kết quả để in */

#endif /* FLASH_STORAGE_H */
