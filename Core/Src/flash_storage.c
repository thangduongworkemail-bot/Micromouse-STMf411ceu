/**
  ******************************************************************************
  * @file    flash_storage.c
  * @brief   Lưu / đọc bản đồ mê cung ở Flash sector 7 (xem flash_storage.h)
  *
  * Sector 7 = dãy ô (slot) liên tiếp, mỗi ô 1 FlashMapData. Ô trống = toàn 0xFF. Bản ghi được ghi tuần tự từ word đầu
  * nên "word đầu = 0xFFFFFFFF" nghĩa là ô đó và mọi ô sau nó đều trống.
  ******************************************************************************
  */
#include "flash_storage.h"
#include "main.h"           /* HAL Flash */
#include <stddef.h>         /* offsetof */
#include <string.h>         /* memcmp, memcpy */

#define REC_BYTES       ((uint32_t)sizeof(FlashMapData))
#define REC_WORDS       (REC_BYTES / 4U)
#define SLOT_COUNT      (FLASH_MAP_BYTES / REC_BYTES)
#define BLANK_WORD      0xFFFFFFFFUL
#define NO_SLOT         0xFFFFFFFFUL

_Static_assert((sizeof(FlashMapData) % 4U) == 0U, "FlashMapData phai la boi so 4 byte (ghi theo word)");

/* Đọc Flash như bộ nhớ thường (mô phỏng trên máy tính thay macro này) */
#ifndef FLASH_MAP_PTR
#define FLASH_MAP_PTR(addr)   ((const uint8_t *)(uintptr_t)(addr))
#endif

static uint32_t Slot_Addr(uint32_t i)
{
    return (uint32_t)FLASH_MAP_ADDR + (i * REC_BYTES);
}
static uint32_t Word_At(uint32_t addr)
{
    uint32_t w;
    memcpy(&w, FLASH_MAP_PTR(addr), 4U);
    return w;
}
static bool Slot_Blank(uint32_t i)
{
    bool blank = true;
    for (uint32_t k = 0U; (k < REC_WORDS) && blank; k++) {
        blank = (Word_At(Slot_Addr(i) + (k * 4U)) == BLANK_WORD);
    }
    return blank;
}
/* CRC-32 (đa thức 0xEDB88320), không cần bảng */
static uint32_t Crc32(const uint8_t *p, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0U; i < n; i++) {
        crc ^= p[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = ((crc & 1UL) != 0UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return ~crc;
}
static bool Rec_Valid(const FlashMapData *r)
{
    return (r->magic_word == FLASH_MAP_MAGIC) && (r->version == FLASH_MAP_VERSION) && (r->record_size == REC_BYTES) &&
           (r->checksum == Crc32((const uint8_t *)r, (uint32_t)offsetof(FlashMapData, checksum)));
}
/* Quét sector: ô trống đầu tiên (*next_free, SLOT_COUNT nếu đầy) + ô chứa bản hợp lệ mới nhất (*latest, NO_SLOT nếu
 * không có). Bản ghi nối tiếp với sequence tăng dần -> bản mới nhất là ô hợp lệ cuối cùng trước ô trống: dò ngược từ
 * đó (thường chỉ kiểm tra checksum 1 ô). Ô ghi dở (không trống, không hợp lệ) thì bỏ qua */
static void Scan(uint32_t *latest, uint32_t *latest_seq, uint32_t *next_free)
{
    static FlashMapData r;
    uint32_t i = 0U;
    while ((i < SLOT_COUNT) && (Word_At(Slot_Addr(i)) != BLANK_WORD)) {
        i++;                                   /* Ghi tuần tự: từ ô trống đầu tiên trở đi đều trống */
    }
    *next_free = i;
    *latest = NO_SLOT;
    *latest_seq = 0U;
    while (i > 0U) {
        i--;
        memcpy(&r, FLASH_MAP_PTR(Slot_Addr(i)), REC_BYTES);
        if (Rec_Valid(&r)) {
            *latest = i;
            *latest_seq = r.sequence;
            break;
        }
    }
}
/* Sau khi ghi / xóa: bỏ dữ liệu cũ trong bộ đệm đọc Flash (ART data cache) để đọc lại thấy dữ liệu mới */
static void Flash_FlushDataCache(void)
{
    if ((FLASH->ACR & FLASH_ACR_DCEN) != 0U) {
        __HAL_FLASH_DATA_CACHE_DISABLE();
        __HAL_FLASH_DATA_CACHE_RESET();
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }
}
static void Flash_ClearErrors(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}
/* Xóa sector 7 (Flash đã Unlock) */
static FlashResult Erase_Sector(void)
{
    FLASH_EraseInitTypeDef e;
    uint32_t sector_error = 0U;
    e.TypeErase    = FLASH_TYPEERASE_SECTORS;
    e.Banks        = FLASH_BANK_1;
    e.Sector       = FLASH_MAP_SECTOR;
    e.NbSectors    = 1U;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;     /* VDD 2.7-3.6V: xóa / ghi song song 32 bit */
    if ((HAL_FLASHEx_Erase(&e, &sector_error) != HAL_OK) || (sector_error != 0xFFFFFFFFUL)) {
        return FLASH_ERR_ERASE;
    }
    return FLASH_OK;
}

FlashResult Flash_LoadMap(FlashMapData *out)
{
    uint32_t latest;
    uint32_t seq;
    uint32_t next_free;
    Scan(&latest, &seq, &next_free);
    if (latest == NO_SLOT) {
        return FLASH_EMPTY;
    }
    memcpy(out, FLASH_MAP_PTR(Slot_Addr(latest)), REC_BYTES);
    return FLASH_OK;
}

FlashResult Flash_SaveMap(FlashMapData *rec)
{
    uint32_t latest;
    uint32_t seq;
    uint32_t slot;
    FlashResult res = FLASH_OK;
    const uint32_t *src = (const uint32_t *)(const void *)rec;

    Scan(&latest, &seq, &slot);
    rec->magic_word  = FLASH_MAP_MAGIC;
    rec->version     = FLASH_MAP_VERSION;
    rec->record_size = (uint16_t)REC_BYTES;
    rec->sequence    = seq + 1U;
    rec->checksum    = Crc32((const uint8_t *)rec, (uint32_t)offsetof(FlashMapData, checksum));

    /* Ô "trống" theo word đầu nhưng còn rác phía sau (mất điện lúc ghi dở bản trước): sang ô kế */
    while ((slot < SLOT_COUNT) && !Slot_Blank(slot)) {
        slot++;
    }

    HAL_FLASH_Unlock();                         /* 1. Mở khóa */
    Flash_ClearErrors();
    if ((FLASH_ERASE_EVERY_SAVE != 0) || (slot >= SLOT_COUNT)) {
        res = Erase_Sector();                   /* 2. Xóa sector 7 (đầy, hoặc chọn xóa mỗi lần) */
        slot = 0U;
    }
    for (uint32_t k = 0U; (k < REC_WORDS) && (res == FLASH_OK); k++) {   /* 3. Ghi từng word */
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Slot_Addr(slot) + (k * 4U), src[k]) != HAL_OK) {
            res = FLASH_ERR_PROGRAM;
        }
    }
    HAL_FLASH_Lock();                           /* 4. Khóa lại */
    Flash_FlushDataCache();

    if ((res == FLASH_OK) && (memcmp(FLASH_MAP_PTR(Slot_Addr(slot)), rec, REC_BYTES) != 0)) {
        res = FLASH_ERR_VERIFY;                 /* 5. Đọc lại phải đúng từng byte */
    }
    return res;
}

FlashResult Flash_EraseMap(void)
{
    FlashResult res;
    HAL_FLASH_Unlock();
    Flash_ClearErrors();
    res = Erase_Sector();
    HAL_FLASH_Lock();
    Flash_FlushDataCache();
    return res;
}

const char *Flash_ResultName(FlashResult r)
{
    static const char *const names[] = { "OK", "TRONG", "LOI XOA", "LOI GHI", "LOI DOC LAI" };
    return ((uint32_t)r < (sizeof(names) / sizeof(names[0]))) ? names[r] : "?";
}
