/* SPDX-License-Identifier: BSD-3-Clause */
/**
 * @file bno085.c
 * @brief Portable BNO085 SPI transport, SHTP framing and SH-2 report parser.
 *        可移植的 BNO085 SPI 传输、SHTP 组帧和 SH-2 报告解析核心。
 *
 * Receive path / 接收路径:
 * H_INTN -> SHTP header -> complete cargo -> SH-2 reports -> typed cache.
 * H_INTN -> SHTP 包头 -> 完整载荷 -> SH-2 子报告 -> 类型化缓存。
 *
 * This file contains no STM32 symbols.  All private helpers and protocol-only
 * constants remain local to this translation unit.
 * 本文件不含 STM32 符号；内部工具函数和协议私有宏只在本编译单元可见。
 */
#include "bno085.h"
#include "bno085_port.h"

#include <math.h>
#include <string.h>

/* SHTP transport framing and local buffer limits. / SHTP 帧与本地缓冲配置。 */
#define SHTP_HEADER_SIZE                 4U
#define SHTP_CHANNEL_COUNT               6U
#define IO_BUFFER_SIZE                 260U
#define CARGO_BUFFER_SIZE              256U
#define SPI_TIMEOUT_MS                 100U
#define COMMAND_TIMEOUT_MS             200U
#define STARTUP_TIMEOUT_MS            2000U
#define DRAIN_TIMEOUT_MS               100U
#define DRAIN_PACKET_LIMIT              32U
#define PRODUCT_ID_RESPONSE_COUNT        4U
#define WAIT_FOREVER            0xFFFFFFFFUL

/* SHTP channel numbers assigned by the BNO08X firmware. / 固件规定的通道号。 */
#define CHANNEL_COMMAND                  0U
#define CHANNEL_EXECUTABLE               1U
#define CHANNEL_CONTROL                  2U
#define CHANNEL_NON_WAKE                 3U

/* SH-2 report identifiers used or safely skipped by this driver.
 * 本驱动需要解析或安全跳过的 SH-2 报告 ID。 */
#define REPORT_ACCELEROMETER           0x01U
#define REPORT_ROTATION_VECTOR         0x05U
#define REPORT_PRODUCT_ID_RESPONSE     0xF8U
#define REPORT_PRODUCT_ID_REQUEST      0xF9U
#define REPORT_SET_FEATURE             0xFDU
#define REPORT_GET_FEATURE_RESPONSE    0xFCU
#define REPORT_COMMAND_RESPONSE        0xF1U
#define REPORT_FRS_READ_RESPONSE       0xF3U
#define REPORT_FRS_WRITE_RESPONSE      0xF5U
#define REPORT_BASE_TIMESTAMP          0xFBU
#define REPORT_TIMESTAMP_REBASE        0xFAU

/* Fixed-point conversion factors from the SH-2 report definitions.
 * SH-2 报告规定的定点数缩放系数。 */
#define Q14_SCALE        (1.0f / 16384.0f)
#define Q12_SCALE         (1.0f / 4096.0f)
#define Q8_SCALE           (1.0f / 256.0f)
#define RAD_TO_DEG        57.2957795131f

/*
 * Allocation-free, compact single-instance driver state.
 * 无动态内存的单实例状态：静态缓冲避免大数组反复压入任务栈。
 *
 * Getters copy these snapshots only after the parser finishes a complete
 * report, so yaw/roll/pitch cannot be mixed across two sensor frames.
 * 解析完整报告后才更新快照，因此三个角度不会来自不同传感器帧。
 */
static uint8_t tx_sequence[SHTP_CHANNEL_COUNT];
static uint8_t io_tx[IO_BUFFER_SIZE];
static uint8_t io_rx[IO_BUFFER_SIZE];
static uint8_t cargo_buffer[CARGO_BUFFER_SIZE];
static bool initialized;
static bool have_rotation_vector;
static bool have_euler;
static bool have_acceleration;
static BNO085_RotationVector_t latest_rotation_vector;
static BNO085_Euler_t latest_euler;
static BNO085_Acceleration_t latest_acceleration;

/** Decode little-endian unsigned 16-bit data. / 读取小端无符号 16 位数。 */
static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/** Decode little-endian signed 16-bit data. / 读取小端有符号 16 位数。 */
static int16_t read_s16_le(const uint8_t *p)
{
    return (int16_t)read_u16_le(p);
}

/** Decode little-endian unsigned 32-bit data. / 读取小端无符号 32 位数。 */
static uint32_t read_u32_le(const uint8_t *p)
{
    return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/** Overflow-safe timeout test based on unsigned subtraction. / 可处理时基回卷的超时判断。 */
static bool timed_out(uint32_t start_ms, uint32_t timeout_ms)
{
    if (timeout_ms == WAIT_FOREVER) {
        return false;
    }
    return ((uint32_t)(BNO085_Port_GetTimeMs() - start_ms) >= timeout_ms);
}

/** Return the remaining budget for nested waits. / 计算嵌套等待可使用的剩余时间。 */
static uint32_t remaining_time(uint32_t start_ms, uint32_t timeout_ms)
{
    uint32_t elapsed;

    if (timeout_ms == WAIT_FOREVER) {
        return WAIT_FOREVER;
    }
    elapsed = (uint32_t)(BNO085_Port_GetTimeMs() - start_ms);
    return (elapsed >= timeout_ms) ? 0U : (timeout_ms - elapsed);
}

/**
 * Wait until active-low H_INTN says the device accepts/has a transaction.
 * 等待低有效 H_INTN，表示设备已有数据或已响应主机 WAKE。
 * For writes, WAKE is asserted first; reads never need to wake an already
 * interrupting device. / 写事务先拉低 WAKE，读事务无需重复唤醒。
 */
static BNO085_Status_t wait_for_interrupt(uint32_t timeout_ms,
                                          bool assert_wake)
{
    uint32_t start_ms = BNO085_Port_GetTimeMs();

    if (assert_wake) {
        BNO085_Port_SetWake(true);
    }
    while (!BNO085_Port_IsInterruptAsserted()) {
        if (timed_out(start_ms, timeout_ms)) {
            if (assert_wake) {
                BNO085_Port_SetWake(false);
            }
            return BNO085_ERR_TIMEOUT;
        }
    }
    return BNO085_OK;
}

/**
 * Clock bytes out of BNO085 using a zero-filled host frame.
 * 主机发送全零帧，为 BNO085 提供时钟并接收指定字节数。
 *
 * Sending 0x00 is essential: the first two MOSI bytes represent an empty
 * host SHTP packet.  0xFF would look like an invalid 0xFFFF length header.
 * 必须发送 0x00；0xFF 会被设备误判为长度 0xFFFF 的非法主机包。
 */
static BNO085_Status_t spi_read_bytes(uint8_t *data, uint16_t length)
{
    uint16_t offset = 0U;

    while (offset < length) {
        uint16_t count = (uint16_t)(length - offset);
        uint8_t *destination;

        if (count > IO_BUFFER_SIZE) {
            count = IO_BUFFER_SIZE;
        }
        destination = (data != NULL) ? (data + offset) : io_rx;
        /* Zero SHTP length: host has no cargo while clocking device data. */
        memset(io_tx, 0, count);
        if (!BNO085_Port_SPITransfer(io_tx, destination, count,
                                     SPI_TIMEOUT_MS)) {
            return BNO085_ERR_PORT;
        }
        offset = (uint16_t)(offset + count);
    }
    return BNO085_OK;
}

/**
 * Send header and cargo in one continuous SPI transfer.
 * 将 SHTP 包头和载荷拼接成一次连续 SPI 传输。
 * Splitting them into separate CS transactions would create two invalid
 * packets. / 若中途释放 CS，设备会看到两个不完整包。
 */
static BNO085_Status_t spi_write_packet(const uint8_t *header,
                                         const uint8_t *data,
                                         uint16_t length)
{
    uint16_t packet_length = (uint16_t)(SHTP_HEADER_SIZE + length);

    if (packet_length > IO_BUFFER_SIZE) {
        return BNO085_ERR_BAD_PARAM;
    }
    memcpy(io_tx, header, SHTP_HEADER_SIZE);
    if (length > 0U) {
        memcpy(io_tx + SHTP_HEADER_SIZE, data, length);
    }
    if (!BNO085_Port_SPITransfer(io_tx, io_rx, packet_length,
                                 SPI_TIMEOUT_MS)) {
        return BNO085_ERR_PORT;
    }
    return BNO085_OK;
}

/**
 * Read one complete physical SHTP packet while CS stays low.
 * 在 CS 持续为低期间读取一个完整 SHTP 物理包。
 *
 * The four-byte header reveals cargo length and channel.  If the caller's
 * buffer is short, excess bytes are still clocked out to preserve framing.
 * 先读四字节包头获得长度和通道；即使缓冲不足也必须排空剩余字节，防止下包错位。
 */
static BNO085_Status_t read_packet(uint8_t *buffer, uint16_t capacity,
                                   uint16_t *cargo_length, uint8_t *channel,
                                   bool *continuation, uint32_t timeout_ms)
{
    uint8_t header[SHTP_HEADER_SIZE];
    uint16_t raw_length;
    uint16_t packet_length;
    uint16_t copy_length;
    BNO085_Status_t status;

    status = wait_for_interrupt(timeout_ms, false);
    if (status != BNO085_OK) {
        return status;
    }

    BNO085_Port_SetChipSelect(true);
    status = spi_read_bytes(header, sizeof(header));
    if (status != BNO085_OK) {
        BNO085_Port_SetChipSelect(false);
        return status;
    }

    /* Header bytes / 包头字节:
     * [0..1] total length including header; bit15 = continuation
     * [2]    channel
     * [3]    sequence number (BNO085 -> host direction)
     * [0..1] 为含包头总长度，bit15 表示分片；[2] 通道；[3] 序号。 */
    raw_length = read_u16_le(header);
    packet_length = (uint16_t)(raw_length & 0x7FFFU);
    if ((packet_length < SHTP_HEADER_SIZE) || (raw_length == 0xFFFFU) ||
        (header[2] >= SHTP_CHANNEL_COUNT)) {
        BNO085_Port_SetChipSelect(false);
        return BNO085_ERR_INVALID_REPORT;
    }

    *cargo_length = (uint16_t)(packet_length - SHTP_HEADER_SIZE);
    *channel = header[2];
    *continuation = ((header[1] & 0x80U) != 0U);
    copy_length = (*cargo_length > capacity) ? capacity : *cargo_length;

    if (copy_length > 0U) {
        status = spi_read_bytes(buffer, copy_length);
    }
    if ((status == BNO085_OK) && (*cargo_length > copy_length)) {
        status = spi_read_bytes(NULL,
                                (uint16_t)(*cargo_length - copy_length));
    }
    BNO085_Port_SetChipSelect(false);

    if (status != BNO085_OK) {
        return status;
    }
    return (*cargo_length > capacity) ?
           BNO085_ERR_BUFFER_TOO_SMALL : BNO085_OK;
}

/**
 * Drain unsolicited input until H_INTN releases before a host write.
 * 主机写入前排空未读输入，直到 H_INTN 释放为高。
 *
 * BNO085 starts a host-write handshake from an idle/high interrupt state;
 * stale sensor packets would otherwise be mistaken for write readiness.
 * 写握手应从 H_INTN 高电平开始，否则旧传感器包会被误认为写请求响应。
 */
static BNO085_Status_t drain_pending_packets(void)
{
    uint32_t start_ms = BNO085_Port_GetTimeMs();
    uint32_t packet_count = 0U;

    BNO085_Port_SetWake(false);
    while (BNO085_Port_IsInterruptAsserted()) {
        uint16_t cargo_length = 0U;
        uint8_t channel = 0U;
        bool continuation = false;
        BNO085_Status_t status;

        status = read_packet(cargo_buffer, sizeof(cargo_buffer),
                             &cargo_length, &channel, &continuation, 10U);
        if ((status != BNO085_OK) &&
            (status != BNO085_ERR_BUFFER_TOO_SMALL)) {
            return status;
        }
        packet_count++;
        BNO085_Port_DelayMs(1U);
        if ((packet_count >= DRAIN_PACKET_LIMIT) ||
            timed_out(start_ms, DRAIN_TIMEOUT_MS)) {
            return BNO085_ERR_TIMEOUT;
        }
    }
    return BNO085_OK;
}

/**
 * Complete the two-stage boot handshake.
 * 完成两阶段启动握手：先收 channel 0 advertisement，再等待 channel 1 的
 * executable reset-complete(0x01)。只有第二阶段完成后 SH-2 才接受配置。
 */
static BNO085_Status_t wait_for_advertisement(void)
{
    uint16_t cargo_length = 0U;
    uint8_t channel = 0U;
    bool continuation = false;
    uint32_t start_ms;
    BNO085_Status_t status;

    status = read_packet(cargo_buffer, sizeof(cargo_buffer),
                         &cargo_length, &channel, &continuation,
                         STARTUP_TIMEOUT_MS);
    if ((status != BNO085_OK) &&
        (status != BNO085_ERR_BUFFER_TOO_SMALL)) {
        return status;
    }
    if ((channel != CHANNEL_COMMAND) || (cargo_length == 0U) ||
        continuation) {
        return BNO085_ERR_NO_RESPONSE;
    }

    start_ms = BNO085_Port_GetTimeMs();
    while (!timed_out(start_ms, STARTUP_TIMEOUT_MS)) {
        if (!BNO085_DataReady()) {
            BNO085_Port_DelayMs(1U);
            continue;
        }
        status = read_packet(cargo_buffer, sizeof(cargo_buffer),
                             &cargo_length, &channel, &continuation, 10U);
        if ((status != BNO085_OK) &&
            (status != BNO085_ERR_BUFFER_TOO_SMALL)) {
            return status;
        }
        if ((channel == CHANNEL_EXECUTABLE) &&
            (cargo_length >= 1U) && (cargo_buffer[0] == 1U) &&
            !continuation) {
            return BNO085_OK;
        }
    }
    return BNO085_ERR_TIMEOUT;
}

/**
 * Return the fixed length of one SH-2 sub-report.
 * 返回一个 SH-2 子报告的固定长度，用于遍历同一 cargo 中的多个报告。
 */
static uint8_t report_length(uint8_t report_id)
{
    switch (report_id) {
        case 0x01U: return 10U;
        case 0x02U: return 10U;
        case 0x03U: return 10U;
        case 0x04U: return 10U;
        case 0x05U: return 14U;
        case 0x06U: return 10U;
        case 0x07U: return 16U;
        case 0x08U: return 12U;
        case 0x09U: return 14U;
        case 0x0FU: return 16U;
        case 0x14U: return 16U;
        case 0x15U: return 16U;
        case 0x16U: return 16U;
        case REPORT_COMMAND_RESPONSE: return 16U;
        case REPORT_FRS_READ_RESPONSE: return 16U;
        case REPORT_FRS_WRITE_RESPONSE: return 4U;
        case REPORT_PRODUCT_ID_RESPONSE: return 16U;
        case REPORT_GET_FEATURE_RESPONSE: return 17U;
        case REPORT_TIMESTAMP_REBASE: return 5U;
        case REPORT_BASE_TIMESTAMP: return 5U;
        default: return 0U;
    }
}

/**
 * Build and send one unfragmented host SHTP cargo.
 * 构造并发送一个不分片的主机 SHTP cargo。
 * Sequence numbers are independent per channel as required by SHTP.
 * SHTP 要求每个通道分别维护自己的 sequence number。
 */
static BNO085_Status_t send_packet(const uint8_t *data, uint16_t length,
                                    uint8_t channel)
{
    uint8_t header[SHTP_HEADER_SIZE];
    uint16_t packet_length;
    BNO085_Status_t status;

    if (!initialized) {
        return BNO085_ERR_NO_RESPONSE;
    }
    if (((data == NULL) && (length != 0U)) ||
        (channel >= SHTP_CHANNEL_COUNT) ||
        ((uint16_t)(length + SHTP_HEADER_SIZE) > IO_BUFFER_SIZE)) {
        return BNO085_ERR_BAD_PARAM;
    }

    packet_length = (uint16_t)(length + SHTP_HEADER_SIZE);
    /* Host SHTP header / 主机发送包头。Length includes these four bytes.
     * 长度字段必须包含包头本身的四个字节。 */
    header[0] = (uint8_t)(packet_length & 0xFFU);
    header[1] = (uint8_t)((packet_length >> 8) & 0x7FU);
    header[2] = channel;
    header[3] = tx_sequence[channel];

    /* A write uses WAKE/H_INTN handshake.  Clear old device-to-host traffic
     * first, then assert WAKE and wait for H_INTN low.
     * 写入前先清旧包，再拉低 WAKE 并等待 H_INTN 拉低。 */
    status = drain_pending_packets();
    if (status != BNO085_OK) {
        return status;
    }
    status = wait_for_interrupt(COMMAND_TIMEOUT_MS, true);
    if (status != BNO085_OK) {
        return status;
    }

    /* Keep CS active for header+cargo.  WAKE may be released once transfer
     * starts because H_INTN has already acknowledged it.
     * 包头和载荷期间 CS 始终为低；H_INTN 已响应后即可释放 WAKE。 */
    BNO085_Port_SetChipSelect(true);
    BNO085_Port_SetWake(false);
    tx_sequence[channel]++;
    status = spi_write_packet(header, data, length);
    BNO085_Port_SetChipSelect(false);
    BNO085_Port_DelayMs(1U);
    return status;
}

/**
 * Receive the next complete cargo on the requested channel.
 * 接收指定通道的下一完整 cargo；等待期间会消费其他通道的异步数据。
 */
static BNO085_Status_t receive_channel(uint8_t *buffer, uint16_t *length,
                                        uint8_t expected_channel,
                                        uint32_t timeout_ms)
{
    uint16_t capacity;
    uint32_t start_ms;

    if ((buffer == NULL) || (length == NULL) ||
        (expected_channel >= SHTP_CHANNEL_COUNT)) {
        return BNO085_ERR_BAD_PARAM;
    }

    capacity = *length;
    start_ms = BNO085_Port_GetTimeMs();
    for (;;) {
        uint16_t cargo_length = 0U;
        uint8_t channel = 0U;
        bool continuation = false;
        uint32_t remaining = remaining_time(start_ms, timeout_ms);
        BNO085_Status_t status;

        if ((remaining == 0U) && !BNO085_DataReady()) {
            return BNO085_ERR_TIMEOUT;
        }
        status = read_packet(buffer, capacity, &cargo_length, &channel,
                             &continuation, remaining);
        if ((status != BNO085_OK) &&
            (status != BNO085_ERR_BUFFER_TOO_SMALL)) {
            return status;
        }
        if (channel == expected_channel) {
            *length = cargo_length;
            if (continuation) {
                return BNO085_ERR_INVALID_REPORT;
            }
            return status;
        }
        if (timed_out(start_ms, timeout_ms)) {
            return BNO085_ERR_TIMEOUT;
        }
    }
}

/**
 * Emit the 17-byte SH-2 Set Feature command.
 * 发送 17 字节 SH-2 Set Feature 命令；周期字段为小端微秒数。
 */
static BNO085_Status_t set_report_interval(uint8_t report_id,
                                            uint32_t interval_us)
{
    uint8_t command[17];

    if (interval_us == 0U) {
        return BNO085_ERR_BAD_PARAM;
    }
    /* Set Feature layout used here / 本驱动使用的 Set Feature 字段:
     * byte 0: report ID 0xFD
     * byte 1: sensor feature ID
     * byte 5..8: report interval in microseconds, little endian
     * other fields: zero = no batching, default sensitivity. */
    memset(command, 0, sizeof(command));
    command[0] = REPORT_SET_FEATURE;
    command[1] = report_id;
    command[5] = (uint8_t)(interval_us & 0xFFU);
    command[6] = (uint8_t)((interval_us >> 8) & 0xFFU);
    command[7] = (uint8_t)((interval_us >> 16) & 0xFFU);
    command[8] = (uint8_t)((interval_us >> 24) & 0xFFU);
    return send_packet(command, sizeof(command), CHANNEL_CONTROL);
}

/**
 * Convert quaternion to intrinsic Z-Y-X Euler angles in degrees.
 * 将四元数转换为 Z-Y-X 欧拉角（度）：yaw(Z)、pitch(Y)、roll(X)。
 * Clamp asin input to absorb fixed-point rounding beyond [-1, 1].
 * 对 asin 输入限幅，避免定点量化使其轻微越过 [-1, 1]。
 */
static void quaternion_to_euler(const BNO085_RotationVector_t *q,
                                BNO085_Euler_t *euler)
{
    float sin_roll = 2.0f * (q->w * q->x + q->y * q->z);
    float cos_roll = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    float sin_pitch = 2.0f * (q->w * q->y - q->z * q->x);
    float sin_yaw = 2.0f * (q->w * q->z + q->x * q->y);
    float cos_yaw = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);

    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    euler->yaw_deg = atan2f(sin_yaw, cos_yaw) * RAD_TO_DEG;
    euler->roll_deg = atan2f(sin_roll, cos_roll) * RAD_TO_DEG;
    euler->pitch_deg = asinf(sin_pitch) * RAD_TO_DEG;
    euler->accuracy = q->accuracy;
    euler->timestamp_us = q->timestamp_us;
}

/**
 * Walk all SH-2 reports carried by one non-wake sensor cargo.
 * 遍历一次 non-wake cargo 中的全部 SH-2 报告。
 *
 * Acceleration uses Q8 m/s^2; quaternion uses Q14; angular accuracy uses
 * Q12 radians.  Cache publication happens only after every field is decoded.
 * 加速度为 Q8 m/s^2，四元数为 Q14，角精度为 Q12 弧度；字段完整后才发布缓存。
 */
static BNO085_Status_t parse_sensor_payload(const uint8_t *payload,
                                             uint16_t length,
                                             uint32_t *events)
{
    uint16_t cursor = 0U;
    uint32_t timestamp_us = BNO085_Port_GetTimeMs() * 1000U;

    while (cursor < length) {
        uint8_t item_length = report_length(payload[cursor]);
        const uint8_t *p = payload + cursor;

        if ((item_length == 0U) ||
            ((uint16_t)(cursor + item_length) > length)) {
            return BNO085_ERR_INVALID_REPORT;
        }
        if (p[0] == REPORT_BASE_TIMESTAMP) {
            /* Subsequent reports in this cargo share this sensor timebase.
             * 后续子报告共享该传感器时间基准。 */
            timestamp_us = read_u32_le(p + 1U);
        } else if (p[0] == REPORT_ACCELEROMETER) {
            /* Common sensor header: [0]ID [1]seq [2]status [3]delay;
             * vector data begins at byte 4. / 前四字节为通用传感器头。 */
            latest_acceleration.x_mps2 =
                (float)read_s16_le(p + 4U) * Q8_SCALE;
            latest_acceleration.y_mps2 =
                (float)read_s16_le(p + 6U) * Q8_SCALE;
            latest_acceleration.z_mps2 =
                (float)read_s16_le(p + 8U) * Q8_SCALE;
            latest_acceleration.sequence = p[1];
            latest_acceleration.accuracy = (uint8_t)(p[2] & 0x03U);
            latest_acceleration.timestamp_us = timestamp_us;
            have_acceleration = true;
            *events |= BNO085_EVENT_ACCELEROMETER;
        } else if (p[0] == REPORT_ROTATION_VECTOR) {
            /* SH-2 order is i, j, k, real = x, y, z, w.
             * SH-2 顺序为 i、j、k、real，即 x、y、z、w。 */
            latest_rotation_vector.x =
                (float)read_s16_le(p + 4U) * Q14_SCALE;
            latest_rotation_vector.y =
                (float)read_s16_le(p + 6U) * Q14_SCALE;
            latest_rotation_vector.z =
                (float)read_s16_le(p + 8U) * Q14_SCALE;
            latest_rotation_vector.w =
                (float)read_s16_le(p + 10U) * Q14_SCALE;
            latest_rotation_vector.accuracy_rad =
                (float)read_u16_le(p + 12U) * Q12_SCALE;
            latest_rotation_vector.sequence = p[1];
            latest_rotation_vector.accuracy = (uint8_t)(p[2] & 0x03U);
            latest_rotation_vector.timestamp_us = timestamp_us;
            have_rotation_vector = true;
            quaternion_to_euler(&latest_rotation_vector, &latest_euler);
            have_euler = true;
            *events |= BNO085_EVENT_ROTATION_VECTOR;
        }
        cursor = (uint16_t)(cursor + item_length);
    }
    return BNO085_OK;
}

/* Public API implementation / 公共 API 实现。 */
BNO085_Status_t BNO085_Init(void)
{
    if (!BNO085_Port_IsReady()) {
        return BNO085_ERR_PORT;
    }
    return BNO085_Reset();
}

BNO085_Status_t BNO085_Reset(void)
{
    BNO085_Status_t status;

    if (!BNO085_Port_IsReady()) {
        return BNO085_ERR_PORT;
    }
    /* Invalidate old samples first so getters never expose pre-reset data.
     * 先使旧缓存失效，避免复位后读到上一次运行的数据。 */
    initialized = false;
    have_rotation_vector = false;
    have_euler = false;
    have_acceleration = false;
    memset(tx_sequence, 0, sizeof(tx_sequence));
    memset(&latest_rotation_vector, 0, sizeof(latest_rotation_vector));
    memset(&latest_euler, 0, sizeof(latest_euler));
    memset(&latest_acceleration, 0, sizeof(latest_acceleration));

    /* PS0 doubles as WAKE after boot.  It must be high while reset is sampled
     * to select SPI together with PS1=high. / 复位采样期间 PS0 必须为高。 */
    BNO085_Port_SetChipSelect(false);
    BNO085_Port_SetWake(false);
    BNO085_Port_SetReset(true);
    BNO085_Port_DelayMs(10U);
    BNO085_Port_SetReset(false);

    status = wait_for_advertisement();
    if (status == BNO085_OK) {
        initialized = true;
    }
    return status;
}

bool BNO085_DataReady(void)
{
    return BNO085_Port_IsReady() && BNO085_Port_IsInterruptAsserted();
}

/** Request and consume all four product-ID responses. / 请求并消费四条产品信息响应。 */
BNO085_Status_t BNO085_GetProductInfo(BNO085_ProductInfo_t *info)
{
    uint8_t request[2] = { REPORT_PRODUCT_ID_REQUEST, 0U };
    uint8_t response_count = 0U;
    bool found = false;
    uint32_t start_ms;
    BNO085_Status_t status;

    if (info == NULL) {
        return BNO085_ERR_BAD_PARAM;
    }
    memset(info, 0, sizeof(*info));
    status = send_packet(request, sizeof(request), CHANNEL_CONTROL);
    if (status != BNO085_OK) {
        return status;
    }

    start_ms = BNO085_Port_GetTimeMs();
    for (;;) {
        uint16_t length = sizeof(cargo_buffer);
        uint16_t cursor = 0U;
        uint32_t remaining = remaining_time(start_ms, 500U);

        if (remaining == 0U) {
            return BNO085_ERR_TIMEOUT;
        }
        status = receive_channel(cargo_buffer, &length, CHANNEL_CONTROL,
                                 remaining);
        if (status != BNO085_OK) {
            return status;
        }
        while (cursor < length) {
            uint8_t item_length = report_length(cargo_buffer[cursor]);
            const uint8_t *p = cargo_buffer + cursor;

            if ((item_length == 0U) ||
                ((uint16_t)(cursor + item_length) > length)) {
                return BNO085_ERR_INVALID_REPORT;
            }
            if (p[0] == REPORT_PRODUCT_ID_RESPONSE) {
                /* The compact API exposes the first entry, but all four must
                 * be drained before the next host write. / 返回第一条，但必须排空四条。 */
                if (!found) {
                    info->reset_cause = p[1];
                    info->sw_major = p[2];
                    info->sw_minor = p[3];
                    info->sw_part_number = read_u32_le(p + 4U);
                    info->build_number = read_u32_le(p + 8U);
                    info->sw_patch = read_u16_le(p + 12U);
                    found = true;
                }
                response_count++;
                if (response_count >= PRODUCT_ID_RESPONSE_COUNT) {
                    return BNO085_OK;
                }
            }
            cursor = (uint16_t)(cursor + item_length);
        }
    }
}

BNO085_Status_t BNO085_EnableRotationVector(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_ROTATION_VECTOR, report_interval_us);
}

BNO085_Status_t BNO085_EnableAccelerometer(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_ACCELEROMETER, report_interval_us);
}

/** Streaming fast path: one I/O/parser entry for all enabled reports.
 *  流式快速路径：所有已启用报告共用一个收包与解析入口。 */
BNO085_Status_t BNO085_Poll(uint32_t timeout_ms, uint32_t *events)
{
    uint32_t start_ms;

    if (events == NULL) {
        return BNO085_ERR_BAD_PARAM;
    }
    if (!initialized) {
        return BNO085_ERR_NO_RESPONSE;
    }
    *events = 0U;
    start_ms = BNO085_Port_GetTimeMs();

    for (;;) {
        uint16_t length = sizeof(cargo_buffer);
        uint32_t remaining = remaining_time(start_ms, timeout_ms);
        BNO085_Status_t status;

        if ((remaining == 0U) && !BNO085_DataReady()) {
            return BNO085_ERR_TIMEOUT;
        }
        status = receive_channel(cargo_buffer, &length, CHANNEL_NON_WAKE,
                                 remaining);
        if (status != BNO085_OK) {
            return status;
        }
        status = parse_sensor_payload(cargo_buffer, length, events);
        if (status != BNO085_OK) {
            return status;
        }
        if (*events != 0U) {
            return BNO085_OK;
        }
        if (timed_out(start_ms, timeout_ms)) {
            return BNO085_ERR_TIMEOUT;
        }
    }
}

/* Cached snapshot getters: deliberately no SPI/HAL calls.
 * 缓存快照 Getter：有意不执行任何 SPI/HAL 调用。 */
BNO085_Status_t BNO085_GetRotationVector(BNO085_RotationVector_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_rotation_vector) return BNO085_ERR_NO_DATA;
    *value = latest_rotation_vector;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetEuler(BNO085_Euler_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_euler) return BNO085_ERR_NO_DATA;
    *value = latest_euler;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetYaw(float *yaw_deg)
{
    if (yaw_deg == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_euler) return BNO085_ERR_NO_DATA;
    *yaw_deg = latest_euler.yaw_deg;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetRoll(float *roll_deg)
{
    if (roll_deg == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_euler) return BNO085_ERR_NO_DATA;
    *roll_deg = latest_euler.roll_deg;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetPitch(float *pitch_deg)
{
    if (pitch_deg == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_euler) return BNO085_ERR_NO_DATA;
    *pitch_deg = latest_euler.pitch_deg;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetAcceleration(BNO085_Acceleration_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_acceleration) return BNO085_ERR_NO_DATA;
    *value = latest_acceleration;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetAccelerationX(float *x_mps2)
{
    if (x_mps2 == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_acceleration) return BNO085_ERR_NO_DATA;
    *x_mps2 = latest_acceleration.x_mps2;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetAccelerationY(float *y_mps2)
{
    if (y_mps2 == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_acceleration) return BNO085_ERR_NO_DATA;
    *y_mps2 = latest_acceleration.y_mps2;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetAccelerationZ(float *z_mps2)
{
    if (z_mps2 == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_acceleration) return BNO085_ERR_NO_DATA;
    *z_mps2 = latest_acceleration.z_mps2;
    return BNO085_OK;
}

const char *BNO085_StatusString(BNO085_Status_t status)
{
    switch (status) {
        case BNO085_OK: return "ok";
        case BNO085_ERR_TIMEOUT: return "timeout";
        case BNO085_ERR_NO_RESPONSE: return "no response";
        case BNO085_ERR_INVALID_REPORT: return "invalid report";
        case BNO085_ERR_BUFFER_TOO_SMALL: return "buffer too small";
        case BNO085_ERR_BAD_PARAM: return "bad parameter";
        case BNO085_ERR_COMMAND_FAILED: return "command failed";
        case BNO085_ERR_PORT: return "platform I/O error";
        case BNO085_ERR_NO_DATA: return "no cached data";
        default: return "unknown";
    }
}
