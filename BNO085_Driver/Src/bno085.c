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
#define DEFAULT_SPI_TIMEOUT_MS          100U
#define DEFAULT_COMMAND_TIMEOUT_MS      500U
#define DEFAULT_STARTUP_TIMEOUT_MS     2000U
#define DEFAULT_DRAIN_TIMEOUT_MS        100U
#define DEFAULT_DRAIN_PACKET_LIMIT       32U
#define DEFAULT_FEATURE_RETRY_COUNT       2U
#define PRODUCT_ID_RESPONSE_COUNT        4U
#define WAIT_FOREVER            0xFFFFFFFFUL

/* SHTP channel numbers assigned by the BNO08X firmware. / 固件规定的通道号。 */
#define CHANNEL_COMMAND                  0U
#define CHANNEL_EXECUTABLE               1U
#define CHANNEL_CONTROL                  2U
#define CHANNEL_NON_WAKE                 3U
#define CHANNEL_WAKE                     4U

/* SH-2 report identifiers used or safely skipped by this driver.
 * 本驱动需要解析或安全跳过的 SH-2 报告 ID。 */
#define REPORT_ACCELEROMETER           0x01U
#define REPORT_GYROSCOPE               0x02U
#define REPORT_MAGNETOMETER            0x03U
#define REPORT_LINEAR_ACCELERATION     0x04U
#define REPORT_ROTATION_VECTOR         0x05U
#define REPORT_GRAVITY                 0x06U
#define REPORT_GYROSCOPE_UNCAL         0x07U
#define REPORT_GAME_ROTATION_VECTOR    0x08U
#define REPORT_MAGNETOMETER_UNCAL      0x0FU
#define REPORT_TAP_DETECTOR            0x10U
#define REPORT_STEP_COUNTER            0x11U
#define REPORT_STABILITY_CLASSIFIER    0x13U
#define REPORT_RAW_ACCELEROMETER       0x14U
#define REPORT_RAW_GYROSCOPE           0x15U
#define REPORT_RAW_MAGNETOMETER        0x16U
#define REPORT_STEP_DETECTOR           0x17U
#define REPORT_FLUSH_COMPLETE          0xEFU
#define REPORT_FORCE_FLUSH             0xF0U
#define REPORT_PRODUCT_ID_RESPONSE     0xF8U
#define REPORT_PRODUCT_ID_REQUEST      0xF9U
#define REPORT_SET_FEATURE             0xFDU
#define REPORT_GET_FEATURE_REQUEST     0xFEU
#define REPORT_GET_FEATURE_RESPONSE    0xFCU
#define REPORT_COMMAND_RESPONSE        0xF1U
#define REPORT_FRS_READ_RESPONSE       0xF3U
#define REPORT_FRS_WRITE_RESPONSE      0xF5U
#define REPORT_BASE_TIMESTAMP          0xFBU
#define REPORT_TIMESTAMP_REBASE        0xFAU
#define REPORT_COMMAND_REQUEST         0xF2U
#define COMMAND_SAVE_DCD               0x06U
#define COMMAND_ME_CALIBRATION         0x07U
#define COMMAND_TARE                   0x03U

/* Fixed-point conversion factors from the SH-2 report definitions.
 * SH-2 报告规定的定点数缩放系数。 */
#define Q14_SCALE        (1.0f / 16384.0f)
#define Q12_SCALE         (1.0f / 4096.0f)
#define Q8_SCALE           (1.0f / 256.0f)
#define Q9_SCALE           (1.0f / 512.0f)
#define Q4_SCALE            (1.0f / 16.0f)
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
static uint8_t rx_sequence[SHTP_CHANNEL_COUNT];
static bool rx_sequence_valid[SHTP_CHANNEL_COUNT];
static uint8_t sensor_sequence[256];
static bool sensor_sequence_valid[256];
static uint8_t command_sequence;
static uint8_t io_tx[IO_BUFFER_SIZE];
static uint8_t io_rx[IO_BUFFER_SIZE];
static uint8_t cargo_buffer[CARGO_BUFFER_SIZE];
static bool initialized;
static bool have_rotation_vector;
static bool have_euler;
static bool have_acceleration;
static bool have_gyroscope;
static bool have_magnetometer;
static bool have_game_rotation_vector;
static bool have_game_euler;
static bool have_linear_acceleration;
static bool have_gravity;
static bool have_uncalibrated_gyroscope;
static bool have_uncalibrated_magnetometer;
static bool have_raw_accelerometer;
static bool have_raw_gyroscope;
static bool have_raw_magnetometer;
static bool have_tap;
static bool have_step_counter;
static bool have_step_detector;
static bool have_stability;
static BNO085_RotationVector_t latest_rotation_vector;
static BNO085_Euler_t latest_euler;
static BNO085_Acceleration_t latest_acceleration;
static BNO085_Gyroscope_t latest_gyroscope;
static BNO085_Magnetometer_t latest_magnetometer;
static BNO085_RotationVector_t latest_game_rotation_vector;
static BNO085_Euler_t latest_game_euler;
static BNO085_LinearAcceleration_t latest_linear_acceleration;
static BNO085_Gravity_t latest_gravity;
static BNO085_UncalibratedGyroscope_t latest_uncalibrated_gyroscope;
static BNO085_UncalibratedMagnetometer_t latest_uncalibrated_magnetometer;
static BNO085_RawVector_t latest_raw_accelerometer;
static BNO085_RawGyroscope_t latest_raw_gyroscope;
static BNO085_RawVector_t latest_raw_magnetometer;
static BNO085_Tap_t latest_tap;
static BNO085_StepCounter_t latest_step_counter;
static BNO085_StepDetector_t latest_step_detector;
static BNO085_Stability_t latest_stability;
static BNO085_Diagnostics_t diagnostics;
static BNO085_Config_t driver_config = {
    DEFAULT_SPI_TIMEOUT_MS,
    DEFAULT_COMMAND_TIMEOUT_MS,
    DEFAULT_STARTUP_TIMEOUT_MS,
    DEFAULT_DRAIN_TIMEOUT_MS,
    DEFAULT_DRAIN_PACKET_LIMIT,
    DEFAULT_FEATURE_RETRY_COUNT
};
static BNO085_Callbacks_t driver_callbacks;

/* Non-blocking receive state.  CS remains asserted from the header DMA until
 * the last cargo DMA completes. / 非阻塞收包状态；包头到载荷结束期间 CS 保持低。 */
typedef enum {
    ASYNC_RX_IDLE = 0,
    ASYNC_RX_HEADER,
    ASYNC_RX_CARGO
} AsyncRxState_t;

static AsyncRxState_t async_rx_state;
static uint8_t async_header[SHTP_HEADER_SIZE];
static uint16_t async_cargo_length;
static uint16_t async_remaining;
static uint16_t async_copied;
static uint16_t async_chunk_length;
static uint8_t async_channel;
static bool async_continuation;
static bool async_chunk_is_cargo;
static uint32_t async_transfer_start_ms;
static uint32_t async_packet_timestamp_us;

static void async_rx_stop(void);

/** Record one physical packet and detect channel-local sequence gaps.
 *  记录物理包，并按通道检测 SHTP 序号跳变。 */
static void record_shtp_header(uint8_t channel, uint8_t sequence,
                               bool continuation)
{
    if (rx_sequence_valid[channel] &&
        (sequence != (uint8_t)(rx_sequence[channel] + 1U))) {
        diagnostics.shtp_sequence_gaps++;
    }
    rx_sequence[channel] = sequence;
    rx_sequence_valid[channel] = true;
    diagnostics.shtp_packets++;
    if (continuation) {
        diagnostics.continuation_packets++;
    }
}

/** Snapshot the platform's vendor-specific error for application logging.
 *  保存平台层最近一次底层错误，供应用诊断。 */
static void record_port_error(void)
{
    uint32_t raw_error = 0U;
    (void)BNO085_Port_GetLastError(&raw_error);
    diagnostics.port_errors++;
    diagnostics.port_raw_error = raw_error;
}

/** Detect missing SH-2 sensor reports independently from SHTP packets.
 *  独立于 SHTP 包序号，检测每种传感器报告的丢帧。 */
static void record_sensor_sequence(uint8_t report_id, uint8_t sequence)
{
    if (sensor_sequence_valid[report_id] &&
        (sequence != (uint8_t)(sensor_sequence[report_id] + 1U))) {
        diagnostics.sensor_sequence_gaps++;
    }
    sensor_sequence[report_id] = sequence;
    sensor_sequence_valid[report_id] = true;
}

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

/** Decode little-endian signed 32-bit data. / 读取小端有符号 32 位数。 */
static int32_t read_s32_le(const uint8_t *p)
{
    return (int32_t)read_u32_le(p);
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
                                     driver_config.spi_timeout_ms)) {
            record_port_error();
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
                                 driver_config.spi_timeout_ms)) {
        record_port_error();
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
        diagnostics.invalid_packets++;
        return BNO085_ERR_INVALID_REPORT;
    }

    *cargo_length = (uint16_t)(packet_length - SHTP_HEADER_SIZE);
    *channel = header[2];
    *continuation = ((header[1] & 0x80U) != 0U);
    record_shtp_header(*channel, header[3], *continuation);
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
        if ((packet_count >= driver_config.drain_packet_limit) ||
            timed_out(start_ms, driver_config.drain_timeout_ms)) {
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
                         driver_config.startup_timeout_ms);
    if ((status != BNO085_OK) &&
        (status != BNO085_ERR_BUFFER_TOO_SMALL)) {
        return status;
    }
    if ((channel != CHANNEL_COMMAND) || (cargo_length == 0U) ||
        continuation) {
        return BNO085_ERR_NO_RESPONSE;
    }

    start_ms = BNO085_Port_GetTimeMs();
    while (!timed_out(start_ms, driver_config.startup_timeout_ms)) {
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
        case REPORT_TAP_DETECTOR: return 5U;
        case REPORT_STEP_COUNTER: return 12U;
        case REPORT_STABILITY_CLASSIFIER: return 6U;
        case 0x14U: return 16U;
        case 0x15U: return 16U;
        case 0x16U: return 16U;
        case REPORT_STEP_DETECTOR: return 8U;
        case REPORT_FLUSH_COMPLETE: return 2U;
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

    /* A public configuration/command API may be called while streaming DMA
     * owns the bus. Abort that partial read and restore CS before beginning
     * the blocking host-write handshake. / 动态配置可能发生在 DMA 收包期间；
     * 写命令前必须中止该事务并释放 CS，避免两条 SPI 路径同时占用总线。 */
    if (async_rx_state != ASYNC_RX_IDLE) {
        async_rx_stop();
    }

    /* A write uses WAKE/H_INTN handshake.  Clear old device-to-host traffic
     * first, then assert WAKE and wait for H_INTN low.
     * 写入前先清旧包，再拉低 WAKE 并等待 H_INTN 拉低。 */
    status = drain_pending_packets();
    if (status != BNO085_OK) {
        return status;
    }
    status = wait_for_interrupt(driver_config.command_timeout_ms, true);
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

/** Receive either normal or wake sensor traffic. / 接收普通或唤醒传感器通道。 */
static BNO085_Status_t receive_sensor_packet(uint8_t *buffer,
                                              uint16_t *length,
                                              uint32_t timeout_ms)
{
    uint16_t capacity;
    uint32_t start_ms;
    if ((buffer == NULL) || (length == NULL)) return BNO085_ERR_BAD_PARAM;
    capacity = *length;
    start_ms = BNO085_Port_GetTimeMs();
    for (;;) {
        uint16_t cargo_length = 0U;
        uint8_t channel = 0U;
        bool continuation = false;
        BNO085_Status_t status = read_packet(buffer, capacity, &cargo_length,
            &channel, &continuation, remaining_time(start_ms, timeout_ms));
        if ((status != BNO085_OK) &&
            (status != BNO085_ERR_BUFFER_TOO_SMALL)) return status;
        if ((channel == CHANNEL_NON_WAKE) || (channel == CHANNEL_WAKE)) {
            *length = cargo_length;
            if (continuation) return BNO085_ERR_INVALID_REPORT;
            return status;
        }
        if (timed_out(start_ms, timeout_ms)) return BNO085_ERR_TIMEOUT;
    }
}

/**
 * Emit the 17-byte SH-2 Set Feature command.
 * 发送 17 字节 SH-2 Set Feature 命令；周期字段为小端微秒数。
 */
static bool is_supported_sensor_report(uint8_t report_id)
{
    switch (report_id) {
        case REPORT_ACCELEROMETER:
        case REPORT_GYROSCOPE:
        case REPORT_MAGNETOMETER:
        case REPORT_LINEAR_ACCELERATION:
        case REPORT_ROTATION_VECTOR:
        case REPORT_GRAVITY:
        case REPORT_GYROSCOPE_UNCAL:
        case REPORT_GAME_ROTATION_VECTOR:
        case REPORT_MAGNETOMETER_UNCAL:
        case REPORT_TAP_DETECTOR:
        case REPORT_STEP_COUNTER:
        case REPORT_STABILITY_CLASSIFIER:
        case REPORT_RAW_ACCELEROMETER:
        case REPORT_RAW_GYROSCOPE:
        case REPORT_RAW_MAGNETOMETER:
        case REPORT_STEP_DETECTOR:
            return true;
        default:
            return false;
    }
}

static BNO085_Status_t get_report_config(uint8_t report_id,
                                          BNO085_ReportConfig_t *config)
{
    uint8_t request[2] = { REPORT_GET_FEATURE_REQUEST, report_id };
    uint32_t start_ms;
    BNO085_Status_t status;

    status = send_packet(request, sizeof(request), CHANNEL_CONTROL);
    if (status != BNO085_OK) return status;
    start_ms = BNO085_Port_GetTimeMs();
    while (!timed_out(start_ms, driver_config.command_timeout_ms)) {
        uint16_t length = sizeof(cargo_buffer);
        uint16_t cursor = 0U;
        status = receive_channel(cargo_buffer, &length, CHANNEL_CONTROL,
            remaining_time(start_ms, driver_config.command_timeout_ms));
        if (status != BNO085_OK) return status;
        while (cursor < length) {
            uint8_t item_length = report_length(cargo_buffer[cursor]);
            const uint8_t *p = cargo_buffer + cursor;
            if ((item_length == 0U) ||
                ((uint16_t)(cursor + item_length) > length)) {
                diagnostics.invalid_packets++;
                return BNO085_ERR_INVALID_REPORT;
            }
            if ((p[0] == REPORT_GET_FEATURE_RESPONSE) &&
                (p[1] == report_id)) {
                config->flags = p[2];
                config->change_sensitivity = read_u16_le(p + 3U);
                config->interval_us = read_u32_le(p + 5U);
                config->batch_interval_us = read_u32_le(p + 9U);
                config->sensor_specific = read_u32_le(p + 13U);
                return BNO085_OK;
            }
            cursor = (uint16_t)(cursor + item_length);
        }
    }
    return BNO085_ERR_TIMEOUT;
}

/** Send and verify every SH-2 Set Feature field. / 发送并验证完整报告配置。 */
static BNO085_Status_t set_report_config(uint8_t report_id,
                                          const BNO085_ReportConfig_t *config)
{
    uint8_t command[17];
    uint8_t request[2] = { REPORT_GET_FEATURE_REQUEST, report_id };
    uint8_t attempt;
    BNO085_Status_t last_status = BNO085_ERR_COMMAND_FAILED;

    if ((config == NULL) || !is_supported_sensor_report(report_id) ||
        ((config->flags & 0xE0U) != 0U)) return BNO085_ERR_BAD_PARAM;
    memset(command, 0, sizeof(command));
    command[0] = REPORT_SET_FEATURE;
    command[1] = report_id;
    command[2] = config->flags;
    command[3] = (uint8_t)config->change_sensitivity;
    command[4] = (uint8_t)(config->change_sensitivity >> 8);
    command[5] = (uint8_t)config->interval_us;
    command[6] = (uint8_t)(config->interval_us >> 8);
    command[7] = (uint8_t)(config->interval_us >> 16);
    command[8] = (uint8_t)(config->interval_us >> 24);
    command[9] = (uint8_t)config->batch_interval_us;
    command[10] = (uint8_t)(config->batch_interval_us >> 8);
    command[11] = (uint8_t)(config->batch_interval_us >> 16);
    command[12] = (uint8_t)(config->batch_interval_us >> 24);
    command[13] = (uint8_t)config->sensor_specific;
    command[14] = (uint8_t)(config->sensor_specific >> 8);
    command[15] = (uint8_t)(config->sensor_specific >> 16);
    command[16] = (uint8_t)(config->sensor_specific >> 24);

    for (attempt = 0U; attempt < driver_config.feature_retry_count; attempt++) {
        uint32_t start_ms;
        last_status = send_packet(command, sizeof(command), CHANNEL_CONTROL);
        if (last_status != BNO085_OK) continue;
        last_status = send_packet(request, sizeof(request), CHANNEL_CONTROL);
        if (last_status != BNO085_OK) continue;

        /* Keep Set Feature and its Get Feature verification in one sequence.
         * Some firmware revisions process these control transactions as an
         * ordered pair. / 保持设置与读回确认位于同一控制事务序列。 */
        start_ms = BNO085_Port_GetTimeMs();
        while (!timed_out(start_ms, driver_config.command_timeout_ms)) {
            uint16_t length = sizeof(cargo_buffer);
            uint16_t cursor = 0U;
            last_status = receive_channel(cargo_buffer, &length,
                CHANNEL_CONTROL, remaining_time(start_ms,
                                      driver_config.command_timeout_ms));
            if (last_status != BNO085_OK) break;
            while (cursor < length) {
                uint8_t item_length = report_length(cargo_buffer[cursor]);
                const uint8_t *p = cargo_buffer + cursor;
                BNO085_ReportConfig_t effective;
                if ((item_length == 0U) ||
                    ((uint16_t)(cursor + item_length) > length)) {
                    diagnostics.invalid_packets++;
                    last_status = BNO085_ERR_INVALID_REPORT;
                    break;
                }
                if ((p[0] == REPORT_GET_FEATURE_RESPONSE) &&
                    (p[1] == report_id)) {
                    effective.flags = p[2];
                    effective.change_sensitivity = read_u16_le(p + 3U);
                    effective.interval_us = read_u32_le(p + 5U);
                    effective.batch_interval_us = read_u32_le(p + 9U);
                    effective.sensor_specific = read_u32_le(p + 13U);
                    diagnostics.last_feature_id = report_id;
                    diagnostics.requested_interval_us = config->interval_us;
                    diagnostics.effective_interval_us = effective.interval_us;
                    diagnostics.requested_batch_interval_us =
                        config->batch_interval_us;
                    diagnostics.effective_batch_interval_us =
                        effective.batch_interval_us;
                    diagnostics.requested_sensor_specific =
                        config->sensor_specific;
                    diagnostics.effective_sensor_specific =
                        effective.sensor_specific;
                    diagnostics.requested_change_sensitivity =
                        config->change_sensitivity;
                    diagnostics.effective_change_sensitivity =
                        effective.change_sensitivity;
                    diagnostics.requested_feature_flags = config->flags;
                    diagnostics.effective_feature_flags = effective.flags;
                    /* Periods may be quantized by firmware. Discrete values
                     * explicitly selected by the host must round-trip; zero
                     * sensitivity/sensor-specific values select defaults.
                     * 周期允许固件量化；主机显式配置的离散字段必须读回一致，
                     * 灵敏度和报告专用字段为 0 时则采用固件默认值。 */
                    if (((config->interval_us == 0U) ?
                         (effective.interval_us == 0U) :
                         ((effective.interval_us >=
                                             config->interval_us / 2U) &&
                          (effective.interval_us <= config->interval_us +
                                             config->interval_us / 2U))) &&
                        ((config->batch_interval_us == 0U) ?
                         (effective.batch_interval_us == 0U) :
                         ((effective.batch_interval_us >=
                                       config->batch_interval_us / 2U) &&
                          (effective.batch_interval_us <=
                                       config->batch_interval_us +
                                       config->batch_interval_us / 2U))) &&
                        (effective.flags == config->flags) &&
                        ((config->change_sensitivity == 0U) ||
                         (effective.change_sensitivity ==
                                           config->change_sensitivity)) &&
                        ((config->sensor_specific == 0U) ||
                         (effective.sensor_specific ==
                                           config->sensor_specific))) {
                        return BNO085_OK;
                    }
                    last_status = BNO085_ERR_COMMAND_FAILED;
                    break;
                }
                cursor = (uint16_t)(cursor + item_length);
            }
        }
    }
    diagnostics.feature_verify_failures++;
    return last_status;
}

static BNO085_Status_t set_report_interval(uint8_t report_id,
                                            uint32_t interval_us)
{
    BNO085_ReportConfig_t config;
    memset(&config, 0, sizeof(config));
    config.interval_us = interval_us;
    return set_report_config(report_id, &config);
}

/** Send an SH-2 command and match its response by command and sequence.
 *  发送 SH-2 命令，并同时按命令号和原请求序号匹配响应。 */
static BNO085_Status_t send_command_and_wait(uint8_t command_id,
                                              const uint8_t parameters[9])
{
    uint8_t request[12];
    uint8_t request_sequence = command_sequence++;
    uint32_t start_ms;
    BNO085_Status_t status;

    async_rx_stop();
    memset(request, 0, sizeof(request));
    request[0] = REPORT_COMMAND_REQUEST;
    request[1] = request_sequence;
    request[2] = command_id;
    if (parameters != NULL) {
        memcpy(request + 3U, parameters, 9U);
    }
    status = send_packet(request, sizeof(request), CHANNEL_CONTROL);
    if (status != BNO085_OK) {
        return status;
    }

    start_ms = BNO085_Port_GetTimeMs();
    while (!timed_out(start_ms, driver_config.command_timeout_ms)) {
        uint16_t length = sizeof(cargo_buffer);
        uint16_t cursor = 0U;

        status = receive_channel(cargo_buffer, &length, CHANNEL_CONTROL,
                                 remaining_time(start_ms,
                                                driver_config.command_timeout_ms));
        if (status != BNO085_OK) {
            return status;
        }
        while (cursor < length) {
            uint8_t item_length = report_length(cargo_buffer[cursor]);
            const uint8_t *p = cargo_buffer + cursor;

            if ((item_length == 0U) ||
                ((uint16_t)(cursor + item_length) > length)) {
                diagnostics.invalid_packets++;
                return BNO085_ERR_INVALID_REPORT;
            }
            if ((p[0] == REPORT_COMMAND_RESPONSE) &&
                ((p[2] & 0x7FU) == command_id) &&
                (p[3] == request_sequence)) {
                return (p[5] == 0U) ? BNO085_OK :
                                      BNO085_ERR_COMMAND_FAILED;
            }
            cursor = (uint16_t)(cursor + item_length);
        }
    }
    return BNO085_ERR_TIMEOUT;
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
                                             uint32_t packet_timestamp_us,
                                             uint32_t *events)
{
    uint16_t cursor = 0U;
    /* Keep timestamp arithmetic wider than the signed 32-bit wire fields.
     * This also makes negating INT32_MIN well-defined. / 时间运算使用 64 位，
     * 避免对 INT32_MIN 取负或累加 rebase 时产生有符号溢出。 */
    int64_t reference_delta_ticks = 0;

    while (cursor < length) {
        uint8_t item_length = report_length(payload[cursor]);
        const uint8_t *p = payload + cursor;
        uint32_t timestamp_us = packet_timestamp_us;

        if ((item_length == 0U) ||
            ((uint16_t)(cursor + item_length) > length)) {
            return BNO085_ERR_INVALID_REPORT;
        }
        if (p[0] == REPORT_BASE_TIMESTAMP) {
            /* Signed delta is measured backwards from the host interrupt.
             * 有符号基准差以 H_INTN 时刻为参考，单位 100 us。 */
            int32_t delta = read_s32_le(p + 1U);
            if (delta != INT32_MAX) {
                reference_delta_ticks = -(int64_t)delta;
            }
        } else if (p[0] == REPORT_TIMESTAMP_REBASE) {
            reference_delta_ticks += (int64_t)read_s32_le(p + 1U);
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
        } else if (p[0] == REPORT_GYROSCOPE) {
            /* Calibrated gyro vector is signed Q9 radians/second.
             * 校准陀螺仪为有符号 Q9，单位 rad/s。 */
            latest_gyroscope.x_rps =
                (float)read_s16_le(p + 4U) * Q9_SCALE;
            latest_gyroscope.y_rps =
                (float)read_s16_le(p + 6U) * Q9_SCALE;
            latest_gyroscope.z_rps =
                (float)read_s16_le(p + 8U) * Q9_SCALE;
            latest_gyroscope.sequence = p[1];
            latest_gyroscope.accuracy = (uint8_t)(p[2] & 0x03U);
            latest_gyroscope.timestamp_us = timestamp_us;
            have_gyroscope = true;
            *events |= BNO085_EVENT_GYROSCOPE;
        } else if (p[0] == REPORT_MAGNETOMETER) {
            /* Calibrated magnetic field is signed Q4 microtesla.
             * 校准磁场为有符号 Q4，单位 uT。 */
            latest_magnetometer.x_uT =
                (float)read_s16_le(p + 4U) * Q4_SCALE;
            latest_magnetometer.y_uT =
                (float)read_s16_le(p + 6U) * Q4_SCALE;
            latest_magnetometer.z_uT =
                (float)read_s16_le(p + 8U) * Q4_SCALE;
            latest_magnetometer.sequence = p[1];
            latest_magnetometer.accuracy = (uint8_t)(p[2] & 0x03U);
            latest_magnetometer.timestamp_us = timestamp_us;
            have_magnetometer = true;
            *events |= BNO085_EVENT_MAGNETOMETER;
        } else if (p[0] == REPORT_LINEAR_ACCELERATION) {
            latest_linear_acceleration.x_mps2 =
                (float)read_s16_le(p + 4U) * Q8_SCALE;
            latest_linear_acceleration.y_mps2 =
                (float)read_s16_le(p + 6U) * Q8_SCALE;
            latest_linear_acceleration.z_mps2 =
                (float)read_s16_le(p + 8U) * Q8_SCALE;
            latest_linear_acceleration.sequence = p[1];
            latest_linear_acceleration.accuracy = (uint8_t)(p[2] & 0x03U);
            have_linear_acceleration = true;
            *events |= BNO085_EVENT_LINEAR_ACCELERATION;
        } else if (p[0] == REPORT_GRAVITY) {
            latest_gravity.x_mps2 = (float)read_s16_le(p + 4U) * Q8_SCALE;
            latest_gravity.y_mps2 = (float)read_s16_le(p + 6U) * Q8_SCALE;
            latest_gravity.z_mps2 = (float)read_s16_le(p + 8U) * Q8_SCALE;
            latest_gravity.sequence = p[1];
            latest_gravity.accuracy = (uint8_t)(p[2] & 0x03U);
            have_gravity = true;
            *events |= BNO085_EVENT_GRAVITY;
        } else if (p[0] == REPORT_GYROSCOPE_UNCAL) {
            latest_uncalibrated_gyroscope.x_rps =
                (float)read_s16_le(p + 4U) * Q9_SCALE;
            latest_uncalibrated_gyroscope.y_rps =
                (float)read_s16_le(p + 6U) * Q9_SCALE;
            latest_uncalibrated_gyroscope.z_rps =
                (float)read_s16_le(p + 8U) * Q9_SCALE;
            latest_uncalibrated_gyroscope.bias_x_rps =
                (float)read_s16_le(p + 10U) * Q9_SCALE;
            latest_uncalibrated_gyroscope.bias_y_rps =
                (float)read_s16_le(p + 12U) * Q9_SCALE;
            latest_uncalibrated_gyroscope.bias_z_rps =
                (float)read_s16_le(p + 14U) * Q9_SCALE;
            latest_uncalibrated_gyroscope.sequence = p[1];
            latest_uncalibrated_gyroscope.accuracy =
                (uint8_t)(p[2] & 0x03U);
            have_uncalibrated_gyroscope = true;
            *events |= BNO085_EVENT_GYROSCOPE_UNCAL;
        } else if (p[0] == REPORT_MAGNETOMETER_UNCAL) {
            latest_uncalibrated_magnetometer.x_uT =
                (float)read_s16_le(p + 4U) * Q4_SCALE;
            latest_uncalibrated_magnetometer.y_uT =
                (float)read_s16_le(p + 6U) * Q4_SCALE;
            latest_uncalibrated_magnetometer.z_uT =
                (float)read_s16_le(p + 8U) * Q4_SCALE;
            latest_uncalibrated_magnetometer.bias_x_uT =
                (float)read_s16_le(p + 10U) * Q4_SCALE;
            latest_uncalibrated_magnetometer.bias_y_uT =
                (float)read_s16_le(p + 12U) * Q4_SCALE;
            latest_uncalibrated_magnetometer.bias_z_uT =
                (float)read_s16_le(p + 14U) * Q4_SCALE;
            latest_uncalibrated_magnetometer.sequence = p[1];
            latest_uncalibrated_magnetometer.accuracy =
                (uint8_t)(p[2] & 0x03U);
            have_uncalibrated_magnetometer = true;
            *events |= BNO085_EVENT_MAGNETOMETER_UNCAL;
        } else if (p[0] == REPORT_TAP_DETECTOR) {
            latest_tap.flags = p[4];
            latest_tap.sequence = p[1];
            latest_tap.accuracy = (uint8_t)(p[2] & 0x03U);
            have_tap = true;
            *events |= BNO085_EVENT_TAP;
        } else if (p[0] == REPORT_STEP_COUNTER) {
            latest_step_counter.latency_us = read_u32_le(p + 4U);
            latest_step_counter.steps = read_u32_le(p + 8U);
            latest_step_counter.sequence = p[1];
            latest_step_counter.accuracy = (uint8_t)(p[2] & 0x03U);
            have_step_counter = true;
            *events |= BNO085_EVENT_STEP_COUNTER;
        } else if (p[0] == REPORT_STEP_DETECTOR) {
            latest_step_detector.latency_us = read_u32_le(p + 4U);
            latest_step_detector.sequence = p[1];
            latest_step_detector.accuracy = (uint8_t)(p[2] & 0x03U);
            have_step_detector = true;
            *events |= BNO085_EVENT_STEP_DETECTOR;
        } else if (p[0] == REPORT_STABILITY_CLASSIFIER) {
            latest_stability.classification = (BNO085_StabilityClass_t)p[4];
            latest_stability.sequence = p[1];
            latest_stability.accuracy = (uint8_t)(p[2] & 0x03U);
            have_stability = true;
            *events |= BNO085_EVENT_STABILITY;
        } else if (p[0] == REPORT_RAW_ACCELEROMETER) {
            latest_raw_accelerometer.x_counts = read_s16_le(p + 4U);
            latest_raw_accelerometer.y_counts = read_s16_le(p + 6U);
            latest_raw_accelerometer.z_counts = read_s16_le(p + 8U);
            latest_raw_accelerometer.sensor_timestamp_us = read_u32_le(p + 12U);
            latest_raw_accelerometer.sequence = p[1];
            latest_raw_accelerometer.accuracy = (uint8_t)(p[2] & 0x03U);
            have_raw_accelerometer = true;
            *events |= BNO085_EVENT_RAW_ACCELEROMETER;
        } else if (p[0] == REPORT_RAW_GYROSCOPE) {
            latest_raw_gyroscope.x_counts = read_s16_le(p + 4U);
            latest_raw_gyroscope.y_counts = read_s16_le(p + 6U);
            latest_raw_gyroscope.z_counts = read_s16_le(p + 8U);
            latest_raw_gyroscope.temperature_counts = read_s16_le(p + 10U);
            latest_raw_gyroscope.sensor_timestamp_us = read_u32_le(p + 12U);
            latest_raw_gyroscope.sequence = p[1];
            latest_raw_gyroscope.accuracy = (uint8_t)(p[2] & 0x03U);
            have_raw_gyroscope = true;
            *events |= BNO085_EVENT_RAW_GYROSCOPE;
        } else if (p[0] == REPORT_RAW_MAGNETOMETER) {
            latest_raw_magnetometer.x_counts = read_s16_le(p + 4U);
            latest_raw_magnetometer.y_counts = read_s16_le(p + 6U);
            latest_raw_magnetometer.z_counts = read_s16_le(p + 8U);
            latest_raw_magnetometer.sensor_timestamp_us = read_u32_le(p + 12U);
            latest_raw_magnetometer.sequence = p[1];
            latest_raw_magnetometer.accuracy = (uint8_t)(p[2] & 0x03U);
            have_raw_magnetometer = true;
            *events |= BNO085_EVENT_RAW_MAGNETOMETER;
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
        } else if (p[0] == REPORT_GAME_ROTATION_VECTOR) {
            /* Game RV is a 6-axis quaternion: i, j, k, real in Q14.  It has
             * no two-byte angular-accuracy estimate because magnetometer is
             * intentionally excluded. / Game RV 为无磁力计的六轴 Q14 四元数。 */
            latest_game_rotation_vector.x =
                (float)read_s16_le(p + 4U) * Q14_SCALE;
            latest_game_rotation_vector.y =
                (float)read_s16_le(p + 6U) * Q14_SCALE;
            latest_game_rotation_vector.z =
                (float)read_s16_le(p + 8U) * Q14_SCALE;
            latest_game_rotation_vector.w =
                (float)read_s16_le(p + 10U) * Q14_SCALE;
            latest_game_rotation_vector.accuracy_rad = 0.0f;
            latest_game_rotation_vector.sequence = p[1];
            latest_game_rotation_vector.accuracy = (uint8_t)(p[2] & 0x03U);
            latest_game_rotation_vector.timestamp_us = timestamp_us;
            have_game_rotation_vector = true;
            quaternion_to_euler(&latest_game_rotation_vector,
                                &latest_game_euler);
            have_game_euler = true;
            *events |= BNO085_EVENT_GAME_ROTATION_VECTOR;
        }
        if ((p[0] != REPORT_BASE_TIMESTAMP) &&
            (p[0] != REPORT_TIMESTAMP_REBASE)) {
            /* SH-2 encodes delay as an 8-bit value and a three-bit exponent
             * in status bits 4:2; the final unit is 100 us.
             * delay 为 8 位数值乘 2^指数，指数位于 status[4:2]。 */
            int64_t delay_ticks = (int64_t)((uint32_t)p[3] <<
                                             ((p[2] >> 2) & 0x07U));
            int64_t adjusted = (int64_t)packet_timestamp_us +
                (reference_delta_ticks + delay_ticks) * 100LL;
            timestamp_us = (uint32_t)adjusted;
            record_sensor_sequence(p[0], p[1]);

            /* Timestamp is assigned after decoding so every supported report
             * below receives the same corrected sampling time. / 在完整解析
             * 后统一写入修正采样时刻，避免把接收完成时刻误当采样时刻。 */
            switch (p[0]) {
                case REPORT_ACCELEROMETER:
                    latest_acceleration.timestamp_us = timestamp_us; break;
                case REPORT_GYROSCOPE:
                    latest_gyroscope.timestamp_us = timestamp_us; break;
                case REPORT_MAGNETOMETER:
                    latest_magnetometer.timestamp_us = timestamp_us; break;
                case REPORT_ROTATION_VECTOR:
                    latest_rotation_vector.timestamp_us = timestamp_us;
                    latest_euler.timestamp_us = timestamp_us; break;
                case REPORT_GAME_ROTATION_VECTOR:
                    latest_game_rotation_vector.timestamp_us = timestamp_us;
                    latest_game_euler.timestamp_us = timestamp_us; break;
                case REPORT_LINEAR_ACCELERATION:
                    latest_linear_acceleration.timestamp_us = timestamp_us; break;
                case REPORT_GRAVITY:
                    latest_gravity.timestamp_us = timestamp_us; break;
                case REPORT_GYROSCOPE_UNCAL:
                    latest_uncalibrated_gyroscope.timestamp_us = timestamp_us; break;
                case REPORT_MAGNETOMETER_UNCAL:
                    latest_uncalibrated_magnetometer.timestamp_us = timestamp_us; break;
                case REPORT_TAP_DETECTOR:
                    latest_tap.timestamp_us = timestamp_us; break;
                case REPORT_STEP_COUNTER:
                    latest_step_counter.timestamp_us = timestamp_us; break;
                case REPORT_STEP_DETECTOR:
                    latest_step_detector.timestamp_us = timestamp_us; break;
                case REPORT_STABILITY_CLASSIFIER:
                    latest_stability.timestamp_us = timestamp_us; break;
                case REPORT_RAW_ACCELEROMETER:
                    latest_raw_accelerometer.timestamp_us = timestamp_us; break;
                case REPORT_RAW_GYROSCOPE:
                    latest_raw_gyroscope.timestamp_us = timestamp_us; break;
                case REPORT_RAW_MAGNETOMETER:
                    latest_raw_magnetometer.timestamp_us = timestamp_us; break;
                default: break;
            }
        }
        cursor = (uint16_t)(cursor + item_length);
    }
    return BNO085_OK;
}

#if defined(BNO085_TESTING)
/* These hooks exist only in the host-test object. They keep private protocol
 * helpers out of bno085.h and out of production firmware. / 以下钩子仅在主机
 * 测试构建中存在，不污染公共 API，也不进入量产固件。 */
BNO085_Status_t BNO085_Test_ParseSensorPayload(const uint8_t *payload,
                                                uint16_t length,
                                                uint32_t packet_timestamp_us,
                                                uint32_t *events)
{
    if ((payload == NULL) || (events == NULL)) {
        return BNO085_ERR_BAD_PARAM;
    }
    return parse_sensor_payload(payload, length, packet_timestamp_us, events);
}

void BNO085_Test_ResetState(void)
{
    async_rx_stop();
    initialized = false;
    memset(&diagnostics, 0, sizeof(diagnostics));
    memset(sensor_sequence_valid, 0, sizeof(sensor_sequence_valid));
    have_rotation_vector = false;
    have_euler = false;
    have_acceleration = false;
    have_gyroscope = false;
    have_magnetometer = false;
    have_game_rotation_vector = false;
    have_game_euler = false;
    have_linear_acceleration = false;
    have_gravity = false;
    have_uncalibrated_gyroscope = false;
    have_uncalibrated_magnetometer = false;
    have_raw_accelerometer = false;
    have_raw_gyroscope = false;
    have_raw_magnetometer = false;
    have_tap = false;
    have_step_counter = false;
    have_step_detector = false;
    have_stability = false;
}

void BNO085_Test_SetTransportActive(void)
{
    initialized = true;
    async_rx_state = ASYNC_RX_CARGO;
}
#endif

/** Return the async transport to a safe idle bus state. / 异步传输恢复为空闲总线。 */
static void async_rx_stop(void)
{
    BNO085_Port_SPITransferAsyncAbort();
    BNO085_Port_SetChipSelect(false);
    async_rx_state = ASYNC_RX_IDLE;
    async_remaining = 0U;
    async_chunk_length = 0U;
}

/** Start one zero-filled DMA/interrupt clocking operation. / 启动一次全零异步收包。 */
static BNO085_Status_t async_rx_start(uint8_t *destination,
                                      uint16_t length,
                                      bool destination_is_cargo)
{
    if ((destination == NULL) || (length == 0U) ||
        (length > IO_BUFFER_SIZE)) {
        return BNO085_ERR_BAD_PARAM;
    }

    memset(io_tx, 0, length);
    async_chunk_length = length;
    async_chunk_is_cargo = destination_is_cargo;
    async_transfer_start_ms = BNO085_Port_GetTimeMs();
    if (!BNO085_Port_SPITransferAsync(io_tx, destination, length)) {
        record_port_error();
        return BNO085_ERR_PORT;
    }
    return BNO085_PENDING;
}

/** Start the next bounded cargo chunk, draining overflow safely. / 启动下一段载荷。 */
static BNO085_Status_t async_rx_start_cargo_chunk(void)
{
    uint16_t count = async_remaining;
    uint8_t *destination;
    bool destination_is_cargo;

    if (count > IO_BUFFER_SIZE) {
        count = IO_BUFFER_SIZE;
    }
    if (async_copied < CARGO_BUFFER_SIZE) {
        uint16_t available = (uint16_t)(CARGO_BUFFER_SIZE - async_copied);
        if (count > available) {
            count = available;
        }
        destination = cargo_buffer + async_copied;
        destination_is_cargo = true;
    } else {
        destination = io_rx;
        destination_is_cargo = false;
    }
    return async_rx_start(destination, count, destination_is_cargo);
}

/* Public API implementation / 公共 API 实现。 */
void BNO085_GetDefaultConfig(BNO085_Config_t *config)
{
    if (config == NULL) return;
    config->spi_timeout_ms = DEFAULT_SPI_TIMEOUT_MS;
    config->command_timeout_ms = DEFAULT_COMMAND_TIMEOUT_MS;
    config->startup_timeout_ms = DEFAULT_STARTUP_TIMEOUT_MS;
    config->drain_timeout_ms = DEFAULT_DRAIN_TIMEOUT_MS;
    config->drain_packet_limit = DEFAULT_DRAIN_PACKET_LIMIT;
    config->feature_retry_count = DEFAULT_FEATURE_RETRY_COUNT;
}

BNO085_Status_t BNO085_SetConfig(const BNO085_Config_t *config)
{
    if ((config == NULL) || (config->spi_timeout_ms == 0U) ||
        (config->command_timeout_ms == 0U) ||
        (config->startup_timeout_ms == 0U) ||
        (config->drain_timeout_ms == 0U) ||
        (config->drain_packet_limit == 0U) ||
        (config->feature_retry_count == 0U)) return BNO085_ERR_BAD_PARAM;
    driver_config = *config;
    return BNO085_OK;
}

BNO085_Status_t BNO085_InitWithConfig(const BNO085_Config_t *config)
{
    BNO085_Status_t status = BNO085_SetConfig(config);
    return (status == BNO085_OK) ? BNO085_Init() : status;
}

void BNO085_SetCallbacks(const BNO085_Callbacks_t *callbacks)
{
    if (callbacks == NULL) memset(&driver_callbacks, 0, sizeof(driver_callbacks));
    else driver_callbacks = *callbacks;
}

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
    /* Stop a streaming DMA before changing CS/reset pins. / 改变总线前先停 DMA。 */
    async_rx_stop();

    /* Invalidate old samples first so getters never expose pre-reset data.
     * 先使旧缓存失效，避免复位后读到上一次运行的数据。 */
    initialized = false;
    have_rotation_vector = false;
    have_euler = false;
    have_acceleration = false;
    have_gyroscope = false;
    have_magnetometer = false;
    have_game_rotation_vector = false;
    have_game_euler = false;
    have_linear_acceleration = false;
    have_gravity = false;
    have_uncalibrated_gyroscope = false;
    have_uncalibrated_magnetometer = false;
    have_raw_accelerometer = false;
    have_raw_gyroscope = false;
    have_raw_magnetometer = false;
    have_tap = false;
    have_step_counter = false;
    have_step_detector = false;
    have_stability = false;
    memset(tx_sequence, 0, sizeof(tx_sequence));
    memset(rx_sequence, 0, sizeof(rx_sequence));
    memset(rx_sequence_valid, 0, sizeof(rx_sequence_valid));
    memset(sensor_sequence_valid, 0, sizeof(sensor_sequence_valid));
    command_sequence = 0U;
    diagnostics.hard_resets++;
    memset(&latest_rotation_vector, 0, sizeof(latest_rotation_vector));
    memset(&latest_euler, 0, sizeof(latest_euler));
    memset(&latest_acceleration, 0, sizeof(latest_acceleration));
    memset(&latest_gyroscope, 0, sizeof(latest_gyroscope));
    memset(&latest_magnetometer, 0, sizeof(latest_magnetometer));
    memset(&latest_game_rotation_vector, 0,
           sizeof(latest_game_rotation_vector));
    memset(&latest_game_euler, 0, sizeof(latest_game_euler));
    memset(&latest_linear_acceleration, 0, sizeof(latest_linear_acceleration));
    memset(&latest_gravity, 0, sizeof(latest_gravity));
    memset(&latest_uncalibrated_gyroscope, 0,
           sizeof(latest_uncalibrated_gyroscope));
    memset(&latest_uncalibrated_magnetometer, 0,
           sizeof(latest_uncalibrated_magnetometer));
    memset(&latest_raw_accelerometer, 0, sizeof(latest_raw_accelerometer));
    memset(&latest_raw_gyroscope, 0, sizeof(latest_raw_gyroscope));
    memset(&latest_raw_magnetometer, 0, sizeof(latest_raw_magnetometer));
    memset(&latest_tap, 0, sizeof(latest_tap));
    memset(&latest_step_counter, 0, sizeof(latest_step_counter));
    memset(&latest_step_detector, 0, sizeof(latest_step_detector));
    memset(&latest_stability, 0, sizeof(latest_stability));

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

BNO085_Status_t BNO085_RecoverTransport(void)
{
    if (!BNO085_Port_IsReady()) {
        return BNO085_ERR_PORT;
    }
    async_rx_stop();
    BNO085_Port_SetWake(false);
    BNO085_Port_SetChipSelect(false);
    if (!BNO085_Port_Recover()) {
        record_port_error();
        return BNO085_ERR_PORT;
    }
    /* A transfer may have been discarded; accept the next sequence as a new
     * baseline instead of reporting a misleading gap. / 丢弃中的包不计假丢包。 */
    memset(rx_sequence_valid, 0, sizeof(rx_sequence_valid));
    diagnostics.transport_recoveries++;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetDiagnostics(BNO085_Diagnostics_t *value)
{
    if (value == NULL) {
        return BNO085_ERR_BAD_PARAM;
    }
    *value = diagnostics;
    return BNO085_OK;
}

void BNO085_ClearDiagnostics(void)
{
    memset(&diagnostics, 0, sizeof(diagnostics));
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
        uint32_t remaining = remaining_time(start_ms,
                                            driver_config.command_timeout_ms);

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

BNO085_Status_t BNO085_EnableGyroscope(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_GYROSCOPE, report_interval_us);
}

BNO085_Status_t BNO085_EnableMagnetometer(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_MAGNETOMETER, report_interval_us);
}

BNO085_Status_t BNO085_EnableGameRotationVector(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_GAME_ROTATION_VECTOR,
                               report_interval_us);
}

BNO085_Status_t BNO085_EnableLinearAcceleration(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_LINEAR_ACCELERATION,
                               report_interval_us);
}

BNO085_Status_t BNO085_EnableGravity(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_GRAVITY, report_interval_us);
}

BNO085_Status_t BNO085_EnableUncalibratedGyroscope(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_GYROSCOPE_UNCAL, report_interval_us);
}

BNO085_Status_t BNO085_EnableUncalibratedMagnetometer(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_MAGNETOMETER_UNCAL, report_interval_us);
}

BNO085_Status_t BNO085_EnableRawAccelerometer(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_RAW_ACCELEROMETER, report_interval_us);
}

BNO085_Status_t BNO085_EnableRawGyroscope(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_RAW_GYROSCOPE, report_interval_us);
}

BNO085_Status_t BNO085_EnableRawMagnetometer(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_RAW_MAGNETOMETER, report_interval_us);
}

BNO085_Status_t BNO085_EnableTapDetector(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_TAP_DETECTOR, report_interval_us);
}

BNO085_Status_t BNO085_EnableStepCounter(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_STEP_COUNTER, report_interval_us);
}

BNO085_Status_t BNO085_EnableStepDetector(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_STEP_DETECTOR, report_interval_us);
}

BNO085_Status_t BNO085_EnableStabilityClassifier(uint32_t report_interval_us)
{
    return set_report_interval(REPORT_STABILITY_CLASSIFIER,
                               report_interval_us);
}

BNO085_Status_t BNO085_ConfigureReport(BNO085_ReportId_t report_id,
                                       const BNO085_ReportConfig_t *config)
{
    return set_report_config((uint8_t)report_id, config);
}

BNO085_Status_t BNO085_GetReportConfig(BNO085_ReportId_t report_id,
                                       BNO085_ReportConfig_t *config)
{
    if ((config == NULL) ||
        !is_supported_sensor_report((uint8_t)report_id)) {
        return BNO085_ERR_BAD_PARAM;
    }
    return get_report_config((uint8_t)report_id, config);
}

BNO085_Status_t BNO085_DisableReport(BNO085_ReportId_t report_id)
{
    BNO085_ReportConfig_t config;
    memset(&config, 0, sizeof(config));
    return set_report_config((uint8_t)report_id, &config);
}

BNO085_Status_t BNO085_FlushReport(BNO085_ReportId_t report_id)
{
    uint8_t request[2];
    if (!is_supported_sensor_report((uint8_t)report_id)) {
        return BNO085_ERR_BAD_PARAM;
    }
    request[0] = REPORT_FORCE_FLUSH;
    request[1] = (uint8_t)report_id;
    return send_packet(request, sizeof(request), CHANNEL_CONTROL);
}

BNO085_Status_t BNO085_SetCalibration(uint8_t calibration_flags)
{
    uint8_t parameters[9] = {0};

    if ((calibration_flags & 0xE0U) != 0U) {
        return BNO085_ERR_BAD_PARAM;
    }
    parameters[0] = (calibration_flags & BNO085_CAL_ACCELEROMETER) ? 1U : 0U;
    parameters[1] = (calibration_flags & BNO085_CAL_GYROSCOPE) ? 1U : 0U;
    parameters[2] = (calibration_flags & BNO085_CAL_MAGNETOMETER) ? 1U : 0U;
    parameters[3] = 0U; /* Configure subcommand. / 配置子命令。 */
    parameters[4] = (calibration_flags & BNO085_CAL_PLANAR) ? 1U : 0U;
    parameters[5] = (calibration_flags & BNO085_CAL_ON_TABLE) ? 1U : 0U;
    return send_command_and_wait(COMMAND_ME_CALIBRATION, parameters);
}

BNO085_Status_t BNO085_SaveCalibration(void)
{
    return send_command_and_wait(COMMAND_SAVE_DCD, NULL);
}

BNO085_Status_t BNO085_TareNow(uint8_t axes, BNO085_TareBasis_t basis)
{
    uint8_t parameters[9] = {0};

    if ((axes == 0U) || ((axes & 0xF8U) != 0U) ||
        (basis > BNO085_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR)) {
        return BNO085_ERR_BAD_PARAM;
    }
    parameters[0] = 0U; /* Tare Now subcommand. / 立即 Tare 子命令。 */
    parameters[1] = axes;
    parameters[2] = (uint8_t)basis;
    return send_command_and_wait(COMMAND_TARE, parameters);
}

BNO085_Status_t BNO085_PersistTare(void)
{
    uint8_t parameters[9] = {0};
    parameters[0] = 1U;
    return send_command_and_wait(COMMAND_TARE, parameters);
}

BNO085_Status_t BNO085_ClearTare(void)
{
    uint8_t parameters[9] = {0};
    /* Set Reorientation with an all-zero quaternion clears runtime tare.
     * 使用全零四元数执行 Set Reorientation 可清除运行时 Tare。 */
    parameters[0] = 2U;
    return send_command_and_wait(COMMAND_TARE, parameters);
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
        status = receive_sensor_packet(cargo_buffer, &length, remaining);
        if (status != BNO085_OK) {
            return status;
        }
        status = parse_sensor_payload(cargo_buffer, length,
                                      BNO085_Port_GetTimeMs() * 1000U,
                                      events);
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

BNO085_Status_t BNO085_PollAsync(uint32_t *events)
{
    BNO085_PortAsyncStatus_t port_status;
    BNO085_Status_t status;

    if (events == NULL) {
        return BNO085_ERR_BAD_PARAM;
    }
    *events = 0U;
    if (!initialized) {
        return BNO085_ERR_NO_RESPONSE;
    }

    if (async_rx_state == ASYNC_RX_IDLE) {
        if (!BNO085_DataReady()) {
            return BNO085_PENDING;
        }

        /* One physical SHTP packet is one CS-low interval.  Header and cargo
         * are separate DMA operations only because cargo length is in header.
         * 一个物理包只拉低一次 CS；因长度位于包头，DMA 分两段启动。 */
        BNO085_Port_SetChipSelect(true);
        async_packet_timestamp_us = BNO085_Port_GetTimeMs() * 1000U;
        async_rx_state = ASYNC_RX_HEADER;
        status = async_rx_start(async_header, sizeof(async_header), false);
        if (status != BNO085_PENDING) {
            async_rx_stop();
            return status;
        }
        return BNO085_PENDING;
    }

    port_status = BNO085_Port_SPITransferAsyncStatus();
    if (port_status == BNO085_PORT_ASYNC_BUSY) {
        if (timed_out(async_transfer_start_ms, driver_config.spi_timeout_ms)) {
            diagnostics.dma_timeouts++;
            async_rx_stop();
            return BNO085_ERR_TIMEOUT;
        }
        return BNO085_PENDING;
    }
    if (port_status != BNO085_PORT_ASYNC_COMPLETE) {
        record_port_error();
        async_rx_stop();
        return BNO085_ERR_PORT;
    }

    if (async_rx_state == ASYNC_RX_HEADER) {
        uint16_t raw_length = read_u16_le(async_header);
        uint16_t packet_length = (uint16_t)(raw_length & 0x7FFFU);

        if ((packet_length < SHTP_HEADER_SIZE) ||
            (raw_length == 0xFFFFU) ||
            (async_header[2] >= SHTP_CHANNEL_COUNT)) {
            async_rx_stop();
            diagnostics.invalid_packets++;
            return BNO085_ERR_INVALID_REPORT;
        }

        async_cargo_length = (uint16_t)(packet_length - SHTP_HEADER_SIZE);
        async_remaining = async_cargo_length;
        async_copied = 0U;
        async_channel = async_header[2];
        async_continuation = ((async_header[1] & 0x80U) != 0U);
        record_shtp_header(async_channel, async_header[3],
                           async_continuation);
        if (async_remaining == 0U) {
            async_rx_stop();
            return BNO085_OK;
        }

        async_rx_state = ASYNC_RX_CARGO;
        status = async_rx_start_cargo_chunk();
        if (status != BNO085_PENDING) {
            async_rx_stop();
            return status;
        }
        return BNO085_PENDING;
    }

    /* A cargo DMA chunk has completed. / 一段载荷 DMA 已完成。 */
    async_remaining = (uint16_t)(async_remaining - async_chunk_length);
    if (async_chunk_is_cargo) {
        async_copied = (uint16_t)(async_copied + async_chunk_length);
    }
    if (async_remaining > 0U) {
        status = async_rx_start_cargo_chunk();
        if (status != BNO085_PENDING) {
            async_rx_stop();
            return status;
        }
        return BNO085_PENDING;
    }

    async_rx_stop();
    if (async_cargo_length > CARGO_BUFFER_SIZE) {
        return BNO085_ERR_BUFFER_TOO_SMALL;
    }
    if (async_continuation) {
        diagnostics.invalid_packets++;
        return BNO085_ERR_INVALID_REPORT;
    }
    if ((async_channel != CHANNEL_NON_WAKE) &&
        (async_channel != CHANNEL_WAKE)) {
        /* Control/executable traffic is harmless during streaming. / 流式阶段忽略其他通道。 */
        return BNO085_OK;
    }
    return parse_sensor_payload(cargo_buffer, async_cargo_length,
                                async_packet_timestamp_us, events);
}

BNO085_Status_t BNO085_Process(uint32_t *events)
{
    uint32_t local_events = 0U;
    BNO085_Status_t status;
    if (events == NULL) events = &local_events;
    status = BNO085_PollAsync(events);
    if ((status == BNO085_OK) && (*events != 0U) &&
        (driver_callbacks.on_data != NULL)) {
        driver_callbacks.on_data(*events, driver_callbacks.user_context);
    } else if ((status != BNO085_OK) && (status != BNO085_PENDING) &&
               (driver_callbacks.on_error != NULL)) {
        driver_callbacks.on_error(status, driver_callbacks.user_context);
    }
    return status;
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

BNO085_Status_t BNO085_GetGyroscope(BNO085_Gyroscope_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_gyroscope) return BNO085_ERR_NO_DATA;
    *value = latest_gyroscope;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetGyroscopeX(float *x_rps)
{
    if (x_rps == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_gyroscope) return BNO085_ERR_NO_DATA;
    *x_rps = latest_gyroscope.x_rps;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetGyroscopeY(float *y_rps)
{
    if (y_rps == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_gyroscope) return BNO085_ERR_NO_DATA;
    *y_rps = latest_gyroscope.y_rps;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetGyroscopeZ(float *z_rps)
{
    if (z_rps == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_gyroscope) return BNO085_ERR_NO_DATA;
    *z_rps = latest_gyroscope.z_rps;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetMagnetometer(BNO085_Magnetometer_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_magnetometer) return BNO085_ERR_NO_DATA;
    *value = latest_magnetometer;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetMagnetometerX(float *x_uT)
{
    if (x_uT == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_magnetometer) return BNO085_ERR_NO_DATA;
    *x_uT = latest_magnetometer.x_uT;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetMagnetometerY(float *y_uT)
{
    if (y_uT == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_magnetometer) return BNO085_ERR_NO_DATA;
    *y_uT = latest_magnetometer.y_uT;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetMagnetometerZ(float *z_uT)
{
    if (z_uT == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_magnetometer) return BNO085_ERR_NO_DATA;
    *z_uT = latest_magnetometer.z_uT;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetGameRotationVector(BNO085_RotationVector_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_game_rotation_vector) return BNO085_ERR_NO_DATA;
    *value = latest_game_rotation_vector;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetGameEuler(BNO085_Euler_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_game_euler) return BNO085_ERR_NO_DATA;
    *value = latest_game_euler;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetLinearAcceleration(BNO085_LinearAcceleration_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_linear_acceleration) return BNO085_ERR_NO_DATA;
    *value = latest_linear_acceleration;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetGravity(BNO085_Gravity_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_gravity) return BNO085_ERR_NO_DATA;
    *value = latest_gravity;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetUncalibratedGyroscope(BNO085_UncalibratedGyroscope_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_uncalibrated_gyroscope) return BNO085_ERR_NO_DATA;
    *value = latest_uncalibrated_gyroscope;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetUncalibratedMagnetometer(BNO085_UncalibratedMagnetometer_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_uncalibrated_magnetometer) return BNO085_ERR_NO_DATA;
    *value = latest_uncalibrated_magnetometer;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetRawAccelerometer(BNO085_RawVector_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_raw_accelerometer) return BNO085_ERR_NO_DATA;
    *value = latest_raw_accelerometer;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetRawGyroscope(BNO085_RawGyroscope_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_raw_gyroscope) return BNO085_ERR_NO_DATA;
    *value = latest_raw_gyroscope;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetRawMagnetometer(BNO085_RawVector_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_raw_magnetometer) return BNO085_ERR_NO_DATA;
    *value = latest_raw_magnetometer;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetTap(BNO085_Tap_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_tap) return BNO085_ERR_NO_DATA;
    *value = latest_tap;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetStepCounter(BNO085_StepCounter_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_step_counter) return BNO085_ERR_NO_DATA;
    *value = latest_step_counter;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetStepDetector(BNO085_StepDetector_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_step_detector) return BNO085_ERR_NO_DATA;
    *value = latest_step_detector;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetStability(BNO085_Stability_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    if (!have_stability) return BNO085_ERR_NO_DATA;
    *value = latest_stability;
    return BNO085_OK;
}

BNO085_Status_t BNO085_GetCalibrationStatus(BNO085_CalibrationStatus_t *value)
{
    if (value == NULL) return BNO085_ERR_BAD_PARAM;
    value->accelerometer = have_acceleration ? latest_acceleration.accuracy : 0U;
    value->gyroscope = have_gyroscope ? latest_gyroscope.accuracy : 0U;
    value->magnetometer = have_magnetometer ? latest_magnetometer.accuracy : 0U;
    value->rotation_vector = have_rotation_vector ?
                             latest_rotation_vector.accuracy : 0U;
    return (have_acceleration || have_gyroscope || have_magnetometer ||
            have_rotation_vector) ? BNO085_OK : BNO085_ERR_NO_DATA;
}

bool BNO085_IsCalibrationReady(uint8_t minimum_accuracy)
{
    if (minimum_accuracy > 3U) return false;
    return have_acceleration && have_gyroscope && have_magnetometer &&
           have_rotation_vector &&
           (latest_acceleration.accuracy >= minimum_accuracy) &&
           (latest_gyroscope.accuracy >= minimum_accuracy) &&
           (latest_magnetometer.accuracy >= minimum_accuracy) &&
           (latest_rotation_vector.accuracy >= minimum_accuracy);
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
        case BNO085_PENDING: return "pending";
        default: return "unknown";
    }
}
