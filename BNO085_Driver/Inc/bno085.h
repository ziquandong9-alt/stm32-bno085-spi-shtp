/* SPDX-License-Identifier: BSD-3-Clause */
/**
 * @file bno085.h
 * @brief BNO085 SPI/SHTP driver public API.
 *        BNO085 SPI/SHTP 驱动的应用层公共接口。
 *
 * This header intentionally contains no STM32 or other vendor HAL types.
 * The application calls BNO085_Poll() to receive data, then reads the latest
 * coherent sample through the cached getters.
 *
 * 本头文件有意不包含 STM32 或其他厂商 HAL 类型。应用先调用 BNO085_Poll()
 * 接收并解析数据，再通过缓存 Getter 获取同一帧中的最新结果。
 */
#ifndef BNO085_H
#define BNO085_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/** BNO085_Poll() decoded a new rotation-vector sample. / 收到新姿态样本。 */
#define BNO085_EVENT_ROTATION_VECTOR  (1UL << 0)
/** BNO085_Poll() decoded a new accelerometer sample. / 收到新加速度样本。 */
#define BNO085_EVENT_ACCELEROMETER    (1UL << 1)
/** A calibrated gyroscope sample is available. / 收到校准陀螺仪样本。 */
#define BNO085_EVENT_GYROSCOPE        (1UL << 2)
/** A calibrated magnetic-field sample is available. / 收到校准磁场样本。 */
#define BNO085_EVENT_MAGNETOMETER     (1UL << 3)
/** A Game Rotation Vector sample is available. / 收到游戏旋转矢量样本。 */
#define BNO085_EVENT_GAME_ROTATION_VECTOR (1UL << 4)
#define BNO085_EVENT_LINEAR_ACCELERATION  (1UL << 5)
#define BNO085_EVENT_GRAVITY              (1UL << 6)
#define BNO085_EVENT_GYROSCOPE_UNCAL      (1UL << 7)
#define BNO085_EVENT_MAGNETOMETER_UNCAL   (1UL << 8)
#define BNO085_EVENT_RAW_ACCELEROMETER    (1UL << 9)
#define BNO085_EVENT_RAW_GYROSCOPE        (1UL << 10)
#define BNO085_EVENT_RAW_MAGNETOMETER     (1UL << 11)
/** A SHTP command-channel error list was received. / 收到 SHTP 错误列表。 */
#define BNO085_EVENT_DIAGNOSTIC           (1UL << 31)

/** Driver result codes. / 驱动返回状态。 */
typedef enum {
    BNO085_OK = 0,                 /**< Success. / 成功。 */
    BNO085_ERR_TIMEOUT,            /**< Timed out waiting for H_INTN/data. / 等待超时。 */
    BNO085_ERR_NO_RESPONSE,        /**< Device/core is not ready. / 设备或核心未就绪。 */
    BNO085_ERR_INVALID_REPORT,     /**< Malformed/unsupported SHTP cargo. / 报告非法。 */
    BNO085_ERR_BUFFER_TOO_SMALL,   /**< Cargo exceeded internal capacity. / 缓冲不足。 */
    BNO085_ERR_BAD_PARAM,          /**< NULL or invalid argument. / 参数错误。 */
    BNO085_ERR_COMMAND_FAILED,     /**< SH-2 rejected a command. / SH-2 命令失败。 */
    BNO085_ERR_PORT,               /**< Platform SPI/GPIO failure. / 平台层 I/O 失败。 */
    BNO085_ERR_NO_DATA,            /**< Getter called before first sample. / 尚无缓存。 */
    BNO085_PENDING                 /**< Async transfer is idle/in progress. / 异步事务尚未完成。 */
} BNO085_Status_t;

/** Product and firmware identity returned by SH-2. / SH-2 产品与固件信息。 */
typedef struct {
    uint8_t reset_cause;
    uint8_t sw_major;
    uint8_t sw_minor;
    uint16_t sw_patch;
    uint32_t sw_part_number;
    uint32_t build_number;
} BNO085_ProductInfo_t;

/** Raw SH-2 rotation-vector quaternion. / SH-2 Rotation Vector 四元数原始结果。 */
typedef struct {
    float w;                 /**< Quaternion scalar. / 四元数标量。 */
    float x;                 /**< Quaternion X. / 四元数 X。 */
    float y;                 /**< Quaternion Y. / 四元数 Y。 */
    float z;                 /**< Quaternion Z. / 四元数 Z。 */
    float accuracy_rad;      /**< Estimated angular error in radians. / 角误差估计。 */
    uint8_t accuracy;        /**< SH-2 status: 0 unreliable ... 3 high. / 精度 0~3。 */
    uint8_t sequence;        /**< Sensor report sequence number. / 传感器报告序号。 */
    uint32_t timestamp_us;   /**< MCU time corrected to sampling instant. / 修正到采样时刻的 MCU 微秒时间。 */
} BNO085_RotationVector_t;

/** Z-Y-X Euler angles converted once per sample. / 每帧只换算一次的 Z-Y-X 欧拉角。 */
typedef struct {
    float yaw_deg;           /**< Rotation about Z, degrees. / 绕 Z 轴偏航角。 */
    float roll_deg;          /**< Rotation about X, degrees. / 绕 X 轴横滚角。 */
    float pitch_deg;         /**< Rotation about Y, degrees. / 绕 Y 轴俯仰角。 */
    uint8_t accuracy;
    uint32_t timestamp_us;
} BNO085_Euler_t;

/** Calibrated acceleration including gravity. / 含重力的校准三轴加速度。 */
typedef struct {
    float x_mps2;            /**< X acceleration, m/s^2. / X 轴加速度。 */
    float y_mps2;            /**< Y acceleration, m/s^2. / Y 轴加速度。 */
    float z_mps2;            /**< Z acceleration, m/s^2. / Z 轴加速度。 */
    uint8_t accuracy;
    uint8_t sequence;
    uint32_t timestamp_us;
} BNO085_Acceleration_t;

/** Calibrated angular velocity in radians per second. / 校准角速度，单位 rad/s。 */
typedef struct {
    float x_rps;
    float y_rps;
    float z_rps;
    uint8_t accuracy;
    uint8_t sequence;
    uint32_t timestamp_us;
} BNO085_Gyroscope_t;

/** Calibrated magnetic field in microtesla. / 校准磁场，单位 uT。 */
typedef struct {
    float x_uT;
    float y_uT;
    float z_uT;
    uint8_t accuracy;
    uint8_t sequence;
    uint32_t timestamp_us;
} BNO085_Magnetometer_t;

/** Linear acceleration excludes gravity; gravity contains gravity only.
 *  线性加速度已去除重力，重力向量则只保留重力分量。 */
typedef BNO085_Acceleration_t BNO085_LinearAcceleration_t;
typedef BNO085_Acceleration_t BNO085_Gravity_t;

/** Uncalibrated angular velocity plus estimated drift, Q9 rad/s on wire.
 *  未校准角速度及估计零偏；线上格式为 Q9 rad/s。 */
typedef struct {
    float x_rps, y_rps, z_rps;
    float bias_x_rps, bias_y_rps, bias_z_rps;
    uint8_t accuracy, sequence;
    uint32_t timestamp_us;
} BNO085_UncalibratedGyroscope_t;

/** Uncalibrated magnetic field plus estimated hard-iron bias, Q4 uT.
 *  未校准磁场及估计硬铁偏置；线上格式为 Q4 uT。 */
typedef struct {
    float x_uT, y_uT, z_uT;
    float bias_x_uT, bias_y_uT, bias_z_uT;
    uint8_t accuracy, sequence;
    uint32_t timestamp_us;
} BNO085_UncalibratedMagnetometer_t;

/** Raw accelerometer/magnetometer ADC counts and sensor-local sample timer.
 *  加速度计/磁力计原始 ADC 计数及传感器本地采样计时。 */
typedef struct {
    int16_t x_counts, y_counts, z_counts;
    uint32_t sensor_timestamp_us;
    uint8_t accuracy, sequence;
    uint32_t timestamp_us;
} BNO085_RawVector_t;

/** Raw gyroscope additionally reports a raw temperature ADC count.
 *  原始陀螺仪还包含温度 ADC 计数。 */
typedef struct {
    int16_t x_counts, y_counts, z_counts, temperature_counts;
    uint32_t sensor_timestamp_us;
    uint8_t accuracy, sequence;
    uint32_t timestamp_us;
} BNO085_RawGyroscope_t;

/** Runtime counters for field diagnosis; counters saturate only at uint32 wrap.
 *  现场诊断计数；可用于判断丢包、协议错误和总线恢复是否发生。 */
typedef struct {
    uint32_t shtp_packets;
    uint32_t shtp_sequence_gaps;
    uint32_t sensor_sequence_gaps;
    uint32_t invalid_packets;
    uint32_t continuation_packets;
    uint32_t port_errors;
    uint32_t dma_timeouts;
    uint32_t feature_verify_failures;
    uint32_t shtp_error_reports;
    uint32_t transport_recoveries;
    uint32_t hard_resets;
    uint32_t port_raw_error;
    uint32_t requested_interval_us;
    uint32_t effective_interval_us;
    uint8_t last_shtp_error;
    uint8_t last_error_channel;
    uint8_t last_feature_id;
} BNO085_Diagnostics_t;

/**
 * Initialize and hardware-reset BNO085. / 初始化并硬件复位 BNO085。
 * @pre A platform adapter such as BNO085_STM32_Port_Init() is ready.
 *      必须先初始化平台适配层。
 */
BNO085_Status_t BNO085_Init(void);

/** Repeat the complete reset/handshake sequence. / 重新执行完整复位和握手。 */
BNO085_Status_t BNO085_Reset(void);

/** Recover SPI/DMA only, preserving SH-2 report configuration. / 仅恢复总线，不复位传感器。 */
BNO085_Status_t BNO085_RecoverTransport(void);

/** Copy/clear diagnostic counters. / 读取或清零诊断计数。 */
BNO085_Status_t BNO085_GetDiagnostics(BNO085_Diagnostics_t *diagnostics);
void BNO085_ClearDiagnostics(void);

/** Request the first SH-2 product-ID entry. / 请求第一条产品与固件信息。 */
BNO085_Status_t BNO085_GetProductInfo(BNO085_ProductInfo_t *info);

/** Enable 9-axis Rotation Vector; 10000 us requests 100 Hz. / 启用九轴姿态。 */
BNO085_Status_t BNO085_EnableRotationVector(uint32_t report_interval_us);

/** Enable calibrated acceleration including gravity. / 启用含重力校准加速度。 */
BNO085_Status_t BNO085_EnableAccelerometer(uint32_t report_interval_us);

/** Enable calibrated gyroscope, Q9 rad/s. / 启用校准陀螺仪。 */
BNO085_Status_t BNO085_EnableGyroscope(uint32_t report_interval_us);

/** Enable calibrated magnetic field, Q4 uT. / 启用校准磁力计。 */
BNO085_Status_t BNO085_EnableMagnetometer(uint32_t report_interval_us);

/** Enable 6-axis Game Rotation Vector (no magnetometer). / 启用六轴游戏旋转矢量。 */
BNO085_Status_t BNO085_EnableGameRotationVector(uint32_t report_interval_us);
BNO085_Status_t BNO085_EnableLinearAcceleration(uint32_t report_interval_us);
BNO085_Status_t BNO085_EnableGravity(uint32_t report_interval_us);
BNO085_Status_t BNO085_EnableUncalibratedGyroscope(uint32_t report_interval_us);
BNO085_Status_t BNO085_EnableUncalibratedMagnetometer(uint32_t report_interval_us);
BNO085_Status_t BNO085_EnableRawAccelerometer(uint32_t report_interval_us);
BNO085_Status_t BNO085_EnableRawGyroscope(uint32_t report_interval_us);
BNO085_Status_t BNO085_EnableRawMagnetometer(uint32_t report_interval_us);

/** Enable selected MotionEngine calibration algorithms (bitwise OR flags).
 *  启用选定的 MotionEngine 校准算法，各标志可按位或。 */
#define BNO085_CAL_ACCELEROMETER  (1U << 0)
#define BNO085_CAL_GYROSCOPE      (1U << 1)
#define BNO085_CAL_MAGNETOMETER   (1U << 2)
#define BNO085_CAL_PLANAR         (1U << 3)
#define BNO085_CAL_ON_TABLE       (1U << 4)
BNO085_Status_t BNO085_SetCalibration(uint8_t calibration_flags);
/** Save current Dynamic Calibration Data to BNO085 flash. / 保存当前动态校准。 */
BNO085_Status_t BNO085_SaveCalibration(void);

/** Tare axes and rotation-vector bases. / Tare 轴掩码与参考旋转矢量。 */
#define BNO085_TARE_X  (1U << 0)
#define BNO085_TARE_Y  (1U << 1)
#define BNO085_TARE_Z  (1U << 2)
typedef enum {
    BNO085_TARE_BASIS_ROTATION_VECTOR = 0,
    BNO085_TARE_BASIS_GAME_ROTATION_VECTOR = 1,
    BNO085_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR = 2
} BNO085_TareBasis_t;
/** Apply a runtime tare; use XYZ or Z for firmware-portable behavior.
 *  执行运行时归零；为兼容固件，建议使用 XYZ 全轴或仅 Z 轴。 */
BNO085_Status_t BNO085_TareNow(uint8_t axes, BNO085_TareBasis_t basis);
/** Persist the last eligible tare to flash. / 把最近一次可持久化 Tare 写入闪存。 */
BNO085_Status_t BNO085_PersistTare(void);
/** Clear the current runtime tare without erasing DCD. / 清除当前运行时 Tare。 */
BNO085_Status_t BNO085_ClearTare(void);

/**
 * Wait for one useful sensor cargo, decode it and update the cache.
 * 等待并解析一组有效传感器报告，然后更新内部缓存。
 * @param timeout_ms Maximum wait time in milliseconds. / 最大等待毫秒数。
 * @param events ORed BNO085_EVENT_* flags updated by this call. / 本次事件位。
 */
BNO085_Status_t BNO085_Poll(uint32_t timeout_ms, uint32_t *events);

/**
 * Advance one non-blocking receive step. / 推进一次非阻塞接收状态机。
 *
 * The platform adapter starts an interrupt/DMA transfer and this function
 * returns BNO085_PENDING immediately. Call it again after H_INTN or the SPI
 * completion interrupt wakes the task. BNO085_OK means one packet was fully
 * decoded; BNO085_PENDING is not an error.
 *
 * 平台层会启动中断或 DMA 传输，本函数立即返回 BNO085_PENDING；H_INTN 或
 * SPI 完成中断唤醒任务后再次调用。返回 BNO085_OK 表示一个包已完整解析。
 */
BNO085_Status_t BNO085_PollAsync(uint32_t *events);

/** Return true while active-low H_INTN is asserted. / H_INTN 为低时返回 true。 */
bool BNO085_DataReady(void);

/** Copy the latest quaternion snapshot; no SPI access. / 复制最新四元数，不访问 SPI。 */
BNO085_Status_t BNO085_GetRotationVector(BNO085_RotationVector_t *value);
/** Copy yaw/roll/pitch from one coherent snapshot. / 复制同一帧的三个欧拉角。 */
BNO085_Status_t BNO085_GetEuler(BNO085_Euler_t *value);
/** Read cached yaw in degrees. / 获取缓存的偏航角，单位度。 */
BNO085_Status_t BNO085_GetYaw(float *yaw_deg);
/** Read cached roll in degrees. / 获取缓存的横滚角，单位度。 */
BNO085_Status_t BNO085_GetRoll(float *roll_deg);
/** Read cached pitch in degrees. / 获取缓存的俯仰角，单位度。 */
BNO085_Status_t BNO085_GetPitch(float *pitch_deg);

/** Copy one coherent acceleration snapshot. / 复制同一帧三轴加速度。 */
BNO085_Status_t BNO085_GetAcceleration(BNO085_Acceleration_t *value);
/** Read cached X acceleration in m/s^2. / 获取缓存 X 轴加速度。 */
BNO085_Status_t BNO085_GetAccelerationX(float *x_mps2);
/** Read cached Y acceleration in m/s^2. / 获取缓存 Y 轴加速度。 */
BNO085_Status_t BNO085_GetAccelerationY(float *y_mps2);
/** Read cached Z acceleration in m/s^2. / 获取缓存 Z 轴加速度。 */
BNO085_Status_t BNO085_GetAccelerationZ(float *z_mps2);

/** Copy the latest calibrated angular-velocity sample. / 获取校准角速度快照。 */
BNO085_Status_t BNO085_GetGyroscope(BNO085_Gyroscope_t *value);
/** Read cached X angular velocity in rad/s. / 获取缓存 X 轴角速度。 */
BNO085_Status_t BNO085_GetGyroscopeX(float *x_rps);
/** Read cached Y angular velocity in rad/s. / 获取缓存 Y 轴角速度。 */
BNO085_Status_t BNO085_GetGyroscopeY(float *y_rps);
/** Read cached Z angular velocity in rad/s. / 获取缓存 Z 轴角速度。 */
BNO085_Status_t BNO085_GetGyroscopeZ(float *z_rps);

/** Copy the latest calibrated magnetic-field sample. / 获取校准磁场快照。 */
BNO085_Status_t BNO085_GetMagnetometer(BNO085_Magnetometer_t *value);
/** Read cached X magnetic field in uT. / 获取缓存 X 轴磁场。 */
BNO085_Status_t BNO085_GetMagnetometerX(float *x_uT);
/** Read cached Y magnetic field in uT. / 获取缓存 Y 轴磁场。 */
BNO085_Status_t BNO085_GetMagnetometerY(float *y_uT);
/** Read cached Z magnetic field in uT. / 获取缓存 Z 轴磁场。 */
BNO085_Status_t BNO085_GetMagnetometerZ(float *z_uT);

/** Game Rotation Vector and its cached Euler angles. / 游戏旋转矢量及其欧拉角。 */
BNO085_Status_t BNO085_GetGameRotationVector(BNO085_RotationVector_t *value);
BNO085_Status_t BNO085_GetGameEuler(BNO085_Euler_t *value);
BNO085_Status_t BNO085_GetLinearAcceleration(BNO085_LinearAcceleration_t *value);
BNO085_Status_t BNO085_GetGravity(BNO085_Gravity_t *value);
BNO085_Status_t BNO085_GetUncalibratedGyroscope(BNO085_UncalibratedGyroscope_t *value);
BNO085_Status_t BNO085_GetUncalibratedMagnetometer(BNO085_UncalibratedMagnetometer_t *value);
BNO085_Status_t BNO085_GetRawAccelerometer(BNO085_RawVector_t *value);
BNO085_Status_t BNO085_GetRawGyroscope(BNO085_RawGyroscope_t *value);
BNO085_Status_t BNO085_GetRawMagnetometer(BNO085_RawVector_t *value);

/** Human-readable status text for diagnostics. / 将状态码转换为调试字符串。 */
const char *BNO085_StatusString(BNO085_Status_t status);

#ifdef __cplusplus
}
#endif
#endif
