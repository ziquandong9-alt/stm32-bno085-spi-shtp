/* SPDX-License-Identifier: BSD-3-Clause */
/* Host-side parser regression tests. / 在 PC 上运行的解析器回归测试。 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bno085.h"
#include "bno085_port.h"
#include "bno085_test_hooks.h"

static uint32_t fake_time_ms;
static bool fake_recover_ok = true;
static BNO085_PortError_t fake_port_error = BNO085_PORT_ERROR_NONE;
static uint32_t fake_raw_error;

bool BNO085_Port_IsReady(void) { return true; }
bool BNO085_Port_SPITransfer(const uint8_t *tx, uint8_t *rx,
                            uint16_t length, uint32_t timeout_ms)
{ (void)tx; (void)rx; (void)length; (void)timeout_ms; return false; }
bool BNO085_Port_SPITransferAsync(const uint8_t *tx, uint8_t *rx,
                                 uint16_t length)
{ (void)tx; (void)rx; (void)length; return false; }
BNO085_PortAsyncStatus_t BNO085_Port_SPITransferAsyncStatus(void)
{ return BNO085_PORT_ASYNC_IDLE; }
void BNO085_Port_SPITransferAsyncAbort(void) {}
BNO085_PortError_t BNO085_Port_GetLastError(uint32_t *raw_error)
{ if (raw_error != NULL) *raw_error = fake_raw_error; return fake_port_error; }
bool BNO085_Port_Recover(void) { return fake_recover_ok; }
void BNO085_Port_SetChipSelect(bool asserted) { (void)asserted; }
void BNO085_Port_SetWake(bool asserted) { (void)asserted; }
void BNO085_Port_SetReset(bool asserted) { (void)asserted; }
bool BNO085_Port_IsInterruptAsserted(void) { return false; }
uint32_t BNO085_Port_GetTimeMs(void) { return fake_time_ms; }
void BNO085_Port_DelayMs(uint32_t delay_ms) { fake_time_ms += delay_ms; }

static bool close_to(float actual, float expected)
{
    return fabsf(actual - expected) < 0.0001f;
}

static void test_timestamp_and_acceleration(void)
{
    uint8_t payload[] = {
        0xFBU, 100U, 0U, 0U, 0U,       /* base: host - 10,000 us */
        0x01U, 0U, 0U, 5U,             /* delay: +500 us */
        0x00U, 0x01U, 0x00U, 0xFEU, 0x80U, 0x00U
    };
    uint32_t events = 0U;
    BNO085_Acceleration_t value;

    assert(BNO085_Test_ParseSensorPayload(payload, sizeof(payload), 1000000U,
                                          &events) == BNO085_OK);
    assert((events & BNO085_EVENT_ACCELEROMETER) != 0U);
    assert(BNO085_GetAcceleration(&value) == BNO085_OK);
    assert(close_to(value.x_mps2, 1.0f));
    assert(close_to(value.y_mps2, -2.0f));
    assert(close_to(value.z_mps2, 0.5f));
    assert(value.timestamp_us == 990500U);

    /* status bits 4:2 contain exponent 2: delay = 5 * 2^2 * 100 us. */
    payload[6] = 1U;
    payload[7] = 0x08U;
    assert(BNO085_Test_ParseSensorPayload(payload, sizeof(payload), 1000000U,
                                          &events) == BNO085_OK);
    assert(BNO085_GetAcceleration(&value) == BNO085_OK);
    assert(value.timestamp_us == 992000U);
}

static void test_extended_reports_and_gap_counter(void)
{
    uint8_t payload[] = {
        0x04U, 0U, 0U, 0U, 0x00U, 0x01U, 0U, 0U, 0U, 0U,
        0x06U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0x00U, 0x01U,
        0x07U, 0U, 0U, 0U, 0x00U, 0x02U, 0U, 0U, 0U, 0U,
               0x00U, 0x01U, 0U, 0U, 0U, 0U,
        0x0FU, 0U, 0U, 0U, 0x10U, 0U, 0U, 0U, 0U, 0U,
               0x08U, 0U, 0U, 0U, 0U, 0U,
        /* Sequence 2 is deliberately absent: the gap counter must advance. */
        0x01U, 3U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
    };
    uint32_t events = 0U;
    BNO085_LinearAcceleration_t linear;
    BNO085_Gravity_t gravity;
    BNO085_UncalibratedGyroscope_t gyro;
    BNO085_UncalibratedMagnetometer_t mag;

    assert(BNO085_Test_ParseSensorPayload(payload, sizeof(payload), 2000000U,
                                          &events) == BNO085_OK);
    assert(BNO085_GetLinearAcceleration(&linear) == BNO085_OK);
    assert(BNO085_GetGravity(&gravity) == BNO085_OK);
    assert(BNO085_GetUncalibratedGyroscope(&gyro) == BNO085_OK);
    assert(BNO085_GetUncalibratedMagnetometer(&mag) == BNO085_OK);
    assert(close_to(linear.x_mps2, 1.0f));
    assert(close_to(gravity.z_mps2, 1.0f));
    assert(close_to(gyro.x_rps, 1.0f));
    assert(close_to(gyro.bias_x_rps, 0.5f));
    assert(close_to(mag.x_uT, 1.0f));
    assert(close_to(mag.bias_x_uT, 0.5f));
    {
        BNO085_Diagnostics_t diagnostics;
        assert(BNO085_GetDiagnostics(&diagnostics) == BNO085_OK);
        assert(diagnostics.sensor_sequence_gaps == 1U);
    }
}

static void test_transport_recovery_fault_injection(void)
{
    BNO085_Diagnostics_t before;
    BNO085_Diagnostics_t after;

    assert(BNO085_GetDiagnostics(&before) == BNO085_OK);

    BNO085_Test_SetTransportActive();
    fake_recover_ok = true;
    assert(BNO085_RecoverTransport() == BNO085_OK);
    assert(BNO085_GetDiagnostics(&after) == BNO085_OK);
    assert(after.transport_recoveries == before.transport_recoveries + 1U);

    fake_recover_ok = false;
    fake_port_error = BNO085_PORT_ERROR_DMA;
    fake_raw_error = 0x1234U;
    assert(BNO085_RecoverTransport() == BNO085_ERR_PORT);
    assert(BNO085_GetDiagnostics(&after) == BNO085_OK);
    assert(after.port_errors == before.port_errors + 1U);
    assert(after.port_raw_error == 0x1234U);
    fake_recover_ok = true;
}

static void test_motion_reports_and_runtime_config(void)
{
    uint8_t payload[] = {
        0x10U, 1U, 2U, 0U, BNO085_TAP_Z | BNO085_TAP_DOUBLE,
        0x11U, 1U, 1U, 0U, 0x40U, 0x42U, 0x0FU, 0U,
               0x2AU, 0U, 0U, 0U,
        0x13U, 1U, 3U, 0U, BNO085_STABILITY_STATIONARY, 0U,
        0x17U, 1U, 0U, 0U, 0x20U, 0xA1U, 0x07U, 0U
    };
    uint32_t events = 0U;
    BNO085_Tap_t tap;
    BNO085_StepCounter_t counter;
    BNO085_StepDetector_t detector;
    BNO085_Stability_t stability;
    BNO085_Config_t config;

    assert(BNO085_Test_ParseSensorPayload(payload, sizeof(payload), 3000000U,
                                          &events) == BNO085_OK);
    assert((events & (BNO085_EVENT_TAP | BNO085_EVENT_STEP_COUNTER |
                      BNO085_EVENT_STEP_DETECTOR |
                      BNO085_EVENT_STABILITY)) ==
                     (BNO085_EVENT_TAP | BNO085_EVENT_STEP_COUNTER |
                      BNO085_EVENT_STEP_DETECTOR |
                      BNO085_EVENT_STABILITY));
    assert(BNO085_GetTap(&tap) == BNO085_OK);
    assert(BNO085_GetStepCounter(&counter) == BNO085_OK);
    assert(BNO085_GetStepDetector(&detector) == BNO085_OK);
    assert(BNO085_GetStability(&stability) == BNO085_OK);
    assert(tap.flags == (BNO085_TAP_Z | BNO085_TAP_DOUBLE));
    assert(counter.latency_us == 1000000U);
    assert(counter.steps == 42U);
    assert(detector.latency_us == 500000U);
    assert(stability.classification == BNO085_STABILITY_STATIONARY);

    BNO085_GetDefaultConfig(&config);
    assert(config.feature_retry_count == 2U);
    assert(BNO085_SetConfig(&config) == BNO085_OK);
    config.spi_timeout_ms = 0U;
    assert(BNO085_SetConfig(&config) == BNO085_ERR_BAD_PARAM);
}

static void test_truncated_and_unknown_reports(void)
{
    static const uint8_t acceleration[] = {
        0x01U, 7U, 0U, 0U, 1U, 0U, 2U, 0U, 3U, 0U
    };
    uint16_t length;

    for (length = 1U; length < sizeof(acceleration); length++) {
        uint32_t events = 0U;
        assert(BNO085_Test_ParseSensorPayload(acceleration, length, 0U,
                                              &events) ==
               BNO085_ERR_INVALID_REPORT);
        assert(events == 0U);
    }
    {
        uint8_t unknown[] = {0xEEU, 0U, 0U, 0U};
        uint32_t events = 0U;
        assert(BNO085_Test_ParseSensorPayload(unknown, sizeof(unknown), 0U,
                                              &events) ==
               BNO085_ERR_INVALID_REPORT);
    }
}

static void test_timestamp_extreme_does_not_overflow(void)
{
    uint8_t payload[] = {
        /* Base timestamp delta = INT32_MIN. / 基准时间差为 INT32_MIN。 */
        0xFBU, 0U, 0U, 0U, 0x80U,
        /* Rebase by INT32_MAX, then publish one acceleration sample. */
        0xFAU, 0xFFU, 0xFFU, 0xFFU, 0x7FU,
        0x01U, 9U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
    };
    uint32_t events = 0U;

    assert(BNO085_Test_ParseSensorPayload(payload, sizeof(payload), 1U,
                                          &events) == BNO085_OK);
    assert((events & BNO085_EVENT_ACCELEROMETER) != 0U);
}

static uint32_t fuzz_next(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

/** Deterministic malformed-input sweep. Sanitizer CI turns any out-of-bounds
 * access or undefined behavior into a test failure. / 确定性异常输入扫描；
 * Sanitizer 会把越界和未定义行为变成测试失败。 */
static void test_deterministic_fuzz_inputs(void)
{
    uint8_t payload[128];
    uint32_t state = 0xB0855EEDU;
    uint32_t iteration;

    for (iteration = 0U; iteration < 20000U; iteration++) {
        uint16_t length = (uint16_t)(fuzz_next(&state) % sizeof(payload));
        uint16_t index;
        uint32_t events = 0U;
        BNO085_Status_t status;

        for (index = 0U; index < length; index++) {
            payload[index] = (uint8_t)fuzz_next(&state);
        }
        status = BNO085_Test_ParseSensorPayload(payload, length,
                                                fuzz_next(&state), &events);
        assert((status == BNO085_OK) ||
               (status == BNO085_ERR_INVALID_REPORT));
        assert((events & ~(BNO085_EVENT_ROTATION_VECTOR |
                           BNO085_EVENT_ACCELEROMETER |
                           BNO085_EVENT_GYROSCOPE |
                           BNO085_EVENT_MAGNETOMETER |
                           BNO085_EVENT_GAME_ROTATION_VECTOR |
                           BNO085_EVENT_LINEAR_ACCELERATION |
                           BNO085_EVENT_GRAVITY |
                           BNO085_EVENT_GYROSCOPE_UNCAL |
                           BNO085_EVENT_MAGNETOMETER_UNCAL |
                           BNO085_EVENT_RAW_ACCELEROMETER |
                           BNO085_EVENT_RAW_GYROSCOPE |
                           BNO085_EVENT_RAW_MAGNETOMETER |
                           BNO085_EVENT_TAP |
                           BNO085_EVENT_STEP_COUNTER |
                           BNO085_EVENT_STEP_DETECTOR |
                           BNO085_EVENT_STABILITY)) == 0U);
    }
}

int main(void)
{
    BNO085_Test_ResetState();
    test_timestamp_and_acceleration();
    test_extended_reports_and_gap_counter();
    test_transport_recovery_fault_injection();
    test_motion_reports_and_runtime_config();
    test_truncated_and_unknown_reports();
    test_timestamp_extreme_does_not_overflow();
    test_deterministic_fuzz_inputs();
    puts("bno085 parser tests: PASS");
    return 0;
}
