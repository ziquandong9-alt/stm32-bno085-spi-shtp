/* SPDX-License-Identifier: BSD-3-Clause */
/* Host-side parser regression tests. / 在 PC 上运行的解析器回归测试。 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "bno085_port.h"

/* Include the implementation so this test can exercise private, pure parser
 * helpers without exposing test-only symbols in the public API.
 * 直接纳入实现文件，以测试私有纯解析函数而不污染公共 API。 */
#include "../BNO085_Driver/Src/bno085.c"

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

    assert(parse_sensor_payload(payload, sizeof(payload), 1000000U,
                                &events) == BNO085_OK);
    assert((events & BNO085_EVENT_ACCELEROMETER) != 0U);
    assert(BNO085_GetAcceleration(&value) == BNO085_OK);
    assert(close_to(value.x_mps2, 1.0f));
    assert(close_to(value.y_mps2, -2.0f));
    assert(close_to(value.z_mps2, 0.5f));
    assert(value.timestamp_us == 990500U);
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
        0x01U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U
    };
    uint32_t events = 0U;
    BNO085_LinearAcceleration_t linear;
    BNO085_Gravity_t gravity;
    BNO085_UncalibratedGyroscope_t gyro;
    BNO085_UncalibratedMagnetometer_t mag;

    assert(parse_sensor_payload(payload, sizeof(payload), 2000000U,
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
    assert(diagnostics.sensor_sequence_gaps == 1U);
}

static void test_transport_recovery_fault_injection(void)
{
    uint32_t recoveries = diagnostics.transport_recoveries;
    uint32_t errors = diagnostics.port_errors;

    initialized = true;
    async_rx_state = ASYNC_RX_CARGO;
    fake_recover_ok = true;
    assert(BNO085_RecoverTransport() == BNO085_OK);
    assert(async_rx_state == ASYNC_RX_IDLE);
    assert(diagnostics.transport_recoveries == recoveries + 1U);

    fake_recover_ok = false;
    fake_port_error = BNO085_PORT_ERROR_DMA;
    fake_raw_error = 0x1234U;
    assert(BNO085_RecoverTransport() == BNO085_ERR_PORT);
    assert(diagnostics.port_errors == errors + 1U);
    assert(diagnostics.port_raw_error == 0x1234U);
    fake_recover_ok = true;
}

int main(void)
{
    memset(&diagnostics, 0, sizeof(diagnostics));
    memset(sensor_sequence_valid, 0, sizeof(sensor_sequence_valid));
    test_timestamp_and_acceleration();
    test_extended_reports_and_gap_counter();
    test_transport_recovery_fault_injection();
    puts("bno085 parser tests: PASS");
    return 0;
}
