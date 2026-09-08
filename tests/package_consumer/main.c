#include "bno085.h"
#include "bno085_port_callbacks.h"

int main(void)
{
    BNO085_Config_t config;
    BNO085_CallbackPortConfig_t port = {0};
    BNO085_GetDefaultConfig(&config);
    return (config.spi_timeout_ms != 0U && port.context == 0) ? 0 : 1;
}
