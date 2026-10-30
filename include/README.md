# Project Header Dependencies

- pcf8591_adc.h — PCF8591 ADC declarations and I2C address.
- rpi_gpio_qnx.h — Raspberry Pi GPIO, ultrasonic, and servo declarations.
- citysync_types.h — Shared subsystem IDs, message structures, freshness/status values, sample intervals, deadline/stale thresholds, latency/report settings, and QNX scheduling priorities.

Shared acquisition intervals: Fire 100 ms, Current 200 ms, Water 300 ms, Traffic 500 ms, Temperature 1000 ms. TwinSync requests FIFO priority 63. Its timer pulse delivery priority is 10; that pulse is not a separate application thread.
