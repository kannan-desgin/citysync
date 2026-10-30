# CitySync RT+ – Data Acquisition Flowchart

The flowchart illustrates the sensor acquisition threads, message preparation, QNX message passing, TwinSync validation, and CLI output.

The PNG version of this flowchart is included in the downloadable project ZIP at `docs/images/data-acquisition-flowchart.png`. It is not yet committed to this GitHub repository.

## Flow Summary

1. Initialize GPIO and the PCF8591 ADC.
2. Configure sensor pins and connect to TwinSync.
3. Initialize the ADC mutex and create five QNX sensor threads.
4. Read flame, ultrasonic, ADC current-simulation, IR, and temperature inputs.
5. Prepare sensor messages with sequence numbers and timestamps.
6. Send messages to TwinSync using QNX Native Message Passing.
7. TwinSync receives and validates each message.
8. Valid sensor data is updated and displayed through the CLI; invalid messages are rejected.
