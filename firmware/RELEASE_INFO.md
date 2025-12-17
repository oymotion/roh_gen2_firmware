# Firmware Versions

| Version  | Date       | Comments                                                              |
| -------- | ---------- | --------------------------------------------------------------------- |
| V3.1.58  | 2025-06-20 | Initial release                                                       |
| V3.1.79  | 2025-07-03 | Fixed bug: Wrong force data length                                    |
|          |            | Added: Read and write manufacturer information                        |
| V3.1.111 | 2025-09-01 | Fixed bug: Incorrect force sensor data quantity under serial protocol |
|          |            | Added: Support for multiple sensors(default: Dot matrix sensor)       |
|          |            | Adjusted: Thumb calibration range                                     |
| V3.1.151 | 2025-12-17 | Added: Read/write manufacturer data.                                  |
|          |            | Added: Read/write speed control parameters.                           |
|          |            | Added: Read/write stall protection parameters.                        |
|          |            | Reduced: Data transmission latency for CAN communication.             |
|          |            | Adjusted: Valid range for ID setting to 0-247.                        |
