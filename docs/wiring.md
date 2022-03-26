## 2022 AVC Car Wiring
Based on [the 2020 code](https://github.com/sdsmt-robotics/nrc-avc-2020/tree/master/driveCode).

Pin Name | Location | Comment
--- | --- | ---
Right Servo PWM | Arduino D9
Left Servo PWM | Arduino D10
ESC PWM | Arduino D11
ESC Diagnostic (single white wire on 3-pin header) | Unused
Encoder Signal | Arduino D5 | Power encoder with 3.3V, signal wire is the output of the LM393 comparator module - not direct from the hall effect sensor
LED Strip Red (Yellow Wire)| Arduino D8 | Doesn't go straight to Arduino, runs through a discrete transistor
LED Strip Green (Green Wire)| Arduino D7 | Doesn't go straight to Arduino, runs through a discrete transistor
LED Strip Blue (Blue Wire) | Arduino D6 | Doesn't go straight to Arduino, runs through a discrete transistor
GPS RX | Arduino D2 | Power GPS with 3.3V
GPS TX | Arduino D3 | Power GPS with 3.3V
