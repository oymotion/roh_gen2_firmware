# ROH Firmware

Firmware, protocol and desktop application for Robotic OHand，i.e.，ROH.

## Directories & Files

- *[FAQs](FAQs)*: frequently asked questions
  - [FAQs.md](FAQs/FAQs_EN.md): FAQs in English
- *[firmware](firmware)*: firmware update files
  - files in subdirectory [ModBus-RTU](firmware/ModBus-RTU) are ModBus RTU versions of ROH-AP001.
  - files in subdirectory [SerialCtrl](firmware/SerialCtrl) are SerialCtrl versions of ROH-AP001, can process commands at 90Hz+.
  - files in subdirectory [ModBus-RTU+SerialCtrl](firmware/ModBus-RTU+SerialCtrl) are for mixed version of ROH-AP001, recommended.
  - files in subdirectory [Can](firmware/Can) are Can versions of ROH-AP001.
  - files in subdirectory [Realman_Plus_Protocol](firmware/Realman_Plus_Protocol) are for Realman plus protocol of ROH-AP001. After the upgrade, the Realman debugging tool [Debugging Tool](https://gitee.com/RealManRobot/rm_arm_plus_protocol/tree/main) must be used
  - [RELEASE_INFO.md](firmware/RELEASE_INFO.md): release information for each firmware version.
- *[OHandSetting](OHandSetting)*: desktop application for Robotic OHand (also for Prosthetic OHand, so it's name is "OHandSetting"). Windows and Ubuntu system are supported currently.
  - [OHandSetting-Instruction-Manual-V1.x.pdf](OHandSetting/OHandSetting-Instruction-Manual-V1.6.pdf): User manual for desktop application.
- *[protocol](protocol)*: protocol specification for ModBus-RTU and SerialCtrl versions
  - [OHandModBusRTUProtocol_EN.md](protocol/OHandModBusRTUProtocol_EN.md): ModBus-RTU protocol specification.
  - [OHandSerialProtocol_EN.md](protocol/OHandSerialProtocol_EN.md): SerialCtrl and Can protocol specification.
  - [roh_registers_v2.h](protocol/roh_registers_v2.h)/[roh_registers_v2.py](protocol/roh_registers_v2.py): registers definitions for C, C++ & Python.
- *[res](res)*: resource file for markdown files
- *[UserManual](UserManual)*: user manual for ROH
  - [ROH-AP001-Dexterous-Hand-V1.x.x.pdf](UserManual/ROH-AP001-Dexterous-Hand-V1.0.6.pdf): ROH-AP001 User manual
  - [ROH-AP002-Dexterous-Hand-V1.x.x.pdf](UserManual/ROH-AP002-Dexterous-Hand-V1.0.1.pdf): ROH-AP002 User manual

## Firmware Update

1. Power off the Robotic OHand.
2. Connect USB-485 converter to Robotic OHand and computer.
3. Found your COM port in system device manager
4. Open OHandSetting, select your right COM port then click "Menu"-"File"-"Force Update"
5. Choose local source and firmware update file in directory "firmware".
6. Follow the instructions.
7. Done.
