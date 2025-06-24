# ROH Firmware

Firmware, protocol and desktop application for Robotic OHand，i.e.，ROH.

## Directories & Files

- *[FAQs](FAQs)*: frequently asked questions
  - [FAQs.md](FAQs/FAQs_EN.md): FAQs in English
- *[firmware](firmware)*: firmware update files
  - files in subdirectory [ROH-AP001_ModBus-RTU](firmware/ROH-AP001/ModBus-RTU) and [ROH-LiteS001_ModBus-RTU](firmware/ROH-LiteS001/ModBus-RTU) are ModBus RTU versions of ROH-AP001 and ROH-LiteS001, respectively.
  - files in subdirectory [ROH-AP001_SerialCtrl](firmware/ROH-AP001/SerialCtrl) and [ROH-LiteS001_SerialCtrl](firmware/ROH-LiteS001/SerialCtrl) are SerialCtrl versions of ROH-AP001 and ROH-LiteS001, respectively, can process commands at 90Hz+.
  - files in subdirectory [ROH-AP001_ModBus-RTU+SerialCtrl](firmware/ROH-AP001/ModBus-RTU+SerialCtrl) and [ROH-LiteS001_ModBus-RTU+SerialCtrl](firmware/ROH-AP001/ModBus-RTU+SerialCtrl) are for mixed version, respectively, recommended.
  - files in subdirectory [ROH-AP001_Can](firmware/ROH-AP001/Can) 和 [ROH-LiteS001_Can](firmware/ROH-LiteS001/Can) are Can versions of ROH-AP001 and ROH-LiteS001, respectively.
  - [RELEASE_INFO.md](firmware/RELEASE_INFO.md): release information for each firmware version.
- *[OHandSetting](OHandSetting)*: desktop application for Robotic OHand (also for Prosthetic OHand, so it's name is "OHandSetting"). Windows and Ubuntu system are supported currently.
  - [OHandSetting-User-Manual-V1.x.pdf](OHandSetting/OHandSetting-User-Manual-V1.1.pdf): User manual for desktop application.
- *[protocol](protocol)*: protocol specification for ModBus-RTU and SerialCtrl versions
  - [OHandModBusRTUProtocol_EN.md](protocol/OHandModBusRTUProtocol_EN.md): ModBus-RTU protocol specification.
  - [OHandSerialProtocol_EN.md](protocol/OHandSerialProtocol_EN.md): SerialCtrl and Can protocol specification.
  - [roh_registers_v2.h](protocol/roh_registers_v2.h)/[roh_registers_v2.py](protocol/roh_registers_v2.py): registers definitions for C, C++ & Python.
- *[res](res)*: resource file for markdown files
- *[UserManual](UserManual)*: user manual for ROH
  - [ROH-AP001-Dexterous-Hand-V1.x.x.pdf](UserManual/ROH-AP001-Dexterous-Hand-V1.0.4.pdf): ROH-AP001 User manual
  - [ROH-LiteS001-Dexterous-Hand-V1.x.x.pdf](UserManual/ROH-LiteS001-Dexterous-Hand-V1.0.3.pdf): ROH-LiteS001 User manual

## Firmware Update

1. Power off the Robotic OHand.
2. Connect USB-485 converter to Robotic OHand and computer.
3. Found your COM port in system device manager
4. Open OHandSetting, select your right COM port then click "Menu"-"File"-"Force Update"
5. Choose local source and firmware update file in directory "firmware".
6. Follow the instructions.
7. Done.
