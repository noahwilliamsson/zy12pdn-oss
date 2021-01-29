# Overview

This is an attempt to take advantage of the FUSB302 chip on the YZStudio ZY12PDN USB-PD Trigger to explore USB-PD VDMs (Vendor Defined Message) on Apple devices.  See [Asahi Linux docs on USB-PD](https://github.com/AsahiLinux/docs/wiki/HW:USB-PD).

- connect the SWD 3.3V pad to pin 1 (3.3V) on the Raspberry Pi
- connect the SWD GND pad to pin 6 (GND) on the Raspberry Pi
- program the STM32F0 via the SWD pads on the back of the ZY12PDN board
- solder wires onto the STM32F0 chip's PA2 (USART1_TX) and PA3 (USART1_RX) pins (3.3V I/O) to gain access to the UART (the screw terminal must be desoldered first)
- connect the PA2 (USART1_TX and PA3 (USART1_RX) wires to the Raspberry Pi's pin 10 (GPIO 15) and pin 8 (GPIO 14) respectively (see https://pinout.xyz)
- use an USB Type-C breakout board to gain access to the D+/D- or SBU1/SBU2 signals since they're not accessible on the ZY12PDN

Quickstart:

    sudo apt install -y openocd python3 python3-serial
    vi platformio.ini    # optionally update upload_protocol for your SWD programmer
    pio run
    pio run -t upload
    miniterm --raw /dev/ttyS0 115200

NOTE: [platformio.ini](platformio.ini) is currently setup for a J-Link EDU Mini SWD programmer.

Sample output:

    ZY12PDN OSS
    Saved mode: 0
    FUSB302B__X A_revB
    USB PD comm
    Event: protocol_changed
    RX: data_source_capabilities
    Event: source_caps_changed
    RX: ctrl_accept
    Event: power_accepted
    RX: ctrl_ps_ready
    Event: power_ready
    Voltage: 5000
    RX: data_source_capabilities
    Event: source_caps_changed
    RX: ctrl_accept
    Event: power_accepted
    RX: ctrl_ps_ready
    Event: power_ready
    Voltage: 5000
    RX: data_vendor_defined, objs: 0x01
    RX:   data: 0xff008001
    RX:   discover identity

    > help
    Usage:
      sop <sop|sop1|sop2|debug1|debug2> (send with given SOP)
    Apple VDM commands:
      aalist                 (send 0x10 Get Actions List)
      aainfo <action>        (send 0x11 Get Action Info)
      aaexec <action> [arg]  (send 0x12 Perform Action)
      aareboot               (reboot via 0x12,0x105)

    > aalist
    TX: Apple VDM 0x05ac8010
    RX: data_vendor_defined, objs: 0x04
    RX:   data: 0x05ac8050
    RX:   data: 0x01050303 [0x0105 0x0303]
    RX:   data: 0x08030809 [0x0803 0x0809]
    RX:   data: 0x01030000 [0x0103 0x0000]

    > aainfo 105
    TX: Apple VDM 0x05ac8011
    TX:   action: 0x0105
    RX: data_vendor_defined, objs: 0x02
    RX:   data: 0x05ac8051
    RX:   data: 0x80000000 [0x8000 0x0000]


Below is the original README from https://github.com/manuelbl/zy12pdn-oss (on which this code is based).


# Open-Source Firmware for ZY12PDN USB-PD

Open-source firmware for USB Power Delivery trigger board based on an FUSB302B power delivery controller and a STM32F030F4 MCU.

![ZY12PDN board](doc/board.jpg)


## Building

- Clone the project from GitHub
- Open it with Visual Studio Code
- Install the PlatformIO extension
- Click the build icon in the status bar


## Upload

The ZY12PDN board has a 4-pin SWD pads at the bottom. Either solder wires to them or use a 4-pin adapter with pogo pins.

![SWD](doc/swd.jpg)

Connect the SWD pads with an ST-Link, J-Link or Black Magic Probe to your computer and click the upload icon in the status bar of Visual Studio Code.

In most cases, you will need to try twice since the board does not enable the SWD pins quickly enough.


## Hardware

See [Hardware](doc/hardware.md) for a detailled description of the board and its components (incl. schematic).


## Usage Instructions

The user interface – if it can be called so – is similar to the original ZY12PDN. The board can be configured to work in one of several modes:

| Color  | Voltage | Mode |
| :----- | :-- | :---- |
| Red    | 5V  | By pressing the button, the board switches between available voltages. |
| Yellow | 9V  | 9V if available, 5V otherwise. |
| Green  | 12V | 12V if available, 5V otherwise. Note: Few  power supllies support 12V. |
| Cyan   | 15V | 15V if availabke, 5V otherwise. |
| Blue   | 20V | 20V if available, 5V otherwise. |
| Purple | –   | The maximum voltage is selected. |


Except when configuring the board, the LED color indicates the voltage. If the board has been configured for a fixed voltage and the voltage is not available, the LED will slowly flash in red.

### Configuring the Board

The configuration mode is entered by plugging in the board while pressing the button. The LED will flash quickly in cyan until the button is released. The mode can be selected by pressing the button. To save the configuration, the button is pressed for a longer period until the LED goes off. During configuration, the output voltage remains at 5V.


## Supported PD Messages

 - *Capabilities*: The source announces the supported voltages. The sink must immediately request one of the voltages.
 - *Request*: The sink requests a specific voltage and maximum current.
 - *Accept*: The source confirms the requested voltage. The new voltage is not ready yet.
 - *Reject*: The source rejects the requested voltage.
 - *PS_RDY*: The source indicates that the request voltage has been applied.


## Notes

- If the event type of the `pd_sink` callback is `callback_event::source_caps_changed`, `request_power()` must be called to request a voltage -- even if it is 5V. Otherwise the source is likely to reset.
- The firmware is currently limited to the fixed voltages. Additionally capabilities (variable voltages etc.) can be easily added.
- Using the build flag `-D PD_DEBUG`, debugging output can be enabled. In order to see it, you have to solder a wire to PA2 (USART2 TX pin) and connect it to a serial adapter. The baud rate is 115,200 bps.
- All the code is very timing sensitive. Be very careful with debugging output in the `source_caps_changed` callback. It the debugging output takes too long, the USB power supply will likely reset and even cut the power.


## Firmware Mode

The SWDIO line is shared with the interrupt line of the USB PD controller. Therefore, uploading firmware is tricky.

The firmware needs to decide if a debugger is connected or not:

- If a debugger is connected, the FUSB302B is turned off so it releases the SWDIO pin and the SWD has to be enabled.
- If no debugger is connected, the FUSB302B is configured, and it uses the SWDIO to signal interrupts.

Currently, SWCLK is initially configured as input with an external interrupt. If activity is detected, FSUSB302B is shut down and the SWCLK and SWDIO are restored for SWD operation. This has two disadvantages: it can be inadvertently triggered by touching the SWCLK pad on the bottom of the board, and swtich to firmware mode is not always fast enough so uploading firmware takes two attemps.

If you know of a better approach to detect a debugger, let me know.

SWD can be used to upload new firmware. But debugging is not possible as – in normal operation – the SWDIO pin is used as the interrupt pin.


## Acknowledgements

Thanks to the people that have also analzed the ZY12PDN board and contributed to this work:

- Alex Whittemore: [Notes on USB PD Triggers (and ZY12PDN Instructions)](https://www.alexwhittemore.com/notes-on-usb-pd-triggers-and-zy12pdn-instructions/) and [ZY12PDN Reverse Engineering Part 1](https://www.alexwhittemore.com/zy12pdn-reverse-engineering-part-1/).
- Brian Lough: [Powering your projects uing USB-C Power Delivery (YouTube)](https://www.youtube.com/watch?v=iumAnPiQSj8)
- *OxPeter* and *MarkOlsson* on further people on [Brian Lough's Discord Channel](https://discord.gg/nnezpvq)
