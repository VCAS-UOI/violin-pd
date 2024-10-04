# Violin VLC receive firmware

The firmware code for the Violin VLC rcv board is kept in this directory.

## Credits

The VIOLIN project has been co-financed by the European Union and Greek national
funds through the Operational Program Competitiveness, Entrepreneurship and
Innovation, under the call RESEARCH-CREATE-INNOVATE (project code: T1EDK-02419).

The COBS encoder is from the corresponding [Wikipedia article](https://en.wikipe
dia.org/wiki/Consistent_Overhead_Byte_Stuffing).

The firmware uses FreeRTOS and the STM32 USB Device Library which are bundled with the [STM32Cube FW_H7 library](https://github.com/STMicroelectronics/STM32CubeH7).

The firmware depends on the STM32 HAL library (see license files under `Drivers`), FreeRTOS (see license in 'Middlewares/Third_Party/FreeRTOS/Source'), and the STM32 USB Device library (see license in `Middlewares/ST/STM32_USB_Device_Library').

## Code generation

This project was tested with STM32CubeMX 6.12.0, STM32Cube FW_H7 V1.11.2, [STM32 VS Code Extension v2.1.0](https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension), STM32CubeCLT v1.16.

Generate initialization code by running STM32cubeMX on `vlcRcv.ioc`.

The following modifications additions from the generated code are required:

- Modify `cmake/gcc-arm-none-eabi.cmake` to use the [linker script](./STM32H743ZITx_FLASH_mod_sramSections.ld)
- Modify `cmake/stm32cubemx/CMakeLists.txt` to use the [modified assembly startup file](./startup_stm32h743xx_sramSections.s).
Note that this file is overwriten by cubeMX when re-generating code!

- Modify `Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c`: before the `USBD_CDC_DeviceQualifierDesc` and `USBD_CDC_CfgDesc`, add the macro `CFG_USB_MEM_SECTION` which instructs the linker to place them in a memory section which can be used for DMA access.

- Modify `Inc/usbd_conf.h` to add the memory section definition used in the above and other files (see below).
  Look under the  USER CODE BEGIN INCLUDE comment.

- Modify `Src/usbd_cdc_if.c` at numerous places.
  All of the changes are in USER CODE parts, so safe from cubeMX regeneration.

- Modify `Src/usbd_conf.c`.
  These changes **are not** in `USER CODE` parts, thus are removed when re-generating code from cubeMX.
  1. Add `CFG_USB_MEM_SECTION` to `hpcd_USB_OTG_FS`.
  2. Add `CFG_USB_MEM_SECTION uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef)/4)+1];/* On 32-bit boundary */` just above the `USBD_static_malloc` function.
  3. Comment out the `static uint32_t mem` line in the `USBD_static_malloc` function.

- Modify `Src/usbd_desc.c` to add `CFG_USB_MEM_SECTION` to multiple variables. 
  These changes **are not** in USER CODE parts, thus are removed when re-generating code from cubeMX.
  1. `FS_Desc`. This also needs alignment markers: `CFG_USB_MEM_SECTION __ALIGN_BEGIN USBD_DescriptorsTypeDef FS_Desc __ALIGN_END =`
  2. `USBD_FS_DeviceDesc`
  3. `USBD_FS_BOSDesc`
  4. `USBD_LangIDDesc`
  5. `USBD_StrDesc`
  6. `USBD_StringSerial`

- Modify `Src/usb_device.c` to add `CFG_USB_MEM_SECTION` for `hUsbDeviceFS`.

- Modify `main.c`. It needs a number of changes.
  - global variables.
  - in `MX_TIM2_Init` the setting of the filter. - this is not in a USER CODE part, so must be re-written after cubeMX code regeneration.
  - move the USB initialization out of the FreeRTOS task code and into `main()` as the freeRTOS task is overiden by our code in `tasks.c`
  - `HAL_TIM` callback functions inside the USER CODE 4 part.

- Add the new files: `Src/tasks.c`, `Src/demodulate.c`, `Src/printf.c`, `Src/FreeRTOS-openocd.c`, `Inc/demodulate.h`, `Inc/printf.h`.

- Add the new source files to `CMakeLists.txt` under `target_sources` with a  `../../Src/` prefix and add `DATA_IN_D2_SRAM` under `target_compile_definitions`

## Nucleo board connections

### Power supply

The board can be supplied via the STLink USB connector. By setting the power selection jumper, other power sources can be used. Refer to the board manual for details.

### USB/Serial connection

The board communicates with the PC-host via the second USB connecteor of the board, the one located next to the ethernet connector.

Although theoretically the board can sink power through this connector, it is not advisable. I once destroyed some of the MCUs GPIO pins doing that.

### Connection to the photodiode analog front end.

The photodiode analog front end circuit amplifies the optical signal to digital levels so that a '1' is nearly 3.3V and a '0' is nearly 0V. 
We therefore don't need to use an ADC to aquire the signal.
We use the input capture feature of a timer peripheral instead.

Note that the MCU pin is 5V resistant, but cannot be driven under 0V.
Our analog circuit produces an output clipped to 0 - 3.3V.

The pin used is PA15 (`TIM2_CH1`).

For testing without a VLC link you can connect the xmit board's output pin to the rcv board's input pin directly.

## Firmware description

### Timer input capture 

The analog front end provides a digital [Manchester-coded](https://en.wikipedia.org/wiki/Manchester_code) signal, which is captured using the input-capture capability of an STM32 timer.

We use a 32-bit timer (`TIM2`) so that we can capture signals of a wide range of frequencies.
When an edge is detected in the input signal, the current counter value is written to a memory buffer using DMA and the counter is reset.
The DMA is set to circular mode (wrapping to the start when the buffer is full) and it interrupts the core when each half of the buffer has been filled.
The HAL timer interrupt handler calls our callback functions (`HAL_TIM_IC_Capture[HalfCplt]Callback`), which notify (using FreeRTOS task notification) the demodulation task (`task_dmaReceive`) that a half buffer is filled.
The task must be fast enough to process this half buffer, before the DMA has filled in the other half!

### Brief task description

Two concurent tasks are used, `task_dmaReceive` and `task_usbXmit`, which communicate using two queues: `freeQ`, `flitQ`.
These queues handle memory buffers for flit storage. 
Initally all buffers are in the `freeQ`.
As flits get received, they are placed in the `flitQ` and after they get sent to the host via USB, the buffer is returned to the `freeQ`.

A third queue, `usbCmdQ`, is used to pass incoming 'commands' from the host, which instruct the rcv board to start or stop receiving VLC frames.
A callback function pseudotask_usbRcv, which is called by the USB stack when an incoming USB message is received from the host, writes the start/stop commands to the `usbCmdQ` from where they are picked up by `task_dmaReceive`.

After `task_dmaReceive` reads a start command from `usbCmdQ`, it starts the timer input capture in DMA mode. 
After a half buffer is complete, the task calculates the transmition frequency from the widths of the input signal's pulses. This information can then be used to demodulate the incoming signal.
The task then enters a loop where it waits to be notified for another half buffer of data from the timer. 
It demodulates the data and checks if a complete VLC frame has been received.
It also checks if a stop command has been received from the host via USB, in which case it stops the timer and goes back to an outer loop waiting for a start command.
When a complete VLC frame is received, its size and other information are passed to the `flitQ` from where it will be picked up by `task_usbXmit`.
Then a free buffer from `freeQ` is used to demodulate and store the remaining timer data.

`task_usbXmit` waits for a new item on `flitQ`.
It transmits it to the host via USB and wait for a task notification from a USB callback function which signals the completion of the transmission to the host.
It then places the flit buffer to `freeQ` and loops back to wait for the next flit.


