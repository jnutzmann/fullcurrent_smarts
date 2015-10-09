# fullCURRENT

This is a project with code for the fullCURRENT motor controller.  It was based off a demo project of FreeRTOS running on a STM32F4 Discovery board located here: [website](https://github.com/wangyeee/STM32F4-FreeRTOS)

### Install the toolchain

The pre-built version of GNU Tools for ARM can be downloaded from its [website](https://launchpad.net/gcc-arm-embedded). It's available for most systems. Follow the instructions in the readme file and installed the toolchain to your system. To verify your installation, simply type `arm-none-eabi-gcc --version` in your terminal, if everything goes right, you'll get output like this:

### ST Link Use
Clone this [git](https://github.com/texane/stlink), follow the instructions on that page and install st-util to your system.

### Compile this example
The only thing you need to do is to edit the makefile and let it know your toolchain installation path. Change the `TOOLCHARN_ROOT` variable at the third line of makefile and point it to where you installed the toolchain. The you can simply type `make` and compile the example.

### Debug
Connect your STM32F4Discovery with a USB cable. You can flash the binary into the board with this:

`$ st-flash write binary/FreeRTOS.bin 0x8000000`

The code is wrote directly into internal flash of STM32 processor and it starts to run after reset. To debug it, first start the GDB server:

`$ st-util &`

And then GDB:

```
$ arm-none-eabi-gdb binary/FreeRTOS.elf
(gdb) tar ext :4242
(gdb) b main
(gdb) c
```

You'll get breakpoint triggered at `main` function, and enjoy!
# fullcurrent_smarts
