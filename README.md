# fullCURRENT Smarts

This is a project with code for the fullCURRENT motor controller.

It was based off a demo project of FreeRTOS running on a STM32F4 Discovery board located here: [website](https://github.com/wangyeee/STM32F4-FreeRTOS)


### Initializing the submodule(s)

This project uses submodules for common libraries like D1K.  When you first clone the repo, you have to initialize them:

```bash
git submodule update --init --recursive
```


### Install the toolchain

The pre-built version of GNU Tools for ARM can be downloaded from its [website](https://launchpad.net/gcc-arm-embedded).

The makefile assumes it is installed in /usr/local/lib/gcc-arm-none-eabi-4_9-2015q3

Also note, you might need to install a 32 bit library if you get weird errors when trying to run this version of GCC.


### Making
This project is makefile driven.  You can make the project by simply typing `make`.

Other commands include:

clean: `make clean`
program with JLink: `make jlink`
clean and build: `make cb`
clean, build, and flash: `make cbf`
build and flash: `make bf`


### Programming
I have been using a Segger J-Link to program.  To do this, you need to install their "software and documentation pack" from [here](https://www.segger.com/jlink-software.html).




## Archive

### ST Link Use
Clone this [git](https://github.com/texane/stlink), follow the instructions on that page and install st-util to your system.

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
