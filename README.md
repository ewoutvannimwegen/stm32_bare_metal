#Bare metal STM32

Develop code for STM32 boards with a very minimal code base.
I've added some of my own examples for the L476RG (cortex m4) board. These examples are 100% compatible with all platforms. Just install the following tools for your system.

## <b>Tools</b>

- [arm compiler & debugger](https://developer.arm.com/tools-and-software/embedded/arm-compiler)
- make or [make for windows](http://gnuwin32.sourceforge.net/packages/make.htm)
- [openocd](http://openocd.org/)
- [st link](https://www.st.com/en/development-tools/st-link-v2.html#tools-software)

Debian based systems

```console
apt-get install build-essential gcc-arm-none-eabi openocd
```

## <b>Text Editor</b>

To edit these files I recommend these two editors with some plugins.

##### [visual studio code](https://code.visualstudio.com/)  
* Language server: [C/C++](https://code.visualstudio.com/docs/languages/cpp) 
* Debug [cortex debug](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug)

##### [neovim nightly](https://github.com/neovim/neovim) 
* Language server client: [nvim-lspconfig](https://github.com/neovim/nvim-lspconfig) 
* Language server: [clangd](https://clangd.llvm.org/installation.html) 
* compile_commands generator: [bear](https://github.com/rizotto/Bear)

## <b>Sources</b>

The CMSIS core files for the Cortex-M4 where used, to access the register 
locations of the L476RG board in a sufficient way.

- [cmsis-core cortex-m](https://arm-software.github.io/CMSIS_5/Core/html/index.html)
