# 裸机编程指南

[![License: MIT](https://img.shields.io/badge/license-MIT-blue)](https://opensource.org/licenses/MIT)
[![Build Status]( https://github.com/cpq/bare-metal-programming-guide/workflows/build/badge.svg)](https://github.com/cpq/bare-metal-programming-guide/actions)

[English](README.md) | [中文](README_zh-CN.md)

本指南是为那些希望用GCC编译器和数据手册而无需其他任何东西就能开始为微控制器（单片机）编程的开发者而写的。本指南中的基础知识可以帮助你更好地理解像STM32Cube、Keil、Arduino和其他框架或IDE是怎么工作的。

本指南涵盖了以下话题：

- 存储和寄存器
- 中断向量表
- 启动代码
- 链接脚本
- 使用`make`进行自动化构建
- GPIO外设和闪烁LED
- SysTick定时器
- UART外设和调试输出
- `printf`重定向到UART
- 用Segger Ozone进行调试
- 系统时钟配置
- 实现一个带设备仪表盘的web服务器

我们将使用[Nucleo-F429ZI](https://www.st.com/en/evaluation-tools/nucleo-f429zi.html)开发板([淘宝购买](https://item.taobao.com/item.htm?spm=a230r.1.14.232.74e4559brlH7oU&id=655793165717&ns=1&abbucket=5#detail))贯穿整个指南的实践，每个章节都有一个相关的完整小项目可以实战。最后一个web服务器项目非常完整，可以作为你自己项目的框架，因此这个示例项目也提供了其他开发板的适配：

- [STM32 Nucleo-F429ZI](step-7-webserver/nucleo-f429zi/)
- [TI EK-TM4C1294XL](step-7-webserver/ek-tm4c1294xl/)
- [树莓派 Pico-W](step-7-webserver/pico-w/)

对其他板子的适配支持还在进行中，可以提交issue来建议适配你正在用的板子。

## 工具配置

为继续进行，需要以下工具：

- ARM GCC, https://launchpad.net/gcc-arm-embedded - 编译和链接
- GNU make, http://www.gnu.org/software/make/ - 构建自动化
- ST link, https://github.com/stlink-org/stlink - 烧写固件

### Mac安装

打开终端，执行：

```sh
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
$ brew install gcc-arm-embedded make stlink
```

### Linux(Ubuntu)安装

打开终端，执行：

```sh
$ sudo apt -y install gcc-arm-none-eabi make stlink-tools
```

### Windows安装

- 下载并安装 [gcc-arm-none-eabi-10.3-2021.10-win32.exe](https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-win32.exe?rev=29bb46cfa0434fbda93abb33c1d480e6&hash=3C58D05EA5D32EF127B9E4D13B3244D26188713C)，安装过程注意勾选"Add path to environment variable"。
- 创建 `C:\tools` 文件夹
- 下载 [stlink-1.7.0-x86_64-w64-mingw32.zip](https://github.com/stlink-org/stlink/releases/download/v1.7.0/stlink-1.7.0-x86_64-w64-mingw32.zip)，解压 `bin/st-flash.exe` 到 `C:\tools`
- 下载 [make-4.4-without-guile-w32-bin.zip](https://sourceforge.net/projects/ezwinports/files/make-4.4-without-guile-w32-bin.zip/download)，解压 `bin/make.exe` 到 `C:\tools`
- 添加 `C:\tools` 到 `Path` 环境变量
- 验证安装：
  - 下载[这个仓库](https://github.com/cpq/bare-metal-programming-guide/archive/refs/heads/main.zip)，解压到 `C:\`
  - 打开命令行，执行：
  <pre style="color: silver;">
  C:\Users\YOURNAME> <b style="color: black;">cd \</b>
  C:\> <b style="color: black;">cd bare-metal-programming-guide-main\step-0-minimal</b>
  C:\bare-metal-programming-guide-main\step-0-minimal> <b style="color: black;">make</b>
  arm-none-eabi-gcc main.c  -W -Wall -Wextra -Werror ...
  </pre>

### 需要的数据手册

- [STM32F429 MCU datasheet](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
- [Nucleo-F429ZI board datasheet](https://www.st.com/resource/en/user_manual/dm00244518-stm32-nucleo144-boards-mb1137-stmicroelectronics.pdf)

## 微控制器介绍

微控制器（microcontroller，uC或MCU）是一个小计算机，典型地包含CPU、RAM、存储固件代码的Flash，以及一些引脚。其中一些引脚为MCU供电，通常被标记为VCC和GND。其他引脚通过高低电压来与MCU通信，最简单的通信方法之一就是把一个LED接在引脚上：LED一端接地，另一端串接一个限流电阻，然后接到MCU信号引脚。在固件代码中设置引脚电压的高低就可以使LED闪烁：

<img src="images/mcu.svg" height="200" />

### 存储和寄存器

MCU的32位地址空间按区分割。例如，一些存储区被映射到特定的地址，这里是MCU的片内flash，固件代码指令在这些存储区读和执行。另一些区是RAM，也被映射到特定的地址，我们可以读或写任意值到RAM区。

从STM32F429数据手册的2.3.1节，我们可以了解到RAM区从地址0x20000000开始，共有192KB。从2.4节我们可以了解到flash被映射到0x08000000，共2MB，所以flash和RAM的位置像这样：

<img src="images/mem.svg" />

从数据手册中我们也可以看到还有很多其它存储区，它们的地址在2.3节"Memory Map"给出，例如："GPIOA"区从地址0x40020000开始，长度为1KB。

这些存储区被关联到MCU芯片内部不同的外设电路上，以特殊的方式控制外设引脚的行为。一个外设存储区是一些32位寄存器的集合，每个寄存器有4字节的空间，在特定的地址，控制着外设的特定功能。通过向寄存器写入值，或者说向特定的地址写一个32位的值，我们就可以控制外设的行为。通过读寄存器的值，我们就可以得到外设的数据或配置。

MCU通常有许多不同的外设，其中比较简单的就是GPIO（General Purpose Input Output，通用输入输出），它允许用户将MCU引脚设为输出模式，然后置“高”或置“低”；或者设置为输入模式，然后读引脚电压的“高”或“低”。还有UART外设，可以使用串行协议通过两个引脚收发数据。还有许多其它外设。

在MCU中，一个相同外设通常会有多个“实例”，比如GPIOA、GPIOB等等，它们控制着MCU引脚的不同集合。类似地，也有UART1、UART2等等，可以实现多通道。在Nucleo-F429上，有多个GPIO和UART外设。

例如，GPIOA外设起始地址为0x40020000，我们可以从数据手册8.4节找到GPIO寄存器的描述，上面说 `GPIOA_MODER` 寄存器偏移为0，意味着它的地址是 `0x40020000 + 0`，寄存器地址格式如下：

<img src="images/moder.png" style="max-width: 100%" />

数据手册显示MODER这个32位寄存器是由16个2位的值组成。因此，一个MODER寄存器控制16个物理引脚，0-1位控制引脚0，2-3位控制引脚1，以此类推。这个2位的值编码了引脚模式：'00'代表输入，'01'代表输出，'10'代表替代功能——在其它部分进行描述，'11'代表模拟引脚。因为这个外设命名为'GPIOA'，所以对应引脚名为'A0'、'A1'，等等。对于外设'GPIOB'，引脚则对应叫'B0'、'B1'，等等。

如果我们向MODER寄存器写入32位的值'0'，就会把从A0到A15这16个引脚设为输入模式：

```c
  * (volatile uint32_t *) (0x40020000 + 0) = 0;  // Set A0-A15 to input mode
```

通过设置独立的位，我们就可以把特定的引脚设为想要的模式。例如，下面的代码将A3设为输出模式：

```c
  * (volatile uint32_t *) (0x40020000 + 0) &= ~(3 << 6);  // CLear bit range 6-7
  * (volatile uint32_t *) (0x40020000 + 0) |= 1 << 6;     // Set bit range 6-7 to 1
```

我来解释下上面的位操作。我们的目标是把控制GPIOA外设引脚3的位，也就是6-7，设为特定值，在这里是1。这个需要2步，首先，我们必须将6-7位的当前值清除，也就是清'0'，因为这两位可能已经有值；然后，我们再将6-7设为期望值。

所以，第一步，我们先把6-7位清'0'，怎么做呢？4步：

- 使一个数有连续的N位'1'
  - 1位用1：  `0b1`
  - 2位用3:   `0b11`
  - 3位用7：  `0b111`
  - 4位用15： `0b1111`
  - 以此类推，对于N位，数值应为 `2^N - 1`。对于2位，数值为 `3`，或者写为二进制 `0b00000000000000000000000000000011`
- 将数字左移位。如果我们需要设置位 X-Y，则将数字左移X位。在我们的例子中，左移6位：`(3 << 6)`，得到 `0b00000000000000000000000011000000`
- 取反：0变1，1变0：`~(3 << 6)`, 得到 `0xb11111111111111111111111100111111`
- 现在，将寄存器值与我们的数字进行逻辑"与"操作，6-7位与'0'后会变0，其它位与'1'后不变，这就是我们想要的：`REG &= ~(3 << 6)`。注意，保持其它位的值不变是重要的，我们并不想改变其它位的配置。

一般地，如果我们想将 X-Y 位清除，或者说设为0，这样做：

```c
PERIPHERAL->REGISTER &= ~(NUMBER_WITH_N_BITS << X);
```

最后，我们把那些位设为我们想要的值，则需要把想要的值左移X位，然后与寄存器当前值进行逻辑"或"运算：

```c
PERIPHERAL->REGISTER |= VALUE << X;
```

现在，你应该明白了，下面的两行代码将把GPIOA MODER寄存器的6-7位设为1，即输出模式：

```c
  * (volatile uint32_t *) (0x40020000 + 0) &= ~(3 << 6);  // CLear bit range 6-7
  * (volatile uint32_t *) (0x40020000 + 0) |= 1 << 6;     // Set bit range 6-7 to 1
```

还有一些寄存器没有被映射到MCU外设，而是被映射到了ARM CPU的配置和控制。例如，有一个"Reset and clock control"单元（RCC），在数据手册第6节有描述，这些寄存器用来配置系统时钟和一些其它的事情。

### 可读性更好的外设寄存器编程

在前一节我们已经学习到可以通过直接访问存储地址来读写外设寄存器，下面复习下将GPIO A3设为输出模式的代码：

```c
  * (volatile uint32_t *) (0x40020000 + 0) &= ~(3 << 6);  // CLear bit range 6-7
  * (volatile uint32_t *) (0x40020000 + 0) |= 1 << 6;     // Set bit range 6-7 to 1
```

这段代码有些诡秘，如果不加以注释，很难理解。我们可以把这段代码重写成更易读的形式，方法就是用一个包含32位域的结构体来表示整个外设。我们来看一下数据手册8.4节中描述的GPIO外设的寄存器，它们是MODER、OTYPER、OSPEEDR、PUPDR、IDR、ODR、BSRR、LCKR、AFR，它们的偏移量分别是0、4、8，等等，以此类推，这意味着我们可以用一个32位域的结构体来表示，然后这样定义GPIOA：

```c
struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};

#define GPIOA ((struct gpio *) 0x40020000)
```

这样我们就可以定义一个设置GPIO引脚模式的函数：

```c
// Enum values are per datasheet: 0, 1, 2, 3
enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG};

static inline void gpio_set_mode(struct gpio *gpio, uint8_t pin, uint8_t mode) {
  gpio->MODER &= ~(3U << (pin * 2));        // Clear existing setting
  gpio->MODER |= (mode & 3) << (pin * 2);   // Set new mode
}
```

现在重写上面将GPIO A3设为输出模式的代码：

```c
gpio_set_mode(GPIOA, 3 /* pin */, GPIO_MODE_OUTPUT);  // Set A3 to output
```

MCU有好多个GPIO外设（也常被叫作'banks'）：A、B、C...K，在数据手册2.3节可以看到，它们映射的存储空间相隔1KB，GPIOA起始地址为0x40020000，GPIOB起始地址为0x40020400，以此类推：

```c
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))
```

我们可以给引脚进行编号，既包含组号，也包含序号。为了做到这一点，我们用一个2字节的`uint16_t`类型的数，高字节表示组号，低字节表示序号：

```c
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
```

通过这种方法，我们可以指定任意GPIO引脚：

```c
  uint16_t pin1 = PIN('A', 3);    // A3   - GPIOA pin 3
  uint16_t pin2 = PIN('G', 11);   // G11  - GPIOG pin 11
```

现在，我们用这个方法再次改写`gpio_set_mode()`函数：

```c
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin)); // GPIO bank
  uint8_t n = PINNO(pin);                 // Pin number
  gpio->MODER &= ~(3U << (n * 2));        // Clear existing setting
  gpio->MODER |= (mode & 3) << (n * 2);   // Set new mode
}
```

这样再设置GPIO A3为输出模式就很明了了：

```c
  uint16_t pin = PIN('A', 3);            // Pin A3
  gpio_set_mode(pin, GPIO_MODE_OUTPUT);  // Set to output
```

至此我们已经为GPIO外设创建了一个有用的初始化API，其它外设，比如串口，也可以用相似的方法来实现。这是一种很好的编程实践，可以让代码清晰可读。

## MCU启动和向量表

当STM32F429 MCU启动时，它会从flash存储区最前面的位置读取一个叫作“向量表”的东西。“向量表”的概念所有ARM MCU都通用，它是一个包含32位中断处理程序地址的数组。对于所有的ARM MCU，向量表前16个地址由ARM保留，其余的作为外设中断处理程序入口，由MCU厂商定义。越简单的MCU中断处理程序入口越少，越复杂的MCU中断处理程序入口则会更多。

STM32F429的向量表在数据手册表62中描述，我们可以看到它在16个ARM保留的标准中断处理程序入口外还有91个外设中断处理程序入口。

在向量表中，我们当前对前两个入口点比较感兴趣，它们在MCU启动过程中扮演了关键角色。这两个值是：初始堆栈指针和执行启动函数的地址（固件程序入口点）。

所以现在我们知道，我们必须确保固件中第2个32位值包含启动函数的地址，当MCU启动时，它会从flash读取这个地址，然后跳转到我们的启动函数。

## 最小固件

现在我们创建一个 `main.c` 文件，指定一个初始进入无限循环什么都不做的启动函数，并把包含16个标准入口和91个STM32入口的向量表放进去。用你常用的编辑器创建 `main.c` 文件，并写入下面的内容：

```c
// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  for (;;) (void) 0;  // Infinite loop
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*tab[16 + 91])(void) = {
  _estack, _reset
};
```

对于 `_reset()` 函数，我们使用了GCC编译器特定的 `naked` 和 `noreturn` 属性，这意味着标准函数的进入和退出不会被编译器创建，这个函数永远不会返回。

`void (*tab[16 + 91])(void)` 这个表达式的意思是：定义一个16+91个指向没有返回也没有参数的函数的指针数组，每个这样的函数都是一个中断处理程序，这个指针数组就是向量表。

我们把 `tab` 向量表放到一个独立的叫作 `.vectors` 的区段，后面需要告诉链接器把这个区段放到固件最开始的地址，也就是flash存储区最开始的地方。前2个入口分别是：堆栈指针和固件入口，目前先把向量表其它值用0填充。

### 编译

我们来编译下代码，打开终端并执行：

```sh
$ arm-none-eabi-gcc -mcpu=cortex-m4 main.c -c
```

成功了！编译器生成了 `main.o` 文件，包含了最小固件，虽然这个固件程序什么都没做。这个 `main.o` 文件是ELF二进制格式的，包含了多个区段，我们来具体看一下：

```sh
$ arm-none-eabi-objdump -h main.o
...
Sections:
Idx Name          Size      VMA       LMA       File off  Algn
  0 .text         00000002  00000000  00000000  00000034  2**1
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
  1 .data         00000000  00000000  00000000  00000036  2**0
                  CONTENTS, ALLOC, LOAD, DATA
  2 .bss          00000000  00000000  00000000  00000036  2**0
                  ALLOC
  3 .vectors      000001ac  00000000  00000000  00000038  2**2
                  CONTENTS, ALLOC, LOAD, RELOC, DATA
  4 .comment      0000004a  00000000  00000000  000001e4  2**0
                  CONTENTS, READONLY
  5 .ARM.attributes 0000002e  00000000  00000000  0000022e  2**0
                  CONTENTS, READONLY
```

注意现在所有区段的 VMA/LMA 地址都是0，这表示 `main.o` 还不是一个完整的固件，因为它没有包含各个区段从哪个地址空间载入的信息。我们需要链接器从 `main.o` 生成一个完整的固件 `firmware.elf`。

`.text` 区段包含固件代码，在上面的例子中，只有一个 `_reset()` 函数，2个字节长，是跳转到自身地址的 `jump` 指令。`.data` 和 `.bss`(初始化为0的数据) 区段都是空的。我们的固件将被拷贝到偏移0x8000000的flash区，但是数据区段应该被放到RAM里，因此 `_reset()` 函数应该把 `.data` 区段拷贝到RAM，并把整个 `.bss` 区段写入0。现在 `.data` 和 `.bss` 区段是空的，我们修改下 `_reset()` 函数让它处理好这些。

为了做到这一点，我们必须知道堆栈从哪开始，也需要知道 `.data` 和 `.bss` 区段从哪开始。这些可以通过“链接脚本”指定，链接脚本是一个带有链接器指令的文件，这个文件里存有各个区段的地址空间以及对应的符号。

### 链接脚本

创建一个链接脚本文件 `link.ld`，然后把一下内容拷进去：

```
ENTRY(_reset);
MEMORY {
  flash(rx)  : ORIGIN = 0x08000000, LENGTH = 2048k
  sram(rwx) : ORIGIN = 0x20000000, LENGTH = 192k  /* remaining 64k in a separate address space */
}
_estack     = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */

SECTIONS {
  .vectors  : { KEEP(*(.vectors)) }   > flash
  .text     : { *(.text*) }           > flash
  .rodata   : { *(.rodata*) }         > flash

  .data : {
    _sdata = .;   /* .data section start */
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;  /* .data section end */
  } > sram AT > flash
  _sidata = LOADADDR(.data);

  .bss : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram

  . = ALIGN(8);
  _end = .;     /* for cmsis_gcc.h  */
}
```

下面分段解释下：

```
ENTRY(_reset);
```

这行是告诉链接器在生成的ELF文件头中 "entry point" 属性的值。没错，这跟向量表重复了，这个的目的是为像 Ozone 这样的调试器设置固件起始的断点。调试器是不知道向量表的，所以只能依赖ELF文件头。

```
MEMORY {
  flash(rx)  : ORIGIN = 0x08000000, LENGTH = 2048k
  sram(rwx) : ORIGIN = 0x20000000, LENGTH = 192k  /* remaining 64k in a separate address space */
}
```

这是告诉链接器有2个存储区空间，以及它们的起始地址和大小。

```
_estack     = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */
```

这行告诉链接器创建一个 `_estack` 符号，它的值是RAM区的最后，这也是初始化堆栈指针的值。

```
  .vectors  : { KEEP(*(.vectors)) }   > flash
  .text     : { *(.text*) }           > flash
  .rodata   : { *(.rodata*) }         > flash
```

这是告诉链接器把向量表放在flash区最前，然后是 `.text` 区段（固件代码），再然后是只读数据 `.rodata`。

```
  .data : {
    _sdata = .;   /* .data section start */
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;  /* .data section end */
  } > sram AT > flash
  _sidata = LOADADDR(.data);
```

这是 `.data` 区段，告诉链接器创建 `_sdata` 和 `_edata` 两个符号，我们将在 `_reset()` 函数中使用它们将数据拷贝到RAM。

```
  .bss : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram
```

`.bss` 区段也是一样。

### 启动代码

现在我们来更新下 `_reset` 函数，把 `.data` 区段拷贝到RAM，然后把 `.bss` 区段初始化为0，再然后调用 `main()` 函数，在 `main()` 函数有返回的情况下进入无限循环：

```c
int main(void) {
  return 0; // Do nothing so far
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *src = &_sbss; src < &_ebss; src++) *src = 0;
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}
```

下面的框图演示了 `_reset()` 如何初始化 `.data` 和 `.bss`：

![](images/mem2.svg)

`firmware.bin` 文件由3部分组成：`.vectors`(中断向量表)、`.text`(代码)、`.data`(数据)。这些部分根据链接脚本被分配到不同的存储空间：`.vectors` 在flash的最前面，`.text` 紧随其后，`.data` 则在那之后很远的地方。`.text` 中的地址在flash区，`.data` 在RAM区。例如，一个函数的地址是 `0x8000100`，则它位于flash中。而如果代码要访问 `.data` 中的变量，比如位于 `0x20000200`，那里将什么也没有，因为在启动时 `firmware.bin` 中 `.data` 还在flash里！这就是为什么必须要在启动代码中将 `.data` 区段拷贝到RAM。

现在我们可以生成完整的 `firmware.elf` 固件了：

```sh
$ arm-none-eabi-gcc -T link.ld -nostdlib main.o -o firmware.elf
```

再次检验 `firmware.elf` 中的区段：

```sh
$ arm-none-eabi-objdump -h firmware.elf
...
Sections:
Idx Name          Size      VMA       LMA       File off  Algn
  0 .vectors      000001ac  08000000  08000000  00010000  2**2
                  CONTENTS, ALLOC, LOAD, DATA
  1 .text         00000058  080001ac  080001ac  000101ac  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
...
```

可以看到，`.vectors` 区段在flash的起始地址0x8000000，`.text` 紧随其后。我们在代码中没有创建任何变量，所以没有 `.data` 区段。

## 烧写固件

现在可以把这个固件烧写到板子上了！

先把 `firmware.elf` 中各个区段抽取到一个连续二进制文件中：

```sh
$ arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
```

然后使用 `st-link` 工具将 `firmware.bin` 烧入板子，连接好板子，然后执行：

```sh
$ st-flash --reset write firmware.bin 0x8000000
```

这样就把固件烧写到板子上了。

## Makefile：构建自动化

我们可以用 `make` 命令行工具替代手动敲入“编译”、“链接”、“烧写”这些命令，自动完成整个过程。`make` 工具使用一个名为 `Makefile` 的配置文件，从中读取执行动作的指令。这种自动化方式非常棒，因为这样可以把构建固件的过程、使用了哪些编译标记等也文档化。

在 https://makefiletutorial.com 上有一个非常好的给初学者的Makefile教程，强烈建议看一下。下面我将列出一些非常必要的概念以理解我们所使用的Makefile。对于已经很熟悉 `make` 的朋友，可以跳过这一部分。

其实 `Makefile` 的格式并不复杂：

```make
action1:
	command ...     # Comments can go after hash symbol
	command ....    # IMPORTANT: command must be preceded with the TAB character

action2:
	command ...     # Don't forget about TAB. Spaces won't work!
```

现在我们可以跟动作名（也被称作目标）一起调用 `make` 来执行相应的动作：

```sh
$ make action1
```

当然，也可以在命令中定义和使用变量，动作也可以是需要创建的文件名：

```make
firmware.elf:
	COMPILATION COMMAND .....
```

任何动作都可以有一个依赖列表。例如，`firmware.elf` 依赖源文件 `main.c`，当 `main.c` 改变时，`make build` 就会重新构建 `firmware.elf`:

```
build: firmware.elf

firmware.elf: main.c
	COMPILATION COMMAND
```

我们已经准备好为固件编写 `Makefile`，定义一个 `build` 动作/目标：

```make
CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -g3 -Os -ffunction-sections -fdata-sections -I. \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c 

build: firmware.elf

firmware.elf: $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@
```

在这里我们定义了一些编译标记。`?=` 表示这是默认值，我们可以在命令行中覆盖它们，像这样：

```sh
$ make build CFLAGS="-O2 ...."
```

上面的 `Makefile` 文件中定义了 `CFLAGS`、`LDFLAGS`、`SOURCES` 变量，然后我们告诉 `make` ，当要 `build` 时创建 `firmware.elf` 文件，它依赖 `main.c` 文件，使用 `arm-none-eabi-gcc` 编译器和给定的编译标记生成它。`$@` 特殊变量会被展开成动作/目标名，在这个例子中是 `firmware.elf`。

现在调用 `make` 试一下：

``` sh
$ make build
arm-none-eabi-gcc main.c  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common -Wconversion -g3 -Os -ffunction-sections -fdata-sections -I. -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16  -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=firmware.elf.map -o firmware.elf
```

如果我们再次运行：

```sh
$ make build
make: Nothing to be done for `build'.
```

`make` 会检查 `firmware.elf` 和依赖项 `main.c` 的修改时间，如果是它们是最新的，则什么都不做。如果我们修改下 `main.c`，则会重新构建：

```sh
$ touch main.c # Simulate changes in main.c
$ make build
```

现在，还剩下“烧写”这个动作/目标：

```make
firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $(TARGET).bin 0x8000000
```

OK，现在从终端中执行命令 `make flash` 就会创建 `firmware.bin` 文件，然后通过 `st-link` 烧入板子。当 `main.c` 改变时，这个命令也会重新构建，因为 `firmware.bin` 依赖 `firmware.elf`，`firmware.elf` 又依赖 `main.c`。所以我们的开发循环就是这样的两步：

```sh
# Develop code in main.c
$ make flash
```

还有一个良好实践就是在 `Makefile` 中添加 `clean` 动作，以删除构建生成的文件：

```
clean:
	rm -rf firmware.*
```

完整项目的源代码可以在 [step-0-minimal](step-0-minimal) 文件夹找到。

## 闪烁LED

现在我们已经搭建好了完整的构建、烧写的基础设施，是时候让固件做点儿有用的事情了。什么是有用的事情？当然是闪烁LED了！Nucleo-F429ZI开发板有3颗LED，在开发板数据手册的6.5节，我们可以看到板载LED连接的引脚：

- PB0: green LED
- PB7: blue LED
- PB14: red LED

再次修改 `main.c` 文件，添加上引脚定义，然后把蓝色LED引脚设为输出模式，开始无限循环。首先，把我们之前讨论过的GPIO定义和模式设置拷贝过来，注意，现在又新加了一个 `BIT(position)` 工具宏：

```c
#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))

// Enum values are per datasheet: 0, 1, 2, 3
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3) << (n * 2);    // Set new mode
}
```

某些微控制器在上电时会把所有外设都自动使能，然而，STM32微控制器在上电时外设是默认关闭的，以降低功耗。为了使能GPIO外设，我们需要通过RCC单元使能外设时钟。在芯片数据手册7.3.10节，可以找到AHB1ENR寄存器与此相关，还是先定义整个RCC单元：

```c
struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
};
#define RCC ((struct rcc *) 0x40023800)
```

在AHB1ENR寄存器文档中可以看到0-8位控制GPIOA - GPIOI的时钟：

```c
int main(void) {
  uint16_t led = PIN('B', 7);            // Blue LED
  RCC->AHB1ENR |= BIT(PINBANK(led));     // Enable GPIO clock for LED
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  for (;;) asm volatile("nop");          // Infinite loop
  return 0;
}
```

接下来需要做的就是找到如何开关GPIO引脚，然后在主循环中点亮LED，延时，熄灭LED，延时。在芯片数据手册8.4.7节，可以看到BSRR寄存器与设置电压高低有关，低16位设置ODR寄存器输出高，高16位设置ODR寄存器输出低。为此定义一个API函数：

```c
static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR |= (1U << PINNO(pin)) << (val ? 0 : 16);
}
```

下一步我们需要实现一个延时函数，目前还不需要精确延时，所以定义一个 `spin()` 函数，执行NOP指令给定的次数：

```c
static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}
```

最后，修改主循环来让LED闪烁起来：

```c
  for (;;) {
    gpio_write(pin, true);
    spin(999999);
    gpio_write(pin, false);
    spin(999999);
  }
```

执行 `make flash` 来看蓝色LED闪烁吧！

完整项目源码可以在 [step-1-blinky](step-1-blinky) 文件夹找到。

## 用SysTick中断实现闪烁

为了实现精确的时间控制，我们应该使能ARM的SysTick中断。SysTick是一个24位的硬件计数器，是ARM核的一部分，因为在ARM的数据手册中有它的文档。从芯片数据手册中可以看到，SysTick有4个寄存器：

- CTRL，使能/禁能SysTick
- LOAD，初始计数值
- VAL，当前计数值，每个时钟周期递减
- CALIB，校准寄存器

每次VAL减到0，就会产生一个SysTick中断，SysTick中断在向量表中的索引为15，我们需要设置它。在启动时，Nucleo-F429ZI的时钟是16MHz，我们可以配置SysTick计数器使其每毫秒产生一个中断。

首先，定义SysTick外设，我们知道有4个寄存器，并且从芯片数据手册中可以知道SysTick地址为0xe000e010，编写代码：

```c
struct systick {
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};
#define SYSTICK ((struct systick *) 0xe000e010)
```

接下来定义一个API函数配置SysTick，我们需要在 `SYSTICK->CTRL` 寄存器使能SysTick，同时也需要在 `RCC->APB2ENR` 中使能它的时钟，这一点在芯片数据手册7.4.14节描述：

```c
#define BIT(x) (1UL << (x))
static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APB2ENR |= BIT(14);                   // Enable SYSCFG
}
```

默认情况下，Nucleo-F429ZI运行频率为16MHz，这表示如果我们调用 `systick_init(16000000 / 1000)`，就会每毫秒产生一个SysTick中断。还需要定义一个中断处理函数，现在只是在中断处理函数中递增一个32位数：

```c
static volatile uint32_t s_ticks; // volatile is important!!
void SysTick_Handler(void) {
  s_ticks++;
}
```

注意要用 `s_ticks` 的 `volatile` 说明符，在中断处理函数中的任何变量都应该标记为 `volatile`，这样可以避免编译器通过在寄存器中缓存变量值的优化操作。`volatile` 关键字可以是生成的代码总是从存储空间载入变量值。

现在把SysTick中断处理函数加到向量表中：

```c
__attribute__((section(".vectors"))) void (*tab[16 + 91])(void) = {
    0, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
```

这样我们就有精确的毫秒时钟了！再创建一个任意周期的定时器工具函数：

```c
// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint64_t prd, uint64_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}
```

使用这个精确的定时器函数更新闪烁LED的主循环，例如，每250ms闪烁一次：

```c
  uint32_t timer, period = 250;          // Declare timer and 250ms period
  for (;;) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on;       // This block is executed
      gpio_write(led, on);  // Every `period` milliseconds
      on = !on;             // Toggle LED state
    }
    // Here we could perform other activities!
  }
```

通过使用 `SysTick` 和 `timer_expired()` 工具函数，使主循环成为非阻塞的。这意味着在主循环中我们还可以执行许多动作，例如，创建不同周期的定时器，它们都能及时被触发。

完整项目源码可以在 [step-2-systick](step-2-systick) 文件夹找到。
