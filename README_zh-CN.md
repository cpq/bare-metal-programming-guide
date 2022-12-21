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

MCU有好多个GPIO外设（也常被叫做'banks'）：A、B、C...K，在数据手册2.3节可以看到，它们映射的存储空间相隔1KB，GPIOA起始地址为0x40020000，GPIOB起始地址为0x40020400，以此类推：

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

