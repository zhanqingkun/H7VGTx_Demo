# H7VGTx_Demo

## FPU单元说明

![](doc/image/freertos_ENABLE_FPU.png)

CubeMX中在FREEROTS有关于FPU的使能，经研究发现

H7是用不到ENABLE_MPU的，使能与否都没关系，它是给ARMv8M架构新版准备的，详情看以下链接：

https://www.freertos.org/2020/04/using-freertos-on-armv8-m-microcontrollers.html

在HAL库中 FPU的开启与否在system_stm32h7xx.c中的SystemInit函数，如下

```
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif
```

**对于具体的FPU功能添加如下（本工程已添加）：**

1. ![](doc/image/FPU1.png)由CubeMX生成的代码第一次生成勾选添加全部相关库，会在rm_main\Drivers\CMSIS中加上各种库，其中就有DSP库。(后续将各种库删除，并将选项改为只添加所需相关库来减小工程大小)

2. 移植和添加库和头文件

   将所需的文件移植到\rm_main\User\Math\DSP中，其中

   arm_cortexM7lfdp_math.lib（双精度浮点Cortex-M7小端模式）为库文件，添加进工程中。

   \Include为各头文件，添加进工程头文件路径。

   \Source为各函数具体实现，不用添加进工程，需要研究具体实现时查看即可。

3. 添加宏定义

   打开Keil工程的配置界面，切换至C/C++选项卡，添加如下宏定义：

   FPU_PRESENT ==1U,ARM_MATH_CM4,CC_ARM,__TARGET_FPU_VFP

   可以在core_cm7.h文件中看到起效，且system_stm32h7xx.c中的SystemInit函数中的宏定义也起效。

   ![](doc/image/FPU2.png)

4. 直接在main.h中添加arm_math.h文件，为整个工程配置上DSP库

**附加说明**

<img src="doc/image/FPU3.png" style="zoom: 50%;" />

此处注意是Double Precision
