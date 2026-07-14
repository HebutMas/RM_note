# 串口+DMA接收DT7遥控器数据

## 一、串口的结构框图

​	先了解串口的工作原理，如图所示最上方的PWDATA和PRDATA分别表示写入的数据和读出的数据，串口是stm32的一个外设，肯定具有自己的寄存器，那就是发送数据寄存器TDR和接收数据寄存器RDR，这两个寄存器的地址不会改变。串口一次只能写入或者读取1type的数据，也就是8bit。数据从PWDATA写到TDR以后，通过发送唯一寄存器，向右位移，一位一位地将数据传到IrDA SIR编解码模块中，这个模块将数据转化为高低电平的电信号，从TX输出。读取同理不再介绍。

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/871094cf58ce8627354a9cf2fae81f3a.png)



## 二、DMA的结构框图

​	Cortex—M3为MCU，加上Flash和SRAM为cpu，Flash和SRAM为cpu内部的寄存器，其中Flash存储代码，默认只读，只能通过Flash接口控制器进行写操作，SRAM存储程序中的变量，可读可写。右下角的各个外设都有自己的寄存器。

​	DMA外设就是数据转运的工具，能实现存储器到存储器、存储器到外设，外设到存储器和外设到外设的数据装运且不经过CPU的处理。左边为DMA1和DMA2，都有不同的通道，当有不同的通道同时请求DMA时，仲裁器就会根据优先级来选择谁先使用DMA。

![img](https://i-blog.csdnimg.cn/blog_migrate/3b0530da4579e0a522d21dd6745d1557.png)

![image-20250217162415965](C:\Users\15269\AppData\Roaming\Typora\typora-user-images\image-20250217162415965.png)

​	这是程序框图，每进行一次DMA的转运，程序计数器就减一，直到自动重装器清零，DMA就停止工作，M2M表示工作方式为存储器到存储器，软件触发：最快速度将传输计数器清零，不能和循环模式重复使用。硬件触发：外设触发，例如串口传输一次数据触发一次DMA，当开启自动重装器开启以后，就为循环模式。开关控制就是是否打开DMA。

​	在CubeMX中配置DMA时，可以选择模式为普通或者循环，右边可以选择在一次转运以后是否将寄存器地址自动右移，和移动的大小，字节，半字和字。



## 三、一些库函数

`HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);`



第一个函数为DMA接收函数，当DMA接收完成以后触发中断，进入中断服务函数



`void DMAx_Stream1_IRQHandler(void)`



在这个服务函数中清除DMA和中断的标志位



`HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);`



最后一个是在第一个函数的基础上加上了空闲中断的启动函数，当总线空闲或者DMA接收完成就会进入中断服务函数，进行数据解算和开启下一次数据的收集。



`void USARTx_IRQHandler(void)`



在这个服务函数中清除串口中断的标志位

[具体可以看这个博客的内容](https://blog.csdn.net/qq_41555003/article/details/143892557)

这些可以实现固定长度的数据的接收，比如遥控器，下面我以大疆的官步历程讲解一下另一种方法。



## 四、DMA+空闲中断+双缓冲收集定长和不定长数据

​	在这里真的很后悔没有早点看明白官方的代码，导致长时间都用效率低下的方式接收数据。下面我不进行具体的讲解，我只附上代码和注释，具体的寄存器以及标志位，你们需要自己看F4的手册理解。

​	可能很多同学会有困惑，如果使用上面的空闲中断，不定长数据应该怎么接收呢，而且DBUS，电管，图传三个串口，公用同一个回调函数，那回调函数写在哪里呢？下面，我们可以使用中断服务函数USARTx_IRQHandler来解决，这样就可以在不同的地方进行数据接收，没必要写在回调函数里面。

​	本来USARTx_IRQHandler是写在it文件里的，不方便当作弱函数进行修改，我们需要在cubemx中修改。在NVIC配置中，code generate中将对应串口中断是否生成服务函数的√取消，这样就不会在it文件里生成这些服务函数了。

```c
	uint8_t Remote_buf1[25];
	uint8_t Remote_buf2[25];

	//初始化代码
	SET_BIT(huart3.Instance->CR3,USART_CR3_DMAR);				//打开串口的DMA请求
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);                 //打开串口的空闲中断
	__HAL_DMA_DISABLE(&hdma_usart3_rx);                         //失能DMA后再进行配置
	while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)            
    {                                                           
        __HAL_DMA_DISABLE(&hdma_usart3_rx);                     
    }                                                           
	hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);   //外设寄存器为串口3的DR接收寄存器，即源地址
	hdma_usart3_rx.Instance->M0AR = (uint32_t)(Remote_buf1);    //选择缓冲区1的地址
	hdma_usart3_rx.Instance->M1AR = (uint32_t)(Remote_buf2);    //缓冲区2的地址
	hdma_usart3_rx.Instance->NDTR = 50;                         //只有禁用DMA数据流时才能写这个寄存器
	SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);         //打开双缓冲模式，传输结束自动切换缓冲区（失能DMA也																算结束），且自动使能循环模式，DMA_SxCR 中的 CIRC 																位的状态是无关
	__HAL_DMA_ENABLE(&hdma_usart3_rx);                          //使能DMA
```

这里有有一些寄存器的修改只能在DMA关闭的状态下进行修改，所以对FDMA的一些配置和重新配置都需要关闭DMA再配置，其中==PAR==，==M0AR==，==M1AR==，==NDTR==，==CR==等寄存器自己去手册看他们的作用！！！

```c
void USART3_IRQHandler(void)
{
	
	if(huart3.Instance->SR & UART_FLAG_RXNE)//这两个清除标志位为校验错误标志位，这里我认为是避免后面处理出错才清除的
	{
		__HAL_UART_CLEAR_PEFLAG(&huart3); 
	}
	else if(USART3->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_PEFLAG(&huart3);   

		if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)//获取CR寄存器CT位的状态，0表示目标缓冲区是0
		{
			__HAL_DMA_DISABLE(&hdma_usart3_rx); 
			this_time_rx_len = 50 - hdma_usart3_rx.Instance->NDTR;
			hdma_usart3_rx.Instance->NDTR = 50;
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;//CR寄存器的CT位 |= 1，即将CT位改为1，手动切换缓冲区1
			__HAL_DMA_ENABLE(&hdma_usart3_rx);
			
			if(this_time_rx_len == 25) 
            { 
                sbus_recode(&Remote,Remote_buf1); 
            }
		}
		else
		{
			__HAL_DMA_DISABLE(&hdma_usart3_rx); 
			this_time_rx_len = 50 - hdma_usart3_rx.Instance->NDTR; 
			hdma_usart3_rx.Instance->NDTR = 50;
			hdma_usart3_rx.Instance->CR &= ~(DMA_SxCR_CT);//同理，手动切换缓冲区0，如果看不懂建议去看寄存器，以及查询这															  个宏定义是什么意思
			__HAL_DMA_ENABLE(&hdma_usart3_rx); 
			if(this_time_rx_len == 25)
			{
				sbus_recode(&Remote,Remote_buf2);
			}
		}
	}
}
```

不定长数据也差不多，自己修改吧，但是要注意的是，==裁判系统我们需要接收的0x201和0x202等数据包会发生粘包现象==，我们直接对这一整个包进行解算就行了。

![](C:\Users\15269\Pictures\Screenshots\屏幕截图 2025-04-07 001342.png)



## 五、数据解析

​	sbus的协议解析

​	[STM32 Futaba SBUS协议解析_接收机与bus-CSDN博客](https://blog.csdn.net/Brendon_Tan/article/details/89854751)

​	dbus协议解析

![image-20250710133354671](C:\Users\15269\AppData\Roaming\Typora\typora-user-images\image-20250710133354671.png)

sbus

```c
	Remote->R_RL  =  ((int16_t)Remote_buf[1] >> 0 | ((int16_t)Remote_buf[2] << 8 )) & 0x07FF;
	Remote->R_UD  =  ((int16_t)Remote_buf[2] >> 3 | ((int16_t)Remote_buf[3] << 5 )) & 0x07FF;
	Remote->L_UD =  ((int16_t)Remote_buf[3] >> 6 | ((int16_t)Remote_buf[4] << 2 ) | (int16_t)Remote_buf[5] << 10 ) & 0x07FF;
	Remote->L_RL =  ((int16_t)Remote_buf[5] >> 1 | ((int16_t)Remote_buf[6] << 7 )) & 0x07FF;
	Remote->SWA =  ((int16_t)Remote_buf[6] >> 4 | ((int16_t)Remote_buf[7] << 4 )) & 0x07FF;
	Remote->SWB =  ((int16_t)Remote_buf[7] >> 7 | ((int16_t)Remote_buf[8] << 1 ) | (int16_t)Remote_buf[9] << 9 ) & 0x07FF;
	Remote->SWC =  ((int16_t)Remote_buf[9] >> 2 | ((int16_t)Remote_buf[10] << 6 )) & 0x07FF;
	Remote->SWD =  ((int16_t)Remote_buf[10] >> 5 | ((int16_t)Remote_buf[11] << 3 )) & 0x07FF;
	Remote->VRA = ((int16_t)Remote_buf[12] >> 0 | ((int16_t)Remote_buf[13] << 8 )) & 0x07FF;
	Remote->VRB = ((int16_t)Remote_buf[13] >> 3 | ((int16_t)Remote_buf[14] << 5 )) & 0x07FF;
	
	Remote->ONLINE = Remote_buf[23];
	
	/*  其他通道的数据解析	
	 *  Remote = ((int16_t)Remote_buf[14] >> 6 | ((int16_t)Remote_buf[15] << 2 ) | (int16_t)Remote_buf[16] << 10 ) & 0x07FF;
	 *	Remote = ((int16_t)Remote_buf[16] >> 1 | ((int16_t)Remote_buf[17] << 7 )) & 0x07FF;
	 *	Remote = ((int16_t)Remote_buf[17] >> 4 | ((int16_t)Remote_buf[18] << 4 )) & 0x07FF;
	 *	Remote = ((int16_t)Remote_buf[18] >> 7 | ((int16_t)Remote_buf[19] << 1 ) | (int16_t)Remote_buf[20] << 9 ) & 0x07FF;
	 *	Remote = ((int16_t)Remote_buf[20] >> 2 | ((int16_t)Remote_buf[21] << 6 )) & 0x07FF;
	 *	Remote = ((int16_t)Remote_buf[21] >> 5 | ((int16_t)Remote_buf[22] << 3 )) & 0x07FF;
	 */
```

dbus

```c
/*摇杆通道*/
	Remote->Channel[0] = (Remote_data[0] | (Remote_data[1] << 8)) & 0x07ff;
	Remote->Channel[1] = ((Remote_data[1] >> 3) | (Remote_data[2] << 5)) & 0x07ff;
	Remote->Channel[2] = ((Remote_data[2] >> 6) | (Remote_data[3] << 2) | (Remote_data[4] << 10)) &0x07ff; 
	Remote->Channel[3] = ((Remote_data[4] >> 1) | (Remote_data[5] << 7)) & 0x07ff;
	
	/*拨杆*/
	Remote->S1 = ((Remote_data[5] >> 4) & 0x000C) >> 2; 
	Remote->S2 = ((Remote_data[5] >> 4) & 0x0003);
	
	/*鼠标移动*/
	Remote->Mouse_x = Remote_data[6] | (Remote_data[7] << 8);
	Remote->Mouse_y = Remote_data[8] | (Remote_data[9] << 8);
	Remote->Mouse_z = Remote_data[10] | (Remote_data[11] << 8);
	
	/*鼠标左右键*/
	Remote->Mouse_left = Remote_data[12];
	Remote->Mouse_right = Remote_data[13];
	
	/*键盘按键*/
	Remote->Key.W = Remote_data[14] && 0x01;
	Remote->Key.S = (Remote_data[14] >> 1) && 0x01;
	Remote->Key.A = (Remote_data[14] >> 2) && 0x01;
	Remote->Key.D = (Remote_data[14] >> 3) && 0x01;
	Remote->Key.Q = (Remote_data[14] >> 4) && 0x01;
	Remote->Key.E = (Remote_data[14] >> 5) && 0x01;
	Remote->Key.Shift = (Remote_data[14] >> 6) && 0x01;
	Remote->Key.Ctrl = (Remote_data[14] >> 7) && 0x01;
	
	/*遥控器拨轮*/
	Remote->wheel = (Remote_data[16] | Remote_data[17] << 8);
```

