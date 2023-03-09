# M031BSP_SPI_Slave_RX
 M031BSP_SPI_Slave_RX

update @ 2023/03/09

1. Intial USCI0 , to emulate SPI master send data , and always pull SS low

    - SPI CLK : USCIx_CLK PA11
	
	- SPI SS  : USCIx_CTL0 PC14
	
	- SPI MOSI : USCIx_DAT0 PA10
	
	- SPI MISO : USCIx_DAT1 PA9 , NOT USED

2. Inital SPI0 slave , with SPI unit transfer interrupt , to receive data from SPI master

	- PA.2 is SPI0_CLK,

    - PA.3 is SPI0_SS,   
	      	   	   
	- PA.0 is SPI0_MOSI

	- PA.1 is SPI0_MISO , NOT USED

3. Below is log message , SPI master send data per 10ms , and printf SPI slave result

![image](https://github.com/released/M031BSP_SPI_Slave_RX/blob/main/log.jpg)	

4. Below is LA capture , 

![image](https://github.com/released/M031BSP_SPI_Slave_RX/blob/main/LA_all.jpg)	

Below is LA capture , first byte

![image](https://github.com/released/M031BSP_SPI_Slave_RX/blob/main/LA_5A00.jpg)	

Below is LA capture , second byte

![image](https://github.com/released/M031BSP_SPI_Slave_RX/blob/main/LA_5A01.jpg)	


