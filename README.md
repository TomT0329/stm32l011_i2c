# stm32l011_i2c

include all the communication peripha

開發環境

platform = ststm32

framework = libopencm3

board = nucleo_l011k4

ide = VSCODE + platformIO


I2C詳細介紹可見[實作實驗室](https://makerpro.cc/2019/12/intro-to-inter-integrated-circuit/)，可以了解電路行為以及i2c protocol。
VScode and platformIO [介紹](https://ithelp.ithome.com.tw/articles/10290514)。


* platformio.ini：PIO 的專案設定檔，可以在這裡面調整專案設定，例如設定多種環境。
* .gitignore：PIO 會自動設定好基本的 .gitignore。
* src/：存放主要程式碼，如 main.c。
* include/：可以用來存 .h 標頭檔。
* lib/：存放要調用的 Library，PIO 會自動處理。
* test/：存放單元測試（Unit Testing）的程式。
