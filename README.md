# README

### 前置说明

- 本分支内容针对**2804**电机进行更新，2715电机不再适用

### 改动说明

1. 电机修改成2804电机
2. 外框修改适配2804电机
3. 主控PCB
    1. 下载接口由1.27排针修改成HC-1.0-6PWT
    2. 删除MCU2的陶瓷天线
4. 下载器PCB
    1. 增加HC-1.0-6PWT接口，可以直接使用1.0 6P线与主控下载口连接并下载代码
    2. 预留2.54 6P孔位
5. IMU PCB
    1. 把5P接口移动到PCB中间，避免与主控干涉
6. MCU1固件变更
    1. 删除GA指令
    2. 增加指令
        1. 配置WIFI是否开启
        2. 配置WIFI SSID
        3. 配置WIFI PASSWORD
        4. 恢复默认参数
        5. 进入边平衡角度自动校准模块
        6. 进入点平衡角度自动校准模块
    3. 调整FreeRTOS任务
    4. 限制电机最大电压为5V
    5. 提高IMU初始化的零偏阈值到10
    6. 把LQR默认参数移动到config.h文件
    7. 删除第一次下载固件配置flash写入的配置，修改为自动识别是否为第一次下载固件，如为第一次下载固件则自动写入默认配置参数到flash
    8. 适配2804电机，调整LQR参数和方向
7. 直接提供已经编译好的固件

### 乐鑫科技FLASH下载软件地址：

```cpp
https://www.espressif.com.cn/zh-hans/support/download/other-tools
```

### 下载器(CH340)驱动

```cpp
https://www.wch.cn/downloads/CH343SER_EXE.html
```

### 固件下载说明

- 查看5.Doc/ESP32固件下载说明.pdf

### 调试命令

- 查看5.Doc/调参命令说明.pdf