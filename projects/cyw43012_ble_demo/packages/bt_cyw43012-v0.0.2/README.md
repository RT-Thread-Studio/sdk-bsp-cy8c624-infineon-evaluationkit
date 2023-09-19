# cyw43012 ble for RT-Thread

## 1. 简介

**cyw43012 ble for RT-Thread**是由英飞凌基于 WIFI&&BLE SOC的 BLE模块。该仓库为**cyw43012**在 RT-Thread 上的移植。

### 1.1. 文件结构

| 文件夹 | 说明 |
| ---- | ---- |
| abstraction-rtos  | RTOS 抽象层，目前配合 FREERTOS 兼容层实现，后面会替换为RTT |
| btstack | 英飞凌提供的蓝牙协议栈 |
| btstack-integration | 开发板适配 |
| kv-store | kv存储组件 |

### 1.2 许可证

cyw43012 ble for RT-Thread package 遵循 Apache 2.0 许可，详见 `LICENSE` 文件。

### 1.3 依赖

- RT-Thread 4.0+

## 2. 注意事项

无

## 4. 联系方式

- 维护：RT-Thread 开发团队
