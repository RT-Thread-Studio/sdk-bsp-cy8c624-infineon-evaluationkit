# cyw43012 模块 BLE Demo

## 模块简介
**cyw43012 模块** 是由英飞凌基于 CYW43012 开发的 SDIO 高速 wifi&&蓝牙 模块。CYW43012 提供超低功耗的Wi-Fi®和蓝牙®连接，可延长可穿戴设备、智能家居产品和便携式音频应用的电池续航时间。同时，其先进的共存引擎可为2.4与5-GHz双频Wi-Fi以及双模式蓝牙/低功耗蓝牙 (BLE)5.0应用同时提供最佳的组合性能。

<img src="https://oss-club.rt-thread.org/uploads/20230918/10d7f1e93c7c0e361824f435199534e1.jpg.webp" width="30%" height="30%" />

### 硬件连接

psoc6-evaluationkit-062S2 开发板上一开始就预留了这款模块的接口，只需要把模块插上去就好了，注意方向不要接错了。
因为这个模块的工作电压是1.8v，所以如果需要使用这个模块的话，需要手动切换sdio接口的电压。在老版的开发板上预留了1.8v和3.3v切换的电阻。需要手动把3.3v的电阻换下来，然后短接1.8v。

> PS 新版的开发板，添加了1.8v 和3.3v的切换开关，可以通过切换拨码开关完成SDIO接口电压的切换

<img src="https://oss-club.rt-thread.org/uploads/20230918/7bdec17219030c97d367008bd34cfa6a.jpg.webp" width="50%" height="50%" />

<img src="https://oss-club.rt-thread.org/uploads/20230918/d74cff286ae4ccb0bcb332a28722aa60.jpg.webp" width="50%" height="50%" />



## 运行测试

下载并复位开发板，让程序运行起来，可以看到，BLE 固件已经正常初始化了。

![screenshot_image.png](https://oss-club.rt-thread.org/uploads/20230918/2de7cb07524ecd889ffac9969459a36e.png.webp)
稍等一会等待蓝牙模块初始化完成。

![screenshot_image.png](https://oss-club.rt-thread.org/uploads/20230918/26230c5b4c12847c179dc1b64aa642d9.png.webp)

## 体验 hello 示例
1. 安装英飞凌 蓝牙连接APP
扫描下面的二维码安装：

![screenshot_image.png](https://oss-club.rt-thread.org/uploads/20230918/cf55d02c02b2343c1a335732997ac09d.png)
2. 按照下面的使用说明体验 hello 示例

![screenshot_image.png](https://oss-club.rt-thread.org/uploads/20230918/0c0e55a50ca21409aa23341e6c075a23.png.webp)

关于示例demo的详细介绍可以参考 https://github.com/Infineon/mtb-example-btstack-freertos-hello-sensor