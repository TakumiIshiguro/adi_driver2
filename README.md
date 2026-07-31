# adi_driver2 [![build-test](https://github.com/uhobeike/adi_driver2/actions/workflows/build-test.yaml/badge.svg)](https://github.com/uhobeike/adi_driver2/actions/workflows/build-test.yaml)


## Overview
[tork-a/adi_driver](https://github.com/tork-a/adi_driver)のROS 2実装です。

## Output

### Output

#### ⚠launchファイルで立ち上げた場合は、imuというネームスペースが付くようになっています
| **Name（Topic）**        | **Type**                                 | **Description**                                      | 
| -------------------- | ------------------------------------ | ------------------------------------------------ | 
| `/imu/data_raw`      | `sensor_msgs::msg::Imu`         | IMUのデータ                                           | 
| `/imu/temperature`      | `sensor_msgs::msg::Temperature`        | IMUの温度データ                                   | 

### Parameters

| **Name（Parameter）**   | **Type**        | **Description**            | 
| ------------------- | ----------- | ---------------------- | 
| `device`           | `std::string` | デバイスファイル名           | 
| `frame_id`          | `std::string` | IMU座標系       | 
| `burst_mode`         | `bool` | バーストモードを使用するか         |
| `publish_temperature`       | `bool`         | 温度のtopicを公開するか   | 
| `rate`   | `double`      | センサ出力およびtopicのパブリッシュレート（既定値: 200 Hz） |


## Run

```
ros2 launch adi_driver2 adis16465.launch.py
ros2 run rqt_plot rqt_plot  /imu/data_raw/angular_velocity /imu/data_raw/linear_acceleration
```
