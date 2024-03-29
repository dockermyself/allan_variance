
Allan 方差的计算步骤如下：
1.收集数据：首先，需要收集时间序列数据，通常是频率或相位测量数据，这些数据可以是从传感器、设备或系统中采集得到的。
2.计算平均差值：将数据按照不同的时间间隔分组，并计算每个时间间隔内的差值。
3.计算方差：对每个时间间隔内的差值进行平方并求平均，得到方差。 重复计算：重复上述步骤，不断增大时间间隔，得到一系列方差值。
4.绘制 Allan 方差图：将得到的方差值按时间间隔的对数尺度绘制成图表，即可得到 Allan 方差图。
5.通过分析 Allan 方差图，可以识别出频率稳定性的特征，例如白噪声、布朗噪声和随机游走等。
6.本文参考
https://github.com/ori-drs/allan_variance_ros
在其基础上重构了相关代码，去除ROS环境需求，使得使用更加方便。

![acceleration](data/acceleration.png)
![gyro](data/gyro.png)
我的CSDN博客地址:
https://blog.csdn.net/DeepLearnerZxf/article/details/137144257?csdn_share_tail=%7B%22type%22%3A%22blog%22%2C%22rType%22%3A%22article%22%2C%22rId%22%3A%22137144257%22%2C%22source%22%3A%22DeepLearnerZxf%22%7D