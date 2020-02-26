# 小拍机器人计算节点
## 作者：钱宇辰

### 人脸检测方法比较

|机器|检测方法|选项|分辨率|时间|
|---|---|---|---|---|
|i7-8550u|[Haar detector](https://github.com/prateekvjoshi/Body-Parts-Detection)||640*480|300ms~700ms|
|||-O3|320*240|100ms~200ms|
||[libfacedetection](https://github.com/ShiqiYu/libfacedetection)||640*480|500ms~600ms|
||||320*240|100ms~200ms|
|||-O3|320*240|10ms~40ms|

Haar检测子不具备旋转不变性，检测精度不高（实验中出现把插座口识别成人脸的情况），在低分辨率下难以识别出目标且速度较慢。

libfacedetection具有一定的旋转不变性，精度较高，在低分辨率下也运行良好，速度快，在-O3优化下可以满足实时性的要求。