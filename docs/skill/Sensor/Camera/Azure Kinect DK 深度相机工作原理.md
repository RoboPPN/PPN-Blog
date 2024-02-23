Azure Kinect DK 深度相机实现调幅连续波 (AMCW) 时差测距 (ToF) 原理。 该相机将近红外 (NIR) 频谱中的调制光投射到场景中。 然后，它会记录光线从相机传播到场景，然后从场景返回到相机所花费的间接时间测量值

处理这些测量值可以生成深度图。 深度图是图像每个像素的一组 Z 坐标值，以毫米为单位

连同深度图一起，我们还可以获得所谓的清晰 IR 读数。 清晰 IR 读数中的像素值与从场景返回的光线量成正比。 图像类似于普通的 IR 图像。 下图显示了示例深度图（左）的对应的清晰 IR 图像（右）

![请添加图片描述](https://docs.microsoft.com/zh-cn/azure/kinect-dk/media/concepts/depth-camera-depth-ir.png)

## 参考致谢

- [Azure Kinect DK 深度相机](https://docs.microsoft.com/zh-cn/azure/kinect-dk/depth-camera)
