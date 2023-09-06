# Multi-Cameras Sample

使用[pypylon](https://github.com/basler/pypylon)编写了basler相机采样和下游任务适配的代码.

## 启动PTP服务

在ubuntu终端执行

```bash
sudo ptpd -i br0 -M -C
```

若不执行,相机并不会参考主机的时间,而是以自己作为时间参考.

## 相机采样

采样前确定相机间已经通过硬件进行了连接,并保证没有其他进程使用相机.通过如下命令执行`src/grab.py`

```bash
cd src
python grab.py \
    --rate 10
    --path1 ./cam0/data/
    --path2 ./cam1/data/
    --exposure_time 5000
    --width 2448
    --height 2048
```

采样的图片以采样时刻的时间戳为名称,两个相机无法做到完全同时拍摄.

## 时间戳对齐

`src\sync.py`用于左右相机的时间戳对齐,会为左相机匹配右侧相机拍摄的时间差异最小且在thresh_hold$[ns]$内的图像,执行命令:

```bash
python sync.py --path1 /path/to/left_camera/samples --path2 /path/to/rightcamera/samples \
    --out_path_name data_ --thresh_hold 100000
```

特别的,若在某一次触发两个相机中的一个存在漏帧,则同步结果该时刻就会缺少采样结果.

## 生成csv文件

`src\csv.py`用于为采样的结果生成如下形式的.csv文件:

```
# timestamp [ns],filename
1691737722515168334,1691737722515168334.png
1691737722565310041,1691737722565310041.png
......
1691737724965095190,1691737724965095190.png
```

执行命令:

```bash
python csv.py --path /path/to/images/ --name data
```