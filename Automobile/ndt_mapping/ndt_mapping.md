## 使用NDT（正态分布变换）进行点云建图和定位
### 前言
定位模块是自动驾驶最核心的模块之一，定位又包括全局定位和局部定位，对于自动驾驶，其精度需要达到厘米级别。本文我们将讨论全局定位，即确定无人车在全局下的位置。  
传统的AGV使用一类SLAM（simultaneous localization and mapping）的方法进行同时建图和定位，但是该方法实现代价高，难度大，难以应用到自动驾驶领域。自动驾驶车辆行驶速度快，距离远，环境复杂，使得SLAM的精度下降，同时远距离的行驶将导致实时构建的地图偏移过大。因此，如果在已有高精度的全局地图地图的情况下进行无人车的定位，将极大的简化该问题。因此，将问题分为独立的两部分：建图Mapping和定位Matching。NDT是一种点云配准算法，可同时用于点云的建图和定位。

### 概要
本文包含以下内容
* NDT配准的原理
* 应用：
    * NDT-Mapping
    * NDT-Matching
    <!-- * 修改NDT_GPU以适应TX2 -->


#### NDT配准的原理
以下是一张用于配准的target_map,即已经建好的点云地图
<img src="https://user-images.githubusercontent.com/44689665/62818102-c9567e00-bb74-11e9-87b2-062b481e6972.png" width="800" height="500"/>

以下是实时扫描到的一帧点云
<img src="https://user-images.githubusercontent.com/44689665/62818101-c5c2f700-bb74-11e9-8e43-46a968a60609.png" width="800" height="500"/>

以下是两幅点云进行配准之后的结果,中间输出的坐标轴为当前位置(x y z Y P R)
<img src="https://user-images.githubusercontent.com/44689665/62818104-cb204180-bb74-11e9-938f-3989c3704904.png" width="800" height="500"/>

通过不断的比对实时扫描到的点云和已经建好的全局点云地图,我们就可以持续获得我们当前的位置.ICP(迭代最近点)等配准算法通过对所有的点或者提取的特征点进行匹配配准以确定当前的位置,但是这样就有一个问题:我们所处的环境是在不断变化的,比如树木的稀疏程度,或者环境中车辆及行人的移动,乃至固有的测量误差,这些都会导致我们实时扫描到的点云与已建立的点云地图有些许的差别,从而导致较大匹配误差.  

而NDT可以在很大程序上消除这种不确定性.NDT没有计算两个点云中点与点之间的差距,而是先将参考点云（即高精度地图）转换为**多维变量的正态分布**，如果变换参数能使得两幅激光数据匹配的很好，那么变换点在参考系中的概率密度将会很大。因此，可以考虑用优化的方法,比如牛顿法,求出使得概率密度之和最大的变换参数，此时两幅激光点云数据将匹配的最好。  

可以这样来做一个通俗的理解:NDT把我们所处的三维世界按照一定长度的立方体(比如30cm*30cm*30cm)进行了划分,类似于一个魔方,每个立方体内并不是存储一个或一些确切的点,而且存储这个立方体被占据的概率密度.当接收到需要匹配的点云时,也按照这样的划分方式进行划分,然后进行配准.  

因此,NDT具有以下的特征:
> 1. 支持更大的地图,更稠密的点云 --因为最终还是要划分成voxel的形式
> 2. 相比于ICP等基于点的匹配算法,速度更快
> 3. 更加容忍环境的细微变化

关于NDT原理详细的数学推倒,请参考AdamShan的博客[CSDN](https://blog.csdn.net/AdamShan/article/details/79230612)  
或直接参考原论文[The Normal Distributions Transform: A New Approach to Laser Scan Matching](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.10.7059&rep=rep1&type=pdf)

### 应用
得益于如今众多的开源算法,我们不必重复造轮子了.  
[AutoWare](https://gitlab.com/autowarefoundation/autoware.ai)是由日本名古屋大学和Tier IV主导的全栈开源自动驾驶系统,其core_percepetion模块中对ndt_mapping和ndt_localization进行了很好的实现.本文将从Autoware的这两个package入手,先梳理其架构和代码,再对其进行修改,以适配本地环境,如TX2等.

#### NDT_Mapping
以下是NDT_Mapping的结构图(注:不包含imu和odom,对imu和odom信息的融合将在另一篇[TODO]()文章里进行)  
<img src="https://user-images.githubusercontent.com/44689665/62818748-0d9a4c00-bb7e-11e9-9aa2-18a2fad6845c.png" />

1. 输入点云处理
    1.1 截取有效范围
        对于激光雷达,其过近的点由于落在车体上,过远的点已经非常稀疏,因此都需要被过滤掉.
    1.2 降采样
```c++
  // Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);  // 室外可以稍微大些,如0.5-2.0;室内需要小一些
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);  // filtered_scan_ptr接收降采样后的点云
```
*注:对于第一帧点云,需要直接加入map中,作为初始target_map点云*

2. 设置NDT
```c++
ndt.setTransformationEpsilon(trans_eps); // 两次变换之间允许的最大值,用于判断是否收敛,作为迭代计算完成的阈值; =0.01
ndt.setMaximumIterations(max_iter); // 最大迭代次数,超过则停止计算; =30
ndt.setStepSize(step_size); // 牛顿法优化的最大步长; =0.1
ndt.setResolution(ndt_res); // voxel的边长大小,过小造成内存占用过高,过大会导致误差偏大; =1.0
ndt.setInputSource(filtered_scan_ptr);  // 输入source点云
```
*注:ndt.setInputTarget()在更新global_map的时候进行,即直接将global_map输入到ndt_target中*

3. 进行ndt配准,计算变换矩阵
```c++
ndt.align(*output_cloud, init_guess);
```
output_cloud: 存储source经过配准后的点云(由于source_pointcloud被极大的降采样了,因此这个数据没什么用)
```c++
pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);  // 对原始点云进行变换(framd_id: /velodyne->/map)
```
init_guess: ndt计算的初始估计位置,在不使用gnss/imu/odom的情况下,以上一帧车辆位置的变换矩阵作为init_guess
**注:ndt对位置不敏感,通常在3m以内都可以迭代计算过去,但是ndt对角度比较敏感,因此初始初始的角度不能与实际相差过大(最好在±45°之内)**

4. 每n米更新一次全局地图
```c++
if (shift >= min_add_scan_shift)
  {
    map += *transformed_scan_ptr;

    // update NDT
    ndt.setInputTarget(map_ptr);

    // publish global_map
    // ...
  }
```  

同时使用单独的线程,按照一定的频率进行地图的保存.

最终效果:

<img src="https://user-images.githubusercontent.com/44689665/62829296-1564f980-bc2d-11e9-9e53-26673aa71aaf.png" />

<!-- <iframe width="560" height="315" src="https://www.youtube.com/embed/Zp7aIUpvcSE" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe> -->
<!-- <iframe src="https://www.youtube.com/embed/Zp7aIUpvcSE" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>  

<video src="https://www.youtube.com/embed/Zp7aIUpvcSE" controls="controls"></video> -->

或移步[YouTube](https://www.youtube.com/embed/Zp7aIUpvcSE)

**备注**
每帧扫描到的点云中,落在地面上的点云大概占30%.一般来讲,由于地面作为一个平面其相似性很强,因此这部分点云对定位的作用是不大的,可以考虑去除.但是有另一个因素需要考虑,即地面上的点云对抑制z轴漂移还是有很大作用的,因此,**在NDT_Mapping中进行配准时保留地面,但在NDT_Matching中配准时使用去除地面的点云**.


#### NDT_Matching
以下是NDT_Matching的结构图(同样不包含对imu和odom数据的融合,该数据融合部分会在另一篇中讨论)
<img src="https://user-images.githubusercontent.com/44689665/62829465-bc976000-bc30-11e9-8fba-3f4950f23de8.png" />

可以看出NDT_Matching的逻辑还是很简单清晰的,即不断的接收实时扫描到的点云,以及异步更新target_map,并使用NDT算法不断进行配准获取当前位置.

此处接收的点云需要进行去地面处理,因此我们将新建一个节点进行点云去除地面的操作,NDT_Matching将订阅该节点发布的去除地面后的点云.关于如何有效去除地面将在[TODO]()该篇中讨论.

此处的target_map是不断更新的,即使用一个单独的节点进行地图加载的操作,该节点订阅`/current_pose`,并按照一定的步长读取/更新匹配用target_map.关于如何有效的进行target_map管理将在[TODO]()该篇中讨论.

由于NDT_Matching和NDT_Mapping极为相似,主要更新target_map上不同,因此NDT_Matching的代码不在此处展开了.直接进行定位的视频展示.

<video src="https://youtu.be/PUCrF_T-Nvw" controls="controls"></video>  
或移步[YouTube](https://youtu.be/PUCrF_T-Nvw)


<!-- #### 修改NDT_GPU以适应TX2
我们的小车所用的计算平台是Jetson TX2,为了充分利用其GPU的性能,我们将AutoWare的NDT_GPU等package编译成动态链接库直接调用.(使用方法见[这里](https://github.com/zju-sclab/NDT-library))

但是此时是无法使用的,原因是TX2的BLOCK_SIZE要比GTX系列的GPU小的多,因此需要修改CUDA的BLOCK_SIZE.

在`ndt_gpu/include/common.h`中修改:
```c++
#define BLOCK_SIZE_X 1024
#define BLOCK_SIZE_X2 512
#define BLOCK_SIZE_X3 256
```
保存后重新编译.
*注意:调用动态链接库时,头文件`common.h`中这几个参数要与动态链接库编译时的参数一致.*

以下是我们改进后的小车的视频:
<video src="https://youtu.be/PUCrF_T-Nvw" controls="controls"></video>  
或移步[YouTube]() -->