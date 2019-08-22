## 点云地图后处理和地图动态加载
本文包含以下内容：
　　1. 对[NDT_Mapping](https://zhuanlan.zhihu.com/p/77623762)建立的点云地图进行后处理，包括降采样和网格划分；
　　2. 通过`points_map_loader`，根据当前位置实时加载可视区域内的匹配地图；

### 点云地图后处理
#### 1. 降采样
**目的**
　　建立好的点云文件中，有很多点是重合的，需要通过采用合适的`voxel_leaf_size`以减小点云文件体积，便于传输和加载，通常降采样后体积可以降到原来的一半以下。同时由于NDT的特性，降采样后并不会影响最终的匹配定位效果；

**实现**
实现部分比较简单，即不断的读入点云文件，然后进行`voxel_filter`，然后保存到新文件中。  
由于我们的目的是为了滤除重合点，因此`leaf_size`不宜过大，通常为0.1~0.2即可。
```c++
// voxel_filter过滤
pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
voxel_grid_filter.setInputCloud(base_map);
voxel_grid_filter.filter(*map_filtered);
```
`voxel_filter`可以作为单独的模块进行调用，但一般都将其作为某个package中的一部分（函数），进行降采样操作。

#### 2. 点云地图网格划分
**目的**
　　1. 通过按照一定的网格大小对点云地图进行区域划分，可以实现动态加载`target_map`,理论上可以支持区域无限大的场景；
　　2. 实验测得，对地图进行区域划分有序化后，加载速度比原来快很多；

**实现**
　　一半来讲网格化通常是按照方形区域进行划分，因此需要指定划分区域的边长。
　　点云文件命名：`{grid_size}_{min_x}_{min_y}.pcd`
例如`30_-60_90.pcd`表示该点云区域范围为`[-60:-30, 90:120]`
具体的实现也比较简单，可以直接参考我的源代码。
最终效果：
<img src="https://user-images.githubusercontent.com/44689665/62829296-1564f980-bc2d-11e9-9e53-26673aa71aaf.png" />

<img src="https://user-images.githubusercontent.com/44689665/62829295-14cc6300-bc2d-11e9-95e7-74a0364ff3ad.png" />

**加载速度对比**：在我的渣机上，加载未进行网格化处理的点云需要大约5s，而经过了网格化处理后，其加载时间可以小于1s
以下是视频对比：

或移步[YouTube](https://youtu.be/doJXGnz4C0o)
#### 3. 点云地图动态加载points_map_loader
**目的**
根据车辆当前位置，实时加载当前区域地图，供`NDT_Matching`更新`target_map`。
<img src="https://user-images.githubusercontent.com/44689665/62844713-ba4b0980-bcf5-11e9-89bc-4f50123870e7.png"/>

**实现**
定义用于存储点云点及其信息的结构体
```c++
struct Area
{
	std::string filename;
	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;
	sensor_msgs::PointCloud2 points;
};
```

初始化节点和各参数
```c++
pnh_.param<double>("margin", MARGIN, 100.0);  // 需要加载的区域的范围，比如velodyne16的有效距离是100m，则加载范围最好应>120m
pnh_.param<double>("update_interval", UPDATE_INTERVAL, 1000.0); // ms，更新频率
pnh_.param<std::string>("map_dir", MAP_DIR, "/map/"); // 存储点云文件的目录
```

定义接收的topic
```c++
ros::Subscriber current_sub = nh_.subscribe("/ndt/current_pose", 10, publish_current_pcd);
ros::Subscriber initial_sub = nh_.subscribe("initialpose", 1, publish_dragged_pcd);
```

主要函数：
```c++
sensor_msgs::PointCloud2 create_pcd(const geometry_msgs::Point &p) // 载入p±MARGIN区域内的点云
{
	AreaList wanted_AreaList = create_wantedAreaList(p);  // std::vector<Area>, 存储目标点云文件信息，此时并没有进行点云的io加载
	AreaList tmp_AreaList;

	bool cache_update = false;
	for (Area &area : wanted_AreaList) // cached_AreaList记录缓存的点云及其信息：由于两次发布之间大多数点云是重复的，因此使用cached_AreaList进行缓存，以减少IO次数
	{
		int index = is_in_cachedAreaList(area);
		if (index == -1)
		{ // io
			Area in_area(area);
			pcl::io::loadPCDFile(MAP_DIR + area.filename, in_area.points);
			tmp_AreaList.push_back(in_area);
			cache_update = true;
		}
		else
		{
			Area in_area(cached_AreaList[index]);
			tmp_AreaList.push_back(in_area);
		}
	}
	if (cache_update)
	{
		cached_AreaList.clear();
		cached_AreaList = tmp_AreaList;  // 更新cached_AreaList
	}
	else
	{
		sensor_msgs::PointCloud2 msg_pcd;
		msg_pcd.width = 0;  // 若该次应发布的点云和上次的相同，则不发布
		return msg_pcd;
	}

	sensor_msgs::PointCloud2 msg_pcd;
	for (const Area &area : cached_AreaList)  // 构建需要更新发布的点云消息
	{
		if (msg_pcd.width == 0)
			msg_pcd = area.points;
		else
		{
			msg_pcd.width += area.points.width;
			msg_pcd.row_step += area.points.row_step;
			msg_pcd.data.insert(msg_pcd.data.end(), area.points.data.begin(), area.points.data.end());
		}
	}
	return msg_pcd;
}
```

*NDT_Matching*中的实现：
　　由于ndt加载target_map需要进行包括voxel等一系列操作，相当费时间，因此NDT_Matching中使用单独的线程进行`ndt target_map`的更新，新建ndt实例并进行初始化，然后在主线程中进行ndt实例的替换，以避免占用过多主线程的时间。

更新ndt的线程
```c++
void *thread_func(void *args)
{
    ros::NodeHandle nh_map;
    ros::CallbackQueue map_callback_queue;
    nh_map.setCallbackQueue(&map_callback_queue);

    sub_map_ = nh_map.subscribe<sensor_msgs::PointCloud2>("/points_map", 10, mapCB);
    ros::Rate ros_rate(10);
    while (nh_map.ok())
    {
        map_callback_queue.callAvailable(ros::WallDuration());
        ros_rate.sleep();
    }

    return nullptr;
}

void mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (model_pc_num_ == msg->width)
    {
        // suppose it is same map.
        return;
    }
    if (!pose_init_)
    {
        ROS_WARN("initial pose not set, cannot update target_map");
        return;
    }
    model_pc_num_ = msg->width;
    pcl::fromROSMsg(*msg, model_pc_);
    PointCloudT::Ptr target_map_ptr(new PointCloudT(model_pc_)); // 存储用于更新ndt的target_map

    // set NDT target
    if (param_method_type_ == METHOD_CUDA)
    {
#ifdef CUDA_FOUND
        std::shared_ptr<gpu::GNormalDistributionsTransform> new_anh_gpu_ndt_ptr = std::make_shared<gpu::GNormalDistributionsTransform>();
        new_anh_gpu_ndt_ptr->setResolution(param_ndt_resolution_);
        new_anh_gpu_ndt_ptr->setInputTarget(target_map_ptr);
        new_anh_gpu_ndt_ptr->setMaximumIterations(param_ndt_max_iterations_);
        new_anh_gpu_ndt_ptr->setStepSize(param_ndt_step_size_);
        new_anh_gpu_ndt_ptr->setTransformationEpsilon(param_ndt_epsilon_);

        PointCloudT::Ptr dummy_scan_ptr(new PointCloudT());
        PointT dummy_point;
        dummy_scan_ptr->push_back(dummy_point); 
        new_anh_gpu_ndt_ptr->setInputSource(dummy_scan_ptr);
        new_anh_gpu_ndt_ptr->align(Eigen::Matrix4f::Identity());

        pthread_mutex_lock(&mutex);
        anh_gpu_ndt_ptr = new_anh_gpu_ndt_ptr;  // 此步将更新主线程中的ndt实例
        pthread_mutex_unlock(&mutex);

#else
        ROS_ERROR("param method_type set to cuda, but cuda_found not defined!");
#endif
    }
    map_init_ = true;
}
```
(由于所做项目是在特定园区中进行，因此没有对向网络请求地图的部分做实现。)
最终效果（为了展示，设置MARGIN<100.0）:


或移步[YouTube](https://youtu.be/PUCrF_T-Nvw)