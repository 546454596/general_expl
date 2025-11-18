# Polygon Draw Plugin for RViz

这是一个RViz插件，用于手动绘制多边形并发布多边形消息。插件包含三个主要组件：工具(Tool)、面板(Panel)和显示器(Display)。

## 功能特性

- **手动绘制多边形**: 通过鼠标点击在RViz中绘制多边形
- **实时可视化**: 实时显示正在绘制的点和线段
- **保存和发布**: 将绘制的多边形保存并发布为ROS消息
- **撤销功能**: 支持清除最后一个点和清除所有点
- **可配置显示**: 可自定义线条颜色、点的颜色和大小

## 组件说明

### 1. PolygonDrawTool (绘制工具)
- 激活后可以通过鼠标左键点击添加多边形顶点
- 鼠标右键完成多边形绘制并发布
- 快捷键: 'P'

### 2. PolygonDrawPanel (控制面板)
- 提供按钮控制多边形绘制操作
- **保存并发布多边形**: 发布当前多边形并清除所有点
- **删除最后一个点**: 撤销最后添加的点
- **清除所有点**: 清除当前绘制的所有点
- **发布当前多边形**: 发布当前多边形但不清除点
- 可配置发布话题名称

### 3. PolygonDrawDisplay (显示器)
- 订阅并显示多边形消息
- 可配置线条和点的颜色、大小
- 支持开关点的显示

## 安装和编译

1. 将此插件包放入你的catkin工作空间的src目录：
```bash
cd ~/catkin_ws/src
# 复制插件文件到这里
```

2. 创建必要的目录结构：
```bash
mkdir -p polygon_draw_plugin/include
mkdir -p polygon_draw_plugin/src
```

3. 将头文件放入include目录，源文件放入src目录

4. 编译插件：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 启动插件

1. 启动RViz：
```bash
rosrun rviz rviz
```

2. 添加插件组件：
   - **添加工具**: 在工具栏右键 → Add Tool → polygon_draw_plugin/PolygonDrawTool
   - **添加面板**: Panels → Add New Panel → polygon_draw_plugin/PolygonDrawPanel
   - **添加显示**: Add → polygon_draw_plugin/PolygonDrawDisplay

### 绘制多边形

1. 选择PolygonDrawTool工具（或按快捷键'P'）
2. 在RViz视图中左键点击添加多边形顶点
3. 右键点击完成多边形绘制并自动发布
4. 使用控制面板的按钮进行更精确的控制

### 控制面板操作

- **Save & Publish Polygon**: 保存当前多边形，发布消息，并清除所有点
- **Remove Last Point**: 删除最后添加的点
- **Clear All Points**: 清除所有已添加的点
- **Publish Current Polygon**: 发布当前多边形但保留点（用于预览）
- **Topic**: 设置发布多边形消息的话题名称

## 发布的消息类型

### geometry_msgs/PolygonStamped
```bash
rostopic echo /polygon_draw
```
发布的多边形消息包含：
- header: 时间戳和坐标系
- polygon: 多边形顶点列表

### visualization_msgs/MarkerArray
```bash
rostopic echo /polygon_draw_markers
```
发布的可视化标记包含：
- 多边形边线（LINE_STRIP）
- 多边形顶点（SPHERE）

## 配置参数

### Display 显示器参数
- **Topic**: 订阅的多边形话题
- **Line Color**: 多边形边线颜色
- **Point Color**: 顶点颜色
- **Line Width**: 边线宽度
- **Point Size**: 顶点大小
- **Show Points**: 是否显示顶点

## 示例使用场景

1. **路径规划**: 绘制机器人的允许活动区域
2. **障碍物定义**: 标记环境中的障碍物区域
3. **工作区域设定**: 定义机器人的工作边界
4. **地图标注**: 在地图上标记特殊区域

## 故障排除

### 常见问题

1. **插件未显示在RViz中**
   - 确保已正确编译并source了工作空间
   - 检查plugin.xml路径是否正确

2. **无法添加点**
   - 确保选择了PolygonDrawTool
   - 检查是否有有效的ground plane用于投影

3. **消息未发布**
   - 检查话题名称是否正确
   - 确保至少有3个点才能构成多边形

4. **可视化不显示**
   - 检查坐标系设置
   - 确保Display已启用

### 调试命令

```bash
# 查看发布的话题
rostopic list | grep polygon

# 查看多边形消息
rostopic echo /polygon_draw

# 查看可视化标记
rostopic echo /polygon_draw_markers

# 检查插件是否加载
rospack plugins --attribs=plugin rviz
```

## 自定义扩展

### 添加新功能
1. 修改头文件添加新的方法声明
2. 在源文件中实现新功能
3. 在面板中添加对应的UI控件
4. 重新编译插件

### 修改默认参数
编辑源文件中的默认值：
- 默认话题名称
- 默认颜色设置
- 默认尺寸参数

## 许可证

此插件使用BSD许可证发布。

## 贡献

欢迎提交bug报告和功能请求。如需贡献代码，请提交pull request。
