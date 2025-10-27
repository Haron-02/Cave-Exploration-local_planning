Done (加入了是否更新的检测) surface frontier cluster出视野后还会消失 

Done 计算法向量

Done 初始化时选择obstacle

Done 简单的直线采样，考虑了雷达的俯角

Done 采样视点

Done 有些vp就算飞近了也消不掉，要想办法忽略这些vp（两个条件判断 isSurfaceFrontierCovered 函数）

Done planning改成3D的

Done 键盘控制加上高度控制

Done 雷达改成上下两个

Done 合并小的聚类

Done 把 max_gain_viewpoint 与对应的 cluster 在 Rviz 中连起来,用来分析

Done 测试lkh解CTSP

Done 设计聚类策略(暂时用可见性聚类+2端点表示)

Done 搜路并行化

Done 有bug,会出现到原点的路径

Done 法向量计算并行化

Done 将ego planner的轨迹信息发给探索算法

Done 视点聚类的消除

Done 路径搜索要进行拉直

Done viewpoint的publish有问题

Done max_gain_viewpoint 为什么和global path不对应？ (聚类后肯定不对应)

Done 洞穴场景中roadmap生成不了（调整参数）

Done 同步两个地图的膨胀方法

Done viewpointRefine时要看点可不可达

Done 莫名其妙会暴死

TODO

把unknown的格子作为frontier？

有些视点的forntier根本消不掉，要强制消

视点还是会在障碍物里

add frontier info 后
Find path in grid map failed 会卡死找不到路


angle_thres  也会影响角度聚类

FUEL会卡障碍物里

把refine和direction_cost都关了

ego 远目标点加速很慢，把目标点距离设近一些

ego-plnner的感知范围限制为15m

在狭窄区域还是容易撞墙

长距离下，在grid中搜索当前位置到vp的路径会失败


有些viewpoint在网格上会搜索失败

traj fully executed 似乎会出bug

viewpointRefine会die掉

TSP时间记录

是否要在远距离运动的时候只进行路径规划,

EXEC_TRAJ 是否要主动cover frontier

检查各种东西发布所需的时间，尤其是occupancy_map

实时统计各种数据

ego-planner还是会减速

自动算的sequence cost好像不太对

viewpoint cluster的计算还存在一些问题

viewpoint refine中更加考虑运动方向变化惩罚

拆分覆盖率很低的frontier cluster

视点raycast的时候，unknown也认为可视

添加45度方向上的视点惩罚（似乎有些效果）

在第一个节点路径上考虑路径上的探索增益

仔细分析路径变化，减少左右摇摆

惩罚特别长的cur to viewpoint路径

FUEL的frontier要稳定很多,而且不会留下一片在那

不要生成近的视点(对近的视点,加入了惩罚)

上升下降仍有卡顿

ego-planner有问题

考虑根据采样情况将forntier聚类继续拆分

全局规划的触发还有问题，有时还有卡顿

FUEL的代码中，似乎在所有cost中都加了方向变化惩罚

洞穴场景，降低速度方向增益

有些散的frontier聚不起来

全局规划不是规划 视点的访问顺序，而是规划frontier聚类的访问顺序。 局部优化就是选择前几个聚类，采样视点，并选择。

考虑前n个cluster对应的frontier，选一个新的视点作为下一个目标点

提高封闭frontier区域的增益

frontier 聚类的变化会导致飞机上下摆动（减小分割聚类的阈值？）

roadmap路径搜索优化

z轴惩罚优化

viewpoint聚类时，vp之间有障碍物阻挡就不聚类

实际执行回调函数的频率过低

ego规划出的路径会突然往回一下

左右都有frontier时的左右摇摆问题

updateObstacleClusters处突然die了

ego planner会卡墙

月球洞穴换成能够探索的

FSM优化,现在还是会卡在目标点

测试柱子图

设计椭圆聚类

造一个洞穴场景(先用月球洞穴)

修复frontier聚类邻居的计算方式


仔细看看Star Searcher的tsp计算

surface frontier 引导

地图结构：FAO

基于法向量的frontier聚类

基于体积均匀的viewpoint采样

基于邻接关系的viewpoint聚类（相比基于可见性的聚类，能生成更大的聚类，表示一片更大的区域）

DTSP

加入障碍物距离惩罚的Ego-Planner，看看能不能实现

TSP的结果有些问题？

会出现障碍物后面的frontier，需要去除

优化roadmap的参数，优化roadmap的搜索，并行搜索

算法向量时，最近邻点的半径会很影响计算速度

frontier cluster的分解换成fuel那样的？

TSP算不动，将远处的frontier聚类再聚类？

Roadmap算的路径不准

先解决TSP第一个点跳来跳去的问题

再解决轨迹执行完后的卡顿问题


太近的点会导致飞机一顿一顿的，如何处理？（frontier消掉后，旁边的frontier聚类会采样出很近的点）人为改一下采样的方向？

TSP应该逐渐忽略较后视点的成本？视点较多时，为了路径总体成本低，第一个视点可能离无人机很远

目标点扫描完毕的全局规划的触发有问题

仔细研究一下全局规划和局部规划的时序问题

TSP中算vp之间的cost太慢了

法向量矫正那里，向无人机位置进行矫正

当TSP维度小于3时，做对应处理

加多一个雷达

建新的简单点的地图

还是先用回传统的运动方向一致性cost？

把每个聚类的增益, 覆盖率显示出来

测试fast-planner和ego-planner的性能，测好再设计局部规划的策略

设计local refine策略，全局规划时先不算覆盖点，local refine时再算？

基于法向量来聚类，使用覆盖路径来选择视点，防止聚类看不完

如何确保已经飞到的vp不再出现？现在会原地打转

优化局部地图中的搜路径,防止在map3d中找不到vp

如何找聚类的代表点？用vp来代表，先找好覆盖vp，再TSP

采样方式还需要优化，先TSP选择聚类再对第一个聚类仔细采样？




优化增益的计算方式

看一下yaw角规划的流程，以及末速度的规划，优化一下

看专注于表面的三维重建的工作，看看方法，和表面联通性的设定

还是先尝试把大的frontier聚类切分？单个frontier还是太难处理了 根据法向量进行聚类？
可以定量测一下，普通聚类下，采点能得到的最大覆盖率；根据法向量进行聚类下，采点能得到的最大覆盖率
以及聚出来的类的大小（也许可作为一个创新点？）

用ikdtree存vp，防止vp之间距离太近

如何确保已经看到的vp不再出现？

加入z轴变化惩罚

merge还是需要的？？（先不管）

聚类后，有些obstacle cell会丢失（可以先不管）

采样时可以考虑lidar模型？尽量不要采出无人机附近的点，因为这些点意义不大（如果这个点可以看到frontier，那它也基本上可以在现在位置看到，因此应该已经被消掉了）

感觉还是需要锥形采样




