# Octree 八叉树原理
Octree 是一种用于三维空间划分的数据结构，它将一个三维空间递归地划分成 8 个子空间。

# 构建过程
1. 初始化：定义一个根节点，其范围覆盖整个三维空间（只能是 cube 大小）  
**如果点云不是 cube，在创建 octree 过程中，对于没有点的且 voxel size 小于 resolution 的体素可以不用分裂**  
2. 插入数据：有新的点插入时，需要检查该点位于根节点的哪个子区域。如果该子区域已经是叶子节点且还有空间容纳新点，则将点插入该叶子节点；否则该叶子节点分裂成八个子节点，并将该点插入到对应的子节点中，可能还需要进一步递归分裂子节点。  
**叶子节点分布分裂都可以根据需要设置不同的规则，比如说，当 voxel size 还比较大且体素内点多，那么可以进行进一步分裂，但如果 voxel size 已经达到预设的最小值，那么就不应该分裂了**  
3. 递归划分：重复上述插入过程，直到所有数据点都被插入到合适叶子节点中。

# index 计算
查找一个 point 在哪个体素，可以计算其所在子节点的 index。这句话吧，有点不好理解，之前我一直以为计算的 index 是一个全局的index，类似于二维数组的一维 index 计算那种。但实际上吧，并不是，它这是递归计算的，index 只可能是 0 ~ 7.   
也就是说，如果给定一颗 octree 的根节点和一个 point，第一次计算的时候会先算出 point 是属于根节点的 8 个子节点中的哪个，比如第 index1 个子节点；只要这个 index1 节点不是叶子节点，那么就继续在 index1 中计算这个 point 属于 index1 节点的哪个子节点，。。。如此重复下去，一直到叶子节点为止。  

假设当前节点的边界框为 (min_x, min_y, min_z) 和 (max_x, max_y, max_z)，其中心点坐标为  
mid_x = (min_x + max_x) / 2  
mid_y = (min_y + max_y) / 2  
mid_z = (min_z + max_z) / 2  

| 子区域编号 | x 范围 | y 范围 | z 范围 |  
| -------- | --------- | -------- | ------- | 
| 0 | [min_x, mid_x) | [min_y, mid_y) | [min_z, mid_z) |  
| 1 | [mid_x, max_x] | [min_y, mid_y) | [min_z, mid_z) |  
| 2 | [min_x, mid_x) | [mid_y, max_y] | [min_z, mid_z) |  
| 3 | [mid_x, max_x] | [mid_y, max_y] | [min_z, mid_z) |  
| 4 | [min_x, mid_x) | [min_y, mid_y) | [mid_z, max_z] |  
| 5 | [mid_x, max_x] | [min_y, mid_y) | [mid_z, max_z] |  
| 6 | [min_x, mid_x) | [mid_y, max_y] | [mid_z, max_z] |  
| 7 | [mid_x, max_x] | [mid_y, max_y] | [mid_z, max_z] |  

对于 x 维度来说，从 mid_x 的左边或右边，在 [min_x, mid_x) 为 true，在 [mid_x, max_x] 为 false。  

int index = 0；
[min_x, mid_x) 这个范围分别占了 ID 0,2,4,6，二进制表示分别为 0000，0010,0100,0110  右边第 1 位为 0  
[mid_x, max_x] 这个范围分别占了 ID 1,3,4,7，二进制表示分别为 0001，0011,0101,0111  右边第 1 位为 1  
所以，当 x < mid_x 时，index 最后一位是 0；当 x >= mid_x 时，index 最后一位是 1，即 index |= 1

[min_y, mid_y) 这个范围分别占了 ID 0,1,4,5，二进制表示分别为 0000，0001,0100,0101  右边第 2 位为 0  
[mid_y, max_y] 这个范围分别占了 ID 2,3,6,7，二进制表示分别为 0010，0011,0110,0111  右边第 2 位为 1  
所以，当 y < mid_y 时，index 倒数第二位是 0；当 y >= mid_y 时，index 倒数第二位是 1，即 index |= 2  

[min_z, mid_z) 这个范围分别占了 ID 0,1,2,3，二进制表示分别为 0000，0001,0010,0011  右边第 3 位为 0
[mid_z, max_z] 这个范围分别占了 ID 4,5,6,7，二进制表示分别为 0100，0101,0110,0111  右边第 3 位为 1  
所以，当 z < mid_z 时，index 倒数第三位是 0；当 z >= mid_z 时，index 倒数第三位是 1，即 index |= 4  

**疑问**：为什么 x 只能决定最后一位，y 只能决定倒数第 2 位，z 只能决定倒数第 3 位？  
| x | y | z | 二进制 | 十进制（二进制逆转） |
| -- | -- | -- | ----- | ---------------- |
| 0 | 0 | 0 | 000 |  0 |  
| 1 | 0 | 0 | 100 |  1 |  
| 0 | 1 | 0 | 010 |  2 |  
| 1 | 1 | 0 | 110 |  3 |  
| 0 | 0 | 1 | 001 |  4 |  
| 1 | 0 | 1 | 101 |  5 |  
| 0 | 1 | 1 | 011 |  6 |  
| 1 | 1 | 1 | 111 |  7 |  
每个维度都有两种情况，分别用二进制的 0， 1 表示，组合成的二进制进行逆转，转到十进制正好就是 0 ~ 7. 所以 x 的 0、1 对应的最低位，y 对应的中间位，z 对应的最高位。

# 应用场景
## 碰撞检测


## octree 聚类
最近正做一项关于车端激光雷达点云聚类的任务。对于激光雷达来说，远处的点之间的距离会越来越大，导致随着距离的增加，点云渐渐稀疏。稍远点的同一个目标经常没法聚成一个簇，这就导致后边的跟踪效果很差。原来的算法是聚类之前用 PCL 的 voxeleaf 做了降采样，然后用 PCL 的 DBSCAN 和 KDTree 搜索来完成聚类的。降采样的 voxel size 是固定的 0.15，这就导致中间某一个区域本来就不多的点云更加稀疏了，更远处的位置，通常一个体素要么没点要么一个点，影响倒是不大。而 PCL 的 DBSCAN 聚类呢，tolerance 是一个固定值，也就是说为了保证近处不把不同目标聚成一个簇，就要忍受远程的同一个目标被聚成多个类。  

后来好不容易找了个自适应的 DBSCAN 算法来用，结果这聚类时间都 90 ms了。。。实在不能接受，点云帧率是 10 Hz诶。

想起之前看碰撞检测的时候知道一个不太熟悉的东西 octree，这个效率还挺高的，所以这就上手试试了。不过第一天搞这个，从可视化看效果还可以的，但是！！！耗时太太太长了。。。  
主要吧，第一次用 octree，确实不太熟，还得好好捋一下，然后再找合适的方案优化一下算法，感觉应该是有希望的。

### 函数功能描述
1. octree 插入点时，把 point 存储到对应的 node 中  
2. 查找 point 所在的节点的子节点 index，然后进行插入或分裂  
3. 每个体素需要一个更为方便的 index（全局的），用于制作 index和 node 映射  
4. 通过一个节点获取其邻近的节点  
5. 通过体素进行聚类，需要 tolerance 和 邻近节点 以及 visited 标识，得到每个簇的 indices（全局的）    
6. 通过第 3 步的 map 找到一个簇对应的多个体素，将 points 取出到 cloud 里，后续就可以用于生成包围框了。  
****