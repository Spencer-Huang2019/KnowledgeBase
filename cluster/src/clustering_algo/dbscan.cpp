

void adaptive_dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>>& clusters)
{
	float alpha = 2;
	int k = 10;
	int min_pts = 10;
	float max_dist = 1.0;

	std::vector<int> labels(cloud->size(), -1);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<double> local_eps(cloud->size(), 0.0);
    std::vector<double> smooth_eps(cloud->size(), 0.0);
    double lambda = 0.5;

    // 计算每个点的自适应 eps
	double local_eps_min = 0.1;
    for (size_t i = 0; i < cloud->size(); ++i) 
	{
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        if (kdtree.nearestKSearch(cloud->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) 
		{
            double sum_dist = 0.0;
			int n = 0;
            for (float dist : pointNKNSquaredDistance) 
			{
				if (dist < max_dist) 
				{
					n += 1;
					sum_dist += std::sqrt(dist);
                }
            }
			double tmp = alpha * (sum_dist / n);
            local_eps[i] = std::max(tmp, local_eps_min);
        }
    }

	for (size_t i = 0; i < cloud->size(); ++i) 
	{
        std::vector<int> k_indices(k);
        std::vector<float> k_sqr_distances(k);
        double sum_eps = 0.0;

        if (kdtree.nearestKSearch(cloud->points[i], k, k_indices, k_sqr_distances) > 0) {
            for (int idx : k_indices) {
                sum_eps += local_eps[idx];
            }
            smooth_eps[i] = (1 - lambda) * local_eps[i] + lambda * (sum_eps / k);
        } else {
            smooth_eps[i] = local_eps[i];
        }
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    int points_num = cloud->size();
    uint16_t count = 1;
    std::vector<bool> visited(points_num, false);
    for (int i = 0; i < points_num; i++)
    {
        if (!visited[i])
        {
            visited[i] = true;
            // std::vector<int> same_cluster_idxes;
            // same_cluster_idxes.push_back(i);
            std::queue<int> q;
            q.push(i);
            while (q.size() > 0)
            {
                int q_size = q.size();
                for (int j = 0; j < q_size; j++)
                {
                    int to_be_search_idx = q.front();
                    std::vector<int> pointIdxRadiusSearch;
                    std::vector<float> pointRadiusSquaredDistance;
                    pcl::PointXYZ searchPoint;
                    searchPoint.x = cloud->points[to_be_search_idx].x;
                    searchPoint.y = cloud->points[to_be_search_idx].y;
                    searchPoint.z = cloud->points[to_be_search_idx].z;
                    if (tree.radiusSearch(searchPoint, local_eps[i], pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
                    {
                        for (auto find_idx : pointIdxRadiusSearch)
                        {
                            if (!visited[find_idx])
                            {
                                visited[find_idx] = true;
                                q.push(find_idx);
                                // same_cluster_idxes.push_back(find_idx);
                                labels[find_idx] = count;
                            }
                        }
                    }
                    q.pop();
                }
            }
            // clusters[count] = same_cluster_idxes;
            count++;
        }
    }

    /* #####       GPT: DBSCAN       ######
    
    int cluster_id = 0;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (labels[i] != -1) continue; // 已经分类
        std::vector<int> neighbors;
        std::vector<float> neighborDists;
        
        kdtree.radiusSearch(cloud->points[i], local_eps[i], neighbors, neighborDists);
        
        if (neighbors.size() < min_pts) {
            labels[i] = -1;  // 噪声
            continue;
        }

        labels[i] = cluster_id;
        for (size_t j = 0; j < neighbors.size(); ++j) {
            int n = neighbors[j];
            if (labels[n] == -1) labels[n] = cluster_id;  // 直接加入簇
        }
        cluster_id++;
    }

	std::vector<Eigen::Vector3f> cluster_centroids(cluster_id, Eigen::Vector3f::Zero());
	pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<int> cluster_sizes(cluster_id, 0);

    for (size_t i = 0; i < cloud->size(); ++i) 
	{
        if (labels[i] == -1) continue;
        int label = labels[i];
        cluster_centroids[label] += Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        cluster_sizes[label]++;
    }

    for (int i = 0; i < cluster_id; ++i) 
	{
        if (cluster_sizes[i] > 0) 
		{
            cluster_centroids[i] /= cluster_sizes[i];
			pcl::PointXYZ pt;
			pt.x = cluster_centroids[i](0);
			pt.y = cluster_centroids[i](1);
			pt.z = cluster_centroids[i](2);

			center_cloud_ptr->points.push_back(pt);
        }
    }

	center_cloud_ptr->height = cluster_id;
	center_cloud_ptr->width = 1;

    **合并簇**
	double merge_threshold = 1.0;
	std::vector<int> merged_labels(cluster_id, -1);
	int new_cluster_id = 0;
	for (size_t i = 0; i < center_cloud_ptr->size(); ++i) 
	{
		if (merged_labels[i] != -1) continue; // 已经分类
		merged_labels[i] = new_cluster_id;
		int k = 5;
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        if (kdtree.nearestKSearch(center_cloud_ptr->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) 
		{
            for (int m = 0; m < k; ++m) 
			{
				if (pointNKNSquaredDistance[m] < merge_threshold) 
				{
					merged_labels[pointIdxNKNSearch[m]] = new_cluster_id;
                }
            }
        }

		new_cluster_id += 1;
    }

    double merge_threshold = 10.0; // 合并阈值
    std::vector<int> merged_labels(cluster_id, -1);
    int new_cluster_id = 0;

    for (int i = 0; i < cluster_id; ++i) 
	{
        if (merged_labels[i] != -1) continue;
        merged_labels[i] = new_cluster_id;

        for (int j = i + 1; j < cluster_id; ++j) {
            double dist = (cluster_centroids[i] - cluster_centroids[j]).norm();
            if (dist < merge_threshold * std::min(smooth_eps[i], smooth_eps[j])) {
                merged_labels[j] = new_cluster_id;
            }
        }
        new_cluster_id++;
    }
        
    #####       GPT: DBSCAN      ###### */

	std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_map;
    for (size_t i = 0; i < labels.size(); ++i) {
        int label = labels[i];
        if (label == -1) continue;  // -1 代表噪声点，忽略

        if (cluster_map.find(label) == cluster_map.end()) {
            cluster_map[label] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        }
        cluster_map[label]->push_back(cloud->points[i]);
    }

	// std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_map;
    // for (size_t i = 0; i < cloud->size(); ++i) {
    //     int label = labels[i];
    //     if (label == -1) continue;
    //     label = merged_labels[label];  // 映射到合并后的簇
    //     if (cluster_map.find(label) == cluster_map.end()) {
    //         cluster_map[label] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //     }
    //     cluster_map[label]->points.push_back(cloud->points[i]);
    // }

    clusters.clear();
    for (auto it = cluster_map.begin(); it != cluster_map.end(); it++) 
	{
		auto cluster_pair = it->second;
		int point_num = cluster_pair->points.size();
		if (point_num < min_cluster_size_ || point_num >= max_cluster_size_)
		{
			continue;
		}

		for (auto pt:cluster_pair->points)
		{
			cluster_cloud.push_back(pt);
		}

        clusters.push_back(cluster_cloud);
    }
}