#include "convex_clustering_code/convex_clustering_code.h"


bool CVC::initialize()
{ 
    if (ros::ok())
    {
        // Initialize Subscriber for input Pointcloud2 6
        input_points = nh_.subscribe("input_pointcloud", 1, &CVC::cloudCallback, this);
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("output_marker", 10);
        time_init = ros::Time::now().toSec(); // for real world test
        return true;
    }
    else
    {
        return false;
    }
}

// Subscribe Pointcloud from velodyne Simulator
void CVC::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_point(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cluster_point); // sensor_msgs::PointCloud to pcl::Pointcloud

	std::cout << "cluster_point " << cluster_point->points.size() << std::endl;
	std::vector<PointAPR> capr;
	calculateAPR(*cluster_point, capr);

	std::unordered_map<int, Voxel> hash_table;
	build_hash_table(capr, hash_table);

	std::vector<int> cluster_indices;	
	cluster_indices = cluster(hash_table, capr);
	std::vector<int> cluster_id;
	most_frequent_value(cluster_indices, cluster_id);
    for (int i = 0; i < cluster_id.size(); i++) {
        std::cout << cluster_id[i] << ' ';
    int max = *max_element(cluster_indices.begin(), cluster_indices.end());
    std::vector<std::vector<pcl::PointXYZ>> cluster_results(max+1);
    std::vector<pcl::PointXYZ>* centroids(new std::vector<pcl::PointXYZ>);
    cluster_result(cluster_indices,cluster_results,*cluster_point,*centroids);
    // Publish Marker
    PublishMarker(*centroids);
    }
}

void CVC::spinNode()
{
    ros::Duration(1/frequency).sleep(); // Synchronize to lidar frequency 
    ros::spinOnce();
}

float CVC::euc_dist(Vector3d P1, Vector3d P2)
{
    return std::sqrt((P1(0)-P2(0))*(P1(0)-P2(0)) + (P1(1)-P2(1))*(P1(1)-P2(1)) + (P1(2)-P2(2))*(P1(2)-P2(2)));
}

bool compare_cluster(std::pair<int,int> a,std::pair<int,int> b){
    return a.second>b.second; // 두번째 항 비교
} // upper sort

float Polar_angle_cal(float x, float y){
	float temp_tangle = 0;
	if(x== 0 && y ==0){
	     temp_tangle = 0;
	}else if(y>=0){
	     temp_tangle = (float)atan2(y,x);
	}else if(y<0){
	     temp_tangle = (float)atan2(y,x)+2*M_PI;
	}
	return temp_tangle; // Calculate by using arctan
}


// Cloud_IN = Input PointCloud, vapr = Pointcloud에 대한 APR를 담은 vector
// APR : azimuth (theta), Polar angle (pi), range (rho)
void CVC::calculateAPR(const pcl::PointCloud<pcl::PointXYZ>& cloud_IN, std::vector<PointAPR>& vapr){ 
    for (int i =0; i<cloud_IN.points.size(); ++i){
        PointAPR par;
        // Convert to Spherical
        par.polar_angle = Polar_angle_cal(cloud_IN.points[i].x, cloud_IN.points[i].y);
        par.range = sqrt(cloud_IN.points[i].x*cloud_IN.points[i].x + cloud_IN.points[i].y*cloud_IN.points[i].y);
        par.azimuth =(float) atan2(cloud_IN.points[i].z, par.range);
        if(par.range < min_range_){ // (min/max) Range Update
            min_range_ = par.range;
        }
        if(par.range > max_range_){
            max_range_ = par.range;
        }
        vapr.push_back(par); // vector_apr ?
    }
    // Param for Hash Table, 전체 개수?
	length_ = int((max_range_ - min_range_)/deltaR_)+1;
	width_  = round(360/deltaP_); // 가로
	height_ = int(((max_azimuth_ - min_azimuth_)*180/M_PI)/deltaA_)+1; // 세로
}


void CVC::build_hash_table(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel> &map_out){
    std::vector<int> ri; // range_index vector
    std::vector<int> pi; // polar_index vector
    std::vector<int> ai; // azimuth_index vector
    // delta is Unit size parameter
    for(int i =0; i< vapr.size(); ++i){
        int azimuth_index = int(((vapr[i].azimuth-min_azimuth_)*180/M_PI)/deltaA_); // Divide by delta_theta to find the index
        int polar_index = int(vapr[i].polar_angle*180/M_PI/deltaP_); // Divide by delta_pi to find the index
        int range_index = int((vapr[i].range - min_range_)/deltaR_); // Divide by delta_rho to find the index
        
        int voxel_index = (polar_index*(length_)+range_index)+azimuth_index*(length_)*(width_); 
        ri.push_back(range_index);
        pi.push_back(polar_index);
        ai.push_back(azimuth_index);
        std::unordered_map<int, Voxel>::iterator it_find;
        it_find = map_out.find(voxel_index);
        if (it_find != map_out.end()){
            it_find->second.index.push_back(i);  // 미리 찾은 voxel에 지금 보고 있는 pointcloud의 index를 추가
        }
        else {
            Voxel vox;
            vox.haspoint = true;
            vox.index.push_back(i); 
            vox.index.swap(vox.index);
            map_out.insert(std::make_pair(voxel_index,vox)); // vox의 index와 voxel_index의 차이는?
            // i는 들어온 pointcloud의 개수, voxel_index는 이 pointcloud의 점이 어느 voxel에 있는지 저장
        }
    }
    auto maxPosition = max_element(ai.begin(), ai.end());
    auto maxPosition1 = max_element(ri.begin(), ri.end());
    auto maxPosition2 = max_element(pi.begin(), pi.end());
}

void CVC::find_neighbors(int polar, int range, int azimuth, std::vector<int>& neighborindex) {
	for (int z = azimuth - 1; z <= azimuth + 1; z++) {
		if (z < 0 || z > (height_ -1)) {
			continue;
		}

		for (int y = range - 1; y <= range + 1; y++) {
			if (y < 0 || y > (length_-1)) {
				continue;
			}

			for (int x = polar - 1; x <= polar + 1; x++) {
                    int px = x;
				if (x < 0 ) {
					px=width_-1;
				}
                    if (x > (width_-1)) {
                         px=0;
                    }
				neighborindex.push_back((px*(length_)+y)+z*(length_)*(width_)); // Point가 속한 Voxel의 인접한 Voxel_index를 push_backs
            }
        }
    }
}


bool CVC::most_frequent_value(std::vector<int> values, std::vector<int> &cluster_index) {
	std::unordered_map<int, int> histcounts;
	for (int i = 0; i < values.size(); i++) {
		if (histcounts.find(values[i]) == histcounts.end()) {
			histcounts[values[i]] = 1;
		}
		else {
			histcounts[values[i]] += 1;
		}
	}
	int max = 0, maxi;
	std::vector<std::pair<int, int>> tr(histcounts.begin(), histcounts.end());
    sort(tr.begin(),tr.end(),compare_cluster); // 위의 방법으로 정렬
    for(int i = 0 ; i< tr.size(); ++i){
        if (tr[i].second > 10){
        cluster_index.push_back(tr[i].first);
        }
    }
	return true;
}



std::vector<pcl::PointXYZ> CVC::getCentroid( std::vector<std::vector<pcl::PointXYZ>> cluster_results)
{
    // To separate each cluster out of the vector<PointIndices> we have to 
    // iterate through cluster_indices, create a new PointCloud for each 
    // entry and write all points of the current cluster in the PointCloud. 
    
    
    // std::vector<int>::const_iterator pit;
    std::cout << "ok15 " << std::endl;
    std::vector<pcl::PointXYZ> clusterCentroids;
    clusterCentroids.reserve(cluster_results.size());
    std::cout << "ok16: " << cluster_results.size() << std::endl;
    for (int it = 0 ; it < cluster_results.size(); ++it) 
    {
        Vector3d Pi; 
        Vector3d Pj; 
        Vector3d Pk; 

        Vector3d Vij; // vector between Pi, Pj
        Vector3d Po(0,0,0); // origin vector

        // 1. get Pi, Pj (First, Second Point)
        // get Vij norm and select vector that maximizes norm
        float dist_max = -1;
        for(int i=0; i < cluster_results[it].size(); i++)
        {
            for(int j=i+1; j < cluster_results[it].size(); j++)
            {
                float dist;     

                Vector3d P1; 
                Vector3d P2;     
                // pit = i;              
                P1(0) = cluster_results[it][i].x;
                P1(1) = cluster_results[it][i].y;
                P1(2) = cluster_results[it][i].z;
                // pit = j;
                P2(0) = cluster_results[it][j].x;
                P2(1) = cluster_results[it][j].y;
                P2(2) = cluster_results[it][j].z;

                dist = euc_dist(P1, P2);
                if (dist > dist_max) 
                {
                    Pi = P1; 
                    Pj = P2;
                    Vij(0) = (P2(1)-P1(1))/(P2(0)-P1(0));
                    Vij(1) = -1;
                    Vij(2) = Vij(0)*(-P1(0))+P1(1);
                    dist_max = dist;
                }
            }
        }

        // 2. get Pk (third Point)
        dist_max = -1; // initialize dist_max 
        for(int k=0; k < cluster_results[it].size(); k++)
        {
            float dist;

            Vector3d P3;
            // pit = k;
            P3(0) = cluster_results[it][k].x;
            P3(1) = cluster_results[it][k].y;
            P3(2) = cluster_results[it][k].z;

            // Euclidean distance between point and line
            dist = std::abs(Vij(0)*P3(0) + Vij(1)*P3(1) + Vij(2))/std::sqrt(Vij(0)*Vij(0) + Vij(1)*Vij(1));
            if (dist > dist_max)
            {
                if(Pj==P3 || Pi==P3)
                {
                    continue;
                }
                Pk = P3;
                dist_max = dist;
            }
        }
        
        // 3. circumcenter coordinates from cross and dot products
        float A = Pj(0) - Pi(0);
        float B = Pj(1) - Pi(1);
        float C = Pk(0) - Pi(0);
        float D = Pk(1) - Pi(1);
        float E = A * (Pi(0) + Pj(0)) + B * (Pi(1) + Pj(1));
        float F = C * (Pi(0) + Pk(0)) + D * (Pi(1) + Pk(1));
        float G = 2.0 * (A * (Pk(1) - Pj(1)) - B * (Pk(0) - Pj(0)));

        pcl::PointXYZ centroid; 
        if(G==0)
        {
            centroid.x = Pi(0);
            centroid.y = Pi(1);
            centroid.z = 0.0;
        }

        else
        {
            centroid.x = (D * E - B * F) / G;
            centroid.y = (A * F - C * E) / G;
            centroid.z = 0.0;
        }
        clusterCentroids.push_back(centroid);

        // set radius for publishObstacles
        Vector3d V_centroid(centroid.x, centroid.y, centroid.z);
        float obstacle_radius = euc_dist(V_centroid, Pj);
        if (obstacle_radius > 0.3) // obstacle radius constraint
        {
            obstacle_radius = 0.3;
        }
    }
    return clusterCentroids;
}




bool CVC::cluster_result(std::vector<int> cluster_indices, std::vector<std::vector<pcl::PointXYZ>> &cluster_results, 
                        const pcl::PointCloud<pcl::PointXYZ>& cloud_IN, std::vector<pcl::PointXYZ> &centroids) {
    for (int i = 0 ; i < cluster_indices.size() ; i++){
        if (cluster_indices[i] != -1) {
            // std::cout << "cnt :" << i << ", voxel_index :" << cluster_indices[i] <<std::endl;
            cluster_results[cluster_indices[i]].push_back(cloud_IN.points[i]);
        }
    }
    centroids = getCentroid(cluster_results);

    // for (int j = 0 ; j < cluster_results.size() ; j++){
        

        // float centroid_x = 0;
        // float centroid_y = 0;
        // float centroid_z = 0;
        // for (int k = 0 ; k < cluster_results[j].size() ; k++){
        //     centroid_x += cluster_results[j][k].x;
        //     centroid_y += cluster_results[j][k].y;
        //     centroid_z += cluster_results[j][k].z;
        // }
        // std::vector<float> centroid;
        // centroid.push_back(centroid_x/cluster_results[j].size());
        // centroid.push_back(centroid_y/cluster_results[j].size());
        // centroid.push_back(centroid_z/cluster_results[j].size());
        // centroids.push_back(centroid);
    // }
	return true;
}







std::vector<int> CVC::cluster(std::unordered_map<int, Voxel> &map_in,const std::vector<PointAPR>& vapr){
    int current_cluster = 0;
    std::cout << " ------ Doing CVC cluster ------- " << std::endl;
    std::vector<int> cluster_indices = std::vector<int>(vapr.size(), -1);
    for(int i = 0; i< vapr.size(); ++i){

        if (cluster_indices[i] != -1)
			continue;
        int azimuth_index = int((vapr[i].azimuth - min_azimuth_)*180/M_PI/deltaA_);
        int polar_index   = int(vapr[i].polar_angle*180/M_PI/deltaP_);
        int range_index   = int((vapr[i].range-min_range_)/deltaR_);
        int voxel_index   = (polar_index*(length_)+range_index)+azimuth_index*(length_)*(width_);
        
        std::unordered_map<int, Voxel>::iterator it_find;
        std::unordered_map<int, Voxel>::iterator it_find2;

        it_find = map_in.find(voxel_index);
        std::vector<int> neightbors;

        if (it_find != map_in.end()){

            std::vector<int> neighborid;
            find_neighbors(polar_index, range_index, azimuth_index, neighborid);
            for (int k =0; k<neighborid.size(); ++k){
                    
                it_find2 = map_in.find(neighborid[k]);

                if (it_find2 != map_in.end()){

                    for(int j =0 ; j<it_find2->second.index.size(); ++j){
                    neightbors.push_back(it_find2->second.index[j]);
                    }
                }
            }
        }
    
        neightbors.swap(neightbors);

        if(neightbors.size()>0){
            for(int j =0 ; j<neightbors.size(); ++j){
                    int oc = cluster_indices[i] ;
                    int nc = cluster_indices[neightbors[j]];
                if (oc != -1 && nc != -1) {
                    if (oc != nc){
                            mergeClusters(cluster_indices, oc, nc);
                    }
                }
                else {
                    if (nc != -1) {
                            cluster_indices[i] = nc;
                    }
                    else {
                            if (oc != -1) {
                                cluster_indices[neightbors[j]] = oc;
                            }
                    }
                }
            }
        }
        if (cluster_indices[i] == -1) {
            current_cluster++;
            cluster_indices[i] = current_cluster;
            for(int s = 0 ; s < neightbors.size(); ++s){             
                cluster_indices[neightbors[s]] = current_cluster;
            }
        }
    }
	return cluster_indices;
}

void CVC::mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2) {
	for (int i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}


void CVC::PublishMarker(std::vector<pcl::PointXYZ> &centroids){ // Rviz에 Cluster ID 별로 visualization
    Point p; 
    nh_.getParam("input_color_green", RVIZ_COLOR_GREEN);
    nh_.getParam("input_color_red", RVIZ_COLOR_RED);
    nh_.getParam("rviz_base", VELODYNE_BASE);
    cout << "velodyne_base is :" << VELODYNE_BASE << endl;
    std::vector<Point> vec_point;
    for (int i = 1 ; i < centroids.size() ; i++){
        p.x = centroids[i].x;
        p.y = centroids[i].y;
        p.z = centroids[i].z;
        vec_point.push_back(p);
    }
    std::cout << "num of Centroids : " << vec_point.size() << std::endl;
    visualization_msgs::MarkerArray node_arr;
    int cnt = 0;
    for (size_t i = 0; i < vec_point.size(); i++){
        Point o_node = vec_point[i];

        visualization_msgs::Marker node; 
        node.header.frame_id = VELODYNE_BASE; 
        // map frame 기준 
        node.header.stamp = ros::Time::now(); 
        node.type = visualization_msgs::Marker::SPHERE; 
        node.id = i+1; 
        node.action = visualization_msgs::Marker::ADD; 
        node.pose.orientation.w = 1.0; 
        node.pose.position.x = o_node.x; //노드의 x 좌표 
        node.pose.position.y = o_node.y; //노드의 y 좌표 
        // Points are green 
        node.color.g = RVIZ_COLOR_GREEN; 
        node.color.r = RVIZ_COLOR_RED; 
        node.color.a = 1.0; 
        node.scale.x = 1.0; 
        node.scale.y = 1.0; 
        node_arr.markers.push_back(node); 

        visualization_msgs::Marker node_name; 
        node_name.header.frame_id = VELODYNE_BASE; // frame 기준 
        node_name.header.stamp = ros::Time::now(); 
        node_name.text = std::to_string(i+1); 
        node_name.color.a = 1.0; 
        node_name.scale.z = 1.0; 
        node_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING; 
        node_name.id = i + 100; 
        node_name.action = visualization_msgs::Marker::ADD; 
        node_name.pose.orientation.w = 1.0; 
        node_name.pose.position.x = o_node.x; //노드의 x 좌표 
        node_name.pose.position.y = o_node.y; //노드의 y 좌표 
        node_arr.markers.push_back(node_name);
    }
    marker_pub.publish(node_arr);
}

