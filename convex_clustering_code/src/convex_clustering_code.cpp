#include "convex_clustering_code/convex_clustering_code.h"

bool CVC::initialize()
{ 
    if (ros::ok())
    {
        // Initialize Subscriber for input Pointcloud2 6
        input_points = nh_.subscribe("velodyne_points", 1, &CVC::cloudCallback, this);
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

	std::cout<< "cluster_point " << cluster_point->points.size() << std::endl;
	std::vector<PointAPR> capr;
	calculateAPR(*cluster_point, capr);

	std::cout<<"capr : "<<capr.size() << std::endl;
	std::unordered_map<int, Voxel> hash_table;
	build_hash_table(capr, hash_table);

	std::vector<int> cluster_indices;	
	cluster_indices = cluster(hash_table, capr);

	std::vector<int> cluster_id;
	most_frequent_value(cluster_indices, cluster_id);
}

void CVC::spinNode()
{
    ros::Duration(1/frequency).sleep(); // Synchronize to lidar frequency 
    ros::spinOnce();
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

// Gaussian을 따르는 clustering 방법을 추가해서 output으로 받기 추가
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
                    if (x>(width_-1)) {
                         px=0;
                    }
				neighborindex.push_back((px*(length_)+y)+z*(length_)*(width_));
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
        sort(tr.begin(),tr.end(),compare_cluster);
        for(int i = 0 ; i< tr.size(); ++i){
            if(tr[i].second>10){
            cluster_index.push_back(tr[i].first);
        }
    }
	return true;
}

std::vector<int> CVC::cluster(std::unordered_map<int, Voxel> &map_in,const std::vector<PointAPR>& vapr){
    int current_cluster = 0;
    printf(" ------ Doing CVC cluster ------- \n");
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
            for(int s =0 ; s<neightbors.size(); ++s){             
                cluster_indices[neightbors[s]] = current_cluster;
            }
        }
    }
    int max = *max_element(cluster_indices.begin(), cluster_indices.end());
    cout << "Cluster indices max: " << max << endl;
	return cluster_indices;
}

void CVC::mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2) {
	for (int i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}