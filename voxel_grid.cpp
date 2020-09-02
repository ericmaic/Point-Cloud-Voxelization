#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/pcl_visualizer.h> 

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream> 
#include <fstream>
#include <string>
#include <cstring>
#include <math.h>

using namespace std::chrono_literals;
using namespace std;

//Create space index arry
int space_Index[192][128][128];

int main(int argc, char** argv) {
	
	//estimate whether inputs are correct or not
	if(argc < 3 || argc > 5){
	
		cout << "Please input as follows: " << endl;
		cout << "./voxel_grid POINTS_FILE.txt VOXEL_SIZE[0.011,0.016]" << endl;
		cout << "OR" << endl;
		cout << "./voxel_grid POINTS_FILE.txt VOXEL_SIZE[0.011,0.016] BOX_MODE(AABB / CUSTOM) RENDERING_MODE(A / B / C)" << endl;
		return 0;
	}
	
	if(argc == 4){
	
		cout << "Please input as follows: " << endl;
		cout << "./voxel_grid POINTS_FILE.txt VOXEL_SIZE[0.011,0.016]" << endl;
		cout << "OR" << endl;
		cout << "./voxel_grid POINTS_FILE.txt VOXEL_SIZE[0.011,0.016] BOX_MODE(AABB / CUSTOM) RENDERING_MODE(A / B / C)" << endl;
		return 0;
	}
	
	typedef struct point_XYZ{
	
		double point_X;
		double point_Y;
		double point_Z;
		double point_R;
	}POINT;
	
	typedef struct Array_List{
	
		double index_X;
		double index_Y;
		double index_Z;
		double index_R;
	}ARRAY;
	
	typedef struct point_voxel_Match{
	
		int point_Index;
		int voxel_Index;
		
		point_voxel_Match(int point_Index_, int voxel_Index_) : point_Index(point_Index_), voxel_Index(voxel_Index_) {}
		
		bool operator < (const point_voxel_Match &p) const {
		
			return (voxel_Index < p.voxel_Index);
		}
		
	}Match;
	
	//point clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_txt(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_grid(new pcl::PointCloud<pcl::PointXYZ>());
	
	/*------------------------------------------------------------------
	*Read hair points(3D: x,y,z) data from txt file, create new pcd file 
	*-------------------------------------------------------------------
	*Input: txt point 3D data
	*Output: new pcd file
	*/
	
	FILE *file;
	
	int point_Sum; 
	point_XYZ point;
	vector<point_XYZ> vec_Point;
	
	file = fopen(argv[1], "r");
	if(file){
	
		while(fscanf(file, "%lf %lf %lf", &point.point_X, &point.point_Y, &point.point_Z) != EOF){
		
			vec_Point.push_back(point);
		}
	}
	else{
	
		cout << "Could not load data from " << argv[1] <<" file..." <<endl;
		return 0;
	}
	
	fclose(file);
	point_Sum = vec_Point.size();
	
	//fill data into pointcloud
	cloud_txt -> width = point_Sum;
	cloud_txt -> height = 1;
	cloud_txt -> is_dense = false;
	cloud_txt -> points.resize(cloud_txt->width * cloud_txt->height);
	
	for(size_t i = 0; i< cloud_txt -> points.size(); ++i){
	
		cloud_txt -> points[i].x = vec_Point[i].point_X;
		cloud_txt -> points[i].y = vec_Point[i].point_Y;
		cloud_txt -> points[i].z = vec_Point[i].point_Z;
	}
	
	//saved into new pcd file
	string fileName_S(argv[1]);
	int location = fileName_S.find(".");
	string pcdName = fileName_S.substr(0, location+1) + "pcd";
	
	pcl::io::savePCDFileASCII(pcdName, *cloud_txt);
	
	std::cerr << "[Success] | New pcd file: " << pcdName << " has been created..." << std::endl << "            " <<cloud_txt -> points.size() << " data have been added..." <<  std::endl;
	
	/*------------------------------------------------------------------
	*Determine two kinds of bounding boxes: AABB(Axially Aligned Bounding Box), fixed 128*128*192 box
	*-------------------------------------------------------------------
	*Input: hair point data, voxel grid size
	*Output: min and max points for AABB and fixed box
	*/
	
	//read the pcd file 
	if(pcl::io::loadPCDFile(pcdName, *cloud) == -1){
	
		cout << "Could not open the file: " << pcdName << "..." <<endl;
		return -1;
	}
	
	if(cloud -> points.size() == 0){
	
		cout << "The file: " << pcdName << "has no point..." << endl;
		return -1;
	}
	
	//instantiation momentof
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	
	//declaration variables
	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	
	point_XYZ min_point;
	point_XYZ max_point;
	 
	//compute values of feature, finding min and max points (x, y, z)
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);
	
	min_point.point_X = (*min_element(vec_Point.begin(), vec_Point.end(), [](point_XYZ &a, point_XYZ &b){return a.point_X < b.point_X;})).point_X;
	min_point.point_Y = (*min_element(vec_Point.begin(), vec_Point.end(), [](point_XYZ &a, point_XYZ &b){return a.point_Y < b.point_Y;})).point_Y;
	min_point.point_Z = (*min_element(vec_Point.begin(), vec_Point.end(), [](point_XYZ &a, point_XYZ &b){return a.point_Z < b.point_Z;})).point_Z;
	min_point.point_R = 1;
	
	max_point.point_X = (*max_element(vec_Point.begin(), vec_Point.end(), [](point_XYZ &a, point_XYZ &b){return a.point_X < b.point_X;})).point_X;
	max_point.point_Y = (*max_element(vec_Point.begin(), vec_Point.end(), [](point_XYZ &a, point_XYZ &b){return a.point_Y < b.point_Y;})).point_Y;
	max_point.point_Z = (*max_element(vec_Point.begin(), vec_Point.end(), [](point_XYZ &a, point_XYZ &b){return a.point_Z < b.point_Z;})).point_Z;
	max_point.point_R = 1;
	
	//Construct voxel bounding box 
	double total_reg_X = 128 * atof(argv[2]);
	double total_reg_Y = 192 * atof(argv[2]);
	double total_reg_Z = 128 * atof(argv[2]);
	
	point_XYZ bound_min;
	point_XYZ bound_max;
	
	bound_min.point_X = -(total_reg_X/2);
	bound_max.point_X = total_reg_X/2;
	bound_min.point_Y = 0.0;
	bound_max.point_Y = total_reg_Y;
	bound_min.point_Z = -(total_reg_Z/2);
	bound_max.point_Z = total_reg_Z/2;
	
	//cout << "The bound min x is: " << bound_min.point_X << endl;
	//cout << "The bound max x is: " << bound_max.point_X << endl;
	
	//cout << "The bound min y is: " << bound_min.point_Y << endl;
	//cout << "The bound max y is: " << bound_max.point_Y << endl;
	
	//cout << "The bound min z is: " << bound_min.point_Z << endl;
	//cout << "The bound max z is: " << bound_max.point_Z << endl;
	
	//cout<< "The max y of model is:" << max_point_AABB.y << endl;
	//cout<< "The min y of model is:" << min_point_AABB.y << endl;
	
	cout << "[Success] | The bounding box has been constructed..." << endl;
	
	/*------------------------------------------------------------------
	*The process of voxelization, generate space index array and write into new txt file
	*-------------------------------------------------------------------
	*Input: voxel grid size, hair points
	*Output: space index array and new txt file
	*/
	
	//Find out the inverse of voxel grid size
	Array_List inverse_voxel_size;
	
	inverse_voxel_size.index_X = 1 / atof(argv[2]);
	inverse_voxel_size.index_Y = 1 / atof(argv[2]);
	inverse_voxel_size.index_Z = 1 / atof(argv[2]);
	inverse_voxel_size.index_R = 1;
	
	//point and vexel match rule
	Array_List index_rule;
	
	index_rule.index_X = 1;
	index_rule.index_Y = 128;
	index_rule.index_Z = 128 * 192;
	
	//initialize the space index array x, y, z
	for(int s_z = 0; s_z < 128; s_z++){
	
		for(int s_x = 0; s_x < 192; s_x++){
		
			for(int s_y = 0; s_y < 128; s_y++){
			
				space_Index[s_x][s_y][s_z] = 0;
			}
		}
	}
	
	cout << "[Success] | space index array has initial value..." << endl;
	
	//traverse all hair points and obtain the voxel index that each point belongs to
	vector<point_voxel_Match> p_v_match;
	p_v_match.reserve(vec_Point.size());
	
	for(int i = 0; i < vec_Point.size(); ++i){
	
		//if(vec_Point[i].point_X >= bound.min.point_X && vec_)
		int ijk0 = static_cast<int> (ceil(vec_Point[i].point_X * inverse_voxel_size.index_X) - (bound_min.point_X * inverse_voxel_size.index_X));
		int ijk1 = static_cast<int> (floor(vec_Point[i].point_Y * inverse_voxel_size.index_Y) - (bound_min.point_Y * inverse_voxel_size.index_Y));
		int ijk2 = static_cast<int> (floor(vec_Point[i].point_Z * inverse_voxel_size.index_Z) - (bound_min.point_Z * inverse_voxel_size.index_Z));
		
		//compute the voxel index
		int idx = ijk0 * index_rule.index_X + ijk1 * index_rule.index_Y + ijk2 * index_rule.index_Z;
		p_v_match.push_back(point_voxel_Match(i, idx));
		
		//save the occupied voxel grid into space index array
		//space_Index[ijk0-1][ijk1-1][ijk2-1] = 1;
		space_Index[ijk1][ijk0-1][ijk2] = 1;
	}
	
	cout << "[Success] | Find out the occupied voxel grids..." << endl;
	cout << "[Success] | Load data into space index array..." << endl;
	
	//find out the nums of 1 value and 0 value
	int i_space = 0;
	int j_space = 0;
	
	for(int s_z = 0; s_z < 128; s_z++){
	
		for(int s_x = 0; s_x < 192; s_x++){
		
			for(int s_y = 0; s_y < 128; s_y++){
			
				if(space_Index[s_x][s_y][s_z] == 1){
				
					i_space ++;
				
				}else if(space_Index[s_x][s_y][s_z] == 0){
				
					j_space ++;
				}	 
			}
		}
	}
	
	//cout << "The space has: " << i_space << " value of 1" << endl;
	//cout << "The space has: " << j_space << " value of 0" << endl;
	
	//sort the vector
	std::sort(p_v_match.begin(), p_v_match.end(), std::less<point_voxel_Match>());
	
	//count the number of voxel grids
	int voxel_total = 0;
	int v_index = 0;
	
	vector<std::pair<int, int>> voxel_interval;
	voxel_interval.reserve(p_v_match.size());
	
	while(v_index < p_v_match.size()){
	
		int i = v_index + 1;
		while(i < p_v_match.size() && p_v_match[i].voxel_Index == p_v_match[v_index].voxel_Index){
			
			++i;
		}
		
		if((i - v_index) >= 0){
		
			++voxel_total;	
			voxel_interval.push_back(std::pair<int, int>(v_index, i));
		}
		
		v_index = i;
	}
	
	//compute central points and insert them into voxel grids
	int ini_index;
	int last_index;
	
	double x_sum;
	double y_sum;
	double z_sum;
	
	point_XYZ center_point;
	vector<point_XYZ> center_cloud;
	
	for(int j = 0; j < voxel_interval.size(); ++j){
	
		ini_index = voxel_interval[j].first;
		last_index = voxel_interval[j].second;
		
		x_sum = 0;
		y_sum = 0;
		z_sum = 0;
		
		for(int k = ini_index; k < last_index; ++k){
		
			x_sum += vec_Point[p_v_match[k].point_Index].point_X;
			y_sum += vec_Point[p_v_match[k].point_Index].point_Y;
			z_sum += vec_Point[p_v_match[k].point_Index].point_Z;
		}
		
		center_point.point_X = x_sum / (last_index - ini_index);
		center_point.point_Y = y_sum / (last_index - ini_index);
		center_point.point_Z = z_sum / (last_index - ini_index);
		
		/*
		cloud_voxel_grid -> points[j].x = center_point.point_X;
		cloud_voxel_grid -> points[j].y = center_point.point_Y;
		cloud_voxel_grid -> points[j].z = center_point.point_Z;
		*/
		
		center_cloud.push_back(center_point);
	}
	
	//VoxelGrid filtering ***PCL***
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(atof(argv[2]), atof(argv[2]), atof(argv[2]));
	sor.filter(*cloud_filter);
	
	int filterPointSize = cloud_filter -> points.size();
	
	//Write the space_index array into new txt file
	string fileName_T(argv[1]);
	int location_T = fileName_T.find(".");
	string txtName = fileName_T.substr(0, location_T) + "_Array" + ".txt";
	
	ofstream OutFile(txtName);
	
	string txtTitle = "#" + fileName_T.substr(0, location_T) + " has " + to_string(point_Sum) + " points, " + to_string(voxel_total) + " occupied voxel grids." + '\n' + "#" + "voxel grid size is " + to_string(atof(argv[2])) + ", " + "The bounding box min x y z are: " + to_string(bound_min.point_X) + ", " + to_string(bound_min.point_Y) + ", " + to_string(bound_min.point_Z) + ", The bounding box max x y z are: " + to_string(bound_max.point_X) + ", " + to_string(bound_max.point_Y) + ", " + to_string(bound_max.point_Z) + '\n';
	
	string txtContent;
	
	for(int s_z = 0; s_z < 128; s_z++){
	
		for(int s_x = 0; s_x < 192; s_x++){
		
			for(int s_y = 0; s_y < 128; s_y++){
			
				txtContent.append(to_string(space_Index[s_x][s_y][s_z]));
				//txtContent.append(" ");
			}
			
			txtContent.append("\n");
		}
		
		txtContent.append("\n");
		txtContent.append("\n");
	}
	
	OutFile << txtTitle << endl;
	OutFile << txtContent << endl;
	
	OutFile.close();
	
	cout << "[Success] | New txt file has created..." << endl;
	cout << "[Success] | Find out " << voxel_total << " occupied voxel grids." << endl;
	cout << "            There are " << 128 * 192 * 128 << " voxel grids." << endl;
	cout << "            Space index array has " << i_space << " value of [1]" << endl;
	cout << "            Space index array has " << j_space << " value of [0]" << endl;
	
	/*------------------------------------------------------------------
	*voxel grid visualization in three different ways and use two kinds of bounding boxes
	*-------------------------------------------------------------------
	*A). AABB (Axially Aligned Bounding Box)
	*B). CUSTOM (fixed 128*128*192 box)
	*
	*a). point cloud mode 
	*b). solid cube mode
	*c). wireframe and point cloud mode
	*/
	
	if(argc == 5){
	
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("Voxelization"));
	viewer -> setBackgroundColor(0,0,0);
	
	double voxelSize = atof(argv[2]);
	
	string box_mode(argv[3]);
	string rendering_mode(argv[4]);
	
	if(box_mode == "AABB"){
		
		viewer -> addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z,
		1.0, 1.0, 1.0, "AABB");
		viewer -> setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
		
		if(rendering_mode == "A"){
		
			//rendering by z axis
			pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> v_color(cloud,"z"); 
			viewer -> addPointCloud<pcl::PointXYZ>(cloud, v_color, "vertices color");
		
		}else if(rendering_mode == "B"){
		
			for(int i=0; i<filterPointSize; i++){
	
			double x = cloud_filter->points[i].x;
			double y = cloud_filter->points[i].y;
			double z = cloud_filter->points[i].z;
		
			Eigen::Vector3f center(floor(x / voxelSize)*voxelSize + voxelSize/2, floor(y / voxelSize)*voxelSize + voxelSize/2, floor(z / voxelSize)*voxelSize + voxelSize/2);
		
			Eigen::Quaternionf rotation(1,0,0,0);
			string cube = "AABB"+to_string(i);
			viewer -> addCube(center,rotation,voxelSize, voxelSize, voxelSize, cube);
			
			}
		
		}else if(rendering_mode == "C"){
		
			//rendering by z axis
			pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> v_color(cloud,"z"); 
			viewer -> addPointCloud<pcl::PointXYZ>(cloud, v_color, "vertices color");
			
			for(int i=0; i<filterPointSize; i++){
	
			double x = cloud_filter->points[i].x;
			double y = cloud_filter->points[i].y;
			double z = cloud_filter->points[i].z;
		
			Eigen::Vector3f center(floor(x / voxelSize)*voxelSize + voxelSize/2, floor(y / voxelSize)*voxelSize + voxelSize/2, floor(z / voxelSize)*voxelSize + voxelSize/2);
		
			Eigen::Quaternionf rotation(1,0,0,0);
			string cube = "AABB"+to_string(i);
			viewer -> addCube(center,rotation,voxelSize, voxelSize, voxelSize, cube);
			
			//shape rendering
			viewer -> setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
			}
		
		}else{
		
			cout << "Input mode of rendering does not exists, please choose one of them (A / B / C)..." << endl;
			return -1;
		}
	
	
	}else if(box_mode == "CUSTOM"){
	
		viewer -> addCube (bound_min.point_X, bound_max.point_X, bound_min.point_Y, bound_max.point_Y, bound_min.point_Z, bound_max.point_Z, 1.0, 1.0, 1.0, "AABB");
		viewer -> setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
		
		//add coordinate
		//viewer -> addCoordinateSystem(1.0);
	
		if(rendering_mode == "A"){
		
			//rendering by z axis
			pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> v_color(cloud,"z"); 
			viewer -> addPointCloud<pcl::PointXYZ>(cloud, v_color, "vertices color");
		
		}else if(rendering_mode == "B"){
		
			for(int i=0; i < center_cloud.size(); i++){
	
			double x = center_cloud[i].point_X;
			double y = center_cloud[i].point_Y;
			double z = center_cloud[i].point_Z;
		
			Eigen::Vector3f center(floor(x / voxelSize)*voxelSize + voxelSize/2, floor(y / voxelSize)*voxelSize + voxelSize/2, floor(z / voxelSize)*voxelSize + voxelSize/2);
		
			Eigen::Quaternionf rotation(1,0,0,0);
			string cube = "AABB"+to_string(i);
			viewer -> addCube(center,rotation,voxelSize, voxelSize, voxelSize, cube);	
			}
		
		}else if(rendering_mode == "C"){
		
			//rendering by z axis
			pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> v_color(cloud,"z"); 
			viewer -> addPointCloud<pcl::PointXYZ>(cloud, v_color, "vertices color");
			
			for(int i=0; i < center_cloud.size(); i++){
	
			double x = center_cloud[i].point_X;
			double y = center_cloud[i].point_Y;
			double z = center_cloud[i].point_Z;
		
			Eigen::Vector3f center(floor(x / voxelSize)*voxelSize + voxelSize/2, floor(y / voxelSize)*voxelSize + voxelSize/2, floor(z / voxelSize)*voxelSize + voxelSize/2);
		
			Eigen::Quaternionf rotation(1,0,0,0);
			string cube = "AABB"+to_string(i);
			viewer -> addCube(center,rotation,voxelSize, voxelSize, voxelSize, cube);
			
			viewer -> setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);	
			}
		
		}else{
		
			cout << "Input mode of rendering does not exists, please choose one of them (A / B / C)..." << endl;
			return -1;
		}	
	
	}else{
	
		cout << "Input mode of bounding box does not exists, please choose one of them (AABB / CUSTOM)..." << endl;
		return -1;
	}
	
	//Rendering loop
	while(!viewer -> wasStopped()){
	
		viewer -> spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
	
	}
	
	return 0;
}


