

#define REPR_NUM 1500
using namespace std;
float u_pfh[125];
double *L2_dis;
int *pfh_sort_num;
struct regist {
	int source_idx;
	vector<int> target_idx;
};
void addGaussnoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, size_t miu, float sigma)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr icloud(new pcl::PointCloud<pcl::PointXYZ>);
	icloud->points.resize(cloud->points.size());//将点云的cloud的size赋值给噪声 
	icloud->header = cloud->header;
	icloud->width = cloud->width;
	icloud->height = cloud->height;

	boost::mt19937 zhongzi;
	zhongzi.seed(static_cast<unsigned int>(time(0)));
	boost::normal_distribution<> nd(miu, sigma);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> ok(zhongzi, nd);
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		cout << "random:" << static_cast<float>(ok()) << endl;
		icloud->points[i].x = cloud->points[i].x + static_cast<float>(ok());
		icloud->points[i].y = cloud->points[i].y + static_cast<float>(ok());
		icloud->points[i].z = cloud->points[i].z + static_cast<float>(ok());
		//icloud->points[i].x = cloud->points[i].x + (rand() % 100)*0.001;//max/100=0.002
		//icloud->points[i].y = cloud->points[i].y + (rand() % 100)*0.001;
		//icloud->points[i].z = cloud->points[i].z + (rand() % 100)*0.001;
	}
	ofstream fout("noise3.ply");
	fout << "ply" << endl << "format ascii 1.0" << endl
		<< "element vertex " + std::to_string(icloud->size()) << endl
		<< "property float x" << endl << "property float y" << endl << "property float z" << endl
		<< "end_header" << endl;
	for (int i = 0; i < icloud->points.size(); i++)
	{
		fout << icloud->points[i].x
			<< " " << icloud->points[i].y
			<< " " << icloud->points[i].z
			<< endl;
	}
	fout.close();
	return;
}
void addGaussnoise_random(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, size_t miu, float sigma,float ratio)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr icloud(new pcl::PointCloud<pcl::PointXYZ>);
	icloud->points.resize(cloud->points.size());//将点云的cloud的size赋值给噪声 
	icloud->header = cloud->header;
	icloud->width = cloud->width;
	icloud->height = cloud->height;
	int number = ratio * cloud->points.size();
	cout << "cloud_size:" << cloud->points.size() << endl;
	cout << "number" << number << endl;
	boost::mt19937 zhongzi;
	zhongzi.seed(static_cast<unsigned int>(time(0)));
	boost::normal_distribution<> nd(miu, sigma);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> ok(zhongzi, nd);
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		//cout << "random:" << static_cast<float>(ok()) << endl;
		icloud->points[i].x = cloud->points[i].x ;
		icloud->points[i].y = cloud->points[i].y ;
		icloud->points[i].z = cloud->points[i].z ;
		//icloud->points[i].x = cloud->points[i].x + (rand() % 100)*0.001;//max/100=0.002
		//icloud->points[i].y = cloud->points[i].y + (rand() % 100)*0.001;
		//icloud->points[i].z = cloud->points[i].z + (rand() % 100)*0.001;
	}
	int step = cloud->points.size() / number;
	cout << "step:" << step << endl;
	int flag1 = 0;
	for (size_t i = 0; i < number; ++i)
	{

		icloud->points[flag1].x = cloud->points[flag1].x + static_cast<float>(ok());
		icloud->points[flag1].y = cloud->points[flag1].y + static_cast<float>(ok());
		icloud->points[flag1].z = cloud->points[flag1].z + static_cast<float>(ok());
		flag1 = flag1 + step;

	}
	ofstream fout("noise3.ply");
	fout << "ply" << endl << "format ascii 1.0" << endl
		<< "element vertex " + std::to_string(icloud->size()) << endl
		<< "property float x" << endl << "property float y" << endl << "property float z" << endl
		<< "end_header" << endl;
	for (int i = 0; i < icloud->points.size(); i++)
	{
		fout << icloud->points[i].x
			<< " " << icloud->points[i].y
			<< " " << icloud->points[i].z
			<< endl;
	}
	fout.close();
	//system("pause");
	return;
}
float point_distance(pcl::PointXYZ point1, pcl::PointXYZ point2) {
	float distance = sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2) + pow((point1.z - point2.z), 2));
	return distance;
}
float compute_u_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin)
{
	

	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_origin);
	std::vector<int> pointIdxKSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointKSquaredDistance;//KD-TREE搜索时点的距离

	float distance_sum=0;
	float distance1;
	float distance2;
	for (int j = 0; j < cloud_origin->points.size(); j++) {
		searchPoint.x = cloud_origin->points[j].x;
		searchPoint.y = cloud_origin->points[j].y;
		searchPoint.z = cloud_origin->points[j].z;
		if (kdtree.nearestKSearch(searchPoint, 2, pointIdxKSearch, pointKSquaredDistance)>0)
		{
			distance1 = point_distance(cloud_origin->at(j), cloud_origin->at(pointIdxKSearch[0]));
			distance2 = point_distance(cloud_origin->at(j), cloud_origin->at(pointIdxKSearch[1]));
			//cout << "ditstance" << j << "to 1 is :" << distance1 << endl;
			//cout << "ditstance" << j << "to 2 is :" << distance2 << endl;
			distance_sum += distance2;
		}
	}
	float u_cloud = distance_sum / cloud_origin->points.size();
	cout << "u_cloud:" << u_cloud << endl;
	return u_cloud;
		

}

float computeError(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
	float error = 0;
	int cnt = 0;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(target);
	vector<int> nn_index(1);
	vector<float> nn_distance(1);
	for (int i = 0; i < source->size(); ++i) {

		tree.nearestKSearch(*source, i, 1, nn_index, nn_distance);
		//bunny
		//if (nn_distance[0] > 0.004) continue;     // 对于距离太远的点，则将其排除误差，此处需要结合点云分辨率设定阈值
		//amo
		//if (nn_distance[0] > 0.6) continue;     // 对于距离太远的点，则将其排除误差，此处需要结合点云分辨率设定阈值
		error += nn_distance[0];
		cnt++;
	}
	return error / cnt;
}
void compute_u_high(pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhptr) {
	for (int i = 0; i < pfhptr->points.size(); i++)
	{
		for (int j = 0; j < 125; j++) {
			u_pfh[j] += pfhptr->points[i].histogram[j];
		}	
	}
	for (int i = 0; i < 125; i++) {
		u_pfh[i] = u_pfh[i] / (pfhptr->points.size());
		//cout << "u" << i << ":" << u_pfh[i] << endl;
	}
}

void compute_FPFH_u_high(pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfhptr) {
	for (int i = 0; i < pfhptr->points.size(); i++)
	{
		for (int j = 0; j < 33; j++) {
			u_pfh[j] += pfhptr->points[i].histogram[j];
		}
	}
	for (int i = 0; i < 33; i++) {
		u_pfh[i] = u_pfh[i] / (pfhptr->points.size());
		//cout << "u" << i << ":" << u_pfh[i] << endl;
	}
}
//a
void quick_sort(double a[],int b[],int low,int high) {
	if (low > high) {
		return;
	}
	int i = low;
	int j = high;
	double key = a[low];
	int key1 = b[low];
	while (i<j)
	{
		while (i < j&&a[j] >= key) {
			j--;
		}
		a[i] = a[j];
		b[i] = b[j];
		while (i < j&&a[i] <= key) {
			i++;
		}
		a[j] = a[i];
		b[j] = b[i];
	}
	a[i] = key;
	b[i] = key1;
	quick_sort(a, b, low, i - 1);
	quick_sort(a, b, i+1, high);
}
void compute_couple_sort(std::vector<float>& a, std::vector<int>& b,int low,int high) {
	if (low > high) {
		return;
	}
	int i = low;
	int j = high;
	//float key = a[low];
	float key = a[b[low]];
	int key_val = b[low];
	while (i<j)
	{
		while (i < j&&a[b[j]] >= key) {
			j--;
		}

		b[i] = b[j];
		while (i < j&&a[b[i]] <= key) {
			i++;
		}
		b[j] = b[i];
	}
	b[i] = key_val;
	compute_couple_sort(a, b, low, i - 1);
	compute_couple_sort(a, b, i + 1, high);
}
void compute_L2_dis(pcl::PointCloud<pcl::PFHSignature125>::Ptr L2_ptr) {
	L2_dis = new double[L2_ptr->points.size()];
	double dis = 0;
	for (int i = 0; i < L2_ptr->points.size(); i++)
	{
		for (int j = 0; j < 125; j++) {
			//L1
			//dis += abs(L2_ptr->points[i].histogram[j] - u_pfh[j]);
			//L2
			dis += pow((L2_ptr->points[i].histogram[j] - u_pfh[j]), 2);
		}
		//L1
		//L2_dis[i] = dis;
		//L2
		L2_dis[i] = sqrt(dis);
		dis = 0;
	}
}
void compute_FPFH_L2_dis(pcl::PointCloud<pcl::FPFHSignature33>::Ptr L2_ptr) {
	L2_dis = new double[L2_ptr->points.size()];
	double dis = 0;
	for (int i = 0; i < L2_ptr->points.size(); i++)
	{
		for (int j = 0; j < 33; j++) {
			//L1
			//dis += abs(L2_ptr->points[i].histogram[j] - u_pfh[j]);
			//L2
			dis += pow((L2_ptr->points[i].histogram[j] - u_pfh[j]), 2);
		}
		//L1
		//L2_dis[i] = dis;
		//L2
		L2_dis[i] = sqrt(dis);
		dis = 0;
	}
}
//void compute_kl_dis(pcl::pointcloud<pcl::pfhsignature125>::ptr kl_ptr) {
//	kl_dis = new double [kl_ptr->points.size()];
//	//int size = kl_ptr->points.size();
//	//double kl_dis[size];
//	double dis = 0;
//	for (int i = 0; i < kl_ptr->points.size(); i++)
//	{
//		for (int j = 0; j < 125; j++) {
//			if ((kl_ptr->points[i].histogram[j] + u_pfh[j])==0)
//			{
//				dis = dis;
//			}
//			else
//			{
//				dis += pow((kl_ptr->points[i].histogram[j] - u_pfh[j]), 2) / (kl_ptr->points[i].histogram[j] + u_pfh[j]);
//			}
//			
//			//if (u_pfh[j] == 0)
//			//{
//			//	dis = dis;
//			//}
//			//else
//			//{
//			//	dis += (kl_ptr->points[i].histogram[j]-u_pfh[j])*(log((kl_ptr->points[i].histogram[j])/ (u_pfh[j])));
//			//	//dis += abs(kl_ptr->points[i].histogram[j] - u_pfh[j]);
//			//}
//			//dis += (kl_ptr->points[i].histogram[j]-u_pfh[j])*(log((kl_ptr->points[i].histogram[j])/ (u_pfh[j])));
//			//dis += abs(kl_ptr->points[i].histogram[j] - u_pfh[j]);
//		}
//		kl_dis[i] = dis;
//		//cout << dis << endl;
//		dis = 0;
//	}
//}
//以.ply文件格式保存点云文件
void savefile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_result, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename, string method)
{
	ofstream fout(method + "_" + filename);
	fout << "ply" << endl << "format ascii 1.0" << endl
		<< "element vertex " + std::to_string(cloud_result->size()) << endl
		<< "property float x" << endl << "property float y" << endl << "property float z" << endl
		<< "property float nx" << endl << "property float ny" << endl << "property float nz" << endl
		<< "property uchar red" << endl << "property uchar green" << endl << "property uchar blue" << endl
		<< "end_header" << endl;
	for (int i = 0; i < cloud_result->points.size(); i++)
	{
		fout << cloud_result->points[i].x
			<< " " << cloud_result->points[i].y
			<< " " << cloud_result->points[i].z
			<< " " << cloud_normals->points[i].normal_x
			<< " " << cloud_normals->points[i].normal_y
			<< " " << cloud_normals->points[i].normal_z
			<< " " << std::to_string(cloud_result->at(i).r) << " " << std::to_string(cloud_result->at(i).g) << " " << std::to_string(cloud_result->at(i).b)
			<< endl;
	}
	fout.close();
}
void Computer_Harris(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source)
{
	

	
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	pcl::PointCloud<pcl::PointXYZI>::Ptr Harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(17);
	harris.setRadius(1.5);//设置法向量估计的半径
	//harris.setRadiusSearch(0.004);//设置关键点估计的近邻搜索半径
	harris.setInputCloud(cloud_source);
	///////////
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	harris.setNormals(source_normals);//如果有预先计算的法线，则设置法线
	//////
	//harris.setNormals(normal);//如果有预先计算的法线，则设置法线
	harris.setNonMaxSupression(true);//设置是否应应用非极大值抑制或应返回每个点的响应（非必需参数）
	harris.setNumberOfThreads(6);//初始化调度程序并设置要使用的线程数
	harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::LOWE);//设置要计算响应的方法（可以不设置）
	harris.compute(*Harris_keypoints);

	cout << "Harris_keypoints的大小是" << Harris_keypoints->size() << endl;
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZI>("Harris_keypoints.ply", *Harris_keypoints, false);



}

void Computer_two_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{

	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	//for (int i = 0; i < 0; i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("Harris_Feature.ply", final);


}
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename)
pcl::PointCloud<pcl::PointXYZRGB>::Ptr computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, string filename)
{
//void computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename)

	float radius;
	int k_number ;
	
	cout << "input radius/k_nubmer of pfh_search: ";
	cin >> radius;
	//cin >> k_number;

	////计算法线特征////
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	ne.setInputCloud(cloud_origin);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);//设置近邻搜索算法
							 // 输出点云 带有法线描述
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrigin(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>& cloud_normals = *cloud_normals_ptr;
	// Use all neighbors in a sphere of radius 3m
	ne.setKSearch(10);
	//ne.setRadiusSearch(0.002);//半价内搜索临近点 3m
						  // 计算表面法线特征
	//ne.setRadiusSearch(3);
	ne.compute(cloud_normals);

	//#################PFH###################
	//time_t Start, End;
	//Start = clock();
	//pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh; //声明pfh类对象
	//pfh.setInputCloud(cloud_origin); //设置输入点云
	//pfh.setInputNormals(cloud_normals_ptr); //设置输入点云的法线
	////pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//pfh.setSearchMethod(tree); //设置近邻搜索方式
	//pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_fe_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	//																									//pfh.setRadiusSearch(radius);

	//pfh.setRadiusSearch(radius);
	////pfh.setKSearch(k_number); //设置近邻个数
	//pfh.compute(*pfh_fe_ptr); //计算每个点的pfh
	//End = clock() - Start;
	//std::cout <<"PFH Time is :"<< End << std::endl;
	//compute_u_high(pfh_fe_ptr);
	////
	//compute_L2_dis(pfh_fe_ptr);
	////pcl::visualization::PCLPlotter plotter;
	//pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter();
	////plotter.addFeatureHistogram(*pfh_fe_ptr,1000); //设置的很坐标长度，该值越大，则显示的越细致

	////////////////输出u——直方图/////////
	////std::vector<double> arr_x(125, 0);
	////for (int i = 0; i < 125; i++) {
	////	arr_x[i] = i;
	////}
	////std::vector<double> arr_y(125, 0);
	////for (int i = 0; i < 125; i++) {
	////	arr_y[i] = u_pfh[i];
	////}
	//////plotter.addfeaturehistogram(*pfh_fe_ptr,300000);
	////plotter->addPlotData(arr_x, arr_y);
	//////plotter->addplotdata(arr_x,arr_y);
	////plotter->plot();
	////////////////输出u——直方图/////////


	//////////////输出l2——直方图/////////
	///*std::vector<double> arr_x(pfh_fe_ptr->points.size(), 0);
	//std::vector<double> arr_y(pfh_fe_ptr->points.size(), 0);
	//for (int i = 0; i < pfh_fe_ptr->points.size(); i++) {
	//arr_x[i] = i;
	//arr_y[i] = L2_dis[i];
	//}
	//plotter->setTitle("Feature persistence");
	//plotter->setXTitle("Point index");
	//plotter->setYTitle("L2 distance value ");
	//plotter->addPlotData(arr_x, arr_y);
	//plotter->plot();*/
	//////////////输出l2——直方图/////////


	//////////////////L2排序///////////////
	//pfh_sort_num = new int[cloud_origin->points.size()];
	//for (int i = 0; i < cloud_origin->points.size(); i++)
	//{
	//	pfh_sort_num[i] = i;
	//}
	///*double c[10] = {0.001,0.0015,0.009,0.0002,0.004,0.0003,0.00035,0.00045,0.0034,0.002};
	//int d1[10] = {0,1,2,3,4,5,6,7,8,9};
	//quick_sort(c, d1, 0, 9);
	//for (int i = 0; i < 10; i++)
	//{
	//cout << i << ":" << c[i] << "" << endl;
	//}
	//for (int i = 0; i < 10; i++)
	//{
	//cout << i << ":" << d1[i] << "" << endl;
	//}*/
	//quick_sort(L2_dis, pfh_sort_num, 0, cloud_origin->points.size() - 1);
	///*for (int i = 0; i < cloud_origin->points.size(); i++)
	//{
	//cout << i << ":" << pfh_sort_num[i] << "" << endl;
	//}*/
	////system("pause");
	///*for (int i = cloud_origin->points.size()-1; i > cloud_origin->points.size()-301; i--)
	//{
	//cout << i + 1 << ":" << L2_dis[i]<< "" << "序号：" << pfh_sort_num[i] << endl;
	//}*/
	//////////////////L2排序///////////////



	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_b->resize(cloud_origin->size());
	//for (int i = 0; i < cloud_origin->points.size(); i++)
	//{
	//	cloud_b->points[i].x = cloud_origin->points[i].x;
	//	cloud_b->points[i].y = cloud_origin->points[i].y;
	//	cloud_b->points[i].z = cloud_origin->points[i].z;
	//}

	//Eigen::Vector3d d, n1, n2;
	//double d_x, d_y, d_z;
	//cout << "ready" << endl;
	////printf("pfh size %d", pfh_fe_ptr->points.size());
	//for (int i = 0; i < pfh_fe_ptr->points.size(); i++)
	//{
	//	//cout << "ready color" << endl;
	//	//cout << i << "_64:" << pfh_fe_ptr->points[i].histogram[62] << endl;
	//	//if (pfh_fe_ptr->points[i].histogram[62] < 50) //设置第63个组合区间的阈值为50，将特征点标记为红色
	//	if (L2_dis[i]>140)
	//	{
	//		//cout << "have coloor:"<<i << endl;
	//		cloud_b->at(i).r = 255;
	//		cloud_b->at(i).g = 0;
	//		cloud_b->at(i).b = 0;
	//	}
	//	else
	//	{
	//		//printf("no color");
	//		cloud_b->at(i).r = 100;
	//		cloud_b->at(i).g = 100;
	//		cloud_b->at(i).b = 100;
	//	}
	//}
	//savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
	//cout << "ok" << endl;
	//system("pause");

	//return cloud_b;
	//#################PFH###################

	//#################FPFH####################
	time_t Start1, End1;
	Start1 = clock();
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;//声明fpfh类对象
	fpfh.setInputCloud(cloud_origin); //设置输入点云
	fpfh.setInputNormals(cloud_normals_ptr); //设置输入点云的法线
											//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	fpfh.setSearchMethod(tree); //设置近邻搜索方式
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_fe_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
																										//pfh.setRadiusSearch(radius);

	fpfh.setRadiusSearch(radius);
	//pfh.setKSearch(k_number); //设置近邻个数
	fpfh.compute(*fpfh_fe_ptr); //计算每个点的pfh

	End1 = clock() - Start1;
	std::cout << "FPFH Time is :" << End1 << std::endl;
	//计算所有点的特征平均
	compute_FPFH_u_high(fpfh_fe_ptr);
	//
	compute_FPFH_L2_dis(fpfh_fe_ptr);
	//pcl::visualization::PCLPlotter plotter;
	pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter();
	//plotter.addFeatureHistogram(*pfh_fe_ptr,1000); //设置的很坐标长度，该值越大，则显示的越细致

	//////////////输出u——直方图/////////
	//std::vector<double> arr_x(125, 0);
	//for (int i = 0; i < 125; i++) {
	//	arr_x[i] = i;
	//}
	//std::vector<double> arr_y(125, 0);
	//for (int i = 0; i < 125; i++) {
	//	arr_y[i] = u_pfh[i];
	//}
	////plotter.addfeaturehistogram(*pfh_fe_ptr,300000);
	//plotter->addPlotData(arr_x, arr_y);
	////plotter->addplotdata(arr_x,arr_y);
	//plotter->plot();
	//////////////输出u——直方图/////////



	////////////输出l2——直方图/////////
	std::vector<double> arr_x(fpfh_fe_ptr->points.size(), 0);
	std::vector<double> arr_y(fpfh_fe_ptr->points.size(), 0);
	for (int i = 0; i < fpfh_fe_ptr->points.size(); i++) {
	arr_x[i] = i;
	arr_y[i] = L2_dis[i];
	}
	plotter->setTitle("Feature persistence");
	plotter->setXTitle("Point index");
	plotter->setYTitle("L2 distance value ");
	plotter->addPlotData(arr_x, arr_y);
	plotter->plot();
	////////////输出l2——直方图/////////



	////////////////L2排序///////////////
	pfh_sort_num = new int[cloud_origin->points.size()];
	for (int i = 0; i < cloud_origin->points.size(); i++)
	{
		pfh_sort_num[i] = i;
	}
	/*double c[10] = {0.001,0.0015,0.009,0.0002,0.004,0.0003,0.00035,0.00045,0.0034,0.002};
	int d1[10] = {0,1,2,3,4,5,6,7,8,9};
	quick_sort(c, d1, 0, 9);
	for (int i = 0; i < 10; i++)
	{
	cout << i << ":" << c[i] << "" << endl;
	}
	for (int i = 0; i < 10; i++)
	{
	cout << i << ":" << d1[i] << "" << endl;
	}*/
	quick_sort(L2_dis, pfh_sort_num, 0, cloud_origin->points.size() - 1);
	/*for (int i = 0; i < cloud_origin->points.size(); i++)
	{
	cout << i << ":" << pfh_sort_num[i] << "" << endl;
	}*/
	//system("pause");
	/*for (int i = cloud_origin->points.size()-1; i > cloud_origin->points.size()-301; i--)
	{
	cout << i + 1 << ":" << L2_dis[i]<< "" << "序号：" << pfh_sort_num[i] << endl;
	}*/
	////////////////L2排序///////////////



	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_b->resize(cloud_origin->size());
	for (int i = 0; i < cloud_origin->points.size(); i++)
	{
		cloud_b->points[i].x = cloud_origin->points[i].x;
		cloud_b->points[i].y = cloud_origin->points[i].y;
		cloud_b->points[i].z = cloud_origin->points[i].z;
	}

	Eigen::Vector3d d, n1, n2;
	double d_x, d_y, d_z;
	cout << "ready" << endl;
	//printf("pfh size %d", pfh_fe_ptr->points.size());
	for (int i = 0; i < fpfh_fe_ptr->points.size(); i++)
	{
		//cout << "ready color" << endl;
		//cout << i << "_64:" << pfh_fe_ptr->points[i].histogram[62] << endl;
		//if (pfh_fe_ptr->points[i].histogram[62] < 50) //设置第63个组合区间的阈值为50，将特征点标记为红色
		if (L2_dis[i]>70)
		{
			//cout << "have coloor:"<<i << endl;
			cloud_b->at(i).r = 255;
			cloud_b->at(i).g = 0;
			cloud_b->at(i).b = 0;
		}
		else
		{
			//printf("no color");
			cloud_b->at(i).r = 100;
			cloud_b->at(i).g = 100;
			cloud_b->at(i).b = 100;
		}
	}
	savefile(cloud_b, cloud_normals_ptr, filename, "FPFH"); //将点云以.ply文件格式保存
	cout << "ok" << endl;
	system("pause");

	return cloud_b;
	//#################FPFH###################

}

double getAngleTwoVectors(const Eigen::Vector3d & v1, const Eigen::Vector3d & v2) {
	double radian_angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
	return radian_angle;   //[0,PI]
}
double getAngle3D(const Eigen::Vector3d & v1, const Eigen::Vector3d & v2) {
	double radian_angle = acos(v1.normalized().dot(v2.normalized()));
	radian_angle = radian_angle * 180 / acos(-1);
	return radian_angle;   //[0,PI]

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr computeANGLE_Point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, string filename)
{
	//void computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename)

	float radius;
	int k_number;
	//int REPR_NUM;
	cout << "input radius/k_nubmer of pfh_search: ";
	cin >> radius;
	//cin >> k_number;
	cout << "REPR_NUM:";
	//cin >> REPR_NUM;
	////计算法线特征////
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_origin);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);//设置近邻搜索算法
							 // 输出点云 带有法线描述
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrigin(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>& cloud_normals = *cloud_normals_ptr;
	// Use all neighbors in a sphere of radius 3m
	ne.setKSearch(10);
	//ne.setRadiusSearch(0.003);//半价内搜索临近点 3m
						  // 计算表面法线特征
							  //ne.setRadiusSearch(3);
	ne.compute(cloud_normals);
	pcl::PointXYZ searchPoint;
	//searchPoint.x = -0.0294813;
	//searchPoint.y = 0.179959;
	//searchPoint.z = -0.00726258;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_origin);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	float angle;
	float angle1;
	float angle_sum = 0;
	float more_threshod;
	float less_threshod;
	float k = 0.15;
	float u_radius;
	float sear_rad;
	float stand_distance;
	long counter;
	time_t Start, End;
	Start = clock();
	for (int j = 0; j < cloud_origin->points.size(); j++) {
		searchPoint.x = cloud_origin->points[j].x;
		searchPoint.y = cloud_origin->points[j].y;
		searchPoint.z = cloud_origin->points[j].z;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2]/5))
					{
						stand_distance = pointRadiusSquaredDistance[2];
					}
					else
					{
						stand_distance = pointRadiusSquaredDistance[1];
					}
					if (pointRadiusSquaredDistance[i]<(1.5*stand_distance))
					{
						/*if (j>31770)
						{
							cout << "参数:" << i << ":" << pointRadiusSquaredDistance[i] << endl;
						}
						cout << "参数:" << i << ":" << pointRadiusSquaredDistance[i] << endl;*/
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						/*if (j>31770)
						{
							cout << "u_radius:" << j << ":" << u_radius << endl;
						}*/
						
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;

			if (pointIdxRadiusSearch.size()==1)
			{
				sear_rad = 2 * radius;
			}
			///*if (j>31770)
			//{
			//	cout << "u_radius:" << j << ":" << u_radius << endl;
			//	cout << "counter:" << j << counter << endl;
			//}*/
			else
			{
				u_radius = u_radius / counter;
				sear_rad = 1.2*u_radius * 1000;
				counter = 0;
				/*if (j>31770)
				{
					cout << "sear_rad:" << j << ":" << sear_rad << endl;
				}*/
			}

		}
		if (sear_rad<0.0015)
		{
			sear_rad = 0.0015;
		}
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{  
				if (pointRadiusSquaredDistance[i]==0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(cloud_normals_ptr->points[j].normal_x,
						cloud_normals_ptr->points[j].normal_y,
						cloud_normals_ptr->points[j].normal_z),

						v2(cloud_normals_ptr->points[pointIdxRadiusSearch[i]].normal_x,
							cloud_normals_ptr->points[pointIdxRadiusSearch[i]].normal_y,
							cloud_normals_ptr->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					angle_sum = angle_sum + angle1;
					if (angle1>15) {
						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad/1000-u_radius, 2)));
						more_threshod++;
					}
					else
					{
						//less_threshod= less_threshod+ 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod++;
					}

					//if (j<10)
					//{

					//	std::cout << "第" << j << "个的临近点"
					//		//<< " " << cloud_origin->points[pointIdxRadiusSearch[i]].x
					//		//<< " " << cloud_origin->points[pointIdxRadiusSearch[i]].y
					//		//<< " " << cloud_origin->points[pointIdxRadiusSearch[i]].z
					//		<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")"
					//		//<< " " << cloud_normals_ptr->points[pointIdxRadiusSearch[i]].normal_x
					//		//<< " " << cloud_normals_ptr->points[pointIdxRadiusSearch[i]].normal_y
					//		//<< " " << cloud_normals_ptr->points[pointIdxRadiusSearch[i]].normal_z
					//		<<"u_rad:"<<u_radius
					//		<<"ratio:"<< exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)))
					//		//<< "angle: " << angle 
					//	    << "angle1: " << angle1 << std::endl;
					//}
				}
			}
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########older vision#########
			//if ((more_threshod>1)&&(less_threshod<=1))
			//{
			//	anglepointidx.push_back(j);
			//}
			//if((more_threshod>1) && (less_threshod > 1)) {
			//	boundpointidx.push_back(j);
			//}
			//if ((more_threshod<=1) && (less_threshod > 1)) {
			//	facepointidx.push_back(j);
			//}
			//#########older vision#########
			
			//#########newer vision#########
			if (more_threshod>=less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;
			
		}
	}
	End = clock() - Start;
	std::cout << "ANGLE Time is :" << End << std::endl;

	//################ Compute sort#############
	
	compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size()-1);
	cout << "angle sort is ready" << endl;
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	compute_couple_sort(pointAngle, BoundPointIdx, 0, BoundPointIdx.size() - 1);
	cout << "bound sort is ready" << endl;
	cout << "BoundPointIdx.size()" << BoundPointIdx.size() << endl;
	//compute_couple_sort(pointAngle, FacePointIdx, 0, FacePointIdx.size() - 1);
	/*for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		cout << "第" << i << "序号:" << AnglePointIdx[i] << "" << "val:" << pointAngle[AnglePointIdx[i]] << endl;
	}*/
	/*for (int j = 0; j < cloud_origin->points.size(); j++)
	{
		cout << "pfh_bunj:" << pfh_sort_num[j] <<"value:"<<L2_dis[j]<< endl;
	}*/
		//cout << i + 1 << ":" << L2_dis[i] << "" << "序号：" << pfh_sort_num[i] << endl;
		//cout << "Ang
	//################ Compute sort#############


	//**************** Compute Repetition************//
	int Repetition_counter = 0;
	for (int j = cloud_origin->points.size() - 1; j > cloud_origin->points.size() - REPR_NUM-1; j--)
	{
		//cout << "pfhsort_num:" << pfh_sort_num[j] <<"value:"<<L2_dis[j]<< endl;
		//cout << i + 1 << ":" << L2_dis[i] << "" << "序号：" << pfh_sort_num[i] << endl;
		//cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
		for (int i = AnglePointIdx.size()-1; i >= 0; i--) {
			//cout << "j:" << j << endl;
			//cout << "pfh_sort_numj:" << pfh_sort_num[j] << endl;
			if (AnglePointIdx.size() - i>REPR_NUM)
			{
				break;
			}
			if (pfh_sort_num[j]== AnglePointIdx[i])
			{
				//cout << "angle_num:" << AnglePointIdx[i] << endl;
				Repetition_counter++;
				//cout << "Repetition_counter" << Repetition_counter << endl;
				break;
			}
			if ((i == 0)&& AnglePointIdx.size()<REPR_NUM)
			{
				//cout << "go to bound" << endl;
				for (int k = BoundPointIdx.size()-1; k >= 0; k--) {
					if (AnglePointIdx.size()+ BoundPointIdx.size()-k>REPR_NUM)
					{
						break;
					}
					if (pfh_sort_num[j] == BoundPointIdx[i])
					{
						//cout << "Repetition_counter" << Repetition_counter << endl;
						Repetition_counter++;
						break;
					}
				}
			}
		}
	}
	float Repetition_ratio = 100*Repetition_counter /REPR_NUM;
	cout << "retition:" << Repetition_ratio <<"%"<< endl;
	//system("pause");
	//**************** Compute Repetition************//


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_b->resize(cloud_origin->size());
	for (int i = 0; i < cloud_origin->points.size(); i++)
	{
		
		cloud_b->points[i].x = cloud_origin->points[i].x;
		cloud_b->points[i].y = cloud_origin->points[i].y;
		cloud_b->points[i].z = cloud_origin->points[i].z;
	}

	//Eigen::Vector3d d, n1, n2;

	cout << "ready" << endl;

	//printf("pfh size %d", pfh_fe_ptr->points.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		//cout << "angle:" << AnglePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_b->at(AnglePointIdx[i]).r = 255;
		cloud_b->at(AnglePointIdx[i]).g = 0;
		cloud_b->at(AnglePointIdx[i]).b = 0;
	}
	for (int i = 0; i < BoundPointIdx.size(); i++)
	{
		//cout << "bound:" << BoundPointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_b->at(BoundPointIdx[i]).r = 0;
		cloud_b->at(BoundPointIdx[i]).g = 255;
		cloud_b->at(BoundPointIdx[i]).b = 0;
	}
	for (int i = 0; i < FacePointIdx.size(); i++)
	{
		//cout << "face:" << FacePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_b->at(FacePointIdx[i]).r = 100;
		cloud_b->at(FacePointIdx[i]).g = 100;
		cloud_b->at(FacePointIdx[i]).b = 100;
	}
	cout << "总数：" << FacePointIdx.size() + BoundPointIdx.size() + AnglePointIdx.size();
	savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
	cout << "ok" << endl;
	system("pause");
	return cloud_b;
	//system(0);
	
	//exit(0);
}
void Sac_ia_PFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target )
{
	//void computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename)

	float radius;
	int k_number;

	cout << "input radius/k_nubmer of pfh_search: ";
	cin >> radius;
	//cin >> k_number;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh; //声明pfh类对象
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(10);
	pfh.setSearchMethod(tree); //设置近邻搜索方式
	pfh.setRadiusSearch(radius);
	////########计算Source点云############////
	//time_t Start, End;
	//Start = clock();
	//pcl::PointCloud<pcl::Normal>::Ptr source_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>& source_normals = *source_normals_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_source_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_source); //设置输入点云
	pfh.setInputNormals(source_normals); //设置输入点云的法线
	pfh.compute(*pfh_source_ptr); //计算每个点的pfh										
	//End = clock() - Start;
	//std::cout << "PFH Time is :" << End << std::endl;
	////########计算Source点云############////

	////########计算Target点云############////
	//pcl::PointCloud<pcl::Normal>::Ptr target_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>& target_normals = *target_normals_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_target_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_target); //设置输入点云
	pfh.setInputNormals(target_normals); //设置输入点云的法线
	pfh.compute(*pfh_target_ptr); //计算每个点的pfh										
    ////########计算Target点云############////
	
	////计算所有点的特征平均
	//compute_u_high(pfh_source_ptr);
	////
	//compute_L2_dis(pfh_source_ptr);
	//////////////////L2排序///////////////
	//pfh_sort_num = new int[cloud_source->points.size()];
	//for (int i = 0; i < cloud_source->points.size(); i++)
	//{
	//	pfh_sort_num[i] = i;
	//}
	//
	//quick_sort(L2_dis, pfh_sort_num, 0, cloud_source->points.size() - 1);
	//
	//////////////////L2排序///////////////
	int k_search = 3;
	pcl::search::KdTree<pcl::PFHSignature125> pfh_tree;
	pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	std::vector<int> pfh_index;
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 10;
	
	
	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 0.02;

	Eigen::Matrix4d final_transformation_;
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();
		//final_match.clear();
		int id = rand() % cloud_source->points.size();
		sample_idx.push_back(id);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			id = rand() % cloud_source->points.size();
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_source->at(id), cloud_source->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(id);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				id = rand() % cloud_source->points.size();
				sample_idx.push_back(id);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
			cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		for (size_t i = 0; i < 3; i++)
		{
			/*for (int j = 0; j < 125; i++)
			{
				search_pfh.histogram[j] = pfh_source_ptr->points[sample_idx[i]].histogram[j];
			}*/
			pfh_tree.nearestKSearch(*pfh_source_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}
		
		/*cout << "finished k_search" << endl;
		for (size_t i = 0; i < 3; i++)
		{
			cout << "the num " << i << ":" << matches[i].source_idx << endl;
		}*/

		//#############为随机的sample_size个点找到k_search个近似的点##############

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			if (id==0)
			{
				final_match[1][i] = matches[i].target_idx[id+1];
			}
			else
			{
				final_match[1][i] = matches[i].target_idx[id];
			}
			
		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
			cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);
		
		//cout << "66" << endl;
		pcl::transformPointCloud(*cloud_source, *new_source, T.matrix());
		//cout << "77" << endl;
		error = computeError(new_source, cloud_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		//cout << "99" << endl;
	}
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);
	
	
	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result1.ply", final);

	

}
void Computer_Harris_err(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float u_rad) {
	float radius = 2.5*u_rad;
	time_t Start, End;
	Start = clock();
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	pcl::PointCloud<pcl::PointXYZI>::Ptr Harris_keypoints_source(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr Harris_keypoints_target(new pcl::PointCloud<pcl::PointXYZI>());
	
	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(17);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setSearchMethod(tree); //设置近邻搜索方式
	fpfh.setRadiusSearch(1.85*radius);
	//fpfh.setRadiusSearch(5);
	//bun
	harris.setRadiusSearch(0.3);//设置关键点估计的近邻搜索半径
	//AMO
	//harris.setRadius(1.5);//设置法向量估计的半径
	//harris.setRadiusSearch(2.6);//设置关键点估计的近邻搜索半径
	//Bunny
	//harris.setRadius(1.5);//设置法向量估计的半径
	//harris.setRadiusSearch(0.007);//设置关键点估计的近邻搜索半径
	//harris.setInputCloud(cloud_source);
	//harris.setNormals(normal);//如果有预先计算的法线，则设置法线
	harris.setNonMaxSupression(true);//设置是否应应用非极大值抑制或应返回每个点的响应（非必需参数）
	harris.setNumberOfThreads(6);//初始化调度程序并设置要使用的线程数
	harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::LOWE);//设置要计算响应的方法（可以不设置）
	/////compute the source cloud Harris feature point/////
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	harris.setNormals(source_normals);//如果有预先计算的法线，则设置法线
	harris.setInputCloud(cloud_source);
	harris.compute(*Harris_keypoints_source);

	/////compute the Target cloud Harris feature point/////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	harris.setNormals(target_normals);//如果有预先计算的法线，则设置法线
	harris.setInputCloud(cloud_target);
	harris.compute(*Harris_keypoints_target);

	End = clock() - Start;
	cout << "the time of Feature point exaction " << End << endl;
	cout << "Harris_keypoints_source的大小是" << Harris_keypoints_source->size() << endl;
	cout << "Harris_keypoints_Target的大小是" << Harris_keypoints_target->size() << endl;

	time_t Start1, End1;
	Start1 = clock();
	//构建目标点云的特征点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature->resize(Harris_keypoints_source->size());
	for (int i = 0; i < Harris_keypoints_source->size(); i++)
	{
		cloud_feature->points[i].x = Harris_keypoints_source->points[i].x;
		cloud_feature->points[i].y = Harris_keypoints_source->points[i].y;
		cloud_feature->points[i].z = Harris_keypoints_source->points[i].z;
	}
	//构建特征点云

	//构建目标点云的特征点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_target(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature_target->resize(Harris_keypoints_target->size());
	for (int i = 0; i < Harris_keypoints_target->size(); i++)
	{
		cloud_feature_target->points[i].x = Harris_keypoints_target->points[i].x;
		cloud_feature_target->points[i].y = Harris_keypoints_target->points[i].y;
		cloud_feature_target->points[i].z = Harris_keypoints_target->points[i].z;
	}
	//构建特征点云
	//cout << "AnglePointIdx.size()" << Harris_keypoints_target->size() << endl;

	///计算源点云特征点的FPFH特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source_feature_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_feature); //设置输入点云
	fpfh.setInputNormals(source_normals); //设置输入点云的法线
	fpfh.setSearchSurface(cloud_source);
	fpfh.compute(*fpfh_source_feature_ptr); //计算每个点的pfh
	//cout << "finish source feature" << endl;
	//目标点云的FPFH特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_target_feature_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_feature_target); //设置输入点云
	fpfh.setInputNormals(target_normals); //设置输入点云的法线
	fpfh.setSearchSurface(cloud_target);
	fpfh.compute(*fpfh_target_feature_ptr); //计算每个点的pfh
	cout << "finish target feature" << endl;

	int k_search = 3;
	pcl::search::KdTree<pcl::FPFHSignature33> fpfh_tree;

	fpfh_tree.setInputCloud(fpfh_target_feature_ptr);
	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	pcl::PointXYZ searchPoint;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 10000;
	//期待所达到的误差中断循环
	float respect_err = 0.1;
	//float respect_err = 0.0000000001;
	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4 * radius;

	Eigen::Matrix4d final_transformation_;
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_feature;

	kdtree_feature.setInputCloud(cloud_feature);
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();

		int id = rand() % cloud_feature->points.size();
		sample_idx.push_back(id);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			//id = rand() % AnglePointIdx.size();

			id = rand() % cloud_feature->points.size();
			//searchPoint.x = cloud_feature->points[id].x;
			//searchPoint.y = cloud_feature->points[id].y;
			//searchPoint.z = cloud_feature->points[id].z;
			//kdtree_feature.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target);
			////cout << "pointIdxRadiusSearch_target.size()" << pointIdxRadiusSearch_target.size() << endl;
			//if (pointIdxRadiusSearch_target.size()<4)
			//{
			//	continue;
			//}
			//cout << "enter" << endl;
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_feature->at(id), cloud_feature->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(id);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				//id = rand() % AnglePointIdx.size();
				id = rand() % cloud_feature->points.size();

				sample_idx.push_back(id);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			fpfh_tree.nearestKSearch(*fpfh_source_feature_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}
		/*for (size_t i = 0; i < k_search; i++)
		{

		cout << "distance of k_search" <<i<<":"<< pfh_distance[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			/*if (id == 0)
			{
			final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
			final_match[1][i] = matches[i].target_idx[id];
			}*/
			final_match[1][i] = matches[i].target_idx[id];

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;

		pcl::transformPointCloud(*cloud_feature, *new_source, T.matrix());
		//cout << "77" << endl;

		error = computeError(new_source, cloud_feature_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		if (lowest_error < respect_err)
		{
			cout << "迭代次数：" << i_iter << endl;
			break;
		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "part error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);
	error = computeError(cloud_source, cloud_target);
	cout << "error:" << error << endl;
	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
	//void computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename)

	float radius;
	int k_number;

	cout << "input radius/k_nubmer of pfh_search: ";
	cin >> radius;
	//cin >> k_number;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(10);
	fpfh.setSearchMethod(tree); //设置近邻搜索方式
	fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	Start = clock();
	//pcl::PointCloud<pcl::Normal>::Ptr source_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>& source_normals = *source_normals_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_source_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_source); //设置输入点云
	fpfh.setInputNormals(source_normals); //设置输入点云的法线
	fpfh.compute(*pfh_source_ptr); //计算每个点的pfh										
								  //End = clock() - Start;
								  //std::cout << "PFH Time is :" << End << std::endl;
								  ////########计算Source点云############////

								  ////########计算Target点云############////
								  //pcl::PointCloud<pcl::Normal>::Ptr target_normals_ptr(new pcl::PointCloud<pcl::Normal>);
								  //pcl::PointCloud<pcl::Normal>& target_normals = *target_normals_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_target_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_target); //设置输入点云
	fpfh.setInputNormals(target_normals); //设置输入点云的法线
	fpfh.compute(*pfh_target_ptr); //计算每个点的pfh
	
	////########计算Target点云############////

	//计算所有点的特征平均
	compute_FPFH_u_high(pfh_source_ptr);
	//
	compute_FPFH_L2_dis(pfh_source_ptr);
	////////////////L2排序///////////////
	pfh_sort_num = new int[cloud_source->points.size()];
	for (int i = 0; i < cloud_source->points.size(); i++)
	{
	pfh_sort_num[i] = i;
	}
								  
	quick_sort(L2_dis, pfh_sort_num, 0, cloud_source->points.size() - 1);
	End = clock() - Start;
	std::cout << "Feature Time is :" << End << std::endl;
	////////////////L2排序///////////////

	int k_search = 3;
	pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	std::vector<int> pfh_index;
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 100;
	//bunny
	//float min_distance = 0.03;
	//float respect_err = 0.00000000001;
	//amo
	float min_distance = 4;
	float respect_err = 0.1;

	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	
	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();
		//final_match.clear();
		//for (int j = cloud_source->points.size() - 1; j > cloud_source->points.size() - REPR_NUM - 1; j--)
		//any points
		int id = rand() % cloud_source->points.size();
		//set the point of REPR_NUM
		//int id = pfh_sort_num[cloud_source->points.size() - 1-rand() % REPR_NUM];
		sample_idx.push_back(id);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			//any points
			id = rand() % cloud_source->points.size();
			//set the point of REPR_NUM
			// id = pfh_sort_num[cloud_source->points.size() - 1 - rand() % REPR_NUM];
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_source->at(id), cloud_source->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(id);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				//any points
				id = rand() % cloud_source->points.size();
				//set the point of REPR_NUM
				//id = pfh_sort_num[cloud_source->points.size() - 1 - rand() % REPR_NUM];
				sample_idx.push_back(id);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		for (size_t i = 0; i < 3; i++)
		{
			/*for (int j = 0; j < 125; i++)
			{
			search_pfh.histogram[j] = pfh_source_ptr->points[sample_idx[i]].histogram[j];
			}*/
			pfh_tree.nearestKSearch(*pfh_source_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}

		/*cout << "finished k_search" << endl;
		for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << matches[i].source_idx << endl;
		}*/

		//#############为随机的sample_size个点找到k_search个近似的点##############

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			/*if (id == 0)
			{
				final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
				final_match[1][i] = matches[i].target_idx[id];
			}*/
			final_match[1][i] = matches[i].target_idx[id];
		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;
		pcl::transformPointCloud(*cloud_source, *new_source, T.matrix());
		//cout << "77" << endl;
		error = computeError(new_source, cloud_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		if (lowest_error < respect_err)
		{
			cout << "迭代次数：" << i_iter << endl;
			break;

		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	std::cout << "REP Time is :" << End1 << std::endl;
	cout << "error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);


	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result1.ply", final);

	system("pause");

}
void Sac_ia_FPFH_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float u_rad)
{
	float radius1 = 2.5*u_rad;
	float radius = 0.16;
	//void computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename)

//	float radius;
	int k_number;

	//cout << "input radius/k_nubmer of pfh_search: ";
	//cin >> radius;
	//cin >> k_number;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(17);
	fpfh.setSearchMethod(tree); //设置近邻搜索方式
	fpfh.setRadiusSearch(1.85*radius);
	//fpfh.setRadiusSearch(8);
	////########计算Source点云############////
	time_t Start, End;
	Start = clock();
	//pcl::PointCloud<pcl::Normal>::Ptr source_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>& source_normals = *source_normals_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_source_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_source); //设置输入点云
	fpfh.setInputNormals(source_normals); //设置输入点云的法线
	fpfh.compute(*pfh_source_ptr); //计算每个点的pfh										
								   //End = clock() - Start;
								   //std::cout << "PFH Time is :" << End << std::endl;
								   ////########计算Source点云############////

	////########计算Target点云############////
								   //pcl::PointCloud<pcl::Normal>::Ptr target_normals_ptr(new pcl::PointCloud<pcl::Normal>);
								   //pcl::PointCloud<pcl::Normal>& target_normals = *target_normals_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_target_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_target); //设置输入点云
	fpfh.setInputNormals(target_normals); //设置输入点云的法线
	fpfh.compute(*pfh_target_ptr); //计算每个点的pfh

	////########计算Target点云############////

								   //计算所有点的特征平均
	compute_FPFH_u_high(pfh_source_ptr);
	//
	compute_FPFH_L2_dis(pfh_source_ptr);
	////////////////L2排序///////////////
	pfh_sort_num = new int[cloud_source->points.size()];
	for (int i = 0; i < cloud_source->points.size(); i++)
	{
		pfh_sort_num[i] = i;
	}

	quick_sort(L2_dis, pfh_sort_num, 0, cloud_source->points.size() - 1);

	
	////////////////L2排序///////////////

	//特征源点云构建
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature->resize(REPR_NUM);
	int flag = 0;
	for (int  i= cloud_source->points.size() - 1; i > cloud_source->points.size() - REPR_NUM - 1; i--)
	{

		cloud_feature->points[flag].x = cloud_source->points[pfh_sort_num[i]].x;
		cloud_feature->points[flag].y = cloud_source->points[pfh_sort_num[i]].y;
		cloud_feature->points[flag].z = cloud_source->points[pfh_sort_num[i]].z;
		flag++;
	}
	End = clock() - Start;
	std::cout << "Feature Time is :" << End << std::endl;

	//构建特征点云
	cout << "AnglePointIdx.size()" << REPR_NUM << endl;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_source_feature_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_feature); //设置输入点云
	fpfh.setInputNormals(source_normals); //设置输入点云的法线
	fpfh.setSearchSurface(cloud_source);
	fpfh.compute(*fpfh_source_feature_ptr); //计算每个点的pfh
											//cout << "finish source feature" << endl;
	compute_FPFH_u_high(pfh_target_ptr);
	//
	compute_FPFH_L2_dis(pfh_target_ptr);
	////////////////L2排序///////////////
	//pfh_sort_num = new int[cloud_target->points.size()];
	for (int i = 0; i < cloud_target->points.size(); i++)
	{
		pfh_sort_num[i] = i;
	}

	quick_sort(L2_dis, pfh_sort_num, 0, cloud_source->points.size() - 1);
	////
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_target(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature_target->resize(REPR_NUM);
	flag = 0;
	for (int i = cloud_target->points.size() - 1; i > cloud_target->points.size() - REPR_NUM - 1; i--)
	{
		cloud_feature_target->points[flag].x = cloud_target->points[pfh_sort_num[i]].x;
		cloud_feature_target->points[flag].y = cloud_target->points[pfh_sort_num[i]].y;
		cloud_feature_target->points[flag].z = cloud_target->points[pfh_sort_num[i]].z;
		flag++;
	}
	//目标点云的FPFH特征
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_target_feature_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_feature_target); //设置输入点云
	fpfh.setInputNormals(target_normals); //设置输入点云的法线
	fpfh.setSearchSurface(cloud_target);
	fpfh.compute(*fpfh_target_feature_ptr); //计算每个点的pfh
											//cout << "finish target feature" << endl;



	time_t Start1, End1;
	Start1 = clock();

	int k_search = 3;
	pcl::search::KdTree<pcl::FPFHSignature33> fpfh_tree;

	fpfh_tree.setInputCloud(fpfh_target_feature_ptr);
	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	pcl::PointXYZ searchPoint;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 10000;
	//期待所达到的误差中断循环
	float respect_err = 0.1;
	//float respect_err = 0.00001;
	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4 * radius;

	Eigen::Matrix4d final_transformation_;


	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_feature;

	kdtree_feature.setInputCloud(cloud_feature);
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();

		int id = rand() % cloud_feature->points.size();
		sample_idx.push_back(id);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			//id = rand() % AnglePointIdx.size();

			id = rand() % cloud_feature->points.size();
			//searchPoint.x = cloud_feature->points[id].x;
			//searchPoint.y = cloud_feature->points[id].y;
			//searchPoint.z = cloud_feature->points[id].z;
			//kdtree_feature.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target);
			////cout << "pointIdxRadiusSearch_target.size()" << pointIdxRadiusSearch_target.size() << endl;
			//if (pointIdxRadiusSearch_target.size()<4)
			//{
			//	continue;
			//}
			//cout << "enter" << endl;
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_feature->at(id), cloud_feature->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(id);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				//id = rand() % AnglePointIdx.size();
				id = rand() % cloud_feature->points.size();

				sample_idx.push_back(id);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			fpfh_tree.nearestKSearch(*fpfh_source_feature_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}
		/*for (size_t i = 0; i < k_search; i++)
		{

		cout << "distance of k_search" <<i<<":"<< pfh_distance[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			/*if (id == 0)
			{
			final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
			final_match[1][i] = matches[i].target_idx[id];
			}*/
			final_match[1][i] = matches[i].target_idx[id];

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;

		pcl::transformPointCloud(*cloud_feature, *new_source, T.matrix());
		//cout << "77" << endl;

		error = computeError(new_source, cloud_feature_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		if (lowest_error < respect_err)
		{
			cout << "迭代次数：" << i_iter << endl;
			break;

		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "part error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);
	error = computeError(cloud_source, cloud_target);
	cout << "error:" << error << endl;
	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");


}
void Sac_ia_ANGLE_BUFF(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
	//void computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename)

	float radius;
	int k_number;

	cout << "input radius/k_nubmer of pfh_search: ";
	cin >> radius;
	//cin >> k_number;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(10);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	Start = clock();
	//pcl::PointCloud<pcl::Normal>::Ptr source_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>& source_normals = *source_normals_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
		/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle_sum = 0;
	float more_threshod;
	float less_threshod;
	float k = 0.15;
	float u_radius;
	float sear_rad;
	float stand_distance;
	long counter = 0;
	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			/*for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}*/
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];
				//cout << "stand_distance 2:" << stand_distance << endl;
			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];
				//cout << "stand_distance 1:" << stand_distance << endl;
			}
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					
					if (pointRadiusSquaredDistance[i]<(1.5*stand_distance))
					{
						//cout << "distance" << " " << counter << ":" << pointRadiusSquaredDistance[i] << endl;
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;

			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				sear_rad = 1.2*u_radius * 1000;
				//sear_rad = 1.2*u_radius ;
				counter = 0;

			}

		}
		//bunny
		if (sear_rad<0.0015)
		{
			sear_rad = 0.0015;
		}
		//amo
		/*if (sear_rad<0.35)
		{
			sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					angle_sum = angle_sum + angle1;
					if (angle1>15) {
						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad/1000-u_radius, 2)));
						//more_threshod++;
					}
					else
					{
						less_threshod= less_threshod+ 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						//less_threshod++;
					}

				}
			}
			more_threshod_idx.push_back(more_threshod);
			less_threshod_idx.push_back(less_threshod);
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			if (more_threshod >= less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;

		}
	}
	//compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
	//cout << "angle sort1 is ready" << endl;
	//cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	cout << "finished source" << endl;

	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target;
	float less_threshod_target;
	float k_target = 0.15;
	float u_radius_target;
	float sear_rad_target;
	float stand_distance_target;
	long counter_target;
	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			/*for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}*/
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[i]<(1.5*stand_distance_target))
					{
						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				sear_rad_target = 1.2*u_radius_target * 1000;
				//sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}

		}
		if (sear_rad_target<0.0015)
		{
			sear_rad_target = 0.0015;
		}
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>15) {
						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target /1000-u_radius_target, 2)));
						//more_threshod_target++;
					}
					else
					{
						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target / 1000 - u_radius_target, 2)));
						//less_threshod_target++;
					}
				}
			}
			more_threshod_target_idx.push_back(more_threshod_target);
			less_threshod_target_idx.push_back(less_threshod_target);
			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			if (more_threshod_target >= less_threshod_target)
			{
				if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					AnglePointIdx_target.push_back(j);
				}
			}
			else
			{
				if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					FacePointIdx_target.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	//cout << "AnglePointIdx.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////

	int k_search = 3;
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 10;

	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 0.02;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();
		//final_match.clear();
		int id = rand() % AnglePointIdx.size();
		sample_idx.push_back(AnglePointIdx[id]);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			id = rand() % AnglePointIdx.size();
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_source->at(AnglePointIdx[id]), cloud_source->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				id = rand() % AnglePointIdx.size();
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			for (size_t j = 0; j < AnglePointIdx_target.size(); j++)
			{
				if (pfh_index.size()<3)
				{
					pfh_index[idx_cout] = AnglePointIdx_target[j];
					idx_cout++;
				}
				else
				{
					//第J个目标的角点与第I个sample点的差距
					
					//dx1 = pointAngle_target[AnglePointIdx_target[j]] - pointAngle[sample_idx[i]];
					dx1 = sqrt(pow((more_threshod_target_idx[AnglePointIdx_target[j]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[AnglePointIdx_target[j]] - less_threshod_idx[sample_idx[i]]), 2));
					for (size_t k = 0; k < pfh_index.size(); k++)
					{
						//第K个要找的对应点与第I个sample点的差距
						dx2 = sqrt(pow((more_threshod_target_idx[pfh_index[k]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[pfh_index[k]] - less_threshod_idx[sample_idx[i]]), 2));
						//dx2 = pointAngle_target[pfh_index[k]] - pointAngle[sample_idx[i]];
						if (dx1<dx2) {
							pfh_index[k] = AnglePointIdx_target[j];
						}
					}
				}
			}

			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}

		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			if (id == 0)
			{
				final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
				final_match[1][i] = matches[i].target_idx[id];
			}

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;
		pcl::transformPointCloud(*cloud_source, *new_source, T.matrix());
		//cout << "77" << endl;
		error = computeError(new_source, cloud_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);

	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_ANGLE_AMO(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
	//void computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename)

	float radius;
	int k_number;

	cout << "input radius/k_nubmer of angle_search: ";
	cin >> radius;
	//cin >> k_number;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(10);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	Start = clock();
	//pcl::PointCloud<pcl::Normal>::Ptr source_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>& source_normals = *source_normals_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle_sum = 0;
	float more_threshod;
	float less_threshod;
	float k = 0.15;
	float u_radius;
	float sear_rad;
	float stand_distance;
	long counter = 0;
	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];
				//cout << "stand_distance 2:" << stand_distance << endl;
			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];
				//cout << "stand_distance 1:" << stand_distance << endl;
			}
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {

					if (pointRadiusSquaredDistance[i]<(1.5*stand_distance))
					{
						//cout << "distance" << " " << counter << ":" << pointRadiusSquaredDistance[i] << endl;
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;

			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				//sear_rad = 1.2*u_radius * 1000;
				sear_rad = 1.2*u_radius;
				counter = 0;

			}

		}
		//bunny
		/*if (sear_rad<0.0015)
		{
		sear_rad = 0.0015;
		}*/
		//amo
		/*if (sear_rad<0.35)
		{
		sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					angle_sum = angle_sum + angle1;
					if (angle1>15) {
						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						//more_threshod++;
					}
					else
					{
						less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						//less_threshod++;
					}

				}
			}
			more_threshod_idx.push_back(more_threshod);
			less_threshod_idx.push_back(less_threshod);
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			if (more_threshod >= less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;

		}
	}
	//compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
	//cout << "angle sort1 is ready" << endl;
	//cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	//cout << "finished source" << endl;

	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target;
	float less_threshod_target;
	float k_target = 0.15;
	float u_radius_target;
	float sear_rad_target;
	float stand_distance_target;
	long counter_target;
	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[i]<(1.5*stand_distance_target))
					{
						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				//sear_rad_target = 1.2*u_radius_target * 1000;
				sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}

		}
		/*if (sear_rad_target<0.0015)
		{
		sear_rad_target = 0.0015;
		}*/
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>15) {
						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target / 1000 - u_radius_target, 2)));
						//more_threshod_target++;
					}
					else
					{
						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target / 1000 - u_radius_target, 2)));
						//less_threshod_target++;
					}
				}
			}
			more_threshod_target_idx.push_back(more_threshod_target);
			less_threshod_target_idx.push_back(less_threshod_target);
			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			if (more_threshod_target >= less_threshod_target)
			{
				if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					AnglePointIdx_target.push_back(j);
				}
			}
			else
			{
				if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					FacePointIdx_target.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	//cout << "AnglePointIdx.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////

	int k_search = 3;
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 100;

	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();
		//final_match.clear();
		int id = rand() % AnglePointIdx.size();
		sample_idx.push_back(AnglePointIdx[id]);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			id = rand() % AnglePointIdx.size();
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_source->at(AnglePointIdx[id]), cloud_source->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				id = rand() % AnglePointIdx.size();
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			for (size_t j = 0; j < AnglePointIdx_target.size(); j++)
			{
				if (pfh_index.size()<3)
				{
					pfh_index[idx_cout] = AnglePointIdx_target[j];
					idx_cout++;
				}
				else
				{
					//第J个目标的角点与第I个sample点的差距

					//dx1 = pointAngle_target[AnglePointIdx_target[j]] - pointAngle[sample_idx[i]];
					dx1 = sqrt(pow((more_threshod_target_idx[AnglePointIdx_target[j]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[AnglePointIdx_target[j]] - less_threshod_idx[sample_idx[i]]), 2));
					for (size_t k = 0; k < pfh_index.size(); k++)
					{
						//第K个要找的对应点与第I个sample点的差距
						dx2 = sqrt(pow((more_threshod_target_idx[pfh_index[k]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[pfh_index[k]] - less_threshod_idx[sample_idx[i]]), 2));
						//dx2 = pointAngle_target[pfh_index[k]] - pointAngle[sample_idx[i]];
						if (dx1<dx2) {
							pfh_index[k] = AnglePointIdx_target[j];
						}
					}
				}
			}

			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}

		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			if (id == 0)
			{
				final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
				final_match[1][i] = matches[i].target_idx[id];
			}

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;
		pcl::transformPointCloud(*cloud_source, *new_source, T.matrix());
		//cout << "77" << endl;
		error = computeError(new_source, cloud_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);

	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_ANGLE_FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
	//void computePFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, string filename)

	float radius;
	int k_number;

	cout << "input radius/k_nubmer of pfh_search: ";
	cin >> radius;
	//cin >> k_number;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(10);
	fpfh.setSearchMethod(tree); //设置近邻搜索方式
	fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	Start = clock();
	//pcl::PointCloud<pcl::Normal>::Ptr source_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	//pcl::PointCloud<pcl::Normal>& source_normals = *source_normals_ptr;
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_source_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_source); //设置输入点云
	fpfh.setInputNormals(source_normals); //设置输入点云的法线
	fpfh.compute(*pfh_source_ptr);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	float angle;
	float angle1;
	float angle_sum = 0;
	float more_threshod;
	float less_threshod;
	float k = 0.15;
	float u_radius;
	float sear_rad;
	float stand_distance;
	long counter;
	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
					{
						stand_distance = pointRadiusSquaredDistance[2];
					}
					else
					{
						stand_distance = pointRadiusSquaredDistance[1];
					}
					if (pointRadiusSquaredDistance[i]<(1.5*stand_distance))
					{
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;

			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				sear_rad = 1.2*u_radius * 1000;
				counter = 0;

			}

		}
		if (sear_rad<0.0015)
		{
			sear_rad = 0.0015;
		}
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					angle_sum = angle_sum + angle1;
					if (angle1>15) {
						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad/1000-u_radius, 2)));
						more_threshod++;
					}
					else
					{
						//less_threshod= less_threshod+ 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod++;
					}
				}
			}
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			if (more_threshod >= less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;

		}
	}
	//compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
	//cout << "angle sort1 is ready" << endl;
	//cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;


	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_target_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //声明一个数据结构，来存储每个点的pfh
	fpfh.setInputCloud(cloud_target); //设置输入点云
	fpfh.setInputNormals(target_normals); //设置输入点云的法线
	fpfh.compute(*pfh_target_ptr); //计算每个点的pfh
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target;
	float less_threshod_target;
	float k_target = 0.15;
	float u_radius_target;
	float sear_rad_target;
	float stand_distance_target;
	long counter_target;
	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
					{
						stand_distance_target = pointRadiusSquaredDistance_target[2];
					}
					else
					{
						stand_distance_target = pointRadiusSquaredDistance_target[1];
					}
					if (pointRadiusSquaredDistance_target[i]<(1.5*stand_distance_target))
					{
						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				sear_rad_target = 1.2*u_radius_target * 1000;
				counter_target = 0;

			}

		}
		if (sear_rad_target<0.0015)
		{
			sear_rad_target = 0.0015;
		}
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>15) {
						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad/1000-u_radius, 2)));
						more_threshod_target++;
					}
					else
					{
						//less_threshod= less_threshod+ 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod_target++;
					}
				}
			}
			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			if (more_threshod_target >= less_threshod_target)
			{
				if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					AnglePointIdx_target.push_back(j);
				}
			}
			else
			{
				if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					FacePointIdx_target.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	//cout << "AnglePointIdx.size()" << AnglePointIdx_target.size() << endl;
	End = clock() - Start;
	std::cout << "Feature Time is :" << End << std::endl;
	////########计算Target点云############////

	int k_search = 3;
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 100;

	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 0.02;

	Eigen::Matrix4d final_transformation_;
	pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	pfh_tree.setInputCloud(pfh_target_ptr);
	time_t Start1, End1;
	Start1 = clock();
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();
		//final_match.clear();
		int id = rand() % 300;
		sample_idx.push_back(AnglePointIdx[id]);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			id = rand() % 300;
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_source->at(AnglePointIdx[id]), cloud_source->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				id = rand() % 300;
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		//int idx_cout = 0;
		//float dx1, dx2;
		//for (size_t i = 0; i < 3; i++)
		//{

		//	for (size_t j = 0; j < AnglePointIdx_target.size(); j++)
		//	{
		//		if (pfh_index.size()<3)
		//		{
		//			pfh_index[idx_cout] = AnglePointIdx_target[j];
		//			idx_cout++;
		//		}
		//		else
		//		{
		//			//第J个目标的角点与第I个sample点的差距
		//			dx1 = pointAngle_target[AnglePointIdx_target[j]] - sample_idx[i];
		//			for (size_t k = 0; k < pfh_index.size(); k++)
		//			{
		//				//第K个要找的对应点与第I个sample点的差距
		//				dx2 = pointAngle_target[pfh_index[k]] - sample_idx[i];
		//				if (dx1<dx2) {
		//					pfh_index[k] = AnglePointIdx_target[j];
		//				}
		//			}
		//		}
		//	}

		//	matches[i].source_idx = sample_idx[i];
		//	matches[i].target_idx = pfh_index;
		//}
		
		for (size_t i = 0; i < 3; i++)
		{
			/*for (int j = 0; j < 125; i++)
			{
			search_pfh.histogram[j] = pfh_source_ptr->points[sample_idx[i]].histogram[j];
			}*/
			pfh_tree.nearestKSearch(*pfh_source_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}


		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			if (id == 0)
			{
				final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
				final_match[1][i] = matches[i].target_idx[id];
			}

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;
		pcl::transformPointCloud(*cloud_source, *new_source, T.matrix());
		//cout << "77" << endl;
		error = computeError(new_source, cloud_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	std::cout << "REP Time is :" << End1 << std::endl;
	cout << "error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);

	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_ANGLE_AUTO(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,float u_rad)
{
	float radius = 2.5*u_rad;
	//float radius = 1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(10);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	Start = clock();
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle_sum = 0;
	float more_threshod=0;
	float less_threshod=0;
	float k = 0.15;
	float u_radius=0;
	float sear_rad=0;
	float stand_distance;
	long counter = 0;

	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];
			

			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];
		
			}
			//cout << "stand_distance：" << stand_distance << endl;
			//cout << "size:" << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				//cout << "point_distance：" << pointRadiusSquaredDistance[i] << endl;
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					//计算1-1.5倍区域点的距离平均
					//&& (pointRadiusSquaredDistance[i]>stand_distance)
					if (pointRadiusSquaredDistance[i]<=(1.5*stand_distance))
					//if (pointRadiusSquaredDistance[i]<=(1.5*stand_distance) && (pointRadiusSquaredDistance[i]>=stand_distance))
					{
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;			
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;
			cout << "u_radius:" << u_radius << endl;
			cout << "counter" << counter << endl;
			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				//sear_rad = 1.2*u_radius * 1000;
				sear_rad = 1.2*u_radius;
				counter = 0;

			}
			cout << "sear_rad:" << sear_rad << endl;

		}
		//bunny
		/*if (sear_rad<0.0015)
		{
		sear_rad = 0.0015;
		}*/
		//amo
		/*if (sear_rad<0.35)
		{
		sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		//cout << "pointIdxRadiusSearch"<<pointIdxRadiusSearch.size() << endl;
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					angle_sum = angle_sum + angle1;
					if (angle1>15) {
						
						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						//more_threshod++;
					}
					else
					{
	
						//less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						
						//less_threshod++;
					}

				}
			}

			more_threshod_idx.push_back(more_threshod);
			less_threshod_idx.push_back(less_threshod);
		
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			if (more_threshod >= less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;

		}
	}
	//compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
	//cout << "angle sort1 is ready" << endl;
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	cout << "finished source" << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_b->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_b->points[i].x = cloud_source->points[i].x;
		cloud_b->points[i].y = cloud_source->points[i].y;
		cloud_b->points[i].z = cloud_source->points[i].z;
	}

	//Eigen::Vector3d d, n1, n2;

	cout << "ready cloud" << endl;

	//printf("pfh size %d", pfh_fe_ptr->points.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		//cout << "angle:" << AnglePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_b->at(AnglePointIdx[i]).r = 255;
		cloud_b->at(AnglePointIdx[i]).g = 0;
		cloud_b->at(AnglePointIdx[i]).b = 0;
	}
	for (int i = 0; i < BoundPointIdx.size(); i++)
	{
		//cout << "bound:" << BoundPointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_b->at(BoundPointIdx[i]).r = 0;
		cloud_b->at(BoundPointIdx[i]).g = 255;
		cloud_b->at(BoundPointIdx[i]).b = 0;
	}
	for (int i = 0; i < FacePointIdx.size(); i++)
	{
		//cout << "face:" << FacePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_b->at(FacePointIdx[i]).r = 100;
		cloud_b->at(FacePointIdx[i]).g = 100;
		cloud_b->at(FacePointIdx[i]).b = 100;
	}
	cout << "总数：" << FacePointIdx.size() + BoundPointIdx.size() + AnglePointIdx.size();
	//savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
	pcl::PointCloud<pcl::PointXYZRGB> final2 = *cloud_b;
	pcl::io::savePLYFile("source_cloud.ply", final2);
	cout << "ok" << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target=0;
	float less_threshod_target=0;
	float k_target = 0.15;
	float u_radius_target=0;
	float sear_rad_target=0;
	float stand_distance_target;
	long counter_target=0;

	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); i++)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[i] <= (1.5*stand_distance_target))
					//if (pointRadiusSquaredDistance_target[i]<=(1.5*stand_distance_target) && (pointRadiusSquaredDistance_target[i] >= stand_distance_target))
					{

						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				//sear_rad_target = 1.2*u_radius_target * 1000;
				sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}
	

		}
		/*if (sear_rad_target<0.0015)
		{
		sear_rad_target = 0.0015;
		}*/
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
				{
					pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
				}
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>15) {

						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));
						
						//more_threshod_target++;
					}
					else
					{
						
						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));
						
						//less_threshod_target++;
					}
				}
			}

			more_threshod_target_idx.push_back(more_threshod_target);
			less_threshod_target_idx.push_back(less_threshod_target);

			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			if (more_threshod_target >= less_threshod_target)
			{
				if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					AnglePointIdx_target.push_back(j);
				}
			}
			else
			{
				if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					FacePointIdx_target.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	cout << "AnglePointIdx_target.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_c->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_c->points[i].x = cloud_target->points[i].x;
		cloud_c->points[i].y = cloud_target->points[i].y;
		cloud_c->points[i].z = cloud_target->points[i].z;
	}

	//Eigen::Vector3d d, n1, n2;

	cout << "ready cloud" << endl;

	//printf("pfh size %d", pfh_fe_ptr->points.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		//cout << "angle:" << AnglePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_c->at(AnglePointIdx_target[i]).r = 255;
		cloud_c->at(AnglePointIdx_target[i]).g = 0;
		cloud_c->at(AnglePointIdx_target[i]).b = 0;
	}
	for (int i = 0; i < BoundPointIdx_target.size(); i++)
	{
		//cout << "bound:" << BoundPointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_c->at(BoundPointIdx_target[i]).r = 0;
		cloud_c->at(BoundPointIdx_target[i]).g = 255;
		cloud_c->at(BoundPointIdx_target[i]).b = 0;
	}
	for (int i = 0; i < FacePointIdx_target.size(); i++)
	{
		//cout << "face:" << FacePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_c->at(FacePointIdx_target[i]).r = 100;
		cloud_c->at(FacePointIdx_target[i]).g = 100;
		cloud_c->at(FacePointIdx_target[i]).b = 100;
	}
	cout << "总数：" << FacePointIdx_target.size() + BoundPointIdx_target.size() + AnglePointIdx_target.size();
	//savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
	pcl::PointCloud<pcl::PointXYZRGB> final1 = *cloud_c;
	pcl::io::savePLYFile("source_target.ply", final1);
	cout << "ok" << endl;





	int k_search = 3;
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 20;

	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();
		//final_match.clear();
		int id = rand() % AnglePointIdx.size();
		sample_idx.push_back(AnglePointIdx[id]);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			id = rand() % AnglePointIdx.size();
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_source->at(AnglePointIdx[id]), cloud_source->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				id = rand() % AnglePointIdx.size();
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			for (size_t j = 0; j < AnglePointIdx_target.size(); j++)
			{
				if (pfh_index.size()<3)
				{
					pfh_index[idx_cout] = AnglePointIdx_target[j];
					idx_cout++;
				}
				else
				{
					//第J个目标的角点与第I个sample点的差距

					//dx1 = pointAngle_target[AnglePointIdx_target[j]] - pointAngle[sample_idx[i]];
					dx1 = sqrt(pow((more_threshod_target_idx[AnglePointIdx_target[j]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[AnglePointIdx_target[j]] - less_threshod_idx[sample_idx[i]]), 2));
					for (size_t k = 0; k < pfh_index.size(); k++)
					{
						//第K个要找的对应点与第I个sample点的差距
						dx2 = sqrt(pow((more_threshod_target_idx[pfh_index[k]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[pfh_index[k]] - less_threshod_idx[sample_idx[i]]), 2));
						//dx2 = pointAngle_target[pfh_index[k]] - pointAngle[sample_idx[i]];
						if (dx1<dx2) {
							pfh_index[k] = AnglePointIdx_target[j];
						}
					}
				}
			}

			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}

		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			if (id == 0)
			{
				final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
				final_match[1][i] = matches[i].target_idx[id];
			}

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;
		pcl::transformPointCloud(*cloud_source, *new_source, T.matrix());
		//cout << "77" << endl;
		error = computeError(new_source, cloud_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);

	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_ANGLE_AUTO_noise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float u_rad)
{
	float radius = 2.5*u_rad;
	//float radius = 1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(20);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	Start = clock();
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle2;
	float angle_sum = 0;
	float more_threshod = 0;
	float less_threshod = 0;
	float k = 0.15;
	float u_radius = 0;
	float sear_rad = 0;
	float stand_distance;
	long counter = 0;

	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];


			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];

			}
			//cout << "stand_distance：" << stand_distance << endl;
			//cout << "size:" << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				//cout << "point_distance：" << pointRadiusSquaredDistance[i] << endl;
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					//计算1-1.5倍区域点的距离平均
					//&& (pointRadiusSquaredDistance[i]>stand_distance)
					if (pointRadiusSquaredDistance[i] <= (1.5*stand_distance))
						//if (pointRadiusSquaredDistance[i]<=(1.5*stand_distance) && (pointRadiusSquaredDistance[i]>=stand_distance))
					{
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;
			//cout << "u_radius:" << u_radius << endl;
			//cout << "counter" << counter << endl;
			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				//sear_rad = 1.2*u_radius * 1000;
				sear_rad = 1.2*u_radius;
				counter = 0;

			}
			//cout << "sear_rad:" << sear_rad << endl;

		}
		//bunny
		/*if (sear_rad<0.0015)
		{
		sear_rad = 0.0015;
		}*/
		//amo
		/*if (sear_rad<0.35)
		{
		sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		//cout << "pointIdxRadiusSearch"<<pointIdxRadiusSearch.size() << endl;
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					if (angle1>90)
					{
						angle1 = 180 - angle1;
					}
					angle_sum = angle_sum + angle1;
					/*if (pointRadiusSquaredDistance[i]<= sear_rad / 1.2)
					{
						angle2 = angle1;
						cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
							<< "distance:" << pointRadiusSquaredDistance[i]
							<< "w:1" <<"angle2:"<<angle2
							<< endl;
					}
					else
					{
						angle2 = angle1*exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
							<< "distance:" << pointRadiusSquaredDistance[i]
							<< "w:" << exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)))
							<< "angle2:" << angle2
							<< endl;
					}*/
					
					if (angle1>15) {

						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						//more_threshod++;
					}
					else
					{

						//less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));

						//less_threshod++;
					}

				}
			}

			more_threshod_idx.push_back(more_threshod);
			less_threshod_idx.push_back(less_threshod);

			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			/*if (more_threshod >= less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}*/
			//######### Newer vision #########

			//######### Laster vision #########
			if (more_threshod > 0 && less_threshod == 0)
			{
				AnglePointIdx.push_back(j);
			}
			if (more_threshod == 0 && less_threshod > 0)
			{
				FacePointIdx.push_back(j);
			}
			/*if (less_threshod / (more_threshod + less_threshod)>k|| more_threshod / (more_threshod + less_threshod)>k)
			{

			}*/
			if (more_threshod > 0 && less_threshod > 0)
			{
				BoundPointIdx.push_back(j);
			}
			/*else
			{
				BoundPointIdx.push_back(j);
			}*/

			//######### Laster vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;

		}
	}
	//cout << "start sort" << endl;
	//compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
	cout << "angle sort1 is ready" << endl;
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	cout << "finished source" << endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_b->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_b->points[i].x = cloud_source->points[i].x;
		cloud_b->points[i].y = cloud_source->points[i].y;
		cloud_b->points[i].z = cloud_source->points[i].z;
	}

	//Eigen::Vector3d d, n1, n2;

	cout << "ready cloud" << endl;

	//printf("pfh size %d", pfh_fe_ptr->points.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		//cout << "angle:" << AnglePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_b->at(AnglePointIdx[i]).r = 255;
		cloud_b->at(AnglePointIdx[i]).g = 0;
		cloud_b->at(AnglePointIdx[i]).b = 0;
	}
	for (int i = 0; i < BoundPointIdx.size(); i++)
	{
		//cout << "bound:" << BoundPointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_b->at(BoundPointIdx[i]).r = 0;
		cloud_b->at(BoundPointIdx[i]).g = 255;
		cloud_b->at(BoundPointIdx[i]).b = 0;
	}
	for (int i = 0; i < FacePointIdx.size(); i++)
	{
		//cout << "face:" << FacePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_b->at(FacePointIdx[i]).r = 100;
		cloud_b->at(FacePointIdx[i]).g = 100;
		cloud_b->at(FacePointIdx[i]).b = 100;
	}
	cout << "总数：" << FacePointIdx.size() + BoundPointIdx.size() + AnglePointIdx.size();
	//savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
	pcl::PointCloud<pcl::PointXYZRGB> final2 = *cloud_b;
	pcl::io::savePLYFile("source_cloud.ply", final2);
	cout << "ok" << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target = 0;
	float less_threshod_target = 0;
	float k_target = 0.15;
	float u_radius_target = 0;
	float sear_rad_target = 0;
	float stand_distance_target;
	long counter_target = 0;

	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); i++)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[i] <= (1.5*stand_distance_target))
						//if (pointRadiusSquaredDistance_target[i]<=(1.5*stand_distance_target) && (pointRadiusSquaredDistance_target[i] >= stand_distance_target))
					{

						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				//sear_rad_target = 1.2*u_radius_target * 1000;
				sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}


		}
		/*if (sear_rad_target<0.0015)
		{
		sear_rad_target = 0.0015;
		}*/
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
				{
					pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
				}
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					if (angle1_target>90)
					{
						angle1_target = 180 - angle1_target;
					}
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>15) {

						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//more_threshod_target++;
					}
					else
					{

						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//less_threshod_target++;
					}
				}
			}

			more_threshod_target_idx.push_back(more_threshod_target);
			less_threshod_target_idx.push_back(less_threshod_target);

			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			/*if (more_threshod_target >= less_threshod_target)
			{
				if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					AnglePointIdx_target.push_back(j);
				}
			}
			else
			{
				if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					FacePointIdx_target.push_back(j);
				}
			}*/
			if (more_threshod_target > 0 && less_threshod_target == 0)
			{
				AnglePointIdx_target.push_back(j);
			}
			if (more_threshod_target == 0 && less_threshod_target > 0)
			{
				FacePointIdx_target.push_back(j);
			}
			/*if (less_threshod / (more_threshod + less_threshod)>k|| more_threshod / (more_threshod + less_threshod)>k)
			{

			}*/
			if (more_threshod_target > 0 && less_threshod_target > 0)
			{
				BoundPointIdx_target.push_back(j);
			}
			/*else
			{
				BoundPointIdx_target.push_back(j);
			}*/
			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	cout << "AnglePointIdx_target.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_c->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_c->points[i].x = cloud_target->points[i].x;
		cloud_c->points[i].y = cloud_target->points[i].y;
		cloud_c->points[i].z = cloud_target->points[i].z;
	}

	//Eigen::Vector3d d, n1, n2;

	cout << "ready cloud" << endl;

	//printf("pfh size %d", pfh_fe_ptr->points.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		//cout << "angle:" << AnglePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_c->at(AnglePointIdx_target[i]).r = 255;
		cloud_c->at(AnglePointIdx_target[i]).g = 0;
		cloud_c->at(AnglePointIdx_target[i]).b = 0;
	}
	for (int i = 0; i < BoundPointIdx_target.size(); i++)
	{
		//cout << "bound:" << BoundPointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_c->at(BoundPointIdx_target[i]).r = 0;
		cloud_c->at(BoundPointIdx_target[i]).g = 255;
		cloud_c->at(BoundPointIdx_target[i]).b = 0;
	}
	for (int i = 0; i < FacePointIdx_target.size(); i++)
	{
		//cout << "face:" << FacePointIdx.size() << endl;
		//cout << "ready color" << endl;
		cloud_c->at(FacePointIdx_target[i]).r = 100;
		cloud_c->at(FacePointIdx_target[i]).g = 100;
		cloud_c->at(FacePointIdx_target[i]).b = 100;
	}
	cout << "总数：" << FacePointIdx_target.size() + BoundPointIdx_target.size() + AnglePointIdx_target.size();
	//savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
	pcl::PointCloud<pcl::PointXYZRGB> final1 = *cloud_c;
	pcl::io::savePLYFile("source_target.ply", final1);
	cout << "ok" << endl;

	int k_search = 3;
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 20;

	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();
		//final_match.clear();
		//int id = rand() % AnglePointIdx.size();
		//使用target和source最小值来确定
		int id = AnglePointIdx.size()-rand() % AnglePointIdx_target.size()-1;
		sample_idx.push_back(AnglePointIdx[id]);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			//id = rand() % AnglePointIdx.size();
			id = AnglePointIdx.size() - rand() % AnglePointIdx_target.size() - 1;
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_source->at(AnglePointIdx[id]), cloud_source->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				//id = rand() % AnglePointIdx.size();
				id = AnglePointIdx.size() - rand() % AnglePointIdx_target.size() - 1;
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			for (size_t j = 0; j < AnglePointIdx_target.size(); j++)
			{
				if (pfh_index.size()<3)
				{
					pfh_index[idx_cout] = AnglePointIdx_target[j];
					idx_cout++;
				}
				else
				{
					//第J个目标的角点与第I个sample点的差距

					//dx1 = pointAngle_target[AnglePointIdx_target[j]] - pointAngle[sample_idx[i]];
					dx1 = sqrt(pow((more_threshod_target_idx[AnglePointIdx_target[j]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[AnglePointIdx_target[j]] - less_threshod_idx[sample_idx[i]]), 2));
					for (size_t k = 0; k < pfh_index.size(); k++)
					{
						//第K个要找的对应点与第I个sample点的差距
						dx2 = sqrt(pow((more_threshod_target_idx[pfh_index[k]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[pfh_index[k]] - less_threshod_idx[sample_idx[i]]), 2));
						//dx2 = pointAngle_target[pfh_index[k]] - pointAngle[sample_idx[i]];
						if (dx1<dx2) {
							pfh_index[k] = AnglePointIdx_target[j];
						}
					}
				}
			}

			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}

		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			if (id == 0)
			{
				final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
				final_match[1][i] = matches[i].target_idx[id];
			}

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;
		pcl::transformPointCloud(*cloud_source, *new_source, T.matrix());
		//cout << "77" << endl;
		error = computeError(new_source, cloud_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);

	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}

void Sac_ia_rough_area_double_param_test(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float u_rad)
{
	float k;
	int thetol;
	//手动输入
	/*cout << "angle ";
	cin >> thetol;
	cout << "ratio ";
	cin >> k;*/
	//hetol = 17;
	thetol = 17;
	k = 0.15;
	
	float radius = 2.5*u_rad;
	//float radius = 1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(17);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	/*time_t Start_source_src, End_source_src;
	time_t Start_source_ext, End_source_ext;
	time_t Start_target_src, End_target_src;
	time_t Start_target_ext, End_target_ext;*/
	Start = clock();
	
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle2;
	float angle_sum = 0;
	float more_threshod = 0;
	float less_threshod = 0;
	//float k = 0.17;
	//int thetol = 17;
	float u_radius = 0;
	float sear_rad = 0;
	float stand_distance;
	long counter = 0;

	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;
		
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];


			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];
			}
			//cout << "stand_distance：" << stand_distance << endl;
			//cout << "size:" << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				//cout << "point_distance：" << pointRadiusSquaredDistance[i] << endl;
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					//计算1-1.5倍区域点的距离平均
					//&& (pointRadiusSquaredDistance[i]>stand_distance)
					if (pointRadiusSquaredDistance[i] <= (1.5*stand_distance))
						//if (pointRadiusSquaredDistance[i]<=(1.5*stand_distance) && (pointRadiusSquaredDistance[i]>=stand_distance))
					{
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;
			//cout << "u_radius:" << u_radius << endl;
			//cout << "counter" << counter << endl;
			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				//sear_rad = 1.2*u_radius * 1000;
				sear_rad = 1.2*u_radius;
				counter = 0;

			}
			//cout << "sear_rad:" << sear_rad << endl;

		}
		//bunny
		/*if (sear_rad<0.0015)
		{
		sear_rad = 0.0015;
		}*/
		//amo
		/*if (sear_rad<0.35)
		{
		sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		//cout << "pointIdxRadiusSearch"<<pointIdxRadiusSearch.size() << endl;
		//Start_source_src = clock();
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					if (angle1>90)
					{
						angle1 = 180 - angle1;
					}
					angle_sum = angle_sum + angle1;
					/*if (pointRadiusSquaredDistance[i]<= sear_rad / 1.2)
					{
					angle2 = angle1;
					cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
					<< "distance:" << pointRadiusSquaredDistance[i]
					<< "w:1" <<"angle2:"<<angle2
					<< endl;
					}
					else
					{
					angle2 = angle1*exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
					cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
					<< "distance:" << pointRadiusSquaredDistance[i]
					<< "w:" << exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)))
					<< "angle2:" << angle2
					<< endl;
					}*/

					if (angle1>thetol) {

						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						//more_threshod++;
					}
					else
					{

						//less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));

						//less_threshod++;
					}

				}
			}
			/*End_source_src = clock() - Start_source_src;
			cout << "source 特征评分时间：" << End_source_src << endl;*/
			more_threshod_idx.push_back(more_threshod);
			less_threshod_idx.push_back(less_threshod);
			//Start_source_ext = clock();
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			if (more_threshod >= less_threshod)
			{
			if ((less_threshod / (more_threshod + less_threshod))>k) {
			BoundPointIdx.push_back(j);
			}
			else
			{
			AnglePointIdx.push_back(j);
			}
			}
			else
			{
			if ((more_threshod / (more_threshod + less_threshod))>k) {
			BoundPointIdx.push_back(j);
			}
			else
			{
			FacePointIdx.push_back(j);
			}
			}
			//######### Newer vision #########

			//######### Laster vision #########
			/*if (more_threshod > 0 && less_threshod == 0)
			{
				AnglePointIdx.push_back(j);
			}
			if (more_threshod == 0 && less_threshod > 0)
			{
				FacePointIdx.push_back(j);
			}
			if (more_threshod > 0 && less_threshod > 0)
			{
				BoundPointIdx.push_back(j);
			}*/
		
			//######### Laster vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;
			/*End_source_ext = clock() - Start_source_ext;
			cout << "source 特征点提取时间：" << End_source_src << endl;*/

		}
	}
	//构建特征点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature->resize(AnglePointIdx.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		cloud_feature->points[i].x = cloud_source->points[AnglePointIdx[i]].x;
		cloud_feature->points[i].y = cloud_source->points[AnglePointIdx[i]].y;
		cloud_feature->points[i].z = cloud_source->points[AnglePointIdx[i]].z;
	}
	//构建特征点云
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	pcl::PointCloud<pcl::Normal>::Ptr source_feature_normals(new pcl::PointCloud<pcl::Normal>);
	source_feature_normals->resize(AnglePointIdx.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		source_feature_normals->points[i].normal_x = source_normals->points[AnglePointIdx[i]].normal_x;
		source_feature_normals->points[i].normal_y = source_normals->points[AnglePointIdx[i]].normal_y;
		source_feature_normals->points[i].normal_z = source_normals->points[AnglePointIdx[i]].normal_z;
	}
	pfh.setSearchMethod(tree); //设置近邻搜索方式
	pfh.setRadiusSearch(1.85*radius);
	
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_source_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature); //设置输入点云
	pfh.setInputNormals(source_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_source_feature_ptr); //计算每个点的pfh


										  ////保存特征点云
										  //pcl::PointCloud<pcl::PointXYZ> final2 = *cloud_feature;
										  //pcl::io::savePLYFile("source_cloud_rough_area.ply", final2);
										  ////保存特征点云

										  //cout << "start sort" << endl;
										  //compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
										  /*cout << "angle sort1 is ready" << endl;
										  cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
										  cout << "finished source" << endl;*/

										  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
										  //cloud_b->resize(cloud_source->size());
										  //for (int i = 0; i < cloud_source->points.size(); i++)
										  //{

										  //	cloud_b->points[i].x = cloud_source->points[i].x;
										  //	cloud_b->points[i].y = cloud_source->points[i].y;
										  //	cloud_b->points[i].z = cloud_source->points[i].z;
										  //}

										  ////Eigen::Vector3d d, n1, n2;

										  //cout << "ready cloud" << endl;

										  ////printf("pfh size %d", pfh_fe_ptr->points.size());
										  //for (int i = 0; i < AnglePointIdx.size(); i++)
										  //{
										  //	//cout << "angle:" << AnglePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_b->at(AnglePointIdx[i]).r = 255;
										  //	cloud_b->at(AnglePointIdx[i]).g = 0;
										  //	cloud_b->at(AnglePointIdx[i]).b = 0;
										  //}
										  //for (int i = 0; i < BoundPointIdx.size(); i++)
										  //{
										  //	//cout << "bound:" << BoundPointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_b->at(BoundPointIdx[i]).r = 0;
										  //	cloud_b->at(BoundPointIdx[i]).g = 255;
										  //	cloud_b->at(BoundPointIdx[i]).b = 0;
										  //}
										  //for (int i = 0; i < FacePointIdx.size(); i++)
										  //{
										  //	//cout << "face:" << FacePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_b->at(FacePointIdx[i]).r = 100;
										  //	cloud_b->at(FacePointIdx[i]).g = 100;
										  //	cloud_b->at(FacePointIdx[i]).b = 100;
										  //}
										  //cout << "总数：" << FacePointIdx.size() + BoundPointIdx.size() + AnglePointIdx.size();
										  ////savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
										  //pcl::PointCloud<pcl::PointXYZRGB> final2 = *cloud_b;
										  //pcl::io::savePLYFile("source_cloud.ply", final2);
	cout << "ok" << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target = 0;
	float less_threshod_target = 0;
	float k_target = 0.15;
	float u_radius_target = 0;
	float sear_rad_target = 0;
	float stand_distance_target;
	long counter_target = 0;

	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); i++)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[i] <= (1.5*stand_distance_target))
						//if (pointRadiusSquaredDistance_target[i]<=(1.5*stand_distance_target) && (pointRadiusSquaredDistance_target[i] >= stand_distance_target))
					{

						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				//sear_rad_target = 1.2*u_radius_target * 1000;
				sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}


		}
		/*if (sear_rad_target<0.0015)
		{
		sear_rad_target = 0.0015;
		}*/
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
				{
					pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
				}
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					if (angle1_target>90)
					{
						angle1_target = 180 - angle1_target;
					}
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>thetol) {

						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//more_threshod_target++;
					}
					else
					{

						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//less_threshod_target++;
					}
				}
			}

			more_threshod_target_idx.push_back(more_threshod_target);
			less_threshod_target_idx.push_back(less_threshod_target);

			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			if (more_threshod_target >= less_threshod_target)
			{
			if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
			BoundPointIdx_target.push_back(j);
			}
			else
			{
			AnglePointIdx_target.push_back(j);
			}
			}
			else
			{
			if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
			BoundPointIdx_target.push_back(j);
			}
			else
			{
			FacePointIdx_target.push_back(j);
			}
			}
			/*if (more_threshod_target > 0 && less_threshod_target == 0)
			{
				AnglePointIdx_target.push_back(j);
			}
			if (more_threshod_target == 0 && less_threshod_target > 0)
			{
				FacePointIdx_target.push_back(j);
			}
		
			if (more_threshod_target > 0 && less_threshod_target > 0)
			{
				BoundPointIdx_target.push_back(j);
			}*/
		
			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	cout << "AnglePointIdx_target.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_target(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature_target->resize(AnglePointIdx_target.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		cloud_feature_target->points[i].x = cloud_target->points[AnglePointIdx_target[i]].x;
		cloud_feature_target->points[i].y = cloud_target->points[AnglePointIdx_target[i]].y;
		cloud_feature_target->points[i].z = cloud_target->points[AnglePointIdx_target[i]].z;
	}
	//构建特征点云
	pcl::PointCloud<pcl::Normal>::Ptr target_feature_normals(new pcl::PointCloud<pcl::Normal>);
	target_feature_normals->resize(AnglePointIdx_target.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		target_feature_normals->points[i].normal_x = target_normals->points[AnglePointIdx_target[i]].normal_x;
		target_feature_normals->points[i].normal_y = target_normals->points[AnglePointIdx_target[i]].normal_y;
		target_feature_normals->points[i].normal_z = target_normals->points[AnglePointIdx_target[i]].normal_z;
	}

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_target_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature_target); //设置输入点云
	pfh.setInputNormals(target_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_target_feature_ptr); //计算每个点的pfh
										  //pcl::PointCloud<pcl::PointXYZ> final1 = *cloud_feature_target;
										  //pcl::io::savePLYFile("target_cloud_rough_area.ply", final1);

										  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZRGB>);
										  //cloud_c->resize(cloud_target->size());
										  //for (int i = 0; i < cloud_target->points.size(); i++)
										  //{

										  //	cloud_c->points[i].x = cloud_target->points[i].x;
										  //	cloud_c->points[i].y = cloud_target->points[i].y;
										  //	cloud_c->points[i].z = cloud_target->points[i].z;
										  //}



										  //cout << "ready cloud" << endl;

										  ////printf("pfh size %d", pfh_fe_ptr->points.size());
										  //for (int i = 0; i < AnglePointIdx_target.size(); i++)
										  //{
										  //	//cout << "angle:" << AnglePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_c->at(AnglePointIdx_target[i]).r = 255;
										  //	cloud_c->at(AnglePointIdx_target[i]).g = 0;
										  //	cloud_c->at(AnglePointIdx_target[i]).b = 0;
										  //}
										  //for (int i = 0; i < BoundPointIdx_target.size(); i++)
										  //{
										  //	//cout << "bound:" << BoundPointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_c->at(BoundPointIdx_target[i]).r = 0;
										  //	cloud_c->at(BoundPointIdx_target[i]).g = 255;
										  //	cloud_c->at(BoundPointIdx_target[i]).b = 0;
										  //}
										  //for (int i = 0; i < FacePointIdx_target.size(); i++)
										  //{
										  //	//cout << "face:" << FacePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_c->at(FacePointIdx_target[i]).r = 100;
										  //	cloud_c->at(FacePointIdx_target[i]).g = 100;
										  //	cloud_c->at(FacePointIdx_target[i]).b = 100;
										  //}
										  //cout << "总数：" << FacePointIdx_target.size() + BoundPointIdx_target.size() + AnglePointIdx_target.size();
										  ////savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
										  //pcl::PointCloud<pcl::PointXYZRGB> final1 = *cloud_c;
										  //pcl::io::savePLYFile("source_target.ply", final1);
	cout << "ok" << endl;

	int k_search = 3;
	pcl::search::KdTree<pcl::PFHSignature125> pfh_tree;

	pfh_tree.setInputCloud(pfh_target_feature_ptr);
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	//viewer->setBackgroundColor(255, 255, 255);  //设置背景颜色为白色
	//											//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> harris_color_handler(Harris_keypoints, 255, 0, 0);
	//											//viewer->addPointCloud<pcl::PointXYZI>(Harris_keypoints, harris_color_handler, "Harris_keypoints");

	//viewer->addPointCloud(cloud_target, "input_cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "input_cloud");
	//viewer->addPointCloud(cloud_feature_target, "input_cloud_target");
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler(cloud_feature_target, 255, 0, 0);
	////viewer->addPointCloud<pcl::PointXYZ>(cloud_feature_target, harris_color_handler, "input_cloud_target");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "input_cloud_target");

	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud_target");
	/*while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}*/

	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 10000;
	//期待所达到的误差中断循环
	//float respect_err = 3;
	float respect_err = 0.3;
	//float respect_err = 0.0000000001;
	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4 * radius;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_feature;

	kdtree_feature.setInputCloud(cloud_feature);
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();

		int id = rand() % cloud_feature->points.size();
		sample_idx.push_back(id);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			//id = rand() % AnglePointIdx.size();

			id = rand() % cloud_feature->points.size();
			searchPoint.x = cloud_feature->points[id].x;
			searchPoint.y = cloud_feature->points[id].y;
			searchPoint.z = cloud_feature->points[id].z;
			kdtree_feature.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target);
			//cout << "pointIdxRadiusSearch_target.size()" << pointIdxRadiusSearch_target.size() << endl;
			if (pointIdxRadiusSearch_target.size()<4)
			{
				continue;
			}
			//cout << "enter" << endl;
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_feature->at(id), cloud_feature->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(id);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				//id = rand() % AnglePointIdx.size();
				id = rand() % cloud_feature->points.size();

				sample_idx.push_back(id);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			pfh_tree.nearestKSearch(*pfh_source_feature_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}
		/*for (size_t i = 0; i < k_search; i++)
		{

		cout << "distance of k_search" <<i<<":"<< pfh_distance[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			/*if (id == 0)
			{
			final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
			final_match[1][i] = matches[i].target_idx[id];
			}*/
			final_match[1][i] = matches[i].target_idx[id];

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;

		pcl::transformPointCloud(*cloud_feature, *new_source, T.matrix());
		//cout << "77" << endl;

		error = computeError(new_source, cloud_feature_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		if (lowest_error < respect_err)
		{
			cout << "迭代次数：" << i_iter << endl;
			break;
			
		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "part error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);
	error = computeError(cloud_source, cloud_target);
	cout << "error:" << error << endl;
	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_rough_area_double_param_test_limit_anglepoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float u_rad)
{
	float k;
	int thetol;
	//手动输入
	/*cout << "angle ";
	cin >> thetol;
	cout << "ratio ";
	cin >> k;*/
	//hetol = 17;
	thetol = 19;
	k = 0.15;

	float radius = 2.5*u_rad;
	//float radius = 1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(17);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	/*time_t Start_source_src, End_source_src;
	time_t Start_source_ext, End_source_ext;
	time_t Start_target_src, End_target_src;
	time_t Start_target_ext, End_target_ext;*/
	Start = clock();

	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle2;
	float angle_sum = 0;
	float more_threshod = 0;
	float less_threshod = 0;
	//float k = 0.17;
	//int thetol = 17;
	float u_radius = 0;
	float sear_rad = 0;
	float stand_distance;
	long counter = 0;

	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;

		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];


			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];
			}
			//cout << "stand_distance：" << stand_distance << endl;
			//cout << "size:" << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				//cout << "point_distance：" << pointRadiusSquaredDistance[i] << endl;
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					//计算1-1.5倍区域点的距离平均
					//&& (pointRadiusSquaredDistance[i]>stand_distance)
					if (pointRadiusSquaredDistance[i] <= (1.5*stand_distance))
						//if (pointRadiusSquaredDistance[i]<=(1.5*stand_distance) && (pointRadiusSquaredDistance[i]>=stand_distance))
					{
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;
			//cout << "u_radius:" << u_radius << endl;
			//cout << "counter" << counter << endl;
			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				//sear_rad = 1.2*u_radius * 1000;
				sear_rad = 1.2*u_radius;
				counter = 0;

			}
			//cout << "sear_rad:" << sear_rad << endl;

		}
		//bunny
		/*if (sear_rad<0.0015)
		{
		sear_rad = 0.0015;
		}*/
		//amo
		/*if (sear_rad<0.35)
		{
		sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		//cout << "pointIdxRadiusSearch"<<pointIdxRadiusSearch.size() << endl;
		//Start_source_src = clock();
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					if (angle1>90)
					{
						angle1 = 180 - angle1;
					}
					angle_sum = angle_sum + angle1;
					/*if (pointRadiusSquaredDistance[i]<= sear_rad / 1.2)
					{
					angle2 = angle1;
					cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
					<< "distance:" << pointRadiusSquaredDistance[i]
					<< "w:1" <<"angle2:"<<angle2
					<< endl;
					}
					else
					{
					angle2 = angle1*exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
					cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
					<< "distance:" << pointRadiusSquaredDistance[i]
					<< "w:" << exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)))
					<< "angle2:" << angle2
					<< endl;
					}*/

					if (angle1>thetol) {

						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						//more_threshod++;
					}
					else
					{

						//less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));

						//less_threshod++;
					}

				}
			}
			/*End_source_src = clock() - Start_source_src;
			cout << "source 特征评分时间：" << End_source_src << endl;*/
			more_threshod_idx.push_back(more_threshod);
			less_threshod_idx.push_back(less_threshod);
			//Start_source_ext = clock();
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			if (more_threshod >= less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}
			//######### Newer vision #########

			//######### Laster vision #########
			/*if (more_threshod > 0 && less_threshod == 0)
			{
			AnglePointIdx.push_back(j);
			}
			if (more_threshod == 0 && less_threshod > 0)
			{
			FacePointIdx.push_back(j);
			}
			if (more_threshod > 0 && less_threshod > 0)
			{
			BoundPointIdx.push_back(j);
			}*/

			//######### Laster vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;
			/*End_source_ext = clock() - Start_source_ext;
			cout << "source 特征点提取时间：" << End_source_src << endl;*/

		}
	}
	//构建特征点云
	compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature->resize(REPR_NUM);
	int flag = 0;
	for (int i = AnglePointIdx.size() - 1; i > AnglePointIdx.size() - REPR_NUM - 1; i--)
	{
		cloud_feature->points[flag].x = cloud_source->points[AnglePointIdx[i]].x;
		cloud_feature->points[flag].y = cloud_source->points[AnglePointIdx[i]].y;
		cloud_feature->points[flag].z = cloud_source->points[AnglePointIdx[i]].z;
		flag++;
	}
	//构建特征点云
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	pcl::PointCloud<pcl::Normal>::Ptr source_feature_normals(new pcl::PointCloud<pcl::Normal>);
	source_feature_normals->resize(REPR_NUM);
	int flag4 = 0;
	for (int i = AnglePointIdx.size() - 1; i > AnglePointIdx.size() - REPR_NUM - 1; i--)
	{
		source_feature_normals->points[flag4].normal_x = source_normals->points[AnglePointIdx[i]].normal_x;
		source_feature_normals->points[flag4].normal_y = source_normals->points[AnglePointIdx[i]].normal_y;
		source_feature_normals->points[flag4].normal_z = source_normals->points[AnglePointIdx[i]].normal_z;
		flag4++;

	}
	pfh.setSearchMethod(tree); //设置近邻搜索方式
	pfh.setRadiusSearch(1.85*radius);

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_source_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature); //设置输入点云
	pfh.setInputNormals(source_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_source_feature_ptr); //计算每个点的pfh


										  ////保存特征点云
										  //pcl::PointCloud<pcl::PointXYZ> final2 = *cloud_feature;
										  //pcl::io::savePLYFile("source_cloud_rough_area.ply", final2);
										  ////保存特征点云

										  //cout << "start sort" << endl;
										  //compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
										  /*cout << "angle sort1 is ready" << endl;
										  cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
										  cout << "finished source" << endl;*/

										  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
										  //cloud_b->resize(cloud_source->size());
										  //for (int i = 0; i < cloud_source->points.size(); i++)
										  //{

										  //	cloud_b->points[i].x = cloud_source->points[i].x;
										  //	cloud_b->points[i].y = cloud_source->points[i].y;
										  //	cloud_b->points[i].z = cloud_source->points[i].z;
										  //}

										  ////Eigen::Vector3d d, n1, n2;

										  //cout << "ready cloud" << endl;

										  ////printf("pfh size %d", pfh_fe_ptr->points.size());
										  //for (int i = 0; i < AnglePointIdx.size(); i++)
										  //{
										  //	//cout << "angle:" << AnglePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_b->at(AnglePointIdx[i]).r = 255;
										  //	cloud_b->at(AnglePointIdx[i]).g = 0;
										  //	cloud_b->at(AnglePointIdx[i]).b = 0;
										  //}
										  //for (int i = 0; i < BoundPointIdx.size(); i++)
										  //{
										  //	//cout << "bound:" << BoundPointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_b->at(BoundPointIdx[i]).r = 0;
										  //	cloud_b->at(BoundPointIdx[i]).g = 255;
										  //	cloud_b->at(BoundPointIdx[i]).b = 0;
										  //}
										  //for (int i = 0; i < FacePointIdx.size(); i++)
										  //{
										  //	//cout << "face:" << FacePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_b->at(FacePointIdx[i]).r = 100;
										  //	cloud_b->at(FacePointIdx[i]).g = 100;
										  //	cloud_b->at(FacePointIdx[i]).b = 100;
										  //}
										  //cout << "总数：" << FacePointIdx.size() + BoundPointIdx.size() + AnglePointIdx.size();
										  ////savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
										  //pcl::PointCloud<pcl::PointXYZRGB> final2 = *cloud_b;
										  //pcl::io::savePLYFile("source_cloud.ply", final2);
	cout << "ok" << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target = 0;
	float less_threshod_target = 0;
	float k_target = 0.15;
	float u_radius_target = 0;
	float sear_rad_target = 0;
	float stand_distance_target;
	long counter_target = 0;

	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); i++)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[i] <= (1.5*stand_distance_target))
						//if (pointRadiusSquaredDistance_target[i]<=(1.5*stand_distance_target) && (pointRadiusSquaredDistance_target[i] >= stand_distance_target))
					{

						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				//sear_rad_target = 1.2*u_radius_target * 1000;
				sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}


		}
		/*if (sear_rad_target<0.0015)
		{
		sear_rad_target = 0.0015;
		}*/
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
				{
					pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
				}
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					if (angle1_target>90)
					{
						angle1_target = 180 - angle1_target;
					}
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>thetol) {

						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//more_threshod_target++;
					}
					else
					{

						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//less_threshod_target++;
					}
				}
			}

			more_threshod_target_idx.push_back(more_threshod_target);
			less_threshod_target_idx.push_back(less_threshod_target);

			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			if (more_threshod_target >= less_threshod_target)
			{
				if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					AnglePointIdx_target.push_back(j);
				}
			}
			else
			{
				if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					FacePointIdx_target.push_back(j);
				}
			}
			/*if (more_threshod_target > 0 && less_threshod_target == 0)
			{
			AnglePointIdx_target.push_back(j);
			}
			if (more_threshod_target == 0 && less_threshod_target > 0)
			{
			FacePointIdx_target.push_back(j);
			}

			if (more_threshod_target > 0 && less_threshod_target > 0)
			{
			BoundPointIdx_target.push_back(j);
			}*/

			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	cout << "AnglePointIdx_target.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////
	compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_target(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature_target->resize(REPR_NUM);
	int flag1 = 0;
	for (int i = AnglePointIdx_target.size() - 1; i > AnglePointIdx_target.size() - REPR_NUM - 1; i--)
	{
		cloud_feature_target->points[flag1].x = cloud_target->points[AnglePointIdx_target[i]].x;
		cloud_feature_target->points[flag1].y = cloud_target->points[AnglePointIdx_target[i]].y;
		cloud_feature_target->points[flag1].z = cloud_target->points[AnglePointIdx_target[i]].z;
		flag1++;
	}
	//构建特征点云
	pcl::PointCloud<pcl::Normal>::Ptr target_feature_normals(new pcl::PointCloud<pcl::Normal>);
	target_feature_normals->resize(REPR_NUM);
	int flag2 = 0;
	for (int i = AnglePointIdx_target.size() - 1; i > AnglePointIdx_target.size() - REPR_NUM - 1; i--)
	{
		target_feature_normals->points[flag2].normal_x = target_normals->points[AnglePointIdx_target[i]].normal_x;
		target_feature_normals->points[flag2].normal_y = target_normals->points[AnglePointIdx_target[i]].normal_y;
		target_feature_normals->points[flag2].normal_z = target_normals->points[AnglePointIdx_target[i]].normal_z;
		flag2++;
	}

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_target_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature_target); //设置输入点云
	pfh.setInputNormals(target_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_target_feature_ptr); //计算每个点的pfh
										  //pcl::PointCloud<pcl::PointXYZ> final1 = *cloud_feature_target;
										  //pcl::io::savePLYFile("target_cloud_rough_area.ply", final1);

										  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZRGB>);
										  //cloud_c->resize(cloud_target->size());
										  //for (int i = 0; i < cloud_target->points.size(); i++)
										  //{

										  //	cloud_c->points[i].x = cloud_target->points[i].x;
										  //	cloud_c->points[i].y = cloud_target->points[i].y;
										  //	cloud_c->points[i].z = cloud_target->points[i].z;
										  //}



										  //cout << "ready cloud" << endl;

										  ////printf("pfh size %d", pfh_fe_ptr->points.size());
										  //for (int i = 0; i < AnglePointIdx_target.size(); i++)
										  //{
										  //	//cout << "angle:" << AnglePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_c->at(AnglePointIdx_target[i]).r = 255;
										  //	cloud_c->at(AnglePointIdx_target[i]).g = 0;
										  //	cloud_c->at(AnglePointIdx_target[i]).b = 0;
										  //}
										  //for (int i = 0; i < BoundPointIdx_target.size(); i++)
										  //{
										  //	//cout << "bound:" << BoundPointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_c->at(BoundPointIdx_target[i]).r = 0;
										  //	cloud_c->at(BoundPointIdx_target[i]).g = 255;
										  //	cloud_c->at(BoundPointIdx_target[i]).b = 0;
										  //}
										  //for (int i = 0; i < FacePointIdx_target.size(); i++)
										  //{
										  //	//cout << "face:" << FacePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_c->at(FacePointIdx_target[i]).r = 100;
										  //	cloud_c->at(FacePointIdx_target[i]).g = 100;
										  //	cloud_c->at(FacePointIdx_target[i]).b = 100;
										  //}
										  //cout << "总数：" << FacePointIdx_target.size() + BoundPointIdx_target.size() + AnglePointIdx_target.size();
										  ////savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
										  //pcl::PointCloud<pcl::PointXYZRGB> final1 = *cloud_c;
										  //pcl::io::savePLYFile("source_target.ply", final1);
	cout << "ok" << endl;

	int k_search = 3;
	pcl::search::KdTree<pcl::PFHSignature125> pfh_tree;

	pfh_tree.setInputCloud(pfh_target_feature_ptr);
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	//viewer->setBackgroundColor(255, 255, 255);  //设置背景颜色为白色
	//											//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> harris_color_handler(Harris_keypoints, 255, 0, 0);
	//											//viewer->addPointCloud<pcl::PointXYZI>(Harris_keypoints, harris_color_handler, "Harris_keypoints");

	//viewer->addPointCloud(cloud_target, "input_cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "input_cloud");
	//viewer->addPointCloud(cloud_feature_target, "input_cloud_target");
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler(cloud_feature_target, 255, 0, 0);
	////viewer->addPointCloud<pcl::PointXYZ>(cloud_feature_target, harris_color_handler, "input_cloud_target");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "input_cloud_target");

	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud_target");
	/*while (!viewer->wasStopped())
	{
	viewer->spinOnce(100);
	boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}*/

	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 10000;
	//期待所达到的误差中断循环
	//float respect_err = 3;
	float respect_err = 0.3;
	//float respect_err = 0.0000000001;
	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4 * radius;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_feature;

	kdtree_feature.setInputCloud(cloud_feature);
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();

		int id = rand() % cloud_feature->points.size();
		sample_idx.push_back(id);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			//id = rand() % AnglePointIdx.size();

			id = rand() % cloud_feature->points.size();
			searchPoint.x = cloud_feature->points[id].x;
			searchPoint.y = cloud_feature->points[id].y;
			searchPoint.z = cloud_feature->points[id].z;
			kdtree_feature.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target);
			//cout << "pointIdxRadiusSearch_target.size()" << pointIdxRadiusSearch_target.size() << endl;
			if (pointIdxRadiusSearch_target.size()<4)
			{
				continue;
			}
			//cout << "enter" << endl;
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_feature->at(id), cloud_feature->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(id);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				//id = rand() % AnglePointIdx.size();
				id = rand() % cloud_feature->points.size();

				sample_idx.push_back(id);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			pfh_tree.nearestKSearch(*pfh_source_feature_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}
		/*for (size_t i = 0; i < k_search; i++)
		{

		cout << "distance of k_search" <<i<<":"<< pfh_distance[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			/*if (id == 0)
			{
			final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
			final_match[1][i] = matches[i].target_idx[id];
			}*/
			final_match[1][i] = matches[i].target_idx[id];

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;

		pcl::transformPointCloud(*cloud_feature, *new_source, T.matrix());
		//cout << "77" << endl;

		error = computeError(new_source, cloud_feature_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		if (lowest_error < respect_err)
		{
			cout << "迭代次数：" << i_iter << endl;
			break;

		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "part error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);
	error = computeError(cloud_source, cloud_target);
	cout << "error:" << error << endl;
	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_rough_area_double_param_test_more_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float u_rad)
{
	float k;
	int thetol;
	//手动输入
	/*cout << "angle ";
	cin >> thetol;
	cout << "ratio ";
	cin >> k;*/
	//hetol = 17;
	thetol = 19;
	k = 0.15;

	float radius = 2.5*u_rad;
	//float radius = 1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(17);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	/*time_t Start_source_src, End_source_src;
	time_t Start_source_ext, End_source_ext;
	time_t Start_target_src, End_target_src;
	time_t Start_target_ext, End_target_ext;*/
	Start = clock();

	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle2;
	float angle_sum = 0;
	float more_threshod = 0;
	float less_threshod = 0;
	//float k = 0.17;
	//int thetol = 17;
	float u_radius = 0;
	float sear_rad = 0;
	float stand_distance;
	long counter = 0;

	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;

		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];


			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];
			}
			//cout << "stand_distance：" << stand_distance << endl;
			//cout << "size:" << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				//cout << "point_distance：" << pointRadiusSquaredDistance[i] << endl;
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					//计算1-1.5倍区域点的距离平均
					//&& (pointRadiusSquaredDistance[i]>stand_distance)
					if (pointRadiusSquaredDistance[i] <= (1.5*stand_distance))
						//if (pointRadiusSquaredDistance[i]<=(1.5*stand_distance) && (pointRadiusSquaredDistance[i]>=stand_distance))
					{
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;
			//cout << "u_radius:" << u_radius << endl;
			//cout << "counter" << counter << endl;
			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				//sear_rad = 1.2*u_radius * 1000;
				sear_rad = 1.2*u_radius;
				counter = 0;

			}
			//cout << "sear_rad:" << sear_rad << endl;

		}
		//bunny
		/*if (sear_rad<0.0015)
		{
		sear_rad = 0.0015;
		}*/
		//amo
		/*if (sear_rad<0.35)
		{
		sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		//cout << "pointIdxRadiusSearch"<<pointIdxRadiusSearch.size() << endl;
		//Start_source_src = clock();
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					if (angle1>90)
					{
						angle1 = 180 - angle1;
					}
					angle_sum = angle_sum + angle1;
					

					if (angle1>thetol) {

						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						//more_threshod++;
					}
					else
					{

						//less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));

						//less_threshod++;
					}

				}
			}
			/*End_source_src = clock() - Start_source_src;
			cout << "source 特征评分时间：" << End_source_src << endl;*/
			more_threshod_idx.push_back(more_threshod);
			less_threshod_idx.push_back(less_threshod);
			//Start_source_ext = clock();
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			if (more_threshod >= less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}
			//######### Newer vision #########

		
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;
			/*End_source_ext = clock() - Start_source_ext;
			cout << "source 特征点提取时间：" << End_source_src << endl;*/

		}
	}
	//构建特征点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature1(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature1->resize(AnglePointIdx.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		cloud_feature1->points[i].x = cloud_source->points[AnglePointIdx[i]].x;
		cloud_feature1->points[i].y = cloud_source->points[AnglePointIdx[i]].y;
		cloud_feature1->points[i].z = cloud_source->points[AnglePointIdx[i]].z;
	}
	//构建特征点云
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	pcl::PointCloud<pcl::Normal>::Ptr source_feature_normals(new pcl::PointCloud<pcl::Normal>);
	source_feature_normals->resize(AnglePointIdx.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		source_feature_normals->points[i].normal_x = source_normals->points[AnglePointIdx[i]].normal_x;
		source_feature_normals->points[i].normal_y = source_normals->points[AnglePointIdx[i]].normal_y;
		source_feature_normals->points[i].normal_z = source_normals->points[AnglePointIdx[i]].normal_z;
	}
	pfh.setSearchMethod(tree); //设置近邻搜索方式
	pfh.setRadiusSearch(1.85*radius);

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_source_feature_ptr1(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature1); //设置输入点云
	pfh.setInputNormals(source_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_source_feature_ptr1); //计算每个点的pfh

	compute_u_high(pfh_source_feature_ptr1);
	//
	compute_L2_dis(pfh_source_feature_ptr1);
	pfh_sort_num = new int[cloud_feature1->points.size()];
	for (int i = 0; i < cloud_feature1->points.size(); i++)
	{
		pfh_sort_num[i] = i;
	}

	quick_sort(L2_dis, pfh_sort_num, 0, cloud_feature1->points.size() - 1);
	//特征源点云构建
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_source_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>());
	cloud_feature->resize(REPR_NUM);
	pfh_source_feature_ptr->resize(REPR_NUM);
	int flag = 0;
	for (int i = cloud_feature1->points.size() - 1; i > cloud_feature1->points.size() - REPR_NUM - 1; i--)
	{

		cloud_feature->points[flag].x = cloud_feature1->points[pfh_sort_num[i]].x;
		cloud_feature->points[flag].y = cloud_feature1->points[pfh_sort_num[i]].y;
		cloud_feature->points[flag].z = cloud_feature1->points[pfh_sort_num[i]].z;
		for (int j = 0; j < 125; j++) {
			pfh_source_feature_ptr->points[flag].histogram[j] = pfh_source_feature_ptr1->points[pfh_sort_num[i]].histogram[j];
		}
		flag++;
	}
	
	
	End = clock() - Start;
	std::cout << "Feature Time is :" << End << std::endl;
	cout << "ok" << endl;

	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target = 0;
	float less_threshod_target = 0;
	float k_target = 0.15;
	float u_radius_target = 0;
	float sear_rad_target = 0;
	float stand_distance_target;
	long counter_target = 0;

	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); i++)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[i] <= (1.5*stand_distance_target))
						//if (pointRadiusSquaredDistance_target[i]<=(1.5*stand_distance_target) && (pointRadiusSquaredDistance_target[i] >= stand_distance_target))
					{

						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				//sear_rad_target = 1.2*u_radius_target * 1000;
				sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}


		}
		/*if (sear_rad_target<0.0015)
		{
		sear_rad_target = 0.0015;
		}*/
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
				{
					pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
				}
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					if (angle1_target>90)
					{
						angle1_target = 180 - angle1_target;
					}
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>thetol) {

						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//more_threshod_target++;
					}
					else
					{

						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//less_threshod_target++;
					}
				}
			}

			more_threshod_target_idx.push_back(more_threshod_target);
			less_threshod_target_idx.push_back(less_threshod_target);

			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			if (more_threshod_target >= less_threshod_target)
			{
				if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					AnglePointIdx_target.push_back(j);
				}
			}
			else
			{
				if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					FacePointIdx_target.push_back(j);
				}
			}
		
			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	cout << "AnglePointIdx_target.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_target1(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature_target1->resize(AnglePointIdx_target.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		cloud_feature_target1->points[i].x = cloud_target->points[AnglePointIdx_target[i]].x;
		cloud_feature_target1->points[i].y = cloud_target->points[AnglePointIdx_target[i]].y;
		cloud_feature_target1->points[i].z = cloud_target->points[AnglePointIdx_target[i]].z;
	}
	//构建特征点云
	pcl::PointCloud<pcl::Normal>::Ptr target_feature_normals(new pcl::PointCloud<pcl::Normal>);
	target_feature_normals->resize(AnglePointIdx_target.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		target_feature_normals->points[i].normal_x = target_normals->points[AnglePointIdx_target[i]].normal_x;
		target_feature_normals->points[i].normal_y = target_normals->points[AnglePointIdx_target[i]].normal_y;
		target_feature_normals->points[i].normal_z = target_normals->points[AnglePointIdx_target[i]].normal_z;
	}

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_target_feature_ptr1(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature_target1); //设置输入点云
	pfh.setInputNormals(target_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_target_feature_ptr1); //计算每个点的pfh



	compute_u_high(pfh_target_feature_ptr1);
	//
	compute_L2_dis(pfh_target_feature_ptr1);
	pfh_sort_num = new int[cloud_feature_target1->points.size()];
	for (int i = 0; i < cloud_feature_target1->points.size(); i++)
	{
		pfh_sort_num[i] = i;
	}

	quick_sort(L2_dis, pfh_sort_num, 0, cloud_feature_target1->points.size() - 1);
	//特征源点云构建
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_target_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>());
	cloud_feature_target->resize(REPR_NUM);
	pfh_target_feature_ptr->resize(REPR_NUM);
	int flag1 = 0;
	for (int i = cloud_feature_target1->points.size() - 1; i > cloud_feature_target1->points.size() - REPR_NUM - 1; i--)
	{

		cloud_feature_target->points[flag1].x = cloud_feature_target1->points[pfh_sort_num[i]].x;
		cloud_feature_target->points[flag1].y = cloud_feature_target1->points[pfh_sort_num[i]].y;
		cloud_feature_target->points[flag1].z = cloud_feature_target1->points[pfh_sort_num[i]].z;
		for (int j = 0; j < 125; j++) {
			pfh_target_feature_ptr->points[flag1].histogram[j] = pfh_target_feature_ptr1->points[pfh_sort_num[i]].histogram[j];
		}
		flag1++;
	}





	cout << "ok" << endl;

	int k_search = 3;
	pcl::search::KdTree<pcl::PFHSignature125> pfh_tree;

	pfh_tree.setInputCloud(pfh_target_feature_ptr);


	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 10000;
	//期待所达到的误差中断循环
	//float respect_err = 3;
	//float respect_err = 0.3;
	float respect_err = 0.3;
	//float respect_err = 0.0000000001;
	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4 * radius;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_feature;

	kdtree_feature.setInputCloud(cloud_feature);
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();

		int id = rand() % cloud_feature->points.size();
		sample_idx.push_back(id);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			//id = rand() % AnglePointIdx.size();

			id = rand() % cloud_feature->points.size();
			//searchPoint.x = cloud_feature->points[id].x;
			//searchPoint.y = cloud_feature->points[id].y;
			//searchPoint.z = cloud_feature->points[id].z;
			//kdtree_feature.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target);
			////cout << "pointIdxRadiusSearch_target.size()" << pointIdxRadiusSearch_target.size() << endl;
			//if (pointIdxRadiusSearch_target.size()<4)
			//{
			//	continue;
			//}
			//cout << "enter" << endl;
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_feature->at(id), cloud_feature->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << "cut" << endl;
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(id);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				//id = rand() % AnglePointIdx.size();
				id = rand() % cloud_feature->points.size();

				sample_idx.push_back(id);
				cut = 0;
			}
		}
		//cout << "into match" << endl;

		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			pfh_tree.nearestKSearch(*pfh_source_feature_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}
		/*for (size_t i = 0; i < k_search; i++)
		{

		cout << "distance of k_search" <<i<<":"<< pfh_distance[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			/*if (id == 0)
			{
			final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
			final_match[1][i] = matches[i].target_idx[id];
			}*/
			final_match[1][i] = matches[i].target_idx[id];

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		//cout << "into trans" << endl;
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;

		pcl::transformPointCloud(*cloud_feature, *new_source, T.matrix());
		//cout << "77" << endl;

		error = computeError(new_source, cloud_feature_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		if (lowest_error < respect_err)
		{
			cout << "迭代次数：" << i_iter << endl;
			break;

		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "part error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);
	error = computeError(cloud_source, cloud_target);
	cout << "error:" << error << endl;
	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_rough_area_double_param_test_diff_num(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_init, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_init, float u_rad)
{
	float k;
	int thetol;
	//手动输入
	/*cout << "angle ";
	cin >> thetol;
	cout << "ratio ";
	cin >> k;*/
	thetol = 17;
	k = 0.15;

	float radius = 2.5*u_rad;
	//float radius = 1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(17);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	/*time_t Start_source_src, End_source_src;
	time_t Start_source_ext, End_source_ext;
	time_t Start_target_src, End_target_src;
	time_t Start_target_ext, End_target_ext;*/
	Start = clock();

	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle2;
	float angle_sum = 0;
	float more_threshod = 0;
	float less_threshod = 0;
	//float k = 0.17;
	//int thetol = 17;
	float u_radius = 0;
	float sear_rad = 0;
	float stand_distance;
	long counter = 0;

	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;

		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];


			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];
			}
			//cout << "stand_distance：" << stand_distance << endl;
			//cout << "size:" << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				//cout << "point_distance：" << pointRadiusSquaredDistance[i] << endl;
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					//计算1-1.5倍区域点的距离平均
					//&& (pointRadiusSquaredDistance[i]>stand_distance)
					if (pointRadiusSquaredDistance[i] <= (1.5*stand_distance))
						//if (pointRadiusSquaredDistance[i]<=(1.5*stand_distance) && (pointRadiusSquaredDistance[i]>=stand_distance))
					{
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;
			//cout << "u_radius:" << u_radius << endl;
			//cout << "counter" << counter << endl;
			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				//sear_rad = 1.2*u_radius * 1000;
				sear_rad = 1.2*u_radius;
				counter = 0;

			}
			//cout << "sear_rad:" << sear_rad << endl;

		}
		//bunny
		/*if (sear_rad<0.0015)
		{
		sear_rad = 0.0015;
		}*/
		//amo
		/*if (sear_rad<0.35)
		{
		sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		//cout << "pointIdxRadiusSearch"<<pointIdxRadiusSearch.size() << endl;
		//Start_source_src = clock();
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					if (angle1>90)
					{
						angle1 = 180 - angle1;
					}
					angle_sum = angle_sum + angle1;
					/*if (pointRadiusSquaredDistance[i]<= sear_rad / 1.2)
					{
					angle2 = angle1;
					cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
					<< "distance:" << pointRadiusSquaredDistance[i]
					<< "w:1" <<"angle2:"<<angle2
					<< endl;
					}
					else
					{
					angle2 = angle1*exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
					cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
					<< "distance:" << pointRadiusSquaredDistance[i]
					<< "w:" << exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)))
					<< "angle2:" << angle2
					<< endl;
					}*/

					if (angle1>thetol) {

						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						//more_threshod++;
					}
					else
					{

						//less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));

						//less_threshod++;
					}

				}
			}
			/*End_source_src = clock() - Start_source_src;
			cout << "source 特征评分时间：" << End_source_src << endl;*/
			more_threshod_idx.push_back(more_threshod);
			less_threshod_idx.push_back(less_threshod);
			//Start_source_ext = clock();
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			if (more_threshod >= less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}
			//######### Newer vision #########

			//######### Laster vision #########
			/*if (more_threshod > 0 && less_threshod == 0)
			{
			AnglePointIdx.push_back(j);
			}
			if (more_threshod == 0 && less_threshod > 0)
			{
			FacePointIdx.push_back(j);
			}
			if (more_threshod > 0 && less_threshod > 0)
			{
			BoundPointIdx.push_back(j);
			}*/

			//######### Laster vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;
			/*End_source_ext = clock() - Start_source_ext;
			cout << "source 特征点提取时间：" << End_source_src << endl;*/

		}
	}
	//构建特征点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature->resize(AnglePointIdx.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		cloud_feature->points[i].x = cloud_source->points[AnglePointIdx[i]].x;
		cloud_feature->points[i].y = cloud_source->points[AnglePointIdx[i]].y;
		cloud_feature->points[i].z = cloud_source->points[AnglePointIdx[i]].z;
	}
	//构建特征点云
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	pcl::PointCloud<pcl::Normal>::Ptr source_feature_normals(new pcl::PointCloud<pcl::Normal>);
	source_feature_normals->resize(AnglePointIdx.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		source_feature_normals->points[i].normal_x = source_normals->points[AnglePointIdx[i]].normal_x;
		source_feature_normals->points[i].normal_y = source_normals->points[AnglePointIdx[i]].normal_y;
		source_feature_normals->points[i].normal_z = source_normals->points[AnglePointIdx[i]].normal_z;
	}
	pfh.setSearchMethod(tree); //设置近邻搜索方式
	pfh.setRadiusSearch(1.85*radius);

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_source_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature); //设置输入点云
	pfh.setInputNormals(source_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_source_feature_ptr); //计算每个点的pfh


										  ////保存特征点云
										  //pcl::PointCloud<pcl::PointXYZ> final2 = *cloud_feature;
										  //pcl::io::savePLYFile("source_cloud_rough_area.ply", final2);
										  ////保存特征点云

										  //cout << "start sort" << endl;
										  //compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
										  /*cout << "angle sort1 is ready" << endl;
										  cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
										  cout << "finished source" << endl;*/

										  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
										  //cloud_b->resize(cloud_source->size());
										  //for (int i = 0; i < cloud_source->points.size(); i++)
										  //{

										  //	cloud_b->points[i].x = cloud_source->points[i].x;
										  //	cloud_b->points[i].y = cloud_source->points[i].y;
										  //	cloud_b->points[i].z = cloud_source->points[i].z;
										  //}

										  ////Eigen::Vector3d d, n1, n2;

										  //cout << "ready cloud" << endl;

										  ////printf("pfh size %d", pfh_fe_ptr->points.size());
										  //for (int i = 0; i < AnglePointIdx.size(); i++)
										  //{
										  //	//cout << "angle:" << AnglePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_b->at(AnglePointIdx[i]).r = 255;
										  //	cloud_b->at(AnglePointIdx[i]).g = 0;
										  //	cloud_b->at(AnglePointIdx[i]).b = 0;
										  //}
										  //for (int i = 0; i < BoundPointIdx.size(); i++)
										  //{
										  //	//cout << "bound:" << BoundPointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_b->at(BoundPointIdx[i]).r = 0;
										  //	cloud_b->at(BoundPointIdx[i]).g = 255;
										  //	cloud_b->at(BoundPointIdx[i]).b = 0;
										  //}
										  //for (int i = 0; i < FacePointIdx.size(); i++)
										  //{
										  //	//cout << "face:" << FacePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_b->at(FacePointIdx[i]).r = 100;
										  //	cloud_b->at(FacePointIdx[i]).g = 100;
										  //	cloud_b->at(FacePointIdx[i]).b = 100;
										  //}
										  //cout << "总数：" << FacePointIdx.size() + BoundPointIdx.size() + AnglePointIdx.size();
										  ////savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
										  //pcl::PointCloud<pcl::PointXYZRGB> final2 = *cloud_b;
										  //pcl::io::savePLYFile("source_cloud.ply", final2);
	cout << "ok" << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target = 0;
	float less_threshod_target = 0;
	float k_target = 0.15;
	float u_radius_target = 0;
	float sear_rad_target = 0;
	float stand_distance_target;
	long counter_target = 0;

	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); i++)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[i] <= (1.5*stand_distance_target))
						//if (pointRadiusSquaredDistance_target[i]<=(1.5*stand_distance_target) && (pointRadiusSquaredDistance_target[i] >= stand_distance_target))
					{

						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				//sear_rad_target = 1.2*u_radius_target * 1000;
				sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}


		}
		/*if (sear_rad_target<0.0015)
		{
		sear_rad_target = 0.0015;
		}*/
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
				{
					pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
				}
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					if (angle1_target>90)
					{
						angle1_target = 180 - angle1_target;
					}
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>thetol) {

						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//more_threshod_target++;
					}
					else
					{

						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//less_threshod_target++;
					}
				}
			}

			more_threshod_target_idx.push_back(more_threshod_target);
			less_threshod_target_idx.push_back(less_threshod_target);

			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			if (more_threshod_target >= less_threshod_target)
			{
				if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					AnglePointIdx_target.push_back(j);
				}
			}
			else
			{
				if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					FacePointIdx_target.push_back(j);
				}
			}
			/*if (more_threshod_target > 0 && less_threshod_target == 0)
			{
			AnglePointIdx_target.push_back(j);
			}
			if (more_threshod_target == 0 && less_threshod_target > 0)
			{
			FacePointIdx_target.push_back(j);
			}

			if (more_threshod_target > 0 && less_threshod_target > 0)
			{
			BoundPointIdx_target.push_back(j);
			}*/

			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	cout << "AnglePointIdx_target.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_target(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature_target->resize(AnglePointIdx_target.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		cloud_feature_target->points[i].x = cloud_target->points[AnglePointIdx_target[i]].x;
		cloud_feature_target->points[i].y = cloud_target->points[AnglePointIdx_target[i]].y;
		cloud_feature_target->points[i].z = cloud_target->points[AnglePointIdx_target[i]].z;
	}
	//构建特征点云
	pcl::PointCloud<pcl::Normal>::Ptr target_feature_normals(new pcl::PointCloud<pcl::Normal>);
	target_feature_normals->resize(AnglePointIdx_target.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		target_feature_normals->points[i].normal_x = target_normals->points[AnglePointIdx_target[i]].normal_x;
		target_feature_normals->points[i].normal_y = target_normals->points[AnglePointIdx_target[i]].normal_y;
		target_feature_normals->points[i].normal_z = target_normals->points[AnglePointIdx_target[i]].normal_z;
	}

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_target_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature_target); //设置输入点云
	pfh.setInputNormals(target_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_target_feature_ptr); //计算每个点的pfh
										  //pcl::PointCloud<pcl::PointXYZ> final1 = *cloud_feature_target;
										  //pcl::io::savePLYFile("target_cloud_rough_area.ply", final1);

										  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZRGB>);
										  //cloud_c->resize(cloud_target->size());
										  //for (int i = 0; i < cloud_target->points.size(); i++)
										  //{

										  //	cloud_c->points[i].x = cloud_target->points[i].x;
										  //	cloud_c->points[i].y = cloud_target->points[i].y;
										  //	cloud_c->points[i].z = cloud_target->points[i].z;
										  //}



										  //cout << "ready cloud" << endl;

										  ////printf("pfh size %d", pfh_fe_ptr->points.size());
										  //for (int i = 0; i < AnglePointIdx_target.size(); i++)
										  //{
										  //	//cout << "angle:" << AnglePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_c->at(AnglePointIdx_target[i]).r = 255;
										  //	cloud_c->at(AnglePointIdx_target[i]).g = 0;
										  //	cloud_c->at(AnglePointIdx_target[i]).b = 0;
										  //}
										  //for (int i = 0; i < BoundPointIdx_target.size(); i++)
										  //{
										  //	//cout << "bound:" << BoundPointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_c->at(BoundPointIdx_target[i]).r = 0;
										  //	cloud_c->at(BoundPointIdx_target[i]).g = 255;
										  //	cloud_c->at(BoundPointIdx_target[i]).b = 0;
										  //}
										  //for (int i = 0; i < FacePointIdx_target.size(); i++)
										  //{
										  //	//cout << "face:" << FacePointIdx.size() << endl;
										  //	//cout << "ready color" << endl;
										  //	cloud_c->at(FacePointIdx_target[i]).r = 100;
										  //	cloud_c->at(FacePointIdx_target[i]).g = 100;
										  //	cloud_c->at(FacePointIdx_target[i]).b = 100;
										  //}
										  //cout << "总数：" << FacePointIdx_target.size() + BoundPointIdx_target.size() + AnglePointIdx_target.size();
										  ////savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
										  //pcl::PointCloud<pcl::PointXYZRGB> final1 = *cloud_c;
										  //pcl::io::savePLYFile("source_target.ply", final1);
	cout << "ok" << endl;

	int k_search = 3;
	pcl::search::KdTree<pcl::PFHSignature125> pfh_tree;

	pfh_tree.setInputCloud(pfh_target_feature_ptr);
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	//viewer->setBackgroundColor(255, 255, 255);  //设置背景颜色为白色
	//											//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> harris_color_handler(Harris_keypoints, 255, 0, 0);
	//											//viewer->addPointCloud<pcl::PointXYZI>(Harris_keypoints, harris_color_handler, "Harris_keypoints");

	//viewer->addPointCloud(cloud_target, "input_cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "input_cloud");
	//viewer->addPointCloud(cloud_feature_target, "input_cloud_target");
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler(cloud_feature_target, 255, 0, 0);
	////viewer->addPointCloud<pcl::PointXYZ>(cloud_feature_target, harris_color_handler, "input_cloud_target");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "input_cloud_target");

	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud_target");
	/*while (!viewer->wasStopped())
	{
	viewer->spinOnce(100);
	boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}*/

	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 10000;
	//期待所达到的误差中断循环
	float respect_err = 0.3;
	//float respect_err = 0.0000000001;
	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4 * radius;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_feature;

	kdtree_feature.setInputCloud(cloud_feature);
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();

		int id = rand() % cloud_feature->points.size();
		sample_idx.push_back(id);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			//id = rand() % AnglePointIdx.size();

			id = rand() % cloud_feature->points.size();
			searchPoint.x = cloud_feature->points[id].x;
			searchPoint.y = cloud_feature->points[id].y;
			searchPoint.z = cloud_feature->points[id].z;
			kdtree_feature.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target);
			//cout << "pointIdxRadiusSearch_target.size()" << pointIdxRadiusSearch_target.size() << endl;
			if (pointIdxRadiusSearch_target.size()<4)
			{
				continue;
			}
			//cout << "enter" << endl;
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_feature->at(id), cloud_feature->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(id);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				//id = rand() % AnglePointIdx.size();
				id = rand() % cloud_feature->points.size();

				sample_idx.push_back(id);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			pfh_tree.nearestKSearch(*pfh_source_feature_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}
		/*for (size_t i = 0; i < k_search; i++)
		{

		cout << "distance of k_search" <<i<<":"<< pfh_distance[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			/*if (id == 0)
			{
			final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
			final_match[1][i] = matches[i].target_idx[id];
			}*/
			final_match[1][i] = matches[i].target_idx[id];

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;

		pcl::transformPointCloud(*cloud_feature, *new_source, T.matrix());
		//cout << "77" << endl;

		error = computeError(new_source, cloud_feature_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		if (lowest_error < respect_err)
		{
			cout << "迭代次数：" << i_iter << endl;
			break;

		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "part error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source_init, *cloud_source_init, final_transformation_);
	error = computeError(cloud_source_init, cloud_target_init);
	cout << "error:" << error << endl;
	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_rough_area_ANGLE_AUTO_noise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float u_rad)
{
	float radius = 2.5*u_rad;
	//float radius = 1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(17);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	Start = clock();
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle2;
	float angle_sum = 0;
	float more_threshod = 0;
	float less_threshod = 0;
	float k = 0.15;
	float u_radius = 0;
	float sear_rad = 0;
	float stand_distance;
	long counter = 0;

	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];

			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];
			}
			//cout << "stand_distance：" << stand_distance << endl;
			//cout << "size:" << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				//cout << "point_distance：" << pointRadiusSquaredDistance[i] << endl;
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					//计算1-1.5倍区域点的距离平均
					//&& (pointRadiusSquaredDistance[i]>stand_distance)
					if (pointRadiusSquaredDistance[i] <= (1.5*stand_distance))
						//if (pointRadiusSquaredDistance[i]<=(1.5*stand_distance) && (pointRadiusSquaredDistance[i]>=stand_distance))
					{
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;
			//cout << "u_radius:" << u_radius << endl;
			//cout << "counter" << counter << endl;
			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				//sear_rad = 1.2*u_radius * 1000;
				sear_rad = 1.2*u_radius;
				counter = 0;

			}
			//cout << "sear_rad:" << sear_rad << endl;

		}
		//bunny
		/*if (sear_rad<0.0015)
		{
		sear_rad = 0.0015;
		}*/
		//amo
		/*if (sear_rad<0.35)
		{
		sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		//cout << "pointIdxRadiusSearch"<<pointIdxRadiusSearch.size() << endl;
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					if (angle1>90)
					{
						angle1 = 180 - angle1;
					}
					angle_sum = angle_sum + angle1;
					/*if (pointRadiusSquaredDistance[i]<= sear_rad / 1.2)
					{
					angle2 = angle1;
					cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
					<< "distance:" << pointRadiusSquaredDistance[i]
					<< "w:1" <<"angle2:"<<angle2
					<< endl;
					}
					else
					{
					angle2 = angle1*exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
					cout << j << " the " << i << " angle:" << angle1 << "u_rad:" << sear_rad / 1.2
					<< "distance:" << pointRadiusSquaredDistance[i]
					<< "w:" << exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)))
					<< "angle2:" << angle2
					<< endl;
					}*/

					if (angle1>17) {

						//more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						//more_threshod++;
					}
					else
					{

						//less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad / 1000 - u_radius, 2)));
						less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));

						//less_threshod++;
					}

				}
			}

			more_threshod_idx.push_back(more_threshod);
			less_threshod_idx.push_back(less_threshod);

			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			/*if (more_threshod >= less_threshod)
			{
			if ((less_threshod / (more_threshod + less_threshod))>k) {
			BoundPointIdx.push_back(j);
			}
			else
			{
			AnglePointIdx.push_back(j);
			}
			}
			else
			{
			if ((more_threshod / (more_threshod + less_threshod))>k) {
			BoundPointIdx.push_back(j);
			}
			else
			{
			FacePointIdx.push_back(j);
			}
			}*/
			//######### Newer vision #########

			//######### Laster vision #########
			if (more_threshod > 0 && less_threshod == 0)
			{
				AnglePointIdx.push_back(j);
			}
			if (more_threshod == 0 && less_threshod > 0)
			{
				FacePointIdx.push_back(j);
			}
			/*if (less_threshod / (more_threshod + less_threshod)>k|| more_threshod / (more_threshod + less_threshod)>k)
			{

			}*/
			if (more_threshod > 0 && less_threshod > 0)
			{
				BoundPointIdx.push_back(j);
			}
			/*else
			{
			BoundPointIdx.push_back(j);
			}*/

			//######### Laster vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;

		}
	}
	//构建特征点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature->resize(AnglePointIdx.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		cloud_feature->points[i].x = cloud_source->points[AnglePointIdx[i]].x;
		cloud_feature->points[i].y = cloud_source->points[AnglePointIdx[i]].y;
		cloud_feature->points[i].z = cloud_source->points[AnglePointIdx[i]].z;
	}
	//构建特征点云
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	pcl::PointCloud<pcl::Normal>::Ptr source_feature_normals(new pcl::PointCloud<pcl::Normal>);
	source_feature_normals->resize(AnglePointIdx.size());
	for (int i = 0; i < AnglePointIdx.size(); i++)
	{
		source_feature_normals->points[i].normal_x = source_normals->points[AnglePointIdx[i]].normal_x;
		source_feature_normals->points[i].normal_y = source_normals->points[AnglePointIdx[i]].normal_y;
		source_feature_normals->points[i].normal_z = source_normals->points[AnglePointIdx[i]].normal_z;
	}
	pfh.setSearchMethod(tree); //设置近邻搜索方式
	pfh.setRadiusSearch(1.85*radius);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_source_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature); //设置输入点云
	pfh.setInputNormals(source_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_source_feature_ptr); //计算每个点的pfh


	////保存特征点云
	//pcl::PointCloud<pcl::PointXYZ> final2 = *cloud_feature;
	//pcl::io::savePLYFile("source_cloud_rough_area.ply", final2);
	////保存特征点云
	
	//cout << "start sort" << endl;
	//compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
	/*cout << "angle sort1 is ready" << endl;
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	cout << "finished source" << endl;*/

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_b->resize(cloud_source->size());
	//for (int i = 0; i < cloud_source->points.size(); i++)
	//{

	//	cloud_b->points[i].x = cloud_source->points[i].x;
	//	cloud_b->points[i].y = cloud_source->points[i].y;
	//	cloud_b->points[i].z = cloud_source->points[i].z;
	//}

	////Eigen::Vector3d d, n1, n2;

	//cout << "ready cloud" << endl;

	////printf("pfh size %d", pfh_fe_ptr->points.size());
	//for (int i = 0; i < AnglePointIdx.size(); i++)
	//{
	//	//cout << "angle:" << AnglePointIdx.size() << endl;
	//	//cout << "ready color" << endl;
	//	cloud_b->at(AnglePointIdx[i]).r = 255;
	//	cloud_b->at(AnglePointIdx[i]).g = 0;
	//	cloud_b->at(AnglePointIdx[i]).b = 0;
	//}
	//for (int i = 0; i < BoundPointIdx.size(); i++)
	//{
	//	//cout << "bound:" << BoundPointIdx.size() << endl;
	//	//cout << "ready color" << endl;
	//	cloud_b->at(BoundPointIdx[i]).r = 0;
	//	cloud_b->at(BoundPointIdx[i]).g = 255;
	//	cloud_b->at(BoundPointIdx[i]).b = 0;
	//}
	//for (int i = 0; i < FacePointIdx.size(); i++)
	//{
	//	//cout << "face:" << FacePointIdx.size() << endl;
	//	//cout << "ready color" << endl;
	//	cloud_b->at(FacePointIdx[i]).r = 100;
	//	cloud_b->at(FacePointIdx[i]).g = 100;
	//	cloud_b->at(FacePointIdx[i]).b = 100;
	//}
	//cout << "总数：" << FacePointIdx.size() + BoundPointIdx.size() + AnglePointIdx.size();
	////savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
	//pcl::PointCloud<pcl::PointXYZRGB> final2 = *cloud_b;
	//pcl::io::savePLYFile("source_cloud.ply", final2);
	cout << "ok" << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target = 0;
	float less_threshod_target = 0;
	float k_target = 0.15;
	float u_radius_target = 0;
	float sear_rad_target = 0;
	float stand_distance_target;
	long counter_target = 0;

	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); i++)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					if (pointRadiusSquaredDistance_target[i] <= (1.5*stand_distance_target))
						//if (pointRadiusSquaredDistance_target[i]<=(1.5*stand_distance_target) && (pointRadiusSquaredDistance_target[i] >= stand_distance_target))
					{

						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				//sear_rad_target = 1.2*u_radius_target * 1000;
				sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}


		}
		/*if (sear_rad_target<0.0015)
		{
		sear_rad_target = 0.0015;
		}*/
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
				{
					pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
				}
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					if (angle1_target>90)
					{
						angle1_target = 180 - angle1_target;
					}
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>15) {

						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//more_threshod_target++;
					}
					else
					{

						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));

						//less_threshod_target++;
					}
				}
			}

			more_threshod_target_idx.push_back(more_threshod_target);
			less_threshod_target_idx.push_back(less_threshod_target);

			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			/*if (more_threshod_target >= less_threshod_target)
			{
			if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
			BoundPointIdx_target.push_back(j);
			}
			else
			{
			AnglePointIdx_target.push_back(j);
			}
			}
			else
			{
			if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
			BoundPointIdx_target.push_back(j);
			}
			else
			{
			FacePointIdx_target.push_back(j);
			}
			}*/
			if (more_threshod_target > 0 && less_threshod_target == 0)
			{
				AnglePointIdx_target.push_back(j);
			}
			if (more_threshod_target == 0 && less_threshod_target > 0)
			{
				FacePointIdx_target.push_back(j);
			}
			/*if (less_threshod / (more_threshod + less_threshod)>k|| more_threshod / (more_threshod + less_threshod)>k)
			{

			}*/
			if (more_threshod_target > 0 && less_threshod_target > 0)
			{
				BoundPointIdx_target.push_back(j);
			}
			/*else
			{
			BoundPointIdx_target.push_back(j);
			}*/
			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	cout << "AnglePointIdx_target.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_feature_target(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_feature_target->resize(AnglePointIdx_target.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		cloud_feature_target->points[i].x = cloud_target->points[AnglePointIdx_target[i]].x;
		cloud_feature_target->points[i].y = cloud_target->points[AnglePointIdx_target[i]].y;
		cloud_feature_target->points[i].z = cloud_target->points[AnglePointIdx_target[i]].z;
	}
	//构建特征点云
	pcl::PointCloud<pcl::Normal>::Ptr target_feature_normals(new pcl::PointCloud<pcl::Normal>);
	target_feature_normals->resize(AnglePointIdx_target.size());
	for (int i = 0; i < AnglePointIdx_target.size(); i++)
	{
		target_feature_normals->points[i].normal_x = target_normals->points[AnglePointIdx_target[i]].normal_x;
		target_feature_normals->points[i].normal_y = target_normals->points[AnglePointIdx_target[i]].normal_y;
		target_feature_normals->points[i].normal_z = target_normals->points[AnglePointIdx_target[i]].normal_z;
	}

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_target_feature_ptr(new pcl::PointCloud<pcl::PFHSignature125>()); //声明一个数据结构，来存储每个点的pfh
	pfh.setInputCloud(cloud_feature_target); //设置输入点云
	pfh.setInputNormals(target_feature_normals); //设置输入点云的法线
	pfh.compute(*pfh_target_feature_ptr); //计算每个点的pfh
	//pcl::PointCloud<pcl::PointXYZ> final1 = *cloud_feature_target;
	//pcl::io::savePLYFile("target_cloud_rough_area.ply", final1);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZRGB>);
	//cloud_c->resize(cloud_target->size());
	//for (int i = 0; i < cloud_target->points.size(); i++)
	//{

	//	cloud_c->points[i].x = cloud_target->points[i].x;
	//	cloud_c->points[i].y = cloud_target->points[i].y;
	//	cloud_c->points[i].z = cloud_target->points[i].z;
	//}



	//cout << "ready cloud" << endl;

	////printf("pfh size %d", pfh_fe_ptr->points.size());
	//for (int i = 0; i < AnglePointIdx_target.size(); i++)
	//{
	//	//cout << "angle:" << AnglePointIdx.size() << endl;
	//	//cout << "ready color" << endl;
	//	cloud_c->at(AnglePointIdx_target[i]).r = 255;
	//	cloud_c->at(AnglePointIdx_target[i]).g = 0;
	//	cloud_c->at(AnglePointIdx_target[i]).b = 0;
	//}
	//for (int i = 0; i < BoundPointIdx_target.size(); i++)
	//{
	//	//cout << "bound:" << BoundPointIdx.size() << endl;
	//	//cout << "ready color" << endl;
	//	cloud_c->at(BoundPointIdx_target[i]).r = 0;
	//	cloud_c->at(BoundPointIdx_target[i]).g = 255;
	//	cloud_c->at(BoundPointIdx_target[i]).b = 0;
	//}
	//for (int i = 0; i < FacePointIdx_target.size(); i++)
	//{
	//	//cout << "face:" << FacePointIdx.size() << endl;
	//	//cout << "ready color" << endl;
	//	cloud_c->at(FacePointIdx_target[i]).r = 100;
	//	cloud_c->at(FacePointIdx_target[i]).g = 100;
	//	cloud_c->at(FacePointIdx_target[i]).b = 100;
	//}
	//cout << "总数：" << FacePointIdx_target.size() + BoundPointIdx_target.size() + AnglePointIdx_target.size();
	////savefile(cloud_b, cloud_normals_ptr, filename, "PFH"); //将点云以.ply文件格式保存
	//pcl::PointCloud<pcl::PointXYZRGB> final1 = *cloud_c;
	//pcl::io::savePLYFile("source_target.ply", final1);
	cout << "ok" << endl;

	int k_search = 3;
	pcl::search::KdTree<pcl::PFHSignature125> pfh_tree;
	
	pfh_tree.setInputCloud(pfh_target_feature_ptr);
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;

	//viewer
	//boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
	//viewer->setBackgroundColor(255, 255, 255);  //设置背景颜色为白色
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> harris_color_handler(Harris_keypoints, 255, 0, 0);
	//											//viewer->addPointCloud<pcl::PointXYZI>(Harris_keypoints, harris_color_handler, "Harris_keypoints");
	//viewer->addPointCloud(cloud_target, "input_cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "input_cloud");
	//viewer->addPointCloud(cloud_feature_target, "input_cloud_target");
	////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler(cloud_feature_target, 255, 0, 0);
	////viewer->addPointCloud<pcl::PointXYZ>(cloud_feature_target, harris_color_handler, "input_cloud_target");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "input_cloud_target");

	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud_target");
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	//}
	//viewer

	//点云染色
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red1(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red1->resize(cloud_feature_target->size());
	for (int i = 0; i < cloud_feature_target->points.size(); i++)
	{

		cloud_source_red1->points[i].x = cloud_feature_target->points[i].x;
		cloud_source_red1->points[i].y = cloud_feature_target->points[i].y;
		cloud_source_red1->points[i].z = cloud_feature_target->points[i].z;
		cloud_source_red1->at(i).r = 255;
		cloud_source_red1->at(i).g = 0;
		cloud_source_red1->at(i).b = 0;
	}
	
	pcl::PointCloud<pcl::PointXYZRGB> final1 = *cloud_source_red1;
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("Angle_feature.ply", final1);
	//////##########点云染色#############//


	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 10000;

	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4*radius;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_feature;

	kdtree_feature.setInputCloud(cloud_feature);
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();

		int id = rand() % cloud_feature->points.size();
		sample_idx.push_back(id);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			//id = rand() % AnglePointIdx.size();

			id = rand() % cloud_feature->points.size();
			searchPoint.x = cloud_feature->points[id].x;
			searchPoint.y = cloud_feature->points[id].y;
			searchPoint.z = cloud_feature->points[id].z;
			kdtree_feature.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target);
			//cout << "pointIdxRadiusSearch_target.size()" << pointIdxRadiusSearch_target.size() << endl;
			if (pointIdxRadiusSearch_target.size()<4)
			{
				continue;
			}
			//cout << "enter" << endl;
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_feature->at(id), cloud_feature->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(id);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				//id = rand() % AnglePointIdx.size();
				id = rand() % cloud_feature->points.size();
				
				sample_idx.push_back(id);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{
			
			pfh_tree.nearestKSearch(*pfh_source_feature_ptr, sample_idx[i], k_search, pfh_index, pfh_distance);
			//pfh_tree.nearestKSearch(search_pfh, k_search, pfh_index, pfh_distance);
			//cout << "is" << i << "counter" << endl;
			//cout << "sample:" << sample_idx[i] << endl;
			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}
		/*for (size_t i = 0; i < k_search; i++)
		{

			cout << "distance of k_search" <<i<<":"<< pfh_distance[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			/*if (id == 0)
			{
				final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
				final_match[1][i] = matches[i].target_idx[id];
			}*/
			final_match[1][i] = matches[i].target_idx[id];

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;
		
		pcl::transformPointCloud(*cloud_feature, *new_source, T.matrix());
		//cout << "77" << endl;
		
		error = computeError(new_source, cloud_feature_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "part error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);
	error = computeError(cloud_source, cloud_target);
	cout << "error:" << error << endl;
	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
void Sac_ia_ANGLE_AUTO_Average(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, float u_rad)
{
	float radius = 2.5*u_rad;
	//float radius = 1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	ne.setSearchMethod(tree);//设置近邻搜索算法
	ne.setKSearch(10);
	//fpfh.setSearchMethod(tree); //设置近邻搜索方式
	//fpfh.setRadiusSearch(radius);
	////########计算Source点云############////
	time_t Start, End;
	Start = clock();
	pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_source);
	ne.compute(*source_normals);
	/////////######计算角点特征#############/////////
	pcl::PointXYZ searchPoint;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_source);
	std::vector<int> pointIdxRadiusSearch;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle;//每个点的周围角度平均
	std::vector<int> AnglePointIdx;//存储角点的序列
	std::vector<int> BoundPointIdx;//存储边界的序列
	std::vector<int> FacePointIdx;//存储面的序列
	std::vector<float> more_threshod_idx;//储存各个点的高评分
	std::vector<float> less_threshod_idx;//储存各个点的低评分
	float angle;
	float angle1;
	float angle_sum = 0;
	float more_threshod = 0;
	float less_threshod = 0;
	float more_threshod1 = 0;
	float less_threshod1 = 0;
	float k = 0.15;
	float u_radius = 0;
	float sear_rad = 0;
	float stand_distance;
	long counter = 0;
	float more_weight = 0;
	float less_weight = 0;
	int more_count = 0;
	int less_count = 0;

	for (int j = 0; j < cloud_source->points.size(); j++) {
		searchPoint.x = cloud_source->points[j].x;
		searchPoint.y = cloud_source->points[j].y;
		searchPoint.z = cloud_source->points[j].z;
		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//
			if (pointRadiusSquaredDistance[1]<(pointRadiusSquaredDistance[2] / 5))
			{
				stand_distance = pointRadiusSquaredDistance[2];


			}
			else
			{
				stand_distance = pointRadiusSquaredDistance[1];

			}
			//cout << "stand_distance：" << stand_distance << endl;
			//cout << "size:" << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				//cout << "point_distance：" << pointRadiusSquaredDistance[i] << endl;
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else {
					//计算1-1.5倍区域点的距离平均
					//&& (pointRadiusSquaredDistance[i]>stand_distance)
					//if (pointRadiusSquaredDistance[i] <= (1.5*stand_distance))
					if (pointRadiusSquaredDistance[i]<=(1.5*stand_distance) && (pointRadiusSquaredDistance[i]>=stand_distance))
					{
						u_radius = u_radius + pointRadiusSquaredDistance[i];
						counter++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance = 0;

			if (pointIdxRadiusSearch.size() == 1)
			{
				sear_rad = 2 * radius;
			}

			else
			{
				u_radius = u_radius / counter;
				//bunny
				//sear_rad = 1.2*u_radius * 1000;
				sear_rad = 1.2*u_radius;
				counter = 0;

			}

		}
		//bunny
		/*if (sear_rad<0.0015)
		{
		sear_rad = 0.0015;
		}*/
		//amo
		/*if (sear_rad<0.35)
		{
		sear_rad = 0.35;
		}*/
		//cout << "sear_rad:"<<sear_rad << endl;
		//cout << "pointIdxRadiusSearch"<<pointIdxRadiusSearch.size() << endl;
		if (kdtree.radiusSearch(searchPoint, sear_rad, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t l = 0; l < pointRadiusSquaredDistance.size(); l++)
			{
				pointRadiusSquaredDistance[l] = point_distance(searchPoint, cloud_source->at(pointIdxRadiusSearch[l]));
			}
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				if (pointRadiusSquaredDistance[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(source_normals->points[j].normal_x,
						source_normals->points[j].normal_y,
						source_normals->points[j].normal_z),

						v2(source_normals->points[pointIdxRadiusSearch[i]].normal_x,
							source_normals->points[pointIdxRadiusSearch[i]].normal_y,
							source_normals->points[pointIdxRadiusSearch[i]].normal_z);

					angle = getAngleTwoVectors(v1, v2);
					angle1 = getAngle3D(v1, v2);
					angle_sum = angle_sum + angle1;
					if (angle1>15) {

						more_threshod = more_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));

						more_weight = more_weight + exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad  - u_radius, 2)));
						more_threshod1 = more_threshod1 + angle1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));
						more_count++;
						//more_threshod++;
					}
					else
					{

						less_threshod = less_threshod + 1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad - u_radius, 2)));

						less_weight = less_weight + exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad  - u_radius, 2)));
						less_threshod1 = less_threshod1 + angle1 * exp(-pow((pointRadiusSquaredDistance[i] - u_radius), 2) / (2 * pow(sear_rad  - u_radius, 2)));
						less_count++;
						//less_threshod++;
					}

				}
			}
			//cout << "more_thre:" << more_threshod << endl;
			//cout << "less_thre:" << less_threshod << endl; 
			if (more_weight==0)
			{
				more_threshod_idx.push_back(0);
				//cout << "more_thre1:" << 0 << endl;
			}
			else
			{
				//more_threshod_idx.push_back(more_threshod1 / more_weight);
				more_threshod_idx.push_back(more_threshod / more_count);
				//cout << "more_thre1:" << more_threshod1 / more_weight << endl;
			}
			if (less_weight==0)
			{
				less_threshod_idx.push_back(0);
				//cout << "less_thre1:" << 0 << endl;
			}
			else
			{
				less_threshod_idx.push_back(less_threshod / less_count);
				//cout << "less_thre1:" << less_threshod1 / less_weight << endl; 
			}
			more_weight = 0;
			less_weight = 0;
			more_threshod1 = 0;
			less_threshod1 = 0;
			more_count = 0;
			less_count = 0;
			pointAngle.push_back(angle_sum / pointIdxRadiusSearch.size());
			//#########newer vision#########
			if (more_threshod >= less_threshod)
			{
				if ((less_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					AnglePointIdx.push_back(j);
				}
			}
			else
			{
				if ((more_threshod / (more_threshod + less_threshod))>k) {
					BoundPointIdx.push_back(j);
				}
				else
				{
					FacePointIdx.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum = 0;
			more_threshod = 0;
			less_threshod = 0;
			u_radius = 0;
			sear_rad = 0;

		}
	}
	//compute_couple_sort(pointAngle, AnglePointIdx, 0, AnglePointIdx.size() - 1);
	//cout << "angle sort1 is ready" << endl;
	cout << "AnglePointIdx.size()" << AnglePointIdx.size() << endl;
	cout << "finished source" << endl;

	////########计算Target点云############////
	pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(cloud_target);
	ne.compute(*target_normals);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_target;
	kdtree_target.setInputCloud(cloud_target);
	std::vector<int> pointIdxRadiusSearch_target;//KD-TREE搜索临近点的序列
	std::vector<float> pointRadiusSquaredDistance_target;//KD-TREE搜索时点的距离
	std::vector<float> pointAngle_target;//每个点的周围角度平均
	std::vector<int> AnglePointIdx_target;//存储角点的序列
	std::vector<int> BoundPointIdx_target;//存储边界的序列
	std::vector<int> FacePointIdx_target;//存储面的序列
	std::vector<float> more_threshod_target_idx;
	std::vector<float> less_threshod_target_idx;
	float angle_target;
	float angle1_target;
	float angle_sum_target = 0;
	float more_threshod_target = 0;
	float less_threshod_target = 0;
	float more_threshod_target1 = 0;
	float less_threshod_target1 = 0;
	float k_target = 0.15;
	float u_radius_target = 0;
	float sear_rad_target = 0;
	float stand_distance_target;
	long counter_target = 0;
	float more_weight_target = 0;
	float less_weight_target = 0;
	float more_count_target = 0;
	float less_count_target = 0;
	for (int j = 0; j < cloud_target->points.size(); j++) {
		searchPoint.x = cloud_target->points[j].x;
		searchPoint.y = cloud_target->points[j].y;
		searchPoint.z = cloud_target->points[j].z;
		if (kdtree_target.radiusSearch(searchPoint, radius, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0) {
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			if (pointRadiusSquaredDistance_target[1]<(pointRadiusSquaredDistance_target[2] / 5))
			{
				stand_distance_target = pointRadiusSquaredDistance_target[2];
			}
			else
			{
				stand_distance_target = pointRadiusSquaredDistance_target[1];
			}
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); i++)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else {
					//if (pointRadiusSquaredDistance_target[i] <= (1.5*stand_distance_target))
					if (pointRadiusSquaredDistance_target[i]<=(1.5*stand_distance_target) && (pointRadiusSquaredDistance_target[i] >= stand_distance_target))
					{

						u_radius_target = u_radius_target + pointRadiusSquaredDistance_target[i];
						counter_target++;
					}
					else
					{
						break;
					}
				}
			}
			stand_distance_target = 0;

			if (pointIdxRadiusSearch_target.size() == 1)
			{
				sear_rad_target = 2 * radius;
			}

			else
			{
				u_radius_target = u_radius_target / counter_target;
				//sear_rad_target = 1.2*u_radius_target * 1000;
				sear_rad_target = 1.2*u_radius_target;
				counter_target = 0;

			}

		}
		/*if (sear_rad_target<0.0015)
		{
		sear_rad_target = 0.0015;
		}*/
		if (kdtree_target.radiusSearch(searchPoint, sear_rad_target, pointIdxRadiusSearch_target, pointRadiusSquaredDistance_target) > 0)
		{
			for (size_t l = 0; l < pointRadiusSquaredDistance_target.size(); l++)
			{
				pointRadiusSquaredDistance_target[l] = point_distance(searchPoint, cloud_target->at(pointIdxRadiusSearch_target[l]));
			}
			//cout << "最近邻点数: " << pointIdxRadiusSearch.size() << endl;
			for (size_t i = 0; i < pointIdxRadiusSearch_target.size(); ++i)
			{
				if (pointRadiusSquaredDistance_target[i] == 0)
				{
					continue;
				}
				else
				{
					Eigen::Vector3d v1(target_normals->points[j].normal_x,
						target_normals->points[j].normal_y,
						target_normals->points[j].normal_z),

						v2(target_normals->points[pointIdxRadiusSearch_target[i]].normal_x,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_y,
							target_normals->points[pointIdxRadiusSearch_target[i]].normal_z);

					angle_target = getAngleTwoVectors(v1, v2);
					angle1_target = getAngle3D(v1, v2);
					angle_sum_target = angle_sum_target + angle1_target;
					if (angle1_target>15) {
			
						more_threshod_target = more_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));
		
						more_weight_target = more_weight_target + exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));
						more_threshod_target1 = more_threshod_target1 + angle1_target * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));
						more_count_target++;
						//more_threshod_target++;
					}
					else
					{
						
						less_threshod_target = less_threshod_target + 1 * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target  - u_radius_target, 2)));
					
						less_weight_target = less_weight_target + exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target  - u_radius_target, 2)));
						less_threshod_target1 = less_threshod_target1 + angle1_target * exp(-pow((pointRadiusSquaredDistance_target[i] - u_radius_target), 2) / (2 * pow(sear_rad_target - u_radius_target, 2)));
						less_count_target++;
						//less_threshod_target++;
					}
				}
			}
			if (more_weight_target == 0)
			{
				more_threshod_target_idx.push_back(0);
			}
			else
			{
				//more_threshod_target_idx.push_back(more_threshod_target1 / more_weight_target);
				more_threshod_target_idx.push_back(more_threshod_target / more_count_target);
			}
			if (less_weight_target == 0)
			{
				less_threshod_target_idx.push_back(0);
			}
			else
			{
				//less_threshod_target_idx.push_back(less_threshod_target1 / less_weight_target);
				less_threshod_target_idx.push_back(less_threshod_target / less_count_target);
			}
			/*more_threshod_target_idx.push_back(more_threshod_target1 / more_weight_target);
			less_threshod_target_idx.push_back(less_threshod_target1 / less_weight_target);*/
			more_weight_target = 0;
			less_weight_target = 0;
			more_threshod_target1 = 0;
			less_threshod_target1 = 0;
			more_count_target = 0;
			less_count_target = 0;
			pointAngle_target.push_back(angle_sum_target / pointIdxRadiusSearch_target.size());
			//#########newer vision#########
			if (more_threshod_target >= less_threshod_target)
			{
				if ((less_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					AnglePointIdx_target.push_back(j);
				}
			}
			else
			{
				if ((more_threshod_target / (more_threshod_target + less_threshod_target))>k) {
					BoundPointIdx_target.push_back(j);
				}
				else
				{
					FacePointIdx_target.push_back(j);
				}
			}
			//######### Newer vision #########
			angle_sum_target = 0;
			more_threshod_target = 0;
			less_threshod_target = 0;
			u_radius_target = 0;
			sear_rad_target = 0;

		}
	}
	End = clock() - Start;
	cout << "特征提取时间：" << End << endl;
	//compute_couple_sort(pointAngle_target, AnglePointIdx_target, 0, AnglePointIdx_target.size() - 1);
	//cout << "angle sort is ready" << endl;
	cout << "AnglePointIdx_target.size()" << AnglePointIdx_target.size() << endl;
	////########计算Target点云############////

	int k_search = 3;
	//pcl::search::KdTree<pcl::FPFHSignature33> pfh_tree;
	//pfh_tree.setInputCloud(pfh_target_ptr);
	//pcl::PFHSignature125 search_pfh;
	std::vector<int> pfh_index(3);
	std::vector<float> pfh_distance;
	std::vector<int> sample_idx;
	int i_iter = 0;
	float error, lowest_error(0);
	int max_iterations_ = 100;

	int cut = 0;
	int sample_size = 3;
	std::vector<regist> matches(sample_size);
	float min_distance = 4;

	Eigen::Matrix4d final_transformation_;
	time_t Start1, End1;
	Start1 = clock();
	cout << AnglePointIdx.size() << endl;
	for (; i_iter < max_iterations_; ++i_iter) {
		//////////选择sample_size个匹配点/////////
		//std::vector<int> sample_idx;
		vector<vector<int>> final_match;
		sample_idx.clear();
		//final_match.clear();
		int id = rand() % AnglePointIdx.size();
		sample_idx.push_back(AnglePointIdx[id]);
		while (sample_idx.size()<sample_size)
		{
			bool valid_sample = true;
			id = rand() % AnglePointIdx.size();
			for (size_t i = 0; i < sample_idx.size(); i++) {
				float distance = point_distance(cloud_source->at(AnglePointIdx[id]), cloud_source->at(sample_idx[i]));
				//cout << "diatance to :" <<i<< distance;
				if (distance < min_distance) {
					valid_sample = false;
					cut++;
					break;
				}
			}
			//cout << cut << endl;
			if (valid_sample) {
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
			if (cut > 100) {
				sample_idx.clear();
				//cout << "reset" << ":" << sample_idx.size() << endl;
				id = rand() % AnglePointIdx.size();
				sample_idx.push_back(AnglePointIdx[id]);
				cut = 0;
			}
		}
		//////////选择sample_size个匹配点/////////
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num " << i << ":" << sample_idx[i] << endl;
		}*/
		//#############为随机的sample_size个点找到k_search个近似的点##############
		int idx_cout = 0;
		float dx1, dx2;
		for (size_t i = 0; i < 3; i++)
		{

			for (size_t j = 0; j < AnglePointIdx_target.size(); j++)
			{
				if (pfh_index.size()<3)
				{
					pfh_index[idx_cout] = AnglePointIdx_target[j];
					idx_cout++;
				}
				else
				{
					//第J个目标的角点与第I个sample点的差距

					//dx1 = pointAngle_target[AnglePointIdx_target[j]] - pointAngle[sample_idx[i]];
					dx1 = sqrt(pow((more_threshod_target_idx[AnglePointIdx_target[j]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[AnglePointIdx_target[j]] - less_threshod_idx[sample_idx[i]]), 2));
					for (size_t k = 0; k < pfh_index.size(); k++)
					{
						//第K个要找的对应点与第I个sample点的差距
						dx2 = sqrt(pow((more_threshod_target_idx[pfh_index[k]] - more_threshod_idx[sample_idx[i]]), 2) + pow((less_threshod_target_idx[pfh_index[k]] - less_threshod_idx[sample_idx[i]]), 2));
						//dx2 = pointAngle_target[pfh_index[k]] - pointAngle[sample_idx[i]];
						if (dx1<dx2) {
							pfh_index[k] = AnglePointIdx_target[j];
						}
					}
				}
			}

			matches[i].source_idx = sample_idx[i];
			matches[i].target_idx = pfh_index;
		}

		//#############为随机的sample_size个点找到k_search个近似的点##############


		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点
		//cout << "begin to random one point " << endl;
		final_match.push_back(sample_idx);
		final_match.push_back(sample_idx);
		for (size_t i = 0; i < sample_idx.size(); i++) {
			int id = rand() % k_search;
			if (id == 0)
			{
				final_match[1][i] = matches[i].target_idx[id + 1];
			}
			else
			{
				final_match[1][i] = matches[i].target_idx[id];
			}

		}
		//cout << "finished random one point" << endl;
		/*for (size_t i = 0; i < 3; i++)
		{
		cout << "the num one point is " << i << ":" << final_match[1][i] << endl;
		}*/

		//##########对选出来的sample_size个点，在各自匹配的ｋ个目标点云点中随机选择一个作为匹配点

		//##############根据选出的三对匹配点计算相应的旋转与位移############
		vector<pcl::PointXYZ> pt1;
		vector<pcl::PointXYZ> pt2;
		//cout << "11" << endl;
		for (size_t i = 0; i < sample_size; i++) {
			pt1.push_back(cloud_target->at(final_match[1][i]));
			pt2.push_back(cloud_source->at(matches[i].source_idx));
		}
		//cout << "22" << endl;
		// 计算匹配点的中心
		pcl::PointXYZ p1(0, 0, 0);
		pcl::PointXYZ p2(0, 0, 0);
		for (size_t i = 0; i < sample_size; i++) {
			p1.x += pt1[i].x; p1.y += pt1[i].y; p1.z += pt1[i].z;
			p2.x += pt2[i].x; p2.y += pt2[i].y; p2.z += pt2[i].z;
		}
		//cout << "33" << endl;
		p1.x /= sample_size; p1.y /= sample_size; p1.z /= sample_size;
		p2.x /= sample_size; p2.y /= sample_size; p2.z /= sample_size;
		vector<pcl::PointXYZ> q1(sample_size), q2(sample_size);
		for (size_t i = 0; i < sample_size; i++) {
			q1[i].x = pt1[i].x - p1.x; q1[i].y = pt1[i].y - p1.y; q1[i].z = pt1[i].z - p1.z;
			q2[i].x = pt2[i].x - p2.x; q2[i].y = pt2[i].y - p2.y; q2[i].z = pt2[i].z - p2.z;
		}
		//cout << "44" << endl;
		Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < sample_size; i++)
			W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
		// SVD
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV();

		Eigen::Matrix3d R = U * V.transpose();
		Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
		Eigen::Isometry3d T(R);
		//cout << "55" << endl;
		T.pretranslate(t);
		//##############根据选出的三对匹配点计算相应的旋转与位移############
		pcl::PointCloud<pcl::PointXYZ>::Ptr new_source(new pcl::PointCloud<pcl::PointXYZ>);

		//cout << "66" << endl;
		pcl::transformPointCloud(*cloud_source, *new_source, T.matrix());
		//cout << "77" << endl;
		error = computeError(new_source, cloud_target);
		//cout << "88" << endl;
		if (i_iter == 0 || error < lowest_error)
		{
			lowest_error = error;
			final_transformation_ = T.matrix();
		}
		//cout << "99" << endl;
	}
	End1 = clock() - Start1;
	cout << "配准时间：" << End1 << endl;
	cout << "error:" << lowest_error << endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source, final_transformation_);

	//////##########点云染色#############//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_red(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_source_red->resize(cloud_source->size());
	for (int i = 0; i < cloud_source->points.size(); i++)
	{

		cloud_source_red->points[i].x = cloud_source->points[i].x;
		cloud_source_red->points[i].y = cloud_source->points[i].y;
		cloud_source_red->points[i].z = cloud_source->points[i].z;
		cloud_source_red->at(i).r = 255;
		cloud_source_red->at(i).g = 0;
		cloud_source_red->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_green(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_target_green->resize(cloud_target->size());
	for (int i = 0; i < cloud_target->points.size(); i++)
	{

		cloud_target_green->points[i].x = cloud_target->points[i].x;
		cloud_target_green->points[i].y = cloud_target->points[i].y;
		cloud_target_green->points[i].z = cloud_target->points[i].z;
		cloud_target_green->at(i).r = 0;
		cloud_target_green->at(i).g = 255;
		cloud_target_green->at(i).b = 0;
	}
	pcl::PointCloud<pcl::PointXYZRGB> final = *cloud_source_red;
	final += *cloud_target_green;
	//////##########点云染色#############//
	//pcl::io::savePCDFile("result.pcd", final);
	pcl::io::savePLYFile("result.ply", final);

	system("pause");

}
int main()
{
	string filename;
	for (int i = 0; i < 125; i++) {
		u_pfh[i] = 0;
	}
	
	filename = "E:/model/bun/source1.ply";
	string filename1 = "E:/model/bun/trans.ply";

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrigin(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader reader;
	reader.read(filename, *cloudOrigin);

	////////正常测试/////////
	reader.read(filename1, *cloud_source);
	////////正常测试/////////

	float u_radiu = compute_u_cloud(cloudOrigin);
	//addGaussnoise(cloud_source, 0, 0.005);

	////////////用于测试噪点////////////
	/*reader.read(filename1, *cloud_source1);
	addGaussnoise_random(cloud_source1, 0, 0.28, 0.1);
	string filename2 = "D:/Documents/Visual Studio 2015/Projects/Project4/Project4/noise3.ply";
	reader.read(filename2, *cloud_source);*/
	////////////用于测试噪点////////////
	
	////////////用于测试密度////////////
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source3(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOrigin3(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::RandomSample<pcl::PointXYZ> rs;	//创建滤波器对象
	//rs.setInputCloud(cloud_source);				//设置待滤波点云
	//rs.setSample(170000);					//设置下采样点云的点数
	////rs.setSeed(1);						//设置随机函数种子点
	//rs.filter(*cloud_source3);					//执行下采样滤波，保存滤波结果于cloud_sub

	//rs.setInputCloud(cloudOrigin);				//设置待滤波点云
	//rs.setSample(170000);					//设置下采样点云的点数
	////rs.setSeed(1);						//设置随机函数种子点
	//rs.filter(*cloudOrigin3);					//执行下采样滤波，保存滤波结果于cloud_sub
	////////////用于测试密度////////////

	//reader.read(filename, *cloud_normals);
	//computePFH(cloudOrigin, cloud_normals, "bunny.ply");
	//computePFH(cloudOrigin, "bunny.ply");
	//computeANGLE_Point(cloudOrigin, "bunny.ply");
	//Sac_ia_PFH(cloud_source, cloudOrigin);
	//Sac_ia_FPFH(cloud_source, cloudOrigin);
	//Sac_ia_ANGLE_BUFF(cloud_source, cloudOrigin);
	//Sac_ia_ANGLE_AMO(cloud_source, cloudOrigin);
	//Sac_ia_ANGLE_FPFH(cloud_source, cloudOrigin);
	//Sac_ia_ANGLE_AUTO(cloud_source, cloudOrigin, u_radiu);
	//Sac_ia_ANGLE_AUTO_noise(cloud_source, cloudOrigin, u_radiu);
	//Sac_ia_ANGLE_AUTO_Average(cloud_source, cloudOrigin, u_radiu);
	//Computer_Harris(cloudOrigin);
	//Sac_ia_rough_area_ANGLE_AUTO_noise(cloud_source, cloudOrigin, u_radiu);
	//Sac_ia_rough_area_ANGLE_AUTO_noise(cloudOrigin, cloud_source, u_radiu);
	//Computer_Harris(cloudOrigin);
	//Computer_Harris_err(cloud_source, cloudOrigin, u_radiu);
	//Sac_ia_FPFH_point(cloud_source, cloudOrigin, u_radiu);
	//Computer_two_cloud(cloud_source, cloudOrigin);
	//Sac_ia_rough_area_double_param_test(cloud_source, cloudOrigin, u_radiu);
	//Sac_ia_rough_area_double_param_test_more_points(cloud_source, cloudOrigin, u_radiu);
	//Sac_ia_rough_area_double_param_test_limit_anglepoint(cloud_source, cloudOrigin, u_radiu);
	//Sac_ia_rough_area_double_param_test_diff_num(cloud_source3, cloudOrigin3,cloud_source, cloudOrigin, u_radiu);
	delete[] L2_dis;
	delete[] pfh_sort_num;
	return 0;
}
