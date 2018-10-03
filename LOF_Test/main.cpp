/*-------------------------------------
1.基于正态分布的点云离群点检测算法
2.基于VoxelGrid的点云下采样算法
3.作者：pcb
4.日期：2018.10.3
--------------------------------------*/

#include <iostream>
#include <fstream>
#include<algorithm> 
#include <math.h>
#include <vector>
#include <Windows.h>
#include<time.h>


typedef long long       int64_t;
typedef int             int32_t;

using namespace std;


//定义3D点的结构体
struct Point3D
{
	float x;
	float y;
	float z;
};

struct PointKDistance1
{
	Point3D point;
	float Distance;           //存放距离的向量
};

//存放计算每个点云的idx和cloud_point_index的结构体
struct cloud_point_index_idx
{
	unsigned int idx;
	unsigned int cloud_point_index;

	cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}
	bool operator < (const cloud_point_index_idx &p) const { return (idx < p.idx); }
};

struct Array4f
{
	float x;
	float y;
	float z;
	float C;
};

/*----------------------------
* 功能 : 读取一个txt中的数据，将数据放入vector中
*----------------------------
* 函数 : ReadData
* 参数 : str [in]   需要读的txt文件名
* 参数 : Data [in]  读取xt文件数据存放在Data中
*/
int ReadData(const char* str, vector<Point3D>&Data)
{
	fstream ReadDataTxt;
	float X, Y, Z;                                     //用于读取TXT中的数据
	Point3D param;                                     //创建一个用于存储X,Y,X的Point3D结构体
	ReadDataTxt.open(str);
	while (ReadDataTxt >> X >> Y >> Z)
	{
		if (Z >= 400)
		{
			param.x = X;
			param.y = Y;
			param.z = Z;
			Data.push_back(param);
		}
		else
		{
			continue;
		}
	}
	ReadDataTxt.close();

	size_t DataSize = Data.size();

	return 0;
}

/*----------------------------
* 功能 : 向一个txt中写数据
*----------------------------
* 函数 :WriteData
* 参数 : str [in]   需要写的txt文件名
* 参数 : Data [in]  需要写的txt文件数据存放在Data中
*/

int WriteData(string str, vector<Point3D>&Data)
{
	fstream WriteTXT;
	WriteTXT.open("11.txt");
	for (int i = 0; i < Data.size(); i++)
	{
		if (Data[i].x!=0)
		{
			WriteTXT << Data[i].x << " " << Data[i].y << " " << Data[i].z << endl;
		}
		
	}

	WriteTXT.close();
	return 0;
}
/*----------------------------
*功能：计算两点之间的欧几里得距离
*-----------------------------
*输入：两个Point3D结构体类型的点
*输出：两点之间的欧几里得距离
*/

float XYZDistance(Point3D &point1,Point3D&point2)
{
	float Distance_X = (point1.x - point2.x)*(point1.x - point2.x);
	float Distance_Y = (point1.y - point2.y)*(point1.y - point2.y);
	float Distance_Z = (point1.z - point2.z)*(point1.z - point2.z);
	return sqrt(Distance_X+Distance_Y+Distance_Z);

}

/*----------------------------
*功能：冒泡排序法
*-----------------------------
*输入：vector<float>类型的值
*输出：从小到大排列的vector<float>
*/
void BubbleSort(vector <PointKDistance1>&BubbleSortVector)
{
	size_t Num = BubbleSortVector.size();
	for (int i = 0; i < Num;i++)
	{
		for (int j = i ; j < Num;j++)
		{
			if (BubbleSortVector[i].Distance>BubbleSortVector[j].Distance)
			{
				float temp = BubbleSortVector[i].Distance;
				Point3D point12 = BubbleSortVector[i].point;
				BubbleSortVector[i] = BubbleSortVector[j];
				BubbleSortVector[j].Distance= temp;
				BubbleSortVector[j].point = point12;
			}
		}
	}

}

/*----------------------------
*功能：采用高斯分布的方法进行离群点的判别
*-----------------------------
*输入：Piont3D的原始点云数据
*输出：除去离群点之后的Point3D结构的点云数据
*/
void GaussianDistribution_OutlierDetection(vector<Point3D> &InputPointCloud, vector<Point3D>&OutPointCloud)
{
	//均值
	double X_Ave = 0;     
	double Y_Ave = 0; 
	double Z_Ave = 0;
	
	//方差
	double X_Var = 0;      
	double Y_Var = 0;
	double Z_Var = 0;
	
	//求均值
	for (int i = 0; i <InputPointCloud.size(); i++)
	{
		X_Ave +=InputPointCloud[i].x;
		Y_Ave +=InputPointCloud[i].y;
		Z_Ave +=InputPointCloud[i].z;
	}

	X_Ave = X_Ave /InputPointCloud.size();
	Y_Ave = Y_Ave /InputPointCloud.size();
	Z_Ave = Z_Ave /InputPointCloud.size();

	//求方差
	for (int j = 0; j <InputPointCloud.size(); j++)
	{
		X_Var += (InputPointCloud[j].x - X_Ave)*(InputPointCloud[j].x - X_Ave);
		Y_Var += (InputPointCloud[j].y - Y_Ave)*(InputPointCloud[j].y - Y_Ave);
		Z_Var += (InputPointCloud[j].z - Z_Ave)*(InputPointCloud[j].z - Z_Ave);
	}
	X_Var = X_Var /InputPointCloud.size();
	Y_Var = Y_Var /InputPointCloud.size();
	Z_Var = Z_Var /InputPointCloud.size();

	//开始判断是是否是离群点
	for (int k = 0; k <InputPointCloud.size(); k++)
	{
		if (((abs(InputPointCloud[k].z - Z_Ave) / sqrt(Z_Var))<1.5 || (abs(InputPointCloud[k].z + Z_Ave) / sqrt(Z_Var))<1.5)
			&& ((abs(InputPointCloud[k].y - Y_Ave) / sqrt(Y_Var))<1.8 || (abs(InputPointCloud[k].y + Y_Ave) / sqrt(Y_Var))<1.8)
			&& ((abs(InputPointCloud[k].x - X_Ave) / sqrt(X_Var))<1.8 || (abs(InputPointCloud[k].x + X_Ave) / sqrt(X_Var))<1.))
		{
			OutPointCloud.push_back(InputPointCloud[k]);                     //如果不是离群点，则把该点放入CloudPoint中
		}
		
	}
	return;
}


/*----------------------------
*功能：找到输入点云中的包围盒两个点的值（右上和坐下）
*-----------------------------
*输入：Piont3D的原始点云数据
*输出：找到之后的min_p和max_p
*/
void GetMaxMin(vector<Point3D>&InputCloudPoint, Array4f&min_p, Array4f&max_p)
{
	//主要思路是找到x，y,z的最小值,这样就能得到点云立体包围的次村
	//找x,y,z最小值
	if (InputCloudPoint.size() == 0)
	{
		cout << "输入点云为空" << endl;
		return;
	}
	float x_min= (*min_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.x < b.x;})).x;
	float y_min =(*min_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.y < b.y;})).y;
	float z_min =(*min_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.z < b.z;})).z;
	//给min_p赋值
	min_p.x = x_min;
	min_p.y = y_min;
	min_p.z = z_min;
	min_p.C = 1;


	//找x,y,z的最大值
	float x_max = (*max_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.x < b.x; })).x;
	float y_max = (*max_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.y < b.y; })).y;
	float z_max = (*max_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.z < b.z; })).z;
	
	//给max_p赋值
	max_p.x = x_max;
	max_p.y = y_max;
	max_p.z = z_max;
	max_p.C = 1;

	return;
}

/*----------------------------
*功能：体素化网格方法实现下采样（PCL中的源码C++实现）
*-----------------------------
*输入：Piont3D的原始点云数据,下采样的体素大小x,y,z
*输出：采样之后的之后的Point3D结构的点云数据
*/
void VoxelGrid_ApplyFilter(vector<Point3D>&InputCloudPoint, vector<Point3D>&OutPointCloud, float X_Voxel, float Y_Voxel, float Z_Voxel)
{
	//先判断输入的点云是否为空
	if (InputCloudPoint.size()==0)
	{
		cout << "输入点云为空！" << endl;
		return;
	}

	//存放输入点云的最大与最小坐标
	Array4f min_p, max_p;
	GetMaxMin(InputCloudPoint, min_p, max_p);

	Array4f inverse_leaf_size_;
	inverse_leaf_size_.x = 1 / X_Voxel;
	inverse_leaf_size_.y = 1 / Y_Voxel;
	inverse_leaf_size_.z = 1 / Z_Voxel;
	inverse_leaf_size_.C = 1;
    
	//计算最小和最大边界框值
	Array4f min_b_, max_b_, div_b_, divb_mul_;
	min_b_.x = static_cast<int> (floor(min_p.x * inverse_leaf_size_.x));
	max_b_.x = static_cast<int> (floor(max_p.x * inverse_leaf_size_.x));
	min_b_.y = static_cast<int> (floor(min_p.y * inverse_leaf_size_.y));
	max_b_.y = static_cast<int> (floor(max_p.y * inverse_leaf_size_.y));
	min_b_.z = static_cast<int> (floor(min_p.z * inverse_leaf_size_.z));
	max_b_.z = static_cast<int> (floor(max_p.z * inverse_leaf_size_.z));

	//计算沿所有轴所需的分割数
	div_b_.x = max_b_.x - min_b_.x + 1;
	div_b_.y = max_b_.y - min_b_.y + 1;
	div_b_.z = max_b_.z - min_b_.z + 1;
	div_b_.C= 0;

	//设置除法乘数
	divb_mul_.x = 1;
	divb_mul_.y = div_b_.x;
	divb_mul_.z =div_b_.x * div_b_.y;
	divb_mul_.C = 0;

	//用于计算idx和pointcloud索引的存储
	std::vector<cloud_point_index_idx> index_vector;
	index_vector.reserve(InputCloudPoint.size());

	//第一步：遍历所有点并将它们插入到具有计算idx的index_vector向量中;具有相同idx值的点将有助于产生CloudPoint的相同点
	for (int i = 0; i < InputCloudPoint.size();i++)
	{
		int ijk0 = static_cast<int> (floor(InputCloudPoint[i].x * inverse_leaf_size_.x) - static_cast<float> (min_b_.x));
		int ijk1 = static_cast<int> (floor(InputCloudPoint[i].y * inverse_leaf_size_.y) - static_cast<float> (min_b_.y));
		int ijk2 = static_cast<int> (floor(InputCloudPoint[i].z * inverse_leaf_size_.z) - static_cast<float> (min_b_.z));

		//计算质心叶索引
		int idx = ijk0 * divb_mul_.x + ijk1 * divb_mul_.y + ijk2 * divb_mul_.z;
		index_vector.push_back(cloud_point_index_idx(static_cast<unsigned int> (idx), i));
	}
	//第二步：使用表示目标单元格的值作为索引对index_vector向量进行排序;实际上属于同一输出单元格的所有点都将彼此相邻
	std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

	//第三步：计数输出单元格，我们需要跳过所有相同的，相邻的idx值
	unsigned int total = 0;
	unsigned int index = 0;
	unsigned int min_points_per_voxel_ = 0;
	//first_and_last_indices_vector [i]表示属于对应于第i个输出点的体素的index_vector中的第一个点的index_vector中的索引，以及不属于第一个点的索引
	std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
	first_and_last_indices_vector.reserve(index_vector.size());                              //分配内存空间

	while (index < index_vector.size())
	{
		unsigned int i = index + 1;
		while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
			++i;
		if (i - index >= min_points_per_voxel_)
		{
			++total;
			first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int>(index, i));
		}
		index = i;
	}

	//第四步：计算质心，将它们插入最终位置
	//OutPointCloud.resize(total);      //给输出点云分配内存空间
	float x_Sum, y_Sum, z_Sum;
	Point3D PointCloud;
	unsigned int first_index, last_index;
	for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
	{
		// 计算质心 - 来自所有输入点的和值，这些值在index_vector数组中具有相同的idx值
		first_index = first_and_last_indices_vector[cp].first;
		last_index = first_and_last_indices_vector[cp].second;
		x_Sum = 0;
		y_Sum = 0;
		z_Sum = 0;
		for (unsigned int li = first_index; li < last_index; ++li)
		{
			x_Sum += InputCloudPoint[index_vector[li].cloud_point_index].x;
			y_Sum += InputCloudPoint[index_vector[li].cloud_point_index].y;
			z_Sum += InputCloudPoint[index_vector[li].cloud_point_index].z;
		}
		PointCloud.x = x_Sum / (last_index - first_index);
		PointCloud.y = y_Sum / (last_index - first_index);
		PointCloud.z = z_Sum / (last_index - first_index);
		OutPointCloud.push_back(PointCloud);
	}

	return;
}




int main()
{

	const char*str = "PointCloud11.txt";
	string WriteTxt = "11.txt";
	vector<Point3D>Data;
	vector<Point3D>Data1;
	vector<Point3D>Data2;
	ReadData(str, Data);
	
	SYSTEMTIME sys;
	GetLocalTime(&sys);                                                                          //得到系统时间
	double Time0 = sys.wMinute * 60 * 1000 + sys.wSecond * 1000 + sys.wMilliseconds;
	int Point = 0;
	
	VoxelGrid_ApplyFilter(Data,Data1,3,3,3);
	GaussianDistribution_OutlierDetection(Data1, Data2);
	GetLocalTime(&sys);                                                                          //得到系统时间
	double Time1 = sys.wMinute * 60 * 1000 + sys.wSecond * 1000 + sys.wMilliseconds;
	cout << Time1 - Time0 << endl;
	WriteData(WriteTxt, Data2);

	return 0;


}
