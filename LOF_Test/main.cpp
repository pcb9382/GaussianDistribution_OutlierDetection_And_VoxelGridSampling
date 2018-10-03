/*-------------------------------------
1.������̬�ֲ��ĵ�����Ⱥ�����㷨
2.����VoxelGrid�ĵ����²����㷨
3.���ߣ�pcb
4.���ڣ�2018.10.3
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


//����3D��Ľṹ��
struct Point3D
{
	float x;
	float y;
	float z;
};

struct PointKDistance1
{
	Point3D point;
	float Distance;           //��ž��������
};

//��ż���ÿ�����Ƶ�idx��cloud_point_index�Ľṹ��
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
* ���� : ��ȡһ��txt�е����ݣ������ݷ���vector��
*----------------------------
* ���� : ReadData
* ���� : str [in]   ��Ҫ����txt�ļ���
* ���� : Data [in]  ��ȡxt�ļ����ݴ����Data��
*/
int ReadData(const char* str, vector<Point3D>&Data)
{
	fstream ReadDataTxt;
	float X, Y, Z;                                     //���ڶ�ȡTXT�е�����
	Point3D param;                                     //����һ�����ڴ洢X,Y,X��Point3D�ṹ��
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
* ���� : ��һ��txt��д����
*----------------------------
* ���� :WriteData
* ���� : str [in]   ��Ҫд��txt�ļ���
* ���� : Data [in]  ��Ҫд��txt�ļ����ݴ����Data��
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
*���ܣ���������֮���ŷ����þ���
*-----------------------------
*���룺����Point3D�ṹ�����͵ĵ�
*���������֮���ŷ����þ���
*/

float XYZDistance(Point3D &point1,Point3D&point2)
{
	float Distance_X = (point1.x - point2.x)*(point1.x - point2.x);
	float Distance_Y = (point1.y - point2.y)*(point1.y - point2.y);
	float Distance_Z = (point1.z - point2.z)*(point1.z - point2.z);
	return sqrt(Distance_X+Distance_Y+Distance_Z);

}

/*----------------------------
*���ܣ�ð������
*-----------------------------
*���룺vector<float>���͵�ֵ
*�������С�������е�vector<float>
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
*���ܣ����ø�˹�ֲ��ķ���������Ⱥ����б�
*-----------------------------
*���룺Piont3D��ԭʼ��������
*�������ȥ��Ⱥ��֮���Point3D�ṹ�ĵ�������
*/
void GaussianDistribution_OutlierDetection(vector<Point3D> &InputPointCloud, vector<Point3D>&OutPointCloud)
{
	//��ֵ
	double X_Ave = 0;     
	double Y_Ave = 0; 
	double Z_Ave = 0;
	
	//����
	double X_Var = 0;      
	double Y_Var = 0;
	double Z_Var = 0;
	
	//���ֵ
	for (int i = 0; i <InputPointCloud.size(); i++)
	{
		X_Ave +=InputPointCloud[i].x;
		Y_Ave +=InputPointCloud[i].y;
		Z_Ave +=InputPointCloud[i].z;
	}

	X_Ave = X_Ave /InputPointCloud.size();
	Y_Ave = Y_Ave /InputPointCloud.size();
	Z_Ave = Z_Ave /InputPointCloud.size();

	//�󷽲�
	for (int j = 0; j <InputPointCloud.size(); j++)
	{
		X_Var += (InputPointCloud[j].x - X_Ave)*(InputPointCloud[j].x - X_Ave);
		Y_Var += (InputPointCloud[j].y - Y_Ave)*(InputPointCloud[j].y - Y_Ave);
		Z_Var += (InputPointCloud[j].z - Z_Ave)*(InputPointCloud[j].z - Z_Ave);
	}
	X_Var = X_Var /InputPointCloud.size();
	Y_Var = Y_Var /InputPointCloud.size();
	Z_Var = Z_Var /InputPointCloud.size();

	//��ʼ�ж����Ƿ�����Ⱥ��
	for (int k = 0; k <InputPointCloud.size(); k++)
	{
		if (((abs(InputPointCloud[k].z - Z_Ave) / sqrt(Z_Var))<1.5 || (abs(InputPointCloud[k].z + Z_Ave) / sqrt(Z_Var))<1.5)
			&& ((abs(InputPointCloud[k].y - Y_Ave) / sqrt(Y_Var))<1.8 || (abs(InputPointCloud[k].y + Y_Ave) / sqrt(Y_Var))<1.8)
			&& ((abs(InputPointCloud[k].x - X_Ave) / sqrt(X_Var))<1.8 || (abs(InputPointCloud[k].x + X_Ave) / sqrt(X_Var))<1.))
		{
			OutPointCloud.push_back(InputPointCloud[k]);                     //���������Ⱥ�㣬��Ѹõ����CloudPoint��
		}
		
	}
	return;
}


/*----------------------------
*���ܣ��ҵ���������еİ�Χ���������ֵ�����Ϻ����£�
*-----------------------------
*���룺Piont3D��ԭʼ��������
*������ҵ�֮���min_p��max_p
*/
void GetMaxMin(vector<Point3D>&InputCloudPoint, Array4f&min_p, Array4f&max_p)
{
	//��Ҫ˼·���ҵ�x��y,z����Сֵ,�������ܵõ����������Χ�Ĵδ�
	//��x,y,z��Сֵ
	if (InputCloudPoint.size() == 0)
	{
		cout << "�������Ϊ��" << endl;
		return;
	}
	float x_min= (*min_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.x < b.x;})).x;
	float y_min =(*min_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.y < b.y;})).y;
	float z_min =(*min_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.z < b.z;})).z;
	//��min_p��ֵ
	min_p.x = x_min;
	min_p.y = y_min;
	min_p.z = z_min;
	min_p.C = 1;


	//��x,y,z�����ֵ
	float x_max = (*max_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.x < b.x; })).x;
	float y_max = (*max_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.y < b.y; })).y;
	float z_max = (*max_element(InputCloudPoint.begin(), InputCloudPoint.end(), [](Point3D& a, Point3D& b){return a.z < b.z; })).z;
	
	//��max_p��ֵ
	max_p.x = x_max;
	max_p.y = y_max;
	max_p.z = z_max;
	max_p.C = 1;

	return;
}

/*----------------------------
*���ܣ����ػ����񷽷�ʵ���²�����PCL�е�Դ��C++ʵ�֣�
*-----------------------------
*���룺Piont3D��ԭʼ��������,�²��������ش�Сx,y,z
*���������֮���֮���Point3D�ṹ�ĵ�������
*/
void VoxelGrid_ApplyFilter(vector<Point3D>&InputCloudPoint, vector<Point3D>&OutPointCloud, float X_Voxel, float Y_Voxel, float Z_Voxel)
{
	//���ж�����ĵ����Ƿ�Ϊ��
	if (InputCloudPoint.size()==0)
	{
		cout << "�������Ϊ�գ�" << endl;
		return;
	}

	//���������Ƶ��������С����
	Array4f min_p, max_p;
	GetMaxMin(InputCloudPoint, min_p, max_p);

	Array4f inverse_leaf_size_;
	inverse_leaf_size_.x = 1 / X_Voxel;
	inverse_leaf_size_.y = 1 / Y_Voxel;
	inverse_leaf_size_.z = 1 / Z_Voxel;
	inverse_leaf_size_.C = 1;
    
	//������С�����߽��ֵ
	Array4f min_b_, max_b_, div_b_, divb_mul_;
	min_b_.x = static_cast<int> (floor(min_p.x * inverse_leaf_size_.x));
	max_b_.x = static_cast<int> (floor(max_p.x * inverse_leaf_size_.x));
	min_b_.y = static_cast<int> (floor(min_p.y * inverse_leaf_size_.y));
	max_b_.y = static_cast<int> (floor(max_p.y * inverse_leaf_size_.y));
	min_b_.z = static_cast<int> (floor(min_p.z * inverse_leaf_size_.z));
	max_b_.z = static_cast<int> (floor(max_p.z * inverse_leaf_size_.z));

	//����������������ķָ���
	div_b_.x = max_b_.x - min_b_.x + 1;
	div_b_.y = max_b_.y - min_b_.y + 1;
	div_b_.z = max_b_.z - min_b_.z + 1;
	div_b_.C= 0;

	//���ó�������
	divb_mul_.x = 1;
	divb_mul_.y = div_b_.x;
	divb_mul_.z =div_b_.x * div_b_.y;
	divb_mul_.C = 0;

	//���ڼ���idx��pointcloud�����Ĵ洢
	std::vector<cloud_point_index_idx> index_vector;
	index_vector.reserve(InputCloudPoint.size());

	//��һ�����������е㲢�����ǲ��뵽���м���idx��index_vector������;������ͬidxֵ�ĵ㽫�����ڲ���CloudPoint����ͬ��
	for (int i = 0; i < InputCloudPoint.size();i++)
	{
		int ijk0 = static_cast<int> (floor(InputCloudPoint[i].x * inverse_leaf_size_.x) - static_cast<float> (min_b_.x));
		int ijk1 = static_cast<int> (floor(InputCloudPoint[i].y * inverse_leaf_size_.y) - static_cast<float> (min_b_.y));
		int ijk2 = static_cast<int> (floor(InputCloudPoint[i].z * inverse_leaf_size_.z) - static_cast<float> (min_b_.z));

		//��������Ҷ����
		int idx = ijk0 * divb_mul_.x + ijk1 * divb_mul_.y + ijk2 * divb_mul_.z;
		index_vector.push_back(cloud_point_index_idx(static_cast<unsigned int> (idx), i));
	}
	//�ڶ�����ʹ�ñ�ʾĿ�굥Ԫ���ֵ��Ϊ������index_vector������������;ʵ��������ͬһ�����Ԫ������е㶼���˴�����
	std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

	//�����������������Ԫ��������Ҫ����������ͬ�ģ����ڵ�idxֵ
	unsigned int total = 0;
	unsigned int index = 0;
	unsigned int min_points_per_voxel_ = 0;
	//first_and_last_indices_vector [i]��ʾ���ڶ�Ӧ�ڵ�i�����������ص�index_vector�еĵ�һ�����index_vector�е��������Լ������ڵ�һ���������
	std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
	first_and_last_indices_vector.reserve(index_vector.size());                              //�����ڴ�ռ�

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

	//���Ĳ����������ģ������ǲ�������λ��
	//OutPointCloud.resize(total);      //��������Ʒ����ڴ�ռ�
	float x_Sum, y_Sum, z_Sum;
	Point3D PointCloud;
	unsigned int first_index, last_index;
	for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
	{
		// �������� - �������������ĺ�ֵ����Щֵ��index_vector�����о�����ͬ��idxֵ
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
	GetLocalTime(&sys);                                                                          //�õ�ϵͳʱ��
	double Time0 = sys.wMinute * 60 * 1000 + sys.wSecond * 1000 + sys.wMilliseconds;
	int Point = 0;
	
	VoxelGrid_ApplyFilter(Data,Data1,3,3,3);
	GaussianDistribution_OutlierDetection(Data1, Data2);
	GetLocalTime(&sys);                                                                          //�õ�ϵͳʱ��
	double Time1 = sys.wMinute * 60 * 1000 + sys.wSecond * 1000 + sys.wMilliseconds;
	cout << Time1 - Time0 << endl;
	WriteData(WriteTxt, Data2);

	return 0;


}
