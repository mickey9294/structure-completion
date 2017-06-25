#include "MeshRelation.h"

using namespace trimesh;

MeshRelation::MeshRelation(){
	//Transform.setToIdentity();
}

MeshRelation::~MeshRelation(){
	//Transform.setToIdentity();
}

MeshRelation::MeshRelation(MyMesh m1, MyMesh m2){ // 将m1的坐标系转到m2的坐标系下，所求矩阵可用于，将m1坐标系中的模型旋转到m2中（即在m1坐标系中画出m2后，移动m2)
	if (m1.obbCorner.size() == 0 || m2.obbCorner.size() == 0)
		return;

	//首先寻找attach点
	std::vector<point> intersect;
	for (int i = 0; i < m1.vertices.size(); i++){
		if (m2.obb.containPoint(synthesis::Vec3(m1.vertices[i][0], m1.vertices[i][1], m1.vertices[i][2])))
			intersect.push_back(m1.vertices[i]);
	}
	for (int i = 0; i < m2.vertices.size(); i++){
		if (m1.obb.containPoint(synthesis::Vec3(m2.vertices[i][0], m2.vertices[i][1], m2.vertices[i][2])))
			intersect.push_back(m2.vertices[i]);
	}

	point total(0, 0, 0);
	for (int i = 0; i < intersect.size(); i++){
		total += intersect[i];
	}
	
	point attachInOriCord;
	if (intersect.size() > 0)
		attachInOriCord = point(total[0] / intersect.size(), total[1] / intersect.size(), total[2] / intersect.size());
	else{
		point a, b;
		float minDis = 99999;
		for (int i = 0; i < m1.vertices.size(); i++)
			for (int j = 0; j < m2.vertices.size(); j++){
				if (dist(m1.vertices[i], m2.vertices[j]) < minDis){
					minDis = dist(m1.vertices[i], m2.vertices[j]);
					a = m1.vertices[i];
					b = m2.vertices[j];
				}
			}
		attachInOriCord = point((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2);
	}
	
	//求局部坐标系CORD1下的交点坐标
	attachPointInM1Cord = newCoordinate(attachInOriCord, m1.xAxis, m1.yAxis, m1.zAxis, m1.extX, m1.extY, m1.extZ, m1.meshCenter);

	//求局部坐标系CORD2下的交点坐标
	attachPointInM2Cord = newCoordinate(attachInOriCord, m2.xAxis, m2.yAxis, m2.zAxis, m2.extX, m2.extY, m2.extZ, m2.meshCenter);

	////测试
	////-----将新坐标变换回原坐标
	//point oriPoint1 = oriCoordinate(attachPointInM1Cord, m1.xAxis, m1.yAxis, m1.zAxis, m1.extX, m1.extY, m1.extZ, m1.meshCenter);
	//point oriPoint2 = oriCoordinate(attachPointInM2Cord, m2.xAxis, m2.yAxis, m2.zAxis, m2.extX, m2.extY, m2.extZ, m2.meshCenter);
	//if (oriPoint1 == oriPoint2){}
}



void MeshRelation::applyMatrix(point &a, float *M){
	float x = a[0];
	float y = a[1];
	float z = a[2];
	a[0] = M[0] * x + M[4] * y + M[8] * z + M[12];
	a[1] = M[1] * x + M[5] * y + M[9] * z + M[13];
	a[2] = M[2] * x + M[6] * y + M[10] * z + M[14];
}

QMatrix4x4  MeshRelation::coordTranslateMatrix(Vec<3, float>xAxis, Vec<3, float>yAxis, Vec<3, float>zAxis){
	QMatrix4x4 transformOri2C;
	for (int i = 0; i < 16; i++)
		transformOri2C.data()[i] = 0;
	for (int i = 0; i < 3; i++){
		transformOri2C.data()[i] = xAxis[i];
		transformOri2C.data()[i + 4] = yAxis[i];
		transformOri2C.data()[i + 8] = zAxis[i];
	}
	return transformOri2C;
}

//QMatrix4x4  MeshRelation::coordTranslateMatrix(Vec<3, float>oriCoordx, Vec<3, float>oriCoordy, Vec<3, float>oriCoordz,
//	Vec<3, float>newCoordx, Vec<3, float>newCoordy, Vec<3, float>newCoordz)
//{
//	Eigen::Vector3d xAxis1, xAxis2;
//	Eigen::Vector3d yAxis1, yAxis2;
//	xAxis1(0) = oriCoordx[0];
//	xAxis1(1) = oriCoordx[1];
//	xAxis1(2) = oriCoordx[2];
//
//	xAxis2(0) = newCoordx[0];
//	xAxis2(1) = newCoordx[1];
//	xAxis2(2) = newCoordx[2];
//
//	yAxis1(0) = oriCoordy[0];
//	yAxis1(1) = oriCoordy[1];
//	yAxis1(2) = oriCoordy[2];
//
//	
//	//先使得x轴同向
//	Eigen::Matrix3d M;
//	computeRotateMartix(xAxis1, xAxis2, M);
//
//	QMatrix4x4 martix1;
//	for (int i = 0; i < 16; i++){
//		martix1.data()[i] = 0;
//	}
//
//	for (int i = 0; i < 3; i++)
//		for (int j = 0; j < 3; j++){
//			int idx = i * 4 + j;
//			martix1.data()[idx] = M(i, j);
//		}
//
//	//更新y轴方向
//	applyMatrix(newCoordy, martix1.data());
//	yAxis2(0) = newCoordy[0];
//	yAxis2(1) = newCoordy[1];
//	yAxis2(2) = newCoordy[2];
//
//	//再使得y轴同向
//	double angle = getRotateAngle(yAxis1(0), yAxis1(1), yAxis1(2),
//		yAxis2(0), yAxis2(1), yAxis2(2),
//		xAxis2(0), xAxis2(1), xAxis2(2));
//
//	QMatrix4x4 martix2;
//	martix2.setToIdentity();
//	martix2.rotate(angle, xAxis2(0), xAxis2(1), xAxis2(2));
//
//	QMatrix4x4 Transform;
//	Transform = martix1 * martix2;
//
//	//标准化矩阵的每个分量的长度
//	Eigen::Vector3d normXAxis,normYAxis,normZAxis;
//	normXAxis(0) = Transform.data()[0];
//	normXAxis(1) = Transform.data()[1];
//	normXAxis(2) = Transform.data()[2];
//	normXAxis.normalize();
//	normYAxis(0) = Transform.data()[4];
//	normYAxis(1) = Transform.data()[5];
//	normYAxis(2) = Transform.data()[6];
//	normYAxis.normalize();
//	normZAxis(0) = Transform.data()[8];
//	normZAxis(1) = Transform.data()[9];
//	normZAxis(2) = Transform.data()[10];
//	normZAxis.normalize();
//
//	Transform.data()[0] = normXAxis(0);
//	Transform.data()[4] = normXAxis(1);
//	Transform.data()[8] = normXAxis(2);
//	Transform.data()[1] = normYAxis(0);
//	Transform.data()[5] = normYAxis(1);
//	Transform.data()[9] = normYAxis(2);
//	Transform.data()[2] = normZAxis(0) ;
//	Transform.data()[6] = normZAxis(1);
//	Transform.data()[10] = normZAxis(2);
//
//	return Transform;
//}

point MeshRelation::newCoordinate(point oriCoordPoint, Vec<3, float> xAxis, Vec<3, float> yAxis, Vec<3, float> zAxis, float xExt, float yExt, float zExt, point newCenter){
	Vec<3, float> xLAxis = xAxis * xExt;
	Vec<3, float> yLAxis = yAxis * yExt;
	Vec<3, float> zLAxis = zAxis * zExt;
	float newXCoord = ((oriCoordPoint - newCenter) DOT xLAxis) / (len(xLAxis) * xExt);
	float newYCoord = ((oriCoordPoint - newCenter) DOT yLAxis) / (len(yLAxis) * yExt);
	float newZCoord = ((oriCoordPoint - newCenter) DOT zLAxis) / (len(zLAxis) * zExt);
	return point(newXCoord, newYCoord, newZCoord);
}

point MeshRelation::oriCoordinate(point newCoordPoint, Vec<3, float> xAxis, Vec<3, float> yAxis, Vec<3, float> zAxis, float xExt, float yExt, float zExt, point newCenter){
	//求局部坐标系CORD1的变换矩阵
	QMatrix4x4 transformOri2C = coordTranslateMatrix(xAxis*xExt, yAxis*yExt, zAxis*zExt);

	//将局部坐标乘上变换矩阵，还原为标准坐标系的向量
	applyMatrix(newCoordPoint, transformOri2C.data());

	//将向量平移到新坐标系原点，得到原坐标
	point oriPoint(newCoordPoint[0] + newCenter[0], newCoordPoint[1] + newCenter[1], newCoordPoint[2] + newCenter[2]);

	return oriPoint;
}

void MeshRelation::computeRotateMartix(Eigen::Vector3d &sourceV, Eigen::Vector3d &destV, Eigen::Matrix3d &M){
	const double epsilon = 1.0e-6;
	sourceV.normalize();
	destV.normalize();
	Eigen::Vector3d v = sourceV.cross(destV);
	float s = v.norm();
	float c = sourceV.dot(destV);
	Eigen::Matrix3d v2, I;
	Eigen::Matrix3d v1;
	v1(0, 0) = 0;
	v1(0, 1) = -v(2);
	v1(0, 2) = v(1);
	v1(1, 0) = v(2);
	v1(1, 1) = 0;
	v1(1, 2) = -v(0);
	v1(2, 0) = -v(1);
	v1(2, 1) = v(0);
	v1(2, 2) = 0;
	v2 = v1 * v1;
	I.setIdentity();
	if (fabs(s) <= epsilon)
		M = I;
	else
		M = I + v1 + v2*(1 - c) / (s*s);
}


double MeshRelation::getRotateAngle(double x1, double y1, double z1, double x2, double y2, double z2, double xAxis, double yAxis, double zAxis)
{
	const double epsilon = 1.0e-6;
	const double nyPI = acos(-1.0);
	double dist, dot, degree, angle;
	// normalize
	dist = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
	x1 /= dist;
	y1 /= dist;
	z1 /= dist;
	dist = sqrt(x2 * x2 + y2 * y2 + z2 * z2);
	x2 /= dist;
	y2 /= dist;
	z2 /= dist;
	// dot product
	dot = x1 * x2 + y1 * y2 + z1 * z2;
	if (fabs(dot - 1.0) <= epsilon)
		angle = 0.0;
	else if (fabs(dot + 1.0) <= epsilon)
		angle = nyPI;
	else {
		double cross;

		angle = acos(dot);
		//cross product
		cross = xAxis * (y1 * z2 - z1 * y2) - yAxis * (x1 * z2 - z1 * x2) + zAxis * (x1 * y2 - y1 * x2);
		// vector p2 is clockwise from vector p1 
		// with respect to the origin (0.0)
		if (cross < 0) {
			angle = 2 * nyPI - angle;
		}
	}
	degree = angle *  180.0 / nyPI;
	return degree;
}