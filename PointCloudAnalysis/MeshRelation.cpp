#include "MeshRelation.h"

using namespace trimesh;

MeshRelation::MeshRelation(){
	//Transform.setToIdentity();
}

MeshRelation::~MeshRelation(){
	//Transform.setToIdentity();
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