#ifndef MESHRELATION_H
#define MESHRELATION_H
#pragma once
#include "MyMesh.h"
#include <Eigen\Core>
#include <Eigen/Geometry>  
#include <qmatrix4x4.h>
#include "Vec.h"

typedef class MeshRelation{

public:
	MeshRelation();
	~MeshRelation();

	MeshRelation(MyMesh m1,MyMesh m2);// m1 rotate to m2

	void computeRotateMartix(Eigen::Vector3d &sourceV, Eigen::Vector3d &destV, Eigen::Matrix3d &M);

	double getRotateAngle(double x1, double y1, double z1, double x2, double y2, double z2, double xAxis, double yAxis, double zAxis);

	//����׼����任Ϊ�ֲ�����
	trimesh::point newCoordinate(trimesh::point oriCoordPoint, trimesh::Vec<3, float> xAxis, 
		trimesh::Vec<3, float> yAxis, trimesh::Vec<3, float> zAxis, float xExt, float yExt, float zExt, trimesh::point newCenter);
	
	//���ֲ�����任Ϊ��׼����
	trimesh::point oriCoordinate(trimesh::point newCoordPoint, trimesh::Vec<3, float> xAxis, 
		trimesh::Vec<3, float> yAxis, trimesh::Vec<3, float> zAxis, float xExt, float yExt, float zExt, trimesh::point newCenter);
	
	QMatrix4x4 coordTranslateMatrix(trimesh::Vec<3, float>newCoordx, trimesh::Vec<3, float>newCoordy, trimesh::Vec<3, float>newCoordz);// ���㽫��׼����ϵ�任��������ϵ�ľ���,ע�⣺������ϵ����Ϊ�г��ȵ�������

	void applyMatrix(trimesh::point &a, float *M);

	trimesh::point attachPointInM1Cord, attachPointInM2Cord;
	//QMatrix4x4 &Transform;
	//QMatrix4x4 coordTranslateMatrix(Vec<3, float>oriCoordx, Vec<3, float>oriCoordy, Vec<3, float>oriCoordz, Vec<3, float>newCoordx, Vec<3, float>newCoordy, Vec<3, float>newCoordz);

}MeshRelation;


#endif