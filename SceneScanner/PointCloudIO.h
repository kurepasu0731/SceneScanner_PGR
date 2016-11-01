#pragma once

#include "Header.h"

//---�ۑ��֌W---//

// XML�t�@�C���ǂݍ���
std::vector<cv::Point3f> loadXMLfile(const std::string &fileName)
{
	//�ǂݍ��ޓ_�Q
	std::vector<cv::Point3f> reconstructPoints;
	// xml�t�@�C���̓ǂݍ���
	cv::FileStorage cvfs(fileName, cv::FileStorage::READ);
	cvfs["points"] >> reconstructPoints;

	return reconstructPoints;
}


//PLY�`���ŕۑ�(�@���Ȃ�)
void savePLY(std::vector<cv::Point3f> points, const std::string &fileName)
{

	//�t�@�C���I�[�v��
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//�t�@�C���ɏ�������
	//�w�b�_�̐ݒ�
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n", points.size());

	//3�����_�Q
	//m�P�ʂŕۑ��ixml��mm�j
	for (int n = 0; n < points.size(); n++){
	   fprintf(fp, "%f %f %f \n", points[n].x, points[n].y, points[n].z);
	}
	//�t�@�C���N���[�Y
	fclose(fp);
}

//PLY�`���ŕۑ�(�@������)
void savePLY_with_normal(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, const std::string &fileName)
{
	//�t�@�C���I�[�v��
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//�t�@�C���ɏ�������
	//�w�b�_�̐ݒ�
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n", points.size());

	//3�����_�Q
	//m�P�ʂŕۑ��ixml��mm�j
	for (int n = 0; n < points.size(); n++){
		fprintf(fp, "%f %f %f %f %f %f \n", points[n].x, points[n].y, points[n].z, normals[n].x, normals[n].y, normals[n].z);

	}
	//�t�@�C���N���[�Y
	fclose(fp);
}

//PLY�`���ŕۑ�(�@������,mesh����)
void savePLY_with_normal_mesh(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, std::vector<cv::Point3i> meshes, const std::string &fileName)
{
	//�t�@�C���I�[�v��
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//�t�@�C���ɏ�������
	//�w�b�_�̐ݒ�
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face %d\nproperty list ushort int vertex_indices\nend_header\n", points.size(), meshes.size());

	//3�����_�Q
	//m�P�ʂŕۑ��ixml��mm�j
	for (int n = 0; n < points.size(); n++){
	   fprintf(fp, "%f %f %f %f %f %f \n", points[n].x, points[n].y, points[n].z, normals[n].x, normals[n].y, normals[n].z);
	}
	//�ʏ��L�q
	for(int n = 0; n < meshes.size(); n++)
	{
	   fprintf(fp, "3 %d %d %d\n", meshes[n].x, meshes[n].y, meshes[n].z);
	}
	//�t�@�C���N���[�Y
	fclose(fp);
}


//OBJ�`���ŕۑ�(�@������,mesh����)
void saveOBJ_with_normal_mesh(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, std::vector<cv::Point3i> meshes, const std::string &fileName)
{
	//�t�@�C���I�[�v��
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//�t�@�C���ɏ�������
	//�w�b�_�̐ݒ�
	fprintf(fp,"o dorascan");

	//3�����_�Q
	//m�P�ʂŕۑ��ixml��mm�j
	for (int n = 0; n < points.size(); n++){
	   fprintf(fp, "v %f %f %f\n", points[n].x, points[n].y, points[n].z);
	}
	//�@�����L�q
	for(int n = 0; n < normals.size(); n++){
		fprintf(fp, "vn %f %f %f\n", normals[n].x, normals[n].y, normals[n].z);
	}
	//�ʏ��L�q
	for(int n = 0; n < meshes.size(); n++)
	{
	   fprintf(fp, "3 %d %d %d\n", meshes[n].x, meshes[n].y, meshes[n].z);
	}
	//�t�@�C���N���[�Y
	fclose(fp);
}


