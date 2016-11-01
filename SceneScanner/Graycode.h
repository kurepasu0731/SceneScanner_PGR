#ifndef GRAYCODE_H
#define GRAYCODE_H

#pragma once

// GrayCode��p�����􉽕␳

// �O���C�R�[�h�́C�c�C���ʂɍ쐬���C��ō���
// �p�^�[���摜�̓r�b�g���Z��p���č쐬�i������char�^�͎g��Ȃ��j
// �p�^�[���摜��1������������ŕۑ�
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>  // ������X�g���[��
#include <direct.h> // create dir
#include "Header.h"
#include "PGROpenCV.h"


extern TPGROpenCV pgrOpenCV;

/** 
@brief GrayCode��p�����􉽕␳<br>
�Ή��t�����Ȃ�������f�ɂ͋ߖT�̉�f���R�s�[���ĕ��<br>
�ŏI�I�ȃv���W�F�N�^�ƃJ������Ή��t�����z���c->CamPro�Ɋi�[�����B
*/
class GRAYCODE{
public:
	// �v���W�F�N�^�𑜓x
	static const int PRJ_WIDTH = PROJECTOR_WIDTH;
	static const int PRJ_HEIGHT = PROJECTOR_HEIGHT;
	static const int CMR_WIDTH = CAMERA_WIDTH;
	static const int CMR_HEIGHT = CAMERA_HEIGHT;
	static const int PRJ_X = DISPLAY_WIDTH;
	static const int PRJ_Y = DISPLAY_HEIGHT;

	// �O���C�R�[�h�쐬�ɕK�v�ȍ\����
	typedef struct _Graycode {
		int graycode[PRJ_HEIGHT][PRJ_WIDTH];  // �O���C�R�[�h�i�v���W�F�N�^�𑜓x[����][��]�j
		unsigned int h_bit, w_bit;     // �����C���̕K�v�r�b�g��
		unsigned int all_bit;          // ���v�r�b�g���ih_bit + w_bit�j
	} Graycode;

	// �v���W�F�N�^ - �J�����Ή��ɕK�v�ȍ\����
	typedef struct _correspondence {
		int graycode[CMR_HEIGHT][CMR_WIDTH];  // 2�l���R�[�h�𕜌��������̂��J������f�Ɋi�[
		cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH];  // �v���W�F�N�^��f�ɑ΂���J�����Ή���f
		cv::Point ProCam[CMR_HEIGHT][CMR_WIDTH];  // �J������f�ɑ΂���v���W�F�N�^�Ή���f
		std::map<int, cv::Point> *code_map;
		Graycode g;
	} correspondence;

	correspondence *c;

	GRAYCODE();
	~GRAYCODE();
	// �p�^�[���R�[�h���e & �B�e
	void code_projection();
	// 2�l��
	void make_thresh();
	// ������
	void makeCorrespondence();

	//// �摜�ό`�E����
	//// �J�����B�e�̈悩��v���W�F�N�^���e�̈��؂�o��
	void transport_camera_projector(cv::Mat &src, cv::Mat &dst);
	//// ���͉摜���J�����B�e�̈�ɕό`
	void transport_projector_camera(cv::Mat &src, cv::Mat &dst);

	// �J�������W�ɑ΂���v���W�F�N�^�̑Ή��_��Ԃ�
	void getCorrespondProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint);
	// �J�������W�ɑ΂���v���W�F�N�^�̑Ή��_��Ԃ�(�����x��)
	void getCorrespondSubPixelProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, int size = 20);

	// �Ή��̂Ƃꂽ�_��S�ĕԂ�
	void getCorrespondAllPoints(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<int> &flag);

	void getCorrespondAllPoints_ProCam(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<int> &flag);


private:
	// �E�B���h�E�l�[��
	char* GC;
	char* MP;
	float SHUTTER;	// �V���b�^�[���x
	double delay;

	Graycode *g;

	/// �O���C�R�[�h�̍쐬�֘A
	// �J�����̏�����
	void initCamera();
	// �O���C�R�[�h�쐬
	void initGraycode();
	// �p�^�[���R�[�h�摜�쐬
	void makeGraycodeImage();
	// �f�B���N�g���̍쐬
	void createDirs();
	/// ��l���֘A
	// �J�����B�e�摜��ǂݍ��ފ֐�
	void loadCam(cv::Mat &mat, int div_bin, bool flag, bool pattern);
	// �ŏI�I�Ɏg�p����}�X�N�𐶐�����֐�
	void makeMask(cv::Mat &mask);
	// �O���C�R�[�h�̉摜�𗘗p���ă}�X�N�𐶐�����֐�
	// �|�W�ƃl�K�̍����������MASK_THRESH�ȏ�̋P�x�̃s�N�Z���𔒂ɂ���
	void makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue = 25);
	// 2�l�������֐� 
	void thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value );

	/// ���̑�
	// �v���W�F�N�^ - �J�����\���̏�����
	void initCorrespondence();
	// 2�l���R�[�h����
	void code_restore();
	//// ��f��Ԏ�@
	//// �E���̉�f�������Ă���
	//cv::Point getInterpolatedPoint1(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH]);
	// �אڂ����f���玝���Ă���
	cv::Point getInterpolatedPoint2(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH]);
	// ��f���
	void interpolation();

	//// �S��ʕ\��
	//void displayFullScreen(cv::Mat &image);
};


#endif