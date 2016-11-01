#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2\opencv.hpp>
#include <stdlib.h>

class Calibration
{
public:
	Calibration(int _checkerRow, int _checkerCol, float _checkerSize)
		: checkerPattern (cv::Size(_checkerRow, _checkerCol))
		, checkerSize (_checkerSize)
		, calib_flag (false)
	{
		// ���E���W�ɂ�����`�F�b�J�[�p�^�[���̌�_���W������
		for( int i = 0; i < checkerPattern.area(); ++i ) {
			worldPoint.push_back( cv::Point3f(	static_cast<float>( i % checkerPattern.width * checkerSize ),
													static_cast<float>( i / checkerPattern.width * checkerSize ), 0.0 ) );
		}
	};

	~Calibration(){};


	// �摜����`�F�b�J�[�p�^�[���̌�_���擾
	bool getCheckerCorners(std::vector<cv::Point2f> &imagePoint, const cv::Mat &image, cv::Mat &draw_image);		// �摜����`�F�b�J�[�p�^�[���̌�_���擾

	// �ē��e�덷�̌v�Z
	void calcReprojectionError(const std::vector<std::vector<cv::Point3f>> &worldPoints, const std::vector<std::vector<cv::Point2f>> &cameraPoints, const std::vector<std::vector<cv::Point2f>> &projectorPoints,double &cam_error, double &proj_error);

	// �v���W�F�N�^�ƃJ�����̃L�����u���[�V����
	void proCamCalibration(const std::vector<std::vector<cv::Point3f>> &worldPoints, const std::vector<std::vector<cv::Point2f>> &cameraPoints, const std::vector<std::vector<cv::Point2f>> &projectorPoints,
							const cv::Size &camSize, const cv::Size &projSize);

	// �L�����u���[�V�������ʂ̓ǂݍ���
	void loadCalibParam(const std::string &fileName);

	// �������e�ϊ��s��̎擾(�J����)
	cv::Mat getCamPerspectiveMat();
	// �������e�ϊ��s��̎擾(�v���W�F�N�^)
	cv::Mat getProjPerspectiveMat();

	// �J�����ʒu�����[���h���W�Ƃ����ۂ̑Ώە��̂̈ʒu�̎擾
	void getCameraWorldPoint(std::vector<cv::Point3f> &camWorldPoint, const std::vector<cv::Point2f> &imagePoint);

	// 3��������
	void reconstruction(std::vector<cv::Point3f> &reconstructPoint, const std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, const std::vector<int> &flag);

	//�����_�̕���������
	void smoothReconstructPoints(std::vector<cv::Point3f> &reconstructPoint, std::vector<cv::Point3f> &smoothed_reconstructPoint, int size); 

	//�J�����摜���W�Ɛ[�x�l����A�J�������S��3�����_�ɂ���
	cv::Point3f Calibration::getWorldpoint(int u, int v, float Z);

	//���f�B�A���t�B���^
	float medianfilter(int cx, int cy, int size, std::vector<cv::Point3f> reconstructPoint);
	//���σt�B���^
	float movingAveragefilter(int cx, int cy, int size, std::vector<cv::Point3f> reconstructPoint);

	// 3�����_�Q�̕`��
	//void pointCloudRender(const std::vector<cv::Point3f> &reconstructPoint, const std::vector<cv::Point2f> &imagePoint, const cv::Mat &image, std::string &windowName, const cv::Mat& R, const cv::Mat& t);
	void pointCloudRender(const std::vector<cv::Point3f> &reconstructPoint, const cv::Mat &image, std::string &windowName, const cv::Mat& R, const cv::Mat& t);


	/***** �����o�ϐ� *****/
	cv::Size checkerPattern;		// �`�F�b�J�[�p�^�[���̌�_�̐�
	float checkerSize;				// �`�F�b�J�[�p�^�[���̃}�X�ڂ̃T�C�Y(mm)

	std::vector<cv::Point3f> worldPoint;		// �`�F�b�J�[��_���W�ƑΉ����鐢�E���W�̒l���i�[����s��

	// �J����
	cv::Mat cam_K;					// �����p�����[�^�s��
	cv::Mat cam_dist;				// �����Y�c��
	std::vector<cv::Mat> cam_R;		// ��]�x�N�g��
	std::vector<cv::Mat> cam_T;		// ���s�ړ��x�N�g��

	// �v���W�F�N�^
	cv::Mat proj_K;					// �����p�����[�^�s��
	cv::Mat proj_dist;				// �����Y�c��
	std::vector<cv::Mat> proj_R;	// ��]�x�N�g��
	std::vector<cv::Mat> proj_T;	// ���s�ړ��x�N�g��

	// �X�e���I�p�����[�^
	cv::Mat R;						// �J����-�v���W�F�N�^�Ԃ̉�]�s��
	cv::Mat T;						// �J����-�v���W�F�N�^�Ԃ̕��i�x�N�g��
	cv::Mat E;						// ��{�s��
	cv::Mat F;						// ��b�s��

	// �t���O
	bool calib_flag;
};


#endif