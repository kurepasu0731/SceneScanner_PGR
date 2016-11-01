#ifndef WEBCAMERA_H
#define WEBCAMERA_H

#include <opencv2\opencv.hpp>

class WebCamera
{
public:
	//�𑜓x
	int width;
	int height;
	//�ۑ��֘A
	int capture_num;
	std::string save_dir;

	cv::VideoCapture vc;
	cv::Mat frame;
	std::string winName;

	// �J����
	cv::Mat cam_K;					// �����p�����[�^�s��
	cv::Mat cam_dist;				// �����Y�c��
	cv::Mat cam_R;					// ��]�x�N�g��
	cv::Mat cam_T;					// ���s�ړ��x�N�g��
	bool calib_flag;

	WebCamera(){};

	WebCamera(int _width, int _height, std::string _winName)
	{
		width = _width;
		height = _height;
		winName = _winName;

		cam_K = cv::Mat::eye(3, 3, CV_64F);
		cam_dist = cv::Mat::zeros(1, 5, CV_64F);
		cam_R = cv::Mat::eye(3, 3, CV_64F);
		cam_T = cv::Mat::zeros(3, 1, CV_64F);

		vc = cv::VideoCapture(0);
		save_dir = "./capture/";
		capture_num = 0;
		calib_flag = false;

		cv::Size captureSize(width, height);
		vc.set(CV_CAP_PROP_FRAME_WIDTH, captureSize.width);
		vc.set(CV_CAP_PROP_FRAME_HEIGHT, captureSize.height);
		cv::namedWindow(winName);
	};


	void idle(){
		vc >> frame;
		cv::imshow(winName, frame);
	}

	void capture()
	{
		std::string filename = save_dir + "cap"+ std::to_string(capture_num) + ".jpg";
		cv::imwrite(filename, frame);
		capture_num++;
	}

	cv::Mat getFrame()
	{
		vc >> frame;
		return frame;
	}

// �L�����u���[�V�������ʂ̓ǂݍ���(�����p�����[�^�̂�)
void loadCalibParam(const std::string &fileName)
{
	// xml�t�@�C���̓ǂݍ���
	cv::FileStorage cvfs(fileName, cv::FileStorage::READ);

	cvfs["cam_K"] >> cam_K;
	cvfs["cam_dist"] >> cam_dist;

	calib_flag = true;
}

	~WebCamera(){};
};
#endif