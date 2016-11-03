#include "Graycode.h"

GRAYCODE::GRAYCODE()
{
	GC = "Graycode";
	MP = "Measure";
	delay = 150; //���������������āAframe�������ς��ǂݍ���(�V�����̂�ǂ�)�ƁA�O���C�R�[�h���e�������Ȃ�
	g = new Graycode();
	c = new correspondence();
	c->code_map = new std::map<int, cv::Point>();
	// �\���̂̏�����
	c->g.h_bit = (int)ceil( log(PRJ_HEIGHT+1) / log(2) );
	c->g.w_bit = (int)ceil( log(PRJ_WIDTH+1) / log(2) );
	c->g.all_bit = c->g.h_bit + c->g.w_bit;
	createDirs();
}

GRAYCODE::~GRAYCODE()
{
}

void GRAYCODE::createDirs()
{
	_mkdir("./GrayCodeImage");
	// �O���C�R�[�h�B�e�摜
	_mkdir("./GrayCodeImage/CaptureImage");
	// �O���C�R�[�h���摜
	_mkdir("./GrayCodeImage/ProjectionGrayCode");
	// �O���C�R�[�h�B�e�摜�̓�l�������摜
	_mkdir("./GrayCodeImage/ThresholdImage");
}

/***************************
** �O���C�R�[�h�̍쐬�֘A **
****************************/

// �r�b�g���̌v�Z�ƃO���C�R�[�h�̍쐬
void GRAYCODE::initGraycode()
{
	int bin_code_h[PRJ_HEIGHT];  // 2�i�R�[�h�i�c�j
	int bin_code_w[PRJ_WIDTH];   // 2�i�R�[�h�i���j
	int graycode_h[PRJ_HEIGHT];  // �O���C�R�[�h�i�c�j
	int graycode_w[PRJ_WIDTH];   // �O���C�R�[�h�i���j
	//int *graycode_h =  new int[c->g.h_bit];  // �O���C�R�[�h�i�c�j
	//int *graycode_w =  new int[c->g.w_bit];  // �O���C�R�[�h�i���j

	/***** 2�i�R�[�h�쐬 *****/
	// �s�ɂ���
	for( int y = 0; y < PRJ_HEIGHT; y++ )
		bin_code_h[y] = y + 1;
	// ��ɂ���
	for( int x = 0; x < PRJ_WIDTH; x++ )
		bin_code_w[x] = x + 1;

	/***** �O���C�R�[�h�쐬 *****/
	// �s�ɂ���
	for( int y = 0; y < PRJ_HEIGHT; y++ )
		graycode_h[y] = bin_code_h[y] ^ ( bin_code_h[y] >> 1 );
	// ��ɂ���
	for( int x = 0; x < PRJ_WIDTH; x++ )
		graycode_w[x] = bin_code_w[x] ^ ( bin_code_w[x] >> 1 );
	// �s������킹��i�s + ��j
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ )
			c->g.graycode[y][x] = ( graycode_h[y] << c->g.w_bit) | graycode_w[x];
	}
}

// �p�^�[���R�[�h�摜�쐬�i��x���΃v���W�F�N�^�̉𑜓x���ς��Ȃ������蒼���K�v�͂Ȃ��j
void GRAYCODE::makeGraycodeImage()
{
	std::cout << "���e�p�O���C�R�[�h�쐬��" << std::endl;
	//initGraycode();
	cv::Mat posi_img ( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	cv::Mat nega_img ( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	int bit = c->g.all_bit-1;
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit];  // �����t���o��
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];  // �����t���o��

	// �|�W�p�^�[���R�[�h�摜�쐬
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PRJ_HEIGHT; y++ ) {
			for( int x = 0; x < PRJ_WIDTH; x++ ) {
				if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 0 ) {  // �ŏ�ʃr�b�g���珇�ɒ��o���C���̃r�b�g��0��������
					posi_img.at<cv::Vec3b>( y, x )[0] = 0;  // B
					posi_img.at<cv::Vec3b>( y, x )[1] = 0;  // G
					posi_img.at<cv::Vec3b>( y, x )[2] = 0;  // R
				}else if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 1 ) {
					posi_img.at<cv::Vec3b>( y, x )[0] = 255;  // B
					posi_img.at<cv::Vec3b>( y, x )[1] = 255;  // G
					posi_img.at<cv::Vec3b>( y, x )[2] = 255;  // R
				}
			}
		}
		// �A�ԂŃt�@�C������ۑ��i������X�g���[���j
		Filename_posi[z] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_posi[z].str(), posi_img);
		Filename_posi[z] << std::endl;
	}

	// �l�K�p�^�[���R�[�h�摜�쐬
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PRJ_HEIGHT; y++ ) {
			for( int x = 0; x < PRJ_WIDTH; x++ ) {
				if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 1 ) {
					nega_img.at<cv::Vec3b>( y, x )[0] = 0;  // B
					nega_img.at<cv::Vec3b>( y, x )[1] = 0;  // G
					nega_img.at<cv::Vec3b>( y, x )[2] = 0;  // R
				}else if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 0 ) {
					nega_img.at<cv::Vec3b>( y, x )[0] = 255;  // B
					nega_img.at<cv::Vec3b>( y, x )[1] = 255;  // G
					nega_img.at<cv::Vec3b>( y, x )[2] = 255;  // R
				}
			}
		}
		// �A�ԂŃt�@�C������ۑ��i������X�g���[���j
		Filename_nega[z] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_nega[z].str(), nega_img);
		Filename_nega[z] << std::endl;
	}

	delete[] Filename_posi;
	delete[] Filename_nega;
}

// �p�^�[���R�[�h���e & �B�e
void GRAYCODE::code_projection()
{
	// �萔
	typedef enum flag{
		POSI = true,
		NEGA = false,
		VERTICAL = true,
		HORIZONTAL = false,
	} flag;

	Graycode *g = new Graycode();
	//TPGROpenCV	pgrOpenCV;

	//������&�J�����N��
	initGraycode();
	//pgrOpenCV.init( FlyCapture2::PIXEL_FORMAT_BGR );
	//pgrOpenCV.setShutterSpeed(pgrOpenCV.getShutter_h());
	pgrOpenCV.start();

	cv::Mat *posi_img = new cv::Mat[c->g.all_bit];  // �|�W�p�^�[���p
	cv::Mat *nega_img = new cv::Mat[c->g.all_bit];  // �l�K�p�^�[���p

	// �����t���o�́i�O���C�R�[�h�ǂݍ��ݗp�j
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];
	// �����t���o�́i�B�e�摜�������ݗp�j
	std::stringstream *Filename_posi_cam = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega_cam = new std::stringstream[c->g.all_bit];

	// �A�ԂŃt�@�C������ǂݍ��ށi������X�g���[���j
	std::cout << "���e�p�O���C�R�[�h�摜�ǂݍ��ݒ�" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		Filename_posi[i] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		Filename_nega[i] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		// �ǂݍ���
		posi_img[i] = cv::imread(Filename_posi[i].str(), 1);
		nega_img[i] = cv::imread(Filename_nega[i].str(), 1);
		Filename_posi[i] << std::endl;
		Filename_nega[i] << std::endl;
		// �ǂݍ��ޖ���������Ȃ�������O���C�R�[�h�摜����蒼��
		if(posi_img[i].empty() || nega_img[i].empty()){
			std::cout << "ERROR(1)�F���e�p�̃O���C�R�[�h�摜���s�����Ă��܂��B" << std::endl;
			std::cout << "ERROR(2)�F�O���C�R�[�h�摜���쐬���܂��B" << std::endl;
			makeGraycodeImage();
			code_projection();
			return;
		}
	}

	/***** �O���C�R�[�h���e & �B�e *****/
	/*  �S��ʕ\���p�E�B���h�E�̍쐬  */
	cv::namedWindow(GC, 0);
	Projection::MySetFullScrean(DISPLAY_NUMBER, GC);

	// �|�W�p�^�[�����e & �B�e
	//start capturing

	std::cout << "�|�W�p�^�[���B�e��" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// ���e
		cv::imshow(GC, posi_img[i]);
		// �x���҂�
		cv::waitKey(2.0*delay);
		// �B�e
		pgrOpenCV.queryFrame();
		// �B�e�摜��Mat�^�Ɋi�[
		cv::Mat cap = pgrOpenCV.getVideo();
		// �B�e�̗l�q���`�F�b�N
		pgrOpenCV.showCapImg(cap);

		// �|�W�p�^�[���B�e���ʂ�ۑ�
		// ����
		if(i < c->g.h_bit)
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << POSI << ".bmp"; 
		// �c��
		else
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << POSI << ".bmp"; 
		// �ۑ�
		cv::imwrite(Filename_posi_cam[i].str(), pgrOpenCV.getVideo());
		Filename_posi_cam[i] << std::endl;
	}

	// �l�K�p�^�[�����e & �B�e
	std::cout << "�l�K�p�^�[���B�e��" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// ���e
		cv::imshow(GC, nega_img[i]);
		// �x���҂�
		cv::waitKey(2*delay);
		// �B�e
		pgrOpenCV.queryFrame();
		// �B�e�摜��Mat�^�Ɋi�[
		cv::Mat cap = pgrOpenCV.getVideo();
		// �B�e�̗l�q���`�F�b�N
		pgrOpenCV.showCapImg(cap);
		// �|�W�p�^�[���B�e���ʂ�ێ�
		// ����
		if(i < c->g.h_bit)
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << NEGA << ".bmp"; 
		// �c��
		else
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << NEGA << ".bmp"; 
		//Filename_nega_cam[i] << "./output/Camera_nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		cv::imwrite(Filename_nega_cam[i].str(), pgrOpenCV.getVideo());
		Filename_nega_cam[i] << std::endl;
	}
	/***** ���e & �B�e�I�� *****/


	cv::Mat src = cv::Mat(PROJECTOR_WIDTH, PROJECTOR_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::imshow(GC, src);
	cv::waitKey(2*delay);
	pgrOpenCV.queryFrame();
	cv::imwrite("./cap.jpg", pgrOpenCV.getVideo());

	// �J�����I������
	pgrOpenCV.stop();

	/**** �I�� *****/

	// �������̊J��
	delete[] posi_img;
	delete[] nega_img;
	delete[] Filename_posi;
	delete[] Filename_nega;
	delete[] Filename_posi_cam;
	delete[] Filename_nega_cam;
}


/***************
** ��l���֘A **
****************/

// �J�����B�e�摜��ǂݍ��ފ֐�
void GRAYCODE::loadCam(cv::Mat &mat, int div_bin, bool vh, bool pn)
{
	char buf[256];
	sprintf_s(buf, "./GrayCodeImage/CaptureImage/CameraImg%d_%02d_%d.bmp", vh, div_bin, pn);
	mat = cv::imread(buf, 0);
}

// �}�X�N���쐬����C���^�t�F�[�X
void GRAYCODE::makeMask(cv::Mat &mask)
{
	cv::Mat posi_img;
	cv::Mat nega_img;

	// �}�X�N�摜����
	cv::Mat mask_vert, mask_hor;
	static int useImageNumber = 6;
	// y�����̃O���C�R�[�h�摜�ǂݍ���
	loadCam(posi_img, useImageNumber, 0, 1);
	loadCam(nega_img, useImageNumber, 0, 0);

	// ���̃}�X�N�摜Y����
	makeMaskFromCam(posi_img, nega_img, mask_vert);

	// x�����̃O���C�R�[�h�摜�ǂݍ���
	loadCam(posi_img, useImageNumber, 1, 1);
	loadCam(nega_img, useImageNumber, 1, 0);

	// ���̃}�X�N�摜X����
	makeMaskFromCam(posi_img, nega_img, mask_hor);

	// X��Y��OR�����
	// �}�X�N�O�͂ǂ�������Ȃ̂ō�
	// �}�X�N���́i���_�I�ɂ́j�K����������ł�����������Ȃ̂ŁA���ɂȂ�
	// ���ۂ͂��܉��m�C�Y���c���Ă��܂�
	cv::bitwise_or(mask_vert, mask_hor, mask);

	// �c�������܉��m�C�Y�������i���S�}�����S�}���œK�p�����t�ɂȂ�j
	dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
	erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

	cv::imwrite("./GrayCodeImage/mask.bmp", mask);
}

// �O���C�R�[�h�̉摜�𗘗p���ă}�X�N�𐶐�����֐�
// �|�W�ƃl�K�̍����������thresholdValue�ȏ�̋P�x�̃s�N�Z���𔒂ɂ���
void GRAYCODE::makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue)
{
	result = cv::Mat::zeros(cv::Size(CMR_WIDTH,CMR_HEIGHT), CV_8UC1);

	for(int j=0; j<CMR_HEIGHT; j++){
		for(int i=0; i<CMR_WIDTH; i++){
			int posi_i = posi.at<uchar>(j, i);
			int nega_i = nega.at<uchar>(j, i);

			if (abs(posi_i - nega_i) > thresholdValue){
				result.at<uchar>(j, i) = 255;
			}else{
				result.at<uchar>(j, i) = 0;
			}
		}
	}
}

// �B�e�摜��2�l��������C���^�t�F�[�X
void GRAYCODE::make_thresh()
{
	cv::Mat posi_img;
	cv::Mat nega_img;
	cv::Mat Geometric_thresh_img;  // 2�l�����ꂽ�摜
	cv::Mat mask;

	// �}�X�N�𐶐�
	makeMask(mask);

	int h_bit = (int)ceil( log(PRJ_HEIGHT+1) / log(2) );
	int w_bit = (int)ceil( log(PRJ_WIDTH+1) / log(2) );
	int all_bit = h_bit + w_bit;

	std::cout << "��l���J�n" << std::endl;
	// �A�ԂŃt�@�C������ǂݍ���
	for( int i = 0; i < h_bit; i++ ) {
		// �ǂݍ���
		char buf[256];
		// �|�W�p�^�[���ǂݍ���
		loadCam(posi_img, i+1, 0, 1);
		// �l�K�p�^�[���ǂݍ���
		loadCam(nega_img, i+1, 0, 0);

		// 2�l��
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// �}�X�N��K�p����2�l��
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::imwrite(buf, masked_img);

		std::cout << i << ", ";
	}
	for( int i = 0; i < w_bit; i++ ) {
		// �ǂݍ���
		char buf[256];
		// �|�W�p�^�[���ǂݍ���
		loadCam(posi_img, i+1, 1, 1);
		// �l�K�p�^�[���ǂݍ���
		loadCam(nega_img, i+1, 1, 0);

		// 2�l��
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// �}�X�N��K�p����2�l��
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i+h_bit);
		cv::imwrite(buf, masked_img);

		std::cout << i+h_bit << ", ";
	}
	std::cout << std::endl;
	std::cout << "��l���I��" << std::endl;
}

// ���ۂ�2�l������ 
void GRAYCODE::thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value )
{
	thresh_img = cv::Mat(posi.rows, posi.cols, CV_8UC1);
	for( int y = 0; y < posi.rows; y++ ) {
		for(int x = 0; x < posi.cols; x++ ) {
			int posi_pixel = posi.at<uchar>( y, x );
			int nega_pixel = nega.at<uchar>( y, x );

			// thresh_value���傫�����ǂ����œ�l��
			if( posi_pixel - nega_pixel >= thresh_value )
				thresh_img.at<uchar>( y, x ) = 255;
			else
				thresh_img.at<uchar>( y, x ) = 0;
		}
	}
}

/***********************************
** �v���W�F�N�^�ƃJ�����̑Ή��t�� **
************************************/

// 2�l���R�[�h����
void GRAYCODE::code_restore()
{
	// 2�l���R�[�h����
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		char buf[256];
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::Mat a = cv::imread(buf, 0);

		for( int y = 0; y < CMR_HEIGHT; y++ ) {
			for( int x = 0; x < CMR_WIDTH; x++ ) {
				if( a.at<uchar>( y, x ) == 255)
					c->graycode[y][x] = ( 1 << (c->g.all_bit-i-1) ) | c->graycode[y][x]; 
			}
		}
	}

	c->code_map->clear();

	// �A�z�z��ŃO���C�R�[�h�̒l�̏ꏊ�ɍ��W���i�[
	for( int y = 0; y < CMR_HEIGHT; y++ ) {
		for( int x = 0; x < CMR_WIDTH; x++ ) {
			int a = c->graycode[y][x];
			if( a != 0 )
				(*c->code_map)[a] = cv::Point(x, y);

			// ������
			c->ProCam[y][x] = cv::Point(-1, -1);
		}
	}

	// 0�Ԗڂ͎g��Ȃ�
	(*c->code_map)[0] = cv::Point(-1, -1);

	// �v���W�F�N�^�ƃJ�����̑Ή��t��
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			// �O���C�R�[�h�擾
			int a = c->g.graycode[y][x];
			// map���ɑ��݂��Ȃ��R�[�h�i�J�����ŎB�e����肭�����Ȃ����������j�̏ꏊ�ɂ̓G���[�l-1���i�[
			if ( (*c->code_map).find(a) == (*c->code_map).end() ) {
				c->CamPro[y][x] = cv::Point(-1, -1);
			}
			// ���݂���ꍇ�́A�Ή�����O���C�R�[�h�̍��W���i�[
			else {
				c->CamPro[y][x] = (*c->code_map)[a];
				c->ProCam[(*c->code_map)[a].y][(*c->code_map)[a].x] = cv::Point(x, y);
			}
		}
	}
}

// ��f��Ԏ�@
// �E���̉�f�������Ă���
//cv::Point GRAYCODE::getInterpolatedPoint1(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH])
//{
//	for (int i = x + 1; i < PRJ_WIDTH; i++) {
//		if (CamPro[y][i].x != -1) {
//			return CamPro[y][i];
//		}
//	}
//
//	return cv::Point(-1, -1);
//}

// �אڂ����f���玝���Ă���
cv::Point GRAYCODE::getInterpolatedPoint2(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH])
{
	const int MAX_RADIUS = 100;

	for (int radius = 1; radius <= MAX_RADIUS; radius++){

		for (int j = -radius; j <= radius; j++) {
			for (int i = -radius; i <= radius; i++) {
				int yj = j + y;
				int xi = i + x;

				if (0 <= yj && yj < PRJ_HEIGHT && 0 <= xi && xi < PRJ_WIDTH) {
					if ((yj > y) || (yj == y && xi > x)) {
						if (c->CamPro[yj][xi].x != -1) {
							return c->CamPro[yj][xi];
						}
					}
				}
			}
		}

	}

	return cv::Point(-1, -1);
}

// ��f���
void GRAYCODE::interpolation()
{
	for (int y = 0; y < PRJ_HEIGHT; y++) {
		for (int x = 0; x < PRJ_WIDTH; x++) {
			if (c->CamPro[y][x].x == -1) {
				c->CamPro[y][x] = getInterpolatedPoint2(y, x, c->CamPro);
			}
		}
	}
}

// �v���W�F�N�^ - �J�����\���̏�����
void GRAYCODE::initCorrespondence()
{
	initGraycode();

	for( int y = 0; y < CMR_HEIGHT; y++ ) {
		for( int x = 0; x < CMR_WIDTH; x++ ){
			c->graycode[y][x] = 0;
		}
	}
}

// �Ή��t�����s���C���^�[�t�F�[�X
void GRAYCODE::makeCorrespondence()
{
	initCorrespondence();
	code_restore();
	//	�⊮����on off
	//interpolation();
}


/***********************************
** ���̑��i�p�r�s���ȉߋ��̈╨�j **
************************************/

// �摜�ό`�E����
// �J�����B�e�̈悩��v���W�F�N�^���e�̈��؂�o��
void GRAYCODE::transport_camera_projector(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat src_resize;  // ���T�C�Y�����摜
	resize( src, src_resize, cv::Size(CMR_WIDTH, CMR_HEIGHT) );

	dst = cv::Mat( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );  // �􉽕␳���ꂽ�摜�i���e�摜�j

	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			cv::Point p = c->CamPro[y][x];
			if( p.x != -1 ) {
				if(src_resize.at<uchar>( p.y, p.x ) != 0){
				//if(src_resize.at<uchar>( p.y, 3*p.x ) != 0 && src_resize.at<uchar>( p.y, 3*p.x+1 ) != 0 && src_resize.at<uchar>( p.y, 3*p.x+2 ) != 0){
					//printf("x:%d, y:%d, p.x:%d, p.y:%d\n", x, y, p.x, p.y);
					//dst.at<uchar>( y, 3*x ) = src_resize.at<uchar>( p.y, 3*p.x );      // B
					//dst.at<uchar>( y, 3*x+1 ) = src_resize.at<uchar>( p.y, 3*p.x+1 );  // G
					//dst.at<uchar>( y, 3*x+2 ) = src_resize.at<uchar>( p.y, 3*p.x+2 );  // R
					//GRAY
					dst.at<uchar>( y, 3*x ) = src_resize.at<uchar>( p.y, p.x );   // B
					dst.at<uchar>( y, 3*x+1 ) = src_resize.at<uchar>( p.y, p.x);  // G
					dst.at<uchar>( y, 3*x+2 ) = src_resize.at<uchar>( p.y, p.x);  // R
				}
			}
		}
	}
}

// ���͉摜���J�����B�e�̈�ɕό`
void GRAYCODE::transport_projector_camera(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat src_resize;  // ���T�C�Y�����摜
	resize( src, src_resize, cv::Size(PRJ_WIDTH, PRJ_HEIGHT) );

	dst = cv::Mat( CMR_HEIGHT, CMR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );  // �􉽕␳���ꂽ�摜�i���e�摜�j

	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			cv::Point p = c->CamPro[y][x];
			if( p.x != -1 ) {
				//printf("x:%d, y:%d, p.x:%d, p.y:%d\n", x, y, p.x, p.y);
				dst.at<uchar>( p.y, 3*p.x ) = src_resize.at<uchar>( y, 3*x );      // B
				dst.at<uchar>( p.y, 3*p.x+1 ) = src_resize.at<uchar>( y, 3*x+1 );  // G
				dst.at<uchar>( p.y, 3*p.x+2 ) = src_resize.at<uchar>( y, 3*x+2 );  // R
			}
		}
	}
}
//
//// �S��ʕ\��
//void GRAYCODE::displayFullScreen(cv::Mat &image)
//{
//	/*  �S��ʕ\���p�E�B���h�E�̍쐬  */
//	cv::namedWindow(GC, 0);
//	HWND windowHandle = ::FindWindowA(NULL, GC);
//	// �E�B���h�E�X�^�C���ύX�i���j���[�o�[�Ȃ��A�őO��)
//	SetWindowLongPtr(windowHandle,  GWL_STYLE, WS_POPUP);
//	SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);
//	// �ő剻����
//	ShowWindow(windowHandle, SW_MAXIMIZE);
//	cv::setWindowProperty(GC, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN );
//	// �f�B�X�v���C�T�C�Y���擾
//	//int mainDisplayWidth = GetSystemMetrics(SM_CXSCREEN);
//	//int mainDisplayHeight = GetSystemMetrics(SM_CYSCREEN);
//	// �N���C�A���g�̈���f�B�X�v���C�ɍ��킹��
//	SetWindowPos(windowHandle, NULL,
//		PRJ_X, PRJ_Y, PRJ_WIDTH, PRJ_HEIGHT, SWP_FRAMECHANGED | SWP_NOZORDER);
//
//	cv::imshow(GC, image);
//	cv::waitKey(0);
//}



/***********************************
** �ǉ� **
************************************/


// �J�������W�ɑ΂���v���W�F�N�^�̑Ή��_��Ԃ�
void GRAYCODE::getCorrespondProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint)
{
	for (int i=0; i < imagePoint.size(); ++i)
	{
		cv::Point2f point = c->ProCam[int(imagePoint[i].y+0.5f)][int(imagePoint[i].x+0.5f)];

		if( point.x != -1.0f) {
			projPoint.emplace_back(point);
		}
	}
}


// �J�������W�ɑ΂���v���W�F�N�^�̑Ή��_��Ԃ�(�����x��)
void GRAYCODE::getCorrespondSubPixelProjPoints(std::vector<cv::Point2f> &projPoint, const std::vector<cv::Point2f> &imagePoint, int size)
{
	for (int i=0; i < imagePoint.size(); ++i)
	{
		std::vector<cv::Point2f> iPoints, pPoints;
		if(imagePoint[i].x > size && imagePoint[i].x+size < CAMERA_WIDTH && imagePoint[i].y > size && imagePoint[i].y+size < CAMERA_HEIGHT)
		{
			// �̈斈�̑Ή��_
			for( float h = imagePoint[i].y-size; h < imagePoint[i].y+size; h+=1.0f){
				for( float w = imagePoint[i].x-size; w < imagePoint[i].x+size; w+=1.0f){
					cv::Point2f point = c->ProCam[int(h+0.5f)][int(w+0.5f)];
					if( point.x != -1.0f) {
						iPoints.emplace_back(cv::Point2f(w, h));
						pPoints.emplace_back(point);
					}
				}
			}

			// �Ή��_���m��Homography�̌v�Z
			cv::Mat H = cv::findHomography(iPoints, pPoints, CV_RANSAC, 2.0);
			// Homography���g���ă`�F�b�J�[�p�^�[���̌�_���ˉe
			cv::Point3d Q = cv::Point3d(cv::Mat(H * cv::Mat(cv::Point3d(imagePoint[i].x,imagePoint[i].y,1.0))));
			projPoint.emplace_back(cv::Point2f(Q.x/Q.z, Q.y/Q.z));
		}
		else
		{
			cv::Point2f point = c->ProCam[int(imagePoint[i].y+0.5f)][int(imagePoint[i].x+0.5f)];

			if( point.x != -1.0f) {
				projPoint.emplace_back(point);
			}
		}
	}
}


// �Ή��̂Ƃꂽ�_��S�ĕԂ�
void GRAYCODE::getCorrespondAllPoints(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<int> &flag) 
{
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			cv::Point p = c->CamPro[y][x];
			if( p.x != -1 ) {
				flag.emplace_back(1);
				projPoint.emplace_back(cv::Point2f(x, y));
				imagePoint.emplace_back(cv::Point2f(p.x, p.y));
			}
			else
			{
				flag.emplace_back(0);
				projPoint.emplace_back(cv::Point2f(-1, -1));
				imagePoint.emplace_back(cv::Point2f(-1, -1));
			}
		}
	}
}

void GRAYCODE::getCorrespondAllPoints_ProCam(std::vector<cv::Point2f> &projPoint, std::vector<cv::Point2f> &imagePoint, std::vector<int> &flag) 
{
	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {
			cv::Point p = c->ProCam[y][x];
			if( p.x != -1 ) {
				flag.emplace_back(1);
				projPoint.emplace_back(cv::Point2f(p.x, p.y));
				imagePoint.emplace_back(cv::Point2f(x, y));
			}
			else
			{
				flag.emplace_back(0);
				projPoint.emplace_back(cv::Point2f(-1, -1));
				imagePoint.emplace_back(cv::Point2f(-1, -1));
			}
		}
	}
}