#include "Graycode.h"
#include "Header.h"
#include "Calibration.h"
#include "PointCloudIO.h"
#include "PointCloudFilter.h"
#include "FeatureCloud.h"
#include "PGROpenCV.h"



#define MASK_ADDRESS "./GrayCodeImage/mask.bmp"
#define IMAGE_DIRECTORY "./UseImage"
#define SAVE_DIRECTORY "./UseImage/resize"

TPGROpenCV	pgrOpenCV(0);


//�X�L��������3�����_�Q�𓾂�
std::vector<cv::Point3f> scanScene(Calibration calib, GRAYCODE gc)
{
	// �O���C�R�[�h���e
	gc.code_projection();
	gc.make_thresh();
	gc.makeCorrespondence();

	//***�Ή��_�̎擾(�J������f��3�����_)******************************************
	std::vector<cv::Point2f> imagePoint_obj;
	std::vector<cv::Point2f> projPoint_obj;
	std::vector<int> isValid_obj; //�L���ȑΉ��_���ǂ����̃t���O
	std::vector<cv::Point3f> reconstructPoint;
	gc.getCorrespondAllPoints_ProCam(projPoint_obj, imagePoint_obj, isValid_obj);

	// �Ή��_�̘c�ݏ���
	std::vector<cv::Point2f> undistort_imagePoint_obj;
	std::vector<cv::Point2f> undistort_projPoint_obj;
	cv::undistortPoints(imagePoint_obj, undistort_imagePoint_obj, calib.cam_K, calib.cam_dist);
	cv::undistortPoints(projPoint_obj, undistort_projPoint_obj, calib.proj_K, calib.proj_dist);
	for(int i=0; i<imagePoint_obj.size(); ++i)
	{
		if(isValid_obj[i] == 1)
		{
			undistort_imagePoint_obj[i].x = undistort_imagePoint_obj[i].x * calib.cam_K.at<double>(0,0) + calib.cam_K.at<double>(0,2);
			undistort_imagePoint_obj[i].y = undistort_imagePoint_obj[i].y * calib.cam_K.at<double>(1,1) + calib.cam_K.at<double>(1,2);
			undistort_projPoint_obj[i].x = undistort_projPoint_obj[i].x * calib.proj_K.at<double>(0,0) + calib.proj_K.at<double>(0,2);
			undistort_projPoint_obj[i].y = undistort_projPoint_obj[i].y * calib.proj_K.at<double>(1,1) + calib.proj_K.at<double>(1,2);
		}
		else
		{
			undistort_imagePoint_obj[i].x = -1;
			undistort_imagePoint_obj[i].y = -1;
			undistort_projPoint_obj[i].x = -1;
			undistort_projPoint_obj[i].y = -1;
		}
	}

	// 3��������
	std::cout << "3�����������c" << std::ends;
	calib.reconstruction(reconstructPoint, undistort_projPoint_obj, undistort_imagePoint_obj, isValid_obj);
	std::cout << "����" << std::endl;
	return reconstructPoint;
}

// �I�C���[�p���s��ɕϊ�
void eular2rot(double yaw,double pitch, double roll, cv::Mat& dest)
{
    double theta = yaw/180.0*CV_PI;
    double pusai = pitch/180.0*CV_PI;
    double phi = roll/180.0*CV_PI;
 
    double datax[3][3] = {{1.0,0.0,0.0}, 
    {0.0,cos(theta),-sin(theta)}, 
    {0.0,sin(theta),cos(theta)}};
    double datay[3][3] = {{cos(pusai),0.0,sin(pusai)}, 
    {0.0,1.0,0.0}, 
    {-sin(pusai),0.0,cos(pusai)}};
    double dataz[3][3] = {{cos(phi),-sin(phi),0.0}, 
    {sin(phi),cos(phi),0.0}, 
    {0.0,0.0,1.0}};

    cv::Mat Rx(3,3,CV_64F,datax);
    cv::Mat Ry(3,3,CV_64F,datay);
    cv::Mat Rz(3,3,CV_64F,dataz);
    cv::Mat rr=Rz*Rx*Ry;

    rr.copyTo(dest);
}

////�ϊ��s��̏o��
//void print4x4Matrix (const Eigen::Matrix4d & matrix)
//{
//  printf ("Rotation matrix :\n");
//  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
//  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
//  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
//  printf ("Translation vector :\n");
//  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
//}
//
////�ϊ��s���csv�ŕۑ�(1�s�ڂ�T,2�s�ڂ�R)
//void save4x4MatricCSV(const Eigen::Matrix4d & matrix)
//{
//    ofstream ofs("ICPresult.csv"); //�t�@�C���o�̓X�g���[��
//    ofs<< matrix (0, 3) << "," << matrix (1, 3) << "," << matrix (2, 3)<<endl; //T
//    ofs<< matrix (0, 0) << "," << matrix (0, 1) << "," << matrix (0, 2) << 
//			  matrix (1, 0) << "," << matrix (1, 1) << "," << matrix (1, 2) <<
//			  matrix (2, 0) << "," << matrix (2, 1) << "," << matrix (2, 2)  << endl;//R(1�s�ځ�2�s�ځ�3�s��)
//}
//
//
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
////�_�Q�̉���
//void showViewer(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_icp)
//{
//  // Visualization
//  pcl::visualization::PCLVisualizer viewer ("ICP demo");
//  // Create two verticaly separated viewports
//  int v1 (0);
//  viewer.createViewPort (0.0, 0.0, 1.0, 1.0, v1);
//
//  // Original point cloud is white
//  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 255, 255, 255);
//  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in", v1);
//
//  // ICP aligned point cloud is red
//  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
//  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp", v1);
//
//  // Set background color
//  viewer.setBackgroundColor (0, 0, 0, v1);
//
//  // Set camera position and orientation
//  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//  viewer.setSize (1280, 1024);  // Visualiser window size
//
//  // Display the visualiser
//  while (!viewer.wasStopped ())
//  {
//    viewer.spinOnce ();
//  }
//
//}
//
////�����ʒu���킹
//void initialEstimation(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_icp, std::string model_filename)
//{
//
//  // Load the object templates specified in the object_templates.txt file
//  std::vector<FeatureCloud> object_templates;
//  object_templates.resize (0); 
//  FeatureCloud template_cloud;
//  template_cloud.loadInputCloud (model_filename);
//  object_templates.push_back (template_cloud);
//
//  // Preprocess the cloud by...
//  // ...removing distant points
//  const float depth_limit = 1.0;
//  pcl::PassThrough<PointT> pass;
//  pass.setInputCloud (cloud_in);
//  pass.setFilterFieldName ("z");
//  pass.setFilterLimits (0, depth_limit);
//  pass.filter (*cloud_in);
//
//  // ... and downsampling the point cloud
//  const float voxel_grid_size = 0.005f;
//  pcl::VoxelGrid<PointT> vox_grid;
//  vox_grid.setInputCloud (cloud_in);
//  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
//  //vox_grid.filter (*cloud_in); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
//  pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<PointT>); 
//  vox_grid.filter (*tempCloud);
//  cloud_in = tempCloud; 
//
//  // Assign to the target FeatureCloud
//  FeatureCloud target_cloud;
//  target_cloud.setInputCloud (cloud_in);
//
//  // Set the TemplateAlignment inputs
//  TemplateAlignment template_align;
//  for (size_t i = 0; i < object_templates.size (); ++i)
//  {
//    template_align.addTemplateCloud (object_templates[i]);
//  }
//  template_align.setTargetCloud (target_cloud);
//
//  // Find the best template alignment
//  TemplateAlignment::Result best_alignment;
//  int best_index = template_align.findBestAlignment (best_alignment);
//  const FeatureCloud &best_template = object_templates[best_index];
//
//  // Print the alignment fitness score (values less than 0.00002 are good)
//  printf ("Best fitness score: %f\n", best_alignment.fitness_score);
//
//  // Print the rotation matrix and translation vector
//  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
//  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);
//
//  printf ("\n");
//  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
//  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
//  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
//  printf ("\n");
//  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
//
//  //model�_�Q�������ʒu�ֈړ�
//  PointCloudT::Ptr cloud_icp_trans (new PointCloudT);  // Original point cloud
//  pcl::transformPointCloud(*cloud_icp, *cloud_icp_trans, best_alignment.final_transformation);
//  cloud_icp = cloud_icp_trans;
//}
//
////ICP�ɂ��ʒu���o
//int detectPosition(std::string src_file, std::string model_file)
//{
//  // The point clouds we will be using
//  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
//  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
//
//  // Defining a rotation matrix and translation vector
//  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
//
//  //�V�[���_�Q�ƃ��f���_�Q�̓ǂݍ���
//  if (pcl::io::loadPLYFile (src_file, *cloud_in) < 0)
//  {
//    PCL_ERROR ("Error loading cloud %s.\n", src_file);
//    return (-1);
//  }
//  std::cout << "\nLoaded file " << src_file << " (" << cloud_in->size () << " points)" << std::endl;
//  if (pcl::io::loadPLYFile (model_file, *cloud_icp) < 0)
//  {
//    PCL_ERROR ("Error loading cloud %s.\n", model_file);
//    return (-1);
//  }
//  std::cout << "\nLoaded file " << model_file << " (" << cloud_icp->size () << " points)" << std::endl;
//
//  ////�V�[���_�Q�_�E���T���v�����O
//  //float size = 0.005;//5mm
//  //PointCloudT::Ptr cloud_in_sampled (new PointCloudT);  // ICP output point cloud
//  //// Create the filtering object
//  //pcl::VoxelGrid<PointT> sor;
//  //sor.setInputCloud (cloud_in);
//  //sor.setLeafSize (size, size, size);
//  //sor.filter (*cloud_in_sampled);
//  //std::cout << "\nDown sampled " << src_file << " (" << cloud_in_sampled->size () << " points)" << std::endl;
//  ////���f���_�Q�_�E���T���v�����O
//  //PointCloudT::Ptr cloud_icp_sampled (new PointCloudT);  // ICP output point cloud
//  //// Create the filtering object
//  //pcl::VoxelGrid<PointT> sor_;
//  //sor_.setInputCloud (cloud_icp);
//  //sor_.setLeafSize (size, size, size);
//  //sor_.filter (*cloud_icp_sampled);
//  //std::cout << "\nDown sampled " << model_file << " (" << cloud_icp_sampled->size () << " points)" << std::endl;
//
//  //�����ʒu���킹
//  initialEstimation(cloud_in, cloud_icp, model_file);
//
//  //�_�Q�Z�b�g&icp
//  pcl::IterativeClosestPoint<PointT, PointT> icp;
//  //icp.setMaximumIterations(100);
//  icp.setInputSource (cloud_icp);
//  icp.setInputTarget (cloud_in);
//  icp.align (*cloud_icp);
//  if (icp.hasConverged ())
//  {
//    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
//    transformation_matrix = icp.getFinalTransformation ().cast<double>();
//    print4x4Matrix (transformation_matrix);
//	save4x4MatricCSV(transformation_matrix);
//	//viewer
//	showViewer(cloud_in, cloud_icp);
//  }
//  else
//  {
//    PCL_ERROR ("\nICP has not converged.\n");
//    return (-1);
//  }
//
// return (0);
//
//}
//
//�_�Q�m�F
void viewPoints(Calibration calib, const cv::Mat &cam, const std::vector<cv::Point3f> &points)
{
			// �`��
			cv::Mat R = cv::Mat::eye(3,3,CV_64F);
			cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
			int key=0;
			cv::Point3d viewpoint(0.0,0.0,400.0);		// ���_�ʒu
			cv::Point3d lookatpoint(0.0,0.0,0.0);	// ��������
			const double step = 50;

			// �L�[�{�[�h����
			while(true)
			{
				//// ��]�̍X�V
				double x=(lookatpoint.x-viewpoint.x);
				double y=(lookatpoint.y-viewpoint.y);
				double z=(lookatpoint.z-viewpoint.z);
				double pitch =asin(x/sqrt(x*x+z*z))/CV_PI*180.0;
				double yaw   =asin(-y/sqrt(y*y+z*z))/CV_PI*180.0;
				eular2rot(yaw, pitch, 0, R);
				// �ړ��̍X�V
				t.at<double>(0,0)=viewpoint.x;
				t.at<double>(1,0)=viewpoint.y;
				t.at<double>(2,0)=viewpoint.z;

				//�J������f��3�����_
				calib.pointCloudRender(points, cam, std::string("viewer"), R, t);

				key = cv::waitKey(0);
				if(key=='w')
				{
					viewpoint.y+=step;
				}
				if(key=='s')
				{
					viewpoint.y-=step;
				}
				if(key=='a')
				{
					viewpoint.x+=step;
				}
				if(key=='d')
				{
					viewpoint.x-=step;
				}
				if(key=='z')
				{
					viewpoint.z+=step;
				}
				if(key=='x')
				{
					viewpoint.z-=step;
				}
				if(key=='q')
				{
					break;
				}
			}
}


//*****************************************************************************************//
//�J������f���Ƀ��b�V���͂���ply�ŕۑ�
//PLY�`���ŕۑ�(�@���Ȃ�,mesh����)
void savePLY_with_oreore_mesh(std::vector<cv::Point3f> reconstructPoints, const std::string &fileName)
{
	//���b�V���̐���
	std::vector<cv::Point3i> meshes;

	for( int y = 0; y < CAMERA_HEIGHT; y++ ) {
		for( int x = 0; x < CAMERA_WIDTH; x++ ) {
			if((y + 1) < CAMERA_HEIGHT && (x + 1) < CAMERA_WIDTH)
			{

				int index0 = y * CAMERA_WIDTH + x;
				int index1 = y * CAMERA_WIDTH + x + 1;
				int index2 = (y + 1) * CAMERA_WIDTH + x;
				int index3 = (y + 1) * CAMERA_WIDTH + x + 1;

				if(reconstructPoints[index0].x != -1 && reconstructPoints[index1].x != -1 && reconstructPoints[index2].x != -1)
				{
					meshes.emplace_back(cv::Point3i(index0, index2, index1));
				}
				if(reconstructPoints[index1].x != -1 && reconstructPoints[index2].x != -1 && reconstructPoints[index3].x != -1)
				{
					meshes.emplace_back(cv::Point3i(index1, index2, index3));
				}

			}
		}
	}

	//�t�@�C���I�[�v��
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//�t�@�C���ɏ�������
	//�w�b�_�̐ݒ�
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nelement face %d\nproperty list ushort int vertex_indices\nend_header\n", reconstructPoints.size(), meshes.size());

	//3�����_�Q
	//m�P�ʂŕۑ��ixml��mm�j
	for (int n = 0; n < reconstructPoints.size(); n++){
	   fprintf(fp, "%f %f %f\n", reconstructPoints[n].x/1000, reconstructPoints[n].y/1000, reconstructPoints[n].z/1000); //-1�������Ă�
	}
	//�ʏ��L�q
	for(int n = 0; n < meshes.size(); n++)
	{
	   fprintf(fp, "3 %d %d %d\n", meshes[n].x, meshes[n].y, meshes[n].z);
	}
	//�t�@�C���N���[�Y
	fclose(fp);
}
//*****************************************************************************************//


int main()
{
	pgrOpenCV.init(FlyCapture2::PIXEL_FORMAT_MONO8, FlyCapture2::HQ_LINEAR); //HQ_LINEAR
	//pgrOpenCV.setCameraParams(4.0);
	GRAYCODE gc;

	// �J�����摜�m�F�p
	char windowNameCamera[] = "camera";
	cv::namedWindow(windowNameCamera, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(windowNameCamera, 500, 300);

	static bool prjWhite = false;

	// �L�����u���[�V�����p
	Calibration calib(10, 7, 48.0);
	std::vector<std::vector<cv::Point3f>>	worldPoints;
	std::vector<std::vector<cv::Point2f>>	cameraPoints;
	std::vector<std::vector<cv::Point2f>>	projectorPoints;
	int calib_count = 0;

	//�w�i��臒l(mm)
	double thresh = 10.0; //1500

	//�w�i�ƑΏە���3�����_
	std::vector<cv::Point3f> reconstructPoint_back;
	std::vector<cv::Point3f> reconstructPoint_obj;

		printf("====================\n");
		printf("�J�n����ɂ͉����L�[�������Ă�������....\n");
		int command;

		// �����摜��S��ʂœ��e�i�B�e�����m�F���₷�����邽�߁j
		pgrOpenCV.start();
		cv::Mat cam, cam2;
		while(true){
			// true�Ŕ��𓊉e�Afalse�Œʏ�̃f�B�X�v���C��\��
			if(prjWhite){
				cv::Mat white = cv::Mat(PROJECTOR_WIDTH, PROJECTOR_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::namedWindow("white_black", 0);
				Projection::MySetFullScrean(DISPLAY_NUMBER, "white_black");
				cv::imshow("white_black", white);
			}

			// �����̃L�[�����͂��ꂽ�烋�[�v�𔲂���
			command = cv::waitKey(33);
			if ( command > 0 ) break;

			pgrOpenCV.queryFrame();
			cam = pgrOpenCV.getVideo();
			cam.copyTo(cam2);

			//���₷���悤�ɓK���Ƀ��T�C�Y
			cv::resize(cam, cam, cv::Size(), 0.45, 0.45);
			cv::imshow(windowNameCamera, cam);
		}

		// �J�������~�߂�
		pgrOpenCV.stop();


	//1. �L�����u���[�V�����t�@�C���ǂݍ���
	std::cout << "�L�����u���[�V�������ʂ̓ǂݍ��ݒ��c" << std::endl;
	calib.loadCalibParam("calibration.xml");

	std::cout << "1: �ۑ��ςݔw�i(��������)�̓ǂݍ���\n2: �w�i���X�L����&���������ĕۑ�" << std::endl;
	int key = cv::waitKey(0);

	while(true)
	{
		if(key == '1')
		{
			//2-1. �w�i�_�Q(��������)�ǂݍ���
			std::cout << "�w�i�_�Q�̓ǂݍ��ݒ��c" << std::endl;
			reconstructPoint_back = loadXMLfile("reconstructPoints_camera.xml");

			//�m�F
			viewPoints(calib, cam2, reconstructPoint_back);

			break;
		}
		else if(key == '2')
		{
			//2-2. �w�i�X�L�����A���������ĕۑ�//
			std::cout << "�w�i���X�L�������܂��c" << std::endl;
			std::vector<cv::Point3f> reconstructPoint_back_raw;
			reconstructPoint_back_raw = scanScene(calib, gc);

			std::cout << "�w�i�𕽊������܂��c" << std::ends;
			//���f�B�A���t�B���^�ɂ�镽����
			calib.smoothReconstructPoints(reconstructPoint_back_raw, reconstructPoint_back, 11); //z<0�̓_�̓\�[�g�ΏۊO�ɂ���H
			std::cout << "����" << std::endl;
			//==�ۑ�==//
			cv::FileStorage fs_obj("./reconstructPoints_camera.xml", cv::FileStorage::WRITE);
			write(fs_obj, "points", reconstructPoint_back);
			std::cout << "�w�i��ۑ����܂����c" << std::endl;
			
			//�m�F
			viewPoints(calib, cam2, reconstructPoint_back);

			break;
		}
		else 
		{
			key = cv::waitKey(0);
			continue;
		}
	}

	//--�҂�--//
	std::cout << "�Ώە��̂�u���Ă�������\n�������ł����牽���L�[�������Ă��������c" << std::endl;
	cv::waitKey(0);

	//3. �Ώە��̓���̃V�[���ɃO���C�R�[�h���e���A3��������
	std::cout << "�Ώە��̂��X�L�������܂��c" << std::endl;
	reconstructPoint_obj = scanScene(calib, gc);

	//*****************************************************************************************//
	//��f�Ή����Ă邩�m�F
	//--�҂�--//
	std::cout << "�ڕW�摜�� target.jpg �Ƃ��Ĕz�u���Ă��������c" << std::endl;
	cv::waitKey(0);
	//�ڕW�摜�ǂݍ���
	cv::Mat target = cv::imread("target.jpg");
	//�Ή��m�F
	cv::Mat dst;//�J�����摜���ڕW�摜�ƂȂ邽�߂̃v���W�F�N�^�摜
	gc.transport_camera_projector(target, dst);
	//�ۑ�
	cv::imwrite("target_pro.jpg", dst);
	//���e
	cv::namedWindow("dst", 0);
	Projection::MySetFullScrean(DISPLAY_NUMBER, "dst");
	cv::imshow("dst", dst);
	cv::waitKey(0);
	//�J�����摜�ۑ�
	pgrOpenCV.start();
	pgrOpenCV.queryFrame();
	pgrOpenCV.stop();
	cv::imwrite("target_cam.jpg", pgrOpenCV.getVideo());
	std::cout << "�J�����摜��ۑ����܂����c" << std::endl;
	cv::waitKey(0);
	//*****************************************************************************************//
	//�����x3��������(����4~6���ȉ��Œu��������)�@<����>GRAYCODE::makeCorrespondence()����interpolation()��L���ɂ��Ƃ�����
	//*****************************************************************************************//
	//������
	std::vector<cv::Point3f> reconstructPoint_smoothed;
	calib.smoothReconstructPoints(reconstructPoint_obj, reconstructPoint_smoothed, 11);
	//4. �w�i����
	for(int i = 0; i < reconstructPoint_smoothed.size(); i++)
	{
		//臒l�����[�x�̕ω���������������A(-1,-1,-1)�Ŗ��߂�
		if(reconstructPoint_smoothed[i].z != -1 && reconstructPoint_back[i].z != -1 && abs(reconstructPoint_smoothed[i].z - reconstructPoint_back[i].z) < thresh) //�������̕����������H
		{
		reconstructPoint_smoothed[i].x = -1;
		reconstructPoint_smoothed[i].y = -1;
		reconstructPoint_smoothed[i].z = -1;
		}
	}
	//�J������f�̕��т�3�p���b�V�������Eply�ۑ�
	savePLY_with_oreore_mesh(reconstructPoint_smoothed, "reconstructPoints_oreore.ply");
	//*****************************************************************************************//



	////4. �w�i����
	//for(int i = 0; i < reconstructPoint_obj.size(); i++)
	//{
	//	//臒l�����[�x�̕ω���������������A(-1,-1,-1)�Ŗ��߂�
	//	if(reconstructPoint_obj[i].z != -1 && reconstructPoint_back[i].z != -1 && abs(reconstructPoint_obj[i].z - reconstructPoint_back[i].z) < thresh) //�������̕����������H
	//	//if(reconstructPoint_obj[i].z == -1 || reconstructPoint_back[i].z == -1 || abs(reconstructPoint_obj[i].z - reconstructPoint_back[i].z) < thresh)
	//	{
	//	reconstructPoint_obj[i].x = -1;
	//	reconstructPoint_obj[i].y = -1;
	//	reconstructPoint_obj[i].z = -1;
	//	}
	//}


	////5. �@���AMesh����
	//std::cout << "���f���������c" << std::endl;
	////�L���ȓ_�̂ݎ�肾��(= -1�͏���)
	//std::vector<cv::Point3f> validPoints;
	//for(int n = 0; n < reconstructPoint_obj.size(); n++)
	//{
	//	if(reconstructPoint_obj[n].x != -1) validPoints.emplace_back(cv::Point3f(reconstructPoint_obj[n].x/1000, reconstructPoint_obj[n].y/1000, reconstructPoint_obj[n].z/1000)); //�P�ʂ�m��
	//}
	////�@�������߂�
	//std::vector<cv::Point3f> normalVecs = getNormalVectors(validPoints);
	////���b�V�������߂�
	//pcl::PolygonMesh triangles;
	//std::vector<cv::Point3i> meshes = getMeshVectors(validPoints, normalVecs, triangles);


	////6. PLY�`���ŕۑ�
	//std::cout << "���f����ۑ����܂��c" << std::endl;
	//savePLY_with_normal_mesh(validPoints, normalVecs, meshes, "reconstructPoint_obj.ply");
	////6. OBJ�`���ŕۑ�
	//pcl::io::saveOBJFile("reconstructPoint_obj.obj", triangles); //->Unity��ɂ�Rotate(0, 0, 180)�Ŕz�u
	//std::cout << "���f����ۑ����܂����c" << std::endl;


	//7. ICP�ňʒu���o(�ł��Ȃ��B�B)
	//std::cout << "ICP�ɂ��ʒu�����o���܂��c" << std::endl;
	//int result = detectPosition("reconstructPoint_obj.ply", "dora_small.ply");


	std::cout << "�I������ɂ͉����L�[�������Ă�������" << std::endl;

	cv::waitKey(0);
	return 0;
}
