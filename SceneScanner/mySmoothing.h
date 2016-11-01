#pragma once

#include <stdio.h>
#include <tchar.h>
#include <string>


namespace mySmooth{
	double gauss_filter3[3][3] = {
		{1.0/16.0, 2.0/16.0, 1.0/16.0},
		{2.0/16.0, 4.0/16.0, 2.0/16.0},
		{1.0/16.0, 2.0/16.0, 1.0/16.0},
	};

	double gauss_filter5[5][5] = {
		{1.0/256.0, 4.0/256.0, 6.0/256.0, 4.0/256.0, 1.0/256.0},
		{4.0/256.0, 16.0/256.0, 24.0/256.0, 16.0/256.0, 4.0/256.0},
		{6.0/256.0, 24.0/256.0, 36.0/256.0, 24.0/256.0, 6.0/256.0},
		{4.0/256.0, 16.0/256.0, 24.0/256.0, 16.0/256.0, 4.0/256.0},
		{1.0/256.0, 4.0/256.0, 6.0/256.0, 4.0/256.0, 1.0/256.0},
	};

	double gauss_filter7[7][7] = {
		{1.0/4096.0, 6.0/4096.0, 15.0/4096.0, 20.0/4096.0, 15.0/4096.0, 6.0/4096.0, 1.0/4096.0},
		{6.0/4096.0, 36.0/4096.0, 90.0/4096.0, 120.0/4096.0, 90.0/4096.0, 36.0/4096.0, 6.0/4096.0},
		{15.0/4096.0, 90.0/4096.0, 225.0/4096.0, 300.0/4096.0, 225.0/4096.0, 90.0/4096.0, 15.0/4096.0},
		{20.0/4096.0, 120.0/4096.0, 300.0/4096.0, 400.0/4096.0, 300.0/4096.0, 120.0/4096.0, 20.0/4096.0},
		{15.0/4096.0, 90.0/4096.0, 225.0/4096.0, 300.0/4096.0, 225.0/4096.0, 90.0/4096.0, 15.0/4096.0},
		{6.0/4096.0, 36.0/4096.0, 90.0/4096.0, 120.0/4096.0, 90.0/4096.0, 36.0/4096.0, 6.0/4096.0},
		{1.0/4096.0, 6.0/4096.0, 15.0/4096.0, 20.0/4096.0, 15.0/4096.0, 6.0/4096.0, 1.0/4096.0},
	};

	//���ϒl�̎擾
	float get_average(float** src, int kernel, int x, int y){
		float sum = 0.0f;
		int num = 0;//�L���l�̃J�E���^
		int halfRang = (int)floor((double)kernel/2); //�v�Z���ꂽ�l�ɏ����_�ȉ��؂�̂Ă��s�������ʂ�Ԃ�

		for(int i=0; i<kernel; i++){
			for(int t=0; t<kernel; t++){
				if(src[y+(halfRang-t)][x+(halfRang-i)] >= 0.0f)
				{
					sum += (float)src[y+(halfRang-t)][x+(halfRang-i)];
					num++;
				}
			}
		}

		//�J�[�l���̋K��l�ȏオ�L���l�̏ꍇ�A�L���l�̕��ς�����ĕԂ�
		if(num >= kernel * kernel * 0.2)
		{
			//���ς����
			return (float)(sum / num);

			//std::cout << "(" << cx << ", " << "cy): " << dst << std::endl;
		}
		//�L���l���K��l�ȉ��̎��́A�G���[�l��Ԃ�
		else return -1.0f;
	}

	//���ψړ��t�B���^
	//�G�������@�̈��@
	//���ډ�f�̒��ډ�f��8�ߖT�̕��ϒl����f�̒l�Ƃ��邱�Ƃɂ��A�ڂ��������ł���
	//�񐔂𑝂����ƂɎG���͌����Ȃ��Ȃ邪�A�����ɉ摜���ڂ��Ă��܂��Ă���
	void moving_average(int kernel, float** src, float** dst, int w, int h){
		int halfRang = (int)floor((double)kernel/2); //�v�Z���ꂽ�l�ɏ����_�ȉ��؂�̂Ă��s�������ʂ�Ԃ�
	
		for(int y=0; y<h; y++){
			for(int x=0; x<w; x++){
				dst[y][x] = 0;
			}
		}
		float ave;
		for(int y=halfRang; y<h-halfRang; y++){
			for(int x=halfRang; x<w-halfRang; x++){
				ave = get_average(src, kernel, x, y);
				//std::cout << "getave:" << ave << std::endl;
				dst[y][x] = ave;
				//std::cout << "dst:" << dst[y][x] << std::endl;
			}
		}
	}

	//�d�ݕt���̕��ϒl���擾
	double get_average_with_weight(int** src, int kernel, int x, int y){
		double sum=0; 
		int halfRang;
		if(kernel > 7){
			 halfRang = (int)floor(7.0/2);
		}else{
			halfRang = (int)floor((double)kernel/2);
		}

		if(kernel == 3){
			for(int i=0; i<3; i++){
				for(int t=0; t<3; t++){
					sum += (double)src[y+(halfRang-t)][x+(halfRang-i)] * gauss_filter3[i][t];
				}
			}
		}else if(kernel == 5){
			for(int i=0; i<5; i++){
				for(int t=0; t<5; t++){
					sum += (double)src[y+(halfRang-t)][x+(halfRang-i)] * gauss_filter5[i][t];
				}
			}
		}else{
			for(int i=0; i<7; i++){
				for(int t=0; t<7; t++){
					sum += (double)src[y+(halfRang-t)][x+(halfRang-i)] * gauss_filter7[i][t];
				}
			}
		}

		return sum;
	}

	//�K�E�X�t�B���^
	void gauss_filter(int kernel, int** src, double** dst, int w, int h){
		int halfRang = (int)floor((double)kernel/2);

		if(kernel > 7){
			 printf("error : Size too big. Size of gauss filter is 3 or 5 or 7\n");
			 halfRang = (int)floor(7.0/2);
		}
	
		for(int y=0; y<h; y++){
			for(int x=0; x<w; x++){
				dst[y][x] = 0;
			}
		}

		for(int y=halfRang; y<h-halfRang; y++){
			for(int x=halfRang; x<w-halfRang; x++){
				dst[y][x] = get_average_with_weight(src, kernel, x, y);
			}
		}

	}

	//�o�u���\�[�g
	void buble_sort(int length, int* a){
		for(int i=0; i<length; i++){
			for(int j=length-1; j>i; j--){
				if(a[j]<a[j-1]){
					int t=a[j];
					a[j]=a[j-1];
					a[j-1]=t;
				}
			}
		}
	}

	void buble_sort(int length, short int* a){
		for(int i=0; i<length; i++){
			for(int j=length-1; j>i; j--){
				if(a[j]<a[j-1]){
					int t=a[j];
					a[j]=a[j-1];
					a[j-1]=t;
				}
			}
		}
	}

	//�N�C�b�N�\�[�g
	/* x, y, z �̒��Ԓl��Ԃ� */
	float med3(float x, float y, float z){
		if (x < y){
			if (y < z) return y;
			else if (z < x) return x;
			else return z;
		}else{
			if (z < y) return y;
			else if (x < z) return x;
			else return z;
		}
	}
	/* �N�C�b�N�\�[�g
	 * a     : �\�[�g����z��
	 * left  : �\�[�g����f�[�^�̊J�n�ʒu
	 * right : �\�[�g����f�[�^�̏I���ʒu
	 */
	void quicksort(float* a, int left, int right){
		if (left < right) {
			int i = left, j = right;
			float tmp, pivot = med3(a[i], a[i + (j - i) / 2], a[j]); /* (i+j)/2�ł̓I�[�o�[�t���[���Ă��܂� */
			while (1) { /* a[] �� pivot �ȏ�ƈȉ��̏W�܂�ɕ������� */
				while (a[i] < pivot) i++; /* a[i] >= pivot �ƂȂ�ʒu������ */
				while (pivot < a[j]) j--; /* a[j] <= pivot �ƂȂ�ʒu������ */
				if (i >= j) break;
				tmp = a[i]; a[i] = a[j]; a[j] = tmp; /* a[i],a[j] ������ */
				i++; j--;
			}
			quicksort(a, left, i - 1);  /* �������������ċA�I�Ƀ\�[�g */
			quicksort(a, j + 1, right); /* ���������E���ċA�I�Ƀ\�[�g */
		}
	}


	//�����l�̎擾
	float get_median(float* src, int kernel, int x, int y, int w, int h){
		float sum=0; 
		int halfRang = (int)floor((double)kernel/2);
		float* a = (float*)malloc(sizeof(float)*kernel*kernel);

		for(int i=0; i<kernel; i++){
			for(int t=0; t<kernel; t++){
				int index = (y+(halfRang-i))*w + x+(halfRang-t);
				a[i*kernel+t] = src[index];
			}
		}

		 //buble_sort(kernel*kernel, a);
		 quicksort(a, 0, kernel*kernel-1); // �N�C�b�N�\�[�g
		 sum = a[(int)((double)(kernel*kernel)+0.5)/2];

		free(a);
		return sum;
	}

	//���f�B�A���t�B���^
	//���������g���Ă���
	void median_filter(int kernel, float* src, float* dst, int h, int w){//w=512, h=424
		int halfRang = (int)floor((double)kernel/2);
	
		//dst�̏�����
		for(int y=0; y<w; y++){
			for(int x=0; x<h; x++){
				int index = y*h+x;
				dst[index] = 0;
			}
		}
	//dst = cv::Mat::zeros(w, h, CV_8UC1);
		for(int y=halfRang; y<h-halfRang; y++){
			for(int x=halfRang; x<w-halfRang; x++){
				int index = y*w + x;
				dst[index] = get_median(src, kernel, x, y, w, h);
			}
		}
	}
}