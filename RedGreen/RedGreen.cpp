// RedGreen.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"

#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <limits.h>

#include "DSM.h"
#include "EllipseParameterEstimate.h"
#include "matlib.h"      

#define CLASSIFY_ELLIPSE_SIZE 5
#define RESIDUAL_THRESH 5.0
#define GRAY_THRESH_RED 50
#define GRAY_THRESH_GREEN 50
#define GRAY_THRESH_BLUE 55
#define MIN_CIRCLE_PERIMETER 30                         //the lower bound of perimeter
#define MAX_CIRCLE_PERIMETER 200                        //the upper bound of perimeter
#define MIN_CIRCLE_RATE_AREA_PER_PERIMETER2 0.05        //the rate= area/(perimeter^2)
#define MIN_PARAMETER_NUM 6                             //the num of ellipse parameter
#define PI	3.14159265358979

enum color {RED, GREEN};

struct EACH_ELLIPSE           
{
	double center_x;
	double center_y;
	double long_axis;
	double short_axis;
	double rotation_angle;
	double residual;
	color ellipse_color;
};

struct EACH_CLASSIFY_ELLIPSE            //the ellipses in the same target is one classify
{
	int num;
	int classify_code[CLASSIFY_ELLIPSE_SIZE];
	std::array<EACH_ELLIPSE, CLASSIFY_ELLIPSE_SIZE> each_classify_ellipse;
};

using namespace std;
using namespace cv;

void BinaryImg(IplImage* p_source_image_copy, IplImage* p_red_channel, IplImage* p_green_channel, IplImage* p_gray_img);
void EllipseContour(IplImage* p_gray_img, IplImage* p_source_image_copy, std::vector<EACH_ELLIPSE>& AEE_rough);
void LeastSquaresEstimate(std::vector<Double_Point> &data, std::vector<double> &parameters);
void EllipseClassify(std::vector<EACH_ELLIPSE> AEE_rough, std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_rough);
double Distance(double x1, double y1, double x2, double y2);
void FindMinDistPoint(std::vector<EACH_ELLIPSE> &AEE_rough_copy, int* Tag, int i, std::vector<EACH_ELLIPSE> &each_classify_ellipse);
void TraceEqualContourLine(IplImage* p_source_image_copy, std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_rough, std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_precise);
void EncodeClassify(std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_precise, std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_precise_encode);

int _tmain(int argc, _TCHAR* argv[])
{
	/**************************************************************/
	/*load data*/
	IplImage*	p_source_image = cvLoadImage("data\\tmono1.jpg", 1);  
	/**************************************************************/

	if(!p_source_image)
	{
		cout << "Did not find the image" << endl;
		exit(1);
	}
	IplImage*	p_source_image_copy = cvCreateImage(cvGetSize(p_source_image), IPL_DEPTH_8U, 3);    //copy the source image
	IplImage*	p_temp_image = cvCreateImage(cvGetSize(p_source_image), IPL_DEPTH_8U, 3);
	
	//morphological processing
	cvMorphologyEx(p_source_image, p_source_image_copy, p_temp_image, NULL, CV_MOP_OPEN, 1);      
	cvMorphologyEx(p_source_image_copy, p_source_image_copy, p_temp_image, NULL, CV_MOP_CLOSE, 1); 
	cvReleaseImage(&p_temp_image);

	//receive red, green and blue channel data
	IplImage*	p_red_channel = cvCreateImage(cvGetSize(p_source_image_copy), IPL_DEPTH_8U, 1);    
	IplImage*	p_green_channel = cvCreateImage(cvGetSize(p_source_image_copy), IPL_DEPTH_8U, 1);  
	IplImage*	p_blue_channel = cvCreateImage(cvGetSize(p_source_image_copy), IPL_DEPTH_8U, 1);   
	cvSplit(p_source_image_copy, p_blue_channel, p_green_channel, p_red_channel, 0);               

	//binaryzation
	IplImage*	p_gray_img = cvCreateImage(cvGetSize(p_source_image_copy), IPL_DEPTH_8U, 1);
	BinaryImg(p_source_image_copy, p_red_channel, p_green_channel, p_gray_img);                    
	//cvSaveImage("result\\GrayImg.jpg", p_gray_img);

	std::vector<EACH_ELLIPSE> AEE_rough;                         
	EllipseContour(p_gray_img, p_source_image_copy, AEE_rough);    
	
	ofstream p_ellipse_center_result_rough;
	p_ellipse_center_result_rough.open("result\\CircleCenterRough.txt");
	for (int i = 0; i < AEE_rough.size(); i++)
	{
		p_ellipse_center_result_rough << AEE_rough[i].center_x << "  " << AEE_rough[i].center_y << "  "<< AEE_rough[i].long_axis 
			<< "  " << AEE_rough[i].short_axis << "  " << AEE_rough[i].rotation_angle << "  " << AEE_rough[i].residual 
			<< "  " << AEE_rough[i].ellipse_color << endl;
	}
	p_ellipse_center_result_rough.close();

	std::vector<EACH_CLASSIFY_ELLIPSE> ACE_rough;                
	EllipseClassify(AEE_rough, ACE_rough);

	ofstream p_ellipse_center_result_classify;
	p_ellipse_center_result_classify.open("result\\CircleCenterRoughClassify.txt");
	for (int i = 0; i < ACE_rough.size(); i++)
	{
		//p_ellipse_center_result_classify << ACE_rough[i].num << endl;
		for (int j = 0; j < ACE_rough[i].each_classify_ellipse.size(); j++)
		{
			p_ellipse_center_result_classify << ACE_rough[i].each_classify_ellipse[j].center_x << "  " 
				<< ACE_rough[i].each_classify_ellipse[j].center_y << "  " 
				<< ACE_rough[i].each_classify_ellipse[j].long_axis << "  " 
				<< ACE_rough[i].each_classify_ellipse[j].short_axis << "  " 
				<< ACE_rough[i].each_classify_ellipse[j].rotation_angle << "  " 
				<< ACE_rough[i].each_classify_ellipse[j].residual << "  " 
				<< ACE_rough[i].each_classify_ellipse[j].ellipse_color << endl;
		}
	}
	p_ellipse_center_result_classify.close();

	std::vector<EACH_CLASSIFY_ELLIPSE> ACE_precise;
	TraceEqualContourLine(p_source_image_copy, ACE_rough, ACE_precise);

	std::vector<EACH_CLASSIFY_ELLIPSE> ACE_precise_encode;
	EncodeClassify(ACE_precise, ACE_precise_encode);

	ofstream p_ellipse_center_precise_encode;
	p_ellipse_center_precise_encode.open("result\\CircleCenterPreciseClassifyEncode.txt");
	for (int i = 0; i < ACE_precise.size(); i++)
	{
		p_ellipse_center_precise_encode << ACE_precise_encode[i].num << "\t" 
			<< ACE_precise_encode[i].classify_code[0] << ACE_precise_encode[i].classify_code[1]  
			<< ACE_precise_encode[i].classify_code[2] << ACE_precise_encode[i].classify_code[3] 
			<< ACE_precise_encode[i].classify_code[4] << endl;
		for (int j = 0; j < ACE_precise[i].each_classify_ellipse.size(); j++)
		{
			p_ellipse_center_precise_encode << ACE_precise_encode[i].each_classify_ellipse[j].center_x << "  " 
				<< ACE_precise_encode[i].each_classify_ellipse[j].center_y << "  " 
				<< ACE_precise_encode[i].each_classify_ellipse[j].long_axis << "  " 
				<< ACE_precise_encode[i].each_classify_ellipse[j].short_axis << "  " 
				<< ACE_precise_encode[i].each_classify_ellipse[j].rotation_angle << "  " 
				<< ACE_precise_encode[i].each_classify_ellipse[j].residual << "  " 
				<< ACE_precise_encode[i].each_classify_ellipse[j].ellipse_color << "  " << endl;
		}
	}
	p_ellipse_center_precise_encode.close();

	cvReleaseImage(&p_source_image);
	cvReleaseImage(&p_source_image_copy);
	cvReleaseImage(&p_red_channel);
	cvReleaseImage(&p_green_channel);
	cvReleaseImage(&p_blue_channel);
	cvReleaseImage(&p_gray_img);

	printf("Operation finish!");

	return 0;
}

void BinaryImg(IplImage* p_source_image_copy, IplImage* p_red_channel, IplImage* p_green_channel, IplImage* p_gray_img)
{
	IplImage*	p_binary_red = cvCreateImage(cvGetSize(p_source_image_copy), IPL_DEPTH_8U, 1); 
	IplImage*	p_binary_green = cvCreateImage(cvGetSize(p_source_image_copy), IPL_DEPTH_8U, 1); 
	cvThreshold(p_red_channel , p_binary_red , GRAY_THRESH_RED, 255, CV_THRESH_BINARY);         //二值化红波段
	cvThreshold(p_green_channel , p_binary_green , GRAY_THRESH_GREEN, 255, CV_THRESH_BINARY);   //二值化绿波段
	cvSaveImage("result\\BinaryRed.jpg", p_binary_red);
	cvSaveImage("result\\BinaryGreen.jpg", p_binary_green);

	uchar* data_red = (uchar*)p_binary_red->imageData;
	int n_step_red = p_binary_red->widthStep/sizeof(uchar);
	uchar* data_green = (uchar*)p_binary_green->imageData;
	int n_step_green = p_binary_green->widthStep/sizeof(uchar);
	uchar* data = (uchar*)p_gray_img->imageData;
	int n_step = p_gray_img->widthStep/sizeof(uchar);

	for (int i = 0; i < p_gray_img->height; i++)
	{
		for (int j = 0 ;j < p_gray_img->width; j++)
		{
			int red_value, green_value;
			red_value = data_red[i*n_step_red + j];
			green_value = data_green[i*n_step_green + j];
			if(red_value == 0 && green_value == 0)
				data[i*n_step + j] = 0;
			else
				data[i*n_step + j] = 255;
		}
	}
	cvReleaseImage(&p_binary_red);
	cvReleaseImage(&p_binary_green);
}

void EllipseContour(IplImage* p_gray_img, IplImage* p_source_image_copy, std::vector<EACH_ELLIPSE>& AEE_rough)
{
	CvMemStorage*	p_all_contour_storage = cvCreateMemStorage(0);                   
	CvSeq*	p_all_contour = NULL;
	cvFindContours(p_gray_img, p_all_contour_storage, &p_all_contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);  
	//cvDrawContours(p_source_image_copy, p_all_contour, CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), 1, 2, 8, cvPoint(0, 0));

	if (p_all_contour)
	{
		CvSeq*	p_all_next_contour = NULL;                 
		CvTreeNodeIterator	all_contour_iterator;            

		cvInitTreeNodeIterator(&all_contour_iterator, p_all_contour, 1);
		while(0 != (p_all_next_contour = (CvSeq*)cvNextTreeNode(&all_contour_iterator)))
		{
			double d_all_perimeter = cvContourPerimeter(p_all_next_contour);                        //perimeter
			double d_all_area = cvContourArea(p_all_next_contour, CV_WHOLE_SEQ);                    //area
			double d_all_rate_area_per_perimeter2 = d_all_area/(d_all_perimeter*d_all_perimeter);   //rate

			//valid ellipse
			if (d_all_perimeter > MIN_CIRCLE_PERIMETER && d_all_perimeter < MAX_CIRCLE_PERIMETER && 
				d_all_rate_area_per_perimeter2 > MIN_CIRCLE_RATE_AREA_PER_PERIMETER2)
			{
				int	n_contour_point_num = p_all_next_contour->total;
				CvPoint*	p_ellipse_point = (CvPoint*)malloc(sizeof(CvPoint)*n_contour_point_num);
				std::vector<Double_Point*>	v_point_data_ptr;
				std::vector<Double_Point>	v_point_data;             
				
				CvSeqReader	reader;
				CvPoint	pt = cvPoint(0, 0);
				Double_Point temple_point_double = {0, 0};
				cvStartReadSeq(p_all_next_contour, &reader);

				
				Double_Point barycenter = {0, 0};
				for (int i = 0; i < n_contour_point_num; i++)
				{
					CV_READ_SEQ_ELEM(pt, reader);
					p_ellipse_point[i] = pt;
					temple_point_double.x = (double)pt.x;
					temple_point_double.y = (double)pt.y;
					barycenter.x += (double)pt.x/n_contour_point_num;
					barycenter.y += (double)pt.y/n_contour_point_num;
					v_point_data_ptr.push_back(&temple_point_double);
					v_point_data.push_back(*(v_point_data_ptr[i]));
				}

				
				uchar* data = (uchar*)p_source_image_copy->imageData;
				int n_step = p_source_image_copy->widthStep/sizeof(uchar);
				int n_channels = p_source_image_copy->nChannels;
				int red_value, green_value, blue_value, delta_value;
				red_value = data[((int)barycenter.y)*n_step + ((int)barycenter.x)*n_channels + 2];
				green_value	= data[((int)barycenter.y)*n_step + ((int)barycenter.x)*n_channels + 1];
				blue_value = data[((int)barycenter.y)*n_step + ((int)barycenter.x)*n_channels + 0];
				delta_value = abs(red_value - green_value);
				if (delta_value > 30 && (red_value > GRAY_THRESH_RED || green_value > GRAY_THRESH_GREEN) && blue_value < GRAY_THRESH_BLUE)
				{
					EACH_ELLIPSE ee;
					std::vector<double>	v_circle_fitting_parameters_rough;

					if (red_value > green_value)
						ee.ellipse_color = RED;
					else
						ee.ellipse_color = GREEN;

					//use least squares to estimate ellipse parameter
					LeastSquaresEstimate(v_point_data, v_circle_fitting_parameters_rough);

					ee.center_x = v_circle_fitting_parameters_rough[0];
					ee.center_y = v_circle_fitting_parameters_rough[1];
					ee.long_axis = v_circle_fitting_parameters_rough[2];
					ee.short_axis = v_circle_fitting_parameters_rough[3];
					ee.rotation_angle = v_circle_fitting_parameters_rough[4];
					ee.residual = v_circle_fitting_parameters_rough[5];
					AEE_rough.push_back(ee);
				}		
				free(p_ellipse_point);
			}
		}
	}
	cvReleaseMemStorage(&p_all_contour_storage);
}

void LeastSquaresEstimate(std::vector<Double_Point> &data, std::vector<double> &parameters)
{
	parameters.clear();
	if(data.size() < MIN_PARAMETER_NUM)
		return;

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//use matlib.h 
	initM(MATCOM_VERSION);

	int	d_dataSize = data.size();                     
	dMm(X_point);                                     
	dMm(Y_point);                                     
	dMm(x_point_norm);                                
	dMm(y_point_norm);                              
	dMm(design_matrix);                           

	X_point = zeros(d_dataSize, MIN_PARAMETER_NUM);  
	Y_point = zeros(d_dataSize, MIN_PARAMETER_NUM);
	x_point_norm = zeros(d_dataSize, 1);
	y_point_norm = zeros(d_dataSize, 1);
	design_matrix = zeros(d_dataSize, MIN_PARAMETER_NUM);

	double d_x_min, d_x_max, d_x_mean;
	double d_y_min, d_y_max, d_y_mean;
	d_x_mean = 0;	d_x_max = 0;	d_x_min = 1e+10;
	d_y_mean = 0;	d_y_max = 0;	d_y_min = 1e+10;

	for (int i = 0; i < d_dataSize; i++)
	{
		X_point.r(i+1, 1) = (double)data[i].x;
		Y_point.r(i+1, 1) = (double)data[i].y;

		if (X_point.r(i+1, 1) > d_x_max) d_x_max = X_point.r(i+1, 1);
		if (Y_point.r(i+1, 1) > d_y_max) d_y_max = Y_point.r(i+1, 1);

		if (X_point.r(i+1, 1) < d_x_min) d_x_min = X_point.r(i+1, 1);
		if (Y_point.r(i+1, 1) < d_y_min) d_y_min = Y_point.r(i+1, 1);

		d_x_mean = d_x_mean + X_point.r(i+1, 1);
		d_y_mean = d_y_mean + Y_point.r(i+1, 1);
	}
	d_x_mean = d_x_mean/d_dataSize;
	d_y_mean = d_y_mean/d_dataSize;

	double d_x_range = (d_x_max - d_x_min)/2.0;
	double d_y_range = (d_y_max - d_y_min)/2.0;

	for (int i = 0; i < d_dataSize; i++)
	{
		x_point_norm.r(i+1, 1) = (X_point.r(i+1, 1) - d_x_mean)/d_x_range;
		y_point_norm.r(i+1, 1) = (Y_point.r(i+1, 1) - d_y_mean)/d_y_range;
		design_matrix.r(i+1, 1) = x_point_norm.r(i+1, 1)*x_point_norm.r(i+1, 1);
		design_matrix.r(i+1, 2) = x_point_norm.r(i+1, 1)*y_point_norm.r(i+1, 1);
		design_matrix.r(i+1, 3) = y_point_norm.r(i+1, 1)*y_point_norm.r(i+1, 1);
		design_matrix.r(i+1, 4) = x_point_norm.r(i+1, 1);
		design_matrix.r(i+1, 5) = y_point_norm.r(i+1, 1);
		design_matrix.r(i+1, 6) = 1;

	}

	dMm(scatter_matrix);                               //Design matrix
	scatter_matrix = zeros(MIN_PARAMETER_NUM, MIN_PARAMETER_NUM);
	scatter_matrix = transpose(design_matrix)*design_matrix;

	dMm(constraint_matrix);
	constraint_matrix = zeros(MIN_PARAMETER_NUM, MIN_PARAMETER_NUM);
	constraint_matrix.r(1, 3) = -2;
	constraint_matrix.r(2, 2) = 1;
	constraint_matrix.r(3, 1) = -2;

	/*******************************************************************/
	/*   New way, numerically stabler in C [gevec, geval] = eig(S,C)   */
	dMm(tmpA);
	dMm(tmpB);
	dMm(tmpC);
	dMm(tmpD);
	dMm(tmpE);
	tmpA = zeros(3, 3);
	tmpB = zeros(3, 3);
	tmpC = zeros(3, 3);
	tmpD = zeros(3, 3);
	tmpE = zeros(3, 3);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			tmpA.r(i + 1, j + 1) = scatter_matrix.r(i + 1, j + 1);
			tmpB.r(i + 1, j + 1) = scatter_matrix.r(i + 1, j + 4);
			tmpC.r(i + 1, j + 1) = scatter_matrix.r(i + 4, j + 4);
			tmpD.r(i + 1, j + 1) = constraint_matrix.r(i + 1, j + 1);
		}
	}
	tmpE = inv(tmpC)*transpose(tmpB);

	dMm(evec_x);
	dMm(eval_x);
	eig(inv(tmpD)*(tmpA - tmpB*tmpE), i_o, evec_x, eval_x);

	int n_index = 0;
	for (int i = 0; i < 3; i++)
	{
		if (eval_x.r(i + 1, i + 1) <1e-8 && eval_x.r(i + 1, i + 1) < DBL_MAX && eval_x.r(i + 1, i + 1) > -DBL_MAX)
		{
			n_index = i+1;
		}
	}

	dMm(A_half_matrix);
	A_half_matrix = zeros(3, 1);
	for (int i = 0; i < 3; i++)
	{
		A_half_matrix.r(i + 1, 1) = evec_x.r(i + 1, n_index);
	}

	dMm(evec_y);
	evec_y = -tmpE * A_half_matrix;

	dMm(A_matrix);
	A_matrix = zeros(MIN_PARAMETER_NUM, 1);
	for (int i=0; i < MIN_PARAMETER_NUM; i++)
	{
		if (i < 3)
			A_matrix.r(i + 1, 1) = A_half_matrix.r(i + 1, 1);
		else
			A_matrix.r(i + 1, 1) = evec_y.r(i - 2, 1);
	}
	/*******************************************************************/

	dMm(par_vector);                
	par_vector = zeros(MIN_PARAMETER_NUM, 1);
	par_vector.r(1,1) = A_matrix.r(1,1)*d_y_range*d_y_range;
	par_vector.r(2,1) = A_matrix.r(2,1)*d_x_range*d_y_range;
	par_vector.r(3,1) = A_matrix.r(3,1)*d_x_range*d_x_range;
	par_vector.r(4,1) = -2*A_matrix.r(1,1)*d_y_range*d_y_range*d_x_mean - A_matrix.r(2,1)*d_x_range*d_y_range*d_y_mean + 
		A_matrix.r(4,1)*d_x_range*d_y_range*d_y_range;
	par_vector.r(5,1) = -A_matrix.r(2,1)*d_x_range*d_y_range*d_x_mean - 2*A_matrix.r(3,1)*d_x_range*d_x_range*d_y_mean + 
		A_matrix.r(5,1)*d_x_range*d_x_range*d_y_range;
	par_vector.r(6,1) = A_matrix.r(1,1)*d_y_range*d_y_range*d_x_mean*d_x_mean + A_matrix.r(2,1)*d_x_range*d_y_range*d_x_mean*d_y_mean + 
		A_matrix.r(3,1)*d_x_range*d_x_range*d_y_mean*d_y_mean - A_matrix.r(4,1)*d_x_range*d_y_range*d_y_range*d_x_mean - 
		A_matrix.r(5,1)*d_x_range*d_x_range*d_y_range*d_y_mean + A_matrix.r(6,1)*d_x_range*d_x_range*d_y_range*d_y_range;

	//display(par_vector);

	double	d_thetarad = 0.5*atan2(par_vector.r(2,1), par_vector.r(1,1) - par_vector.r(3,1));
	double	d_cost = cos(d_thetarad);
	double	d_sint = sin(d_thetarad);
	double	d_sin_squared = d_sint*d_sint;
	double	d_cos_squared = d_cost*d_cost;
	double	d_cos_sin = d_sint*d_cost;

	double d_Ao = par_vector.r(6,1);
	double d_Au = par_vector.r(4,1)*d_cost + par_vector.r(5,1)*d_sint;
	double d_Av = - par_vector.r(4,1)*d_sint + par_vector.r(5,1)*d_cost;
	double d_Auu = par_vector.r(1,1)*d_cos_squared + par_vector.r(3,1)*d_sin_squared + par_vector.r(2,1)*d_cos_sin;
	double d_Avv = par_vector.r(1,1)*d_sin_squared + par_vector.r(3,1)*d_cos_squared - par_vector.r(2,1)*d_cos_sin;

	// ROTATED = [d_Ao d_Au d_Av d_Auu d_Avv]
	double d_tuCentre = - d_Au/(2*d_Auu);
	double d_tvCentre = - d_Av/(2*d_Avv);
	double d_wCentre = d_Ao - d_Auu*d_tuCentre*d_tuCentre - d_Avv*d_tvCentre*d_tvCentre;

	double d_uCentre = d_tuCentre*d_cost - d_tvCentre*d_sint;
	double d_vCentre = d_tuCentre*d_sint + d_tvCentre*d_cost;

	double d_Ru = -d_wCentre/d_Auu;
	double d_Rv = -d_wCentre/d_Avv;

	d_Ru = sqrt(abs(d_Ru))*sign(d_Ru);
	d_Rv = sqrt(abs(d_Rv))*sign(d_Rv);

	//This part is the key of ellipse parameter estimate function.
	if (d_Ru > d_Rv)
	{
		if (d_thetarad < 0) d_thetarad += PI;
	}
	if (d_Ru < d_Rv)
	{
		double d_mid = 0;
		d_mid = d_Rv;
		d_Rv = d_Ru;
		d_Ru = d_mid;
		if (d_thetarad > PI/2.0) 
			d_thetarad -= PI/2.0;
		else if (d_thetarad > 0 && d_thetarad < PI/2.0)
			d_thetarad += PI/2.0;
		else if (d_thetarad < 0 && d_thetarad > -PI/2.0)
			d_thetarad += PI/2.0;
		else if (d_thetarad < -PI/2.0)
			d_thetarad += 1.5*PI;
	}

	double d_c = sqrt(d_Ru*d_Ru - d_Rv*d_Rv);
	double focus1_x = d_uCentre + cos(d_thetarad)*d_c;
	double focus1_y = d_vCentre + sin(d_thetarad)*d_c;
	double focus2_x = 2*d_uCentre - focus1_x;
	double focus2_y = 2*d_vCentre - focus1_y;

	double d_residual = 0;
	for (int i = 0 ; i < d_dataSize; i++)
	{
		double d_dist;
		d_dist = sqrt((data[i].x - focus1_x)*(data[i].x - focus1_x) + (data[i].y - focus1_y)*(data[i].y - focus1_y)) + 
			sqrt((data[i].x - focus2_x)*(data[i].x - focus2_x) + (data[i].y - focus2_y)*(data[i].y - focus2_y)) - 2*d_Ru;
		d_residual += d_dist*d_dist/d_dataSize;
	}


	if (d_residual < RESIDUAL_THRESH)
	{
		parameters.push_back(d_uCentre);
		parameters.push_back(d_vCentre);
		parameters.push_back(d_Ru);
		parameters.push_back(d_Rv);
		parameters.push_back(d_thetarad);
		parameters.push_back(d_residual);
	}

 	exitM();
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}

//ellipses clustering
void EllipseClassify(std::vector<EACH_ELLIPSE> AEE_rough, std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_rough)
{
	int AEE_num = AEE_rough.size();
	int* Tag = new int[AEE_num];       
	for (int i = 0; i < AEE_num; i++)
		Tag[i] = 0;

	int n_classify_num = 1;
	for (int i = 0; i < AEE_num; i++)
	{
		EACH_CLASSIFY_ELLIPSE ECE;
		std::vector<EACH_ELLIPSE> Classify_EE;
		
		if (0 == Tag[i])
		{
			Classify_EE.push_back(AEE_rough[i]);
			Tag[i] = 1;
			FindMinDistPoint(AEE_rough, Tag, i, Classify_EE);

			if (Classify_EE.size() == CLASSIFY_ELLIPSE_SIZE)
			{
				ECE.num = n_classify_num;
				n_classify_num++;
				for (int h = 0; h < CLASSIFY_ELLIPSE_SIZE; h++)     
				{
					ECE.each_classify_ellipse[h] = Classify_EE[h];
					ECE.classify_code[h] = 0;
				}
				ACE_rough.push_back(ECE);
			}
		}
	}
	delete[] Tag;
}

void FindMinDistPoint(std::vector<EACH_ELLIPSE> &AEE_rough, int* Tag, int i, std::vector<EACH_ELLIPSE> &Classify_EE)
{
	for (int j = 0; j < AEE_rough.size(); j++)
	{
		if (0 == Tag[j])
		{
			double dist = Distance(AEE_rough[i].center_x, AEE_rough[i].center_y, AEE_rough[j].center_x, AEE_rough[j].center_y);
			if (dist < 10*AEE_rough[i].long_axis)
			{
				Classify_EE.push_back(AEE_rough[j]);
				Tag[j] = 1;
				FindMinDistPoint(AEE_rough, Tag, j, Classify_EE);
			}
		}

	}
}

double Distance(double x1, double y1, double x2, double y2)
{
	double dist = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
	return dist;
}

void TraceEqualContourLine(IplImage* p_source_image_copy, std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_rough, std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_precise)
{
	FILE* p_contour_line_result = NULL;     
	errno_t err2;
	err2 = fopen_s( &p_contour_line_result, "result\\ContourLine.txt","w");
	if (err2)
	{
		printf("Cannot open the ContourLine.txt to save result\n");
		exit(1);
	}

	for (int i = 0; i < ACE_rough.size(); i++)
	{
		EACH_CLASSIFY_ELLIPSE ECE;
		ECE.num = i + 1;

		for (int h = 0; h < CLASSIFY_ELLIPSE_SIZE; h++)     
		{
			ECE.classify_code[h] = 0;
		}

		for (int j = 0; j < CLASSIFY_ELLIPSE_SIZE; j++)
		{

			int n_long_axis = (int)(ACE_rough[i].each_classify_ellipse[j].long_axis*2 + 0.5);
			int n_center_x = (int)(ACE_rough[i].each_classify_ellipse[j].center_x + 0.5);
			int n_center_y = (int)(ACE_rough[i].each_classify_ellipse[j].center_y + 0.5);

			CvRect bounding_ellipse = cvRect(n_center_x - n_long_axis, n_center_y - n_long_axis, n_long_axis*2, n_long_axis*2);
			cvSetImageROI(p_source_image_copy, bounding_ellipse);

			IplImage*	p_each_circle = cvCreateImage(cvSize(n_long_axis*2, n_long_axis*2), IPL_DEPTH_8U, 3);       //拷贝原图
			IplImage*	p_red_channel = cvCreateImage(cvGetSize(p_each_circle), IPL_DEPTH_8U, 1);        //原图红波段
			IplImage*	p_green_channel = cvCreateImage(cvGetSize(p_each_circle), IPL_DEPTH_8U, 1);      //原图绿波段
			IplImage*	p_blue_channel = cvCreateImage(cvGetSize(p_each_circle), IPL_DEPTH_8U, 1);       //原图蓝波段

			cvCopy(p_source_image_copy, p_each_circle, NULL);
			cvSplit(p_each_circle, p_blue_channel, p_green_channel, p_red_channel, 0);       //分别获取原图的RGB三个波段


			cvSmooth(p_red_channel, p_red_channel, CV_GAUSSIAN, 3, 3, 0, 0);
			cvSmooth(p_green_channel, p_green_channel, CV_GAUSSIAN, 3, 3, 0, 0);

			DSM* dsm = new DSM;
			EACH_ELLIPSE EE;
			
			//Find the suitable elevation Zk for the ellipse. 
			//And use this Zk to trance contour line
			int n_inter_x = ACE_rough[i].each_classify_ellipse[j].center_x + ACE_rough[i].each_classify_ellipse[j].long_axis*cos(ACE_rough[i].each_classify_ellipse[j].rotation_angle) - bounding_ellipse.x;
			int n_inter_y = ACE_rough[i].each_classify_ellipse[j].center_y + ACE_rough[i].each_classify_ellipse[j].long_axis*sin(ACE_rough[i].each_classify_ellipse[j].rotation_angle) - bounding_ellipse.y;
			double Zk = 0;

			uchar* data_red = (uchar*)p_red_channel->imageData;
			uchar* data_green = (uchar*)p_green_channel->imageData;
			int n_step_red = p_red_channel->widthStep/sizeof(uchar);
			int n_step_green = p_green_channel->widthStep/sizeof(uchar);
			if (ACE_rough[i].each_classify_ellipse[j].ellipse_color == RED)
			{
				EE.ellipse_color = RED;
				Zk = data_red[n_inter_y*n_step_red + n_inter_x];
				dsm->LoadFromImg(p_red_channel);
			}
			else if (ACE_rough[i].each_classify_ellipse[j].ellipse_color == GREEN)
			{
				EE.ellipse_color = GREEN;
				Zk = data_green[n_inter_y*n_step_green + n_inter_x];
				dsm->LoadFromImg(p_green_channel);
			}
			else
			{
				printf("The color of ellipse is wrong\n");
				exit(1);
			}

			//trace contour line
			dsm->TraceCntOne(Zk, bounding_ellipse, p_contour_line_result);

			//filter the part has no contour line
			int n_num_countour_each_area = dsm->m_cnt.size();
			if (n_num_countour_each_area > 0)
			{
				for (int k= 0 ; k < n_num_countour_each_area; k++)
				{
					std::vector<double>	v_temple_parameter;
					EllipseParameterEstimate	cpEstimate(0.5);
					int	numForEstimate = MIN_PARAMETER_NUM;
					double	desiredProbabilityForNoOutliers = 0.99;
					double	maximalOutlierPercentage = 0.2;
					int n_num_point_each_contour = dsm->m_cnt[k]->cnt.size();
					short*	bestVotes = (short*)malloc(sizeof(short)*n_num_point_each_contour);

					//RANSAC algorithm: chose the most suitable 
					//points fron contour line to do least square 
					//to estimate ellipse parameter.
					//It can improve the precision of ellipse fitting
					double	usedData = Ransac<Double_Point, double>::compute(v_temple_parameter, &cpEstimate, 
						dsm->m_cnt[k]->cnt, numForEstimate, desiredProbabilityForNoOutliers, 
						maximalOutlierPercentage, bestVotes );

					//if the delta.x and delta.y are small enouth (3 pixel)
					//it means that this point is the center of a ellipse
					Double_Point delta;
					delta.x = abs(ACE_rough[i].each_classify_ellipse[j].center_x - v_temple_parameter[0]);
					delta.y = abs(ACE_rough[i].each_classify_ellipse[j].center_y - v_temple_parameter[1]);
					if (delta.x < 3 && delta.y < 3) 
					{
						EE.center_x = v_temple_parameter[0];
						EE.center_y = v_temple_parameter[1];
						EE.long_axis = v_temple_parameter[2];
						EE.short_axis = v_temple_parameter[3];
						EE.rotation_angle = v_temple_parameter[4];
						EE.residual = usedData;
					}
					free(bestVotes);
				}
			}
			ECE.each_classify_ellipse[j] = EE;
			cvResetImageROI(p_source_image_copy);
			cvReleaseImage(&p_each_circle);
			cvReleaseImage(&p_red_channel);
			cvReleaseImage(&p_green_channel);
			cvReleaseImage(&p_blue_channel);
		}
		ACE_precise.push_back(ECE);
	}
	fclose(p_contour_line_result);
}

void EncodeClassify(std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_precise, std::vector<EACH_CLASSIFY_ELLIPSE> &ACE_precise_encode)
{
	int num_classify = 1;      
	for (int i = 0; i < ACE_precise.size(); i++)
	{
		EACH_CLASSIFY_ELLIPSE ECE;
		ECE.num = num_classify;

		Double_Point barycenter = {0, 0};
		for (int j = 0; j < CLASSIFY_ELLIPSE_SIZE; j++)
		{
			barycenter.x += ACE_precise[i].each_classify_ellipse[j].center_x/5.0;
			barycenter.y += ACE_precise[i].each_classify_ellipse[j].center_y/5.0;
		}

		double dist[CLASSIFY_ELLIPSE_SIZE] = {0};
		for (int j = 0; j < CLASSIFY_ELLIPSE_SIZE; j++)
		{
			dist[j] = Distance(ACE_precise[i].each_classify_ellipse[j].center_x, ACE_precise[i].each_classify_ellipse[j].center_y, barycenter.x, barycenter.y);
		}

		int index1[2] = {0, 0};               
		double dist_min1, dist_min2;
		dist_min1 = INT_MAX;	dist_min2 = INT_MAX;
		for (int j = 0; j < CLASSIFY_ELLIPSE_SIZE; j++)        
		{
			if (dist[j] < dist_min1)
			{
				dist_min2 = dist_min1;
				dist_min1 = dist[j];
				index1[1] = index1[0];
				index1[0] = j;
			}

			if (dist[j] > dist_min1 && dist[j] < dist_min2)
			{
				dist_min2 = dist[j];
				index1[1] = j;
			}
		}

		ECE.each_classify_ellipse[3] = ACE_precise[i].each_classify_ellipse[index1[0]];
		ECE.classify_code[3] = ACE_precise[i].each_classify_ellipse[index1[0]].ellipse_color;
		ECE.each_classify_ellipse[1] = ACE_precise[i].each_classify_ellipse[index1[1]];
		ECE.classify_code[1] = ACE_precise[i].each_classify_ellipse[index1[1]].ellipse_color;

		int index2 = 0;   

		for (int j = 0; j < CLASSIFY_ELLIPSE_SIZE; j++)
		{
			double dist2[2] = {0};
			if (j != index1[0] && j != index1[1])
			{
				dist2[0] = Distance(ACE_precise[i].each_classify_ellipse[j].center_x, ACE_precise[i].each_classify_ellipse[j].center_y, ECE.each_classify_ellipse[1].center_x, ECE.each_classify_ellipse[1].center_y);
				dist2[1] = Distance(ACE_precise[i].each_classify_ellipse[j].center_x, ACE_precise[i].each_classify_ellipse[j].center_y, ECE.each_classify_ellipse[3].center_x, ECE.each_classify_ellipse[3].center_y);
				
				if (dist2[0] > dist2[1])
				{
					index2 = j;
					ECE.each_classify_ellipse[4] = ACE_precise[i].each_classify_ellipse[j];
					ECE.classify_code[4] = ACE_precise[i].each_classify_ellipse[j].ellipse_color;
					break;
				}
			}
		}

		for (int j = 0; j < CLASSIFY_ELLIPSE_SIZE; j++)
		{
			double cross_product[2] = {0};
			if (j != index1[0] && j != index1[1] && j != index2)
			{
				double vec1_x, vec1_y, vec2_x, vec2_y;
				vec1_x = ACE_precise[i].each_classify_ellipse[j].center_x - ECE.each_classify_ellipse[1].center_x;
				vec1_y = ACE_precise[i].each_classify_ellipse[j].center_y - ECE.each_classify_ellipse[1].center_y;
				vec2_x = ACE_precise[i].each_classify_ellipse[j].center_x - ECE.each_classify_ellipse[3].center_x;
				vec2_y = ACE_precise[i].each_classify_ellipse[j].center_y - ECE.each_classify_ellipse[3].center_y;

				if (vec1_x*vec2_y - vec2_x*vec1_y > 0)
				{
					ECE.each_classify_ellipse[0] = ACE_precise[i].each_classify_ellipse[j];
					ECE.classify_code[0] = ACE_precise[i].each_classify_ellipse[j].ellipse_color;
				}
				else if (vec1_x*vec2_y - vec2_x*vec1_y < 0)
				{
					ECE.each_classify_ellipse[2] = ACE_precise[i].each_classify_ellipse[j];
					ECE.classify_code[2] = ACE_precise[i].each_classify_ellipse[j].ellipse_color;
				}
			}
		}
		ACE_precise_encode.push_back(ECE);
		++num_classify;
	}
}