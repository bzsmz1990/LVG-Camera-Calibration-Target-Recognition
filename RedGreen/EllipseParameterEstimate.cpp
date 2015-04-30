#include "stdafx.h"
#include <math.h>
#include "matlib.h"
#include <limits>

#include "EllipseParameterEstimate.h"

#define MIN_PARAMETER_NUM 6
#define PI	3.14159265358979

EllipseParameterEstimate::EllipseParameterEstimate(double delta) : m_deltaSquared(delta * delta) {}


EllipseParameterEstimate::~EllipseParameterEstimate()
{
}


void EllipseParameterEstimate::estimate(std::vector<Double_Point*> &data, std::vector<double> &parameters)
{
	leastSquaresEstimate(data, parameters);
}

//use least squares estimate ellipse parameter.
void EllipseParameterEstimate::leastSquaresEstimate(std::vector<Double_Point*> &data, std::vector<double> &parameters)
{
	parameters.clear();
	if(data.size()<MIN_PARAMETER_NUM)
		return;

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
		X_point.r(i+1, 1) = (double)data[i]->x;
		Y_point.r(i+1, 1) = (double)data[i]->y;

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
	//display(design_matrix);

	dMm(scatter_matrix);                               //Design matrix
	scatter_matrix = zeros(MIN_PARAMETER_NUM, MIN_PARAMETER_NUM);
	scatter_matrix = transpose(design_matrix)*design_matrix;
	//display(scatter_matrix);

	dMm(constraint_matrix);
	constraint_matrix = zeros(MIN_PARAMETER_NUM, MIN_PARAMETER_NUM);
	constraint_matrix.r(1, 3) = -2;
	constraint_matrix.r(2, 2) = 1;
	constraint_matrix.r(3, 1) = -2;
	//display(constraint_matrix);

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
			tmpA.r(i+1, j+1) = scatter_matrix.r(i+1, j+1);
			tmpB.r(i+1, j+1) = scatter_matrix.r(i+1, j+4);
			tmpC.r(i+1, j+1) = scatter_matrix.r(i+4, j+4);
			tmpD.r(i+1, j+1) = constraint_matrix.r(i+1, j+1);
		}
	}
	tmpE = inv(tmpC)*transpose(tmpB);

	dMm(evec_x);
	dMm(eval_x);
	eig(inv(tmpD)*(tmpA - tmpB*tmpE), i_o, evec_x, eval_x);

	int n_index = 0;
	for (int i=0; i < 3; i++)
	{
		if (eval_x.r(i+1, i+1) <1e-8 && eval_x.r(i+1, i+1) < DBL_MAX && eval_x.r(i+1, i+1) > -DBL_MAX)
		{
			n_index = i+1;
		}
	}

	dMm(A_half_matrix);
	A_half_matrix = zeros(3, 1);
	for (int i=0; i < 3; i++)
	{
		A_half_matrix.r(i+1, 1) = evec_x.r(i+1, n_index);
	}

	dMm(evec_y);
	evec_y = -tmpE * A_half_matrix;

	dMm(A_matrix);
	A_matrix = zeros(MIN_PARAMETER_NUM, 1);
	for (int i=0; i < MIN_PARAMETER_NUM; i++)
	{
		if (i<3)
			A_matrix.r(i+1, 1) = A_half_matrix.r(i+1, 1);
		else
			A_matrix.r(i+1, 1) = evec_y.r(i-2, 1);
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
		if (d_thetarad > PI/2.0 ) 
			d_thetarad -= PI/2.0;
		else if (d_thetarad > 0 && d_thetarad < PI/2.0)
			d_thetarad += PI/2.0;
		else if (d_thetarad < 0 && d_thetarad > -PI/2.0)
			d_thetarad += PI/2.0;
		else if (d_thetarad < -PI/2.0)
			d_thetarad += 1.5*PI;
	}

	if (d_thetarad < 0) d_thetarad += PI;           

	
	parameters.push_back(d_uCentre);
	parameters.push_back(d_vCentre);
	parameters.push_back(d_Ru);
	parameters.push_back(d_Rv);
	parameters.push_back(d_thetarad);

	exitM();
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}


bool EllipseParameterEstimate::agree(std::vector<double> &parameters, Double_Point &data)
{
	double d_a = parameters[2];           
	double d_b = parameters[3];           
	double d_c = sqrt(d_a*d_a - d_b*d_b);   
	double d_eclipse_angle = parameters[4];

	double focus1_x, focus1_y, focus2_x, focus2_y;

	focus1_x = parameters[0] + cos(d_eclipse_angle)*d_c;
	focus1_y = parameters[1] + sin(d_eclipse_angle)*d_c;
	focus2_x = 2*parameters[0] - focus1_x;
	focus2_y = 2*parameters[1] - focus1_y;

	double d_signedDistance = 0;
	d_signedDistance = sqrt((data.x - focus1_x)*(data.x - focus1_x) + (data.y - focus1_y)*(data.y - focus1_y)) + 
		sqrt((data.x - focus2_x)*(data.x - focus2_x) + (data.y - focus2_y)*(data.y - focus2_y)) - 2*d_a;

	return ( ( d_signedDistance * d_signedDistance ) < m_deltaSquared );
}


double EllipseParameterEstimate::medianResidual(std::vector<Double_Point*> &data, std::vector<double> &parameters)
{
	double d_a = parameters[2];           
	double d_b = parameters[3];          
	double d_c = sqrt(d_a*d_a - d_b*d_b);   
	double d_eclipse_angle = parameters[4];

	double focus1_x = parameters[0] + cos(d_eclipse_angle)*d_c;
	double focus1_y = parameters[1] + sin(d_eclipse_angle)*d_c;
	double focus2_x = 2*parameters[0] - focus1_x;
	double focus2_y = 2*parameters[1] - focus1_y;

	double d_residual = 0;
	int n_dataSize = data.size();

	for ( int i = 0; i < n_dataSize; i++ )
	{
		double d_dist;
		d_dist = sqrt((data[i]->x - focus1_x)*(data[i]->x - focus1_x) + (data[i]->y - focus1_y)*(data[i]->y - focus1_y)) + 
			sqrt((data[i]->x - focus2_x)*(data[i]->x - focus2_x) + (data[i]->y - focus2_y)*(data[i]->y - focus2_y)) - 2*d_a;
		d_residual += d_dist*d_dist;
	}

	return d_residual / n_dataSize;
}