#include <malloc.h>

#include "RANSAC.h"

class EllipseParameterEstimate : public ParameterEsitmator<Double_Point , double>
{
public:
	EllipseParameterEstimate(double delta);
	~EllipseParameterEstimate();

	/**
	* Compute the line defined by the given data points.
	* @param data A vector containing two 2D points.
	* @param This vector is cleared and then filled with the computed parameters.
	*        The parameters of the line passing through these points [n_x,n_y,a_x,a_y]
	*        where ||(n_x,ny)|| = 1.
	*        If the vector contains less than two points then the resulting parameters
	*        vector is empty (size = 0).
	*/
	virtual void estimate(std::vector<Double_Point *> &data, std::vector<double> &parameters);

	/**
	* Compute a least squares estimate of the plane defined by the given points.
	* This implementation is of an orthogonal least squares error.
	*
	* @param data The line should minimize the least squares error to these points.
	* @param parameters This vector is cleared and then filled with the computed parameters.
	*                   Fill this vector with the computed line parameters [n_x,n_y,n_z.a_x,a_y,a_z], nrm and mean pt
	*                   where ||(n_x,ny, nz)|| = 1.
	*                   If the vector contains less than three points then the resulting parameters
	*                   vector is empty (size = 0).
	*/
	virtual void leastSquaresEstimate(std::vector<Double_Point *> &data, std::vector<double> &parameters);

	/**
	* Return true if the distance between the line defined by the parameters and the
	* given point is smaller than 'delta' (see constructor).
	* @param parameters The line parameters [n_x,n_y,a_x,a_y].
	* @param data Check that the distance between this point and the line is smaller than 'delta'.
	*/
	virtual bool agree(std::vector<double> &parameters, Double_Point &data);

	//求取中误差
	virtual double medianResidual( std::vector<Double_Point *> &data, std::vector<double> &parameters );

private:
	double m_deltaSquared; //判断点是否在椭圆上的阀值，利用点到两焦点距离和为2a
};