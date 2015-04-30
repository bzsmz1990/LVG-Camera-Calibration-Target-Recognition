#include "stdafx.h"
#include "DSM.h"

DSM::DSM()
{
	Init();
}

DSM::~DSM()
{ 	
	Free();
}
void DSM::Init()
{
	m_ncols=0;         
	m_nrows=0;         
	m_xllcorner=0;
	m_yllcorner=0;     
	m_cellsize=0;
	m_NODATA_value=0;
	///
	m_lpData = NULL; 
	m_lpSrcData = NULL;
	m_Level  = NULL; 
	m_LayerNum = 0;	
	m_fZoom = 1.0;
}
void DSM::Free()
{
	if(m_lpData)
	{
		delete[] m_lpData;
		m_lpData = NULL;
	}   
	if(m_lpSrcData)
	{
		delete[] m_lpSrcData;
		m_lpSrcData = NULL;
	}
	if(m_Level)
		delete m_Level;m_Level=NULL;
	m_cnt.clear();
}

bool DSM::LoadFromImg(IplImage* img)
{

	if(m_lpData) delete[] m_lpData;	m_lpData = NULL;
	m_ncols = img->height;
	m_nrows = img->width;
	m_xllcorner = 0;
	m_yllcorner = 0;
	m_cellsize  =1.0;
	m_NODATA_value = -99999.0;

	m_lpData=new float[m_ncols*m_nrows];
	///save the original DATA
	if(m_lpSrcData) delete[] m_lpSrcData;	m_lpSrcData = NULL;
	m_lpSrcData=new float[m_ncols*m_nrows];

	uchar* data = (uchar*)img->imageData;
	int step = img->widthStep/sizeof(uchar);
	for (int i=0; i < m_nrows; i++)
	{
		for (int j=0; j < m_ncols; j++)
		{
			m_lpData[i*m_ncols + j]=(float)data[i*step + j];
		}
	}
	if(m_lpSrcData) delete[] m_lpSrcData;	m_lpSrcData = NULL;
	m_lpSrcData=new float[m_ncols*m_nrows];
	memcpy(m_lpSrcData,m_lpData,m_ncols*m_nrows*sizeof(float));


	float* lpData=m_lpData;
	float minvalue = 9999;
	float maxvalue = -9999;
	for(int i=0; i < m_nrows; i++)
	{
		for(int j=0; j < m_ncols; j++)
		{
			if(*(lpData) > maxvalue)
				maxvalue = *(lpData);
			if(*(lpData) < minvalue)
				minvalue = *(lpData);

			lpData++;
		}
	}
	m_maxvalue = maxvalue;
	m_minvalue = minvalue;

	return TRUE;
}

//////not need start point
void DSM::TraceH(FILE* point_txt, BYTE* Hs, BYTE* Vs, float Zk, CvRect bounding_ellipse)
{
	for(int i = 3; i < m_nrows - 3; i++)         
	{
		for(int j = 3; j < m_ncols - 3; j++)     
		{
			if(Hs[i*m_ncols + j])
			{			
				ContourVec* theContour = new ContourVec;     
				theContour->attr = Zk;
				theContour->isClose = 0;

				float x1 = j*m_cellsize;
				float y1 = i*m_cellsize;
				float Z0 = m_lpData[i*m_ncols + j];
				float Z1 = m_lpData[i*m_ncols + j + 1];
				Double_Point pt;
				double mid = 0;
				pt.x = x1  + (Zk - Z0)/(Z1 - Z0)* m_cellsize + bounding_ellipse.x;
				pt.y = y1 + bounding_ellipse.y;

				theContour->cnt.push_back(pt);

				Double_Point outpos;
				outpos.x = j;
				outpos.y = i;
				int in = 1;
				int out = 0;
				
				while(1)
				{
					int xin = outpos.x;
					int yin = outpos.y;
					
					switch(in) 
					{
					case 1:
						{
							if(Vs[yin*m_ncols + xin] == 1)
							{
								out = 2;
								Vs[yin*m_ncols + xin] = 0;
								outpos.x = xin - 1;
								outpos.y = yin;
								in = 4;
								break;
							}

							if(Hs[(yin + 1)*m_ncols + xin] == 1)
							{
								out = 3;
								Hs[(yin + 1)*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin + 1;
								in = 1;
								break;
							}

							if(Vs[yin*m_ncols + xin + 1] == 1)
							{
								out = 4;
								Vs[yin*m_ncols + xin + 1] = 0;
								outpos.x = xin + 1;
								outpos.y = yin;
								in = 2;
								break;
							}

							out=0;							 
							break;
						}
					case 2:
						{
							if(Hs[(yin + 1)*m_ncols + xin] == 1)
							{
								out = 3;
								Hs[(yin + 1)*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin + 1;
								in = 1;
								break;
							}
							if(Vs[yin*m_ncols + xin + 1] == 1)
							{
								out = 4;
								Vs[yin*m_ncols + xin + 1] = 0;
								outpos.x = xin + 1;
								outpos.y = yin;
								in = 2;
								break;
							}
							if(Hs[yin*m_ncols + xin] == 1)
							{
								out = 1;
								Hs[yin*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin - 1;
								in = 3;
								break;
							}

							out=0;							 
							break;
						}
					case 3:
						{
							if(Vs[yin*m_ncols + xin + 1] == 1)
							{
								out = 4;
								Vs[yin*m_ncols + xin + 1] = 0;
								outpos.x = xin + 1;
								outpos.y = yin;
								in = 2;
								break;
							}
							if(Hs[yin*m_ncols + xin] == 1)
							{
								out = 1;
								Hs[yin*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin - 1;
								in = 3;
								break;
							}
							if(Vs[yin*m_ncols + xin] == 1)
							{
								out = 2;
								Vs[yin*m_ncols + xin] = 0;
								outpos.x = xin - 1;
								outpos.y = yin;
								in = 4;
								break;
							}

							out=0;							 
							break;
						}
					case 4:
						{
							if(Hs[yin*m_ncols + xin] == 1)
							{
								out = 1;
								Hs[yin*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin - 1;
								in =3;
								break;
							}
							if(Vs[yin*m_ncols + xin] == 1)
							{
								out = 2;
								Vs[yin*m_ncols + xin] = 0;
								outpos.x = xin - 1;
								outpos.y = yin;
								in = 4;
								break;
							}
							if(Hs[(yin+1)*m_ncols + xin] == 1)
							{
								out = 3;
								Hs[(yin + 1)*m_ncols + xin]=0;
								outpos.x = xin;
								outpos.y = yin + 1;
								in = 1;
								break;
							}
							out=0;							 
							break;
						}
					}
					
					if(out == 0) 
						break;
					int jj = outpos.x;
					int ii = outpos.y;
					if((jj < 1)||(jj >= m_ncols - 1)||(ii < 1)||(ii >= m_nrows - 1))
						break;

					if(in == 1||in == 3)
					{
						if(ii == i && jj == j) 
						{
							Hs[i*m_ncols + j] = 0; 
							theContour->isClose = 1;
							break;
						}
						if(in == 1)
						{
							x1 = jj*m_cellsize;
							y1 = ii*m_cellsize;
							Z0 = m_lpData[ii*m_ncols + jj];
							Z1 = m_lpData[ii*m_ncols + jj + 1];
						}
						else
						{
							x1 = jj*m_cellsize;
							y1 = (ii+1)*m_cellsize;
							Z0 = m_lpData[(ii + 1)*m_ncols + jj];
							Z1 = m_lpData[(ii + 1)*m_ncols + jj + 1];
						}

						pt.x = x1  + (Zk - Z0)/(Z1 - Z0)* m_cellsize + bounding_ellipse.x;
						pt.y = y1 + bounding_ellipse.y;
					}
					else if(in == 2 || in == 4)
					{
						if(in == 2)
						{
							x1 = jj*m_cellsize;
							y1 = ii*m_cellsize;	
							Z0 = m_lpData[ii*m_ncols + jj];
							Z1 = m_lpData[(ii + 1)*m_ncols + jj];
						}
						else
						{
							x1 = (jj + 1)*m_cellsize;
							y1 = ii*m_cellsize;
							Z0 = m_lpData[ii*m_ncols + jj + 1];
							Z1 = m_lpData[(ii + 1)*m_ncols + jj + 1];
						}
						pt.x = x1 + bounding_ellipse.x;
						pt.y = y1 + (Zk - Z0)/(Z1 - Z0)* m_cellsize + bounding_ellipse.y;
					}
					/*pt.x += bounding_ellipse.x;
					pt.y += bounding_ellipse.y;*/
					theContour->cnt.push_back(pt);
				}

				if (theContour->isClose && theContour->cnt.size() > Min_Contour)
				{
					for (int i = 0; i < theContour->cnt.size(); i++)
					{
						fprintf(point_txt, "%f\t%f\t%f\t\n", (theContour->cnt)[i].x, (theContour->cnt)[i].y, theContour->attr);
					}
					m_cnt.push_back(theContour);
				}

				
			}
		}
	}
}

//////not need start point
void DSM::TraceV(FILE* point_txt, BYTE* Hs,BYTE* Vs,float Zk, CvRect bounding_ellipse)
{
	
	for(int i = 3; i < m_nrows - 3; i++)
		for(int j = 3; j < m_ncols - 3; j++)
		{
			if(Vs[i*m_ncols + j])
			{
				ContourVec* theContour = new ContourVec;     
				theContour->attr = Zk;
				theContour->isClose = 0;

				float x1 = j*m_cellsize;
				float y1 = i*m_cellsize;
				float Z0 = m_lpData[i*m_ncols + j];
				float Z1 = m_lpData[(i + 1)*m_ncols + j];
				Double_Point pt;
				double mid = 0;
				//pt.x = x1 + (Zk - Z0)/(Z1 - Z0)* m_cellsize;
				//pt.y = y1;
				pt.x = x1 + bounding_ellipse.x;
				pt.y = y1 + (Zk - Z0)/(Z1 - Z0)* m_cellsize + bounding_ellipse.y;

				theContour->cnt.push_back(pt);

				
				Double_Point outpos;
				outpos.x = j;
				outpos.y = i;
				int in =2;
				int out=0;

				while(1)
				{
					int xin = outpos.x;
					int yin = outpos.y;

					switch(in) 
					{
					case 1:
						{

							if(Vs[yin*m_ncols + xin] == 1)
							{
								out =2;
								Vs[yin*m_ncols + xin] = 0;
								outpos.x = xin - 1;
								outpos.y = yin;
								in = 4;
								break;
							}

							if(Hs[(yin+1)*m_ncols + xin] == 1)
							{
								out = 3;
								Hs[(yin+1)*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin + 1;
								in = 1;
								break;
							}

							if(Vs[yin*m_ncols + xin + 1] == 1)
							{
								out = 4;
								Vs[yin*m_ncols + xin + 1] = 0;
								outpos.x = xin + 1;
								outpos.y = yin;
								in = 2;
								break;
							}

							out=0;							 
							break;
						}
					case 2:
						{
							if(Hs[(yin+1)*m_ncols + xin] == 1)
							{
								out = 3;
								Hs[(yin + 1)*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin + 1;
								in = 1;
								break;
							}
							if(Vs[yin*m_ncols + xin + 1] == 1)
							{
								out = 4;
								Vs[yin*m_ncols + xin + 1] = 0;
								outpos.x = xin + 1;
								outpos.y = yin;
								in = 2;
								break;
							}
							if(Hs[yin*m_ncols + xin] == 1)
							{
								out = 1;
								Hs[yin*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin - 1;
								in = 3;
								break;
							}

							out=0;							 
							break;
						}
					case 3:
						{
							if(Vs[yin*m_ncols + xin + 1] == 1)
							{
								out = 4;
								Vs[yin*m_ncols + xin + 1] = 0;
								outpos.x = xin + 1;
								outpos.y = yin;
								in = 2;
								break;
							}
							if(Hs[yin*m_ncols + xin] == 1)
							{
								out = 1;
								Hs[yin*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin - 1;
								in = 3;
								break;
							}
							if(Vs[yin*m_ncols + xin] == 1)
							{
								out = 2;
								Vs[yin*m_ncols + xin] = 0;
								outpos.x = xin - 1;
								outpos.y = yin;
								in = 4;
								break;
							}

							out = 0;							 
							break;
						}
					case 4:
						{
							if(Hs[yin*m_ncols + xin] == 1)
							{
								out = 1;
								Hs[yin*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin - 1;
								in =3;
								break;
							}
							if(Vs[yin*m_ncols + xin] == 1)
							{
								out = 2;
								Vs[yin*m_ncols + xin] = 0;
								outpos.x = xin - 1;
								outpos.y = yin;
								in = 4;
								break;
							}
							if(Hs[(yin+1)*m_ncols + xin] == 1)
							{
								out = 3;
								Hs[(yin + 1)*m_ncols + xin] = 0;
								outpos.x = xin;
								outpos.y = yin + 1;
								in = 1;
								break;
							}
							out=0;							 
							break;
						}

					}

					if(out == 0) 
						break;
					int jj = outpos.x;
					int ii = outpos.y;

					if((jj < 1) || (jj >= m_ncols - 1) || (ii < 1) || (ii >= m_nrows - 1))
						break;

					if(in == 1 || in == 3)
					{
						if(in == 1)
						{
							x1 = jj*m_cellsize;
							y1 = ii*m_cellsize;
							Z0 = m_lpData[ii*m_ncols + jj];
							Z1 = m_lpData[ii*m_ncols + jj + 1];
						}else
						{
							x1 = jj*m_cellsize;
							y1 = (ii + 1)*m_cellsize;
							Z0 = m_lpData[(ii + 1)*m_ncols + jj];
							Z1 = m_lpData[(ii + 1)*m_ncols + jj + 1];
						}

						pt.x = x1  + (Zk - Z0)/(Z1 - Z0)* m_cellsize + bounding_ellipse.x;
						pt.y = y1 + bounding_ellipse.y;

					}
					else if(in == 2 || in == 4)
					{
						if(ii == i && jj == j) 
						{
							Vs[i*m_ncols + j]=0;
							theContour->isClose = 1;
							break;
						}
						if(in==2)
						{
							x1 = jj*m_cellsize;
							y1 = ii*m_cellsize;	
							Z0 = m_lpData[ii*m_ncols + jj];
							Z1 = m_lpData[(ii + 1)*m_ncols + jj];
						}
						else
						{
							x1 = (jj + 1)*m_cellsize;
							y1 = ii*m_cellsize;
							Z0 = m_lpData[ii*m_ncols + jj + 1];
							Z1 = m_lpData[(ii + 1)*m_ncols + jj + 1];
						}

						pt.x = x1 + bounding_ellipse.x;
						pt.y = y1 + (Zk - Z0)/(Z1 - Z0)* m_cellsize + bounding_ellipse.y;

					}
					
					theContour->cnt.push_back(pt);
				}

				if (theContour->isClose && theContour->cnt.size() > Min_Contour)
				{
					for (int i = 0; i < theContour->cnt.size(); i++)
					{
						fprintf(point_txt, "%f\t%f\t%f\t\n", (theContour->cnt)[i].x, (theContour->cnt)[i].y, theContour->attr);
					}
					m_cnt.push_back(theContour);
				}

				
			}
		}
}

void DSM::TraceCnt(float dZ, CvRect bounding_ellipse, FILE* contour_point_txt)
{
	m_cnt.clear();

	BYTE*  Hs = new BYTE[m_ncols*m_nrows];
	BYTE*  Vs = new BYTE[m_ncols*m_nrows];

	float Zmin = (int)(m_minvalue/dZ + 1)*dZ;
	float Zmax = (int)(m_maxvalue/dZ)*dZ;
	int f = (Zmax - Zmin)/dZ;

	for(int k = 0; k <= f; k++)
	{   

		memset(Hs, 0, m_ncols*m_nrows);
		memset(Vs, 0, m_ncols*m_nrows);
		float Zk = Zmin + k*dZ;

		for(int i = 1; i < m_nrows - 1; i++)
		{
			for(int j = 1; j < m_ncols - 1; j++)
			{
				if((m_lpData[i*m_ncols + j] + 0.00000001 - Zk)*(m_lpData[i*m_ncols + j + 1] + 0.00000001 - Zk) < 0)
					Hs[i*m_ncols + j] = 1;
				else 
					Hs[i*m_ncols + j] = 0;

				if((m_lpData[i*m_ncols + j] + 0.00000001 - Zk)*(m_lpData[(i + 1)*m_ncols + j] + 0.00000001 - Zk) < 0)
					Vs[i*m_ncols + j] = 1;
				else 
					Vs[i*m_ncols + j] = 0;

			}
		}
		TraceH(contour_point_txt, Hs, Vs, Zk, bounding_ellipse);
		TraceV(contour_point_txt, Hs, Vs, Zk, bounding_ellipse);

	}
	delete[] Hs;
	delete[] Vs;

}


void DSM::TraceCntOne(float Zk, CvRect bounding_ellipse, FILE* contour_point_txt)
{
	m_cnt.clear();


	BYTE*  Hs = new BYTE[m_ncols*m_nrows];
	BYTE*  Vs = new BYTE[m_ncols*m_nrows];

	memset(Hs, 0, m_ncols*m_nrows);
	memset(Vs, 0, m_ncols*m_nrows);

	for(int i = 1; i < m_nrows - 1; i++)
	{
		for(int j = 1; j < m_ncols - 1; j++)
		{
			if((m_lpData[i*m_ncols + j] + 0.00000001 - Zk)*(m_lpData[i*m_ncols + j + 1] + 0.00000001 - Zk) < 0)
				Hs[i*m_ncols + j] = 1;
			else 
				Hs[i*m_ncols + j] = 0;

			if((m_lpData[i*m_ncols + j] + 0.00000001 - Zk)*(m_lpData[(i + 1)*m_ncols + j] + 0.00000001 - Zk) < 0)
				Vs[i*m_ncols + j] = 1;
			else 
				Vs[i*m_ncols + j] = 0;

		}
	}

	TraceH(contour_point_txt, Hs, Vs, Zk, bounding_ellipse);
	TraceV(contour_point_txt, Hs, Vs, Zk, bounding_ellipse);

	delete[] Hs;
	delete[] Vs;

}