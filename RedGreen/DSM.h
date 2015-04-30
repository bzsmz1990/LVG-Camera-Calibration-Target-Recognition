typedef unsigned int BYTE;
#define Min_Contour 35

struct ContourVec
{
	std::vector<Double_Point> cnt;
	double attr;
	BYTE isClose;
};


class DSM  
{
public:
	DSM();
	virtual ~DSM();
	void Init();
	void Free();
public:	
	// 从BMP中读取DSM文件
	bool LoadFromImg(IplImage* img);	

	//-------------------------------------------------------------
	// ASC 文件的描述
	// ASC 文件头
	int	m_ncols;
	int	m_nrows;
	double m_xllcorner;
	double m_yllcorner;     
	float  m_cellsize;
	double m_NODATA_value;

	///data
	float  *m_lpData; 
	float  *m_lpSrcData;
	float  *m_lpTempData;
	int    *m_Level;		//标注每一个点属于哪一个层次。共m_ncols*m_nrows个。

	///auxdata
	int    m_LayerNum;
	float  m_minvalue;	//存储所有点云中的最低点
	float  m_maxvalue;	//存储所有点云中的最高点
	float  m_fZoom;
	//-------------------------------------------------------------	


	///contour trace
	std::vector<ContourVec*> m_cnt;
	void TraceCnt(float dZ, CvRect bounding_ellipse, FILE* contour_point_txt);
	void TraceCntOne(float Zk, CvRect bounding_ellipse, FILE* contour_point_txt);
	void TraceH(FILE* point_txt, BYTE* Hs,BYTE* Vs,float Zk, CvRect bounding_ellipse);
	void TraceV(FILE* point_txt, BYTE* Hs,BYTE* Vs,float Zk, CvRect bounding_ellipse);
	//void TraceCntofRgn(int i);//跟踪第i个区域的边缘
	//void TraceH(BYTE* Hs,BYTE* Vs,float Zk,int si,int sj,std::vector<Double_Point> &chain);
	//void TraceV(BYTE* Hs,BYTE* Vs,float Zk,int si,int sj,std::vector<Double_Point> &chain);

};