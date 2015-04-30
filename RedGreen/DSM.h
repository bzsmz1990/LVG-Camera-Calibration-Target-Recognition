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
	// ��BMP�ж�ȡDSM�ļ�
	bool LoadFromImg(IplImage* img);	

	//-------------------------------------------------------------
	// ASC �ļ�������
	// ASC �ļ�ͷ
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
	int    *m_Level;		//��עÿһ����������һ����Ρ���m_ncols*m_nrows����

	///auxdata
	int    m_LayerNum;
	float  m_minvalue;	//�洢���е����е���͵�
	float  m_maxvalue;	//�洢���е����е���ߵ�
	float  m_fZoom;
	//-------------------------------------------------------------	


	///contour trace
	std::vector<ContourVec*> m_cnt;
	void TraceCnt(float dZ, CvRect bounding_ellipse, FILE* contour_point_txt);
	void TraceCntOne(float Zk, CvRect bounding_ellipse, FILE* contour_point_txt);
	void TraceH(FILE* point_txt, BYTE* Hs,BYTE* Vs,float Zk, CvRect bounding_ellipse);
	void TraceV(FILE* point_txt, BYTE* Hs,BYTE* Vs,float Zk, CvRect bounding_ellipse);
	//void TraceCntofRgn(int i);//���ٵ�i������ı�Ե
	//void TraceH(BYTE* Hs,BYTE* Vs,float Zk,int si,int sj,std::vector<Double_Point> &chain);
	//void TraceV(BYTE* Hs,BYTE* Vs,float Zk,int si,int sj,std::vector<Double_Point> &chain);

};