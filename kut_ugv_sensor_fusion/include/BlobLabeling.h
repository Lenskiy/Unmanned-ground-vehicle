////////////////////////////////////////////////////
//
//  클래스 : CBlobLabeling
//
//                    by 마틴(http://martinblog.net)
#pragma once

//#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#define FALSE 0
#define TRUE 1
typedef struct 
{
	bool	bVisitedFlag;
	CvPoint ptReturnPoint;
} Visited;

class  CBlobLabeling
{
public:
	CBlobLabeling(void);
public:
	~CBlobLabeling(void);

public:
	cv::Mat		m_Image;				// 레이블링을 위한 이미지
	int			m_nThreshold;			// 레이블링 스레스홀드 값
	Visited*	m_vPoint;				// 레이블링시 방문정보
	int			m_nBlobs;				// 레이블의 갯수
	CvRect*		m_recBlobs;				// 각 레이블 정보


public:
	// 레이블링 이미지 선택
	void		SetParam(cv::Mat image, int nThreshold);

	// 레이블링(실행)
	void		DoLabeling();

private:
	// 레이블링(동작)
	int		 Labeling(cv::Mat image, int nThreshold);

	// 포인트 초기화
	void	 InitvPoint(int nWidth, int nHeight);
	void	 DeletevPoint();

	// 레이블링 결과 얻기
	void	 DetectLabelingRegion(int nLabelNumber, unsigned char *DataBuf, int nWidth, int nHeight);

	// 레이블링(실제 알고리즘)
	int		_Labeling(unsigned char *DataBuf, int nWidth, int nHeight, int nThreshold);
	
	// _Labling 내부 사용 함수
	int		__NRFIndNeighbor(unsigned char *DataBuf, int nWidth, int nHeight, int nPosX, int nPosY, int *StartX, int *StartY, int *EndX, int *EndY );
	int		__Area(unsigned char *DataBuf, int StartX, int StartY, int EndX, int EndY, int nWidth, int nLevel);
};