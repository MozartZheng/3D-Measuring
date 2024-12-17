
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>
#include <stdarg.h>

using namespace std;

#define PI						3.1415926535

#define MAX_NUM_CAMERA_PARAM	20
#define MAX_FILE_PATH_LENGTH    512
#define MAX_POLY_PARA_NUM		512
#define MAX_DIST_PARA_NUM		64
#define MAX_LENGTH_POINT_NAME	64
#define MAX_CAMERA_LEAF_NUM		8

#define EPSION					10e-25

typedef struct POINT2D
{
	int nAttrib;
	int nImgID;
	int nCamID;
	int nPtsID;
	float x;
	float y;

	POINT2D()
	{
		nAttrib = nImgID = nCamID = nPtsID = 0;
		x = y = 0.0;
	}
}PT2D;
typedef struct POINT3D
{
	char name[MAX_LENGTH_POINT_NAME];
	double X;
	double Y;
	double Z;
	int nAttrib;
	int nIPtNum;
	int nIPtSID;

	POINT3D()
	{
		nAttrib = nIPtNum = nIPtSID = 0;
		X = Y = Z = 0.0;
		memset(name, 0, sizeof(char) * 32);
	}

}PT3D;

struct EOP
{
	int nAttrib;
	double t;
	double Xs;
	double Ys;
	double Zs;
	double Ph;
	double Om;
	double Kp;
	EOP(){
		nAttrib = 0;
		t = Xs = Ys = Zs = Ph = Om = Kp = 0.0;
	}
};

typedef struct CAMERA
{
	int nHeight;
	int nWidth;

	float pixelSize;
	float height;
	float width;
	float fx;
	float fy;
	float x0;
	float y0;
	float s;

	double k1;
	double k2;
	double k3;
	double p1;
	double p2;

	double s1;
	double s2;
	double p3;
	double k4;
	double k5;
	double k6;
	double k7;
	double k8;
	double k9;
	CAMERA()
	{
		height = width = 0.0;
		nHeight = nWidth = 0;
		fx = fy = 1;x0 = y0 = s = 0.0;
		k1 = k2 = k3 = p1 = p2 = p3 = 0.0;
		s1 = s2 = k4 = k5 = k6 = k7 = k8 = k9 = 0.0;
	}
}CAM;
typedef struct GEOCAMERA
{
	int leafNum;
	double gMatrix[12];
	double vectorRotate[9];
	double vectorPosition[3];
	CAM camPara;
	GEOCAMERA()
	{
		memset(gMatrix, 0, sizeof(double) * 12);
		memset(vectorRotate, 0, sizeof(double) * 9);
		memset(vectorPosition, 0, sizeof(double) * 3);
	}
}GCAM;
typedef struct SUBMATRIXROWINDEX
{
	__int64 sNum;
	std::vector<int> veID;
	SUBMATRIXROWINDEX()
	{
		sNum = 0;
	}
}SUBID;
typedef struct SUBMATRIX
{
	int sR;
	int sC;
	int nR;
	int nC;

	double * pData;
	SUBMATRIX()
	{
		sR = sC = nR = nC = 0;
		pData = NULL;
	}
}SMA;
struct CORR_IMG{
	int ImgID[2];
	int CamID[2];
	int nObjPtNum;
	std::vector <int> veIptID;
	CORR_IMG()
	{
		nObjPtNum = ImgID[0] = ImgID[1] = 0;
		CamID[0] = CamID[1] = 0;
	}
};
inline int ParseFilePath(char * strInput, char * strPath, char *strTitle, char *strExtension)
{
	if (strInput == NULL) {
		printf("File path is null!\n");
		return 0;
	}
	if (strcmp(strInput, "") == 0) {
		printf("File path is null!\n");
		return 0;
	}
	char strTmp[MAX_FILE_PATH_LENGTH] = { 0 };
	memcpy(strTmp, strInput, sizeof(char)*strlen(strInput));
	char *pS = strrchr(strTmp, '\n');
	if (pS) *pS = 0;
	pS = strrchr(strTmp, '\\');
	if (pS) {
		char *qS = strrchr(pS, '/');
		if (qS) {
			char *oS = strrchr(qS, '.');
			if (oS) {
				if (strExtension) {
					memcpy(strExtension, oS + 1, sizeof(char)*strlen(oS + 1));
				}
				*oS = 0;
				if (strTitle) {
					memcpy(strTitle, qS + 1, sizeof(char)*strlen(qS + 1));
				}
			}
			*qS = 0;
		}
		else {
			char *oS = strrchr(pS, '.');
			if (oS) {
				if (strExtension) {
					memcpy(strExtension, oS + 1, sizeof(char)*strlen(oS + 1));
				}
				*oS = 0;
				if (strTitle) {
					memcpy(strTitle, pS + 1, sizeof(char)*strlen(pS + 1));
				}
			}
			*pS = 0;
		}
	}
	else {
		char *qS = strrchr(strTmp, '/');
		if (qS) {
			char *oS = strrchr(qS, '.');
			if (oS) {
				if (strExtension) {

					memcpy(strExtension, oS + 1, sizeof(char)*strlen(oS + 1));
				}
				*oS = 0;
				if (strTitle) {
					memcpy(strTitle, qS + 1, sizeof(char)*strlen(qS + 1));
				}
			}
			*qS = 0;
		}
		else {
			printf_s("File path format is invalid!\n");
			return 0;
		}
	}
	if (strPath) {
		memcpy(strPath, strTmp, sizeof(char)*strlen(strTmp));
	}

	return 1;
}
inline double sampleNormal( double mean, double stdev ) 
{
	double u = ((double)rand() / (RAND_MAX)) * 2 - 1;
	double v = ((double)rand() / (RAND_MAX)) * 2 - 1;
	double r = u * u + v * v;
	if (r == 0 || r > 1) return sampleNormal(mean, stdev);
	double c = sqrt(-2 * log(r) / r);
	return u * c*stdev + mean;
}
