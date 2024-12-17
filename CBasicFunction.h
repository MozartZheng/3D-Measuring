#pragma once
#include "CXMatrix.h"
#include "StructDef.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

class CBasicFunction
{
public:
	CBasicFunction(void);
	~CBasicFunction(void);

public:
	int GetAngleWithRotateMatrix( double *pR, double &ph, double &om, double &kp );
	int GetRotateMatrixWithAngle( double *pR, double ph, double om, double kp );
	int GetRotateMatrixWithUnitQuaternion( double *pR, double q0, double qx, double qy, double qz );
	int GetUnitQuaternionWithRotateMatrix( double *pR, double &q0, double &qx, double &qy, double &qz );
	int GetAngleWithUnitQuaternion( double q0, double qx, double qy, double qz, double &ph, double &om, double &kp );
	int GetUnitQuaternionWithAngle( double &q0, double &qx, double &qy, double &qz, double ph, double om, double kp );
	int SolveCubicEquation(double a, double b, double c, double d, int &nNum, double *x);
	int GenerateGaussError(double mean, double sigma, double &err);

	bool ValidateDistortionParameters(CAM &cam, double ImgPtScale);
	int CorrectDistortion(bool bEstiRealDistort, double x_d, double y_d, CAM &cam, double &x_u, double &y_u);
	int ComputeDistortionWithUndistortedPoints_Normalized(double x_u, double y_u, CAM &cam, double &rx, double &ry);
	int ComputeDistortionWithDistortedPoints_Normalized(double x_d, double y_d, CAM &cam, double &rx, double &ry);
	int SearchForUndistortedCoordinates_Normalized(double x, double y, CAM &cam, double &nx, double &ny);
	int ComputeReprojErrorPerImgPt(PT3D &p3, PT2D * pP2, EOP * pE, CAM * pC, double &maxErr, int &maxErrID, bool bEstiRealDistort = true);

	int GenerateMiniRandomSample(unsigned int nSampleNum, unsigned int nMinSampleNum, unsigned int* pSampleID);
	int EvaluateModel(unsigned int nSampleNum, unsigned int nMinSampleNum, unsigned int* pSampleID, int nRow, double* pSrc, double* pDst, double InlierThreshold, unsigned int& nInlierCount, int* pMask);
	int UpdateStoppingCount(unsigned int nInlierCount, double conf_threshold, unsigned int nTotSample, unsigned int nSampleSize, unsigned int nMaxIteration);
	int ComputeSimilarity_RANSAC(int nRow, int nCol, double* pSrc, double* pDst, double inlierThreshold, double* pM, int* pMask);
	
	int MultiInterSection(PT3D &p3, PT2D * pP2, EOP * pE, CAM * pC, int CoorControl = 0, bool bEstiRealDistort = true);
	int MultiInterSection(int nPtNum, PT3D * p3, PT2D * p2, EOP * pE, CAM * pC, int CoorControl = 0, bool bEstiRealDistort = true);
	int MultiInterSection(int N, CAM *pCam, EOP *pEop, double *px, double *py, double& X, double& Y, double& Z, double &m0, double &mx, double &my, double &mz, int CoorControl = 0, bool bEstiRealDistort = true);
	int MultiInterSection_ProjectMatrix(int N, PT2D *p2, CAM *pCam, double *pM, double &X, double &Y, double &Z, double *m);
	int GetxyResWithXYZ(CAM &cam, EOP &eop, PT3D &p3, PT2D &p2, double &rx, double &ry, bool bEstiRealDistort = false);
	int GetxyWithXYZ(CAM &cam, EOP &eop, PT3D &p3, double &x, double &y);
	int GetXYWithxyZ(EOP &eop, CAM &cam, double x, double y, double Z, double &X, double &Y, bool bEstiRealDistort = false);

	int InverseMatrix(double* pData, int n);
	int InverseSparseMatrix( double* pData, int nSubNum, int *pSubWidth );
	int MultiMatrix( double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB );
	int TransMultiMatrix( double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB );
	int MultiTransMatrix( double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB );
	int MultiMatrix( double *pW, double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB );
	int TransMultiMatrix( double *pW, double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB );
	int MultiTransMatrix( double *pW, double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB );
	int UpdateNormalMatrix(double * pSub, int nSubR, int nSubC, int nBegR, int nBegC, double * pData, int nR, int nC);
	int UpdateAdvNormalMatrix(double * pSub, int nSubR, int nSubC, int nBegR, int nBegC, double * pData, int nR, int nC);

	int FindBenchmarkPoints(int nNum, int nDimension, double *pSrc, double *pDst);
	int FindCrossEigenVectors(int nNum, int nDimension, double *pSrcA, double *pSrcB, double *pEVectA, double *pEVectB);
	int NomarlizePointCoor(int nNum, int nDimension, double *pSrc, double *pDst, double *pT);
	int NomarlizePointCoor(int nNum, unsigned int* pSampleID, int nSampleNum, double* pSrc, double* pT);
	int NomarlizePointSetWithCenterPoint(int nNum, int nDimension, double *pSrc, int nCenter, double *pDst, double *pT);
	int ComputeMatrixScale(int nNum, double * pSrc, double *pDst, double &scale);
	int ComputeMatrixScale(double *r12, int aR, int  aC, double * pSrc, int bR, int bC, double *pDst, double &scale);
	int ComputeVectorDirection(int nWidth, int nHeight, double *pSrc, double *pDst, int *pDirection);

	int HouseholderTransformation(int nTransNum, unsigned int* pID, int nRow, int nCol, double* pSrc, double* pDstQ, double* pDstT);
	int HouseholderTransformation(int nRow, int nCol, double *pSrc, int &nTransNum, double *pDstQ, double *pDstT);
	int HouseholderTransformation(int nRow, int nDimension, double *pSrc, double *pDstH);
	int GetColumnVector(int index, int nRow, int nCol, double *pSrc, double *pDst);
	double GetVectorLength(int nNum, double *pV);

	double Compute33MatrixDeterminant(double *p, double row, double col);
	int CalcSevenParameters_linear_fast(bool bBenchMark, int nNum, double * pSrc, double *pDst, double *pSimi);
	int CalcSevenParameters_linear_Accurate(int nNum, double * pSrc, double *pDst, double *pSimi);
	int ComputeTranslationAndScaleWithRotation(int nRow, int nCol, double ph, double om, double kp, double *pSrc, double *pDst, double &s, double *t);
	int CalcSevenParameters_linear(int nNum, double * pSrc, double *pDst, double *pSimi);
	int CalcSevenParameters_linear(int nNum, PT3D *ptSrc, PT3D *ptDst, double *pSimi);
	int FindBestBenchMarkPoint(int nNum, double *pSrc, int *maxID);
	int TransPointsWithSevenParameters(int nNum, double *pSimi, double * pSrc, double *pDst, double *pErr);
	int TransPointsWithSevenParameters(int nNum, double *pSimi, PT3D * ptSrc, PT3D *ptDst, double *pErr);
	int TransRotationWithSevenParameters(int nNum, double *pSimi, double * ptSrc, double *ptDst, double *pErr);
	int CalcSevenParameters( int nNum, double * pX, double *pY, double *pZ, double *pObjX, double *pObjY, double *pObjZ, double *pSeven );
	int CalcSevenParameters_Direct(int nNum, double * pX, double *pY, double *pZ, double *pObjX, double *pObjY, double *pObjZ, double *pSeven);
	int CalcSevenParameters(int nNum, PT3D *ptSrc, PT3D *ptDst, double *pSeven);
	int TransDataWithSevenParameters(int nNum, double *pSeven, PT3D *ptSrc, PT3D *ptDst, double *pErr = NULL);
	int TransDataWithSevenParameters(int nNum, double *pSeven, double * pX, double *pY, double *pZ, double *pObjX, double *pObjY, double *pObjZ, double *pErr = NULL);
	int TransDataWithSevenParameters_Direct(int nNum, double *pSeven, double * pX, double *pY, double *pZ, double *pObjX, double *pObjY, double *pObjZ, double *pErr=NULL);
	int RotateAngleWithSevenParameters( int nNum, double *pSeven, double * pPh, double *pOm, double *pKp, double *pObjPh, double *pObjOm, double *pObjKp );
	int ComputeVectorAngle(double *v1, double *v2, double &ang);
	int ComputeCrossProduct(double *v1, double *v2, double *p);
	int SolveFundMatrix_LeastSquare(int nNum, PT2D *pt2, double *pMd, int &minErrPtID);
	int SolveEssenMatrix_LeastSquare(int nNum, PT2D *pt2, CAM &cam, double *pMd, int &minErrPtID);
	int SolveProjectMatrix_LeastSquare(int nNum, PT2D *p2, PT3D *p3, double *pM);

	int EstimatePlaneWith3DPoints(int nNum, PT3D *p3, double *line);

	int ComputeVectorMatrix(PT3D *p3, double *pVecMatrix);
	int MultiIntersection_GCamera_MultiPoints(int nPtNum, GCAM *pGCam, PT3D *pLeafPt, PT2D *p2, PT3D *p3);
	int MultiIntersection_GCamera(int nImgPtNum, GCAM *pGCam, PT3D *pLeafPt, PT2D *p2, PT3D &pt3);
	int GetxyWithXYZ_GCamera(GCAM &gcam, PT3D *pLeafPt, PT3D &pt3, double &x, double &y);

};

