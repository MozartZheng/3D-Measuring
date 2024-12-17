#pragma once
#include "CBasicFunction.h"
#include "windows.h"
#include "io.h"
class CGeoCamera
{
public:
	CGeoCamera();
	~CGeoCamera();
	void InitVariables();

	int SetOutPutPath(char *strPrj, char * strOut);

	int ReadPrj(char * strFile);
	int ReadCameraLeafPt(char * strFile);
	int OutPutResultFile(char *strFile);
	int OutPutResultFile(char *strFile, int nImgNum, int nCamNum, int nObjPtNum, EOP *pEop, GCAM *pGCam, PT3D *p3, PT2D *p2);
	int OutputLeafPtFile(char *strFile);

	int SetSyntheticErrorParameter(double BaselineScale, double LeafError, double ImgPtError);
	int GenerateSyntheticData(char *strPrjRst, char *strLeafRst);
	int GenerateSyntheticData(GCAM &gcam, int nImgNum, PT3D *pLeafPt, int nObjPtNum, PT3D *pObjPt, int nImgPtNum, PT2D *pImgPt, EOP *pEop);

	int ExtractRotationMatrixFromGMatrixbyQRFactorization(double *pG, double *pVectorMatrix, double *pR, double *pK, double *pM);
	int ExtractEOPFromVirtualRotationAndTranslation(PT3D *pLeafPt, double *pPQR_Q, double *pRMat, double *pK, double *pTVec, EOP &eop);
	int ExtractIOPFromGMatrix(double *pG, double *pT, double *pRMat, double *pTVec, double *pK, CAM &cam);
	int ExtractEOPandIOPFromGMatrix(double *pG, PT3D *pLeafPt, EOP &eop, CAM &cam);
	int ExtractPMatrixBenchMark(int nImgNum, PT3D *pLeafPt, double *pBenchMark, double *pPQR_Q);
	int NomarlizePointCoor(int nNum, PT2D *p2, PT2D *tp2, double *pT);
	int NomarlizePointCoor(int nNum, PT3D *p3, PT3D *tp3, double *pT);
	int ProjectiveTransformation(int nNum, PT3D *pR, double *pM, PT2D *pL);
	int ComputeProjectiveMatrix(int nNum, PT2D *p2, PT3D *p3, double *pM, double *pRms);

	int EstimateProjectiveMatrixAndDistortion(int nNum, PT2D *p2, PT3D *p3, double *pM, CAM &cam, double *pRms);

	int OptimizeDistortionParameters_ProjectiveTransformation_GCP(int nPtNum, PT2D *p2, PT3D *p3, double *pM, CAM &cam, double *pRms);
	int OptimizeDistortionParameters_ProjectiveTransformation_GCamera_GCP(int nPtNum, PT2D *p2, PT3D *p3, double *pM, CAM &cam, double *pRms);
	int SolveNormalEquationWithLM_ProjectiveTransformation_GCP(int nPtNum, PT2D *p2, PT3D *p3, int nUnkNum, double *pAtA, double *pAtL, CAM &cam, double rms, double &rmsImprove, double *pM, double *pX, double &maxV);
	int SolveNormalEquationWithLM_ProjectiveTransformation_GCamera_GCP(int nPtNum, PT2D *p2, PT3D *p3, int nUnkNum, double *pAtA, double *pAtL, CAM &cam, double rms, double &rmsImprove, double *pM, double *pX, double &maxV);
	int ComputeErrorEquation_ProjectiveTransformation_GCP(PT2D pt2, PT3D pt3, CAM &cam, double *pM, double *A, double *B, double *L);
	int UpdateUnknowns_ProjectiveTransformation_GCP(double *p, CAM &cam, double *pM, PT2D *p2, PT3D *p3);
	int UpdateUnknowns_ProjectiveTransformation_GCamera_GCP(double *p, CAM &cam, double *pM, PT2D *p2, PT3D *p3);
	int ComputeResiduals_ProjectiveTransformation_GCP(CAM &cam, int nPtNum, PT2D *p2, PT3D *p3, double *pM, double &res, double *pRms = NULL);
	int ComputeResiduals_ProjectiveTransformation_GCamera_GCP(CAM &cam, int nPtNum, PT2D *p2, PT3D *p3, double *pM, double &res, double *pRms = NULL);

	int GeoCameraCalibration();
	int GeoCameraCalibration(GCAM &gcam, int nImgNum, PT3D *pLeafPt, int nObjPtNum, PT3D *pObjPt, int nImgPtNum, PT2D *pImgPt, EOP *pEop);
	int GeoCameraCalibration_EstimateDistortion(int nGCamNum, GCAM *pGCam, int nLeafPtNum, PT3D *pLeafPt, int nObjPtNum, PT3D *pObjPt, int nImgPtNum, PT2D *pImgPt, EOP *pEop);
	int BundleAdjustment_GeoCamera(int nGCamNum, GCAM *pGCam, int nLeafPtNum, PT3D *pLeafPt, int nObjPtNum, PT3D * pObjPt, int nImgPtNum, PT2D *pImgPt, EOP *pEop );

	int BundleAdjustment(PT2D *p2, PT3D *p3, EOP *pEop, CAM *pCam, double *pRms);
	int FindBestBenchMarkImages(int nImgNum, EOP *pEop, int &nBenchMarkID1, int &nBenchMarkID2, int &nMainDirection);
	int BuildingNormalEquation_BundleAdjustment(PT2D *p2, PT3D *p3, EOP *pEop, CAM *pCam, int nUnkNum, double *pAtA, double *pAtL, double &rms);
	int GetErrorEquation_BundleAdjustment(bool model, PT2D &p2, PT3D &p3, EOP &eop, CAM &cam, double * pA, double *pB, double *pC, double *pT, double *pL);
	int SolveNormalEquationWithLM_BundleAdjustment(PT2D *p2, PT3D *p3, int nUnkNum, double *pAtA, double *pAtL, EOP *pEop, CAM *pCam, double rms, double &rmsImprove, double *pX, double &maxV);
	int ComputeResiduals_BundleAdjustment(PT2D *p2, PT3D *p3, EOP *pEop, CAM *pCam, double &rms, double *pRms = NULL);
	int UpdateUnknowns_BundleAdjustment(double *p, EOP *pEop, CAM *pCam, PT3D *p3);
	int GetBlockPosition(int nImgID, int nImgParaNum, int nBenchMarkID1, int nBenchMarkID2, int &sR, int &sC, int &R, int &C);
	int AddDampingCoef(double coef, int nRow, double *ATA);

	int OptimizeCameraParameters_RelativeOrientation(int nPtNum, PT2D *p2, PT3D *p3, EOP *pEop, CAM &cam, double *pRms);
	int ComputeErrorEquation_RelativeOrientation(PT2D pt2l, PT2D pt2r, CAM &cam, EOP *pEop, double *A, double *B, double *L);
	int ComputeResiduals_RelativeOrientation(CAM &cam, int nPtNum, PT2D *p2, PT3D *p3, EOP *pEop, double &res, double *pRms = NULL);
	int UpdateUnknowns_RelativeOrientation(double *p, CAM &cam, EOP *pEop);
	int SolveNormalEquationWithLM_RelativeOrientation(int nPtNum, int nUnkNum, double *pAtA, double *pAtL, CAM &cam, EOP *pEop, double rms, double &rmsImprove, double *pX, double &maxV);


	int OutputReprojectionError(PT2D *p2, PT3D *p3, double *pRms, char *strFile);
	int OutputEstimatedParaError(int nCamNum, CAM *pCam, int nImgNum, EOP *pNewEop, int nObjPtNum, PT3D *p3, char *strFile);
public:
	int m_nGCamNum;
	int m_nLeafPtNum;
	GCAM * m_pGCam;
	PT3D * m_pLeafPt;

	int m_nImgNum;
	int m_nObjPtNum;
	int m_nImgPtNum;
	int m_nCamParaNum;
	int m_nImgParaNum;
	int m_nMain_Direction;
	int * m_pCamID;
	int *m_pImgPtID;
	int *m_pImgPtNum;
	int *m_pImgPtIDOffset;

	PT3D * m_p3;
	PT2D * m_p2;
	EOP * m_pImg;
	double * m_pGMatrix;

	float m_ImgPt_Scale;
	double m_ObjPt_Scale;
	double m_BaselineScale;
	double m_LeafError;
	double m_ImgPtError;

	bool m_bEOP;
	bool m_bIOP;
	bool m_bSetBenchMarkImage;
	int m_DownSize;
	int m_nBenchMarkID1;
	int m_nBenchMarkID2;
	int m_CameraUnkFlag[MAX_NUM_CAMERA_PARAM];

	double m_DampCoef;

	char *m_strImagePath;
	char m_strPrj[512];
	char m_strOut[512];
};

