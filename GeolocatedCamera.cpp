// GeolocatedCamera.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "CBasicFunction.h"
#include "CGeoCamera.h"

int main(int argc, char* argv[])
{
	if (argc < 2){
		return 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////
	
	CGeoCamera gc1, gc2;
	CBasicFunction BF;
	gc1.ReadPrj(argv[1]);
//	gc2.ReadPrj(argv[2]);

	PT3D *ptSrc = new PT3D[gc1.m_nObjPtNum * 3];
	PT3D *ptDst = new PT3D[gc1.m_nObjPtNum * 3];
	double X, Y, Z, r[9] = { 0 };
	double scale = 1.7, tx = 190, ty = -10, tz = 100;
	BF.GetRotateMatrixWithAngle(r, 1.5, 0.4, 1.6);

	gc1.m_nObjPtNum = 3;
//	gc1.m_p3[0].X = 339.060258;	gc1.m_p3[0].Y = -215.674527;gc1.m_p3[0].Z = -28.448049;
//	gc1.m_p3[1].X = -282.325916; gc1.m_p3[1].Y = 86.473321;	gc1.m_p3[1].Z = 114.610918;
//	gc1.m_p3[2].X = 590.695553;	gc1.m_p3[2].Y = -62.604522;	gc1.m_p3[2].Z = -18.245339;

	gc1.m_p3[0].X = 394229.44551860000;	gc1.m_p3[0].Y = 5708916.9460680000; gc1.m_p3[0].Z = 79.832730777440005;
	gc1.m_p3[1].X = 393462.29560550000; gc1.m_p3[1].Y = 5707803.5803439999;	gc1.m_p3[1].Z = 97.255119395869997;
	gc1.m_p3[2].X = 392361.55796990002;	gc1.m_p3[2].Y = 5708988.7313059997;	gc1.m_p3[2].Z = 73.707177104549999;

//	gc1.m_p3[3].X = 392571.0320235;	gc1.m_p3[3].Y = 5708580.761201;	gc1.m_p3[3].Z = 82.83126111781;
	
	double rX, rY, rZ, gross_X, gross_Y, gross_Z;
	rX = rY = rZ = gross_X = gross_Y = gross_Z = 0;
	for (int i = 0; i < gc1.m_nObjPtNum; i++){
		ptSrc[i] = gc1.m_p3[i];
	//	pSrc[i].X = gc1.m_p3[i].X - gc1.m_p3[0].X;
	//	pSrc[i].Y = gc1.m_p3[i].Y - gc1.m_p3[0].Y;
	//	pSrc[i].Z = gc1.m_p3[i].Z - gc1.m_p3[0].Z;


		X = r[0] * ptSrc[i].X + r[1] * ptSrc[i].Y + r[2] * ptSrc[i].Z;
		Y = r[3] * ptSrc[i].X + r[4] * ptSrc[i].Y + r[5] * ptSrc[i].Z;
		Z = r[6] * ptSrc[i].X + r[7] * ptSrc[i].Y + r[8] * ptSrc[i].Z;
	//	Z = -Z;
		rX = sampleNormal(0, 0.0);
		rY = sampleNormal(0, 0.0);
		rZ = sampleNormal(0, 0.0);
		gross_X = gross_Y = gross_Z = 0;
		if (i%3 == 0 ) {
		//	gross_X = 1 + sampleNormal(0, 1.0);
		//	gross_Y = 2 + sampleNormal(0, 1.0);
		//	gross_Z = 3 + sampleNormal(0, 1.0);
		}

		ptDst[i].X = (scale*X + tx + rX + gross_X);
		ptDst[i].Y = (scale*Y + ty + rY + gross_Y);
		ptDst[i].Z = (scale*Z + tz + rZ + gross_Z);
	}

		 			
	ptDst[0].X = -837682.27300619415; ptDst[0].Y = 355944.06959960604; ptDst[0].Z = -9685483.5642732810;
	ptDst[1].X = -837088.05652943079; ptDst[1].Y = 354782.74569187162; ptDst[1].Z = -9683590.8608579151;
	ptDst[2].X = -836440.45250725176; ptDst[2].Y = 353021.34459845285; ptDst[2].Z = -9685600.9637157824;
	
	

	double* pSrc = new double[3 * gc1.m_nObjPtNum];
	double* pDst = new double[3 * gc1.m_nObjPtNum];

	for (int i = 0; i < gc1.m_nObjPtNum; i++) {

		pSrc[0 * gc1.m_nObjPtNum + i] = ptSrc[i].X;
		pSrc[1 * gc1.m_nObjPtNum + i] = ptSrc[i].Y;
		pSrc[2 * gc1.m_nObjPtNum + i] = ptSrc[i].Z;

		pDst[0 * gc1.m_nObjPtNum + i] = ptDst[i].X;
		pDst[1 * gc1.m_nObjPtNum + i] = ptDst[i].Y;
		pDst[2 * gc1.m_nObjPtNum + i] = ptDst[i].Z;
	}

	double inlinerThreshold = 0.3;
	int* pMask = new int[gc1.m_nObjPtNum];
//	BF.ComputeSimilarity_RANSAC(3, gc1.m_nObjPtNum, pSrc, pDst, inlinerThreshold, NULL, pMask);
//	return 0;
	
	double pSimi[12] = { 0 };
	
	for (int i = 0; i < gc1.m_nObjPtNum; i++) {

		pSrc[3 * i + 0] = ptSrc[i].X;
		pSrc[3 * i + 1] = ptSrc[i].Y;
		pSrc[3 * i + 2] = ptSrc[i].Z;

		pDst[3 * i + 0] = ptDst[i].X;
		pDst[3 * i + 1] = ptDst[i].Y;
		pDst[3 * i + 2] = ptDst[i].Z;
	}
	double *pErr = new double[gc1.m_nObjPtNum * 3];
	BF.CalcSevenParameters_linear(gc1.m_nObjPtNum, ptSrc, ptDst, pSimi);

	BF.TransPointsWithSevenParameters(gc1.m_nObjPtNum, pSimi, ptSrc, ptDst, pErr);
	double pSeven[20] = { 0 };
	BF.CalcSevenParameters(gc1.m_nObjPtNum, ptSrc, ptDst, pSeven);
	BF.TransDataWithSevenParameters(gc1.m_nObjPtNum, pSeven, ptSrc, ptDst, pErr);
	
	delete[] pSrc; pSrc = NULL;
	delete[] pDst; pDst = NULL;
	delete[] ptSrc; ptSrc = NULL;
	delete[] ptDst; ptDst = NULL;
	delete[] pErr; pErr = NULL;
	return 0;







	////////////////////////////////////////////////////////////////////////////////////////////
	char strPrj[512], strLeaf[512];
	sprintf_s(strPrj, "%s", argv[1]);
	if (argc > 2){
		sprintf_s(strLeaf, "%s", argv[2]);
	}
	else{
		char strPath[512] = { 0 };
		char strTitle[64] = { 0 };
		ParseFilePath(strPrj, strPath, strTitle, NULL);
		sprintf_s(strLeaf, "%s\\%s.leaf", strPath, strTitle);
	}
	
	int nTrialNum = 1;
	double BaselineScale[50] = { 1.0, 2.0, 2.0, 4.0, 6.0, 8.0, 10.0 };
	double LeafPtError[50] = { 0.3, 0.6, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 15.0, 20.0, 25.0, 30.0 };
	double ImgPtError[50] = { 0.0, 0.2, 0.4, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0 };
	int nBaseLineScaleNum = 1;
	int nLeafPtErrorTypeNum = 1;
	int nImgPtErrorTypeNum = 1;
	int i, j, k;
	for (i = 0; i < nBaseLineScaleNum; i++){
		for (j = 0; j < nLeafPtErrorTypeNum; j++){
			for (k = 0; k < nImgPtErrorTypeNum; k++	){
				CGeoCamera gc;
				if (gc.ReadPrj(strPrj) == 0){
					return 0;
				}
				if (gc.ReadCameraLeafPt(strLeaf) == 0){
					return 0;
				}
				char strSynPrj[512] = { 0 };
				char strSynLeaf[512] = { 0 };
				sprintf_s(strSynPrj, "%s_B-%.1f_P-%.1f_I-%.1f.adj", strPrj, BaselineScale[i], LeafPtError[j], ImgPtError[k]);
				sprintf_s(strSynLeaf, "%s_B-%.1f_P-%.1f_I-%.1f.leaf", strPrj, BaselineScale[i], LeafPtError[j], ImgPtError[k]);
				gc.SetSyntheticErrorParameter(BaselineScale[i], LeafPtError[j], ImgPtError[k]);
				gc.GenerateSyntheticData(strSynPrj, strSynLeaf);
				gc.GeoCameraCalibration();
				
			}
		}
	}

	


	
	return 0;
}

