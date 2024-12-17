#include "stdafx.h"
#include "CGeoCamera.h"
#include<time.h>

CGeoCamera::CGeoCamera()
{
	InitVariables();
}

void CGeoCamera::InitVariables()
{
	m_nGCamNum = 0;
	m_nLeafPtNum = 0;
	m_nImgNum = 0;
	m_nObjPtNum = 0;
	m_nImgPtNum = 0;
	m_ImgPt_Scale = 1;
	m_ObjPt_Scale = 1;
	m_nMain_Direction = 0;

	m_BaselineScale = 0;
	m_LeafError = 0;
	m_ImgPtError = 0;

	

	m_pCamID = NULL;
	m_pImgPtID = NULL;
	m_pImgPtNum = NULL;
	m_pImgPtIDOffset = NULL;

	m_pGCam = NULL;
	m_pLeafPt = NULL;
	m_pGMatrix = NULL;
	m_p3 = NULL;
	m_p2 = NULL;
	m_pImg = NULL;

	m_bEOP = true;
	m_bIOP = true;
	m_bSetBenchMarkImage = true;
	m_DownSize = 6;
	m_nBenchMarkID1 = -1;
	m_nBenchMarkID2 = -1;
	m_CameraUnkFlag[MAX_NUM_CAMERA_PARAM];

	m_DampCoef = 0;


	m_strImagePath = NULL;
	memset(m_strOut, 0, sizeof(char)*512);
	memset(m_CameraUnkFlag, 0, sizeof(int)*MAX_NUM_CAMERA_PARAM);
}
CGeoCamera::~CGeoCamera()
{
}
int CGeoCamera::SetOutPutPath(char *strPrj, char * strOut)
{

	char strTitle[128] = { 0 };
	char strPath[512] = { 0 };

	if (strcmp(m_strOut, "") == 0){
		ParseFilePath(strPrj, strPath, strTitle, NULL);
		if (strcmp(strOut, "") == 0){
			sprintf_s(strOut, 512, "%s\\CCResult_GeoCamera", strPath);
		}

		sprintf_s(m_strOut, "%s", strOut);
	}

	if (_access(m_strOut, 6) < 0){
		if (!CreateDirectoryA(LPCSTR(m_strOut), 0)){
			printf("Create output path failed!\n");
			return 0;
		}
	}

	return 1;
}
int CGeoCamera::SetSyntheticErrorParameter(double BaselineScale, double LeafError, double ImgPtError)
{
	m_BaselineScale = BaselineScale;
	m_LeafError = LeafError;
	m_ImgPtError = ImgPtError;
	return 1;
}
int CGeoCamera::GenerateSyntheticData(char *strPrjRst, char *strLeafRst)
{
	GenerateSyntheticData(m_pGCam[0], m_nImgNum, m_pLeafPt, m_nObjPtNum, m_p3, m_nImgPtNum, m_p2, m_pImg);
	OutPutResultFile(strPrjRst);
	OutputLeafPtFile(strLeafRst);
	return 1;
}

int CGeoCamera::GenerateSyntheticData(GCAM &gcam, int nImgNum, PT3D *pLeafPt, int nObjPtNum, PT3D *pObjPt, int nImgPtNum, PT2D *pImgPt, EOP *pEop)
{
	int i, j;
	double m[3] = { 0 };double pVM[9] = { 0 };
	CAM cam = gcam.camPara;
	EOP eop; PT3D pt3; CBasicFunction BF; double rx, ry;
	double pK[9] = { cam.fx, 0, -cam.x0, 0, cam.fy, -cam.y0, 0, 0, -1 };
	Mat maK(3, 3, CV_64F, pK);

	double sX = 0, sY = 0, sZ = 0;
	for (i = 0; i < nImgNum; i++){
		for (j = 0; j < 4; j++){
			pLeafPt[i * 4 + j].X = pEop[i].Xs + (pLeafPt[i * 4 + j].X - pEop[i].Xs) * m_BaselineScale;
			pLeafPt[i * 4 + j].Y = pEop[i].Ys + (pLeafPt[i * 4 + j].Y - pEop[i].Ys) * m_BaselineScale;
			pLeafPt[i * 4 + j].Z = pEop[i].Zs + (pLeafPt[i * 4 + j].Z - pEop[i].Zs) * m_BaselineScale;
		}
	}

	BF.ComputeVectorMatrix(pLeafPt, pVM);
	eop = pEop[0];pt3 = pLeafPt[0];
	m[0] = pVM[0] * (pt3.X - eop.Xs) + pVM[1] * (pt3.Y - eop.Ys) + pVM[2] * (pt3.Z - eop.Zs);
	m[1] = pVM[3] * (pt3.X - eop.Xs) + pVM[4] * (pt3.Y - eop.Ys) + pVM[5] * (pt3.Z - eop.Zs);
	m[2] = pVM[6] * (pt3.X - eop.Xs) + pVM[7] * (pt3.Y - eop.Ys) + pVM[8] * (pt3.Z - eop.Zs);
	Mat maM(3, 1, CV_64F, m);
	double pX[4], pY[4], pZ[4], pObjX[4], pObjY[4], pObjZ[4];
	
	for (j = 0; j < 4; j++){
		pX[j] = pLeafPt[j].X;
		pY[j] = pLeafPt[j].Y;
		pZ[j] = pLeafPt[j].Z;
		
	}
	double pR0[9] = { 0 };
	BF.GetRotateMatrixWithAngle(pR0, eop.Ph, eop.Om, eop.Kp);
	Mat R0(3, 3, CV_64F, pR0);
	Mat P0(3, 3, CV_64F, pVM);
	Mat G = maK*R0.t()*P0.inv();
	double *pG = G.ptr<double>(0);

	double *pdX = new double[nImgNum * 4];
	double *pdY = new double[nImgNum * 4];
	double *pdZ = new double[nImgNum * 4];
	double *pdm = new double[nImgNum * 3];
	double *pdXs = new double[nImgNum];
	double *pdYs = new double[nImgNum];
	double *pdZs = new double[nImgNum];
	double *pG1 = new double[nImgNum * 9];
	memcpy(pG1, pG, sizeof(double) * 9);
	memcpy(pdm, m, sizeof(double) * 3);
	Mat G1;		
	double *pSrc = new double[4 * 3];
	double *pDst = new double[4 * 3];
	//Generate planar scene
	///////////////////////////////////////////////////////////////////////////////////////////
	double line[4] = { 0 };
	BF.EstimatePlaneWith3DPoints(nObjPtNum, pObjPt, line);
	for (i = 0; i < nObjPtNum; i++){
	//	pObjPt[i].X = -(line[1] * pObjPt[i].Y + line[2] * pObjPt[i].Z + line[3]) / line[0];
	}
	//////////////////////////////////////////////////////////////////////////////////////////
	
	for (i = 1; i < nImgNum; i++){

		for (j = 0; j < 4; j++){
			pObjX[j] = pLeafPt[i * 4 + j].X;
			pObjY[j] = pLeafPt[i * 4 + j].Y;
			pObjZ[j] = pLeafPt[i * 4 + j].Z;

			pSrc[j * 3 + 0] = pX[j];
			pSrc[j * 3 + 1] = pY[j];
			pSrc[j * 3 + 2] = pZ[j];

			pDst[j * 3 + 0] = pObjX[j];
			pDst[j * 3 + 1] = pObjY[j];
			pDst[j * 3 + 2] = pObjZ[j];

		}
		double pSimi[12] = { 0 };
		BF.CalcSevenParameters_linear(4, pSrc, pDst, pSimi);
		double *pErr = new double[12];
		BF.TransPointsWithSevenParameters(4, pSimi, pSrc, pDst, pErr);
		double pSeven[64] = { 0 };
		BF.CalcSevenParameters(4, pX, pY, pZ, pObjX, pObjY, pObjZ, pSeven);

		BF.TransDataWithSevenParameters(4, pSeven, pX, pY, pZ, pObjX, pObjY, pObjZ, pErr);
		
		for (j = 0; j < 4; j++){
			pdX[i*4+j] = pLeafPt[i * 4 + j].X - pObjX[j];
			pdY[i*4+j] = pLeafPt[i * 4 + j].Y - pObjY[j];
			pdZ[i*4+j] = pLeafPt[i * 4 + j].Z - pObjZ[j];

			pLeafPt[i * 4 + j].X = pObjX[j];
			pLeafPt[i * 4 + j].Y = pObjY[j];
			pLeafPt[i * 4 + j].Z = pObjZ[j];

			pSrc[j * 3 + 0] = pX[j];
			pSrc[j * 3 + 1] = pY[j];
			pSrc[j * 3 + 2] = pZ[j];

			pDst[j * 3 + 0] = pObjX[j];
			pDst[j * 3 + 1] = pObjY[j];
			pDst[j * 3 + 2] = pObjZ[j];

		}
		BF.CalcSevenParameters_linear(4, pSrc, pDst, pSimi);
		BF.TransPointsWithSevenParameters(4, pSimi, pSrc, pDst, pErr);
		BF.CalcSevenParameters(4, pX, pY, pZ, pObjX, pObjY, pObjZ, pSeven);
		BF.TransDataWithSevenParameters(4, pSeven, pX, pY, pZ, pObjX, pObjY, pObjZ, pErr);

		BF.ComputeVectorMatrix(pLeafPt + i * 4, pVM);
		
		Mat maP(3, 3, CV_64F, pVM);
		pt3 = pLeafPt[i * 4 + 0];
		
		Mat maSP = maP.inv()*maM;
		double *pSP = maSP.ptr<double>(0);

		
		pdXs[i] = pt3.X - pSP[0] - pEop[i].Xs;
		pdYs[i] = pt3.Y - pSP[1] - pEop[i].Ys;
		pdZs[i] = pt3.Z - pSP[2] - pEop[i].Zs;
		
		pEop[i].Xs = pt3.X - pSP[0];
		pEop[i].Ys = pt3.Y - pSP[1];
		pEop[i].Zs = pt3.Z - pSP[2];
		eop = pEop[i];

		pdm[i * 3 + 0] = pVM[0] * (pt3.X - eop.Xs) + pVM[1] * (pt3.Y - eop.Ys) + pVM[2] * (pt3.Z - eop.Zs);
		pdm[i * 3 + 1] = pVM[3] * (pt3.X - eop.Xs) + pVM[4] * (pt3.Y - eop.Ys) + pVM[5] * (pt3.Z - eop.Zs);
		pdm[i * 3 + 2] = pVM[6] * (pt3.X - eop.Xs) + pVM[7] * (pt3.Y - eop.Ys) + pVM[8] * (pt3.Z - eop.Zs);



		double ph, om, kp;
		BF.RotateAngleWithSevenParameters(1, pSeven, &pEop[0].Ph, &pEop[0].Om, &pEop[0].Kp, &ph, &om, &kp);

		pEop[i].Ph = ph;
		pEop[i].Om = om;
		pEop[i].Kp = kp;
		double pR[9] = { 0 };
		BF.GetRotateMatrixWithAngle(pR, ph, om, kp);
		Mat maR(3, 3, CV_64F, pR);

		G1 = maK*maR.t()*maP.inv();
		memcpy(pG1 + i * 9, G1.ptr<double>(0), sizeof(double) * 9);
	}
	Mat G31 = G1*maM;
	memcpy(m, G31.ptr<double>(0), sizeof(double) * 3);
	double pM[12] = { 0 };

//	srand(unsigned(time(NULL))); 
	for (i = 0; i < nImgNum; i++){

		for (j = 0; j < 4; j++){
			sX = sampleNormal(0, m_LeafError / 1000);
			sY = sampleNormal(0, m_LeafError / 1000);
			sZ = sampleNormal(0, m_LeafError / 1000);
			pLeafPt[i * 4 + j].X += sX;
			pLeafPt[i * 4 + j].Y += sY;
			pLeafPt[i * 4 + j].Z += sZ;

			pSrc[j * 3 + 0] = pLeafPt[0 * 4 + j].X;
			pSrc[j * 3 + 1] = pLeafPt[0 * 4 + j].Y;
			pSrc[j * 3 + 2] = pLeafPt[0 * 4 + j].Z;
			if (i > 0){
				pDst[j * 3 + 0] = pLeafPt[i * 4 + j].X;
				pDst[j * 3 + 1] = pLeafPt[i * 4 + j].Y;
				pDst[j * 3 + 2] = pLeafPt[i * 4 + j].Z;
			}

		}
		double pSeven[64] = { 0 };
		if (i > 0){
			BF.CalcSevenParameters_linear(4, pSrc, pDst, pSeven);
		}
		
	}
	pM[0] = pG1[0]; pM[1] = pG1[1]; pM[2] = pG1[2]; pM[3] = m[0];
	pM[4] = pG1[3]; pM[5] = pG1[4]; pM[6] = pG1[5]; pM[7] = m[1];
	pM[8] = pG1[6]; pM[9] = pG1[7]; pM[10] = pG1[8]; pM[11] = m[2];
	memcpy(m_pGCam[0].gMatrix, pM, sizeof(double) * 12);
	double sx = 0, sy = 0, tx = 0, ty = 0;
	rx = ry = 0;

	for (i = 0; i < nImgPtNum; i++){
		PT2D pt2 = pImgPt[i];
		eop = pEop[pt2.nImgID];
		pt3 = pObjPt[pt2.nPtsID];
		
		BF.GetxyWithXYZ(gcam.camPara, eop, pt3, tx, ty);
		tx += gcam.camPara.x0;
		ty += gcam.camPara.y0;
		BF.ComputeDistortionWithUndistortedPoints_Normalized(tx, ty, gcam.camPara, rx, ry);
	//	pt2.x = float(tx + rx);
	//	pt2.y = float(ty + ry);
		double tx1, ty1;
		BF.CorrectDistortion(true, pt2.x, pt2.y, gcam.camPara, tx1, ty1);
		
		sx = sampleNormal(0, m_ImgPtError);
		sy = sampleNormal(0, m_ImgPtError);
		
	//	pt2.x = float(int(pt2.x / 1) * 1);
	//	pt2.y = float(int(pt2.y / 1) * 1);

		pt2.x = float(int(pt2.x));
		pt2.y = float(int(pt2.y));

	//	rx += (pImgPt[i].x -pt2.x)*(pImgPt[i].x - pt2.x);
	//	ry += (pImgPt[i].y -pt2.y)*(pImgPt[i].y - pt2.y);
		pImgPt[i].x = float(pt2.x+sx);
		pImgPt[i].y = float(pt2.y+sy);
	}
//	rx = sqrt(rx / nImgPtNum);
//	ry = sqrt(ry / nImgPtNum);
	return 1;
}

int CGeoCamera::ReadPrj(char * strFile)
{
	char strExtention[10] = { 0 };
	ParseFilePath(strFile, NULL, NULL, strExtention);
	SetOutPutPath(strFile, m_strOut);
	memcpy(m_strPrj, strFile, sizeof(char) * 512);
	char strTmp[512] = { 0 };

	FILE * fp = NULL;
	fopen_s(&fp, strFile, "r");

	if (fp == NULL){
		printf("Can not open project file:%s\n", strFile);
		return 0;
	}
	int i, j, k, nImgID, nCamID, nAttrib;
	int nNum, nImgNum, nCamNum, nLeafNum, nHeight, nWidth, nGridRow, nGridCol;
	float intervalX = 0, intervalY = 0;
	CAM cam; EOP eop; PT3D pt3; PT2D pt2;


	memset(strTmp, 0, sizeof(char) * 512);
	fgets(strTmp, 512, fp);
	sscanf_s(strTmp, "%d%d%ld%d%d %f%f", &nImgNum, &nCamNum, &nNum, &nGridRow, &nGridCol, &intervalX, &intervalY);
	m_nImgNum = nImgNum;
	m_nGCamNum = nCamNum;
	m_nObjPtNum = nNum;


	m_pImg = new EOP[nImgNum];
	m_pGCam = new GCAM[nCamNum];
	memset(m_pImg, 0, sizeof(EOP)*nImgNum);
	memset(m_pGCam, 0, sizeof(CAM)*nCamNum);
	m_pCamID = new int[nImgNum];
	memset(m_pCamID, 0, sizeof(int)*nImgNum);
	m_strImagePath = new char[nImgNum*MAX_FILE_PATH_LENGTH];
	memset(m_strImagePath, 0, sizeof(char)*nImgNum*MAX_FILE_PATH_LENGTH);

	int downsize = m_DownSize;
	//Read camera info
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	k = 0;
	float fy, fx, x0, y0;
	double k1, k2, k3, p1, p2, pixelSize = 0;
	for (i = 0; i < nCamNum; i++){

		fgets(strTmp, 512, fp);
		sscanf_s(strTmp, "%d%d%d%f%f%f%f %*lf%lf%lf%lf %lf%lf", &nLeafNum, &nWidth, &nHeight, &fx, &fy, &x0, &y0, &k1, &k2, &k3, &p1, &p2);
		///////////////////////////////////////////////////////////////

		nWidth /= downsize;
		nHeight /= downsize;
		nWidth = (nWidth / 2+1) * 2;
		nHeight = (nHeight / 2 + 1) * 2;
		fx /= downsize;
		fy /= downsize;
		x0 /= downsize;
		y0 /= downsize;
		//////////////////////////////////////////////////////////////
	//	k1 = k2 = p1 = p2 = 0;
		///////////////////////////////////////////////////////////////

		cam.nWidth = nWidth;
		cam.nHeight = nHeight;
		cam.height = float(nWidth*m_ImgPt_Scale);
		cam.width = float(nHeight*m_ImgPt_Scale);
		cam.pixelSize = float(pixelSize);
		cam.fx = float(fx*m_ImgPt_Scale);
		cam.fy = float(fy*m_ImgPt_Scale);

		if (fx != fy){

			cam.fx = cam.fy;
		}
		else{

		}
		cam.x0 = float(x0*m_ImgPt_Scale);
		cam.y0 = float(y0*m_ImgPt_Scale);
		cam.k1 = k1 / pow(m_ImgPt_Scale, 2);
		cam.k2 = k2 / pow(m_ImgPt_Scale, 4);
		cam.k3 = k3 / pow(m_ImgPt_Scale, 6);
		cam.p1 = p1 / m_ImgPt_Scale;
		cam.p2 = p2 / m_ImgPt_Scale;
		if (cam.fx == 0){ cam.fx = 10000; }
		if (cam.fy == 0){ cam.fy = 10000; }

		m_pGCam[k].leafNum = nLeafNum;
		m_pGCam[k].camPara = cam;
		k++;
	}
	if (k != m_nGCamNum){
		printf("Error!Camera number is unexpected!\n");
		return 0;
	}
	k = 0;
	CBasicFunction BF;
	double Xs, Ys, Zs, Ph, Om, Kp, a[9];
	int *pImgID = new int[nImgNum];
	memset(pImgID, 0, sizeof(int)*nImgNum);
	//Read image info
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (i = 0; i < nImgNum; i++){
		char strPath[512] = { 0 };

		fgets(strTmp, 512, fp);
		sscanf_s(strTmp, "%d%d%d", &nImgID, &nCamID, &nAttrib);


		fgets(strTmp, 512, fp);
		char *pS = strrchr(strTmp, '\n');
		if (pS) *pS = 0;
		memcpy(m_strImagePath + i*MAX_FILE_PATH_LENGTH, strTmp, sizeof(char) * MAX_FILE_PATH_LENGTH);
		fgets(strTmp, 512, fp);
		sscanf_s(strTmp, "%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8]);
		fgets(strTmp, 512, fp);
		sscanf_s(strTmp, "%lf%lf%lf", &Xs, &Ys, &Zs);

		BF.GetAngleWithRotateMatrix(a, Ph, Om, Kp);
		double b[9] = { 0 }; BF.GetRotateMatrixWithAngle(b, Ph, Om, Kp);
		eop.Ph = Ph;
		eop.Om = Om;
		eop.Kp = Kp;
		eop.Xs = Xs*m_ObjPt_Scale;
		eop.Ys = Ys*m_ObjPt_Scale;
		eop.Zs = Zs*m_ObjPt_Scale;
		pImgID[i] = nAttrib;
		if (nAttrib < 0)
		{
			continue;
		}
		m_pImg[k] = eop;
		m_pCamID[k] = nCamID;
		pImgID[i] = k;
		k++;
	}
	if (k != m_nImgNum){
		printf("Warning!Image number is unexpected!\n");
		m_nImgNum = k;
	}
	if (m_nImgNum < 1){
		printf("Error! Number of images is less than 1!\n");
		return 0;
	}

	m_pImgPtNum = new int[m_nImgNum];
	m_pImgPtIDOffset = new int[m_nImgNum];
	memset(m_pImgPtNum, 0, sizeof(int)*m_nImgNum);
	memset(m_pImgPtIDOffset, 0, sizeof(int)*m_nImgNum);
	//Read feature points
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	k = 0;
	double x, y, X, Y, Z;
	vector<PT2D> veP2;
	m_p3 = new PT3D[m_nObjPtNum];
	for (i = 0; i < nNum; i++){
		fgets(strTmp, 512, fp);
		if (strcmp(strTmp, "\n") == 0){
			fgets(strTmp, 512, fp);
		}
		nAttrib = 0;
		sscanf_s(strTmp, "%s%lf%lf%lf %d", pt3.name, MAX_LENGTH_POINT_NAME, &X, &Y, &Z, &pt3.nAttrib);
		//Set all point as control point
		/////////////////////////////////////////////////////////////////////////////////////////////////
		pt3.nAttrib = 1;
		/////////////////////////////////////////////////////////////////////////////////////////////////
		fgets(strTmp, 512, fp);
		sscanf_s(strTmp, "%d", &pt3.nIPtNum);

		pt3.X = X*m_ObjPt_Scale;
		pt3.Y = Y*m_ObjPt_Scale;
		pt3.Z = Z*m_ObjPt_Scale;
		pt3.nIPtSID = k;

		int l = 0;
		for (j = 0; j < pt3.nIPtNum; j++){
			fgets(strTmp, 512, fp);
			sscanf_s(strTmp, "%d%lf%lf%*lf%*lf%d", &nImgID, &x, &y, &nAttrib);
			x /= downsize;
			y /= downsize;

		//	pt2.x = float(x*m_ImgPt_Scale);
		//	pt2.y = float(y*m_ImgPt_Scale);
			pt2.x = float((float(x) - cam.nWidth / 2.0)*m_ImgPt_Scale);
			pt2.y = float((cam.nHeight / 2.0 - float(y))*m_ImgPt_Scale);
			pt2.nImgID = pImgID[nImgID];
			pt2.nCamID = m_pCamID[nImgID];
			pt2.nPtsID = i;
			pt2.nAttrib = nAttrib;

			if (pt2.nImgID < 0 || int(pt2.nImgID) > m_nImgNum - 1){

				continue;
			}
			else{
				if (fabs(x) > m_pGCam[pt2.nCamID].camPara.nWidth){
					printf("Error!Coordinates of image point %d of object %d out of range!\n", j, i);
				}
				if (fabs(y) > m_pGCam[pt2.nCamID].camPara.nHeight){
					printf("Error!Coordinates of image point %d of object %d out of range!\n", j, i);
				}
			}
			m_pImgPtNum[pt2.nImgID] ++;
			veP2.push_back(pt2);
			k++; l++;
		}
		pt3.nIPtNum = l;
		
		m_p3[i] = pt3;
		if (l < 1){
			m_p3[i].nAttrib = -1;
		}
	}
	fclose(fp);
	m_nImgPtNum = k;

	m_p2 = new PT2D[m_nImgPtNum];
	memcpy(m_p2, veP2.data(), sizeof(PT2D)*m_nImgPtNum);
	
	
	for (i = 0; i < m_nImgNum; i++){
		for (j = 0; j < i; j++){
			m_pImgPtIDOffset[i] += m_pImgPtNum[j];
		}
	}
	m_pImgPtID = new int[m_nImgPtNum];
	memset(m_pImgPtID, 0, sizeof(int)*m_nImgPtNum);

	int *pCount = new int[m_nImgNum];
	memset(pCount, 0, sizeof(int)*m_nImgNum);
	for (i = 0; i < m_nImgPtNum; i++){
		pt2 = m_p2[i];
		int nOffset = m_pImgPtIDOffset[pt2.nImgID];
		m_pImgPtID[nOffset + pCount[pt2.nImgID]] = i;
		pCount[pt2.nImgID] ++;

	}
	
	delete[] pCount; pCount = NULL;
	
	return 1;
}
int CGeoCamera::ReadCameraLeafPt(char * strFile)
{
	char strTmp[512] = { 0 };

	FILE * fp = NULL;
	fopen_s(&fp, strFile, "r");

	if (fp == NULL){
		printf("Can not open project file:%s\n", strFile);
		return 0;
	}
	if (m_pGCam == NULL){
		printf("Error!Camera parameters undefined!");
		fclose(fp);
		return 0;
	}

	int i, j, k, nImgNum, nGCamNum, nPtNum, nImgID, nCamID, nLeafNum;
	memset(strTmp, 0, sizeof(char) * 512);
	fgets(strTmp, 512, fp);
	sscanf_s(strTmp, "%d%d%d", &nImgNum, &nGCamNum, &nPtNum);

	for (i = 0; i < nGCamNum; i++){
		double m[12] = { 0 };
		fgets(strTmp, 512, fp);
		sscanf_s(strTmp, "%d%d %lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf", &nCamID, &nLeafNum, 
			                                                          m, m + 1, m + 2, m + 3, m + 4, m + 5, m + 6, m + 7, m + 8, m + 9, m + 10, m + 11);
		memcpy(m_pGCam[i].gMatrix, m, sizeof(double) * 12);
		if (nLeafNum != m_pGCam[i].leafNum){
			printf("Error!Camera leaf number in consistency!");
			fclose(fp);
			return 0;
		}

	}
	double X, Y, Z;
	PT3D *p3 = new PT3D[nPtNum];
	memset(p3, 0, sizeof(PT3D)*nPtNum);
	k = 0;
	for (i = 0; i < nImgNum; i++){
		fgets(strTmp, 512, fp);
		sscanf_s(strTmp, "%d%d", &nImgID, &nCamID);
		nLeafNum = m_pGCam[nCamID].leafNum;
		for (j = 0; j < nLeafNum; j++){
			fgets(strTmp, 512, fp);
			sscanf_s(strTmp, "%s %lf %lf %lf", p3[k].name, 64, &Y, &X, &Z);

			p3[k].X = X;
			p3[k].Y = Y;
			p3[k].Z = Z;
			k++;
		}
	}
	if (k != nPtNum){
		printf("Warning!Point number inconsistancy!");
		nPtNum = k;
	}
	m_nLeafPtNum = nPtNum;
	m_pLeafPt = new PT3D[m_nLeafPtNum];
	memcpy(m_pLeafPt, p3, sizeof(PT3D)*m_nLeafPtNum);
	//Exchange some leaf point
	/////////////////////////////////////////////////////////////////////////////////////
	for (i = 0; i < nImgNum; i++){
		PT3D pt3 = m_pLeafPt[i * 4 + 0];
		m_pLeafPt[i * 4 + 0] = m_pLeafPt[i * 4 + 3];
		m_pLeafPt[i * 4 + 3] = pt3;

		pt3 = m_pLeafPt[i * 4 + 1];
		m_pLeafPt[i * 4 + 1] = m_pLeafPt[i * 4 + 3];
		m_pLeafPt[i * 4 + 3] = pt3;
	}
	/////////////////////////////////////////////////////////////////////////////////////
	fclose(fp);
	delete[] p3; p3 = NULL;
	return 1;
}

int CGeoCamera::ExtractRotationMatrixFromGMatrixbyQRFactorization(double *pG, double *pVectorMatrix, double *pRMat, double *pK, double *pTVec)
{
	Mat matG(3, 4, CV_64F, pG);
	Mat matP(3, 3, CV_64F, pVectorMatrix);
	Mat matR, matQ;
	
	Mat matT = matG.colRange(0, 3)*matP;
	Mat maG33 = matG.colRange(0, 3);
	double *pG33 = matG.ptr<double>(0);
//	RQDecomp3x3(matP, matR, matQ);
	RQDecomp3x3(matT, matR, matQ);
	Mat matQT = matQ.t();
	memcpy(pK, matR.ptr<double>(0), sizeof(double) * 9);
	memcpy(pRMat, matQT.ptr<double>(0), sizeof(double) * 9);

	double scale = -1/pK[8];
	for (int i = 0; i < 9; i++){
		pK[i] *= scale;
	}
	pTVec[0] = pG[3] * scale;
	pTVec[1] = pG[7] * scale;
	pTVec[2] = pG[11] * scale;
	return 1;
}
int CGeoCamera::ExtractPMatrixBenchMark(int nImgNum, PT3D *pLeafPt, double *pBenchMark, double *pPQR_Q)
{
	int i, j;
	Mat matR, matQ;
	CBasicFunction BF;
	double pT[9] = { 0 };
	for (i = 0; i < nImgNum; i++){

		double pVectorMatrix[9], T[9];

		BF.ComputeVectorMatrix(pLeafPt + i * 4, pVectorMatrix);
		Mat matP(3, 3, CV_64F, pVectorMatrix);
		RQDecomp3x3(matP, matR, matQ);
		memcpy(T, matR.ptr<double>(0), sizeof(double) * 9);
		memcpy(pPQR_Q+i*9, matQ.ptr<double>(0), sizeof(double) * 9);

		for (j = 0; j < 9; j++){
			pT[j] += T[j];
		}
	}
	for (j = 0; j < 9; j++){
		pT[j] /= nImgNum;
	}

	memcpy(pBenchMark, pT, sizeof(double) * 9);


	return 1;
}

int CGeoCamera::ExtractIOPFromGMatrix(double *pG, double *pT, double *pRMat, double *pTVec, double *pK, CAM &cam)
{

	ExtractRotationMatrixFromGMatrixbyQRFactorization(pG, pT, pRMat, pK, pTVec);

	cam.fx = float(pK[0]);
	cam.fy = float(pK[4]);
	cam.x0 = float(-pK[2]);
	cam.y0 = float(-pK[5]);
	cam.s = float(pK[1]);
	return 1;
}
int CGeoCamera::ExtractEOPFromVirtualRotationAndTranslation(PT3D *pLeafPt, double *pPQR_Q, double *pRMat, double *pK, double *pTVec, EOP &eop)
{
	CBasicFunction BF;
	double Xs, Ys, Zs, ph, om, kp;
	double pR[9] = { 0 }; double pT[9] = { 0 };
	Mat matP_Q(3, 3, CV_64F, pPQR_Q);
	Mat matR(3, 3, CV_64F, pRMat);
	Mat matRot = matP_Q.inv()*matR;
	BF.GetAngleWithRotateMatrix(matRot.ptr<double>(0), ph, om, kp);
	Mat matK(3, 3, CV_64F, pK), matM(3, 1, CV_64F, pTVec);

	Mat T = matRot*matK.inv()*matM;
	memcpy(pT, T.ptr<double>(0), sizeof(double) * 3);

	Xs = pLeafPt[0].X - pT[0];
	Ys = pLeafPt[0].Y - pT[1];
	Zs = pLeafPt[0].Z - pT[2];

	eop.Xs = Xs;
	eop.Ys = Ys;
	eop.Zs = Zs;
	eop.Ph = ph;
	eop.Om = om;
	eop.Kp = kp;
	return 1;
}
int CGeoCamera::ExtractEOPandIOPFromGMatrix(double *pG, PT3D *pLeafPt, EOP &eop, CAM &cam)
{
	CBasicFunction BF;
	double pVectorMatrix[9];
	double pR[9], pK[9], pM[3], pT[3];
	double Xs, Ys, Zs, ph, om, kp;
	BF.ComputeVectorMatrix(pLeafPt, pVectorMatrix);
	ExtractRotationMatrixFromGMatrixbyQRFactorization(pG, pVectorMatrix, pR, pK, pM);
	
	BF.GetAngleWithRotateMatrix(pR, ph, om, kp);
	Mat matR(3, 3, CV_64F, pR), matK(3, 3, CV_64F, pK), matM(3, 1, CV_64F, pM);

	Mat T = matR*matK.inv()*matM;
	memcpy(pT, T.ptr<double>(0), sizeof(double) * 3);
	
	Xs = pLeafPt[0].X - pT[0];
	Ys = pLeafPt[0].Y - pT[1];
	Zs = pLeafPt[0].Z - pT[2];

	Mat P(3, 3, CV_64F, pVectorMatrix);

	double dX[3] = { pLeafPt[0].X - Xs, pLeafPt[0].Y - Ys, pLeafPt[0].Z - Zs };
	
	Mat D(3, 1, CV_64F, dX);
	Mat m = P*D;
	double *ptM = m.ptr<double>(0);


	eop.Xs = Xs;
	eop.Ys = Ys;
	eop.Zs = Zs;
	eop.Ph = ph;
	eop.Om = om;
	eop.Kp = kp;

	cam.fx = float(pK[0]);
	cam.fy = float(pK[4]);
	cam.x0 = float(-pK[2]);
	cam.y0 = float(-pK[5]);
	cam.s = float(pK[1]);
	
	return 1;
}

int CGeoCamera::NomarlizePointCoor(int nNum, PT3D *p3, PT3D *tp3, double *pT)
{
	int i = 0;
	double xc, yc, zc, x1, y1, z1;
	double aver_dis;
	aver_dis = 1;

	xc = yc = zc = 0;

	for (i = 0; i < nNum; i++){

		xc += p3[i].X;
		yc += p3[i].Y;
		zc += p3[i].Z;

	}
	xc /= nNum;
	yc /= nNum;
	zc /= nNum;

	for (i = 0; i < nNum; i++){

		x1 = p3[i].X - xc;
		y1 = p3[i].Y - yc;
		z1 = p3[i].Z - zc;
		aver_dis += x1*x1 + y1*y1 + z1*z1;
	}
	aver_dis /= nNum;
	aver_dis = sqrt(aver_dis);

	for (i = 0; i < nNum; i++){

		tp3[i].X = (p3[i].X - xc) / aver_dis;
		tp3[i].Y = (p3[i].Y - yc) / aver_dis;
		tp3[i].Z = (p3[i].Z - zc) / aver_dis;

	}
	pT[0] = pT[5] = pT[10] = 1 / aver_dis; pT[15] = 1;
	pT[3] = -xc / aver_dis; pT[7] = -yc / aver_dis; pT[11] = -zc / aver_dis;
	return 1;
}
int CGeoCamera::NomarlizePointCoor(int nNum, PT2D *p2, PT2D *tp2, double *pT)
{
	int i = 0;
	double xc, yc, x1, y1;
	double aver_dis;
	aver_dis = 1;

	xc = yc = 0;

	for (i = 0; i < nNum; i++){

		xc += p2[i].x;
		yc += p2[i].y;

	}
	xc /= nNum;
	yc /= nNum;

	for (i = 0; i < nNum; i++){

		x1 = p2[i].x - (xc);
		y1 = p2[i].y - (yc);
		aver_dis += x1*x1 + y1*y1;
	}
	aver_dis /= nNum;
	aver_dis = sqrt(aver_dis);

	for (i = 0; i < nNum; i++){

		tp2[i].x = float((p2[i].x - xc) / aver_dis);
		tp2[i].y = float((p2[i].y - yc) / aver_dis);

	}
	pT[0] = pT[4] = 1 / aver_dis; pT[8] = 1;
	pT[2] = -xc / aver_dis; pT[5] = -yc / aver_dis;

	return 1;
}
int CGeoCamera::ProjectiveTransformation(int nNum, PT3D *pR, double *pM, PT2D *pL)
{
	if (pM == NULL){
		printf("Projective matrix is invalid!\n");
		return 0;
	}
	int i = 0;
	double X, Y, Z, hx, hy, hz;

	for (i = 0; i < nNum; i++){
		X = pR[i].X;
		Y = pR[i].Y;
		Z = pR[i].Z;
		hx = pM[0] * X + pM[1] * Y + pM[2] * Z + pM[3] * 1;
		hy = pM[4] * X + pM[5] * Y + pM[6] * Z + pM[7] * 1;
		hz = pM[8] * X + pM[9] * Y + pM[10] * Z + pM[11] * 1;

		if (fabs(hz) < 1e-6){
			printf("Projective matrix is unexpected!\n");
			return 0;
		}
		pL[i].x = float(hx / hz);
		pL[i].y = float(hy / hz);
	}
	return 1;
}
int CGeoCamera::ComputeProjectiveMatrix(int nNum, PT2D *p2, PT3D *p3, double *pM, double *pRms)
{
	int i, j, k;
	PT2D pt2; PT3D pt3;
	CBasicFunction BF;
	double A[22] = { 0 };
	double L[2] = { 0 };
	double AtA[121] = { 0 };
	double AtL[11] = { 0 };
	double x, y, X, Y, Z, rx, ry;

	pM[11] = 1;

	double R[9] = { 0 }; double t[3] = { 0 };

	PT2D * tp2 = new PT2D[nNum];
	PT3D * tp3 = new PT3D[nNum];
	double T2[9] = { 0 };
	double T3[16] = { 0 };

	NomarlizePointCoor(nNum, p2, tp2, T2);
	NomarlizePointCoor(nNum, p3, tp3, T3);
	for (i = 0; i < nNum; i++){
		pt2 = tp2[i];
		pt3 = tp3[i];
		x = pt2.x;
		y = pt2.y;
		X = pt3.X;
		Y = pt3.Y;
		Z = pt3.Z;
		A[0] = X;
		A[1] = Y;
		A[2] = Z;
		A[3] = 1;
		A[4] = 0;
		A[5] = 0;
		A[6] = 0;
		A[7] = 0;
		A[8] = -x*X;
		A[9] = -x*Y;
		A[10] = -x*Z;
		L[0] = x*pM[11];

		A[11 + 0] = 0;
		A[11 + 1] = 0;
		A[11 + 2] = 0;
		A[11 + 3] = 0;
		A[11 + 4] = X;
		A[11 + 5] = Y;
		A[11 + 6] = Z;
		A[11 + 7] = 1;
		A[11 + 8] = -y*X;
		A[11 + 9] = -y*Y;
		A[11 + 10] = -y*Z;
		L[1] = y*pM[11];

		for (j = 0; j < 11; j++){
			for (k = 0; k < 11; k++){
				AtA[j * 11 + k] += A[j] * A[k];
				AtA[j * 11 + k] += A[11 + j] * A[11 + k];
			}
			AtL[j] += A[j] * L[0];
			AtL[j] += A[11 + j] * L[1];
		}
	}
	CXMatrix maAtA, maAtL, maX;
	maAtA.InitMatrix(AtA, 11, 11);
	maAtL.InitMatrix(AtL, 11, 1);
	maX = maAtA.InverseMatrix()*maAtL;

	Mat D, U, V, Vt;

	Mat mATA(11, 11, CV_64F, AtA);
	SVD::compute(mATA, D, U, Vt);

	double *pD = (double*)D.data;
	if (determinant(U) < 0) U = -U;
	if (determinant(Vt) < 0) Vt = -Vt;

	double *p = maX.GetData();
	for (i = 0; i < 11; i++){
		pM[i] = p[i];
	}

	CXMatrix maT2, maT3, maM, maTMT;
	maM.InitMatrix(pM, 3, 4);
	maT2.InitMatrix(T2, 3, 3);
	maT3.InitMatrix(T3, 4, 4);

	maTMT = maT2.InverseMatrix()*maM*maT3;

	memcpy(pM, maTMT.GetData(), sizeof(double) * 12);

	FILE * fp = NULL;
	char strFile[512] = { 0 };
	sprintf_s(strFile, "%s\\ProjectiveTransformationError_%d.res", m_strOut, pt2.nImgID);
	fopen_s(&fp, strFile, "w");
	double M[12] = { 0 };
	memcpy(M, pM, sizeof(double) * 12);

	rx = ry = 0;
	for (i = 0; i < nNum; i++){
		pt2 = p2[i];
		pt3 = p3[i];

		ProjectiveTransformation(1, &pt3, pM, &pt2);

		if (pRms){
			pRms[2 * i + 0] = p2[i].x - (pt2.x + rx);
			pRms[2 * i + 1] = p2[i].y - (pt2.y + ry);
		}

		if (fp){
			fprintf(fp, "%d %12.6lf %12.6lf %12.6lf %12.6lf\n", 0, p2[i].x, p2[i].y, pRms[2 * i + 0], pRms[2 * i + 1]);
		}

	}

	if (fp) fclose(fp);
	return 1;
}
int CGeoCamera::OptimizeDistortionParameters_ProjectiveTransformation_GCamera_GCP(int nPtNum, PT2D *p2, PT3D *p3, double *pM, CAM &cam, double *pRms)
{
	int i = 0, j = 0, j1 = 0, j2 = 0;
	int nUnkNum = 0, nEquaUnkNum = 0, nCamUnkNum = 0, nProjUnkNum = 11;
	if (m_CameraUnkFlag[0]){
		nCamUnkNum += 2;
	}
	for (i = 1; i < 20; i++){
		nCamUnkNum += m_CameraUnkFlag[i];
	}
	int nImgUnkNum = nProjUnkNum;
	nEquaUnkNum = nCamUnkNum;
	nUnkNum = nCamUnkNum + nImgUnkNum;
	int nObjUnkOffset = 0;
	int nObjUnkNum = 3;
	double A[64] = { 0 };
	double B[30] = { 0 };
	double b[2] = { 0 };
	double *AtA = new double[nUnkNum*nUnkNum];
	double *Atb = new double[nUnkNum];
	int k = 0, count = 0, nTotPtNum = 0;
	int max_iteration_num = 1000;
	double maxr = 1, w = 1, res = 10000, sqsum = 0;
	double max_correction = 100, rmsImprove = 1;
	double correct_threshold = 1e-8;
	double rmsImprove_threshold = 1e-5*m_ImgPt_Scale;
	double *ptM = NULL; PT2D pt2; PT3D pt3;  CBasicFunction BF;
	maxr = sqrt(pow(cam.nWidth*m_ImgPt_Scale / 2, 2) + pow(cam.nHeight*m_ImgPt_Scale / 2, 2));
	do
	{
		sqsum = 0; nTotPtNum = 0;
		memset(AtA, 0, sizeof(double) * nUnkNum*nUnkNum);
		memset(Atb, 0, sizeof(double) * nUnkNum);
		for (i = 0; i < nPtNum; i++){
			pt3 = p3[i];
			int nSRow = 0, nSCol = 0;

			for (j = 0; j < pt3.nIPtNum; j++){
				pt2 = p2[pt3.nIPtSID + j];
				ptM = pM;
				nSRow = nCamUnkNum;
				nSCol = nCamUnkNum;
				ComputeErrorEquation_ProjectiveTransformation_GCP(pt2, pt3, cam, ptM, A, B, b);
				sqsum += b[0] * b[0] + b[1] * b[1];
				nTotPtNum++;

				//	w =  pow(2, maxr-r);

				for (j1 = 0; j1 < nCamUnkNum; j1++){
					for (j2 = 0; j2 < nCamUnkNum; j2++){

						AtA[j1*nUnkNum + j2] += A[j1] * w * A[j2];
						AtA[j1*nUnkNum + j2] += A[nEquaUnkNum + j1] * w * A[nEquaUnkNum + j2];
					}
					for (j2 = 0; j2 < nImgUnkNum; j2++){
						AtA[j1*nUnkNum + (nSCol + j2)] += A[j1] * w * B[j2];
						AtA[j1*nUnkNum + (nSCol + j2)] += A[nEquaUnkNum + j1] * w * B[nImgUnkNum + j2];
					}

					Atb[j1] += A[j1] * w * b[0];
					Atb[j1] += A[nEquaUnkNum + j1] * w * b[1];
				}

				for (j1 = 0; j1 < nImgUnkNum; j1++){
					for (j2 = 0; j2 < nImgUnkNum; j2++){
						AtA[(nSRow + j1)*nUnkNum + (nSCol + j2)] += B[j1] * w * B[j2];
						AtA[(nSRow + j1)*nUnkNum + (nSCol + j2)] += B[nImgUnkNum + j1] * w * B[nImgUnkNum + j2];
					}
					for (j2 = 0; j2 < nCamUnkNum; j2++){
						AtA[(nSRow + j1)*nUnkNum + j2] += B[j1] * w * A[j2];
						AtA[(nSRow + j1)*nUnkNum + j2] += B[nImgUnkNum + j1] * w * A[nEquaUnkNum + j2];
					}
					Atb[nSRow + j1] += B[j1] * w * b[0];
					Atb[nSRow + j1] += B[nImgUnkNum + j1] * w * b[1];
				}

			}
		}
		if (count > 0) rmsImprove = res - sqrt(sqsum / nTotPtNum);
		res = sqrt(sqsum / nTotPtNum);
		double *p = NULL; double maxV = 0;

		double *pX = new double[nUnkNum];
		memset(pX, 0, sizeof(double)*nUnkNum);
		SolveNormalEquationWithLM_ProjectiveTransformation_GCamera_GCP(nPtNum, p2, p3, nUnkNum, AtA, Atb, cam, res, rmsImprove, pM, pX, max_correction);
		p = pX;

		/*
		CXMatrix maAtA, maAtb, maX;
		maAtA.InitMatrix(AtA, nUnkNum, nUnkNum);
		maAtb.InitMatrix(Atb, nUnkNum, 1);
		maX = maAtA.InverseMatrix()*maAtb;
		max_correction = maX.GetMaxFabsElement();
		p = maX.GetData();
		*/
		printf("Iteration %d: rms, max correction, and rms improvement %.9lf %.9lf %.9lf\n", count, res, max_correction, rmsImprove);
		UpdateUnknowns_ProjectiveTransformation_GCamera_GCP(p, cam, pM, m_p2, p3);

		count++;
	} while ((fabs(max_correction) > correct_threshold && rmsImprove > rmsImprove_threshold) && count < max_iteration_num);

	ComputeResiduals_ProjectiveTransformation_GCamera_GCP(cam, nPtNum, p2, p3, pM, res, pRms);

	delete[] AtA; AtA = NULL;
	delete[] Atb; Atb = NULL;
	return 1;
}
int CGeoCamera::OptimizeDistortionParameters_ProjectiveTransformation_GCP(int nPtNum, PT2D *p2, PT3D *p3, double *pM, CAM &cam, double *pRms)
{
	int i = 0, j = 0, j1 = 0, j2 = 0;
	int nUnkNum = 0, nEquaUnkNum = 0, nCamUnkNum = 0, nProjUnkNum = 11;
	if (m_CameraUnkFlag[0]){
		nCamUnkNum += 2;
	}
	for (i = 1; i < 20; i++){
		nCamUnkNum += m_CameraUnkFlag[i];
	}
	int nImgUnkNum = nProjUnkNum;
	nEquaUnkNum = nCamUnkNum;
	nUnkNum = nCamUnkNum + nImgUnkNum*m_nImgNum;
	int nObjUnkOffset = 0;
	int nObjUnkNum = 3;
	double A[64] = { 0 };
	double B[30] = { 0 };
	double b[2] = { 0 };
	double *AtA = new double[nUnkNum*nUnkNum];
	double *Atb = new double[nUnkNum];
	int k = 0, count = 0, nTotPtNum = 0;
	int max_iteration_num = 1000;
	double maxr = 1, w = 1, res = 10000, sqsum = 0;
	double max_correction = 100, rmsImprove = 1;
	double correct_threshold = 1e-8;
	double rmsImprove_threshold = 1e-5*m_ImgPt_Scale;
	double *ptM = NULL; PT2D pt2; PT3D pt3;  CBasicFunction BF;
	maxr = sqrt(pow(cam.nWidth*m_ImgPt_Scale / 2, 2) + pow(cam.nHeight*m_ImgPt_Scale / 2, 2));
	do
	{
		sqsum = 0; nTotPtNum = 0;
		memset(AtA, 0, sizeof(double) * nUnkNum*nUnkNum);
		memset(Atb, 0, sizeof(double) * nUnkNum);
		for (i = 0; i < nPtNum; i++){
			pt3 = p3[i];
			int nSRow = 0, nSCol = 0;

			for (j = 0; j < pt3.nIPtNum; j++){
				pt2 = p2[pt3.nIPtSID + j];
				ptM = pM + pt2.nImgID * 12;
				nSRow = nCamUnkNum + pt2.nImgID*nImgUnkNum;
				nSCol = nCamUnkNum + pt2.nImgID*nImgUnkNum;
				ComputeErrorEquation_ProjectiveTransformation_GCP(pt2, pt3, cam, ptM, A, B, b);
				sqsum += b[0] * b[0] + b[1] * b[1];
				nTotPtNum++;

				//	w =  pow(2, maxr-r);

				for (j1 = 0; j1 < nCamUnkNum; j1++){
					for (j2 = 0; j2 < nCamUnkNum; j2++){

						AtA[j1*nUnkNum + j2] += A[j1] * w * A[j2];
						AtA[j1*nUnkNum + j2] += A[nEquaUnkNum + j1] * w * A[nEquaUnkNum + j2];
					}
					for (j2 = 0; j2 < nImgUnkNum; j2++){
						AtA[j1*nUnkNum + (nSCol + j2)] += A[j1] * w * B[j2];
						AtA[j1*nUnkNum + (nSCol + j2)] += A[nEquaUnkNum + j1] * w * B[nImgUnkNum + j2];
					}

					Atb[j1] += A[j1] * w * b[0];
					Atb[j1] += A[nEquaUnkNum + j1] * w * b[1];
				}

				for (j1 = 0; j1 < nImgUnkNum; j1++){
					for (j2 = 0; j2 < nImgUnkNum; j2++){
						AtA[(nSRow + j1)*nUnkNum + (nSCol + j2)] += B[j1] * w * B[j2];
						AtA[(nSRow + j1)*nUnkNum + (nSCol + j2)] += B[nImgUnkNum + j1] * w * B[nImgUnkNum + j2];
					}
					for (j2 = 0; j2 < nCamUnkNum; j2++){
						AtA[(nSRow + j1)*nUnkNum + j2] += B[j1] * w * A[j2];
						AtA[(nSRow + j1)*nUnkNum + j2] += B[nImgUnkNum + j1] * w * A[nEquaUnkNum + j2];
					}
					Atb[nSRow + j1] += B[j1] * w * b[0];
					Atb[nSRow + j1] += B[nImgUnkNum + j1] * w * b[1];
				}

			}
		}
		if (count > 0) rmsImprove = res - sqrt(sqsum / nTotPtNum);
		res = sqrt(sqsum / nTotPtNum);
		double *p = NULL; double maxV = 0;

		double *pX = new double[nUnkNum];
		memset(pX, 0, sizeof(double)*nUnkNum);
		SolveNormalEquationWithLM_ProjectiveTransformation_GCP(nPtNum, p2, p3, nUnkNum, AtA, Atb, cam, res, rmsImprove, pM, pX, max_correction);
		p = pX;

		/*
		CXMatrix maAtA, maAtb, maX;
		maAtA.InitMatrix(AtA, nUnkNum, nUnkNum);
		maAtb.InitMatrix(Atb, nUnkNum, 1);
		maX = maAtA.InverseMatrix()*maAtb;
		max_correction = maX.GetMaxFabsElement();
		p = maX.GetData();
		*/
		printf("Iteration %d: rms, max correction, and rms improvement %.9lf %.9lf %.9lf\n", count, res, max_correction, rmsImprove);
		UpdateUnknowns_ProjectiveTransformation_GCP(p, cam, pM, m_p2, p3);

		count++;
	} while ((fabs(max_correction) > correct_threshold && rmsImprove > rmsImprove_threshold) && count < max_iteration_num);

	ComputeResiduals_ProjectiveTransformation_GCP(cam, nPtNum, p2, p3, pM, res, pRms);

	delete[] AtA; AtA = NULL;
	delete[] Atb; Atb = NULL;

	return 1;
}
int CGeoCamera::SolveNormalEquationWithLM_ProjectiveTransformation_GCP(int nPtNum, PT2D *p2, PT3D *p3, int nUnkNum, double *pAtA, double *pAtL, CAM &cam, double rms, double &rmsImprove, double *pM, double *pX, double &maxV)
{
	int count = 0;
	int nMaxIteration = 100;
	double res1 = 1, res2 = 0;
	double dRes1 = 0, dRes2 = 0;
	double coefInterval = 2;
	double coef = m_DampCoef;

	CXMatrix maU1, maU2;
	double ImproveThreshold = 1e-6;
	double LM_Threshold = 1e-6;
	CXMatrix maAtA, maAtL, maX;
	maAtL.InitMatrix(pAtL, nUnkNum, 1);

	CAM tcam = cam;
	double *ptM = new double[m_nImgNum * 12];
	PT3D *ptP3 = new PT3D[nPtNum];
	res1 = res2 = rms;
	//	m_DampCoef = 1e-6;

	do{
		if (AddDampingCoef(coef, nUnkNum, pAtA) == 0){
			return 0;
		}
		maAtA.InitMatrix(pAtA, nUnkNum, nUnkNum);
		maU2 = maAtA.InverseMatrix()*maAtL;

		maxV = maU2.GetMaxFabsElement();

		tcam = cam;
		memcpy(ptM, pM, sizeof(double)*m_nImgNum * 12);
		memcpy(ptP3, p3, sizeof(PT3D)*nPtNum);
		UpdateUnknowns_ProjectiveTransformation_GCP(maU2.GetData(), tcam, ptM, p2, ptP3);
		ComputeResiduals_ProjectiveTransformation_GCP(tcam, nPtNum, p2, ptP3, ptM, res2);

		dRes2 = res2 - res1;

		printf("LM Iteration:%d %20.9e %20.9e %20.9e %20.9e %20.9e %20.9e %20.9e\n", count, maxV, coef, res1, res2, dRes1, dRes2, rms);

		if ((dRes2 == 0 && dRes1 == 0) || (/*count > 2 &&*/ (res1 - rms)<0 && dRes2 > 0 && dRes1 < 0)){

			if (count == 0){
				maU1 = maU2;
				res1 = res2;

			}
			maX = maU1;

			maxV = maX.GetMaxFabsElement();
			m_DampCoef = coef / coefInterval;
			rmsImprove = rms - res1;
			break;
		}
		else{
			if (coef == 0){ if (m_DampCoef == 0) coef = 1e-9; else coef = m_DampCoef; }
			else{
				coef *= coefInterval;
				m_DampCoef = coef;
			}
		}
		res1 = res2;
		dRes1 = dRes2;
		maU1 = maU2;
		count++;
	} while (count < nMaxIteration);

	delete[] ptM; ptM = NULL;
	delete[] ptP3; ptP3 = NULL;
	if (nMaxIteration>10 && count == nMaxIteration){
		printf("Failed to solve normal equation via LM algorithm\n");

		return 0;
	}

	printf("Max correction:%.9lf, Residuals improvement:%.9lf\n", maxV, rmsImprove);

	memcpy(pX, maX.GetData(), sizeof(double)*nUnkNum);
	return 1;
}
int CGeoCamera::SolveNormalEquationWithLM_ProjectiveTransformation_GCamera_GCP(int nPtNum, PT2D *p2, PT3D *p3, int nUnkNum, double *pAtA, double *pAtL, CAM &cam, double rms, double &rmsImprove, double *pM, double *pX, double &maxV)
{
	int count = 0;
	int nMaxIteration = 100;
	double res1 = 1, res2 = 0;
	double dRes1 = 0, dRes2 = 0;
	double coefInterval = 2;
	double coef = m_DampCoef;

	CXMatrix maU1, maU2;
	double ImproveThreshold = 1e-6;
	double LM_Threshold = 1e-6;
	CXMatrix maAtA, maAtL, maX;
	maAtL.InitMatrix(pAtL, nUnkNum, 1);

	CAM tcam = cam;
	double *ptM = new double[m_nImgNum * 12];
	PT3D *ptP3 = new PT3D[nPtNum];
	res1 = res2 = rms;
	//	m_DampCoef = 1e-6;

	do{
		if (AddDampingCoef(coef, nUnkNum, pAtA) == 0){
			return 0;
		}
		maAtA.InitMatrix(pAtA, nUnkNum, nUnkNum);
		maU2 = maAtA.InverseMatrix()*maAtL;

		maxV = maU2.GetMaxFabsElement();

		tcam = cam;
		memcpy(ptM, pM, sizeof(double)*m_nImgNum * 12);
		memcpy(ptP3, p3, sizeof(PT3D)*nPtNum);
		UpdateUnknowns_ProjectiveTransformation_GCamera_GCP(maU2.GetData(), tcam, ptM, p2, ptP3);
		ComputeResiduals_ProjectiveTransformation_GCamera_GCP(tcam, nPtNum, p2, ptP3, ptM, res2);

		dRes2 = res2 - res1;

		printf("LM Iteration:%d %20.9e %20.9e %20.9e %20.9e %20.9e %20.9e %20.9e\n", count, maxV, coef, res1, res2, dRes1, dRes2, rms);

		if ((dRes2 == 0 && dRes1 == 0) || (/*count > 2 &&*/ (res1 - rms)<0 && dRes2 > 0 && dRes1 < 0)){

			if (count == 0){
				maU1 = maU2;
				res1 = res2;

			}
			maX = maU1;

			maxV = maX.GetMaxFabsElement();
			m_DampCoef = coef / coefInterval;
			rmsImprove = rms - res1;
			break;
		}
		else{
			if (coef == 0){ if (m_DampCoef == 0) coef = 1e-9; else coef = m_DampCoef; }
			else{
				coef *= coefInterval;
				m_DampCoef = coef;
			}
		}
		res1 = res2;
		dRes1 = dRes2;
		maU1 = maU2;
		count++;
	} while (count < nMaxIteration);

	delete[] ptM; ptM = NULL;
	delete[] ptP3; ptP3 = NULL;
	if (nMaxIteration>10 && count == nMaxIteration){
		printf("Failed to solve normal equation via LM algorithm\n");

		return 0;
	}

	printf("Max correction:%.9lf, Residuals improvement:%.9lf\n", maxV, rmsImprove);

	memcpy(pX, maX.GetData(), sizeof(double)*nUnkNum);
	return 1;
}
int CGeoCamera::ComputeErrorEquation_ProjectiveTransformation_GCP(PT2D pt2, PT3D pt3, CAM &cam, double *pM, double *A, double *B, double *L)
{
	int k = 0;
	CBasicFunction BF; PT2D npt2;
	double fx, fy, x, y, X, Y, Z, dx, dy, tx, ty, r, dr;
	double ddr, r_u, dx_x, dx_y, dy_x, dy_y, dx_x0, dy_x0, dx_y0, dy_y0, x_u, y_u;
	ProjectiveTransformation(1, &pt3, pM, &npt2);
	BF.ComputeDistortionWithUndistortedPoints_Normalized(npt2.x, npt2.y, cam, dx, dy);

	X = pt3.X;
	Y = pt3.Y;
	Z = pt3.Z;

	fx = cam.fx;
	fy = cam.fy;

	x = ((pt2.x - cam.x0) - cam.s / fy*(pt2.y - cam.y0)) / fx;
	y = (pt2.y - cam.y0) / fy;
	r = x*x + y*y;

	tx = pt2.x - cam.x0;
	ty = pt2.y - cam.y0;
	r_u = tx*tx + ty*ty;
	k = 0;

	dr = cam.k1*r + cam.k2*r*r + cam.k3*pow(r, 3);
	ddr = 2 * cam.k1 + 4 * cam.k2*r + 6 * cam.k3*r*r;
	dx_x = dr + x*x*ddr + (2 * cam.p1*y + 6 * cam.p2*x)*(1 + cam.p3*r) + (2 * cam.p1*x*y + cam.p2*(r + 2 * x*x)) * 2 * cam.p3*x;
	dx_y = x*y*ddr + (2 * cam.p1*x + 2 * cam.p2*y)*(1 + cam.p3*r) + (2 * cam.p1*x*y + cam.p2*(r + 2 * x*x)) * 2 * cam.p3*y;
	dy_y = dr + y*y*ddr + (2 * cam.p2*x + 6 * cam.p1*y)*(1 + cam.p3*r) + (2 * cam.p2*x*y + cam.p1*(r + 2 * y*y)) * 2 * cam.p3*y;
	dy_x = x*y*ddr + (2 * cam.p2*y + 2 * cam.p1*x)*(1 + cam.p3*r) + (2 * cam.p2*x*y + cam.p1*(r + 2 * y*y)) * 2 * cam.p3*x;

	dx_x0 = dx_x*(-1 / fx);
	dx_y0 = dx_x*(cam.s / fx / fy) + dx_y*(-1 / fy);
	dy_x0 = dy_x*(-1 / fx);
	dy_y0 = dy_x*(cam.s / fx / fy) + dy_y*(-1 / fy);

	if (m_CameraUnkFlag[1]) { A[k] = 1 + fx*dx_x0; k++; }
	if (m_CameraUnkFlag[2]) { A[k] = fx*dx_y0; k++; }
	if (m_CameraUnkFlag[3]) { A[k] = fx*x*r; k++; }
	if (m_CameraUnkFlag[4]) { A[k] = fx*x*r*r; k++; }
	if (m_CameraUnkFlag[5]) { A[k] = fx*x*r*r*r; k++; }
	if (m_CameraUnkFlag[6]) { A[k] = fx * 2 * x*y; k++; }
	if (m_CameraUnkFlag[7]) { A[k] = fx*(r + 2 * x*x); k++; }

	if (m_CameraUnkFlag[1]) { A[k] = fy*dy_x0; k++; }
	if (m_CameraUnkFlag[2]) { A[k] = 1 + fy*dy_y0; k++; }
	if (m_CameraUnkFlag[3]) { A[k] = fy*y*r; k++; }
	if (m_CameraUnkFlag[4]) { A[k] = fy*y*r*r; k++; }
	if (m_CameraUnkFlag[5]) { A[k] = fy*y*r*r*r; k++; }
	if (m_CameraUnkFlag[6]) { A[k] = fy*(r + 2 * y*y); k++; }
	if (m_CameraUnkFlag[7]) { A[k] = fy * 2 * x*y; k++; }

	x_u = x - dx;
	y_u = y - dy;

	double X_, Y_, Z_;
	X_ = pM[0] * X + pM[1] * Y + pM[2] * Z + pM[3];
	Y_ = pM[4] * X + pM[5] * Y + pM[6] * Z + pM[7];
	Z_ = pM[8] * X + pM[9] * Y + pM[10] * Z + pM[11];
	B[0] = X / Z_;
	B[1] = Y / Z_;
	B[2] = Z / Z_;
	B[3] = 1 / Z_;
	B[4] = 0;
	B[5] = 0;
	B[6] = 0;
	B[7] = 0;
	B[8] = -X*X_ / (Z_*Z_);
	B[9] = -Y*X_ / (Z_*Z_);
	B[10] = -Z*X_ / (Z_*Z_);

	B[11 + 0] = 0;
	B[11 + 1] = 0;
	B[11 + 2] = 0;
	B[11 + 3] = 0;
	B[11 + 4] = X / Z_;
	B[11 + 5] = Y / Z_;
	B[11 + 6] = Z / Z_;
	B[11 + 7] = 1 / Z_;
	B[11 + 8] = -X*Y_ / (Z_*Z_);
	B[11 + 9] = -Y*Y_ / (Z_*Z_);
	B[11 + 10] = -Z*Y_ / (Z_*Z_);

	L[0] = pt2.x - dx - npt2.x;
	L[1] = pt2.y - dy - npt2.y;
	return 1;
}
int CGeoCamera::UpdateUnknowns_ProjectiveTransformation_GCamera_GCP(double *p, CAM &cam, double *pM, PT2D *p2, PT3D *p3)
{
	int i = 0, j = 0, k = 0;
	if (m_CameraUnkFlag[0]) { cam.fx += float(p[k]); k++; }
	if (m_CameraUnkFlag[0]) { cam.fy += float(p[k]); k++; }
	if (m_CameraUnkFlag[1]) { cam.x0 += float(p[k]); k++; }
	if (m_CameraUnkFlag[2]) { cam.y0 += float(p[k]); k++; }
	if (m_CameraUnkFlag[17]) { cam.s += float(p[k]); k++; }

	if (m_CameraUnkFlag[3]) { cam.k1 += p[k]; k++; }
	if (m_CameraUnkFlag[4]) { cam.k2 += p[k]; k++; }
	if (m_CameraUnkFlag[5]) { cam.k3 += p[k]; k++; }
	if (m_CameraUnkFlag[6]) { cam.p1 += p[k]; k++; }
	if (m_CameraUnkFlag[7]) { cam.p2 += p[k]; k++; }
	if (m_CameraUnkFlag[8]) { cam.p3 += p[k]; k++; }

	if (m_CameraUnkFlag[9]) { cam.k4 += p[k]; k++; }
	if (m_CameraUnkFlag[10]) { cam.k5 += p[k]; k++; }
	if (m_CameraUnkFlag[11]) { cam.k6 += p[k]; k++; }
	if (m_CameraUnkFlag[12]) { cam.k7 += p[k]; k++; }
	if (m_CameraUnkFlag[13]) { cam.k8 += p[k]; k++; }
	if (m_CameraUnkFlag[14]) { cam.k9 += p[k]; k++; }

	if (m_CameraUnkFlag[15]) { cam.s1 += p[k]; k++; }
	if (m_CameraUnkFlag[16]) { cam.s2 += p[k]; k++; }

	pM[i * 12 + 0] += p[k]; k++;
	pM[i * 12 + 1] += p[k]; k++;
	pM[i * 12 + 2] += p[k]; k++;
	pM[i * 12 + 3] += p[k]; k++;
	pM[i * 12 + 4] += p[k]; k++;
	pM[i * 12 + 5] += p[k]; k++;
	pM[i * 12 + 6] += p[k]; k++;
	pM[i * 12 + 7] += p[k]; k++;
	pM[i * 12 + 8] += p[k]; k++;
	pM[i * 12 + 9] += p[k]; k++;
	pM[i * 12 + 10] += p[k]; k++;

	return 1;
}
int CGeoCamera::UpdateUnknowns_ProjectiveTransformation_GCP(double *p, CAM &cam, double *pM, PT2D *p2, PT3D *p3)
{
	int i = 0, j = 0, k = 0;
	if (m_CameraUnkFlag[0]) { cam.fx += float(p[k]); k++; }
	if (m_CameraUnkFlag[0]) { cam.fy += float(p[k]); k++; }
	if (m_CameraUnkFlag[1]) { cam.x0 += float(p[k]); k++; }
	if (m_CameraUnkFlag[2]) { cam.y0 += float(p[k]); k++; }
	if (m_CameraUnkFlag[17]) { cam.s += float(p[k]); k++; }

	if (m_CameraUnkFlag[3]) { cam.k1 += p[k]; k++; }
	if (m_CameraUnkFlag[4]) { cam.k2 += p[k]; k++; }
	if (m_CameraUnkFlag[5]) { cam.k3 += p[k]; k++; }
	if (m_CameraUnkFlag[6]) { cam.p1 += p[k]; k++; }
	if (m_CameraUnkFlag[7]) { cam.p2 += p[k]; k++; }
	if (m_CameraUnkFlag[8]) { cam.p3 += p[k]; k++; }

	if (m_CameraUnkFlag[9]) { cam.k4 += p[k]; k++; }
	if (m_CameraUnkFlag[10]) { cam.k5 += p[k]; k++; }
	if (m_CameraUnkFlag[11]) { cam.k6 += p[k]; k++; }
	if (m_CameraUnkFlag[12]) { cam.k7 += p[k]; k++; }
	if (m_CameraUnkFlag[13]) { cam.k8 += p[k]; k++; }
	if (m_CameraUnkFlag[14]) { cam.k9 += p[k]; k++; }

	if (m_CameraUnkFlag[15]) { cam.s1 += p[k]; k++; }
	if (m_CameraUnkFlag[16]) { cam.s2 += p[k]; k++; }
	for (i = 0; i < m_nImgNum; i++){
		pM[i * 12 + 0] += p[k]; k++;
		pM[i * 12 + 1] += p[k]; k++;
		pM[i * 12 + 2] += p[k]; k++;
		pM[i * 12 + 3] += p[k]; k++;
		pM[i * 12 + 4] += p[k]; k++;
		pM[i * 12 + 5] += p[k]; k++;
		pM[i * 12 + 6] += p[k]; k++;
		pM[i * 12 + 7] += p[k]; k++;
		pM[i * 12 + 8] += p[k]; k++;
		pM[i * 12 + 9] += p[k]; k++;
		pM[i * 12 + 10] += p[k]; k++;
	}


	return 1;
}
int CGeoCamera:: ComputeResiduals_ProjectiveTransformation_GCamera_GCP(CAM &cam, int nPtNum, PT2D *p2, PT3D *p3, double *pM, double &res, double *pRms)
{
	int i = 0, j = 0, k = 0;
	double dx, dy, rx, ry;
	double sum = 0, sqsum = 0;
	double *ptM = NULL;
	PT2D pt2, ptl, ptr;
	CBasicFunction BF;
	for (i = 0; i < nPtNum; i++){
		PT3D pt3 = p3[i];
		ptr.x = float(pt3.X);
		ptr.y = float(pt3.Y);
		for (j = 0; j < pt3.nIPtNum; j++){
			ptl = p2[pt3.nIPtSID + j];

			ptM = pM;
			ProjectiveTransformation(1, &pt3, ptM, &pt2);
			BF.ComputeDistortionWithUndistortedPoints_Normalized(pt2.x, pt2.y, cam, dx, dy);

			rx = ptl.x - (pt2.x + dx);
			ry = ptl.y - (pt2.y + dy);

			if (pRms){
				pRms[2 * k + 0] = rx;
				pRms[2 * k + 1] = ry;
			}
			sum += sqrt(rx*rx + ry*ry);
			sqsum += rx*rx + ry*ry;
			k++;
		}
	}
	res = sqrt(sqsum / k);
	return 1;
}
int CGeoCamera::ComputeResiduals_ProjectiveTransformation_GCP(CAM &cam, int nPtNum, PT2D *p2, PT3D *p3, double *pM, double &res, double *pRms)
{
	int i = 0, j = 0, k = 0;
	double dx, dy, rx, ry;
	double sum = 0, sqsum = 0;
	double *ptM = NULL;
	PT2D pt2, ptl, ptr;
	CBasicFunction BF;
	for (i = 0; i < nPtNum; i++){
		PT3D pt3 = p3[i];
		ptr.x = float(pt3.X);
		ptr.y = float(pt3.Y);
		for (j = 0; j < pt3.nIPtNum; j++){
			ptl = p2[pt3.nIPtSID + j];

			ptM = pM + ptl.nImgID * 12;
			ProjectiveTransformation(1, &pt3, ptM, &pt2);
			BF.ComputeDistortionWithUndistortedPoints_Normalized(pt2.x, pt2.y, cam, dx, dy);

			rx = ptl.x - (pt2.x + dx);
			ry = ptl.y - (pt2.y + dy);

			if (pRms){
				pRms[2 * k + 0] = rx;
				pRms[2 * k + 1] = ry;
			}
			sum += sqrt(rx*rx + ry*ry);
			sqsum += rx*rx + ry*ry;
			k++;
		}
	}
	res = sqrt(sqsum / k);
	return 1;
}
int CGeoCamera::EstimateProjectiveMatrixAndDistortion(int nNum, PT2D *p2, PT3D *p3, double *pM, CAM &cam, double *pRms)
{
	return 1;
}
int CGeoCamera::GeoCameraCalibration()
{
	
	return GeoCameraCalibration( m_pGCam[0], m_nImgNum, m_pLeafPt, m_nObjPtNum, m_p3, m_nImgPtNum, m_p2, m_pImg);
}
int CGeoCamera::GeoCameraCalibration(GCAM &gcam, int nImgNum, PT3D *pLeafPt, int nObjPtNum, PT3D *pObjPt, int nImgPtNum, PT2D *pImgPt, EOP *pEop)
{
	int i, j, k;
	PT3D *pVirtualPt = new PT3D[nImgPtNum];
	double *pVectorMatrix = new double[nImgNum * 9];
	CBasicFunction BF;
	for (i = 0; i < nImgNum; i++){
		BF.ComputeVectorMatrix(pLeafPt + i*gcam.leafNum, pVectorMatrix + i * 9);
	}
	int nCamID = 0, nImgID = 0;
	double *p; PT3D *tp3;

	k = 0;
	double x_u, y_u;
	double *pm = new double[nImgNum * 3];
	double *pG = new double[nImgNum * 9];
	for (i = 0; i < nImgNum; i++){
		Mat P(3, 3, CV_64F, pVectorMatrix + i * 9);
		double pR[9] = { 0 };
		BF.GetRotateMatrixWithAngle(pR, pEop[i].Ph, pEop[i].Om, pEop[i].Kp);
		Mat R(3, 3, CV_64F, pR);
		Mat G = P*R;
		memcpy(pG + i * 9, G.ptr<double>(0), sizeof(double) * 9);
		double *p = pVectorMatrix + i * 9;
		PT3D pt3 = pLeafPt[i * 4 + 0];
		pm[i * 3 + 0] = p[0] * (pt3.X - pEop[i].Xs) + p[1] * (pt3.Y - pEop[i].Ys) + p[2] * (pt3.Z - pEop[i].Zs);
		pm[i * 3 + 1] = p[3] * (pt3.X - pEop[i].Xs) + p[4] * (pt3.Y - pEop[i].Ys) + p[5] * (pt3.Z - pEop[i].Zs);
		pm[i * 3 + 2] = p[6] * (pt3.X - pEop[i].Xs) + p[7] * (pt3.Y - pEop[i].Ys) + p[8] * (pt3.Z - pEop[i].Zs);
	}
	PT2D *pP2 = new PT2D[nImgPtNum];
	memcpy(pP2, pImgPt, sizeof(PT2D)*nImgPtNum);
	double *pRms = new double[nImgPtNum * 2];
	memset(pRms, 0, sizeof(double)*nImgPtNum * 2);
	for (i = 0; i < nObjPtNum; i++){
		PT3D pt3 = pObjPt[i];
		if (pt3.nAttrib < 0){
			continue;
		}
		for (j = 0; j < pt3.nIPtNum; j++){
			PT2D pt2 = pImgPt[pt3.nIPtSID + j];
			if (pt2.nImgID > 4){
			//	continue;
			}
			BF.CorrectDistortion(true, pt2.x, pt2.y, gcam.camPara, x_u, y_u);
			
		//	pImgPt[pt3.nIPtSID + j].x = float(x_u);
		//	pImgPt[pt3.nIPtSID + j].y = float(y_u);

			pP2[k].x = float(x_u);
			pP2[k].y = float(y_u);
			
			p = pVectorMatrix + pt2.nImgID * 9;
			tp3 = pLeafPt + pt2.nImgID*gcam.leafNum;
			pVirtualPt[k].X = p[0] * (pt3.X - tp3[0].X) + p[1] * (pt3.Y - tp3[0].Y) + p[2] * (pt3.Z - tp3[0].Z);
			pVirtualPt[k].Y = p[3] * (pt3.X - tp3[0].X) + p[4] * (pt3.Y - tp3[0].Y) + p[5] * (pt3.Z - tp3[0].Z);
			pVirtualPt[k].Z = p[6] * (pt3.X - tp3[0].X) + p[7] * (pt3.Y - tp3[0].Y) + p[8] * (pt3.Z - tp3[0].Z);
			pVirtualPt[k].nIPtNum = 1;
			pVirtualPt[k].nIPtSID = k;
			k++;
		}
	}
	nImgPtNum = k;
	double pM[12] = { 0 };

	ComputeProjectiveMatrix(nImgPtNum, pP2, pVirtualPt, pM, pRms);
	char strFile[512] = { 0 };
	sprintf_s(strFile, "%s\\ProjectiveMatrixResiduals_B-%.1f_P-%.1f_I-%.1f.txt", m_strOut, m_BaselineScale, m_LeafError, m_ImgPtError);
	OutputReprojectionError(pImgPt, m_p3, pRms, strFile);
	/*
	memcpy(m_pGCam[0].gMatrix, pM,  sizeof(double) * 12);
	sprintf_s(strFile, "%s\\CalibratedProjectiveMatrix.txt", m_strOut);
	OutputLeafPtFile(strFile);
	*/
	memcpy(pM, m_pGCam[0].gMatrix, sizeof(double) * 12);
	
	CAM *pCam = new CAM[nImgNum];
	CAM cam0 = gcam.camPara;
	Mat matR, matQ; EOP eop;
	double pT[9] = { 0 }; double pK[9] = { 0 }; double pTVec[3] = { 0 }; double pRMat[9] = { 0 };
	double *pPQR_Q = new double[nImgNum * 9];
	ExtractPMatrixBenchMark(nImgNum, pLeafPt, pT, pPQR_Q);
	ExtractRotationMatrixFromGMatrixbyQRFactorization(pM, pT, pRMat, pK, pTVec);

	CAM cam = m_pGCam[0].camPara;
	cam.fx = float(pK[0]);
	cam.fy = float(pK[4]);
	cam.x0 = float(-pK[2]);
	cam.y0 = float(-pK[5]);
//	cam.s = float(pK[1]);

	EOP *pNewEop = new EOP[nImgNum];
	for (i = 0; i < nImgNum; i++){
		ExtractEOPFromVirtualRotationAndTranslation(pLeafPt + i * 4, pPQR_Q + i * 9, pRMat, pK, pTVec, eop);
		pNewEop[i] = eop;
	}
	/*
	for (i = 0; i < nImgNum; i++){
		EOP eop; CAM cam;
		ExtractEOPandIOPFromGMatrix(pM, pLeafPt + i*gcam.leafNum, eop, cam);
		cam.s = 0;
		m_pImg[i] = eop;
		pCam[i] = cam;
	}
	*/
	
	//Estimate distortion
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*
	m_CameraUnkFlag[0] = 0;		//f
	m_CameraUnkFlag[1] = 0;		//x0
	m_CameraUnkFlag[2] = 0;		//y0
	m_CameraUnkFlag[3] = 1;		//k1
	m_CameraUnkFlag[4] = 1;		//k2
	m_CameraUnkFlag[5] = 0;		//k3
	m_CameraUnkFlag[6] = 0;		//p1
	m_CameraUnkFlag[7] = 0;		//p2
	m_CameraUnkFlag[17] = 0;	//s
	OptimizeDistortionParameters_ProjectiveTransformation_GCamera_GCP(nImgPtNum, pP2, pVirtualPt, pM, cam, pRms);
	*/
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	k = 0;
//	gcam.camPara = cam;
	for (i = 0; i < nObjPtNum; i++){
		PT3D pt3 = m_p3[i];
		if (m_p3[i].nAttrib < 0){
			continue;
		}

		for (int j = 0; j < pt3.nIPtNum; j++){

			PT2D pt2 = pP2[pt3.nIPtSID + j];
			if (pt2.nAttrib < 0) continue;
			EOP eop = pNewEop[pt2.nImgID];
			double x, y;
			BF.GetxyWithXYZ(gcam.camPara, eop, pt3, x, y);

			pRms[2 * k + 0] = pt2.x - cam.x0 - x;
			pRms[2 * k + 1] = pt2.y - cam.y0 - y;
			k++;
		}
	}
	
	sprintf_s(strFile, "%s\\ImagePointResiduals_B-%.1f_P-%.1f_I-%.1f.txt", m_strOut, m_BaselineScale, m_LeafError, m_ImgPtError);
	OutputReprojectionError(pImgPt, m_p3, pRms, strFile);

	PT3D *ptp3 = new PT3D[nObjPtNum];
	memcpy(ptp3, m_p3, sizeof(PT3D)*nObjPtNum);
	for (i = 0; i < nObjPtNum; i++){
		if (ptp3[i].nAttrib < 0){
			continue;
		}
		ptp3[i].nAttrib = 0;
	}

	BF.MultiInterSection(nObjPtNum, ptp3, pImgPt, pNewEop, &m_pGCam[0].camPara);

	double *pErr = new double[nObjPtNum * 3];

	for (i = 0; i < nObjPtNum; i++){
		pErr[i * 3 + 0] = m_p3[i].X - ptp3[i].X;
		pErr[i * 3 + 1] = m_p3[i].Y - ptp3[i].Y;
		pErr[i * 3 + 2] = m_p3[i].Z - ptp3[i].Z;

	}


//	gcam.camPara = cam;
//	sprintf_s(strFile, "%s\\ResultPrj_B-%.1f_P-%.1f_I-%.1f.adj", m_strOut, m_BaselineScale, m_LeafError, m_ImgPtError);
//	OutPutResultFile(strFile, m_nImgNum, m_nGCamNum, m_nObjPtNum, pNewEop, &gcam, ptp3, m_p2);

	sprintf_s(strFile, "%s\\EstimatedParaError_B-%.1f_P-%.1f_I-%.1f.txt", m_strOut, m_BaselineScale, m_LeafError, m_ImgPtError);
	OutputEstimatedParaError(m_nGCamNum, &gcam.camPara, m_nImgNum, pNewEop, m_nObjPtNum, ptp3, strFile);
	//Bundle adjustment
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	m_CameraUnkFlag[0] = 0;		//f
	m_CameraUnkFlag[1] = 1;		//x0
	m_CameraUnkFlag[2] = 1;		//y0
	m_CameraUnkFlag[3] = 1;		//k1
	m_CameraUnkFlag[4] = 1;		//k2
	m_CameraUnkFlag[5] = 0;		//k3
	m_CameraUnkFlag[6] = 0;		//p1
	m_CameraUnkFlag[7] = 0;		//p2
	m_CameraUnkFlag[17] = 0;	//s
	m_bEOP = true;
	if (!m_bIOP){
		m_CameraUnkFlag[0] = 0;		//f
		m_CameraUnkFlag[1] = 0;		//x0
		m_CameraUnkFlag[2] = 0;		//y0
		m_CameraUnkFlag[3] = 0;		//k1
		m_CameraUnkFlag[4] = 0;		//k2
		m_CameraUnkFlag[5] = 0;		//k3
		m_CameraUnkFlag[6] = 0;		//p1
		m_CameraUnkFlag[7] = 0;		//p2
		m_CameraUnkFlag[17] = 0;	//s
	}
	double *pBaRms = new double[m_nImgPtNum * 2 + 4];
	memset(pBaRms, 0, sizeof(double)*(m_nImgPtNum * 2 + 4));

	m_bSetBenchMarkImage = false;
	/*
	if (m_bSetBenchMarkImage){
		int nMainDirection = 0;
		FindBestBenchMarkImages(m_nImgNum, m_pImg, m_nBenchMarkID1, m_nBenchMarkID2, m_nMain_Direction);
	}
	
	pNewEop[m_nBenchMarkID1] = m_pImg[m_nBenchMarkID1];
	if (m_nMain_Direction == 0) pNewEop[m_nBenchMarkID2].Xs = m_pImg[m_nBenchMarkID2].Xs;
	if (m_nMain_Direction == 1) pNewEop[m_nBenchMarkID2].Ys = m_pImg[m_nBenchMarkID2].Ys;
	if (m_nMain_Direction == 2) pNewEop[m_nBenchMarkID2].Zs = m_pImg[m_nBenchMarkID2].Zs;
	*/

	BundleAdjustment(pImgPt, ptp3, pNewEop, &m_pGCam[0].camPara, pBaRms);
	/*
	double pSimi[12] = { 0 };
	PT3D *ptSrc = new PT3D[nObjPtNum];
	PT3D *ptDst = new PT3D[nObjPtNum];
	ptSrc[0] = ptp3[0];  ptDst[0] = m_p3[0];
	ptSrc[1] = ptp3[15]; ptDst[1] = m_p3[15];
	ptSrc[2] = ptp3[18]; ptDst[2] = m_p3[18];
	ptSrc[3] = ptp3[29]; ptDst[3] = m_p3[29];
	ptSrc[4] = ptp3[64]; ptDst[4] = m_p3[64];
	ptSrc[5] = ptp3[74]; ptDst[5] = m_p3[74];
//	memcpy(ptSrc, ptp3, sizeof(PT3D)*nObjPtNum);
//	memcpy(ptDst, m_p3, sizeof(PT3D)*nObjPtNum);
	BF.CalcSevenParameters_linear(6, ptSrc, ptDst, pSimi);
	for (i = 0; i < nImgNum; i++){
		double pSrc[3] = { 0 }, pDst[3] = { 0 };
		pSrc[0] = pNewEop[i].Xs;
		pSrc[1] = pNewEop[i].Ys;
		pSrc[2] = pNewEop[i].Zs;
		pDst[0] = m_pImg[i].Xs;
		pDst[1] = m_pImg[i].Ys;
		pDst[2] = m_pImg[i].Zs;
		BF.TransPointsWithSevenParameters(1, pSimi, pSrc, pDst, pErr);
		pNewEop[i].Xs = pDst[0];
		pNewEop[i].Ys = pDst[1];
		pNewEop[i].Zs = pDst[2];
		pSrc[0] = pNewEop[i].Ph;
		pSrc[1] = pNewEop[i].Om;
		pSrc[2] = pNewEop[i].Kp;
		pDst[0] = m_pImg[i].Ph;
		pDst[1] = m_pImg[i].Om;
		pDst[2] = m_pImg[i].Kp;
		BF.TransRotationWithSevenParameters(1, pSimi, pSrc, pDst, pErr);

		pNewEop[i].Ph = pDst[0];
		pNewEop[i].Om = pDst[1];
		pNewEop[i].Kp = pDst[2];
	}
	delete[] ptSrc; ptSrc = NULL;
	delete[] ptDst; ptDst = NULL;
	
	BF.MultiInterSection(nObjPtNum, ptp3, pImgPt, pNewEop, &m_pGCam[0].camPara);
	double rms = 0;
	ComputeResiduals_BundleAdjustment(pImgPt, ptp3, pNewEop, &m_pGCam[0].camPara, rms, pBaRms);
	*/
//	gcam.camPara = cam;
	sprintf_s(strFile, "%s\\ResultPrj_BundleAdjustment_B-%.1f_P-%.1f_I-%.1f.adj", m_strOut, m_BaselineScale, m_LeafError, m_ImgPtError);
	OutPutResultFile(strFile, m_nImgNum, m_nGCamNum, m_nObjPtNum, pNewEop, &gcam, ptp3, m_p2);
	sprintf_s(strFile, "%s\\ImagePointResiduals_BundleAdjustment_B-%.1f_P-%.1f_I-%.1f.txt", m_strOut, m_BaselineScale, m_LeafError, m_ImgPtError);
	OutputReprojectionError(pImgPt, ptp3, pBaRms, strFile);
	
	delete[] pBaRms; pBaRms = NULL;

	for (i = 0; i < nObjPtNum; i++){
		pErr[i * 3 + 0] = m_p3[i].X - ptp3[i].X;
		pErr[i * 3 + 1] = m_p3[i].Y - ptp3[i].Y;
		pErr[i * 3 + 2] = m_p3[i].Z - ptp3[i].Z;
	}

	sprintf_s(strFile, "%s\\EstimatedParaError_BundleAdjustment_B-%.1f_P-%.1f_I-%.1f.txt", m_strOut, m_BaselineScale, m_LeafError, m_ImgPtError);
	OutputEstimatedParaError(m_nGCamNum, &gcam.camPara, m_nImgNum, pNewEop, m_nObjPtNum, ptp3, strFile);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	delete[] pErr; pErr = NULL;
	delete[] ptp3; ptp3 = NULL;
	delete[] pRms; pRms = NULL;
	delete[] pP2; pP2 = NULL;
	return 1;
}
int CGeoCamera::OutputLeafPtFile(char *strFile)
{
	int i, j, k;

	FILE * fp = NULL;
	fopen_s(&fp, strFile, "w");
	if (fp == NULL){
		printf("Create result file failed:%s!\n", strFile);
		return 0;
	}

	fprintf(fp, "%d %d %d\n", m_nImgNum, m_nGCamNum, m_nLeafPtNum);
	for (i = 0; i < m_nGCamNum; i++){
		fprintf(fp, "%d %d ", i, m_pGCam[i].leafNum);
		for (j = 0; j < 12; j++){
			fprintf(fp, "%12.9lf ", m_pGCam[i].gMatrix[j]);
		}
		fprintf(fp, "\n");
	}
	k = 0;
	for (i = 0; i < m_nImgNum; i++){
		fprintf(fp, "%d %d\n", i, m_pCamID[i]);
		int nLeafNum = m_pGCam[m_pCamID[i]].leafNum;
		for (j = 0; j < nLeafNum; j++){
			fprintf(fp, "%s %20.6lf %20.6lf %20.6lf\n", m_pLeafPt[k].name, m_pLeafPt[k].Y, m_pLeafPt[k].X, m_pLeafPt[k].Z);
			k++;
		}
	}
	fclose(fp);
	return 1;
}
int CGeoCamera::OutputEstimatedParaError(int nCamNum, CAM *pCam, int nImgNum, EOP *pNewEop, int nObjPtNum, PT3D *p3, char *strFile)
{
	int i;
	FILE *fp = NULL;
	fopen_s(&fp, strFile, "w");
	CBasicFunction BF;
	for (i = 0; i < nCamNum; i++){
		CAM cam = pCam[i];
		cam.fx /= m_ImgPt_Scale;
		cam.fy /= m_ImgPt_Scale;
		cam.x0 /= m_ImgPt_Scale;
		cam.y0 /= m_ImgPt_Scale;
		cam.k1 *= pow(m_ImgPt_Scale, 2);
		cam.k2 *= pow(m_ImgPt_Scale, 4);
		cam.k3 *= pow(m_ImgPt_Scale, 6);
		cam.k4 *= pow(m_ImgPt_Scale, 8);
		cam.k5 *= pow(m_ImgPt_Scale, 10);
		cam.p1 *= pow(m_ImgPt_Scale, 2);
		cam.p2 *= pow(m_ImgPt_Scale, 2);
		fprintf(fp, "%8d\t%8d\t%8d\t%20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf\n",
			m_pGCam[i].leafNum, m_pGCam[i].camPara.nWidth, m_pGCam[i].camPara.nHeight, cam.fx, cam.fy, cam.x0, cam.y0, 0.0, cam.k1, cam.k2, cam.k3, cam.p1, cam.p2, 0, 0);
		CAM dCam;
		dCam.fx = pCam[i].fx - m_pGCam[i].camPara.fx;
		dCam.fy = pCam[i].fy - m_pGCam[i].camPara.fy;
		dCam.x0 = pCam[i].x0 - m_pGCam[i].camPara.x0;
		dCam.y0 = pCam[i].y0 - m_pGCam[i].camPara.y0;
		dCam.k1 = pCam[i].k1 - m_pGCam[i].camPara.k1;
		dCam.k2 = pCam[i].k2 - m_pGCam[i].camPara.k2;
		dCam.k3 = pCam[i].k3 - m_pGCam[i].camPara.k3;
		dCam.p1 = pCam[i].p1 - m_pGCam[i].camPara.p1;
		dCam.p2 = pCam[i].p2 - m_pGCam[i].camPara.p2;
		fprintf(fp, "%8d\t%8d\t%8d\t%20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf\n",
			m_pGCam[i].leafNum, m_pGCam[i].camPara.nWidth, m_pGCam[i].camPara.nHeight, dCam.fx, dCam.fy, dCam.x0, dCam.y0, 0.0, dCam.k1, dCam.k2, dCam.k3, dCam.p1, dCam.p2, 0, 0);
	}
	double meanX = 0, meanY = 0, meanZ = 0, mXYZ = 0, meanPh = 0, meanOm = 0, meanKp = 0;
	double sqsumX = 0, sqsumY = 0, sqsumZ = 0, sqXYZ = 0, sqPh = 0, sqOm = 0, sqKp = 0;
	for (i = 0; i < nImgNum; i++){
		EOP eop = pNewEop[i];
		eop.Xs /= m_ObjPt_Scale;
		eop.Ys /= m_ObjPt_Scale;
		eop.Zs /= m_ObjPt_Scale;

		fprintf(fp, "%-20.10lf %20.10lf %20.10lf %15.10lf %15.10lf %15.10lf\n", eop.Xs, eop.Ys, eop.Zs, eop.Ph, eop.Om, eop.Kp);
		EOP dEop;
		dEop.Xs = pNewEop[i].Xs - m_pImg[i].Xs;
		dEop.Ys = pNewEop[i].Ys - m_pImg[i].Ys;
		dEop.Zs = pNewEop[i].Zs - m_pImg[i].Zs;
		dEop.Ph = pNewEop[i].Ph - m_pImg[i].Ph;
		dEop.Om = pNewEop[i].Om - m_pImg[i].Om;
		dEop.Kp = pNewEop[i].Kp - m_pImg[i].Kp;

		meanX += dEop.Xs;
		meanY += dEop.Ys;
		meanZ += dEop.Zs;
		meanPh += dEop.Ph;
		meanOm += dEop.Om;
		meanKp += dEop.Kp;

		sqsumX += dEop.Xs*dEop.Xs;
		sqsumY += dEop.Ys*dEop.Ys;
		sqsumZ += dEop.Zs*dEop.Zs;
		sqPh += dEop.Ph*dEop.Ph;
		sqOm += dEop.Om*dEop.Om;
		sqKp += dEop.Kp*dEop.Kp;
		fprintf(fp, "%-20.10lf %20.10lf %20.10lf %15.10lf %15.10lf %15.10lf\n", dEop.Xs, dEop.Ys, dEop.Zs, dEop.Ph, dEop.Om, dEop.Kp);
	}
	meanX /= nImgNum;
	meanY /= nImgNum;
	meanZ /= nImgNum;
	meanPh /= nImgNum;
	meanOm /= nImgNum;
	meanKp /= nImgNum;
	sqsumX = sqrt(sqsumX / nImgNum);
	sqsumY = sqrt(sqsumY / nImgNum);
	sqsumZ = sqrt(sqsumZ / nImgNum);
	sqPh = sqrt(sqPh / nImgNum);
	sqOm = sqrt(sqOm / nImgNum);
	sqKp = sqrt(sqKp / nImgNum);
	fprintf(fp, "Mean:%20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf\n", meanX, meanY, meanZ, meanPh, meanOm, meanKp);
	fprintf(fp, "RMSE:%20.10lf %20.10lf %20.10lf %20.10lf %20.10lf %20.10lf\n", sqsumX, sqsumY, sqsumZ, sqPh, sqOm, sqKp);

	meanX = meanY = meanZ = mXYZ = 0;
	sqsumX = sqsumY = sqsumZ = sqXYZ = 0;
	for (i = 0; i < nObjPtNum; i++){
		PT3D pt3 = p3[i];
		PT3D dPt3;
		dPt3.X = p3[i].X - m_p3[i].X;
		dPt3.Y = p3[i].Y - m_p3[i].Y;
		dPt3.Z = p3[i].Z - m_p3[i].Z;

		meanX += dPt3.X;
		meanY += dPt3.Y;
		meanZ += dPt3.Z;
		mXYZ += sqrt(dPt3.X*dPt3.X + dPt3.Y*dPt3.Y + dPt3.Z*dPt3.Z);

		sqsumX += dPt3.X * dPt3.X;
		sqsumY += dPt3.Y * dPt3.Y;
		sqsumZ += dPt3.Z * dPt3.Z;
		sqXYZ += dPt3.X*dPt3.X + dPt3.Y*dPt3.Y + dPt3.Z*dPt3.Z;
		fprintf(fp, "%8s\t%20.10lf\t%20.10lf\t%20.10lf\t%20.10lf\t%20.10lf\t%20.10lf\t%d\n", pt3.name, pt3.X, pt3.Y, pt3.Z, dPt3.X, dPt3.Y, dPt3.Z, pt3.nAttrib);
	}
	meanX /= nObjPtNum;
	meanY /= nObjPtNum;
	meanZ /= nObjPtNum;
	mXYZ /= nObjPtNum;
	sqsumX = sqrt(sqsumX / nObjPtNum);
	sqsumY = sqrt(sqsumY / nObjPtNum);
	sqsumZ = sqrt(sqsumZ / nObjPtNum);
	sqXYZ = sqrt(sqXYZ / nObjPtNum);
	fprintf(fp, "Mean:%20.10lf %20.10lf %20.10lf %20.10lf\n", meanX, meanY, meanZ, mXYZ);
	fprintf(fp, "RMSE:%20.10lf %20.10lf %20.10lf %20.10lf\n", sqsumX, sqsumY, sqsumZ, sqXYZ);
	fclose(fp);
	return 1;
}
int CGeoCamera::OutPutResultFile(char *strFile, int nImgNum, int nCamNum, int nObjPtNum, EOP *pEop, GCAM *pGCam, PT3D *p3, PT2D *p2)
{
	int i, j, k;

	FILE * fp = NULL;
	fopen_s(&fp, strFile, "w");
	if (fp == NULL){
		printf("Create result file failed:%s!\n", strFile);
		return 0;
	}

	fprintf(fp, "%d %d %d\n", m_nImgNum, m_nGCamNum, m_nObjPtNum);

	for (i = 0; i < nCamNum; i++){

		CAM cam = pGCam[i].camPara;
		cam.fx /= m_ImgPt_Scale;
		cam.fy /= m_ImgPt_Scale;
		cam.x0 /= m_ImgPt_Scale;
		cam.y0 /= m_ImgPt_Scale;
		cam.k1 *= pow(m_ImgPt_Scale, 2);
		cam.k2 *= pow(m_ImgPt_Scale, 4);
		cam.k3 *= pow(m_ImgPt_Scale, 6);
		cam.k4 *= pow(m_ImgPt_Scale, 8);
		cam.k5 *= pow(m_ImgPt_Scale, 10);
		cam.p1 *= pow(m_ImgPt_Scale, 2);
		cam.p2 *= pow(m_ImgPt_Scale, 2);
		fprintf(fp, "%8d\t%8d\t%8d\t%20.10e %20.10e %20.10e %20.10e %20.10e %20.10e %20.10e %20.10e %20.10e %20.10e %20.10e %20.10e\n",
			pGCam[i].leafNum, cam.nWidth, cam.nHeight, cam.fx, cam.fy, cam.x0, cam.y0, 0.0, cam.k1, cam.k2, cam.k3, cam.p1, cam.p2, 0, 0);
	}
	CBasicFunction BF;
	int nImgID = 0, nCamID = 0; EOP eop;
	for (i = 0; i < nImgNum; i++){
		double a[9] = { 0 };

		eop = pEop[i];
		eop.Xs /= m_ObjPt_Scale;
		eop.Ys /= m_ObjPt_Scale;
		eop.Zs /= m_ObjPt_Scale;
		BF.GetRotateMatrixWithAngle(a, eop.Ph, eop.Om, eop.Kp);
		fprintf(fp, "%d %d %d\n", i, nCamID, 0);
		fprintf(fp, "%s\n", m_strImagePath + i*MAX_FILE_PATH_LENGTH);
		fprintf(fp, "%-20.10e %20.10e %20.10e %20.10e %20.10e %20.10e %20.10e %20.10e %20.10e\n", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8]);
		fprintf(fp, "%-20.10lf %20.10lf %20.10lf %15.10lf %15.10lf %15.10lf\n", eop.Xs, eop.Ys, eop.Zs, eop.Ph, eop.Om, eop.Kp);

	}
	k = 0;
	double dx, dy, dxy, sum = 0, sqsum = 0, max, min, err[4] = { 0 };
	for (i = 0; i < nObjPtNum; i++){
		PT3D pt3 = p3[i];
		PT2D *pP2 = new PT2D[pt3.nIPtNum];
		memset(pP2, 0, sizeof(PT2D)*pt3.nIPtNum);

		for (j = 0; j < pt3.nIPtNum; j++){
			pP2[j] = p2[pt3.nIPtSID + j];
		}

		fprintf(fp, "%8s\t%20.10e\t%20.10e\t%20.10e\t%d\n", pt3.name, pt3.X, pt3.Y, pt3.Z, pt3.nAttrib);
		fprintf(fp, "%d\n", pt3.nIPtNum);

		for (j = 0; j < pt3.nIPtNum; j++){

			PT2D pt2 = pP2[j];
			if (pt2.nImgID < 0 || pt2.nImgID > m_nImgNum - 1 || pt2.nCamID < 0 || pt2.nCamID > m_nGCamNum - 1){
				printf("Warning!Image point %d of the control point %d %s is from invalid image %d %d!\n", j, i, pt3.name, pt2.nImgID, pt2.nCamID);
				return 0;
			}
			else{
				EOP eop = pEop[pt2.nImgID];
				CAM cam = pGCam[pt2.nCamID].camPara;
				BF.GetxyResWithXYZ(cam, eop, pt3, pt2, dx, dy, true);
				dxy = dx*dx + dy*dy;
				if (k == 0){ max = min = dxy; }
				sum += sqrt(dxy);
				sqsum += dxy;
				if (dxy > max) { max = dxy; }
				if (dxy < min) { min = dxy; }
				k++;
			}
			pt2.x = pt2.x + float(pGCam[0].camPara.nWidth / 2.0);
			pt2.y = float(pGCam[0].camPara.nHeight / 2.0) - pt2.y;
			pt2.x /= m_ImgPt_Scale;
			pt2.y /= m_ImgPt_Scale;
			dx /= m_ImgPt_Scale;
			dy /= m_ImgPt_Scale;
			fprintf(fp, "%8d\t%12.6lf\t%12.6lf\t%12.6lf\t%12.6lf\n", pt2.nImgID, pt2.x, pt2.y, dx, dy);
		}
		delete[] pP2; pP2 = NULL;
	}
	if (k > 0){
		err[0] = sum / k;
		err[1] = sqrt(sqsum / k);
		err[2] = max;
		err[3] = min;
	}
	fprintf(fp, "Mean of reprojection error: %lf\n", err[0]);
	fprintf(fp, "rms of reprojection error: %lf\n", err[1]);
	fprintf(fp, "max of reprojection error: %lf\n", err[2]);
	fprintf(fp, "min of reprojection error: %lf\n", err[3]);

	fclose(fp);
	return 1;
}
int CGeoCamera::OutPutResultFile(char *strFile)
{
	return OutPutResultFile(strFile, m_nImgNum, m_nGCamNum, m_nObjPtNum, m_pImg, m_pGCam, m_p3, m_p2);

}
int CGeoCamera::FindBestBenchMarkImages(int nImgNum, EOP *pEop, int &nBenchMarkID1, int &nBenchMarkID2, int &nMainDirection)
{
	int i = 0;
	int maxXID = 0, maxYID = 0, maxZID = 0, minXID = 0, minYID = 0, minZID = 0;
	double maxX, maxY, maxZ, minX, minY, minZ;
	for (i = 0; i < nImgNum; i++){
		if (i == 0){
			maxX = minX = pEop[i].Xs;
			maxY = minY = pEop[i].Ys;
			maxZ = minZ = pEop[i].Zs;
		}
		if (pEop[i].Xs > maxX){ maxX = pEop[i].Xs; maxXID = i; }
		if (pEop[i].Ys > maxY){ maxY = pEop[i].Ys; maxYID = i; }
		if (pEop[i].Zs > maxZ){ maxZ = pEop[i].Zs; maxZID = i; }
		if (pEop[i].Xs < minX){ minX = pEop[i].Xs; minXID = i; }
		if (pEop[i].Ys < minY){ minY = pEop[i].Ys; minYID = i; }
		if (pEop[i].Zs < minZ){ minZ = pEop[i].Zs; minZID = i; }
	}
	double dX, dY, dZ;
	dX = maxX - minX;
	dY = maxY - minY;
	dZ = maxZ - minZ;

	if (dX > dY && dX > dZ){
		nMainDirection = 0;
		nBenchMarkID1 = minXID;
		nBenchMarkID2 = maxXID;
	}
	else if (dY > dX && dY > dZ){
		nMainDirection = 1;
		nBenchMarkID1 = minYID;
		nBenchMarkID2 = maxYID;
	}
	else{
		nMainDirection = 2;
		nBenchMarkID1 = minZID;
		nBenchMarkID2 = maxZID;
	}
	return 1;
}
int CGeoCamera::SolveNormalEquationWithLM_RelativeOrientation(int nPtNum, int nUnkNum, double *pAtA, double *pAtL, CAM &cam, EOP *pEop, double rms, double &rmsImprove, double *pX, double &maxV)
{
	int count = 0;
	int nMaxIteration = 100;
	double res1 = 1, res2 = 0;
	double dRes1 = 0, dRes2 = 0;
	double coefInterval = 2;
	double coef = 0;

	CXMatrix maU1, maU2;
	double ImproveThreshold = 1e-6;
	double LM_Threshold = 1e-6;
	CXMatrix maAtA, maAtL, maX;
	maAtL.InitMatrix(pAtL, nUnkNum, 1);

	CAM tcam = cam;
	EOP *ptEop = new EOP[m_nImgNum * 6];
	res1 = res2 = rms;
	m_DampCoef = 1e-6;
	do{
		if (AddDampingCoef(coef, nUnkNum, pAtA) == 0){
			return 0;
		}
		maAtA.InitMatrix(pAtA, nUnkNum, nUnkNum);
		maU2 = maAtA.InverseMatrix()*maAtL;

		maxV = maU2.GetMaxFabsElement();

		tcam = cam;
		memcpy(ptEop, pEop, sizeof(EOP)*m_nImgNum * 6);
		UpdateUnknowns_RelativeOrientation(maU2.GetData(), tcam, ptEop);
		ComputeResiduals_RelativeOrientation(tcam, nPtNum, m_p2, m_p3, ptEop, res2);
		dRes2 = res2 - res1;

		printf("LM Iteration:%d %20.9e %20.9e %20.9e %20.9e %20.9e %20.9e %20.9e\n", count, maxV, coef, res1, res2, dRes1, dRes2, rms);

		if ((dRes2 == 0 && dRes1 == 0) || ((res1 - rms)<0 && dRes2 > 0 && dRes1 < 0)){

			if (count == 0){
				maU1 = maU2;
				res1 = res2;
			}
			maX = maU1;

			maxV = maX.GetMaxFabsElement();
			m_DampCoef = coef / coefInterval;
			rmsImprove = rms - res1;
			break;
		}
		else{
			if (coef == 0){ if (m_DampCoef == 0) coef = 1e-6; else coef = m_DampCoef; }
			else{
				coef *= coefInterval;
				m_DampCoef = coef;
			}
		}
		res1 = res2;
		dRes1 = dRes2;
		maU1 = maU2;
		count++;
	} while (count < nMaxIteration);

	delete[] ptEop; ptEop = NULL;
	if (nMaxIteration>10 && count == nMaxIteration){
		printf("Failed to solve normal equation via LM algorithm\n");

		return 0;
	}

	printf("Max correction:%.9lf, Residuals improvement:%.9lf\n", maxV, rmsImprove);

	memcpy(pX, maX.GetData(), sizeof(double)*nUnkNum);
	return 1;
}
int CGeoCamera::UpdateUnknowns_RelativeOrientation(double *p, CAM &cam, EOP *pEop)
{
	int i = 0, k = 0;
	double w1, w2, w3, dPh, dOm, dKp;
	CAM tcam = cam;
	if (m_CameraUnkFlag[0]) { cam.fx += float(p[k]); k++; }
	if (m_CameraUnkFlag[0]) { cam.fy += float(p[k]); k++; }
	if (m_CameraUnkFlag[1]) { cam.x0 += float(p[k]); k++; }
	if (m_CameraUnkFlag[2]) { cam.y0 += float(p[k]); k++; }
	if (m_CameraUnkFlag[17]) { cam.s += float(p[k]); k++; }
	/*
	cam.k1 *= pow(cam.fx / tcam.fx, 2);
	cam.k2 *= pow(cam.fx / tcam.fx, 4);
	cam.p1 *= cam.fx / tcam.fx;
	cam.p2 *= cam.fx / tcam.fx;
	*/
	if (m_CameraUnkFlag[3]) { cam.k1 += p[k]; k++; }
	if (m_CameraUnkFlag[4]) { cam.k2 += p[k]; k++; }
	if (m_CameraUnkFlag[5]) { cam.k3 += p[k]; k++; }
	if (m_CameraUnkFlag[6]) { cam.p1 += p[k]; k++; }
	if (m_CameraUnkFlag[7]) { cam.p2 += p[k]; k++; }

	for (i = 0; i < m_nImgNum; i++){
		if (i == m_nBenchMarkID1){ continue; }
		else if (i == m_nBenchMarkID2){
			if (m_nMain_Direction != 0){ pEop[i].Xs += p[k]; k++; }
			if (m_nMain_Direction != 1){ pEop[i].Ys += p[k]; k++; }
			if (m_nMain_Direction != 2){ pEop[i].Zs += p[k]; k++; }
		}
		else{
			pEop[i].Xs += p[k]; k++;
			pEop[i].Ys += p[k]; k++;
			pEop[i].Zs += p[k]; k++;
		}

		EOP eop = pEop[i];
		w1 = p[k]; k++;
		w2 = p[k]; k++;
		w3 = p[k]; k++;

		dPh = -w1*sin(eop.Ph)*tan(eop.Om) + w2 + w3*cos(eop.Ph)*tan(eop.Om);
		dOm = -w1*cos(eop.Ph) - w3*sin(eop.Ph);
		dKp = w1*sin(eop.Ph) / cos(eop.Om) - w3*cos(eop.Ph) / cos(eop.Om);

		pEop[i].Ph += dPh;
		pEop[i].Om += dOm;
		pEop[i].Kp += dKp;

		/*
		pEop[i].Ph += p[k]; k++;
		pEop[i].Om += p[k]; k++;
		pEop[i].Kp += p[k]; k++;
		*/
	}
	return 1;
}
int CGeoCamera::ComputeResiduals_RelativeOrientation(CAM &cam, int nPtNum, PT2D *p2, PT3D *p3, EOP *pEop, double &res, double *pRms)
{
	int i, j, k, l = 0;
	int nlSRow = 0, nlSCol = 0, nrSRow = 0, nrSCol = 0;
	PT2D pt2l, pt2r; PT3D pt3;  EOP eop[2]; double b, sum = 0, sqsum = 0;
	for (i = 0; i < nPtNum; i++){
		pt3 = p3[i];
		for (j = 0; j < pt3.nIPtNum; j++){
			pt2l = p2[pt3.nIPtSID + j];
			eop[0] = pEop[pt2l.nImgID];
			for (k = j + 1; k < pt3.nIPtNum; k++){
				pt2r = p2[pt3.nIPtSID + k];
				eop[1] = pEop[pt2r.nImgID];
				ComputeErrorEquation_RelativeOrientation(pt2l, pt2r, cam, eop, NULL, NULL, &b);
				if (pRms){ pRms[l] = b; }
				sum += b;
				sqsum += b * b;
				l++;
			}
		}
	}
	res = sqrt(sqsum / l);

	return 1;
}
int CGeoCamera::ComputeErrorEquation_RelativeOrientation(PT2D pt2l, PT2D pt2r, CAM &cam, EOP *pEop, double *A, double *B, double *L)
{
	int k = 0;
	CBasicFunction BF;
	EOP eop1, eop2; CAM cam1, cam2; PT2D p2l, p2r;
	eop1 = pEop[0]; eop2 = pEop[1];

	double fx, fy, x0, y0, s, x1, y1, x2, y2, r1, r2, dx, dy;
	double Bx, By, Bz, lx, ly, lz, rx, ry, rz;
	double a[9], b[9];

	p2l = pt2l; p2r = pt2r;
	BF.ComputeDistortionWithDistortedPoints_Normalized(pt2l.x, pt2l.y, cam, dx, dy);
	p2l.x = pt2l.x - float(dx);
	p2l.y = pt2l.y - float(dy);
	BF.ComputeDistortionWithDistortedPoints_Normalized(pt2r.x, pt2r.y, cam, dx, dy);
	p2r.x = pt2r.x - float(dx);
	p2r.y = pt2r.y - float(dy);

	BF.GetRotateMatrixWithAngle(a, eop1.Ph, eop1.Om, eop1.Kp);
	BF.GetRotateMatrixWithAngle(b, eop2.Ph, eop2.Om, eop2.Kp);
	x0 = cam.x0;
	y0 = cam.y0;
	fx = cam.fx;
	fy = cam.fy;
	s = cam.s;

	x1 = ((p2l.x - x0) + s / fy*(p2l.y - y0)) / fx;
	y1 = (p2l.y - y0) / fy;
	x2 = ((p2r.x - x0) + s / fy*(p2r.y - y0)) / fx;
	y2 = (p2r.y - y0) / fy;

	r1 = x1*x1 + y1*y1;
	r2 = x2*x2 + y2*y2;
	Bx = eop2.Xs - eop1.Xs;
	By = eop2.Ys - eop1.Ys;
	Bz = eop2.Zs - eop1.Zs;

	lx = a[0] * x1 + a[1] * y1 - a[2];
	ly = a[3] * x1 + a[4] * y1 - a[5];
	lz = a[6] * x1 + a[7] * y1 - a[8];

	rx = b[0] * x2 + b[1] * y2 - b[2];
	ry = b[3] * x2 + b[4] * y2 - b[5];
	rz = b[6] * x2 + b[7] * y2 - b[8];


	if (A || B){
		double x1_fx, y1_fx, x2_fx, y2_fx, x1_fy, y1_fy, x2_fy, y2_fy, x1_x0, y1_x0, x2_x0, y2_x0, x1_y0, y1_y0, x2_y0, y2_y0, x1_s, y1_s, x2_s, y2_s;
		x1_fx = -x1 / fx; y1_fx = 0; x2_fx = -x2 / fx; y2_fx = 0;
		x1_fy = -s / fy / fy*(p2l.y - y0) / fx; y1_fy = -y1 / fy; x2_fy = -s / fy / fy*(p2r.y - y0) / fx; y2_fy = -y2 / fy;
		x1_x0 = -1 / fx; y1_x0 = 0; x2_x0 = -1 / fx; y2_x0 = 0;
		x1_y0 = -s / fy / fx; y1_y0 = -1 / fy; x2_y0 = -s / fy / fx; y2_y0 = -1 / fy;
		x1_s = fy*(p2l.y - y0) / fx; y1_s = 0; x2_s = fy*(p2r.y - y0) / fx; y2_s = 0;
		double x1_k1, y1_k1, x2_k1, y2_k1, x1_p1, y1_p1, x2_p1, y2_p1, x1_p2, y1_p2, x2_p2, y2_p2;
		x1_k1 = -x1*r1 - s / fx*y1*r1;
		y1_k1 = -y1*r1;
		x2_k1 = -x2*r2 - s / fx*y2*r2;
		y2_k1 = -y2*r2;

		x1_p1 = -2 * x1*y1 - s / fx*(r1 + 2 * y1*y1);
		y1_p1 = -(r1 + 2 * y1*y1);
		x2_p1 = -2 * x2*y2 - s / fx*(r2 + 2 * y2*y2);
		y2_p1 = -(r2 + 2 * y2*y2);

		x1_p2 = -(r1 + 2 * x1*x1) - s / fx * 2 * x1*y1;
		y1_p2 = -2 * x1*y1;
		x2_p2 = -(r2 + 2 * x2*x2) - s / fx * 2 * x2*y2;
		y2_p2 = -2 * x2*y2;
		double lx_fx, ly_fx, lz_fx, rx_fx, ry_fx, rz_fx;
		double lx_fy, ly_fy, lz_fy, rx_fy, ry_fy, rz_fy;
		double lx_x0, ly_x0, lz_x0, rx_x0, ry_x0, rz_x0;
		double lx_y0, ly_y0, lz_y0, rx_y0, ry_y0, rz_y0;
		double lx_s, ly_s, lz_s, rx_s, ry_s, rz_s;
		lx_fx = a[0] * x1_fx + a[1] * y1_fx;
		ly_fx = a[3] * x1_fx + a[4] * y1_fx;
		lz_fx = a[6] * x1_fx + a[7] * y1_fx;
		rx_fx = b[0] * x2_fx + b[1] * y2_fx;
		ry_fx = b[3] * x2_fx + b[4] * y2_fx;
		rz_fx = b[6] * x2_fx + b[7] * y2_fx;

		lx_fy = a[0] * x1_fy + a[1] * y1_fy;
		ly_fy = a[3] * x1_fy + a[4] * y1_fy;
		lz_fy = a[6] * x1_fy + a[7] * y1_fy;
		rx_fy = b[0] * x2_fy + b[1] * y2_fy;
		ry_fy = b[3] * x2_fy + b[4] * y2_fy;
		rz_fy = b[6] * x2_fy + b[7] * y2_fy;

		lx_x0 = a[0] * x1_x0 + a[1] * y1_x0;
		ly_x0 = a[3] * x1_x0 + a[4] * y1_x0;
		lz_x0 = a[6] * x1_x0 + a[7] * y1_x0;
		rx_x0 = b[0] * x2_x0 + b[1] * y2_x0;
		ry_x0 = b[3] * x2_x0 + b[4] * y2_x0;
		rz_x0 = b[6] * x2_x0 + b[7] * y2_x0;

		lx_y0 = a[0] * x1_y0 + a[1] * y1_y0;
		ly_y0 = a[3] * x1_y0 + a[4] * y1_y0;
		lz_y0 = a[6] * x1_y0 + a[7] * y1_y0;
		rx_y0 = b[0] * x2_y0 + b[1] * y2_y0;
		ry_y0 = b[3] * x2_y0 + b[4] * y2_y0;
		rz_y0 = b[6] * x2_y0 + b[7] * y2_y0;

		lx_s = a[0] * x1_s + a[1] * y1_s;
		ly_s = a[3] * x1_s + a[4] * y1_s;
		lz_s = a[6] * x1_s + a[7] * y1_s;
		rx_s = b[0] * x2_s + b[1] * y2_s;
		ry_s = b[3] * x2_s + b[4] * y2_s;
		rz_s = b[6] * x2_s + b[7] * y2_s;

		double lx_k1, ly_k1, lz_k1, rx_k1, ry_k1, rz_k1;
		lx_k1 = a[0] * x1_k1 + a[1] * y1_k1;
		ly_k1 = a[3] * x1_k1 + a[4] * y1_k1;
		lz_k1 = a[6] * x1_k1 + a[7] * y1_k1;
		rx_k1 = b[0] * x2_k1 + b[1] * y2_k1;
		ry_k1 = b[3] * x2_k1 + b[4] * y2_k1;
		rz_k1 = b[6] * x2_k1 + b[7] * y2_k1;

		double lx_p1, ly_p1, lz_p1, rx_p1, ry_p1, rz_p1;
		double lx_p2, ly_p2, lz_p2, rx_p2, ry_p2, rz_p2;
		lx_p1 = a[0] * x1_p1 + a[1] * y1_p1;
		ly_p1 = a[3] * x1_p1 + a[4] * y1_p1;
		lz_p1 = a[6] * x1_p1 + a[7] * y1_p1;
		rx_p1 = b[0] * x2_p1 + b[1] * y2_p1;
		ry_p1 = b[3] * x2_p1 + b[4] * y2_p1;
		rz_p1 = b[6] * x2_p1 + b[7] * y2_p1;

		lx_p2 = a[0] * x1_p2 + a[1] * y1_p2;
		ly_p2 = a[3] * x1_p2 + a[4] * y1_p2;
		lz_p2 = a[6] * x1_p2 + a[7] * y1_p2;
		rx_p2 = b[0] * x2_p2 + b[1] * y2_p2;
		ry_p2 = b[3] * x2_p2 + b[4] * y2_p2;
		rz_p2 = b[6] * x2_p2 + b[7] * y2_p2;

		if (p2l.nImgID == m_nBenchMarkID2){
			if (m_nMain_Direction != 0){ A[k] = -ly*rz + lz*ry; k++; }
			if (m_nMain_Direction != 1){ A[k] = -lz*rx + lx*rz; k++; }
			if (m_nMain_Direction != 2){ A[k] = -lx*ry + ly*rx; k++; }
		}
		else{
			A[k] = -ly*rz + lz*ry; k++;
			A[k] = -lz*rx + lx*rz; k++;
			A[k] = -lx*ry + ly*rx; k++;
		}
		A[k] = (Bx*(lz*rz + ly*ry) + By*(-ly*rx - 0 * rz) + Bz*(0 * ry - lz*rx)); k++;
		A[k] = (Bx*(0 * rz - lx*ry) + By*(lx*rx + lz*rz) + Bz*(-lz*ry - 0 * rx)); k++;
		A[k] = (Bx*(-lx*rz - 0 * ry) + By*(0 * rx - ly*rz) + Bz*(ly*ry + lx*rx)); k++;

		k = 0;

		if (p2r.nImgID == m_nBenchMarkID2){
			if (m_nMain_Direction != 0){ A[6 + k] = ly*rz - lz*ry; k++; }
			if (m_nMain_Direction != 1){ A[6 + k] = lz*rx - lx*rz; k++; }
			if (m_nMain_Direction != 2){ A[6 + k] = lx*ry - ly*rx; k++; }
		}
		else{
			A[6 + k] = ly*rz - lz*ry; k++;
			A[6 + k] = lz*rx - lx*rz; k++;
			A[6 + k] = lx*ry - ly*rx; k++;
		}
		A[6 + k] = -(Bx*(rz*lz + ry*ly) + By*(-ry*lx - 0 * lz) + Bz*(0 * ly - rz*lx)); k++;
		A[6 + k] = -(Bx*(0 * lz - rx*ly) + By*(rx*lx + rz*lz) + Bz*(-rz*ly - 0 * lx)); k++;
		A[6 + k] = -(Bx*(-rx*lz - 0 * ly) + By*(0 * lx - ry*lz) + Bz*(ry*ly + rx*lx)); k++;

		k = 0;
		if (m_CameraUnkFlag[0] == 1) { B[k] = Bx*(ly_fx*rz - lz_fx*ry) + By*(lz_fx*rx - lx_fx*rz) + Bz*(lx_fx*ry - ly_fx*rx); k++; }
		if (m_CameraUnkFlag[0] == 1) { B[k] = Bx*(ly_fy*rz - lz_fy*ry) + By*(lz_fy*rx - lx_fy*rz) + Bz*(lx_fy*ry - ly_fy*rx); k++; }
		if (m_CameraUnkFlag[1] == 1) { B[k] = Bx*(ly_x0*rz - lz_x0*ry) + By*(lz_x0*rx - lx_x0*rz) + Bz*(lx_x0*ry - ly_x0*rx); k++; }
		if (m_CameraUnkFlag[2] == 1) { B[k] = Bx*(ly_y0*rz - lz_y0*ry) + By*(lz_y0*rx - lx_y0*rz) + Bz*(lx_y0*ry - ly_y0*rx); k++; }
		if (m_CameraUnkFlag[17] == 1) { B[k] = Bx*(ly_s*rz - lz_s*ry) + By*(lz_s*rx - lx_s*rz) + Bz*(lx_s*ry - ly_s*rx); k++; }

		if (m_CameraUnkFlag[3] == 1) { B[k] = Bx*(ly_k1*rz - lz_k1*ry) + By*(lz_k1*rx - lx_k1*rz) + Bz*(lx_k1*ry - ly_k1*rx); k++; }
		if (m_CameraUnkFlag[4] == 1) { B[k] = r1*(Bx*(ly_k1*rz - lz_k1*ry) + By*(lz_k1*rx - lx_k1*rz) + Bz*(lx_k1*ry - ly_k1*rx)); k++; }
		if (m_CameraUnkFlag[5] == 1) { B[k] = r1*r1*(Bx*(ly_k1*rz - lz_k1*ry) + By*(lz_k1*rx - lx_k1*rz) + Bz*(lx_k1*ry - ly_k1*rx)); k++; }
		if (m_CameraUnkFlag[6] == 1) { B[k] = Bx*(ly_p1*rz - lz_p1*ry) + By*(lz_p1*rx - lx_p1*rz) + Bz*(lx_p1*ry - ly_p1*rx); k++; }
		if (m_CameraUnkFlag[7] == 1) { B[k] = Bx*(ly_p2*rz - lz_p2*ry) + By*(lz_p2*rx - lx_p2*rz) + Bz*(lx_p2*ry - ly_p2*rx); k++; }

		k = 0;
		if (m_CameraUnkFlag[0] == 1) { B[k] += Bx*(ly*rz_fx - lz*ry_fx) + By*(lz*rx_fx - lx*rz_fx) + Bz*(lx*ry_fx - ly*rx_fx); k++; }
		if (m_CameraUnkFlag[0] == 1) { B[k] += Bx*(ly*rz_fy - lz*ry_fy) + By*(lz*rx_fy - lx*rz_fy) + Bz*(lx*ry_fy - ly*rx_fy); k++; }
		if (m_CameraUnkFlag[1] == 1) { B[k] += Bx*(ly*rz_x0 - lz*ry_x0) + By*(lz*rx_x0 - lx*rz_x0) + Bz*(lx*ry_x0 - ly*rx_x0); k++; }
		if (m_CameraUnkFlag[2] == 1) { B[k] += Bx*(ly*rz_y0 - lz*ry_y0) + By*(lz*rx_y0 - lx*rz_y0) + Bz*(lx*ry_y0 - ly*rx_y0); k++; }
		if (m_CameraUnkFlag[17] == 1) { B[k] += Bx*(ly*rz_s - lz*ry_s) + By*(lz*rx_s - lx*rz_s) + Bz*(lx*ry_s - ly*rx_s); k++; }

		if (m_CameraUnkFlag[3] == 1) { B[k] += Bx*(ly*rz_k1 - lz*ry_k1) + By*(lz*rx_k1 - lx*rz_k1) + Bz*(lx*ry_k1 - ly*rx_k1); k++; }
		if (m_CameraUnkFlag[4] == 1) { B[k] += r2*(Bx*(ly*rz_k1 - lz*ry_k1) + By*(lz*rx_k1 - lx*rz_k1) + Bz*(lx*ry_k1 - ly*rx_k1)); k++; }
		if (m_CameraUnkFlag[5] == 1) { B[k] += r2*r2*Bx*(ly*rz_k1 - lz*ry_k1) + By*(lz*rx_k1 - lx*rz_k1) + Bz*(lx*ry_k1 - ly*rx_k1); k++; }
		if (m_CameraUnkFlag[6] == 1) { B[k] += Bx*(ly*rz_p1 - lz*ry_p1) + By*(lz*rx_p1 - lx*rz_p1) + Bz*(lx*ry_p1 - ly*rx_p1); k++; }
		if (m_CameraUnkFlag[7] == 1) { B[k] += Bx*(ly*rz_p2 - lz*ry_p2) + By*(lz*rx_p2 - lx*rz_p2) + Bz*(lx*ry_p2 - ly*rx_p2); k++; }
	}

	L[0] = Bx*(ly*rz - lz*ry) + By*(lz*rx - lx*rz) + Bz*(lx*ry - ly*rx);
	return 1;
}
int CGeoCamera::OptimizeCameraParameters_RelativeOrientation(int nPtNum, PT2D *p2, PT3D *p3, EOP *pEop, CAM &cam, double *pRms)
{


	int i = 0, j = 0, j1 = 0, j2 = 0;
	int nUnkNum = 0, nEquaUnkNum = 0, nCamUnkNum = 0;
	if (m_CameraUnkFlag[0]){
		nCamUnkNum += 2;
	}
	for (i = 1; i < 20; i++){
		nCamUnkNum += m_CameraUnkFlag[i];
	}
	int nImgUnkNum = 6; nEquaUnkNum = 6;
	nUnkNum = nCamUnkNum + nImgUnkNum*m_nImgNum;
	if (m_bSetBenchMarkImage)
	{
		nUnkNum = nCamUnkNum + nImgUnkNum*(m_nImgNum - 1) - 1;
	}

	int nObjUnkOffset = 0;
	int nObjUnkNum = 3;
	double A[64] = { 0 };
	double B[20] = { 0 };
	double b[2] = { 0 };
	double *AtA = new double[nUnkNum*nUnkNum];
	double *Atb = new double[nUnkNum];
	int k = 0, count = 0, nTotPtNum = 0;
	int nlSRow = 0, nlSCol = 0, nrSRow = 0, nrSCol = 0;
	int nlRow = 0, nlCol = 0, nrRow = 0, nrCol = 0;
	int max_iteration_num = 1000;
	double maxr = 1, w = 1, res = 100, sqsum = 0;
	double max_correction = 100, rmsImprove = 10000000000;
	double correct_threshold = 1e-9;
	double rmsImprove_threshold = 1e-9*m_ImgPt_Scale;
	double *ptM = NULL; PT2D pt2l, pt2r; PT3D pt3;  EOP eop[2]; CBasicFunction BF;
	maxr = sqrt(pow(cam.nWidth*m_ImgPt_Scale / 2, 2) + pow(cam.nHeight*m_ImgPt_Scale / 2, 2));


	do
	{
		sqsum = 0; nTotPtNum = 0;
		memset(AtA, 0, sizeof(double) * nUnkNum*nUnkNum);
		memset(Atb, 0, sizeof(double) * nUnkNum);
		for (i = 0; i < nPtNum; i++){
			pt3 = p3[i];
			for (j = 0; j < pt3.nIPtNum; j++){
				pt2l = p2[pt3.nIPtSID + j];
				eop[0] = pEop[pt2l.nImgID];
				GetBlockPosition(pt2l.nImgID, nImgUnkNum, m_nBenchMarkID1, m_nBenchMarkID2, nlSRow, nlSCol, nlRow, nlCol);
				nlSRow += nCamUnkNum;
				nlSCol += nCamUnkNum;
				for (k = j + 1; k < pt3.nIPtNum; k++){

					pt2r = p2[pt3.nIPtSID + k];
					eop[1] = pEop[pt2r.nImgID];
					ComputeErrorEquation_RelativeOrientation(pt2l, pt2r, cam, eop, A, B, b);
					b[0] = -b[0];
					sqsum += b[0] * b[0];
					nTotPtNum++;

					GetBlockPosition(pt2r.nImgID, nImgUnkNum, m_nBenchMarkID1, m_nBenchMarkID2, nrSRow, nrSCol, nrRow, nrCol);
					nrSRow += nCamUnkNum;
					nrSCol += nCamUnkNum;


					for (j1 = 0; j1 < nCamUnkNum; j1++){
						for (j2 = 0; j2 < nCamUnkNum; j2++){
							AtA[j1*nUnkNum + j2] += B[j1] * w * B[j2];
						}
						for (j2 = 0; j2 < nlCol; j2++){
							AtA[j1*nUnkNum + (nlSCol + j2)] += B[j1] * w * A[j2];
						}
						for (j2 = 0; j2 < nrCol; j2++){
							AtA[j1*nUnkNum + (nrSCol + j2)] += B[j1] * w * A[6 + j2];
						}
						Atb[j1] += B[j1] * w * b[0];
					}
					for (j1 = 0; j1 < nlRow; j1++){
						for (j2 = 0; j2 < nlCol; j2++){
							AtA[(nlSRow + j1)*nUnkNum + (nlSCol + j2)] += A[j1] * w * A[j2];
						}
						for (j2 = 0; j2 < nrCol; j2++){
							AtA[(nlSRow + j1)*nUnkNum + (nrSCol + j2)] += A[j1] * w * A[6 + j2];
						}
						for (j2 = 0; j2 < nCamUnkNum; j2++){
							AtA[(nlSRow + j1)*nUnkNum + j2] += A[j1] * w * B[j2];
						}
						Atb[nlSRow + j1] += A[j1] * w * b[0];
					}
					for (j1 = 0; j1 < nrRow; j1++){
						for (j2 = 0; j2 < nlCol; j2++){
							AtA[(nrSRow + j1)*nUnkNum + (nlSCol + j2)] += A[6 + j1] * w * A[j2];
						}
						for (j2 = 0; j2 < nrCol; j2++){
							AtA[(nrSRow + j1)*nUnkNum + (nrSCol + j2)] += A[6 + j1] * w * A[6 + j2];
						}
						for (j2 = 0; j2 < nCamUnkNum; j2++){
							AtA[(nrSRow + j1)*nUnkNum + j2] += A[6 + j1] * w * B[j2];
						}
						Atb[nrSRow + j1] += A[6 + j1] * w * b[0];
					}
				}
			}
		}
		rmsImprove = res - sqrt(sqsum / nTotPtNum);
		res = sqrt(sqsum / nTotPtNum);
		double *p = NULL; double maxV = 0;
		/*
		double coef = 1;
		for (j = 0; j < nCamUnkNum; j++){
		AtA[j*nUnkNum + j] += AtA[j*nUnkNum + j] * coef;
		}
		*/
		/*
		double *pX = new double[nUnkNum];
		memset(pX, 0, sizeof(double)*nUnkNum);
		SolveNormalEquationWithLM_RelativeOrientation(nPtNum, nUnkNum, AtA, Atb, cam, pEop, res, rmsImprove, pX, max_correction);
		p = pX;
		*/

		CXMatrix maAtA, maAtb, maX;
		maAtA.InitMatrix(AtA, nUnkNum, nUnkNum);
		maAtb.InitMatrix(Atb, nUnkNum, 1);
		maX = maAtA.InverseMatrix()*maAtb;
		max_correction = maX.GetMaxFabsElement();
		p = maX.GetData();

		printf("Iteration %d: rms, max correction, and rms improvement %20.10e %20.10e %20.10e\n", count, res, max_correction, rmsImprove);

		UpdateUnknowns_RelativeOrientation(p, cam, pEop);
		if (rmsImprove < rmsImprove_threshold){
			break;
		}
		count++;
	} while (fabs(max_correction) > correct_threshold && count < max_iteration_num);


	BF.MultiInterSection(m_nObjPtNum, p3, m_p2, pEop, &cam, 0);
	ComputeResiduals_BundleAdjustment(m_p2, p3, pEop, &cam, res, pRms);
	printf("Reprojection Error:%.6lf %.6lf %.6lf %.6lf\n", pRms[2 * m_nImgPtNum + 0], pRms[2 * m_nImgPtNum + 1], pRms[2 * m_nImgPtNum + 2], pRms[2 * m_nImgPtNum + 3]);
	delete[] AtA; AtA = NULL;
	delete[] Atb; Atb = NULL;

	return 1;
}
int CGeoCamera::BundleAdjustment(PT2D *p2, PT3D *p3, EOP *pEop, CAM *pCam, double *pRms)
{
	double rms = 0;


	int i = 0, count = 0, nCamParaNum = 0;

	if (m_CameraUnkFlag[0]) nCamParaNum = 2;
	for (i = 1; i < MAX_NUM_CAMERA_PARAM; i++){
		nCamParaNum += m_CameraUnkFlag[i];
	}
	m_nCamParaNum = nCamParaNum; m_nImgParaNum = 6;
	int nCamUnk = m_nGCamNum*m_nCamParaNum;
	int nImgUnk = m_nImgNum*m_nImgParaNum;
	if (m_bSetBenchMarkImage){
		nImgUnk = (m_nImgNum - 1)*m_nImgParaNum - 1;
	}
	if (!m_bEOP){
		nImgUnk = 0;
	}
	int nUnkNum = nImgUnk + nCamUnk;

	double *pAtA = new double[nUnkNum*nUnkNum];
	double *pAtL = new double[nUnkNum];
	double *pX = new double[nUnkNum];
	memset(pX, 0, sizeof(double)*nUnkNum);

	int nMaxIterNum = 1000;
	double rmsImprove = 1, maxCorrect = 1;
	double rmsImprove_threshold = 1e-5*m_ImgPt_Scale;
	double Correction_threshold = 1e-9;
	char strFile[MAX_FILE_PATH_LENGTH] = { 0 };
	double last_rms = 0;
	do{
		
		memset(pAtA, 0, sizeof(double)*nUnkNum*nUnkNum);
		memset(pAtL, 0, sizeof(double)*nUnkNum);

		if (BuildingNormalEquation_BundleAdjustment(p2, p3, pEop, pCam, nUnkNum, pAtA, pAtL, rms) == 0){
			break;
		}
	//	AddDampingCoef(1, nUnkNum, pAtA);
		//	AddDampingCoef(100, nUnkNum - 2, nUnkNum, nUnkNum, pAtA);
		
		CXMatrix maAtA, maAtb, maX;
		maAtA.InitMatrix(pAtA, nUnkNum, nUnkNum);
		/////////////////////////////////////////////////////////////////
		Mat cvATA(nUnkNum, nUnkNum, CV_64F, pAtA);
		Mat U, D, Vt;
		SVD::compute(cvATA, D, U, Vt);
		double *pD = (double*)D.data;
		/////////////////////////////////////////////////////////////////
		/*
		maAtb.InitMatrix(pAtL, nUnkNum, 1);
		maX = maAtA.InverseMatrix()*maAtb;
		maxCorrect = maX.GetMaxFabsElement();
		pX = maX.GetData();
		*/
		SolveNormalEquationWithLM_BundleAdjustment(p2, p3, nUnkNum, pAtA, pAtL, pEop, pCam, rms, rmsImprove, pX, maxCorrect);
		UpdateUnknowns_BundleAdjustment(pX, pEop, pCam, p3);
		last_rms = rms;
		ComputeResiduals_BundleAdjustment(p2, p3, pEop, pCam, rms, pRms);

		rmsImprove = last_rms - rms;

		/*
		char strFile[512] = { 0 };
		sprintf_s(strFile, "%s\\Corrections_%d.txt", m_strOut, count);
		CXMatrix maU;
		maU.InitMatrix(pX, nUnkNum, 1);
		maU.OutPutMatrix(strFile);
		*/
		count++;
	} while (fabs(rmsImprove) > rmsImprove_threshold && fabs(maxCorrect) > Correction_threshold && count < nMaxIterNum);



	delete[]pX; pX = NULL;
	delete[] pAtA; pAtA = NULL;
	delete[] pAtL; pAtL = NULL;

	return 1;
}

int CGeoCamera::OutputReprojectionError(PT2D *p2, PT3D *p3, double *pRms, char *strFile)
{
	int i, k, nImgID;
	FILE *fp = NULL, *fpc = NULL;
	fopen_s(&fp, strFile, "w");
	char strPerImg[MAX_FILE_PATH_LENGTH] = { 0 };
	double dxy, sumx = 0, sumy = 0, sum = 0, sqsum = 0, max, min;
	k = 0;
	for (i = 0; i < m_nImgNum; i++){
		sprintf_s(strPerImg, "%s\\Residuals_%d.txt", m_strOut, i);
		fopen_s(&fpc, strPerImg, "w");
		int nPtNum = m_pImgPtNum[i];
		int nOffset = m_pImgPtIDOffset[i];
		PT2D *tp2 = new PT2D[nPtNum];
		for (int j = 0; j < nPtNum; j++){
			int nPtID = m_pImgPtID[nOffset + j];
			PT2D pt2 = m_p2[nPtID];
			nImgID = pt2.nImgID;
			PT3D pt3 = p3[pt2.nPtsID];
			pRms[2 * nPtID + 0] /= m_ImgPt_Scale;
			pRms[2 * nPtID + 1] /= m_ImgPt_Scale;
			dxy = sqrt(pRms[2 * nPtID + 0] * pRms[2 * nPtID + 0] + pRms[2 * nPtID + 1] * pRms[2 * nPtID + 1]);
			if (k == 0){
				max = min = sqrt(dxy);
			}
			if (dxy > max){ max = dxy; }
			if (dxy < min){ min = dxy; }
			sum += dxy;
			sqsum += dxy*dxy;
			k++;
			fprintf_s(fp, "%8d %8d %12.6f %12.6f %12.6lf %12.6lf %12.6lf %12.6lf\n", p2[i].nPtsID, nImgID, pt3.X, pt3.Y, pt2.x, pt2.y, pRms[2 * nPtID + 0], pRms[2 * nPtID + 1]);
			if (fpc) fprintf_s(fpc, "%8d %8d %12.6f %12.6f %12.6lf %12.6lf %12.6lf %12.6lf\n", p2[i].nPtsID, nImgID, pt3.X, pt3.Y, pt2.x, pt2.y, pRms[2 * nPtID + 0], pRms[2 * nPtID + 1]);
		}
		if (fpc) fclose(fpc);
	}
	if (m_nImgPtNum != k){
		printf("Warning!Image point number is unexpected!\n");
	}
	/*
	pRms[2 * m_nImgPtNum + 0] = sum / k;
	pRms[2 * m_nImgPtNum + 1] = sqrt(sqsum / k);
	pRms[2 * m_nImgPtNum + 2] = max;
	pRms[2 * m_nImgPtNum + 3] = min;
	*/
	double sm = sum / k;
	double sq = sqrt(sqsum / k);
	fprintf(fp, "MEAN:%-12.6lf\n", sm);
	fprintf(fp, "RMSE:%-12.6lf\n", sq);
	fprintf(fp, "MAX:%-12.6lf\n", max);
	fprintf(fp, "MIN:%-12.6lf\n", min);
	fclose(fp);
	return 1;
}
int CGeoCamera::GetErrorEquation_BundleAdjustment(bool model, PT2D &p2, PT3D &p3, EOP &eop, CAM &cam, double * pA, double *pB, double *pC, double *pT, double *pL)
{
	CBasicFunction BF;

	double pR[9];
	BF.GetRotateMatrixWithAngle(pR, eop.Ph, eop.Om, eop.Kp);
	double x, y, x_u, y_u, fx, fy, r, X_, Y_, Z_;

	X_ = pR[0] * (p3.X - eop.Xs) + pR[3] * (p3.Y - eop.Ys) + pR[6] * (p3.Z - eop.Zs);
	Y_ = pR[1] * (p3.X - eop.Xs) + pR[4] * (p3.Y - eop.Ys) + pR[7] * (p3.Z - eop.Zs);
	Z_ = pR[2] * (p3.X - eop.Xs) + pR[5] * (p3.Y - eop.Ys) + pR[8] * (p3.Z - eop.Zs);

	if (Z_ == 0)
		return 0;

	if (BF.CorrectDistortion(model, p2.x, p2.y, cam, x_u, y_u) == 0){
		return 0;
	}
	fx = cam.fx;
	fy = cam.fy;

	x = (x_u - cam.x0) + cam.s / fy*(y_u - cam.y0);
	y = (y_u - cam.y0);


	r = x*x + y*y;
	int k = 0;
	if (pA){
		if (p2.nImgID == m_nBenchMarkID2){
			if (m_nMain_Direction != 0){ pA[k] = (pR[0] * fx + pR[2] * x) / Z_; k++; }
			if (m_nMain_Direction != 1){ pA[k] = (pR[3] * fx + pR[5] * x) / Z_; k++; }
			if (m_nMain_Direction != 2){ pA[k] = (pR[6] * fx + pR[8] * x) / Z_; k++; }
		}
		else{
			pA[k] = (pR[0] * fx + pR[2] * x) / Z_; k++;
			pA[k] = (pR[3] * fx + pR[5] * x) / Z_; k++;
			pA[k] = (pR[6] * fx + pR[8] * x) / Z_; k++;
		}
		/*
		if (p2.nImgID != m_nBenchMarkID2) { pA[k] = (pR[0] * fx + pR[2] * x) / Z_; k++; }
		pA[k] = (pR[3] * fx + pR[5] * x) / Z_; k++;
		pA[k] = (pR[6] * fx + pR[8] * x) / Z_; k++;
		*/
		pA[k] = y*sin(eop.Om) - (x / fx * (x*cos(eop.Kp) - y*sin(eop.Kp)) + fx*cos(eop.Kp)) * cos(eop.Om); k++;
		pA[k] = -fx*sin(eop.Kp) - x / fx * (x*sin(eop.Kp) + y*cos(eop.Kp)); k++;
		pA[k] = y; k++;


		if (p2.nImgID == m_nBenchMarkID2){
			if (m_nMain_Direction != 0){ pA[k] = (pR[1] * fy + pR[2] * y) / Z_; k++; }
			if (m_nMain_Direction != 1){ pA[k] = (pR[4] * fy + pR[5] * y) / Z_; k++; }
			if (m_nMain_Direction != 2){ pA[k] = (pR[7] * fy + pR[8] * y) / Z_; k++; }
		}
		else{
			pA[k] = (pR[1] * fy + pR[2] * y) / Z_; k++;
			pA[k] = (pR[4] * fy + pR[5] * y) / Z_; k++;
			pA[k] = (pR[7] * fy + pR[8] * y) / Z_; k++;
		}
		/*
		if (p2.nImgID != m_nBenchMarkID2){ pA[k] = (pR[1] * fy + pR[2] * y) / Z_; k++; }
		pA[k] = (pR[4] * fy + pR[5] * y) / Z_; k++;
		pA[k] = (pR[7] * fy + pR[8] * y) / Z_; k++;
		*/
		pA[k] = -x*sin(eop.Om) - (y / fy * (x*cos(eop.Kp) - y*sin(eop.Kp)) - fy*sin(eop.Kp)) * cos(eop.Om); k++;
		pA[k] = -fy*cos(eop.Kp) - y / fy * (x*sin(eop.Kp) + y*cos(eop.Kp)); k++;
		pA[k] = -x; k++;
	}
	k = 0;
	if (pB){

		double dr, dx_x0, dy_y0;
		x /= fx;
		y /= fy;
		r = x*x + y*y;
		dr = cam.k1*r + cam.k2*r*r + cam.k3*pow(r, 3) + cam.k4*pow(r, 4) + cam.k5*pow(r, 5) + cam.k6*pow(r, 6) + cam.k7*pow(r, 7) + cam.k8*pow(r, 8) + cam.k9*pow(r, 9);
		dx_x0 = -(dr + (2 * cam.p1*y + cam.p2 * 4 * x)*(1 + cam.p3*r) + cam.s1*r);
		dy_y0 = -(dr + (2 * cam.p2*x + cam.p1 * 4 * y)*(1 + cam.p3*r) + cam.s2*r);

		if (m_CameraUnkFlag[0]) { pB[k] = (x_u - cam.x0) + cam.s / fy*(y_u - cam.y0) / fx; k++; }
		if (m_CameraUnkFlag[0]) { pB[k] = 0; k++; }
		if (m_CameraUnkFlag[1]) { pB[k] = 1 + dx_x0; k++; }
		if (m_CameraUnkFlag[2]) { pB[k] = -cam.s / fy*dx_x0; k++; }
		if (m_CameraUnkFlag[17]) { pB[k] = -(y_u - cam.y0); k++; }

		if (m_CameraUnkFlag[3]) { pB[k] = fx*x*r + cam.s*y*r; k++; }
		if (m_CameraUnkFlag[4]) { pB[k] = fx*x*r*r + cam.s*y*r*r; k++; }
		if (m_CameraUnkFlag[5]) { pB[k] = fx*x*r*r*r + cam.s*y*r*r*r; k++; }
		if (m_CameraUnkFlag[6]) { pB[k] = fx*(r + 2 * x*x) + cam.s * 2 * x*y; k++; }
		if (m_CameraUnkFlag[7]) { pB[k] = fx * 2 * x*y + cam.s * (r + 2 * y*y); k++; }
		if (m_CameraUnkFlag[8]) { pB[k] = fx*r*(2 * cam.p1*x*y + cam.p2*(r + 2 * x*x)) + cam.s*r*(2 * cam.p2*x*y + cam.p1*(r + 2 * y*y)); k++; }

		if (m_CameraUnkFlag[9]) { pB[k] = fx*x*pow(r, 4) + cam.s*y*pow(r, 4); k++; }
		if (m_CameraUnkFlag[10]) { pB[k] = fx*x*pow(r, 5) + cam.s*y*pow(r, 5); k++; }
		if (m_CameraUnkFlag[11]) { pB[k] = fx*x*pow(r, 6) + cam.s*y*pow(r, 6); k++; }
		if (m_CameraUnkFlag[12]) { pB[k] = fx*x*pow(r, 7) + cam.s*y*pow(r, 7); k++; }
		if (m_CameraUnkFlag[13]) { pB[k] = fx*x*pow(r, 8) + cam.s*y*pow(r, 8); k++; }
		if (m_CameraUnkFlag[14]) { pB[k] = fx*x*pow(r, 9) + cam.s*y*pow(r, 9); k++; }

		if (m_CameraUnkFlag[0]) { pB[k] = 0; k++; }
		if (m_CameraUnkFlag[0]) { pB[k] = (y_u - cam.y0) / fy; k++; }
		if (m_CameraUnkFlag[1]) { pB[k] = 0; k++; }
		if (m_CameraUnkFlag[2]) { pB[k] = 1 + dy_y0; k++; }
		if (m_CameraUnkFlag[17]) { pB[k] = 0; k++; }

		if (m_CameraUnkFlag[3]) { pB[k] = fy*y*r; k++; }
		if (m_CameraUnkFlag[4]) { pB[k] = fy*y*r*r; k++; }
		if (m_CameraUnkFlag[5]) { pB[k] = fy*y*r*r*r; k++; }
		if (m_CameraUnkFlag[6]) { pB[k] = fy * 2 * x*y; k++; }
		if (m_CameraUnkFlag[7]) { pB[k] = fy*(r + 2 * y*y); k++; }
		if (m_CameraUnkFlag[8]) { pB[k] = fy*r*(2 * cam.p2*x*y + cam.p1*(r + 2 * y*y)); k++; }

		if (m_CameraUnkFlag[9]) { pB[k] = fy*y*pow(r, 4); k++; }
		if (m_CameraUnkFlag[10]) { pB[k] = fy*y*pow(r, 5); k++; }
		if (m_CameraUnkFlag[11]) { pB[k] = fy*y*pow(r, 6); k++; }
		if (m_CameraUnkFlag[12]) { pB[k] = fy*y*pow(r, 7); k++; }
		if (m_CameraUnkFlag[13]) { pB[k] = fy*y*pow(r, 8); k++; }
		if (m_CameraUnkFlag[14]) { pB[k] = fy*y*pow(r, 9); k++; }
	}
	if (pT){
		pT[0] = -pA[0];
		pT[1] = -pA[1];
		pT[2] = -pA[2];

		pT[3] = -pA[6];
		pT[4] = -pA[7];
		pT[5] = -pA[8];
	}
	x = x_u - cam.x0 + cam.s / fy*(y_u - cam.y0);
	y = y_u - cam.y0;
	if (pL){
		pL[0] = +(x + fx*X_ / Z_);
		pL[1] = +(y + fy*Y_ / Z_);
	}

	return 1;
}
int CGeoCamera::BuildingNormalEquation_BundleAdjustment(PT2D *p2, PT3D *p3, EOP *pEop, CAM *pCam, int nUnkNum, double *pAtA, double *pAtL, double &rms)
{
	int i, j, k;
	double pA[12] = { 0 };
	double pT[6] = { 0 };
	double pL[2] = { 0 };
	double pB[MAX_DIST_PARA_NUM] = { 0 };
	double pC[MAX_POLY_PARA_NUM] = { 0 };

	double sum = 0, sqsum = 0;
	PT3D pt3; PT2D pt2; CBasicFunction BF;
	CXMatrix maA, maB, maC, maP, maT, maL, maW;
	CXMatrix maATA, maBTB, maCTC, maATB, maATC, maBTC, maPTP, maPTL, maATL, maBTL, maCTL, maX;
	int nImgNum = m_nImgNum, nImgParaNum = 0, nTotImgUnkNum = 0;
	int nCamNum = m_nGCamNum, nCamParaNum = 0, nObjParaOffset = 0;
	int nImgPtNum = 0, nObjParaNum = 3, nValidCoefNum = 0, nObjPtNum = m_nObjPtNum;
	int sAR, sAC, AR, AC, sBR, sBC, BR, BC, sCR, sCC, CR, CC;
	nImgParaNum = m_nImgParaNum; nCamParaNum = m_nCamParaNum;
	nTotImgUnkNum = m_nImgNum*nImgParaNum;

	if (!m_bEOP){
		nTotImgUnkNum = 0;
	}

	if (m_bSetBenchMarkImage){
		nTotImgUnkNum = (m_nImgNum - 1)*nImgParaNum - 1;
	}
	for (i = 0; i < nObjPtNum; i++){
		pt3 = p3[i];
		nImgPtNum += pt3.nIPtNum;

		if (pt3.nAttrib < 0){
			continue;
		}
		nObjParaNum = 3;
		if (pt3.nAttrib == 2){
			nObjParaOffset = 1;
			nObjParaNum = 2;
		}
		CXMatrix * pmaXTY = new CXMatrix[pt3.nIPtNum];
		CXMatrix * pmaXTZ = new CXMatrix[pt3.nIPtNum];
		CXMatrix * pmaXTW = new CXMatrix[pt3.nIPtNum];
		//Update normal matrix at the image and camera parameter part
		//////////////////////////////////////////////////////////////////////////
		maW.InitUnitMatrix(2, 2);
		maPTP.InitZeroMatrix(nObjParaNum, nObjParaNum);
		maPTL.InitZeroMatrix(nObjParaNum, 1);


		for (j = 0; j < pt3.nIPtNum; j++){

			pt2 = p2[pt3.nIPtSID + j];
			if (pt2.nAttrib < 0){
				continue;
			}
			if (pt2.nImgID < 0 || pt2.nImgID > m_nImgNum - 1 || pt2.nCamID < 0 || pt2.nCamID > m_nGCamNum - 1){
				printf("Error!Image ID invalid:%d %d %s %d %d %d %d %d %d!\n", i, j, pt3.name, pt3.nAttrib, pt3.nIPtNum, pt3.nIPtSID, pt2.nAttrib, pt2.nImgID, pt2.nCamID);
				return 0;
			}
			CAM cam = pCam[pt2.nCamID];
			EOP eop = pEop[pt2.nImgID];
			memset(pA, 0, sizeof(double) * 12);
			if (GetErrorEquation_BundleAdjustment(true, pt2, pt3, eop, cam, pA, pB, pC, pT, pL) == 0){
				return 0;
			}

			double *pTMP = new double[nObjParaNum * 2];
			for (k = 0; k < nObjParaNum; k++){
				pTMP[k] = pT[nObjParaOffset + k];
				pTMP[nObjParaNum + k] = pT[3 + nObjParaOffset + k];
			}
			sqsum += pL[0] * pL[0] + pL[1] * pL[1];

			sAR = pt2.nImgID*nImgParaNum;
			sAC = pt2.nImgID*nImgParaNum;
			AR = nImgParaNum;
			AC = nImgParaNum;

			if (m_bSetBenchMarkImage){
				GetBlockPosition(pt2.nImgID, nImgParaNum, m_nBenchMarkID1, m_nBenchMarkID2, sAR, sAC, AR, AC);
			}

			maA.InitMatrix(pA, 2, AC);
			maB.InitMatrix(pB, 2, nCamParaNum);
			maP.InitMatrix(pTMP, 2, nObjParaNum);
			maL.InitMatrix(pL, 2, 1);

			maPTP = maPTP + maP.TransposeMatrix() * maW * maP;
			maPTL = maPTL + maP.TransposeMatrix() * maW * maL;

			maATA = maA.TransposeMatrix() * maW * maA;
			maATL = maA.TransposeMatrix() * maW * maL;
			pmaXTY[j] = maA.TransposeMatrix() * maW * maP;
			maBTB = maB.TransposeMatrix() * maW * maB;
			maBTL = maB.TransposeMatrix() * maW * maL;
			pmaXTZ[j] = maB.TransposeMatrix() * maW * maP;
			maATB = maA.TransposeMatrix() * maW * maB;



			sBR = nTotImgUnkNum + pt2.nCamID*nCamParaNum;
			sBC = nTotImgUnkNum + pt2.nCamID*nCamParaNum;
			BR = nCamParaNum;
			BC = nCamParaNum;

			sCR = nTotImgUnkNum + nCamNum*nCamParaNum;
			sCC = nTotImgUnkNum + nCamNum*nCamParaNum;
			CR = nValidCoefNum;
			CC = nValidCoefNum;

			if (m_bEOP){
				BF.UpdateNormalMatrix(maATA.GetData(), AR, AC, sAR, sAC, pAtA, nUnkNum, nUnkNum);
				BF.UpdateNormalMatrix(maATB.GetData(), AR, BC, sAR, sBC, pAtA, nUnkNum, nUnkNum);
				BF.UpdateNormalMatrix(maATB.TransposeMatrix().GetData(), BR, AC, sBR, sAC, pAtA, nUnkNum, nUnkNum);
				BF.UpdateNormalMatrix(maATL.GetData(), AR, 1, sAR, 0, pAtL, nUnkNum, 1);
			}


			BF.UpdateNormalMatrix(maBTB.GetData(), BR, BC, sBR, sBC, pAtA, nUnkNum, nUnkNum);
			BF.UpdateNormalMatrix(maBTL.GetData(), BR, 1, sBR, 0, pAtL, nUnkNum, 1);

		}
		//Eliminating object points unknowns
		//////////////////////////////////////////////////////////////////////////
		if (pt3.nAttrib > 0){

			delete[] pmaXTY; pmaXTY = NULL;
			delete[] pmaXTZ; pmaXTZ = NULL;
			delete[] pmaXTW; pmaXTW = NULL;
			continue;
		}
		/*
		double *ptp = new double[nObjParaNum*nObjParaNum];
		memcpy(ptp, maPTP.GetData(), sizeof(double) * nObjParaNum*nObjParaNum);
		if (pt3.nAttrib != 1){
		AddObjectPointZDampingCoef(100, nObjParaNum, ptp);
		}
		else{
		AddDampingCoef(1000, nObjParaNum, ptp);
		}
		maPTP.InitMatrix(ptp, nObjParaNum, nObjParaNum);
		*/


		CXMatrix maPT_P = maPTP.InverseMatrix();
		for (j = 0; j < pt3.nIPtNum; j++){

			PT2D p2j = p2[pt3.nIPtSID + j];
			if (p2j.nAttrib < 0){
				continue;
			}
			sAR = p2j.nImgID*nImgParaNum;
			AR = nImgParaNum;
			if (m_bSetBenchMarkImage){
				GetBlockPosition(p2j.nImgID, nImgParaNum, m_nBenchMarkID1, m_nBenchMarkID2, sAR, sAC, AR, AC);
			}

			sBR = nTotImgUnkNum + p2j.nCamID*nCamParaNum;
			BR = nCamParaNum;

			sCR = nTotImgUnkNum + nCamNum*nCamParaNum;
			CR = nValidCoefNum;
			CXMatrix maBTA, maCTA, maCTB;
			for (k = 0; k < pt3.nIPtNum; k++){
				PT2D p2k = p2[pt3.nIPtSID + k];
				if (p2k.nAttrib < 0){
					continue;
				}

				maATA = pmaXTY[j] * maPT_P * pmaXTY[k].TransposeMatrix();
				maATB = pmaXTY[j] * maPT_P * pmaXTZ[k].TransposeMatrix();
				maBTA = pmaXTZ[j] * maPT_P * pmaXTY[k].TransposeMatrix();
				maBTB = pmaXTZ[j] * maPT_P * pmaXTZ[k].TransposeMatrix();

				sAC = p2k.nImgID*nImgParaNum;
				AC = nImgParaNum;
				int tAR = 0, tsAR = 0;
				if (m_bSetBenchMarkImage){
					GetBlockPosition(p2k.nImgID, nImgParaNum, m_nBenchMarkID1, m_nBenchMarkID2, tsAR, sAC, tAR, AC);
				}

				sBC = nTotImgUnkNum + p2k.nCamID*nCamParaNum;
				BC = nCamParaNum;
				sCC = nTotImgUnkNum + nCamNum*nCamParaNum;
				CC = nValidCoefNum;

				if (m_bEOP){
					BF.UpdateAdvNormalMatrix(maATA.GetData(), AR, AC, sAR, sAC, pAtA, nUnkNum, nUnkNum);
					BF.UpdateAdvNormalMatrix(maATB.GetData(), AR, BC, sAR, sBC, pAtA, nUnkNum, nUnkNum);
					BF.UpdateAdvNormalMatrix(maBTA.GetData(), BC, AR, sBR, sAC, pAtA, nUnkNum, nUnkNum);
				}

				BF.UpdateAdvNormalMatrix(maBTB.GetData(), BR, BC, sBR, sBC, pAtA, nUnkNum, nUnkNum);

			}
			maATL = pmaXTY[j] * maPT_P * maPTL;
			maBTL = pmaXTZ[j] * maPT_P * maPTL;
			if (m_bEOP){
				BF.UpdateAdvNormalMatrix(maATL.GetData(), AR, 1, sAR, 0, pAtL, nUnkNum, 1);
			}
			BF.UpdateAdvNormalMatrix(maBTL.GetData(), BR, 1, sBR, 0, pAtL, nUnkNum, 1);

		}
		delete[] pmaXTY; pmaXTY = NULL;
		delete[] pmaXTZ; pmaXTZ = NULL;
		delete[] pmaXTW; pmaXTW = NULL;
	}
	rms = sqrt(sqsum / m_nImgPtNum);

	return 1;
}
int CGeoCamera::GetBlockPosition(int nImgID, int nImgParaNum, int nBenchMarkID1, int nBenchMarkID2, int &sR, int &sC, int &R, int &C)
{
	if (m_nBenchMarkID1 < m_nBenchMarkID2){
		if (nImgID < m_nBenchMarkID1){
			sR = nImgID*nImgParaNum;
			sC = nImgID*nImgParaNum;
			R = nImgParaNum;
			C = nImgParaNum;
		}
		else if (nImgID == m_nBenchMarkID1){
			sR = sC = 0;
			R = C = 0;
		}
		else if (nImgID > m_nBenchMarkID1 && nImgID < m_nBenchMarkID2){
			sR = (nImgID - 1)*nImgParaNum;
			sC = (nImgID - 1)*nImgParaNum;
			R = nImgParaNum;
			C = nImgParaNum;
		}
		else if (nImgID == m_nBenchMarkID2){
			sR = (nImgID - 1)*nImgParaNum;
			sC = (nImgID - 1)*nImgParaNum;
			R = nImgParaNum - 1;
			C = nImgParaNum - 1;
		}
		else if (nImgID > m_nBenchMarkID2){
			sR = (nImgID - 1)*nImgParaNum - 1;
			sC = (nImgID - 1)*nImgParaNum - 1;
			R = nImgParaNum;
			C = nImgParaNum;
		}
	}
	else{
		if (nImgID < m_nBenchMarkID2){
			sR = nImgID*nImgParaNum;
			sC = nImgID*nImgParaNum;
			R = nImgParaNum;
			C = nImgParaNum;
		}
		else if (nImgID == m_nBenchMarkID2){
			sR = nImgID*nImgParaNum;
			sC = nImgID*nImgParaNum;
			R = nImgParaNum - 1;
			C = nImgParaNum - 1;
		}
		else if (nImgID > m_nBenchMarkID2 && nImgID < m_nBenchMarkID1){
			sR = nImgID*nImgParaNum - 1;
			sC = nImgID*nImgParaNum - 1;
			R = nImgParaNum;
			C = nImgParaNum;
		}
		else if (nImgID == m_nBenchMarkID1){
			sR = sC = 0;
			R = C = 0;
		}
		else if (nImgID > m_nBenchMarkID1){
			sR = (nImgID - 1)*nImgParaNum - 1;
			sC = (nImgID - 1)*nImgParaNum - 1;
			R = nImgParaNum;
			C = nImgParaNum;
		}
	}

	return 1;
}
int CGeoCamera::SolveNormalEquationWithLM_BundleAdjustment(PT2D *p2, PT3D *p3, int nUnkNum, double *pAtA, double *pAtL, EOP *pEop, CAM *pCam, double rms, double &rmsImprove, double *pX, double &maxV)
{
	int count = 0;
	int nMaxIteration = 100;
	double res1 = 1, res2 = 0;
	double dRes1 = 0, dRes2 = 0;
	double coefInterval = 2;
	double coef = 1e-9;

	CXMatrix maU1, maU2;
	double ImproveThreshold = 1e-6;
	double LM_Threshold = 1e-6;
	CXMatrix maAtA, maAtL, maX;
	maAtL.InitMatrix(pAtL, nUnkNum, 1);
	int nObjPtNum = m_nObjPtNum;
	EOP *ptEop = new EOP[m_nImgNum];
	CAM *ptCam = new CAM[m_nGCamNum];
	PT3D *ptP3 = new PT3D[nObjPtNum];
	res1 = res2 = rms;
	coef = m_DampCoef;
	do{
		if (AddDampingCoef(coef, nUnkNum, pAtA) == 0){
			return 0;
		}
		maAtA.InitMatrix(pAtA, nUnkNum, nUnkNum);
		maU2 = maAtA.InverseMatrix()*maAtL;
		maxV = maU2.GetMaxFabsElement();
		memcpy(ptEop, pEop, sizeof(EOP)*m_nImgNum);
		memcpy(ptCam, pCam, sizeof(CAM)*m_nGCamNum);
		memcpy(ptP3, p3, sizeof(PT3D)*nObjPtNum);

		UpdateUnknowns_BundleAdjustment(maU2.GetData(), ptEop, ptCam, ptP3);

		ComputeResiduals_BundleAdjustment(p2, ptP3, ptEop, ptCam, res2);

		dRes2 = res2 - res1;

		printf("LM Iteration:%d %20.9e %20.9e %20.9e %20.9e %20.9e %20.9e %20.9e\n", count, maxV, coef, res1, res2, dRes1, dRes2, rms);

		if (/*count == 0 || */(dRes2 == 0 && dRes1 == 0) || ((res2 - rms)<0 && dRes2 > 0 && dRes1 < 0)){

			if (count == 0){
				maU1 = maU2;
				res1 = res2;

			}
			maX = maU1;

			maxV = maX.GetMaxFabsElement();
			m_DampCoef = coef / coefInterval;
			rmsImprove = rms - res1;
			break;
		}
		else{
			if (coef == 0){ if (m_DampCoef == 0) coef = 1e-9; else coef = m_DampCoef; }
			else{
				coef *= coefInterval;
				m_DampCoef = coef;
			}
		}
		res1 = res2;
		dRes1 = dRes2;
		maU1 = maU2;
		count++;
	} while (count < nMaxIteration);
	if (nMaxIteration>10 && count == nMaxIteration){
		printf("Failed to solve normal equation via LM algorithm\n");

		return 0;
	}

	printf("Max correction:%.9lf, Residuals improvement:%.9lf\n", maxV, rmsImprove);

	memcpy(pX, maX.GetData(), sizeof(double)*nUnkNum);

	delete[] ptEop; ptEop = NULL;
	delete[] ptCam; ptCam = NULL;
	delete[] ptP3; ptP3 = NULL;
	return 1;
}
int CGeoCamera::AddDampingCoef(double coef, int nRow, double *ATA)
{
	int i;
	for (i = 0; i < nRow; i++){
		ATA[i*nRow + i] += coef*ATA[i*nRow + i];
	}

	return 1;
}
int CGeoCamera::UpdateUnknowns_BundleAdjustment(double *p, EOP *pEop, CAM *pCam, PT3D *p3)
{
	int i, k;
	k = 0;
	if (m_bEOP){
		for (i = 0; i < m_nImgNum; i++){
			if (m_bSetBenchMarkImage && i == m_nBenchMarkID1){ continue; }
			else if (i == m_nBenchMarkID2){
				if (m_nMain_Direction != 0){ pEop[i].Xs += p[k]; k++; }
				if (m_nMain_Direction != 1){ pEop[i].Ys += p[k]; k++; }
				if (m_nMain_Direction != 2){ pEop[i].Zs += p[k]; k++; }
			}
			else{
				pEop[i].Xs += p[k]; k++;
				pEop[i].Ys += p[k]; k++;
				pEop[i].Zs += p[k]; k++;
			}
			pEop[i].Ph += p[k]; k++;
			pEop[i].Om += p[k]; k++;
			pEop[i].Kp += p[k]; k++;
			/*
			if (m_bSetBenchMarkImage && i == m_nBenchMarkID1){
				continue;
			}
			if (i != m_nBenchMarkID2 || !m_bSetBenchMarkImage){ pEop[i].Xs += p[k]; k++; }
			pEop[i].Ys += p[k]; k++;
			pEop[i].Zs += p[k]; k++;
			pEop[i].Ph += p[k]; k++;
			pEop[i].Om += p[k]; k++;
			pEop[i].Kp += p[k]; k++;
			*/

		}
	}


	for (i = 0; i < m_nGCamNum; i++){
		if (m_CameraUnkFlag[0]){ pCam[i].fx = float(pCam[i].fx + p[k]); k++; }
		if (m_CameraUnkFlag[0]){ pCam[i].fy = float(pCam[i].fy + p[k]); k++; }
		if (m_CameraUnkFlag[1]){ pCam[i].x0 = float(pCam[i].x0 + p[k]); k++; }
		if (m_CameraUnkFlag[2]){ pCam[i].y0 = float(pCam[i].y0 + p[k]); k++; }
		if (m_CameraUnkFlag[17]){ pCam[i].s = float(pCam[i].s + p[k]); k++; }
		if (m_CameraUnkFlag[3]){ pCam[i].k1 += p[k]; k++; }
		if (m_CameraUnkFlag[4]){ pCam[i].k2 += p[k]; k++; }
		if (m_CameraUnkFlag[5]){ pCam[i].k3 += p[k]; k++; }
		if (m_CameraUnkFlag[6]){ pCam[i].p1 += p[k]; k++; }
		if (m_CameraUnkFlag[7]){ pCam[i].p2 += p[k]; k++; }
		if (m_CameraUnkFlag[8]) { pCam[i].p3 += p[k]; k++; }

		if (m_CameraUnkFlag[9]) { pCam[i].k4 += p[k]; k++; }
		if (m_CameraUnkFlag[10]) { pCam[i].k5 += p[k]; k++; }
		if (m_CameraUnkFlag[11]) { pCam[i].k6 += p[k]; k++; }
		if (m_CameraUnkFlag[12]) { pCam[i].k7 += p[k]; k++; }
		if (m_CameraUnkFlag[13]) { pCam[i].k8 += p[k]; k++; }
		if (m_CameraUnkFlag[14]) { pCam[i].k9 += p[k]; k++; }

		if (m_CameraUnkFlag[15]) { pCam[i].s1 += p[k]; k++; }
		if (m_CameraUnkFlag[16]) { pCam[i].s2 += p[k]; k++; }
	}

	CBasicFunction BF;
	int CoorControl = 0;
	BF.MultiInterSection(m_nObjPtNum, p3, m_p2, pEop, pCam, CoorControl);

	return 1;
}
int CGeoCamera::ComputeResiduals_BundleAdjustment(PT2D *p2, PT3D *p3, EOP *pEop, CAM *pCam, double &rms, double *pRms)
{
	int i = 0, j = 0, k = 0;
	PT3D pt3; PT2D pt2;
	double pL[2] = { 0 };
	double sum = 0, sqsum = 0, max, min, dxy;
	int nObjPtNum = m_nObjPtNum;
	for (i = 0; i < nObjPtNum; i++){
		pt3 = p3[i];
		if (pt3.nAttrib < 0){
			continue;
		}
		for (j = 0; j < pt3.nIPtNum; j++){

			pt2 = p2[pt3.nIPtSID + j];
			if (pt2.nAttrib < 0){
				continue;
			}
			if (pt2.nImgID < 0 || pt2.nImgID > m_nImgNum - 1 || pt2.nCamID < 0 || pt2.nCamID > m_nGCamNum - 1){
				printf("Error!Image ID invalid:%d %d %s %d %d %d %d %d %d!\n", i, j, pt3.name, pt3.nAttrib, pt3.nIPtNum, pt3.nIPtSID, pt2.nAttrib, pt2.nImgID, pt2.nCamID);
				return 0;
			}
			CAM cam = pCam[pt2.nCamID];
			EOP eop = pEop[pt2.nImgID];
			GetErrorEquation_BundleAdjustment(true, pt2, pt3, eop, cam, NULL, NULL, NULL, NULL, pL);

			if (pRms){
				pRms[2 * k + 0] = pL[0];
				pRms[2 * k + 1] = pL[1];
			}
			dxy = sqrt(pL[0] * pL[0] + pL[1] * pL[1]);
			if (k == 0){ max = min = dxy; }

			if (dxy > max){ max = dxy; }
			if (dxy < min){ min = dxy; }
			sum += dxy;
			sqsum += dxy*dxy;

			k++;
		}
	}
	rms = sqrt(sqsum / k);
	if (pRms){
		pRms[2 * k + 0] = sum / k;
		pRms[2 * k + 1] = rms;
		pRms[2 * k + 2] = max;
		pRms[2 * k + 3] = min;
	}


	return 1;
}