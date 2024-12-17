#include "math.h"
#include "string.h"
#include "time.h"
#include "CBasicFunction.h"
#include <algorithm>
#include <Eigen/Dense>

using namespace Eigen;
CBasicFunction::CBasicFunction(void)
{
}


CBasicFunction::~CBasicFunction(void)
{
}

int CBasicFunction::GetAngleWithRotateMatrix( double *pR, double &ph, double &om, double &kp )
{
	ph = atan2(-pR[2],pR[8]);
	om = asin(-pR[5]);
	kp = atan2(pR[3],pR[4]);
	return 1;
}
int CBasicFunction::GetRotateMatrixWithAngle( double *pR, double ph, double om, double kp )
{
	double a_cos = cos(ph);
	double a_sin = sin(ph);
	double w_cos = cos(om);
	double w_sin = sin(om);
	double k_cos = cos(kp);
	double k_sin = sin(kp);
	pR[0] = a_cos*k_cos - a_sin*w_sin*k_sin;
	pR[1] = - a_cos*k_sin - a_sin*w_sin*k_cos;
	pR[2] = - a_sin * w_cos;
	pR[3] = w_cos*k_sin;
	pR[4] = w_cos*k_cos;
	pR[5] = - w_sin;
	pR[6] = a_sin*k_cos + a_cos*w_sin*k_sin;
	pR[7] = - a_sin*k_sin + a_cos*w_sin*k_cos;
	pR[8] = a_cos * w_cos;

	return 1;
}
int CBasicFunction::GetRotateMatrixWithUnitQuaternion( double *pR, double q0, double qx, double qy, double qz )
{
	pR[0] = q0*q0 - qx*qx - qy*qy - qz*qz;
	pR[1] = 2*(qx*qy - q0*qz );
	pR[2] = 2*(qx*qz + q0*qy );
	pR[3] = 2*(qy*qx + q0*qz );
	pR[4] = q0*q0 - qx*qx + qy*qy - qz*qz;
	pR[5] = 2*(qy*qz - q0*qx );
	pR[6] = 2*(qz*qx - q0*qy );
	pR[7] = 2*(qz*qy + q0*qx );
	pR[8] = q0*q0 - qx*qx - qy*qy + qz*qz;

	return 1;
}
int CBasicFunction::GetUnitQuaternionWithRotateMatrix( double *pR, double &q0, double &qx, double &qy, double &qz )
{
	double qxy = (pR[5]+pR[7])/(pR[2]+pR[6]);
	qx = sqrt( (pR[1]+pR[3])/4/qxy );
	qy = qx*qxy;
	qz = (pR[2]+pR[6])/4/qx;
	q0 = sqrt( 1-qx*qx-qy*qy-qz*qz );

	double a[9] = {0};
	GetRotateMatrixWithUnitQuaternion( a, q0, qx, qy, qz );

	return 1;

}
int CBasicFunction::GetAngleWithUnitQuaternion( double q0, double qx, double qy, double qz, double &ph, double &om, double &kp )
{
	double a[9] = {0};
	GetRotateMatrixWithUnitQuaternion( a,  q0, qx, qy, qz );
	GetAngleWithRotateMatrix( a, ph, om, kp ); 
	return 1;
}
int CBasicFunction::GetUnitQuaternionWithAngle( double &q0, double &qx, double &qy, double &qz, double ph, double om, double kp )
{
	double a[9] = {0};
	GetRotateMatrixWithAngle( a, ph, om, kp);
	GetUnitQuaternionWithRotateMatrix( a, q0, qx, qy, qz );
	return 1;
}


int CBasicFunction::SolveCubicEquation(double a, double b, double c, double d, int &nNum, double *x)
{
	double delta = 0;
	if (a == 0){
		if (b == 0){
			if (c == 0){ return 0; }
			else{ nNum = 1; x[0] = -d / c; return 1; }
		}
		delta = c*c - 4 * b*d;
		if (delta < 0) return 0;
		else if (delta == 0){
			nNum = 1;
			x[0] = -c / (2 * b);
		}
		else{
			nNum = 2;
			x[0] = (-c / +sqrt(delta)) / (2 * b);
			x[1] = (-c / -sqrt(delta)) / (2 * b);
		}
		return 1;
	}
	double p, q;

	p = (3 * a*c - b*b) / (3 * a*a);
	q = (27 * a*a*d - 9 * a*b*c + 2 * b*b*b) / (27 * a*a*a);

	delta = q*q / 4.0 + p*p*p / 27.0;

	double t1, t2, t3, t4;
	if (delta > 0){
		nNum = 1;
		double ts = pow(2, 0.3);
		t1 = -q / 2.0 + sqrt(delta);
		t2 = -q / 2.0 - sqrt(delta);
		t3 = pow(fabs(t1), 1.0/3);
		t4 = pow(fabs(t2), 1.0/3);
		if (t1 < 0) t3 = -t3;
		if (t2 < 0) t4 = -t4;
		x[0] = t3 + t4 - b / (3 * a);

		double dd = a*x[0] * x[0] * x[0] + b*x[0] * x[0] + c*x[0] + d;
		return 1;
	}
	else if (delta == 0){
		if (p == 0 && q == 0){
			nNum = 1;
			x[0] = -b / (3 * a);
			double dd = a*x[0] * x[0] * x[0] + b*x[0] * x[0] + c*x[0] + d;
			return 1;
		}
		else{
			nNum = 2;
			t1 = pow(fabs(-q / 2), 1.0/3.0);
			if (q > 0) t1 = -t1;
			x[0] = 2 * t1 - b / (3 * a);
			x[1] = -t1 - b / (3 * a);

			double dd = a*x[0] * x[0] * x[0] + b*x[0] * x[0] + c*x[0] + d;
			dd = a*x[1] * x[1] * x[1] + b*x[1] * x[1] + c*x[1] + d;
			return 1;
		}
	}
	else{
		nNum = 3;
		double r = sqrt(-p*p*p / 27);
		double alpha = 1.0 / 3.0*acos(-q / (2 * r));

		x[0] = 2 * pow(r, 1.0 / 3.0)*cos(alpha) - b / (3 * a);
		x[1] = 2 * pow(r, 1.0 / 3.0)*cos(alpha + 120.0 / 180 * PI) - b / (3 * a);
		x[2] = 2 * pow(r, 1.0 / 3.0)*cos(alpha + 240.0 / 180 * PI) - b / (3 * a);

		double dd = a*x[0] * x[0] * x[0] + b*x[0] * x[0] + c*x[0] + d;
		dd = a*x[1] * x[1] * x[1] + b*x[1] * x[1] + c*x[1] + d;
		dd = a*x[2] * x[2] * x[2] + b*x[2] * x[2] + c*x[2] + d;
		return 1;
	}

}
int CBasicFunction::GenerateGaussError(double mean, double sigma, double &err)
{

	static double V1, V2, S;
	static int phase = 0;
	double X;

	if (phase == 0) {
		do {
			double U1 = (double)rand() / RAND_MAX;
			double U2 = (double)rand() / RAND_MAX;

			V1 = 2 * U1 - 1;
			V2 = 2 * U2 - 1;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1 || S == 0);

		X = V1 * sqrt(-2 * log(S) / S);
	}
	else
		X = V2 * sqrt(-2 * log(S) / S);

	phase = 1 - phase;

	err = X * sigma + mean;

	return 1;

}
bool CBasicFunction::ValidateDistortionParameters(CAM &cam, double ImgPtScale)
{
	int i = 0, j = 0;
	double x, y, x_, y_, dx, dy, r, k1, k2, k3, p1, p2;
	double margin_x = 0.5, margin_y = 0.5;
	int inter_x = 1, inter_y = 1;
	int virtual_width = cam.nWidth + int(cam.nWidth*margin_x);
	int virtual_height = cam.nHeight + int(cam.nHeight*margin_y);
	k1 = cam.k1;
	k2 = cam.k2;
	k3 = cam.k3;
	p1 = cam.p1;
	p2 = cam.p2;
	double minx, miny, maxx, maxy;

	minx = -cam.nWidth / 2 * ImgPtScale;
	maxx = cam.nWidth / 2 * ImgPtScale;
	miny = -cam.nHeight / 2 * ImgPtScale;
	maxy = cam.nHeight / 2 * ImgPtScale;
	//For x direction
	//////////////////////////////////////////////////////////////
	for (i = 0; i < virtual_height; i += inter_y){
		y = (i - virtual_height / 2)*ImgPtScale;
		for (j = 0; j < virtual_width; j += inter_x){
			x = (j - virtual_width / 2)*ImgPtScale;
			r = x*x + y*y;
			x_ = x*(1 + k1*r + k2*r*r + k3*r*r*r + 2 * p1*y + 2 * p2*x) + p2*r*r;
			if (j == 0 && x_ > minx){
				printf("For x direction:%d %d %lf %lf\n", i, j, x_, minx);
				return false;
			}
			if (j == virtual_width - 1 && x_ < maxx){
				printf("For x direction:%d %d %lf %lf\n", i, j, x_, maxx);
				return false;
			}
			dx = 1 + k1*r + k2*r*r + k3*r*r*r + 2 * x*x*(k1 + 2 * k2*r + 3 * k3*r*r) + 2 * p1*y + 4 * p2*x + 4 * p2*r*x;
			if (dx < 0){
				printf("For x direction:%d %d %lf\n", i, j, dx);
				return false;
			}
		}
	}
	//For y direction
	//////////////////////////////////////////////////////////////
	for (i = 0; i < virtual_width; i += inter_x){
		x = (i - virtual_width / 2)*ImgPtScale;
		for (j = 0; j < virtual_height; j += inter_y){
			y = (j - virtual_height / 2)*ImgPtScale;
			r = x*x + y*y;
			y_ = y*(1 + k1*r + k2*r*r + k3*r*r*r + 2 * p1*y + 2 * p2*x) + p1*r*r;
			if (j == 0 && y_ > miny){
				printf("For y direction:%d %d %lf %lf\n", i, j, y_, miny);
				return false;
			}

			if (j == virtual_height - 1 && y_ < maxy){
				printf("For y direction:%d %d %lf %lf\n", i, j, y_, maxy);
				return false;
			}
			dy = 1 + k1*r + k2*r*r + k3*r*r*r + 2 * y*y*(k1 + 2 * k2*r + 3 * k3*r*r) + 2 * p2*x + 4 * p1*y + 4 * p1*r*y;
			if (dy < 0){
				printf("For y direction:%d %d %lf\n", i, j, dy);
				return false;
			}
		}
	}
	return true;
}
// return undistorted point at the same coordinate system as the input point
int CBasicFunction::CorrectDistortion(bool bEstiRealDistort, double x_d, double y_d, CAM &cam, double &x_u, double &y_u)
{
	double rx, ry;
	if (bEstiRealDistort){
		if (ComputeDistortionWithDistortedPoints_Normalized(x_d, y_d, cam, rx, ry) == 0){
		//	printf("Compute real distortion failed!\n");
			return 0;
		}
	}
	else{
		ComputeDistortionWithUndistortedPoints_Normalized(x_d, y_d, cam, rx, ry);
	}
	x_u = x_d - rx;
	y_u = y_d - ry;
	return 1;
}
int CBasicFunction::ComputeDistortionWithUndistortedPoints_Normalized(double x_u, double y_u, CAM &cam, double &rx, double &ry)
{
	double x, y, r, dr;
	x = ((x_u - cam.x0) + cam.s / cam.fy*(y_u - cam.y0)) / cam.fx;
	y = (y_u - cam.y0) / cam.fy;
	r = x*x + y*y;
	dr = cam.k1*r + cam.k2*r*r + cam.k3*r*r*r + cam.k4*r*r*r*r + cam.k5*pow(r, 5) + cam.k6*pow(r, 6) + cam.k7*pow(r, 7) + cam.k8*pow(r, 8) + cam.k9*pow(r, 9);
	rx = cam.fx*(x*dr + (cam.p1 * 2 * x*y + cam.p2*(r + 2 * x*x))*(1 + cam.p3*r) + cam.s1*r);
	ry = cam.fy*(y*dr + (cam.p2 * 2 * x*y + cam.p1*(r + 2 * y*y))*(1 + cam.p3*r) + cam.s2*r);
	return 1;
}
int CBasicFunction::ComputeDistortionWithDistortedPoints_Normalized(double x_d, double y_d, CAM &cam, double &rx, double &ry)
{
	double x_u = 0, y_u = 0;
	if (SearchForUndistortedCoordinates_Normalized(x_d, y_d, cam, x_u, y_u) == 0){
		return 0;
	}
	rx = x_d - cam.x0 - x_u;
	ry = y_d - cam.y0 - y_u;
	return 1;
}
int CBasicFunction::SearchForUndistortedCoordinates_Normalized(double x, double y, CAM &cam, double &nx, double &ny)
{
	int count = 0;
	int nMaxIteration = 1000;
	double r, dr, d, tx, ty, dx, dy, bx, by, sx, sy;
	double k1, k2, k3, k4, k5, k6, k7, k8, k9, p1, p2, p3, s1, s2;
	k1 = cam.k1;
	k2 = cam.k2;
	k3 = cam.k3;
	k4 = cam.k4;
	k5 = cam.k5;
	k6 = cam.k6;
	k7 = cam.k7;
	k8 = cam.k8;
	k9 = cam.k9;
	p1 = cam.p1;
	p2 = cam.p2;
	p3 = cam.p3;
	s1 = cam.s1;
	s2 = cam.s2;
	bx = x - cam.x0;
	by = y - cam.y0;
	x = (x - cam.x0) / cam.fx;
	y = (y - cam.y0) / cam.fy;
	r = x*x + y*y;
	sx = sy = 0;
	do
	{
		d = k1*r + k2*r*r + k3*r*r*r + k4*r*r*r*r + k5*pow(r, 5) + k6*pow(r, 6) + k7*pow(r, 7) + k8*pow(r, 8) + k9*pow(r, 9);
		tx = (2 * p1*x*y + p2*(r + 2 * x*x))*(1 + p3*r) + s1*r;
		ty = (2 * p2*x*y + p1*(r + 2 * y*y))*(1 + p3*r) + s2*r;
		dx = x*d + tx;
		dy = y*d + ty;

		nx = bx - dx*cam.fx;
		ny = by - dy*cam.fy;
		dr = sqrt(sx*sx + sy*sy) - sqrt(nx*nx + ny*ny);
		if (fabs(dr) < 1e-12){
			break;
		}
		sx = nx;
		sy = ny;
		x = (nx + ny*cam.s / cam.fy) / cam.fx;
		y = ny / cam.fy;
		r = x*x + y*y;

		count++;

	} while (count < nMaxIteration);
	x = (nx + ny*cam.s / cam.fy) / cam.fx;
	y = ny / cam.fy;
	r = x*x + y*y;
	d = k1*r + k2*r*r + k3*r*r*r + k4*r*r*r*r + k5*pow(r, 5) + k6*pow(r, 6) + k7*pow(r, 7) + k8*pow(r, 8) + k9*pow(r, 9);
	tx = (2 * p1*x*y + p2*(r + 2 * x*x))*(1 + p3*r) + s1*r;
	ty = (2 * p2*x*y + p1*(r + 2 * y*y))*(1 + p3*r) + s2*r;
	dx = x*d + tx;
	dy = y*d + ty;
	sx = nx + dx*cam.fx;
	sy = ny + dy*cam.fy;
	if (fabs(sx - bx) > 1e-8 || fabs(sy - by) > 1e-8){
		//	printf("Warning!Compute distortion with distorted points using normalized coordinates accuracy unexpected!\n");
		if (count == nMaxIteration){
			//	printf("Warning!Compute distortion with distorted points using normalized coordinates failed!\n");
			return 0;
		}
	}

	return 1;
}
int CBasicFunction::GetXYWithxyZ(EOP &eop, CAM &cam, double x, double y, double Z, double &X, double &Y, bool bEstiRealDistort)
{
	double pmaR[9];
	double f, a1, a2, a3, b1, b2, b3, c1, c2, c3, X_, Y_, Z_;

	f = cam.fx;

	GetRotateMatrixWithAngle(pmaR, eop.Ph, eop.Om, eop.Kp);

	a1 = pmaR[0]; a2 = pmaR[1]; a3 = pmaR[2];
	b1 = pmaR[3]; b2 = pmaR[4]; b3 = pmaR[5];
	c1 = pmaR[6]; c2 = pmaR[7]; c3 = pmaR[8];

	double x_u, y_u;

	CorrectDistortion(bEstiRealDistort, x, y, cam, x_u, y_u);

	x = x_u - cam.x0;
	y = y_u - cam.y0;

	X_ = a1*x + a2*y - a3*f;
	Y_ = b1*x + b2*y - b3*f;
	Z_ = c1*x + c2*y - c3*f;

	X = eop.Xs + (Z - eop.Zs)*X_ / Z_;
	Y = eop.Ys + (Z - eop.Zs)*Y_ / Z_;

	return 1;
}
int CBasicFunction::EstimatePlaneWith3DPoints(int nNum, PT3D *p3, double *line)
{
	int i = 0;
	double AtA[9] = { 0 };
	double Atl[3] = { 0 };
	double cX = 0, cY = 0, cZ = 0;
	for (i = 0; i < nNum; i++){
		cX += p3[i].X;
		cY += p3[i].Y;
		cZ += p3[i].Z;
	}
	cX /= nNum;
	cY /= nNum;
	cZ /= nNum;
	for (i = 0; i < nNum; i++){
		p3[i].X -= cX;
		p3[i].Y -= cY;
		p3[i].Z -= cZ;
	}
	
	double *A = new double[nNum * 3];
	for (i = 0; i < nNum; i++){
		A[i*3 + 0] = p3[i].X;
		A[i*3 + 1] = p3[i].Y;
		A[i*3 + 2] = p3[i].Z;
	}
	Mat maA(nNum, 3, CV_64F, A);
	Mat U, D, V;
	SVDecomp(maA, U, D, V);
	double *pU = U.ptr<double>(0);
	double *pD = D.ptr<double>(0);
	double *pV = V.ptr<double>(0);
	line[0] = pV[2];
	line[1] = pV[5];
	line[2] = pV[8];

	double *pRes = new double[nNum];

	for (i = 0; i < nNum; i++){
		p3[i].X += cX;
		p3[i].Y += cY;
		p3[i].Z += cZ;
	}
	line[3] = -(line[0] * cX + line[1] * cY + line[2] * cZ);
	for (i = 0; i < nNum; i++){
		pRes[i] = line[0] * p3[i].X + line[1] * p3[i].Y + line[2] * p3[i].Z + line[3];
	}
	/*
	for (i = 0; i < nNum; i++){
		
		AtA[0 * 3 + 0] += p3[i].X*p3[i].X;
		AtA[0 * 3 + 1] += p3[i].X*p3[i].Y;
		AtA[0 * 3 + 2] += p3[i].X*p3[i].Z;
		AtA[1 * 3 + 0] += p3[i].Y*p3[i].X;
		AtA[1 * 3 + 1] += p3[i].Y*p3[i].Y;
		AtA[1 * 3 + 2] += p3[i].Y*p3[i].Z;
		AtA[2 * 3 + 0] += p3[i].Z*p3[i].X;
		AtA[2 * 3 + 1] += p3[i].Z*p3[i].Y;
		AtA[2 * 3 + 2] += p3[i].Z*p3[i].Z;

		Atl[0] += -p3[i].X;
		Atl[1] += -p3[i].Y;
		Atl[2] += -p3[i].Z;
	}
	
	Mat maATA(3, 3, CV_64F, AtA);
	Mat maATL(3, 1, CV_64F, Atl);


	Mat maX = maATA.inv()*maATL;

	memcpy(line, maX.ptr<double>(0), sizeof(double) * 3);




	for (i = 0; i < nNum; i++){
		pRes[i] = line[0] * p3[i].X + line[1] * p3[i].Y + line[2] * p3[i].Z + 1;
	}
	*/
	delete[] pRes; pRes = NULL;
	return 1;
}
int CBasicFunction::ComputeVectorMatrix(PT3D *p3, double *pVecMatrix)
{
	if (p3 == NULL || pVecMatrix == NULL) return 0;
	pVecMatrix[0] = p3[1].X - p3[0].X;
	pVecMatrix[1] = p3[1].Y - p3[0].Y;
	pVecMatrix[2] = p3[1].Z - p3[0].Z;
	pVecMatrix[3] = p3[2].X - p3[0].X;
	pVecMatrix[4] = p3[2].Y - p3[0].Y;
	pVecMatrix[5] = p3[2].Z - p3[0].Z;
	pVecMatrix[6] = p3[3].X - p3[0].X;
	pVecMatrix[7] = p3[3].Y - p3[0].Y;
	pVecMatrix[8] = p3[3].Z - p3[0].Z;

	return 1;
}
int CBasicFunction::GetxyWithXYZ_GCamera(GCAM &gcam, PT3D *pLeafPt, PT3D &pt3, double &x, double &y)
{
	double p[9] = { 0 }; double c[3] = { 0 };
	ComputeVectorMatrix(pLeafPt, p);

	c[0] = p[0] * pt3.X + p[1] * pt3.Y + p[2] * pt3.Z;
	c[1] = p[3] * pt3.X + p[4] * pt3.Y + p[5] * pt3.Z;
	c[2] = p[6] * pt3.X + p[7] * pt3.Y + p[8] * pt3.Z;

	double *pG = gcam.gMatrix;
	double GX, GY, GZ;
	GX = pG[0] * c[0] + pG[1] * c[1] + pG[2] * c[2] + pG[3];
	GY = pG[4] * c[0] + pG[5] * c[1] + pG[6] * c[2] + pG[7];
	GZ = pG[8] * c[0] + pG[9] * c[1] + pG[10] * c[2] + pG[11];

	if (GZ == 0){
		return 0;
	}
	x = GX / GZ;
	y = GY / GZ;
	return 1;
}
int CBasicFunction::GetxyWithXYZ(CAM &cam, EOP &eop, PT3D &p3, double &x, double &y)
{
	double pmaR[9];
	double a1, a2, a3, b1, b2, b3, c1, c2, c3, X_, Y_, Z_;

	GetRotateMatrixWithAngle(pmaR, eop.Ph, eop.Om, eop.Kp);

	a1 = pmaR[0]; a2 = pmaR[1]; a3 = pmaR[2];
	b1 = pmaR[3]; b2 = pmaR[4]; b3 = pmaR[5];
	c1 = pmaR[6]; c2 = pmaR[7]; c3 = pmaR[8];

	X_ = a1*(p3.X - eop.Xs) + b1*(p3.Y - eop.Ys) + c1*(p3.Z - eop.Zs);
	Y_ = a2*(p3.X - eop.Xs) + b2*(p3.Y - eop.Ys) + c2*(p3.Z - eop.Zs);
	Z_ = a3*(p3.X - eop.Xs) + b3*(p3.Y - eop.Ys) + c3*(p3.Z - eop.Zs);

//	f = cam.fx;

	x = -cam.fx*X_ / Z_;
	y = -cam.fy*Y_ / Z_;

	return 1;
}
int CBasicFunction::GetxyResWithXYZ(CAM &cam, EOP &eop, PT3D &p3, PT2D &p2, double &rx, double &ry, bool bEstiRealDistort)
{
	double x, y, cx, cy, x_u, y_u;
	GetxyWithXYZ(cam, eop, p3, cx, cy);
	CorrectDistortion(bEstiRealDistort, p2.x, p2.y, cam, x_u, y_u);
	x = (x_u - cam.x0) + cam.s*(y_u - cam.y0);
	y = y_u - cam.y0;

	rx = x - cx;
	ry = y - cy;
	return 1;
}
int CBasicFunction::ComputeReprojErrorPerImgPt(PT3D &p3, PT2D * pP2, EOP * pE, CAM * pC, double &maxErr, int &maxErrID, bool bEstiRealDistort)
{

	PT2D *pNewP2 = new PT2D[p3.nIPtNum];
	int k = 0;
	for (int i = 0; i < p3.nIPtNum; i++){
		if (pP2[i].nAttrib < 0) continue;
		pNewP2[k] = pP2[i];
		k++;
	}
	PT3D np3 = p3;
	
	np3.nIPtNum = k;
	MultiInterSection(np3, pNewP2, pE, pC);

	p3.X = np3.X;
	p3.Y = np3.Y;
	p3.Z = np3.Z;

	PT3D *pP3 = new PT3D[p3.nIPtNum];
	double dx, dy, cX = 0, cY = 0;
	double *pRes = new double[p3.nIPtNum * 2];
	PT2D p2;
	maxErr = 0;
	for (int j = 0; j < p3.nIPtNum; j++){

		p2 = pP2[j];
		if (p2.nAttrib < 0) continue;
		EOP eop = pE[p2.nImgID];
		CAM cam = pC[p2.nCamID];

		GetxyResWithXYZ(cam, eop, p3, p2, dx, dy, bEstiRealDistort);

		double dis = sqrt(dx*dx + dy*dy);
		if (dis > maxErr){
			maxErr = dis;
			maxErrID = j;
		}
			
		pRes[2 * j + 0] = dx;
		pRes[2 * j + 1] = dy;
	}


	delete[] pNewP2; pNewP2 = NULL;
	delete[] pP3; pP3 = NULL;
	return 1;
}
int CBasicFunction::MultiInterSection(PT3D &p3, PT2D * pP2, EOP * pE, CAM * pC, int CoorControl, bool bEstiRealDistort)
{
	p3.nIPtSID = 0;
	return MultiInterSection(1, &p3, pP2, pE, pC, CoorControl, bEstiRealDistort);
	
}

int CBasicFunction::MultiInterSection(int nPtNum, PT3D * pP3, PT2D * pP2, EOP * pE, CAM * pC, int CoorControl, bool bEstiRealDistort)
{

	int i=0,j=0,k=0;
	
	for( i = 0;i < nPtNum;i ++ ){

		PT3D p3 = pP3[i];
		if (p3.nAttrib < 0){
			continue;
		}
		int nNum = p3.nIPtNum;
		double *px, *py;CAM *pCam; EOP *pEop;

		px = new double[nNum];memset( px, 0, sizeof(double) * nNum );
		py = new double[nNum];memset( py, 0, sizeof(double) * nNum );
		pCam = new CAM[nNum];
		pEop = new EOP[nNum];

		for( j = 0;j < p3.nIPtNum;j ++ ){

			PT2D p2 = pP2[p3.nIPtSID+j];

			pEop[j] = pE[p2.nImgID];
			pCam[j] = pC[p2.nCamID];

			px[j] = p2.x;
			py[j] = p2.y;

			k++;
		}
		double X, Y, Z, m0, mx, my, mz;
		int tCoorControl = CoorControl;
		if (MultiInterSection(nNum, pCam, pEop, px, py, X, Y, Z, m0, mx, my, mz, tCoorControl, bEstiRealDistort) == 0){
			return 0;
		}
		if (p3.nAttrib == 2){
			tCoorControl = 1;
		}
		if (tCoorControl != 1) p3.X = X;
		if (tCoorControl != 2) p3.Y = Y;
		if (tCoorControl != 3) p3.Z = Z;

		
		if (nPtNum == 1 || p3.nAttrib != 1){
			pP3[i] = p3;
		}
		
		delete [] px;px = NULL;
		delete [] py;py = NULL;
		delete[] pCam; pCam = NULL;
		delete[] pEop; pEop = NULL;

	}
	return 1;
}
int CBasicFunction::MultiInterSection(int N, CAM *pCam, EOP *pEop, double *px, double *py, double& X, double& Y, double& Z, double &m0, double &mx, double &my, double &mz, int CoorControl, bool bEstiRealDistort)
{
	int i,j,k;
	double pmaR[9] = {0};
	double A[6] = {0};
	double *pmaX;
	double L[2] = {0};

	double fx = 0,fy = 0, Xs = 0,Ys = 0,Zs = 0,phi = 0,omega = 0,kappa = 0;
	double a1 = 0, a2 = 0, a3 = 0;
	double b1 = 0, b2 = 0, b3 = 0;
	double c1 = 0, c2 = 0, c3 = 0;
	double x = 0,y = 0,x0 = 0,y0 = 0,x_u = 0,y_u = 0, xp = 0, yp = 0, r = 1.0;

	CXMatrix mapATA, mapATL ,mapX;

	double *pA = new double[2*N*3];
	double *pL = new double[2*N];

	for(i = 0;i < 2*N*3;i ++) pA[i] = 0.0;
	for(i = 0;i < 2*N;i ++)   pL[i] = 0.0;
	int nUnkNum = 3;
	if (CoorControl){
		nUnkNum = 2;
	}
	double *pATA = new double[nUnkNum*nUnkNum];
	double *pATL = new double[nUnkNum];
	memset(pATA, 0, sizeof(double)*nUnkNum*nUnkNum);
	memset(pATL, 0, sizeof(double)*nUnkNum);
	for(i=0;i<N;i++)
	{
		fx = pCam[i].fx;
		fy = pCam[i].fy;
		Xs = pEop[i].Xs;
		Ys = pEop[i].Ys;
		Zs = pEop[i].Zs;
		phi = pEop[i].Ph;
		omega = pEop[i].Om;
		kappa = pEop[i].Kp;

		GetRotateMatrixWithAngle(pmaR, phi,omega, kappa);
		a1 = pmaR[0]; a2 = pmaR[1]; a3 = pmaR[2]; 
		b1 = pmaR[3]; b2 = pmaR[4]; b3 = pmaR[5]; 
		c1 = pmaR[6]; c2 = pmaR[7]; c3 = pmaR[8];

		x = px[i];
		y = py[i];
		x0 = pCam[i].x0;
		y0 = pCam[i].y0;

		if (CorrectDistortion(bEstiRealDistort, x, y, pCam[i], x_u, y_u) == 0){
		//	printf("Compute real distortion failed!\n");
			delete[] pA;
			delete[] pL;
			return 0;
		}
		x = (x_u - x0) + pCam[i].s / pCam[i].fy*(y_u - y0);
		y = y_u - y0;
		k = 0;
		if (CoorControl != 1) { A[k] = fx*a1 + x*a3; k++; }
		if (CoorControl != 2) { A[k] = fx*b1 + x*b3; k++; }
		if (CoorControl != 3) { A[k] = fx*c1 + x*c3; k++; }
		L[0] = fx*a1*Xs + fx*b1*Ys + fx*c1*Zs + x*a3*Xs + x*b3*Ys + x*c3*Zs;

		k = 0;
		if (CoorControl != 1){ A[nUnkNum + k] = fy*a2 + y*a3; k++; }
		if (CoorControl != 2){ A[nUnkNum + k] = fy*b2 + y*b3; k++; }
		if (CoorControl != 3){ A[nUnkNum + k] = fy*c2 + y*c3; k++; }
		L[1] = fy*a2*Xs + fy*b2*Ys + fy*c2*Zs + y*a3*Xs + y*b3*Ys + y*c3*Zs;

		for (j = 0; j < nUnkNum; j++)
		{
			pA[i * 2 * nUnkNum + j] = A[j];
			pA[(i * 2 + 1)*nUnkNum + j] = A[j + nUnkNum];

			for (k = 0; k < nUnkNum; k++)
			{
				pATA[j*nUnkNum + k] += A[j] * A[k];
				pATA[j*nUnkNum + k] += A[j + nUnkNum] * A[k + nUnkNum];
			}

			pATL[j] += A[j] * L[0];
			pATL[j] += A[j + nUnkNum] * L[1];
		}
		pL[i*2] = L[0];
		pL[i*2 + 1] = L[1];

	}

	//////////////////////////////////////////////////
	mapATA.InitMatrix(pATA, nUnkNum, nUnkNum);
	mapATL.InitMatrix(pATL, nUnkNum, 1);
	CXMatrix maAT_A = mapATA.InverseMatrix();
	mapX = mapATA.InverseMatrix() * mapATL;

	pmaX = mapX.GetData();

	k = 0;
	if (CoorControl != 1){ X = pmaX[k]; k++; }
	if (CoorControl != 2){ Y = pmaX[k]; k++; }
	if (CoorControl != 3){ Z = pmaX[k]; k++; }

	//Compute RMS
	//////////////////////////////////////////////////////////////////////////

	CXMatrix maA,maL,maVV,mapAT_A;
	maA.InitMatrix(pA, 2 * N, nUnkNum);
	maL.InitMatrix(pL, 2 * N, 1);
	maVV = maA * mapX - maL;
	mapAT_A = mapATA.InverseMatrix();
	double * pV = maVV.GetData();
	double * pAT_A = mapAT_A.GetData();
	double vv = 0;
	for(i = 0;i < 2*N;i ++)
	{
		vv += pV[i] * pV[i];
	}
	k = 0;
	m0 = sqrt(vv / (2 * N - nUnkNum));
	if (CoorControl != 1){ mx = sqrt(pAT_A[0 * nUnkNum + k]) * m0; k++; }
	if (CoorControl != 2){ my = sqrt(pAT_A[1 * nUnkNum + k]) * m0; k++; }
	if (CoorControl != 3){ mz = sqrt(pAT_A[2 * nUnkNum + k]) * m0; k++; }

	delete [] pA;
	delete [] pL;

	return 1;

}
int CBasicFunction::MultiInterSection_ProjectMatrix(int N, PT2D *p2, CAM *pCam, double *pM, double &X, double &Y, double &Z, double *m)
{
	int i, j, k;
	double A[6] = { 0 };
	double L[2] = { 0 };
	double x = 0, y = 0, x_u = 0, y_u = 0;

	CXMatrix mapATA, mapATL, mapX; double *pmaX;

	double *pA = new double[2 * N * 3];
	double *pL = new double[2 * N];
	memset(pA, 0, sizeof(double) * 6 * N);
	memset(pL, 0, sizeof(double) * 2 * N);

	int nUnkNum = 3;

	double *pATA = new double[nUnkNum*nUnkNum];
	double *pATL = new double[nUnkNum];
	memset(pATA, 0, sizeof(double)*nUnkNum*nUnkNum);
	memset(pATL, 0, sizeof(double)*nUnkNum);
	for (i = 0; i<N; i++)
	{
		x = p2[i].x;
		y = p2[i].y;

		if (CorrectDistortion(true, x, y, pCam[p2[i].nCamID], x_u, y_u) == 0){
			//	printf("Compute real distortion failed!\n");
			delete[] pA;
			delete[] pL;
			return 0;
		}
		x = x_u;
		y = y_u;
		k = 0;
		double *ptM = pM + i * 12;
		A[k] = ptM[0] - ptM[8] * x; k++;
		A[k] = ptM[1] - ptM[9] * x; k++;
		A[k] = ptM[2] - ptM[10] * x; k++;
		L[0] = -ptM[3] + ptM[11] * x;

		k = 0;
		A[k] = ptM[4] - ptM[8] * y; k++;
		A[k] = ptM[5] - ptM[9] * y; k++;
		A[k] = ptM[6] - ptM[10] * y; k++;
		L[0] = -ptM[7] + ptM[11] * y;

		for (j = 0; j < nUnkNum; j++)
		{
			pA[i * 2 * nUnkNum + j] = A[j];
			pA[(i * 2 + 1)*nUnkNum + j] = A[j + nUnkNum];

			for (k = 0; k < nUnkNum; k++)
			{
				pATA[j*nUnkNum + k] += A[j] * A[k];
				pATA[j*nUnkNum + k] += A[j + nUnkNum] * A[k + nUnkNum];
			}

			pATL[j] += A[j] * L[0];
			pATL[j] += A[j + nUnkNum] * L[1];
		}
		pL[i * 2] = L[0];
		pL[i * 2 + 1] = L[1];

	}

	//////////////////////////////////////////////////
	mapATA.InitMatrix(pATA, nUnkNum, nUnkNum);
	mapATL.InitMatrix(pATL, nUnkNum, 1);
	CXMatrix maAT_A = mapATA.InverseMatrix();
	mapX = mapATA.InverseMatrix() * mapATL;

	pmaX = mapX.GetData();

	k = 0;
	X = pmaX[0];
	Y = pmaX[1];
	Z = pmaX[2];

	//Compute RMS
	//////////////////////////////////////////////////////////////////////////
	
	CXMatrix maA, maL, maVV, mapAT_A;
	maA.InitMatrix(pA, 2 * N, nUnkNum);
	maL.InitMatrix(pL, 2 * N, 1);
	maVV = maA * mapX - maL;
	mapAT_A = mapATA.InverseMatrix();
	double * pV = maVV.GetData();
	double * pAT_A = mapAT_A.GetData();
	double vv = 0;
	for (i = 0; i < 2 * N; i++)
	{
		vv += pV[i] * pV[i];
	}
	k = 0;
	m[0] = sqrt(vv / (2 * N - nUnkNum));
	m[1] = sqrt(pAT_A[0 * nUnkNum + k]) * m[0]; k++;
	m[2] = sqrt(pAT_A[1 * nUnkNum + k]) * m[0]; k++;
	m[3] = sqrt(pAT_A[2 * nUnkNum + k]) * m[0]; k++;

	delete[] pA;
	delete[] pL;

	return 1;
}
int CBasicFunction::MultiIntersection_GCamera_MultiPoints(int nPtNum, GCAM *pGCam, PT3D *pLeafPt, PT2D *p2, PT3D *p3)
{
	int i = 0;
	for (i = 0; i < nPtNum; i++){
		PT3D pt3 = p3[i];
		if (pt3.nAttrib < 0){
			continue;
		}
		MultiIntersection_GCamera(pt3.nIPtNum, pGCam, pLeafPt, p2+pt3.nIPtSID, pt3);

		if (pt3.nAttrib != 1){
			p3[i] = pt3;
		}
	}
	return 1;
}
int CBasicFunction::MultiIntersection_GCamera(int nImgPtNum, GCAM *pGCam, PT3D *pLeafPt, PT2D *p2, PT3D &pt3)
{
	int i;
	CAM *pCam = new CAM[nImgPtNum];
	double *pM = new double[12 * nImgPtNum];
	double VM[9] = { 0 }; double pQ[16] = { 0 };
	
	for (i = 0; i < nImgPtNum; i++){
		PT2D pt2 = p2[i];
		PT3D tp3 = pLeafPt[pt2.nImgID * 4 + 0];
		pCam[i] = pGCam[i].camPara;
		ComputeVectorMatrix(pLeafPt + pt2.nImgID*4, VM);
		pQ[0] = VM[0]; pQ[1] = VM[1]; pQ[2] = VM[2]; pQ[3] = -(VM[0] * tp3.X + VM[1] * tp3.Y + VM[2] * tp3.Z);
		pQ[4] = VM[3]; pQ[5] = VM[4]; pQ[6] = VM[5]; pQ[7] = -(VM[3] * tp3.X + VM[4] * tp3.Y + VM[5] * tp3.Z);
		pQ[8] = VM[6]; pQ[9] = VM[7]; pQ[10] = VM[8]; pQ[11] = -(VM[6] * tp3.X + VM[7] * tp3.Y + VM[8] * tp3.Z);
		pQ[15] = 1;
		Mat maG(3, 4, CV_64F, pGCam[pt2.nCamID].gMatrix);
		Mat maQ(4, 4, CV_64F, pQ);
		Mat maM = maG*maQ;
		memcpy(pM+i*12, maM.ptr<double>(0), sizeof(double) * 12);
	}
	double X, Y, Z, m[4];
	MultiInterSection_ProjectMatrix(nImgPtNum, p2, pCam, pM, X, Y, Z, m);

	pt3.X = X;
	pt3.Y = Y;
	pt3.Z = Z;


	delete[] pM; pM = NULL;

	return 1;
}
int CBasicFunction::MultiMatrix( double *pW, double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB )
{
	if( pA == 0 || pB == 0 || pAB == 0 || aC != bR ) return 0;
	memset( pAB, 0, sizeof(double) * aR * bC );
	int i = 0;int j = 0;int k = 0;
	for( i = 0;i < aR;i ++ ){
		for( j = 0;j < bC;j ++ ){
			for( k = 0;k < aC; k ++ ){ 
				pAB[i*bC+j] += pA[i*aC+k] * pW[i] * pB[k*bC+j]; }}}
	return 1;
}
int CBasicFunction::MultiMatrix( double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB )
{
	if( pA == 0 || pB == 0 || pAB == 0 || aC != bR ) return 0;
	memset( pAB, 0, sizeof(double) * aR * bC );
	int i = 0;int j = 0;int k = 0;
	for( i = 0;i < aR;i ++ ){
		for( j = 0;j < bC;j ++ ){
			for( k = 0;k < aC; k ++ ){ 
				pAB[i*bC+j] += pA[i*aC+k] * pB[k*bC+j]; }}}
	return 1;
}
int CBasicFunction::TransMultiMatrix( double *pW, double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB )
{
	if( pA == 0 || pB == 0 || pAB == 0 || aR != bR ) return 0;
	memset( pAB, 0, sizeof(double) * aC * bC );
	int i = 0;int j = 0;int k = 0;
	for( i = 0;i < aC;i ++ ){
		for( j = 0;j < bC;j ++ ){
			for( k = 0;k < aR; k ++ ){ 
				pAB[i*bC+j] += pA[k*aC+i] * pW[k] * pB[k*bC+j]; }}}
	return 1;
}
int CBasicFunction::TransMultiMatrix( double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB )
{
	if( pA == 0 || pB == 0 || pAB == 0 || aR != bR ) return 0;
	memset( pAB, 0, sizeof(double) * aC * bC );
	int i = 0;int j = 0;int k = 0;
	for( i = 0;i < aC;i ++ ){
		for( j = 0;j < bC;j ++ ){
			for( k = 0;k < aR; k ++ ){ 
				pAB[i*bC+j] += pA[k*aC+i] * pB[k*bC+j]; }}}
	return 1;
}
int CBasicFunction::MultiTransMatrix( double *pW, double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB )
{
	if( pA == 0 || pB == 0 || pAB == 0 || aC != bC ) return 0;
	memset( pAB, 0, sizeof(double) * aR * bR );
	int i = 0;int j = 0;int k = 0;
	for( i = 0;i < aR;i ++ ){
		for( j = 0;j < bR;j ++ ){
			for( k = 0;k < aC; k ++ ){ 
				pAB[i*bR+j] += pA[i*aC+k] * pW[i] * pB[j*bC+k]; }}}
	return 1;
}
int CBasicFunction::MultiTransMatrix( double *pA, int aR, int aC, double *pB, int bR, int bC, double * pAB )
{
	if( pA == 0 || pB == 0 || pAB == 0 || aC != bC ) return 0;
	memset( pAB, 0, sizeof(double) * aR * bR );
	int i = 0;int j = 0;int k = 0;
	for( i = 0;i < aR;i ++ ){
		for( j = 0;j < bR;j ++ ){
			for( k = 0;k < aC; k ++ ){ 
				pAB[i*bR+j] += pA[i*aC+k] * pB[j*bC+k]; }}}
	return 1;
}
int CBasicFunction::InverseMatrix(double* pData, int n)
{

	if ((pData == 0) || (n <= 0))
		return -1;

	int iMaxR = 0;
	int iMaxC = 0;
	int* pRow = new int[n];
	int* pCol = new int[n];
	double dTmp = 0;
	double dMaxNum = 0;
	int i = 0;
	int j = 0;

	for (int t=0; t<n; t++)
	{
		dMaxNum = 0;
		for (i=t; i<n; i++)
		{
			for (j=t; j<n; j++)
			{
				if (fabs(pData[i*n+j]) > dMaxNum)
				{
					iMaxR = i;
					iMaxC = j;
					dMaxNum = fabs(pData[i*n+j]);
				}
			}
		}

		pRow[t] = iMaxR;
		pCol[t] = iMaxC;

		if (dMaxNum < EPSION)
		{	
			delete[] pRow;
			delete[] pCol;
			return -1;
		}


		if (iMaxR != t)
		{
			for (i=0; i<n; i++)
			{
				dTmp = pData[t*n+i];
				pData[t*n+i] = pData[iMaxR*n+i];
				pData[iMaxR*n+i] = dTmp;
			}
		}
		if (iMaxC != t)
		{
			for (i=0; i<n; i++)
			{
				dTmp = pData[i*n+t];
				pData[i*n+t] = pData[i*n+iMaxC];
				pData[i*n+iMaxC] = dTmp;
			}
		}

		pData[t*n+t] = 1.0/pData[t*n+t];
		double dnn = pData[t*n+t];
		for (i=0; i<n; i++)
		{
			if (i != t)
			{
				pData[t*n+i] *= dnn; 
			}
		}

		for (i=0; i<n; i++)
		{
			if (i != t)
			{
				double dit = pData[i*n+t];
				if (fabs(dit) < EPSION)
					continue;

				for (j=0; j<n; j++)
				{
					if (j != t)
						pData[i*n+j] -= dit*pData[t*n+j];
				}

			}
		}

		for (i=0; i<n; i++)
		{
			if (i != t)
			{
				pData[i*n+t] *= -dnn; 
			}
		}
	}

	for (i=n-1; i>=0; i--)
	{
		if (pCol[i] != i)
		{
			iMaxR = pCol[i];
			for (j=0; j<n; j++)
			{
				dTmp = pData[iMaxR*n+j];
				pData[iMaxR*n+j] = pData[i*n+j];
				pData[i*n+j] = dTmp;
			}
		}

		if (pRow[i] != i)
		{
			iMaxC = pRow[i];
			for (j=0; j<n; j++)
			{
				dTmp = pData[j*n+iMaxC];
				pData[j*n+iMaxC] = pData[j*n+i];
				pData[j*n+i] = dTmp;
			}
		}
	}

	delete[] pRow;
	delete[] pCol;
	return 1;
}
int CBasicFunction::InverseSparseMatrix(double* pData, int nSubNum, int *pSubWidth )
{
	int i = 0;int j = 0;int k = 0;int n = 0;
	for( i = 0;i < nSubNum;i ++ ) n += pSubWidth[i];
	for( i = 0;i < nSubNum;i ++ ){
		double * pSubData = new double[pSubWidth[i]*pSubWidth[i]];
		memset( pSubData, 0, sizeof(double) * pSubWidth[i]*pSubWidth[i] );
		memcpy( pSubData, pData+k, sizeof(double) * pSubWidth[i]*pSubWidth[i] );
		InverseMatrix( pSubData, pSubWidth[i] );
		memcpy( pData+k, pSubData, sizeof(double) * pSubWidth[i]*pSubWidth[i] );
		k += pSubWidth[i]*pSubWidth[i];
		delete [] pSubData;pSubData = NULL;
	}
	return 1;
}

int CBasicFunction::NomarlizePointSetWithCenterPoint(int nNum, int nDimension, double *pSrc, int nCenter, double *pDst, double *pT)
{
	int i = 0;
	double xc, yc, zc, x1, y1, z1;
	double aver_dis;
	aver_dis = 1;

	xc = pSrc[nCenter*nDimension + 0];
	yc = pSrc[nCenter*nDimension + 1];
	zc = pSrc[nCenter*nDimension + 2];

	for (i = 0; i < nNum; i++){

		x1 = pSrc[nDimension * i + 0] - xc;
		y1 = pSrc[nDimension * i + 1] - yc;
		z1 = pSrc[nDimension * i + 2] - zc;
		aver_dis += x1*x1 + y1*y1 + z1*z1;
	}
	aver_dis /= nNum;
	aver_dis = sqrt(aver_dis);
	int k = 0;
	double *pTemp = new double[nDimension*nNum];
	for (i = 0; i < nNum; i++){
		if (i == nCenter) continue;
		pDst[nDimension * k + 0] = (pSrc[nDimension * i + 0] - xc) / aver_dis;
		pDst[nDimension * k + 1] = (pSrc[nDimension * i + 1] - yc) / aver_dis;
		pDst[nDimension * k + 2] = (pSrc[nDimension * i + 2] - zc) / aver_dis;
		k++;
	}

	pT[0] = aver_dis; pT[1] = xc; pT[2] = yc; pT[3] = zc;
	return 1;
}
int CBasicFunction::NomarlizePointCoor(int nNum, unsigned int *pSampleID, int nSampleNum, double* pSrc, double* pT)
{
	int i = 0;
	double xc, yc, zc, x1, y1, z1;
	double aver_dis;
	aver_dis = 1;

	xc = yc = zc = 0;
	int count[3] = { 0 };
	double mdp[9] = { 0 };
	double len[3] = { 0 };
	for (i = 0; i < nSampleNum; i++) {

		xc += pSrc[nNum * 0 + pSampleID[i]];
		yc += pSrc[nNum * 1 + pSampleID[i]];
		zc += pSrc[nNum * 2 + pSampleID[i]];
	}
	xc /= nSampleNum;
	yc /= nSampleNum;
	zc /= nSampleNum;

	for (i = 0; i < nSampleNum; i++) {

		x1 = pSrc[nNum * 0 + pSampleID[i]] - xc;
		y1 = pSrc[nNum * 1 + pSampleID[i]] - yc;
		z1 = pSrc[nNum * 2 + pSampleID[i]] - zc;
		aver_dis += x1 * x1 + y1 * y1 + z1 * z1;
	}
	aver_dis /= nSampleNum;
	aver_dis = sqrt(aver_dis);


	pT[0] = aver_dis; pT[1] = xc; pT[2] = yc; pT[3] = zc;

	return 1;
}
int CBasicFunction::NomarlizePointCoor(int nNum, int nDimension, double *pSrc, double *pDst, double *pT)
{
	int i = 0;
	double xc, yc, zc, x1, y1, z1;
	double aver_dis;
	aver_dis = 1;

	xc = yc = zc = 0;
	int count[3] = { 0 };
	double mdp[9] = { 0 };
	double len[3] = { 0 };
	for (i = 0; i < nNum; i++){

		xc += pSrc[nDimension*i + 0];
		yc += pSrc[nDimension*i + 1];
		zc += pSrc[nDimension*i + 2];
	}
	xc /= nNum;
	yc /= nNum;
	zc /= nNum;

	for (i = 0; i < nNum; i++){

		x1 = pSrc[nDimension * i + 0] - xc;
		y1 = pSrc[nDimension * i + 1] - yc;
		z1 = pSrc[nDimension * i + 2] - zc;
		aver_dis += x1*x1 + y1*y1 + z1*z1;
	}
	aver_dis /= nNum;
	aver_dis = sqrt(aver_dis);
	
	if (pDst) {
		for (i = 0; i < nNum; i++) {

			pDst[0 * nNum + i] = pSrc[nDimension * i + 0] - xc;
			pDst[1 * nNum + i] = pSrc[nDimension * i + 1] - yc;
			pDst[2 * nNum + i] = pSrc[nDimension * i + 2] - zc;

		}
	}


	pT[0] = aver_dis; pT[1] = xc; pT[2] = yc; pT[3] = zc;
	
	return 1;
}
int CBasicFunction::FindBestBenchMarkPoint(int nNum, double *pSrc, int *maxID)
{
	double *pgSrc = new double[3 * nNum];
	double pT[4] = { 0 };
	NomarlizePointCoor(nNum, 3, pSrc, pgSrc, pT);

	double maxL = 0, length = 0;
	double *pLen = new double[nNum];
	for (int i = 0; i < nNum; i++){

		length = pgSrc[0 * nNum + i] * pgSrc[0 * nNum + i] + pgSrc[1 * nNum + i] * pgSrc[1 * nNum + i] + pgSrc[2 * nNum + i] * pgSrc[2 * nNum + i];
		pLen[i] = sqrt(length);
		if (pLen[i] > maxL){
			maxID[0] = i;
			maxL = pLen[i];
		}
	}
	double a = 0, maxAng, maxLength, maxP = 0;
	double *tmp = new double[nNum];
	PT3D maxPt;
	maxPt.X = pgSrc[0 * nNum + maxID[0]];
	maxPt.Y = pgSrc[1 * nNum + maxID[0]];
	maxPt.Z = pgSrc[2 * nNum + maxID[0]];

	for (int i = 0; i < nNum; i++){
		if (i == maxID[0]) continue;
		length = pLen[i];
		a = acos((maxPt.X*pgSrc[0 * nNum + i] + maxPt.Y*pgSrc[1 * nNum + i] + maxPt.Z*pgSrc[2 * nNum + i]) / (maxL*length));
		tmp[i] = a;

		if (sin(a)*length > maxP){
			maxP = sin(a)*length;
			maxAng = a;
			maxLength = length;
			maxID[1] = i;
		}
	}

	delete[] pgSrc; pgSrc = NULL;
	return 1;
}
int CBasicFunction::CalcSevenParameters_linear(int nNum, PT3D *ptSrc, PT3D *ptDst, double *pSimi)
{
	double *pSrc = new double[3 * nNum];
	double *pDst = new double[3 * nNum];

	for (int i = 0; i < nNum; i++){

		pSrc[3 * i + 0] = ptSrc[i].X;
		pSrc[3 * i + 1] = ptSrc[i].Y;
		pSrc[3 * i + 2] = ptSrc[i].Z;
			  
		pDst[3 * i + 0] = ptDst[i].X;
		pDst[3 * i + 1] = ptDst[i].Y;
		pDst[3 * i + 2] = ptDst[i].Z;
	}

	int maxID[2] = { 0 };
	FindBestBenchMarkPoint(nNum, pSrc, maxID);
	int64 T1 = 0, T2 = 0, nIter = 1;
	double time = 0;
	
	T1 = getCPUTickCount();
	for (int i = 0; i < nIter; i++){
		if (CalcSevenParameters_linear_Accurate(nNum, pSrc, pDst, pSimi) == 0){
			delete[] pSrc; pSrc = NULL;
			delete[] pDst; pDst = NULL;
			return 0;
		}
	}
	T2 = getCPUTickCount();
	time = double(T2 - T1)/1000.0;
	
	
	T1 = getCPUTickCount();
	for (int i = 0; i < nIter; i++){
		if (CalcSevenParameters_linear_fast(false, nNum, pSrc, pDst, pSimi) == 0){
			delete[] pSrc; pSrc = NULL;
			delete[] pDst; pDst = NULL;
			return 0;
		}
	}
	T2 = getCPUTickCount();
	time = double(T2 - T1)/1000.0;
	
	//T1 = getCPUTickCount();
	//for (int i = 0; i < nIter; i++){
	//	if (CalcSevenParameters_linear_Accurate(nNum, pSrc, pDst, pSimi) == 0){
	//		delete[] pSrc; pSrc = NULL;
	//		delete[] pDst; pDst = NULL;
	//		return 0;
	//	}
	//}
	//T2 = getCPUTickCount();
	//time = double(T2 - T1) / 1000.0;

	
	delete [] pSrc; pSrc = NULL;
	delete [] pDst; pDst = NULL;
	return 1;
}
int CBasicFunction::FindCrossEigenVectors(int nDimension, int nNum, double *pSrcA, double *pSrcB, double *pEVectA, double *pEVectB)
{
	double p[9] = { 0 };

	for (int i = 0; i < nNum; i++){
		p[0] += pSrcA[0 * nNum + i] * pSrcB[0 * nNum + i];
		p[1] += pSrcA[0 * nNum + i] * pSrcB[1 * nNum + i];
		p[2] += pSrcA[0 * nNum + i] * pSrcB[2 * nNum + i];
		p[3] += pSrcA[1 * nNum + i] * pSrcB[0 * nNum + i];
		p[4] += pSrcA[1 * nNum + i] * pSrcB[1 * nNum + i];
		p[5] += pSrcA[1 * nNum + i] * pSrcB[2 * nNum + i];
		p[6] += pSrcA[2 * nNum + i] * pSrcB[0 * nNum + i];
		p[7] += pSrcA[2 * nNum + i] * pSrcB[1 * nNum + i];
		p[8] += pSrcA[2 * nNum + i] * pSrcB[2 * nNum + i];

	}
	
	Mat U, D, V, Vt;
	Mat P(3, 3, CV_64F, p);
	SVDecomp(P, D, U, Vt);

	V = Vt.t();

	memcpy(pEVectA, U.data, sizeof(double) * 9);
	memcpy(pEVectB, V.data, sizeof(double) * 9);
	
	return 1;
}
int CBasicFunction::FindBenchmarkPoints(int nNum, int nDimension, double *pSrc, double *pDst)
{
	double p[9] = { 0 };
	for (int i = 0; i < nNum; i++){
		p[0] += pSrc[0 * 3 + i] * pSrc[0 * 3 + i];
		p[1] += pSrc[0 * 3 + i] * pSrc[1 * 3 + i];
		p[2] += pSrc[0 * 3 + i] * pSrc[2 * 3 + i];
		p[3] += pSrc[1 * 3 + i] * pSrc[0 * 3 + i];
		p[4] += pSrc[1 * 3 + i] * pSrc[1 * 3 + i];
		p[5] += pSrc[1 * 3 + i] * pSrc[2 * 3 + i];
		p[6] += pSrc[2 * 3 + i] * pSrc[0 * 3 + i];
		p[7] += pSrc[2 * 3 + i] * pSrc[1 * 3 + i];
		p[8] += pSrc[2 * 3 + i] * pSrc[2 * 3 + i];
	}
	
	Mat P(3, 3, CV_64F, p);
	Mat vecs, vals;
	cv::eigen(P, vals, vecs);
//	vecs = vecs.t();
	double temp = cv::determinant(vecs);
	double *pVal = (double*)vals.data;
	double *pVec = (double*)vecs.data;
	
	Mat U, D, Vt;
	double t[9] = { 0 };
	double v[3] = { 0 };
	v[0] = pVec[0];
	v[1] = pVec[1];
	v[2] = pVec[2];
	MultiMatrix(p, 3, 3, v, 3, 1, t);
	SVDecomp(P, D, U, Vt);
	Mat V = Vt.t();
	double *pVt = (double*)Vt.data;
	double *pD = (double*)D.data;
	double *pU = (double*)U.data;
	
//	memcpy(pDst, V.data, sizeof(double) * 9);
	memcpy(pDst, U.data, sizeof(double) * 9);
	return 1;
}
int CBasicFunction::HouseholderTransformation(int nTransNum, unsigned int *pID, int nRow, int nCol, double* pSrc, double* pDstQ, double *pDstT)
{
	int i = 0, i1 = 0, j = 0;
	int nID = 0, nDimension = 0;
	int nSize = nRow * nCol;
	double* pV = NULL; CBasicFunction BF;
	double* pA = new double[nSize];
	memcpy(pA, pSrc, sizeof(double) * nSize);
	double* pH = new double[nRow * nRow];
	double* pTmp = new double[nSize];
	int* pFlag = new int[nCol];
	memset(pFlag, 0, sizeof(int) * nCol);

	for (i = 0; i < nTransNum; i++) {
		nID = pID[i];
		pFlag[nID] = 1;
		nDimension = nRow - i;
		pV = new double[nDimension];

		memset(pV, 0, sizeof(double) * nDimension);
		memset(pH, 0, sizeof(double) * nRow * nRow);


		GetColumnVector(nID, nRow, nCol, pA, pTmp);
		memcpy(pV, pTmp + i, sizeof(double) * nDimension);
		HouseholderTransformation(nRow, nDimension, pV, pH);

		pA[i * nCol + nID] = pV[0] / fabs(pV[0]) * GetVectorLength(nDimension, pV);
		for (j = i + 1; j < nRow; j++) {
			pA[j * nCol + nID] = 0;
		}
		for (j = 0; j < nCol; j++) {
			if (pFlag[j] == 1) {
				continue;
			}

			GetColumnVector(j, nRow, nCol, pA, pTmp + nRow);
			BF.MultiMatrix(pH, nRow, nRow, pTmp + nRow, nRow, 1, pTmp);

			for (i1 = 0; i1 < nRow; i1++) {
				pA[i1 * nCol + j] = pTmp[i1];
			}
		}
		/*
		if (i == 0) {
			memcpy(pDstQ, pH, sizeof(double) * nRow * nRow);
		}
		else {

			memset(pTmp, 0, sizeof(double) * nRow * nRow);
			BF.MultiMatrix(pDstQ, nRow, nRow, pH, nRow, nRow, pTmp);
			memcpy(pDstQ, pTmp, sizeof(double) * nRow * nRow);
		}
		*/


		delete[] pV; pV = NULL;

	}
	/*
	for (j = 0; j < nCol; j++) {

		GetColumnVector(j, nRow, nCol, pSrc, pTmp + nRow);
		BF.MultiMatrix(pDstQ, nRow, nRow, pTmp + nRow, nRow, 1, pTmp);

		for (i1 = 0; i1 < nRow; i1++) {
		//	pA[i1 * nCol + j] = pTmp[i1];
		}
	}
	*/
	memcpy(pDstT, pA, sizeof(double) * nSize);
	BF.MultiMatrix(pDstQ, nRow, nRow, pA, nRow, nCol, pTmp);

	delete[] pA; pA = NULL;
	delete[] pH; pH = NULL;
	delete[]pTmp; pTmp = NULL;
	delete[] pFlag; pFlag = NULL;
	return 1;
}
int CBasicFunction::HouseholderTransformation(int nRow, int nCol, double *pSrc, int &nTransNum, double *pDstQ, double *pDstT)
{
	if (nRow == 0 && nCol == 0){
		return 0;
	}
	int i = 0, i1 = 0, j = 0;
	nTransNum = nRow - 1;
	if (nRow > nCol){
		nTransNum = nCol;
	}
	int nDimension = 0;
	int nSize = nRow*nCol;
	double *pV = NULL; CBasicFunction BF;
	double *pA = new double[nSize];
	memcpy(pA, pSrc, sizeof(double)*nSize);
	double *pH = new double[nRow*nRow];
	double *pTmp = new double[nSize];
	for (i = 0; i < nTransNum; i++){
		nDimension = nRow - i;
		pV = new double[nDimension];
		
		memset(pV, 0, sizeof(double)*nDimension);
		memset(pH, 0, sizeof(double)*nRow*nRow);


		GetColumnVector(i, nRow, nCol, pA, pTmp);
		memcpy(pV, pTmp+i, sizeof(double)*nDimension);
		HouseholderTransformation(nRow, nDimension, pV, pH);

		pA[i*nCol + i] = pV[0]/fabs(pV[0])*GetVectorLength(nDimension, pV);
		for (j = i+1; j < nRow; j++){
			pA[j*nCol + i] = 0;
		}
		for (j = i+1; j < nCol; j++){
			
			GetColumnVector(j, nRow, nCol, pA, pTmp + nRow);
			BF.MultiMatrix(pH, nRow, nRow, pTmp + nRow, nRow, 1, pTmp);

			for (i1 = 0; i1 < nRow; i1++){
				pA[i1*nCol + j] = pTmp[i1];
			}
		}

		if (i == 0){
			memcpy(pDstQ, pH, sizeof(double)*nRow*nRow);
		}
		else{
			
			memset(pTmp, 0, sizeof(double)*nRow*nRow);
			BF.MultiMatrix(pDstQ, nRow, nRow, pH, nRow, nRow, pTmp);
			memcpy(pDstQ, pTmp, sizeof(double)*nRow*nRow);
		}
	}
	if (pDstT){
		pDstT[0] = pA[0];
		pDstT[1] = pA[1 * nCol + 1];
		pDstT[2] = pA[2 * nCol + 2];

	//	memcpy(pDstT, pA, sizeof(double)*nSize);
	}
	
	BF.MultiMatrix(pDstQ, nRow, nRow, pA, nRow, nCol, pTmp);

	delete[] pA; pA = NULL;
	delete[] pH; pH = NULL;
	delete[]pTmp; pTmp = NULL;
	return 1;
}
double CBasicFunction::GetVectorLength(int nNum, double *pV)
{
	double result = 0;
	for (int i = 0; i < nNum; i++){
		result += pV[i] * pV[i];
	}
	result = sqrt(result);
	return result;
}
int CBasicFunction::GetColumnVector(int index, int nRow, int nCol, double *pSrc, double *pDst)
{
	for (int i = 0; i < nRow; i++){
		pDst[i] = pSrc[i*nCol + index];
	}
	return 1;
}
int CBasicFunction::HouseholderTransformation(int nRow, int nDimension, double *pSrc, double *pDstH)
{
	double *w = new double[nDimension];
	memset(w, 0, sizeof(double)*nDimension);

	double length = GetVectorLength(nDimension, pSrc);
	w[0] = pSrc[0] - pSrc[0]/fabs(pSrc[0])*length;
	memcpy(w + 1, pSrc + 1, sizeof(double)*(nDimension-1));
	length = GetVectorLength(nDimension, w);

	int i = 0, j = 0;
	for (i = 0; i < nDimension; i++){
		w[i] /= length;
	}
	for (i = 0; i < nRow; i++){
		pDstH[i*nRow + i] = 1;
	}
	int offset = nRow - nDimension;
	for (i = 0; i < nDimension; i++){
		for (j = 0; j < nDimension; j++){
			pDstH[(offset + i)*nRow + (offset + j)] -= 2*w[i] * w[j];
		}
	}
	delete[] w; w = NULL;
	return 1;
}
int CBasicFunction::CalcSevenParameters_linear_Accurate(int nNum, double * pSrc, double *pDst, double *pSimi)
{
	double pTSrc[4] = { 0 };
	double pTDst[4] = { 0 };
	double *pgSrc = new double[nNum * 3];
	double *pgDst = new double[nNum * 3];

	NomarlizePointCoor(nNum, 3, pSrc, pgSrc, pTSrc);
	NomarlizePointCoor(nNum, 3, pDst, pgDst, pTDst);

	double pVecA[9] = { 0 };
	double pVecB[9] = { 0 };
	FindCrossEigenVectors(3, nNum, pgSrc, pgDst, pVecA, pVecB);

	double r12[9] = { 0 };
	MultiTransMatrix(pVecB, 3, 3, pVecA, 3, 3, r12);
	CBasicFunction BF;
	double deterR = BF.Compute33MatrixDeterminant(r12, 3, 3);
	if (deterR < 0){
		for (int i = 0; i < 9; i++){
			r12[i] = -r12[i];
		}
	}
	double ph, om, kp, scale = 1;
	ComputeMatrixScale(r12, 3, 3, pgSrc, 3, nNum, pgDst, scale);
	/*
	double *pRotatedSrc = new double[nNum * 3];
	MultiMatrix(r12, 3, 3, pgSrc, 3, nNum, pRotatedSrc);
	ComputeMatrixScale(nNum * 3, pRotatedSrc, pgDst, scale);
	delete[] pRotatedSrc; pRotatedSrc = NULL;
	*/
	pSimi[0] = scale*deterR;
	GetAngleWithRotateMatrix(r12, ph, om, kp);
	pSimi[8] = ph;
	pSimi[9] = om;
	pSimi[10] = kp;
	memcpy(pSimi + 1, pTSrc + 1, sizeof(double) * 3);
	memcpy(pSimi + 4, pTDst, sizeof(double) * 4);
	delete[] pgSrc; pgSrc = NULL;
	delete[] pgDst; pgDst = NULL;
	
	return 1;
}
int CBasicFunction::GenerateMiniRandomSample(unsigned int nSampleNum, unsigned int nMinSampleNum, unsigned int* pSampleID)
{
	unsigned int count = 0;
	unsigned int index;
	std::vector<unsigned int>::iterator pos;
	std::vector<unsigned int> veSample;
	veSample.resize(nMinSampleNum);
	pos = veSample.begin();
	do {
		index = rand() % nSampleNum;
		if (find(veSample.begin(), pos, index) == pos)
		{
			veSample[count] = index;
			++count;
			++pos;
		}
	} while (count < nMinSampleNum);

	memcpy(pSampleID, veSample.data(), sizeof(unsigned int) * nMinSampleNum);
	return 1;
}
int CBasicFunction::EvaluateModel(unsigned int nSampleNum, unsigned int nMinSampleNum, unsigned int* pSampleID, int nRow, double* pSrc, double *pDst, double InlierThreshold, unsigned int& nInlierCount, int* pMask)
{
	int i = 0, j = 0, nImgID = 0, nCamID = 0;
	PT2D pt2; PT3D pt3; double d = 0, tmp = 0, scale = 1;
	CBasicFunction BF;
	nInlierCount = 0;
	double pTSrc[4] = { 0 };
	double pTDst[4] = { 0 };
//	double* pgSrc = new double[nMinSampleNum * nRow];
//	double* pgDst = new double[nMinSampleNum * nRow];
	double* pSrcT = new double[nSampleNum * nRow];
	double* pDstT = new double[nSampleNum * nRow];

	double pSrcD[3] = { 0 };
	double pDstD[3] = { 0 };


	NomarlizePointCoor(nSampleNum, pSampleID, nMinSampleNum, pSrc, pTSrc);
	NomarlizePointCoor(nSampleNum, pSampleID, nMinSampleNum, pDst, pTDst);

	for (i = 0; i < nSampleNum; i++) {
		for (j = 0; j < nRow; j++) {
			pSrcT[j * nSampleNum + i] = pSrc[j * nSampleNum + i] - pTSrc[1 + j];
			pDstT[j * nSampleNum + i] = pDst[j * nSampleNum + i] - pTDst[1 + j];
		}
	}
	memcpy(pSrc, pSrcT, sizeof(double) * nSampleNum * nRow);
	memcpy(pDst, pDstT, sizeof(double) * nSampleNum * nRow);

	HouseholderTransformation(nMinSampleNum-1, pSampleID, nRow, nSampleNum, pSrc, NULL, pSrcT);
	HouseholderTransformation(nMinSampleNum-1, pSampleID, nRow, nSampleNum, pDst, NULL, pDstT);
	pSrcD[0] = pSrcT[0 * nSampleNum + pSampleID[0]];
	pSrcD[1] = pSrcT[1 * nSampleNum + pSampleID[1]];
	pSrcD[2] = pSrcT[2 * nSampleNum + pSampleID[2]];
	pDstD[0] = pDstT[0 * nSampleNum + pSampleID[0]];
	pDstD[1] = pDstT[1 * nSampleNum + pSampleID[1]];
	pDstD[2] = pDstT[2 * nSampleNum + pSampleID[2]];
	ComputeMatrixScale(3, pSrcD, pDstD, scale);
	for (i = 0; i < nSampleNum; i++) {
		d = 0;
		for (j = 0; j < nRow; j++) {
			tmp = fabs(pSrcT[j * nSampleNum + i])*scale - fabs(pDstT[j * nSampleNum + i]);
			d += tmp * tmp;
		}
		d = sqrt(d) / pTDst[0];
		if (d < InlierThreshold) {
			nInlierCount++;

		}
		else {
			pMask[i] = -1;
		}
	}
//	delete[] pgSrc; pgSrc = NULL;
//	delete[] pgDst; pgDst = NULL;
	delete[] pSrcT; pSrcT = NULL;
	delete[] pDstT; pDstT = NULL;
	return 1;
}
int CBasicFunction::UpdateStoppingCount(unsigned int nInlierCount, double conf_threshold, unsigned int nTotSample, unsigned int nSampleSize, unsigned int nMaxIteration)
{
	double n_inliers = 1.0;
	double n_pts = 1.0;

	for (unsigned int i = 0; i < nSampleSize; ++i)
	{
		n_inliers *= nInlierCount - i;
		n_pts *= nTotSample - i;
	}
	double prob_good_model = n_inliers / n_pts;

	if (prob_good_model < std::numeric_limits<double>::epsilon())
	{
		return nMaxIteration;
	}
	else if (1 - prob_good_model < std::numeric_limits<double>::epsilon())
	{
		return 1;
	}
	else
	{
		double nusample_s = log(1 - conf_threshold) / log(1 - prob_good_model);
		return (unsigned int)ceil(nusample_s);
	}
	return 1;
}
int CBasicFunction::ComputeSimilarity_RANSAC(int nRow, int nCol, double *pSrc, double *pDst, double inlierThreshold, double* pM, int* pMask)
{
	unsigned int nMinSampleNum = 3;
	unsigned int nInlierCount = 0;
	unsigned int nSampleNum = nCol;
	unsigned int* pSampleID = new unsigned int[nMinSampleNum];
	memset(pSampleID, 0, sizeof(unsigned int) * nMinSampleNum);

	double conf_threshold = 0.99;
	unsigned int nMaxIteration = 1000000;
	unsigned int nIterThreshold = nMaxIteration;
	unsigned int nNewThreshold = 0;
	unsigned int nBestInlierCount = 0;
	unsigned int* pBestSampleID = new unsigned int[nMinSampleNum];
	memset(pBestSampleID, 0, sizeof(unsigned int) * nMinSampleNum);
	int* pBestMask = new int[nSampleNum];
	memset(pBestMask, 0, sizeof(int) * nSampleNum);

	double* pBestModel = new double[12];
	memset(pBestModel, 0, sizeof(double) * 12);

	int nIterCount = 0;
	double* pSrcT = new double[nRow * nCol];
	double* pDstT = new double[nRow * nCol];
	srand(time(NULL));
	do {
		
		GenerateMiniRandomSample(nSampleNum, nMinSampleNum, pSampleID);
		printf("%d %d %d\n", pSampleID[0], pSampleID[1], pSampleID[2]);
		memset(pMask, 0, sizeof(int) * nSampleNum);
		memcpy(pSrcT, pSrc, sizeof(double) * nRow * nCol);
		memcpy(pDstT, pDst, sizeof(double) * nRow * nCol);
		EvaluateModel(nSampleNum, nMinSampleNum, pSampleID, nRow, pSrcT, pDstT, inlierThreshold, nInlierCount, pMask);

		if (nInlierCount > nBestInlierCount) {
			nBestInlierCount = nInlierCount;
		//	memcpy(pBestModel, pM, sizeof(double) * 12);
			memcpy(pBestSampleID, pSampleID, sizeof(unsigned int) * nMinSampleNum);
			memcpy(pBestMask, pMask, sizeof(int) * nSampleNum);
		}

		nNewThreshold = UpdateStoppingCount(nInlierCount, conf_threshold, nSampleNum, nMinSampleNum, nMaxIteration);
		if (nNewThreshold < nIterThreshold) {
			nIterThreshold = nNewThreshold;
		}
		
		nIterCount++;
	} while (nIterCount < nIterThreshold);

	delete[] pSrcT; pSrcT = NULL;
	delete[] pDstT; pDstT = NULL;
	delete[] pSampleID; pSampleID = NULL;
	delete[] pBestSampleID; pBestSampleID = NULL;
	return 1;
}
int CBasicFunction::CalcSevenParameters_linear_fast(bool bBenchMark, int nNum, double * pSrc, double *pDst, double *pSimi)
{
	double pTSrc[4] = { 0 };
	double pTDst[4] = { 0 };
	double *pgSrc = new double[nNum * 3];
	double *pgDst = new double[nNum * 3];

	NomarlizePointCoor(nNum, 3, pSrc, pgSrc, pTSrc);
	NomarlizePointCoor(nNum, 3, pDst, pgDst, pTDst);
	
	int nTransNum = 0;
	double pSrcQ[9] = { 0 }, pDstQ[9] = { 0 };
	double pSrcT[9] = { 0 }, pDstT[9] = { 0 };

	HouseholderTransformation(3, nNum, pgSrc, nTransNum, pSrcQ, pSrcT);
	HouseholderTransformation(3, nNum, pgDst, nTransNum, pDstQ, pDstT);

	double matrixScale = 1;
	ComputeMatrixScale(3, pSrcT, pDstT, matrixScale);
	double deterD = 1;
	double direction[3] = { 0 };
	if (nNum == 3) {
		pSrcT[2] = 1;
		pDstT[2] = 1;
	}
	if (pSrcT[0] * pDstT[0] != 0){ direction[0] = pSrcT[0] * pDstT[0] / fabs(pSrcT[0] * pDstT[0]); deterD *= direction[0]; }
	if (pSrcT[1] * pDstT[1] != 0){ direction[1] = pSrcT[1] * pDstT[1] / fabs(pSrcT[1] * pDstT[1]); deterD *= direction[1]; }
	if (pSrcT[2] * pDstT[2] != 0){ direction[2] = pSrcT[2] * pDstT[2] / fabs(pSrcT[2] * pDstT[2]); deterD *= direction[2]; }

	pSimi[0] = matrixScale*deterD;
	double ph, om, kp, r12[9] = { 0 };
	for (int i = 0; i < 3; i++){
		if (direction[0] != 0) pDstQ[i * 3 + 0] = pDstQ[i * 3 + 0] * direction[0] * deterD;
		if (direction[1] != 0) pDstQ[i * 3 + 1] = pDstQ[i * 3 + 1] * direction[1] * deterD;
		if (direction[2] != 0) pDstQ[i * 3 + 2] = pDstQ[i * 3 + 2] * direction[2] * deterD;
	}
	MultiTransMatrix(pDstQ, 3, 3, pSrcQ, 3, 3, r12);
	GetAngleWithRotateMatrix(r12, ph, om, kp);
	pSimi[8] = ph;
	pSimi[9] = om;
	pSimi[10] = kp;
	memcpy(pSimi + 1, pTSrc + 1, sizeof(double) * 3);
	memcpy(pSimi + 4, pTDst, sizeof(double) * 4);

	delete[] pgSrc; pgSrc = NULL;
	delete[] pgDst; pgDst = NULL;
	return 1;
}
double CBasicFunction::Compute33MatrixDeterminant(double *p, double row, double col)
{
	if (row != 3 || col != 3 ){
		return 0;
	}
	return p[0] * (p[4] * p[8] - p[5] * p[7]) - p[1] * (p[3] * p[8] - p[5] * p[6]) + p[2] * (p[3] * p[7] - p[4] * p[6]);
}
int CBasicFunction::ComputeTranslationAndScaleWithRotation(int nRow, int nCol, double ph, double om, double kp, double *pSrc, double *pDst, double &s, double *t)
{

	return 1;
}
int CBasicFunction::CalcSevenParameters_linear(int nNum, double * pSrc, double *pDst, double *pSimi)
{
	double pTSrc[4] = { 0 };
	double pTDst[4] = { 0 };
	double *pgSrc = new double[nNum * 3];
	double *pgDst = new double[nNum * 3];

	NomarlizePointCoor(nNum, 3, pSrc, pgSrc, pTSrc);
	NomarlizePointCoor(nNum, 3, pDst, pgDst, pTDst);
	
	pSimi[0] = pTDst[0] / pTSrc[0];

	Map <MatrixXd> maSrc(pgSrc, nNum, 3);
	Map <MatrixXd> maDst(pgDst, nNum, 3);

	HouseholderQR<MatrixXd> qrSrc, qrDst;
	qrSrc.compute(maSrc.transpose());
	qrDst.compute(maDst.transpose());

	MatrixXd srcR = qrSrc.matrixQR().triangularView<Upper>();
	MatrixXd srcQ = qrSrc.householderQ();

	MatrixXd dstR = qrDst.matrixQR().triangularView<Upper>();
	MatrixXd dstQ = qrDst.householderQ();
	
	srcR.transposeInPlace();
	dstR.transposeInPlace();
	double *pSrcR = srcR.data();
	double *pDstR = dstR.data();

	double detDstQ = dstQ.determinant();
	double detSrcQ = srcQ.determinant();
	double *pSrcQ = (double*)srcQ.data();
	double *pDstQ = (double*)dstQ.data();
	int direction[3] = {1, 1, 1};
	ComputeVectorDirection(nNum, 3, pSrcR, pDstR, direction);
	double matrixScale = 1;
	ComputeMatrixScale(nNum, pSrcR, pDstR, matrixScale);
	double pDirection[9] = { 0 };
	double detDirect = direction[0] * direction[1] * direction[2];
	pDirection[0] = direction[0] / detDirect;
	pDirection[4] = direction[1] / detDirect;
	pDirection[8] = direction[2] / detDirect;
	Map<Matrix3d> maDirect(pDirection, 3, 3);

	MatrixXd R12 = srcQ*maDirect*dstQ.transpose();
	double *pR12 = R12.data();

	memcpy(pSimi + 1, pTSrc + 1, sizeof(double) * 3);
	memcpy(pSimi + 4, pTDst, sizeof(double) * 4);
	pSimi[0] = matrixScale*detDirect;
	
	double ph, om, kp;
	GetAngleWithRotateMatrix(pR12, ph, om, kp);
	pSimi[8] = ph;
	pSimi[9] = om;
	pSimi[10] = kp;

	delete[] pgSrc; pgSrc = NULL;
	delete[] pgDst; pgDst = NULL;
	return 1;
}
int CBasicFunction::ComputeMatrixScale(double *r12, int aR, int  aC, double * pSrc, int bR, int bC, double *pDst, double &scale)
{

	int i = 0; int j = 0; int k = 0;
	double temp = 0, AB = 0, AA = 0;
	for (i = 0; i < aR; i++) {
		for (j = 0; j < bC; j++) {
			temp = 0;
			for (k = 0; k < aC; k++) {
				temp += r12[i*aC + k] * pSrc[k*bC + j];
			}
			AB += fabs(temp) * fabs(pDst[i*bC+j]);
			AA += temp * temp;
		}
	}
	scale = AB / AA;
	return 1;
}
int CBasicFunction::ComputeMatrixScale(int nNum, double * pSrc, double *pDst, double &scale)
{
	int i = 0;
	double AB = 0, AA = 0;
	for (i = 0; i < nNum; i++){
		AB += fabs(pSrc[i]) * fabs(pDst[i]);
		AA += pSrc[i] * pSrc[i];
	}
	scale = AB / AA;
	return 1;
}
int CBasicFunction::ComputeVectorDirection(int nWidth, int nHeight, double *pSrc, double *pDst, int *pDirection)
{
	int i = 0, j = 0;
	for (i = 0; i < nHeight; i++){
		double AB = 0, AA = 0;
		pDirection[i] = 1;
		for (j = 0; j < nWidth; j++){
			AB += pSrc[i*nWidth + j] * pDst[i*nWidth + j];
			AA += pSrc[i*nWidth + j] * pSrc[i*nWidth + j];
		}
		if (AB / AA < 0) pDirection[i] = -1;
	}
	return 1;
}
int CBasicFunction::CalcSevenParameters(int nNum, PT3D *ptSrc, PT3D *ptDst, double *pSeven)
{
	double *pX = new double[nNum];
	double *pY = new double[nNum];
	double *pZ = new double[nNum];
	memset(pX, 0, sizeof(double)*nNum);
	memset(pY, 0, sizeof(double)*nNum);
	memset(pZ, 0, sizeof(double)*nNum);

	double *pObjX = new double[nNum];
	double *pObjY = new double[nNum];
	double *pObjZ = new double[nNum];
	memset(pObjX, 0, sizeof(double)*nNum);
	memset(pObjY, 0, sizeof(double)*nNum);
	memset(pObjZ, 0, sizeof(double)*nNum);

	for (int i = 0; i < nNum; i++){
		pX[i] = ptSrc[i].X;
		pY[i] = ptSrc[i].Y;
		pZ[i] = ptSrc[i].Z;

		pObjX[i] = ptDst[i].X;
		pObjY[i] = ptDst[i].Y;
		pObjZ[i] = ptDst[i].Z;
	}
	CalcSevenParameters(nNum, pX, pY, pZ, pObjX, pObjY, pObjZ, pSeven);

	delete[] pX; pX = NULL;
	delete[] pY; pY = NULL;
	delete[] pZ; pZ = NULL;
	delete[] pObjX; pObjX = NULL;
	delete[] pObjY; pObjY = NULL;
	delete[] pObjZ; pObjZ = NULL;
	return 1;
}
int CBasicFunction::CalcSevenParameters_Direct(int nNum, double * pX, double *pY, double *pZ, double *pObjX, double *pObjY, double *pObjZ, double *pSeven)
{
	int i = 0; int j = 0; int k = 0; int l = 0; int m = 0;
	double thres = 100.0; double sumX, sumY, sumZ, sumXo, sumYo, sumZo, Xg, Yg, Zg, Xog, Yog, Zog;
	double sumsqX, sumsqY, sumsqZ, sumsqXo, sumsqYo, sumsqZo, sc, scObj;
	double A[21] = { 0 }; double L[3] = { 0 }; double AA[49] = { 0 }; double AL[7] = { 0 };
	double * pXG = new double[nNum]; memset(pXG, 0, sizeof(double) * nNum);
	double * pYG = new double[nNum]; memset(pYG, 0, sizeof(double) * nNum);
	double * pZG = new double[nNum]; memset(pZG, 0, sizeof(double) * nNum);
	double * pXOG = new double[nNum]; memset(pXOG, 0, sizeof(double) * nNum);
	double * pYOG = new double[nNum]; memset(pYOG, 0, sizeof(double) * nNum);
	double * pZOG = new double[nNum]; memset(pZOG, 0, sizeof(double) * nNum);

	sumX = sumY = sumZ = sumXo = sumYo = sumZo = 0;
	sumsqX = sumsqY = sumsqZ = sumsqXo = sumsqYo = sumsqZo = sc = scObj = 0;
	for (i = 0; i < nNum; i++){
		sumX += pX[i]; sumY += pY[i]; sumZ += pZ[i];
		sumXo += pObjX[i]; sumYo += pObjY[i]; sumZo += pObjZ[i];
	}
	Xg = sumX / nNum; Yg = sumY / nNum; Zg = sumZ / nNum;
	Xog = sumXo / nNum; Yog = sumYo / nNum; Zog = sumZo / nNum;
	for (i = 0; i < nNum; i++){
		pXG[i] = pX[i] - Xg;
		pYG[i] = pY[i] - Yg;
		pZG[i] = pZ[i] - Zg;
		pXOG[i] = pObjX[i] - Xog;
		pYOG[i] = pObjY[i] - Yog;
		pZOG[i] = pObjZ[i] - Zog;

		sumsqX += pXG[i] * pXG[i]; sumsqY += pYG[i] * pYG[i]; sumsqZ += pZG[i] * pZG[i];
		sumsqXo += pXOG[i] * pXOG[i]; sumsqYo += pYOG[i] * pYOG[i]; sumsqZo += pZOG[i] * pZOG[i];
	}
	sumsqX = sqrt(sumsqX / nNum); sumsqY = sqrt(sumsqY / nNum); sumsqZ = sqrt(sumsqZ / nNum);
	sumsqXo = sqrt(sumsqXo / nNum); sumsqYo = sqrt(sumsqYo / nNum); sumsqZo = sqrt(sumsqZo / nNum);
	sc = sqrt(sumsqX*sumsqX + sumsqY*sumsqY + sumsqZ*sumsqZ);
	scObj = sqrt(sumsqXo*sumsqXo + sumsqYo*sumsqYo + sumsqZo*sumsqZo);

	for (i = 0; i < nNum; i++){
	//	pXG[i] *= scObj / sc;
	//	pYG[i] *= scObj / sc;
	//	pZG[i] *= scObj / sc;
	}
	
	pSeven[3] = scObj / sc;
//	pSeven[0] = Xog - pSeven[3] * Xg; pSeven[1] = Yog - pSeven[3] * Yg; pSeven[2] = Zog - pSeven[3]*Zg;
	pSeven[7] = Xg; pSeven[8] = Yg; pSeven[9] = Zg;
	pSeven[10] = Xog; pSeven[11] = Yog; pSeven[12] = Zog;
	pSeven[13] = sc; pSeven[14] = scObj;

	do
	{
		memset(AA, 0, sizeof(double) * 49);
		memset(AL, 0, sizeof(double) * 7);
		for (i = 0; i < nNum; i++){
			double a[9], X, Y, Z, Xo, Yo, Zo, S, Xt, Yt, Zt, Ph, Om, Kp;
			S = pSeven[3], Ph = pSeven[4]; Om = pSeven[5]; Kp = pSeven[6];
			GetRotateMatrixWithAngle(a, Ph, Om, Kp);
		//	X = pX[i]; Y = pY[i]; Z = pZ[i]; Xo = pObjX[i]; Yo = pObjY[i]; Zo = pObjZ[i];
			X = pXG[i]; Y = pYG[i]; Z = pZG[i]; Xo = pXOG[i]; Yo = pYOG[i]; Zo = pZOG[i];

			Xt = a[0] * X + a[1] * Y + a[2] * Z;
			Yt = a[3] * X + a[4] * Y + a[5] * Z;
			Zt = a[6] * X + a[7] * Y + a[8] * Z;
			A[0 * 7 + 0] = 1; A[0 * 7 + 1] = 0; A[0 * 7 + 2] = 0; A[0 * 7 + 3] = Xt; A[0 * 7 + 4] = -S*Zt; A[0 * 7 + 5] = -S*Yt*sin(Ph);            A[0 * 7 + 6] = -S*Yt*cos(Ph)*cos(Om) - S*Zt*sin(Om);
			A[1 * 7 + 0] = 0; A[1 * 7 + 1] = 1; A[1 * 7 + 2] = 0; A[1 * 7 + 3] = Yt; A[1 * 7 + 4] = 0;    A[1 * 7 + 5] = S*Xt*sin(Ph) - S*Zt*cos(Ph); A[1 * 7 + 6] = S*Xt*cos(Ph)*cos(Om) + S*Zt*sin(Ph)*cos(Om);
			A[2 * 7 + 0] = 0; A[2 * 7 + 1] = 0; A[2 * 7 + 2] = 1; A[2 * 7 + 3] = Zt; A[2 * 7 + 4] = S*Xt; A[2 * 7 + 5] = S*Yt*cos(Ph);              A[2 * 7 + 6] = S*Xt*sin(Om) - S*Yt*sin(Ph)*cos(Om);
			L[0] = Xo - pSeven[0] - S*Xt;
			L[1] = Yo - pSeven[1] - S*Yt;
			L[2] = Zo - pSeven[2] - S*Zt;

			for (j = 0; j < 7; j++){
				for (k = 0; k < 7; k++){
					for (l = 0; l < 3; l++){ AA[j * 7 + k] += A[l * 7 + j] * A[l * 7 + k]; }
				}
				for (k = 0; k < 3; k++){ AL[j] += A[k * 7 + j] * L[k]; }
			}
		}
		CXMatrix maN, ma_N, maL, maU;
		maN.InitMatrix(AA, 7, 7);
		maL.InitMatrix(AL, 7, 1);
		ma_N = maN.InverseMatrix();
		maU = ma_N * maL;
		if (maU.GetColumn() == 0){
			delete[] pXG;
			delete[] pYG;
			delete[] pZG;
			delete[] pXOG;
			delete[] pYOG;
			delete[] pZOG;
			return 0;
		}
		for (i = 0; i < 7; i++){
		//	if (i == 3) pSeven[i] *= (1 + maU.GetData()[i]);
		//	else pSeven[i] += maU.GetData()[i];
			pSeven[i] += maU.GetData()[i];
		}

		thres = maU.GetMaxFabsElement();
		m++;

	} while (fabs(thres) > 1e-6 && m < 100);


	if (m >= 20){
		delete[] pXG;
		delete[] pYG;
		delete[] pZG;
		delete[] pXOG;
		delete[] pYOG;
		delete[] pZOG;
		return 0;
	}
	delete[] pXG;
	delete[] pYG;
	delete[] pZG;
	delete[] pXOG;
	delete[] pYOG;
	delete[] pZOG;
	return 1;
}
int CBasicFunction::CalcSevenParameters( int nNum, double * pX, double *pY, double *pZ, double *pObjX, double *pObjY, double *pObjZ, double *pSeven )
{
	int i = 0;int j = 0;int k = 0;int l = 0;int m = 0;
	double thres = 100.0;double sumX,sumY,sumZ,sumXo,sumYo,sumZo,Xg,Yg,Zg, Xog,Yog,Zog;
	double sumsqX, sumsqY, sumsqZ, sumsqXo, sumsqYo, sumsqZo, sc, scObj;
	double A[21] = {0};double L[3] = {0};double AA[49] = {0};double AL[7] = {0};
	double * pXG = new double[nNum];memset( pXG, 0, sizeof(double) * nNum );
	double * pYG = new double[nNum];memset( pYG, 0, sizeof(double) * nNum );
	double * pZG = new double[nNum];memset( pZG, 0, sizeof(double) * nNum );
	double * pXOG = new double[nNum];memset( pXOG, 0, sizeof(double) * nNum );
	double * pYOG = new double[nNum];memset( pYOG, 0, sizeof(double) * nNum );
	double * pZOG = new double[nNum];memset( pZOG, 0, sizeof(double) * nNum );
	
	sumX = sumY = sumZ = sumXo = sumYo = sumZo = 0;
	sumsqX = sumsqY = sumsqZ = sumsqXo = sumsqYo = sumsqZo = sc = scObj = 0;
	for( i = 0;i < nNum;i ++ ){
		sumX += pX[i];sumY += pY[i];sumZ += pZ[i];
		sumXo += pObjX[i];sumYo += pObjY[i];sumZo += pObjZ[i];	
	}
	Xg = sumX/nNum;Yg = sumY/nNum;Zg = sumZ/nNum;
	Xog = sumXo/nNum;Yog = sumYo/nNum;Zog = sumZo/nNum;
	for( i = 0;i < nNum;i ++ ){
		pXG[i] = pX[i] - Xg;
		pYG[i] = pY[i] - Yg;
		pZG[i] = pZ[i] - Zg;
		pXOG[i] = pObjX[i] - Xog;
		pYOG[i] = pObjY[i] - Yog;
		pZOG[i] = pObjZ[i] - Zog;

		sumsqX += pXG[i]*pXG[i];sumsqY += pYG[i]*pYG[i];sumsqZ += pZG[i]*pZG[i];
		sumsqXo += pXOG[i]*pXOG[i];sumsqYo += pYOG[i]*pYOG[i];sumsqZo += pZOG[i]*pZOG[i];
	}
	sumsqX = sqrt(sumsqX/nNum);sumsqY = sqrt(sumsqY/nNum);sumsqZ = sqrt(sumsqZ/nNum);
	sumsqXo = sqrt(sumsqXo/nNum);sumsqYo = sqrt(sumsqYo/nNum);sumsqZo = sqrt(sumsqZo/nNum);
	sc = sqrt( sumsqX*sumsqX + sumsqY*sumsqY + sumsqZ*sumsqZ );
	scObj = sqrt( sumsqXo*sumsqXo + sumsqYo*sumsqYo + sumsqZo*sumsqZo );

	for( i = 0;i < nNum;i ++ ){
		pXG[i] *= scObj/sc;
		pYG[i] *= scObj/sc;
		pZG[i] *= scObj/sc;
	}	
	pSeven[3] = 1;
	pSeven[7] = Xg;pSeven[8] = Yg;pSeven[9] = Zg;
	pSeven[10] = Xog;pSeven[11] = Yog;pSeven[12] = Zog;
	pSeven[13] = sc;pSeven[14] = scObj;

	do 
	{
		memset( AA, 0, sizeof(double) * 49 );
		memset( AL, 0, sizeof(double)  *7 );
		for( i = 0;i < nNum;i ++ ){
			double a[9], X, Y, Z, Xo, Yo, Zo, S, Xt,Yt,Zt, Ph, Om, Kp;
			S = pSeven[3], Ph = pSeven[4]; Om = pSeven[5]; Kp = pSeven[6];
			GetRotateMatrixWithAngle( a, Ph, Om, Kp );
			X = pXG[i];Y = pYG[i];Z = pZG[i];Xo = pXOG[i];Yo = pYOG[i];Zo = pZOG[i];


			Xt = a[0]*X+a[1]*Y+a[2]*Z;
			Yt = a[3]*X+a[4]*Y+a[5]*Z;
			Zt = a[6]*X+a[7]*Y+a[8]*Z;
			A[0*7+0] = 1;A[0*7+1] = 0;A[0*7+2] = 0;A[0*7+3] = Xt;A[0*7+4] = -S*Zt;A[0*7+5] = -S*Yt*sin(Ph);            A[0*7+6] = -S*Yt*cos(Ph)*cos(Om)-S*Zt*sin(Om);
			A[1*7+0] = 0;A[1*7+1] = 1;A[1*7+2] = 0;A[1*7+3] = Yt;A[1*7+4] = 0;    A[1*7+5] = S*Xt*sin(Ph)-S*Zt*cos(Ph);A[1*7+6] = S*Xt*cos(Ph)*cos(Om)+S*Zt*sin(Ph)*cos(Om);
			A[2*7+0] = 0;A[2*7+1] = 0;A[2*7+2] = 1;A[2*7+3] = Zt;A[2*7+4] = S*Xt; A[2*7+5] =S*Yt*cos(Ph);              A[2*7+6] = S*Xt*sin(Om)-S*Yt*sin(Ph)*cos(Om);
			L[0] = Xo - pSeven[0] - S*Xt;
			L[1] = Yo - pSeven[1] - S*Yt;
			L[2] = Zo - pSeven[2] - S*Zt;

			for( j = 0;j < 7;j ++ ){
				for( k = 0;k < 7;k ++ ){
					for( l = 0;l < 3;l ++ ){ AA[j*7+k] += A[l*7+j] * A[l*7+k]; }
				}
				for( k = 0;k < 3;k ++ ){ AL[j] += A[k*7+j] * L[k]; }
			}
		}
		CXMatrix maN, ma_N, maL, maU;
		maN.InitMatrix( AA, 7, 7 );
		maL.InitMatrix( AL, 7, 1 );
		ma_N = maN.InverseMatrix();
		maU = ma_N * maL;
		if (maU.GetColumn() == 0){
			delete[] pXG;
			delete[] pYG;
			delete[] pZG;
			delete[] pXOG;
			delete[] pYOG;
			delete[] pZOG;
			return 0;
		}
		for( i = 0;i < 7;i ++ ){
			if( i == 3 ) pSeven[i] *= (1+ maU.GetData()[i]);
			else pSeven[i] += maU.GetData()[i]; 
		}

		thres = maU.GetMaxFabsElement();
		m ++;

	} while ( fabs(thres) > 0.00001 && m < 20 );


	TransDataWithSevenParameters( nNum, pSeven, pX, pY, pZ, pXG, pYG, pZG );

	if( m >= 20 ){
		delete [] pXG;
		delete [] pYG;
		delete [] pZG;
		delete [] pXOG;
		delete [] pYOG;
		delete [] pZOG;
		return 0;
	}
	delete [] pXG;
	delete [] pYG;
	delete [] pZG;
	delete [] pXOG;
	delete [] pYOG;
	delete [] pZOG;
	return 1;
}
int CBasicFunction::TransPointsWithSevenParameters(int nNum, double *pSimi, PT3D * ptSrc, PT3D *ptDst, double *pErr)
{
	double *pSrc = new double[3 * nNum];
	double *pDst = new double[3 * nNum];
	for (int i = 0; i < nNum; i++){

		pSrc[3 * i + 0] = ptSrc[i].X;
		pSrc[3 * i + 1] = ptSrc[i].Y;
		pSrc[3 * i + 2] = ptSrc[i].Z;

		pDst[3 * i + 0] = ptDst[i].X;
		pDst[3 * i + 1] = ptDst[i].Y;
		pDst[3 * i + 2] = ptDst[i].Z;
	}
	if (TransPointsWithSevenParameters(nNum, pSimi, pSrc, pDst, pErr) == 0){

		delete[] pSrc; pSrc = NULL;
		delete[] pDst; pDst = NULL;
		return 0;

	}
	for (int i = 0; i < nNum; i++){

		ptDst[i].X = pDst[3 * i + 0];
		ptDst[i].Y = pDst[3 * i + 1];
		ptDst[i].Z = pDst[3 * i + 2];
	}
	delete[] pSrc; pSrc = NULL;
	delete[] pDst; pDst = NULL;
	return 1;
}
int CBasicFunction::TransPointsWithSevenParameters(int nNum, double *pSimi, double * pSrc, double *pDst, double *pErr)
{
	int i = 0;
	double X, Y, Z, ph, om, kp;
	double r[9] = { 0 };
	ph = pSimi[8]; om = pSimi[9]; kp = pSimi[10];
	GetRotateMatrixWithAngle(r, ph, om, kp);
	for (i = 0; i < nNum; i++){
		X = (pSrc[3 * i + 0] - pSimi[1]) * pSimi[0];
		Y = (pSrc[3 * i + 1] - pSimi[2]) * pSimi[0];
		Z = (pSrc[3 * i + 2] - pSimi[3]) * pSimi[0];
		double tX, tY, tZ;
		tX = r[0] * X + r[1] * Y + r[2] * Z;
		tY = r[3] * X + r[4] * Y + r[5] * Z;
		tZ = r[6] * X + r[7] * Y + r[8] * Z;

		X = tX + pSimi[5];
		Y = tY + pSimi[6];
		Z = tZ + pSimi[7];

		pErr[3 * i + 0] = pDst[3 * i + 0] - X;
		pErr[3 * i + 1] = pDst[3 * i + 1] - Y;
		pErr[3 * i + 2] = pDst[3 * i + 2] - Z;

	//	pDst[3 * i + 0] = X;
	//	pDst[3 * i + 1] = Y;
	//	pDst[3 * i + 2] = Z;
	}
	return 1;
}
int CBasicFunction::TransRotationWithSevenParameters(int nNum, double *pSimi, double * ptSrc, double *ptDst, double *pErr)
{
	int i = 0;
	double a[9] = { 0 };
	GetRotateMatrixWithAngle(a, pSimi[8], pSimi[9], pSimi[10]);

	CXMatrix maA, maB, maAB;
	maA.InitMatrix(a, 3, 3);
	for (i = 0; i < nNum; i++){
		double b[9] = { 0 };
		GetRotateMatrixWithAngle(b, ptSrc[3 * i + 0], ptSrc[3 * i + 1], ptSrc[3 * i + 2]);
		maB.InitMatrix(b, 3, 3);
		maAB = maA * maB;
		double ph, om, kp;
		GetAngleWithRotateMatrix(maAB.GetData(), ph, om, kp);
		if (pErr){
			pErr[3 * i + 0] = ptDst[3 * i + 0] - ph;
			pErr[3 * i + 1] = ptDst[3 * i + 1] - om;
			pErr[3 * i + 2] = ptDst[3 * i + 2] - kp;
		}
		ptDst[3 * i + 0] = ph;
		ptDst[3 * i + 1] = om;
		ptDst[3 * i + 2] = kp;
	}
	return 1;
}
int CBasicFunction::TransDataWithSevenParameters(int nNum, double *pSeven, PT3D *ptSrc, PT3D *ptDst, double *pErr)
{
	double *pX = new double[nNum];
	double *pY = new double[nNum];
	double *pZ = new double[nNum];
	memset(pX, 0, sizeof(double)*nNum);
	memset(pY, 0, sizeof(double)*nNum);
	memset(pZ, 0, sizeof(double)*nNum);

	double *pObjX = new double[nNum];
	double *pObjY = new double[nNum];
	double *pObjZ = new double[nNum];
	memset(pObjX, 0, sizeof(double)*nNum);
	memset(pObjY, 0, sizeof(double)*nNum);
	memset(pObjZ, 0, sizeof(double)*nNum);

	for (int i = 0; i < nNum; i++){
		pX[i] = ptSrc[i].X;
		pY[i] = ptSrc[i].Y;
		pZ[i] = ptSrc[i].Z;

		pObjX[i] = ptDst[i].X;
		pObjY[i] = ptDst[i].Y;
		pObjZ[i] = ptDst[i].Z;
	}
	TransDataWithSevenParameters(nNum, pSeven, pX, pY, pZ, pObjX, pObjY, pObjZ, pErr);

	for (int i = 0; i < nNum; i++){

	//	ptDst[i].X = pObjX[i];
	//	ptDst[i].Y = pObjY[i];
	//	ptDst[i].Z = pObjZ[i];
	}

	delete[] pX; pX = NULL;
	delete[] pY; pY = NULL;
	delete[] pZ; pZ = NULL;
	delete[] pObjX; pObjX = NULL;
	delete[] pObjY; pObjY = NULL;
	delete[] pObjZ; pObjZ = NULL;
	return 1;
}
int CBasicFunction::TransDataWithSevenParameters( int nNum, double *pSeven, double * pX, double *pY, double *pZ, double *pObjX, double *pObjY, double *pObjZ, double *pErr )
{
	int i = 0;double * p7 = pSeven;
	double a[9] = {0};double X, Y, Z, Xo, Yo, Zo;
	GetRotateMatrixWithAngle( a, p7[4], p7[5], p7[6] );
	double Xg = p7[7];double Yg = p7[8];double Zg = p7[9];
	double Xog = p7[10];double Yog = p7[11];double Zog = p7[12];
	double sc = p7[14]/p7[13];
	for( i = 0;i < nNum;i ++ ){
		X = (pX[i] - Xg)*sc;Y = (pY[i] - Yg)*sc;Z = (pZ[i] - Zg)*sc;
		Xo = p7[3]*( a[0]*X + a[1]*Y + a[2]*Z ) + p7[0];
		Yo = p7[3]*( a[3]*X + a[4]*Y + a[5]*Z ) + p7[1];
		Zo = p7[3]*( a[6]*X + a[7]*Y + a[8]*Z ) + p7[2];

		if (pErr){
			pErr[3 * i + 0] = Xo + Xog - pObjX[i];
			pErr[3 * i + 1] = Yo + Yog - pObjY[i];
			pErr[3 * i + 2] = Zo + Zog - pObjZ[i];
		}
		pObjX[i] = Xo + Xog;
		pObjY[i] = Yo + Yog;
		pObjZ[i] = Zo + Zog;

	}
	return 1;
}
int CBasicFunction::TransDataWithSevenParameters_Direct(int nNum, double *pSeven, double * pX, double *pY, double *pZ, double *pObjX, double *pObjY, double *pObjZ, double *pErr)
{
	int i = 0; double * p7 = pSeven;
	double a[9] = { 0 }; double X, Y, Z, Xo, Yo, Zo;
	GetRotateMatrixWithAngle(a, p7[4], p7[5], p7[6]);

	for (i = 0; i < nNum; i++){
		X = pX[i] - pSeven[7]; Y = pY[i] - pSeven[8]; Z = pZ[i] - pSeven[9];
		Xo = p7[3] * (a[0] * X + a[1] * Y + a[2] * Z) + p7[0];
		Yo = p7[3] * (a[3] * X + a[4] * Y + a[5] * Z) + p7[1];
		Zo = p7[3] * (a[6] * X + a[7] * Y + a[8] * Z) + p7[2];

		if (pErr){
			pErr[3 * i + 0] = Xo + pSeven[10]- pObjX[i];
			pErr[3 * i + 1] = Yo + pSeven[11] - pObjY[i];
			pErr[3 * i + 2] = Zo + pSeven[12] - pObjZ[i];
		}
		pObjX[i] = Xo;
		pObjY[i] = Yo;
		pObjZ[i] = Zo;

	}
	return 1;
}
int CBasicFunction::RotateAngleWithSevenParameters( int nNum, double *pSeven, double * pPh, double *pOm, double *pKp, double *pObjPh, double *pObjOm, double *pObjKp )
{
	int i = 0;double * p7 = pSeven;
	double a[9] = {0};
	GetRotateMatrixWithAngle( a, p7[4], p7[5], p7[6] );
	double Xg = p7[7];double Yg = p7[8];double Zg = p7[9];
	double Xog = p7[10];double Yog = p7[11];double Zog = p7[12];
	CXMatrix maA, maB, maAB;
	maA.InitMatrix( a, 3, 3 );
	for( i = 0;i < nNum;i ++ ){
		double b[9] = {0};
		GetRotateMatrixWithAngle( b, pPh[i], pOm[i], pKp[i] );
		maB.InitMatrix( b, 3, 3 );
		maAB = maA * maB;
		GetAngleWithRotateMatrix( maAB.GetData(), pObjPh[i], pObjOm[i], pObjKp[i] );

	}
	return 1;
}
//From v1 to v2
int CBasicFunction::ComputeVectorAngle(double *v1, double *v2, double &ang)
{
	double len1, len2, cos_v = 0;
	len1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
	len2 = sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
	cos_v = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (len1*len2);

	ang = acos(cos_v);


	return 1;
}
int CBasicFunction::ComputeCrossProduct(double *v1, double *v2, double *p)
{
	p[0] = v1[1] * v2[2] - v1[2] * v2[1];
	p[1] = -(v1[0] * v2[2] - v1[2] * v2[0]);
	p[2] = v1[0] * v2[1] - v1[1] * v2[0];
	return 1;
}
int CBasicFunction::SolveEssenMatrix_LeastSquare(int nNum, PT2D *pt2, CAM &cam, double *pMd, int &minErrPtID)
{
	int i, j, k;
	double L[9] = { 0 };
	double A[9] = { 0 };
	double AtA[64] = { 0 };
	double Atb[8] = { 0 };
	PT2D tp2[2];
	double xl, yl, xr, yr, x1, y1, x2, y2;
	double aver_dis1, aver_dis2;
	aver_dis1 = aver_dis2 = 1;

	xl = yl = xr = yr = 0;

	for (i = 0; i < 2*nNum; i++){

		pt2[i].x = (pt2[i].x - cam.x0) / cam.fx;
		pt2[i].y = (pt2[i].y - cam.y0) / cam.fy;

	}
	
	double *pA = new double[nNum * 9];
	for (i = 0; i < nNum; i++){

		x1 = pt2[2 * i + 0].x;
		y1 = pt2[2 * i + 0].y;
		x2 = pt2[2 * i + 1].x;
		y2 = pt2[2 * i + 1].y;

		A[0] = x2*x1;
		A[1] = x2*y1;
		A[2] = x2;
		A[3] = y2*x1;
		A[4] = y2*y1;
		A[5] = y2;
		A[6] = x1;
		A[7] = y1;
		A[8] = 1;

		memcpy(pA + i * 9, A, sizeof(double) * 9);
		for (j = 0; j < 8; j++){
			for (k = 0; k < 8; k++){
				AtA[j * 8 + k] += A[j] * A[k];
			}
			Atb[j] += A[j] * (-A[8]);
		}
	}
	CXMatrix maAtA, maAtb, maX, maT1, maT2, maTET, maK1, maK2, maE;
	maAtA.InitMatrix(AtA, 8, 8);
	maAtb.InitMatrix(Atb, 8, 1);
	maX = maAtA.InverseMatrix()*maAtb;
	double *p = maX.GetData();

	/*
	Mat mA(nNum, 9, CV_64F, pA);
	Mat U, D, Vt;
	SVD::compute(mA, D, U, Vt);
	double *pD = (double*)D.data;
	double d = pD[7] + pD[8];
	*/
	double T1[9] = { 0 };
	double T2[9] = { 0 };
	double E[9] = { 0 };
	memcpy(E, p, sizeof(double) * 8); E[8] = 1;

	//////////////////////////////////////////////////////////////////////////////////

	double fx1, fy1, fx2, fy2, f12, xEx, sum = 0, sqsum = 0;
	double *pxEx = new double[nNum];
	memset(pxEx, 0, sizeof(double)*nNum);
	double *pSampson = new double[nNum];
	memset(pSampson, 0, sizeof(double)*nNum);
	double sampson_err = 0;
	double minErr = 9999999;

	for (i = 0; i < nNum; i++){

		x1 = pt2[2 * i + 0].x;
		y1 = pt2[2 * i + 0].y;
		x2 = pt2[2 * i + 1].x;
		y2 = pt2[2 * i + 1].y;

		fx2 = E[0] * x2 + E[3] * y2 + E[6] * 1;
		fy2 = E[1] * x2 + E[4] * y2 + E[7] * 1;
		f12 = E[2] * x2 + E[5] * y2 + E[8] * 1;

		fx1 = E[0] * x1 + E[1] * y1 + E[2] * 1;
		fy1 = E[3] * x1 + E[4] * y1 + E[5] * 1;

		xEx = x1*fx2 + y1*fy2 + f12;

		sampson_err = xEx*xEx / (fx2*fx2 + fy2*fy2 + fx1*fx1 + fy1*fy1);

		pSampson[i] = sampson_err;

		pxEx[i] = xEx;

		if (fabs(sampson_err) < minErr){
			minErr = fabs(sampson_err);
			minErrPtID = i;
		}
		sum += xEx;
		sqsum += xEx * xEx;
	}
	sum /= nNum;
	sqsum = sqrt(sqsum / nNum);

	if (sqsum > 10){

	}
	memcpy(pMd, E, sizeof(double) * 9);
	delete[] pSampson; pSampson = NULL;
	delete[] pxEx; pxEx = NULL;
	return 1;
	return 1;
}
int CBasicFunction::SolveFundMatrix_LeastSquare(int nNum, PT2D *pt2, double *pMd, int &minErrPtID)
{
	int i, j, k;
	double L[9] = { 0 };
	double A[9] = { 0 };
	double AtA[64] = { 0 };
	double Atb[8] = { 0 };
	PT2D tp2[2];
	double xl, yl, xr, yr, x1, y1, x2, y2;
	double aver_dis1, aver_dis2;
	aver_dis1 = aver_dis2 = 1;

	xl = yl = xr = yr = 0;

	for (i = 0; i < nNum; i++){

		xl += pt2[2 * i + 0].x;
		yl += pt2[2 * i + 0].y;

		xr += pt2[2 * i + 1].x;
		yr += pt2[2 * i + 1].y;

	}
	xl /= nNum;
	yl /= nNum;
	xr /= nNum;
	yr /= nNum;
	for (i = 0; i < nNum; i++){

		tp2[0] = pt2[2 * i + 0];
		tp2[0].x -= float(xl);
		tp2[0].y -= float(yl);
		aver_dis1 += tp2[0].x*tp2[0].x + tp2[0].y*tp2[0].y;

		tp2[1] = pt2[2 * i + 1];
		tp2[1].x -= float(xr);
		tp2[1].y -= float(yr);
		aver_dis2 += tp2[1].x*tp2[1].x + tp2[1].y*tp2[1].y;
	}
	aver_dis1 /= 2 * nNum;
	aver_dis2 /= 2 * nNum;
	aver_dis1 = sqrt(aver_dis1);
	aver_dis2 = sqrt(aver_dis2);

	double *pA = new double[nNum * 9];
	for (i = 0; i < nNum; i++){

		x1 = (pt2[2 * i + 0].x - xl) / aver_dis1;
		y1 = (pt2[2 * i + 0].y - yl) / aver_dis1;
		x2 = (pt2[2 * i + 1].x - xr) / aver_dis2;
		y2 = (pt2[2 * i + 1].y - yr) / aver_dis2;

		A[0] = x2*x1;
		A[1] = x2*y1;
		A[2] = x2;
		A[3] = y2*x1;
		A[4] = y2*y1;
		A[5] = y2;
		A[6] = x1;
		A[7] = y1;
		A[8] = 1;

		memcpy(pA + i * 9, A, sizeof(double) * 9);
		for (j = 0; j < 8; j++){
			for (k = 0; k < 8; k++){
				AtA[j * 8 + k] += A[j] * A[k];
			}
			Atb[j] += A[j] * (-A[8]);
		}
	}
	CXMatrix maAtA, maAtb, maX, maF, maT1, maT2, maTFT, maK1, maK2, maE;
	maAtA.InitMatrix(AtA, 8, 8);
	maAtb.InitMatrix(Atb, 8, 1);
	maX = maAtA.InverseMatrix()*maAtb;
	double *p = maX.GetData();

	/*
	Mat mA(nNum, 9, CV_64F, pA);
	Mat U, D, Vt;
	SVD::compute(mA, D, U, Vt);
	double *pD = (double*)D.data;
	double d = pD[7] + pD[8];
	*/

	double T1[9] = { 0 };
	double T2[9] = { 0 };
	double F[9] = { 0 };
	double E[9] = { 0 };
	memcpy(F, p, sizeof(double) * 8); F[8] = 1;

	for (int i = 0; i < 9; i++){
		if (F[i] == 0){
			int test = 1;
		}
	}
	/*
	double normF = 0;
	for (i = 0; i < 9; i++){
		normF += F[i] * F[i];
	}
	normF = sqrt(normF);
	for (i = 0; i < 9; i++){
		F[i] /= normF;
	}

	*/
	T1[0] = T1[4] = 1 / aver_dis1; T1[8] = 1;
	T1[2] = -xl / aver_dis1; T1[5] = -yl / aver_dis1;
	T2[0] = T2[4] = 1 / aver_dis2; T2[8] = 1;
	T2[2] = -xr / aver_dis2; T2[5] = -yr / aver_dis2;
	maTFT.InitMatrix(F, 3, 3);
	maT1.InitMatrix(T1, 3, 3);
	maT2.InitMatrix(T2, 3, 3);
	maF = maT2.TransposeMatrix()*maTFT*maT1;
	memcpy(F, maF.GetData(), sizeof(double) * 9);


	//////////////////////////////////////////////////////////////////////////////////

	double fx1, fy1, fx2, fy2, f12, xFx, sum = 0, sqsum = 0;
	double *pxFx = new double[nNum];
	memset(pxFx, 0, sizeof(double)*nNum);
	double *pSampson = new double[nNum];
	memset(pSampson, 0, sizeof(double)*nNum);
	double sampson_err = 0;
	double minErr = 9999999;
	std::vector<double> vDis1;
	std::vector<double> vDis2;
	for (i = 0; i < nNum; i++){

		x1 = pt2[2 * i + 0].x;
		y1 = pt2[2 * i + 0].y;
		x2 = pt2[2 * i + 1].x;
		y2 = pt2[2 * i + 1].y;

		fx2 = F[0] * x2 + F[3] * y2 + F[6] * 1;
		fy2 = F[1] * x2 + F[4] * y2 + F[7] * 1;
		f12 = F[2] * x2 + F[5] * y2 + F[8] * 1;

		fx1 = F[0] * x1 + F[1] * y1 + F[2] * 1;
		fy1 = F[3] * x1 + F[4] * y1 + F[5] * 1;

		xFx = x1*fx2 + y1*fy2 + f12;

		sampson_err = xFx*xFx / (fx2*fx2 + fy2*fy2 + fx1*fx1 + fy1*fy1);

		pSampson[i] = sampson_err;

		pxFx[i] = xFx;

		if (fabs(sampson_err) < minErr){
			minErr = fabs(sampson_err);
			minErrPtID = i;
		}
		sum += xFx;
		sqsum += xFx * xFx;
	}
	sum /= nNum;
	sqsum = sqrt(sqsum / nNum);

	if (sqsum > 10 ){

	}
	memcpy(pMd, F, sizeof(double) * 9);
	delete[] pSampson; pSampson = NULL;
	delete[] pxFx; pxFx = NULL;
	return 1;
}
int CBasicFunction::UpdateNormalMatrix(double * pSub, int nSubR, int nSubC, int nBegR, int nBegC, double * pData, int nR, int nC)
{
	int i, j;
	if (pSub == NULL || pData == NULL) return 0;
	if (nSubC <= 0 || nSubR <= 0 || nC <= 0 || nR <= 0) return 0;
	if (nSubC > nC || nSubR > nR) return 0;
	if (nBegC < 0 || nBegR < 0) return 0;
	if (nBegC + nSubC > nC || nBegR + nSubR > nR) return 0;

	for (i = 0; i < nSubR; i++){
		for (j = 0; j < nSubC; j++){
			if ((nBegR + i)*nC + nBegC + j >= nR*nC){
				printf("Error!Sub matrix out of size!\n");
				return 0;
			}
			pData[(nBegR + i)*nC + nBegC + j] += pSub[i*nSubC + j];
		}
	}
	return 1;
}
int CBasicFunction::UpdateAdvNormalMatrix(double * pSub, int nSubR, int nSubC, int nBegR, int nBegC, double * pData, int nR, int nC)
{
	int i, j;
	if (pSub == NULL || pData == NULL) return 0;
	if (nSubC <= 0 || nSubR <= 0 || nC <= 0 || nR <= 0) return 0;
	if (nSubC > nC || nSubR > nR) return 0;
	if (nBegC + nSubC > nC || nBegR + nSubR > nR) return 0;

	for (i = 0; i < nSubR; i++){
		for (j = 0; j < nSubC; j++){
			if ((nBegR + i)*nC + nBegC + j >= nR*nC){
				printf("Error!Sub matrix out of size!\n");
				return 0;
			}
			pData[(nBegR + i)*nC + nBegC + j] -= pSub[i*nSubC + j];
		}
	}
	return 1;
}