// CXMatrix.h: interface for the CXMatrix class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MATRIX_H__BF29AB98_8F8D_4BBC_8171_4ADD41EE27F9__INCLUDED_)
#define AFX_MATRIX_H__BF29AB98_8F8D_4BBC_8171_4ADD41EE27F9__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include <stdio.h>
#include <stdlib.h>


class CXMatrix
{
public:
	CXMatrix();

	CXMatrix(double* pdData, int iRow, int iColumn);
	
  	virtual ~CXMatrix();  

	int Free();	

	CXMatrix(const CXMatrix& maCopy);	

	int Initate();

	int InitMatrix(double* pdData, int iRow, int iColumn);
	int InitMatrix(float* pdData, int iRow, int iColumn);

	int InitZeroMatrix(int iRow, int iColumn);

	int InitUnitMatrix(int iRow, int iColumn);

	int Transpose();

	CXMatrix TransposeMatrix();

	int TransposeMatrix(CXMatrix& maTran);

	int Inverse(double* pData, int n);

	int Inverse_Adv(double* pData, int n);

	CXMatrix InverseMatrix();
	
	int InverseMatrix(CXMatrix& maInver);

	int IsValid();	

	int InsertOneColumn(int nIndex, double* pInsertNum, int iNum);

	int InsertOneRow(int nIndex, double* pInsertNum, int iNum);

	int InsertOneColumnWithSameNumber(int nIndex, double dInsertNum);

	int InsertOneRowWithSameNumber(int nIndex, double dInsertNum);

	int DeleteOneColumn(int nIndex);

	int DeleteOneRow(int nIndex);

	int ComputeMatrixCrltCfct(CXMatrix& maRlt, double& dCorrCoeff);

	int ComputeCrltCfct(double* ptr1, double* ptr2, int m, int n, double& dCorrCoeff);

	int ComputeVectorMold(double* pData, int n, double& dMod);
	int ComputeVectorMold(float* pData, int n, double& dMod);
	int ComputeVectorMold(double& dMod);

	double ComputeVectorMold();

	int Convert2DiagonalMatrix();

	int SetZero();

	int GaussMatrix(double *pData,int m,int n);

	int AnsLineEquation(double *pData,double *pL,int n,double *pX);

	double GetMaxFabsElement();
	double GetMaxFabsElement( double *pData, int iRow, int iCol );

	int OutPutMatrix(char strPath[255]);

	int OutPutMatrix(FILE* fp);

	double GetElement(int iRow, int iLine);
	void CreateData(double *a,int m,int n);
	int AddElement( int iRow, int iLine, double x );
	int AddElement( int nIndex, int iRow, int iLine, CXMatrix x_ma );
	int SetElement( int iRow, int iLine, double x );
	int SetElement( int iRow, int iLine, CXMatrix x_ma );
	int SetElement( int iRow, int iLine, double *pData, int nRow, int nLine );

	void DisplayMatrix();
	bool ChangeLine(int m, int n);
	bool ChangeRow(int m,int n);
	bool InsertLine(int *m,double *a,int n,int t);
	bool InsertRow(int *m,double *a,int n,int t);

	int GetRow();
	int GetColumn();
	double* GetData();
    double GetMatrixValue();
	CXMatrix GetConvertMatrix();

	int InversMatrix(double* pm1, int i4n)const;

    CXMatrix operator * (const CXMatrix& maMul);
	CXMatrix operator * (const double& dMulCoef);
	CXMatrix operator - (const CXMatrix& maSub);
	CXMatrix operator + (const CXMatrix& maAdd);
    CXMatrix operator = (const CXMatrix& maEqua);


private:
	int m_iRow;
	int m_iColumn;
	double *m_pData;
};

#endif // !defined(AFX_MATRIX_H__BF29AB98_8F8D_4BBC_8171_4ADD41EE27F9__INCLUDED_)
