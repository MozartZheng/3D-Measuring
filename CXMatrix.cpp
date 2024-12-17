// Matrix.cpp: implementation of the CXMatrix class.
//
//////////////////////////////////////////////////////////////////////

#include <iostream>
#include "CXMatrix.h"
#include "math.h"
#include "string.h"
//////////////////////////////////////////////////////////////////////
// Construction/Destruction                                         //
//////////////////////////////////////////////////////////////////////
#ifdef WIN32
#else
#endif
#define MinError 1e-20
#define EPSION 1e-25


CXMatrix::CXMatrix()
{
	Initate();
}

CXMatrix::~CXMatrix()
{
	Free();
}

int CXMatrix::Free()
{
	if(NULL != m_pData)
	{
	   delete []m_pData;
	   m_pData = NULL;
	}
	m_iColumn = 0;
	m_iRow =0;
	return 1;
}

CXMatrix::CXMatrix(double* pdData, int iRow, int iColumn)
{
	Initate();
	InitMatrix(pdData, iRow, iColumn);
}

int CXMatrix::Initate()
{
	m_iColumn = 0;
	m_iRow = 0;
	m_pData = NULL;
	return 1;
}

int CXMatrix::InitMatrix(double* pdData, int iRow, int iColumn)
{
	if(iRow <= 0 || iColumn <= 0 || pdData == NULL) 
		return -1;
	
	if(m_pData != NULL)
	{
		delete []m_pData;
		m_pData = NULL;
	}

	m_iRow = iRow;
	m_iColumn = iColumn;
	m_pData= new double[m_iRow*m_iColumn];
	memcpy(m_pData, pdData, sizeof(double)*m_iColumn*m_iRow);

	return 1;
}
int CXMatrix::InitMatrix(float* pdData, int iRow, int iColumn)
{
	if (iRow <= 0 || iColumn <= 0 || pdData == NULL)
		return -1;

	if (m_pData != NULL)
	{
		delete[]m_pData;
		m_pData = NULL;
	}

	m_iRow = iRow;
	m_iColumn = iColumn;
	m_pData = new double[m_iRow*m_iColumn];
	for (int i = 0; i < m_iRow*m_iColumn; i++){
		m_pData[i] = pdData[i];
	}
//	memcpy(m_pData, pdData, sizeof(double)*m_iColumn*m_iRow);

	return 1;
}

int CXMatrix::InitUnitMatrix(int iRow, int iColumn)
{
	if(iRow <= 0 || iColumn <= 0) 
		return -1;

	InitZeroMatrix(iRow, iColumn);
	int minRC = (iRow > iColumn) ? iColumn : iRow;
	int i;

	for (i=0; i<minRC; i++)
	{
		m_pData[i*iColumn + i] = 1;
	}
	
	return 1;
}

int CXMatrix::InitZeroMatrix(int iRow, int iColumn)
{
	if(iRow <= 0 || iColumn <= 0) 
		return -1;
	
	if(m_pData != NULL)
	{
		delete []m_pData;
		m_pData = NULL;
	}

	m_iRow = iRow;
	m_iColumn = iColumn;
	m_pData= new double[m_iRow*iColumn];
	memset(m_pData, 0, sizeof(double)*m_iColumn*m_iRow);
	return 1;
}

int CXMatrix::SetZero()
{
	if(m_pData != NULL)
	{
		memset(m_pData, 0, sizeof(double)*m_iColumn*m_iRow);
	}
	return 1;

}


bool CXMatrix::ChangeRow(int m, int n)
{

   return true;
}

bool CXMatrix::ChangeLine(int m, int n)
{

   return true;
}


void CXMatrix::DisplayMatrix()
{
}


int CXMatrix::Transpose()
{
    if(m_pData == NULL)
		return 0;

	double* pTemp = new double[m_iColumn*m_iRow];
	memcpy(pTemp, m_pData, sizeof(double)*m_iRow*m_iColumn);

	for(int i=0; i<m_iRow; i++)
		for(int j=0; j<m_iColumn; j++)
			m_pData[j*m_iRow+i] = pTemp[i*m_iColumn+j];

	int iTemp = 0;
	iTemp = m_iRow;
	m_iRow = m_iColumn;	
	m_iColumn = iTemp;
	delete []pTemp;

	return 1;

}

CXMatrix CXMatrix::TransposeMatrix()
{
	CXMatrix maTran = *this;
	maTran.Transpose();
	return maTran;
}

int CXMatrix::TransposeMatrix(CXMatrix& maTran)
{
	maTran = *this;
	maTran.Transpose();
	return 1;
}


bool CXMatrix::InsertRow(int *m,double *a,int n,int t)
{

	return true;
}

bool CXMatrix::InsertLine(int *m,double *a,int n,int t)
{

	return true;
}

CXMatrix CXMatrix::operator*(const CXMatrix& maMul)
{
	CXMatrix maResult;

	if(this->m_iColumn != maMul.m_iRow)
		return maResult;

	double dTemp = 0;
	maResult.InitZeroMatrix(this->m_iRow, maMul.m_iColumn);
	int iColumn = maResult.m_iColumn;

	for(int i=0; i<this->m_iRow; i++)
	   for(int j=0; j<maMul.m_iColumn; j++)
	   {
		   for(int k=0; k<this->m_iColumn; k++)
			   dTemp += this->m_pData[i*m_iColumn+k] * maMul.m_pData[k*iColumn+j];

		   maResult.m_pData[i*iColumn+j] = dTemp;
		   dTemp = 0;
	   }
	return maResult;
	
}

CXMatrix CXMatrix::operator*(const double& dMulCoef)
{
	CXMatrix maResult;

	if(NULL == m_pData)
		return maResult;

	maResult.InitZeroMatrix(m_iRow, m_iColumn);
	int iColumn = maResult.m_iColumn;

	for(int i=0; i<m_iRow; i++)
	   for(int j=0; j<m_iColumn; j++)
	   {
		   maResult.m_pData[i*iColumn+j] = m_pData[i*iColumn+j]*dMulCoef;
	   }
	return maResult;
}


int CXMatrix::GetRow()
{
	return m_iRow;
}

int CXMatrix::GetColumn()
{
	return m_iColumn;
}

double* CXMatrix::GetData()
{
	return m_pData;
}

double CXMatrix::GetMatrixValue()
{
 
	return 1;
}


CXMatrix CXMatrix::GetConvertMatrix()
{
	CXMatrix ConMa;
    if(m_iColumn != m_iRow)
	    return  ConMa;

    int y=1,i,j,k,t=0;
    double RLtemp = NULL,f,x=1,*m_DataReplace = NULL;
//////////////////////////////////////////////////////////////////////////
	ConMa.InitZeroMatrix(m_iRow, m_iColumn);
    for(i=0; i<m_iRow; i++) 
		for(j=0; j<m_iColumn; j++)
			ConMa.m_pData[i*m_iColumn+j] = 0;
	for(i=0; i<m_iRow; i++)
		ConMa.m_pData[i*m_iColumn+i] = 1;

	m_DataReplace= new double [m_iColumn*m_iRow];
	memcpy(m_DataReplace, m_pData, sizeof(double)*m_iColumn*m_iRow);


	int* piLineChange = new int[m_iRow];
	int* piRowChange = new int[m_iColumn];

    for(k=0;k<m_iRow;k++)
	{
		//////////////////////////////////////////////////////////////////////////

		int imaxRow = k;
		int imaxLine = k;
		double dmaxNum = fabs(m_DataReplace[k*m_iColumn+k]);
		for(t=k; t<m_iRow; t++)
			for(int t1=k; t1<m_iColumn; t1++)
				if(fabs(m_DataReplace[t*m_iColumn+t1]) > dmaxNum)
				{
					imaxRow = t;
					imaxLine = t1;
					dmaxNum = fabs(m_DataReplace[t*m_iColumn+t1]);
				}
		piLineChange[k] = imaxLine;
		piRowChange[k] = imaxRow;
		if(dmaxNum < MinError) 
			return ConMa;

		if(imaxRow != k)
			for(int i1=0; i1<m_iRow; i1++)
			{			  
				RLtemp = m_DataReplace[imaxRow*m_iColumn+i1];
				m_DataReplace[imaxRow*m_iColumn+i1] = m_DataReplace[k*m_iColumn+i1];
				m_DataReplace[k*m_iColumn+i1] = RLtemp;	
				RLtemp = ConMa.m_pData[imaxRow*m_iColumn+i1];
				ConMa.m_pData[imaxRow*m_iColumn+i1] = ConMa.m_pData[k*m_iColumn+i1];
				ConMa.m_pData[k*m_iColumn+i1] = RLtemp;
			 }
		if(imaxLine != k)
			for(int i1=0; i1<m_iRow; i1++)
			{			  
				RLtemp = m_DataReplace[i1*m_iColumn+imaxLine];
				m_DataReplace[i1*m_iColumn+imaxLine] = m_DataReplace[i1*m_iColumn+k];
				m_DataReplace[i1*m_iColumn+k] = RLtemp;	
				RLtemp = ConMa.m_pData[i1*m_iColumn+imaxLine];
				ConMa.m_pData[i1*m_iColumn+imaxLine] = ConMa.m_pData[i1*m_iColumn+k];
				ConMa.m_pData[i1*m_iColumn+k] = RLtemp;
			 }


		double f1 = m_DataReplace[k*m_iColumn+k];
		m_DataReplace[k*m_iColumn+k] = 1.0/m_DataReplace[k*m_iColumn+k];
		for(int i2=0; i2<m_iRow; i2++)
		{
			if(i2 != k)
			{
			
				m_DataReplace[k*m_iColumn+i2] = m_DataReplace[k*m_iColumn+i2]/f1;
				ConMa.m_pData[k*m_iColumn+i2] = ConMa.m_pData[k*m_iColumn+i2]/f1;
			}
		}

	    for(j=0;j<m_iRow;j++)
		{
		   if(j != k)
		   {			   
			   f = m_DataReplace[j*m_iColumn+k];
			   if(f>-MinError && f<MinError) break;
			   for(i=0; i<m_iRow; i++) 
			   {
				   if(i != k)
				   {				   
						m_DataReplace[j*m_iColumn+i] = m_DataReplace[j*m_iColumn+i] -
											        f*m_DataReplace[k*m_iColumn+i]; 
						ConMa.m_pData[j*m_iColumn+i] = ConMa.m_pData[j*m_iColumn+i] - 
											       f*ConMa.m_pData[k*m_iColumn+i];
					}
			   }
		   }
		}
		for(int j1=0; j1<m_iColumn; j1++)
		{
			if(j1 != k)
				m_DataReplace[j1*m_iColumn+k] *= -m_DataReplace[k*m_iColumn+k];
		}
	}

	int imaxRow= 0;

	for(i=m_iColumn-1; i>=0; i--)
	{		

		if(piLineChange[i] != i)
		{
			imaxRow = piLineChange[i];
			for(int i1=0; i1<m_iRow; i1++)
			{			  
				RLtemp = m_DataReplace[imaxRow*m_iColumn+i1];
				m_DataReplace[imaxRow*m_iColumn+i1] = m_DataReplace[i*m_iColumn+i1];
				m_DataReplace[i*m_iColumn+i1] = RLtemp;
			 }
		} 		
		if(piRowChange[i] != i)
		{
			imaxRow = piRowChange[i];
			for(int i1=0; i1<m_iRow; i1++)
			{			  
				RLtemp = m_DataReplace[i1*m_iColumn+imaxRow];
				m_DataReplace[i1*m_iColumn+imaxRow] = m_DataReplace[i1*m_iColumn+i];
				m_DataReplace[i1*m_iColumn+i] = RLtemp;
			 }
		}
  	}
	delete []piLineChange;
	delete []piRowChange;
	
	delete []ConMa.m_pData;
	//m_pData = NULL;
	ConMa.m_pData = m_DataReplace;
    return ConMa;

}

int CXMatrix::Inverse(double* pData, int n)
{
	
	if ((pData == NULL) || (n <= 0))
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

int CXMatrix::Inverse_Adv(double* pData, int n)
{
	int *is,*js;
	int i,j,k,l,u,v;
	double temp,max_v;
	is=(int *)malloc(n*sizeof(int));
	js=(int *)malloc(n*sizeof(int));
	if(is==NULL||js==NULL)
	{
		printf("out of memory!\n");
		return(-1);
	}
	for(k=0;k<n;k++){
		max_v=0.0;
		for(i=k;i<n;i++)
			for(j=k;j<n;j++)
			{
				temp=fabs(pData[i*n+j]);
				if(temp>max_v)
				{
					max_v=temp; is[k]=i; js[k]=j;
				}
			}
			if(max_v==0.0)
			{
				free(is); free(js);
				printf("invers is not availble!\n");
				return -1;
			}
			if(is[k]!=k)
				for(j=0;j<n;j++){
					u=k*n+j; v=is[k]*n+j;
					temp=pData[u]; pData[u]=pData[v]; pData[v]=temp;
				}
				if(js[k]!=k)
					for(i=0;i<n;i++){
						u=i*n+k; v=i*n+js[k];
						temp=pData[u]; pData[u]=pData[v]; pData[v]=temp;
					}
					l=k*n+k;
					pData[l]=1.0/pData[l];
					for(j=0;j<n;j++)
						if(j!=k){
							u=k*n+j;
							pData[u]*=pData[l];
						}
						for(i=0;i<n;i++)
							if(i!=k)
								for(j=0;j<n;j++)
									if(j!=k){
										u=i*n+j;
										pData[u]-=pData[i*n+k]*pData[k*n+j];
									}
									for(i=0;i<n;i++)
										if(i!=k){
											u=i*n+k;
											pData[u]*=-pData[l];
										}
	}
	for(k=n-1;k>=0;k--){
		if(js[k]!=k)
			for(j=0;j<n;j++){
				u=k*n+j; v=js[k]*n+j;
				temp=pData[u]; pData[u]=pData[v]; pData[v]=temp;
			}
			if(is[k]!=k)
				for(i=0;i<n;i++){
					u=i*n+k; v=i*n+is[k];
					temp=pData[u]; pData[u]=pData[v]; pData[v]=temp;
				}
	}
	free(is); 
	free(js);
	return 1;
}

CXMatrix CXMatrix::InverseMatrix()
{
	CXMatrix maInverse;
	InverseMatrix(maInverse);
	return maInverse;
}

int CXMatrix::Convert2DiagonalMatrix()
{
	memset(m_pData, 0, sizeof(double)*m_iColumn*m_iRow);
	
	int min = m_iRow;
	if (m_iColumn < min)
		min = m_iColumn;

	for (int i=0; i<min; i++)
		m_pData[i*m_iColumn + i] = 1;
	
	return 1;
}

int CXMatrix::InverseMatrix(CXMatrix& maInver)
{

	CXMatrix tmpInver = *this;
	if (Inverse(tmpInver.GetData(),m_iRow) == -1)
	{
		return -1;
	}
	else
	{
		maInver = tmpInver;
		return 1;
	}
}

void CXMatrix::CreateData(double *a,int m,int n)
{
	if(NULL != m_pData)
	{
		delete []m_pData;
		m_pData = NULL;		
	}

	m_iRow = m;
	m_iColumn = n;
	m_pData = new double[m_iRow*m_iColumn];
    for(int i=0;i<m_iRow;i++) 
		for(int j=0;j<m_iColumn;j++) 
			m_pData[i*m_iColumn+j]=a[n*i+j];
}

CXMatrix::CXMatrix(const CXMatrix& maCopy)
{
    m_iRow = maCopy.m_iRow;
	m_iColumn = maCopy.m_iColumn;
    m_pData = new double[m_iColumn*m_iRow];
	memcpy(m_pData, maCopy.m_pData, sizeof(double)*m_iColumn*m_iRow);

}


CXMatrix CXMatrix::operator = (const CXMatrix& maEqua)
{
	if(m_pData)
	{
		delete []m_pData;
		m_pData = NULL;
	}
    m_iRow = maEqua.m_iRow;
	m_iColumn = maEqua.m_iColumn;
	m_pData= new double[m_iRow*m_iColumn];

	for(int i=0;i<m_iRow;i++) 
		for(int j=0;j<m_iColumn;j++) 
			m_pData[i*m_iColumn+j] = maEqua.m_pData[i*m_iColumn+j];
    return *this;

}

CXMatrix CXMatrix::operator-(const CXMatrix& maSub)
{
	CXMatrix maResult;
    if(maSub.m_iColumn != m_iColumn || maSub.m_iRow != m_iRow)
		return maResult;

	maResult.InitZeroMatrix(m_iRow, m_iColumn);

	for(int i=0; i<m_iRow; i++)
		for(int j=0; j<m_iColumn; j++)
		   maResult.m_pData[i*m_iColumn+j] = m_pData[i*m_iColumn+j] - maSub.m_pData[i*m_iColumn+j];

	return maResult;
}

CXMatrix CXMatrix::operator+(const CXMatrix& maAdd)
{
	CXMatrix maResult;
    if(maAdd.m_iColumn != m_iColumn || maAdd.m_iRow != m_iRow)
		return maResult;

	maResult.InitZeroMatrix(m_iRow, m_iColumn);

	for(int i=0; i<m_iRow; i++)
		for(int j=0; j<m_iColumn; j++)
		   maResult.m_pData[i*m_iColumn+j] = m_pData[i*m_iColumn+j] + maAdd.m_pData[i*m_iColumn+j];

	return maResult;
}
int CXMatrix::SetElement( int iRow, int iLine, double x )
{
	int iShift = iRow*m_iColumn + iLine;
	if( iShift > m_iRow * m_iColumn )	return 0;

	else m_pData[iRow*m_iColumn + iLine] = x;

	return 1;
}
int CXMatrix::AddElement( int iRow, int iLine, double x )
{
	int iShift = iRow*m_iColumn + iLine;
	if( iShift > m_iRow * m_iColumn )	return 0;
	else m_pData[iShift] += x;
	return 1;
}
int CXMatrix::AddElement( int nIndex, int iRow, int iLine, CXMatrix x_ma )
{
	int i,j;
	int nRow = x_ma.GetRow();
	int nLine = x_ma.GetColumn();
	double * pData = x_ma.GetData();
	for( i = 0;i < nRow; i ++ ){
		for( j = 0;j < nLine;j ++ ){
			if(nIndex){
				if( !AddElement( iRow+i, iLine+j, pData[i*nLine + j] ) ) return 0;
			}
			else{
				if( !AddElement( iRow+i, iLine+j, -pData[i*nLine + j] ) ) return 0;
			}

		}
	}
	return 1;
}
int CXMatrix::SetElement( int iRow, int iLine, CXMatrix x_ma )
{
	int nRow = x_ma.GetRow();
	int nLine = x_ma.GetColumn();
	double * pData = x_ma.GetData();

	if( pData == NULL ) return 0;
	if( !SetElement( iRow, iLine, pData, nRow, nLine ) ) return 0;
	else return 1;
}
int CXMatrix::SetElement( int iRow, int iLine, double *pData, int nRow, int nLine )
{
	int i = 0;
	int j = 0;
	for( i = 0;i < nRow;i ++ ){
		for( j = 0;j < nLine; j ++ ){
			if( !SetElement( iRow+i, iLine+j, pData[i*nLine+j] ) )
				return 0;
		}
	}
	return 1;
}

double CXMatrix::GetElement(int iRow, int iLine)
{
	int iShift = iRow*m_iColumn+iLine;
	if( iShift > m_iRow * m_iColumn ) return 0;
	else return m_pData[iRow*m_iColumn+iLine];
}


int CXMatrix::InversMatrix(double* pm1, int i4n)const
{ 
	int *pis,*pjs;
	int i,j,k,l,u,v;
	double temp,max_v;
	pis = new int[i4n];
	pjs = new int[i4n];
	if(NULL==pis || NULL==pjs)	return(0);

	for(k=0; k<i4n; k++)
	{
		max_v = 0.0;
		for(i=k; i<i4n; i++)
			for(j=k; j<i4n; j++)
			{
				temp = fabs(pm1[i*i4n+j]);
				if( temp>max_v )
				{
			        max_v = temp; 
					pis[k] = i; 
					pjs[k] = j;
				}
			}
		if(max_v==0.0)
		{
			delete []pis; 
			delete []pjs;
			return(0);
		}
		if(pis[k]!=k)
			for(j=0; j<i4n; j++)
			{
			   u = k*i4n+j;
			   v = pis[k]*i4n+j;
			   temp = pm1[u]; 
			   pm1[u] = pm1[v];
			   pm1[v] = temp;
			}
		if(pjs[k]!=k)
			for(i=0; i<i4n; i++)
			{
				u = i*i4n+k; v = i*i4n+pjs[k];
				temp=pm1[u]; pm1[u]=pm1[v]; pm1[v]=temp;
			}
		l=k*i4n+k;
		pm1[l]=1.0/pm1[l];
		for(j=0; j<i4n; j++)
			if(j!=k)
			{
				u = k*i4n+j;
				pm1[u] *= pm1[l];
			}
		for(i=0; i<i4n; i++)
			if(i!=k)
				for(j=0; j<i4n; j++)
					if(j!=k)
					{
					  u = i*i4n+j;
					  pm1[u] -= pm1[i*i4n+k] * pm1[k*i4n+j];
					}
		for(i=0; i<i4n; i++)
			if(i != k)
			{
				u = i*i4n+k;
				pm1[u] *= -pm1[l];
			}
	}
	for(k=i4n-1; k>=0; k--)
	{
		if(pjs[k]!=k)
			for(j=0; j<i4n; j++)
			{
				u = k*i4n+j; v = pjs[k]*i4n+j;
				temp=pm1[u]; pm1[u]=pm1[v]; pm1[v]=temp;
			}
		if(pis[k] != k)
			for(i=0; i<i4n; i++)
			{
				u=i*i4n+k; v=i*i4n+pis[k];
				temp=pm1[u]; pm1[u]=pm1[v]; pm1[v]=temp;
			}
	}
	delete []pis; delete []pjs;

  return 1;

}

int CXMatrix::OutPutMatrix(char strPath[255])
{

	FILE* fp = NULL;
	fopen_s(&fp, strPath, "w");
	if (fp == NULL){
		return 0;
	}

	for (int i=0; i<m_iRow; i++)
	{
		/*
		if (m_pData[i*m_iColumn + 0] == 0){
			continue;
		}
		*/
		for (int j = 0; j<m_iColumn; j++)
		{
			fprintf(fp,"%20.10e\t",m_pData[i*m_iColumn+j]);
		//	fprintf(fp,"%12.3lf\t",m_pData[i*m_iColumn+j]);
			/*
			if(m_pData[i*m_iColumn+j] != 0)
			{
			fprintf(fp, "%d ", 1);
			}
			else
			fprintf(fp,"%d ", 0);
			*/
			
		}
		fprintf(fp, "\n");
	}
	fclose(fp);

	return 1;
}

int CXMatrix::OutPutMatrix(FILE* fp)
{
	for (int i=0; i<m_iRow; i++)
	{
		for (int j = 0; j<m_iColumn; j++)
		{
			fprintf(fp, "%20.10e\t", m_pData[i*m_iColumn + j]);
		//	fprintf(fp, "%-30.9lf", m_pData[i*m_iColumn+j]);
		}
		fprintf(fp, "\n");
	}
	return 1;
}

int CXMatrix::ComputeVectorMold(double* pData, int n, double& dMod)
{
	if (pData == NULL)
	{
		return -1;
	}
	double* ptr = pData;
	dMod = 0;

	for(int i=0; i<n; i++)
	{
		dMod += (*ptr) * (*ptr);
		ptr ++;
	}
	
	dMod = sqrt(dMod);
	return 1;
}
int CXMatrix::ComputeVectorMold(float* pData, int n, double& dMod)
{
	if (pData == NULL)
	{
		return -1;
	}
	float* ptr = pData;
	dMod = 0;

	for (int i = 0; i<n; i++)
	{
		dMod += (*ptr) * (*ptr);
		ptr++;
	}

	dMod = sqrt(dMod);
	return 1;
}
int CXMatrix::ComputeVectorMold(double& dMod)
{
	if ((this->m_iColumn != 1) && (this->m_iRow != 1))
	{
		return -1;
	}

	int n = this->m_iRow;
	if (n == 1)
	{
		n = this->m_iColumn;
	}

	ComputeVectorMold(this->m_pData, n, dMod);

	return 1;
}

double CXMatrix::ComputeVectorMold()
{
	if ((this->m_iColumn != 1) && (this->m_iRow != 1))
	{
		return -1;
	}
	
	int n = this->m_iRow;
	if (n == 1)
	{
		n = this->m_iColumn;
	}

	double dMod;
	ComputeVectorMold(this->m_pData, n, dMod);
	return dMod;
}

int CXMatrix::IsValid()
{
	if ((m_pData == NULL) || (m_iColumn <= 0) || (m_iRow <=0 ))
		return -1;

	return 1;
}

int CXMatrix::InsertOneColumn(int nIndex, double* pInsertNum, int iNum)
{
	if ((nIndex < 0) || (pInsertNum == NULL) || (nIndex > m_iColumn) || (m_iColumn != 0 && iNum != m_iRow))
		return -1;

	int iNewRow = iNum;
	int iNewCol = 0;
	if (m_pData == NULL)
		iNewCol = 1;
	else
		iNewCol = m_iColumn + 1;

	double* pNewData = new double[iNewRow*iNewCol];
	if (nIndex != 0)
	{
		for (int i=0; i<nIndex; i++)
		{
			for (int j=0; j<iNewRow; j++)
			{
				pNewData[j*iNewCol+i] = m_pData[j*m_iColumn+i];
			}
		}
	}

	for (int j=0; j<iNewRow; j++)
	{
		pNewData[j*iNewCol+nIndex] = pInsertNum[j];
	}

	if (nIndex != m_iColumn)
	{
		for (int i=nIndex+1; i<iNewCol; i++)
		{
			for (int j=0; j<iNewRow; j++)
			{
				pNewData[j*iNewCol+i] = m_pData[j*m_iColumn+(i-1)];
			}
		}
	}

	InitMatrix(pNewData, iNewRow, iNewCol);

	delete[] pNewData;
	return 1;
}

int CXMatrix::InsertOneRow(int nIndex, double* pInsertNum, int iNum)
{
	if ((nIndex < 0) || (pInsertNum == NULL) || (nIndex > m_iRow) || (m_iRow != 0 && iNum != m_iColumn))
		return -1;

	int iNewRow = 0;
	int iNewCol = iNum;
	if (m_pData == NULL)
		iNewRow = 1;
	else
		iNewRow = m_iRow + 1;
	
	double* pNewData = new double[iNewRow*iNewCol];
	if (nIndex != 0)
		memcpy(pNewData, m_pData, sizeof(double)*nIndex*iNewCol);

	memcpy(pNewData+nIndex*iNewCol, pInsertNum, sizeof(double)*iNewCol);

	if (nIndex != m_iRow)
		memcpy(pNewData+(nIndex+1)*iNewCol, m_pData+nIndex*iNewCol, 
				sizeof(double)*iNewCol*(m_iRow-nIndex));

	InitMatrix(pNewData, iNewRow, iNewCol);
	delete[] pNewData;
	return 1;
}
int CXMatrix::InsertOneColumnWithSameNumber(int nIndex, double dInsertNum)
{
	int iNum = m_iRow;
	double* pInsertNum = new double[iNum];
	for (int i=0; i<iNum; i++)
		pInsertNum[i] = dInsertNum;

	int iRes = 0;
	iRes = InsertOneColumn(nIndex, pInsertNum, iNum);
	delete[] pInsertNum;
	return iRes;
}

int CXMatrix::InsertOneRowWithSameNumber(int nIndex, double dInsertNum)
{
	int iNum = m_iColumn;
	double* pInsertNum = new double[iNum];
	for (int i=0; i<iNum; i++)
		pInsertNum[i] = dInsertNum;

	int iRes = 0;
	iRes = InsertOneRow(nIndex, pInsertNum, iNum);	
	delete[] pInsertNum;
	return 1;
}

int CXMatrix::DeleteOneColumn(int nIndex)
{
	if ((nIndex < 0) || (nIndex > m_iColumn-1) || (m_pData == NULL))
		return -1;

	int iNewRow = m_iRow;
	int iNewCol = m_iColumn - 1;
	
	if (iNewCol == 0)
	{
		Free();
		return 1;
	}

	double* pNewData = new double[iNewRow*iNewCol];

	if (nIndex != 0)
	{
		for (int i=0; i<nIndex; i++)
		{
			for (int j=0; j<iNewRow; j++)
			{
				pNewData[j*iNewCol+i] = m_pData[j*m_iColumn+i];
			}
		}
	}

	if (nIndex != m_iColumn-1)
	{
		for (int i=nIndex; i<iNewCol; i++)
		{
			for (int j=0; j<iNewRow; j++)
			{
				pNewData[j*iNewCol+i] = m_pData[j*m_iColumn+(i+1)];
			}
		}
	}

	InitMatrix(pNewData, iNewRow, iNewCol);
	delete[] pNewData;
	return 1;
}

int CXMatrix::DeleteOneRow(int nIndex)
{
	if ((nIndex < 0) || (nIndex > m_iRow-1) || (m_pData == NULL))
		return -1;

	int iNewRow = m_iRow - 1;
	int iNewCol = m_iColumn;
	
	if (iNewRow == 0)
	{
		Free();
		return 1;
	}

	double* pNewData = new double[iNewRow*iNewCol];
	if (nIndex != 0)
		memcpy(pNewData, m_pData, sizeof(double)*nIndex*iNewCol);

	if (nIndex != m_iRow-1)
		memcpy(pNewData+nIndex*iNewCol, m_pData+(nIndex+1)*iNewCol, 
				sizeof(double)*iNewCol*(iNewRow-nIndex));

	InitMatrix(pNewData, iNewRow, iNewCol);
	delete[] pNewData;
	return 1;
}

int CXMatrix::ComputeMatrixCrltCfct(CXMatrix& maRlt, double& dCorrCoeff)
{
	if ((this->m_iColumn != maRlt.m_iColumn) || (this->m_iRow != maRlt.m_iRow))
	{
		if (!( ((this->m_iRow == 1 && maRlt.m_iColumn == 1) && (this->m_iColumn == maRlt.m_iRow)) || 
		     ((this->m_iColumn == 1 && maRlt.m_iRow == 1) && (this->m_iRow == maRlt.m_iColumn)) ))
			return -1;	
	}

	ComputeCrltCfct(this->GetData(), maRlt.GetData(), this->m_iRow, this->m_iColumn, dCorrCoeff);
	
	return 1;
}

int CXMatrix::ComputeCrltCfct(double* ptr1, double* ptr2, int m, int n, double& dCorrCoeff)
{
	if ((ptr1 == NULL) || (ptr2 == NULL))
	{
		return -1;
	}

	double sp1p1, sp1p2, sp2p2, sp1, sp2;
	sp1p1 = sp1p2 = sp2p2 = sp1 = sp2 = 0;
	int i;
	double* tp1 = ptr1;
	double* tp2 = ptr2;

	long mn = m * n;
	for (i=0; i<mn; i++)
	{
		sp1 += *tp1;
		sp2 += *tp2;
		sp1p1 += (*tp1) * (*tp1);
		sp2p2 += (*tp2) * (*tp2);
		sp1p2 += (*tp1) * (*tp2);
		tp1++;
		tp2++;
	}

	dCorrCoeff = (sp1p2 - sp1*sp2/mn)/sqrt((sp1p1-sp1*sp1/mn) * (sp2p2-sp2*sp2/mn));
	return 1;
}

int CXMatrix::GaussMatrix(double *pData,int m,int n)
{
	if(pData != NULL)
	{
		int i,j,k;
		int temp = m;
		double tempk = 0;
		if(n <= m)
			temp = n;
		for(k = 1;k < temp;k ++)
		{
			for(i = k;i < m;i++)
			{
				if(pData[(i-1)*n+i-1] != 0)
				{	
					tempk = pData[i*n+k-1]/pData[(k-1)*n+k-1];
					for(j = 0;j < n;j++)
					{
						pData[i*n+j] = pData[i*n+j] - pData[(k-1)*n+j]*tempk;
					}
				}
				else
				{
					return 0;
				}
			}
		}

		return 1;
		
	}
	else
		return 0;
}
int CXMatrix::AnsLineEquation(double *pData,double *pL,int n,double *pX)
{
	int i,j;
	double *pA = NULL;
	double temp = 0;
	pA = new double[n*(n+1)];
	memset(pX,0,sizeof(double)*n);
	memset(pA,0,sizeof(double)*n*(n+1));
	for(i = 0;i < n;i ++)
	{
		for(j = 0;j < n;j ++)
		{
			pA[i*(n+1)+j] = pData[i*n+j];
		}
		pA[i*(n+1)+n] = pL[i];
	}
	
	GaussMatrix(pA,n,n+1);

	for(i = 0;i < n;i ++)
	{
		for(j = 0;j < n;j ++)
		{
			pData[i*n+j] = pA[i*(n+1)+j];
		}
		 pL[i] =pA[i*(n+1)+n];
	}
	
	pX[n-1] = pL[n-1]/pData[(n-1)*n+n-1];
	for(i = 1;i < n;i ++)
	{
		temp = 0;
		for(j = 0;j < i;j ++)
		{
			temp += pData[(n-i-1)*n+n-j-1]*pX[n-j-1];
		}
		pX[n-i-1] = (pL[n-i-1]-temp)/pData[(n-i-1)*n+n-i-1];		
	}
	return 1;
	delete [] pA;
}
double CXMatrix::GetMaxFabsElement( double *pData, int iRow, int iCol )
{
	int i,j;
	double TempValue = 0;
	double MaxValue = 0;
	for(i = 0;i < iRow;i ++)
	{
		for(j = 0;j < iCol;j ++)
		{
			TempValue = pData[i*iCol + j];
			if( fabs(TempValue) > fabs(MaxValue))
				MaxValue = TempValue;
		}
	}
	return MaxValue;
}
double CXMatrix::GetMaxFabsElement()
{
	return GetMaxFabsElement( m_pData, m_iRow, m_iColumn );
}
