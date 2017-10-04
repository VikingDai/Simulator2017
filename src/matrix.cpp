/* 
	Racetrack
	Automatic Control LiTH

	Filename: matrix.cpp
	Description:
		This file contains the implementation of functions and operators used by matrix
		objects. More details on the LAPACK functions can be found in their 
		documentation. 

	Version:	Date:		Changes:							
		1.0		2011-07-12	First version


	Author/s:
	Isak Nielsen
*/

/* Includes */
#include "matrix.h"

/*
#include <C:/Temp/OSARR_2013/Racetrack files/2011/Kod/CLAPACK-3.2.1/INCLUDE/f2c.h>
#include <C:/Temp/OSARR_2013/Racetrack files/2011/Kod/CLAPACK-3.2.1/INCLUDE/clapack.h>
*/

/*Functions used from lapack*/

extern "C" void sgemm_(char* transa,char* transb,int* M,int* N,int* K,float* alpha, 
			float* A,int* LDA,float* B,int* LDB,float* beta,float* C,int* LDC,int* info);

		
extern "C" void sgesvd_(char* calU,char* calV,int* rowS,int* colS,float* tmpArray,int* rowS2,              
			float* singValues,float* U,float* ldu,float* V,float* ldu2,
			float* workSpace,int* lwork,int* info);

extern "C" void sgetrf_(int* rowS,int* colS,float* tmpMtrx,int* rowS2,
			int* pivotArray,int* info);

extern "C" void sgetri_(int* rowS,float* tmpMtrx,int* rowS2,int* pivotArray,              
			float* workSpace,int* maxS,int* info);

extern "C" void sgeev_(char* cEigVec,char* cEigVec2,int* N,float* tmpArray,int* LDA,float* eigR,float* eigI,          
		float* eigVV,int* ldv,float* eigVR,int* ldv2,float* workSpace,int* lwork,int* info);



using namespace std;

/* Constructors */
/* Construct a matrix with single precision, zeroes as	default elements */
matrix::matrix(int rowS, int colS)
{
	
	/* Save information of sizes of matrix */
	rowSize = int(rowS);
	colSize = int(colS);

	if( rowS == 0 && colS == 0)
	{
		return;
	}
	

	/* Allocate memory for the array containing the values of
		the matrix */
	matrixArray = new float[rowS*colS];

	/* Set all values to zero */
	memset(matrixArray,0,rowS*colS*sizeof(float));
}

/* Copies the matrix m and creates a new object */
matrix::matrix(const matrix& m)
{
	rowSize = m.rowSize;
	colSize = m.colSize;
	
	/* Allocate memory */
	matrixArray = new float[rowSize*colSize];

	/* Copy array */
	memcpy(matrixArray,m.matrixArray,rowSize*colSize*sizeof(float));

	/* Copy error */
	rtError = m.rtError;
}

/* Destructor */
matrix::~matrix()
{
	if(rowSize != 0)
	{
		/* Deallocate memory */
		delete matrixArray;
	}
}

/* Operators */
matrix& matrix::operator=(const matrix& m)
{
	/* Check if the matrix is not initialized */
	if(colSize == 0)
	{
		/* Copy rowSize and colSize */
		rowSize = m.rowSize;
		colSize = m.colSize;
		
		/* Allocate memory */
		matrixArray = new float[rowSize*colSize];

		/* Copy elements of matrixArray */
		memcpy(matrixArray,m.matrixArray,rowSize*colSize*sizeof(float));
		rtError = m.rtError;
	}
	/* Check if the sizes are the same */
	else if((rowSize != m.rowSize) && (colSize != m.colSize))
	{
		/* Sizes not consistent */
		rtError.SetDescription("Sizes not consistent",
			"matrix::operator=");
	}
	else if(rowSize != m.rowSize)
	{
		/* Row size not consistent */
		rtError.SetDescription("Row sizes not consistent",
			"matrix::operator=");
	}
	else if(colSize != m.colSize)
	{
		/* Column size not consistent */
		rtError.SetDescription("Column sizes not consistent",
			"matrix::operator=");
	}
	else
	{
		/* Copy elements of matrixArray */
		memcpy(matrixArray,m.matrixArray,rowSize*colSize*sizeof(float));
		rtError = m.rtError;
	}

	/* Return matrix object */
	return *this;
}

/* Set the matrix according to m = {{a11...a1n},...,{am1...amn}} */
/* THIS ASSIGNEMENT DOES NOT CHECK IF THE ARRAY IS OF CORRECT SIZE */
matrix& matrix::operator=(const float pf[])
{
	/* Set values */
	for(int rI = 0; rI < rowSize; rI++)
	{
		for(int cI = 0; cI < colSize; cI++)
		{
			matrixArray[rI+cI*rowSize]=pf[cI+rI*colSize];
		}
	}
	return *this;
}

matrix& matrix::operator-=(const matrix& m)
{
	/* Check if the sizes are the same */
	if((rowSize != m.rowSize) && (colSize != m.colSize))
	{
		/* Sizes not consistent */
		rtError.SetDescription("Sizes not consistent",
			"matrix::operator-=");
	}
	else if(rowSize != m.rowSize)
	{
		/* Row size not consistent */
		rtError.SetDescription("Row sizes not consistent",
			"matrix::operator-=");
	}
	else if(colSize != m.colSize)
	{
		/* Column size not consistent */
		rtError.SetDescription("Column sizes not consistent",
			"matrix::operator-=");
	}
	else
	{
		/* Loop thorugh every element and change them */
		int maxSize = rowSize*colSize;
		for(int inx = 0; inx < maxSize; inx++)
		{
			matrixArray[inx] -= m.matrixArray[inx];
		}

		rtError = m.rtError;
	}

	return *this;
}

matrix& matrix::operator+=(const matrix& m)
{
	/* Check if the sizes are the same */
	if((rowSize != m.rowSize) && (colSize != m.colSize))
	{
		/* Sizes not consistent */
		rtError.SetDescription("Sizes not consistent",
			"matrix::operator+=");
	}
	else if(rowSize != m.rowSize)
	{
		/* Row size not consistent */
		rtError.SetDescription("Row sizes not consistent",
			"matrix::operator+=");
	}
	else if(colSize != m.colSize)
	{
		/* Column size not consistent */
		rtError.SetDescription("Column sizes not consistent",
			"matrix::operator+=");
	}
	else
	{
		/* Loop thorugh every element and change them */
		int maxSize = rowSize*colSize;
		for(int inx = 0; inx < maxSize; inx++)
		{
			matrixArray[inx] += m.matrixArray[inx];
		}

		rtError = m.rtError;
	}

	return *this;
}

const matrix matrix::operator+(const matrix& m) const
{
	matrix tmpMtrx(*this);

	/* Add the input matrix to the tmpMtrx */
	tmpMtrx += m;

	return tmpMtrx;
}

const matrix matrix::operator-(const matrix& m) const
{
	matrix tmpMtrx(*this);

	/* Subtract the input matrix to the tmpMtrx */
	tmpMtrx -= m;

	return tmpMtrx;
}

const matrix matrix::operator*(const matrix& m) const
{
	matrix tmpMtrx(rowSize,m.colSize);

	/* Check if dimensions of matrix is correct */
	if(colSize != m.rowSize)
	{
		/* Sizes not consistent */
		tmpMtrx.rtError.SetDescription("Sizes not consistent","matrix::operator*");
	}
	else
	{
		/* Perform multiplication with nested for loops */
#if 0		
		for(unsigned int rowC = 0 ; rowC < rowSize ; rowC++)
			for(unsigned int colC = 0 ; colC < m.colSize ; colC++)
				for(unsigned int innerC = 0 ; innerC < colSize ; innerC++)
				{
					tmpMtrx.matrixArray[rowC + colC*rowSize] += matrixArray[rowC + innerC*rowSize]*m.matrixArray[m.rowSize*colC+innerC];
				}
#else
		/* Perform multiplication with Lapack functions */

		/* No transpose */
		
		/*Input data to sgemm_ */
		/*
		
		char transa = 'n';
		char transb = 'n';
		int M=int(this->rowSize);
        int N=int(m.colSize);
		int K=int(this->colSize);
		float alpha = 1;
		float* A=this->matrixArray;
		int LDA = int(M);
		float* B = m.matrixArray;
		int LDB = int(N);
		float beta = 0;
		float* C=tmpMtrx.matrixArray;
		int LDC = tmpMtrx.rowSize;
		int info;


		
		sgemm_(&transa, &transb, &M, &N, &K, &alpha, 
			this->matrixArray, &LDA, m.matrixArray, &LDB, &beta,tmpMtrx.matrixArray,&LDC,&info);
	
		
		*/
		
			char trans = 'N';
		int info=0;
		float alpha = 1;
		float beta = 0;
		int rA = this->rowSize;
		int cB = m.colSize;
		int cA = this->colSize;

		int k1=tmpMtrx.rowSize;
		int k2=m.rowSize;



	    sgemm_(&trans, &trans, &rA, &cB, &cA, &alpha,
			this->matrixArray, &rA, m.matrixArray, &k2,
			&beta,tmpMtrx.matrixArray,&k1,&info);
		


		/* Check if it was sucessfully performed */

		if(info < 0)
		{
			tmpMtrx.rtError.SetDescription("Invalid argument to sgemm_","matrix::operator*");
		}
		else if(info > 0)
		{
			tmpMtrx.rtError.SetDescription("Error performing sgemm_","matrix::operator*");
		}
#endif
	}

	return tmpMtrx;
}

/* Multiplication of matrix*float */
const matrix matrix::operator*(const float f) const   // Går det att använda en funktion istället för att loopa?
{
	matrix tmpMtrx(*this);

	int maxSize = rowSize*colSize;

	/* Loop through every element and set them to f*value */
	for(int inx = 0; inx < maxSize; inx++)
	{
		tmpMtrx.matrixArray[inx] *= f;
	}

	/* Return the matrix obejct */
	return tmpMtrx;
}

/* Returns the transpose of the matrix */
const matrix matrix::operator~() const            // Går det att använda en funktion istället för att loopa?
{
	matrix tmpMtrx(colSize,rowSize);

	/* Transpose the elements (i,j)->(j,i) */
	for(int rI = 0; rI < rowSize; rI++)
	{
		for(int cI = 0; cI < colSize; cI++)
		{
			tmpMtrx.matrixArray[cI+rI*colSize] = matrixArray[rI+cI*rowSize];
		}
	}

	return tmpMtrx;
}

/* Functions to manipulate matrix objects */
/* Set value on an element, indexed from 1->rowSize etc */
RTError matrix::Set(int row, int col, float value)    
{
	RTError rtError;

	/* Check that indexes are in the correct range */
	if((row < 1) || (row > rowSize))
	{
		rtError.SetDescription("Row index ot of range","matrix::Set");
	}
	else if ((col < 1) || (col > colSize))
	{
		rtError.SetDescription("Column index out of range",
			"matrix::Set");
	}
	else
	{
		/* Set value */
		matrixArray[(row-1)+rowSize*(col-1)] = float(value);
	}

	return rtError;
}

/* Returns the inverse of the matrix object */
const matrix matrix::Inv() const
{
	matrix tmpMtrx(*this);

	int rowS = tmpMtrx.rowSize;
	int colS = tmpMtrx.colSize;

	/* Check that this is a square matrix */
	if(rowS != colS)
	{
		tmpMtrx.rtError.SetDescription("Matrix not square","matrix::Inv()");
	}
	else
	{
		/* Create temporary values used by lapacks LU-factorization
			and invere function */
		int* pivotArray;
		float* workSpace;
		int maxS = rowS*colS;
		pivotArray = new int[maxS];
		workSpace = new float[5*maxS];
		int lwork = 5*maxS;
		int info;

		/*Check if it is close to singular (i.e. a singular value close to 0) */
		/* Do not calculate U or V*/
		char calU = 'N';
		char calV = 'N';
		float* singValues;
		float* tmpArray;
		singValues = new float[rowS];
		tmpArray = new float[maxS];
		
		/* Copy data from tmpMtrx to ensure that svd does not corrupt it */
		memcpy(tmpArray,tmpMtrx.matrixArray,maxS*sizeof(float));

		/* Those are not used */
		float U = 1;
		float V = 1;
		float ldu = 1;


		/* Perform svd */
		sgesvd_(&calU,&calV,&rowS,&colS,tmpArray,&rowS,                 //CLAPACK FUNKTION?
			singValues,&U,&ldu,&V,&ldu,workSpace,&lwork,&info);

		/* Check that it was suceesfully performed */
		if(info < 0)
		{
			tmpMtrx.rtError.SetDescription("Invalid argument to sgesvd_","matrix::Inv()");
		}
		else if(info > 0)
		{
			tmpMtrx.rtError.SetDescription("Error performing sgesvd_","matrix::Inv()");
		}

		/* Check values of the smallest singular value, if this is to small, return
			error state SINGULAR */
		if((singValues[rowS-1] > -1.0e-6) && 
			(singValues[rowS-1] < 1.0e-6))
		{
			tmpMtrx.rtError.SetDescription("Matrix singular or close to singular",
				"matrix::Inv()");
		}

		delete singValues;
		delete tmpArray;

	 

		/* Perform LU factorization */
		sgetrf_(&rowS,&colS,tmpMtrx.matrixArray,&rowS,
			pivotArray,&info);                                            //CLAPACK FUNKTION?
		
		/* Check if it was successfully performed */
		if(info < 0)
		{
			tmpMtrx.rtError.SetDescription("Invalid argument to sgetrf_","matrix::Inv()");
		}
		else if(info > 0)
		{
			tmpMtrx.rtError.SetDescription("Error performing sgetrf_","matrix::Inv()");
		}

	

		/* Perform inverse */
		sgetri_(&rowS,tmpMtrx.matrixArray,&rowS,pivotArray,                 //CLAPACK FUNKTION?
			workSpace,&maxS,&info);
		
		/* Check if it was successfully performed */
		if(info < 0)
		{
			tmpMtrx.rtError.SetDescription("Invalid argument to sgetri_","matrix::Inv()");
		}
		else if(info > 0)
		{
			tmpMtrx.rtError.SetDescription("Matrix singular","matrix::Inv()");
		}

		/* Deallocate memory */
		delete pivotArray;
		delete workSpace;
	}

	return tmpMtrx;
}

/** Returns the determinant for a positive definite matrix */
float matrix::PosDefDet()
{
	/* If the matrix is not square, return -1 (impossible determinant
		for a positive semi definite matrix) */
	if(colSize != rowSize)
		return -1;

	/* Copy the matrixArray so sgeev_ does not corrupt data */
	float* tmpArray = new float[rowSize*colSize];
	memcpy(tmpArray,matrixArray,rowSize*colSize*sizeof(float));

	/* Compute determinant as multiplication of the eigenvalues */

	/* Do not calculate eigenvectors */
	char cEigVec = 'N';
	float* eigR;
	float* eigI;

	eigR = new float[rowSize];
	eigI = new float[colSize];

	/* Those are not used */
	float eigVV = 1;
	float eigVR = 1;
	
	int ldv = 1;

	/* Allocate workspace */
	int lwork = 3*rowSize;
	float* workSpace;
	workSpace = new float[lwork];
	int info;

	/* Calculate the eigenvalues */
	int N = int(rowSize);
    int LDA = int(rowSize);

	sgeev_(&cEigVec,&cEigVec,&N,tmpArray,&LDA,eigR,eigI,            
		&eigVV,&ldv,&eigVR,&ldv,workSpace,&lwork,&info);

	/* Calculate the determinant as the product of the eigenvalues */
	float retValue = 1;
	for(int inx = 0; inx < rowSize; inx++)
	{
		retValue *= eigR[inx];
	}

	/* A positive definite matrix must have eigenvalues > 0 */
	if(retValue <= 0)
		return -1;


	/* Deallocate memory */
	delete eigR;
	delete eigI;
	delete workSpace;
	delete tmpArray;

	return retValue;
}


float* matrix::EigVal()
{
		/* If the matrix is not square, return -1 (impossible determinant
		for a positive semi definite matrix) */
	if(colSize != rowSize)
		return 0;

	/* Copy the matrixArray so sgeev_ does not corrupt data */
	float* tmpArray = new float[rowSize*colSize];
	memcpy(tmpArray,matrixArray,rowSize*colSize*sizeof(float));

	/* Compute determinant as multiplication of the eigenvalues */

	/* Do not calculate eigenvectors */
	char cEigVec1 = 'N';
	char cEigvec2= 'N';
	float* eigR;
	float* eigI;

	eigR = new float[rowSize];
	eigI = new float[colSize];

	/* Those are not used */
	float eigVV = 1;
	float eigVR = 1;
	
	int ldv = 1;

	/* Allocate workspace */
	int lwork = 3*rowSize;
	float* workSpace;
	workSpace = new float[lwork];
	int info;

	/* Calculate the eigenvalues */
	int N = int(rowSize);
    int LDA = int(rowSize);

	sgeev_(&cEigVec1,&cEigvec2,&N,tmpArray,&LDA,eigR,eigI,            
		&eigVV,&ldv,&eigVR,&ldv,workSpace,&lwork,&info);

	return eigR;
}

matrix matrix::EigVec()
{
		/* If the matrix is not square, return -1 (impossible determinant
		for a positive semi definite matrix) */
	if(colSize != rowSize)
		return 0;

	/* Copy the matrixArray so sgeev_ does not corrupt data */
	float* tmpArray = new float[rowSize*colSize];
	memcpy(tmpArray,matrixArray,rowSize*colSize*sizeof(float));

	/* Compute determinant as multiplication of the eigenvalues */

	/* Do not calculate eigenvectors */
	char cEigVec1 = 'N';
	char cEigvec2= 'V';
	float* eigR;
	float* eigI;

	eigR = new float[rowSize];
	eigI = new float[colSize];

	/* Those are not used */
	float eigVV = 1;

	int vecSize=rowSize*rowSize;
	float* eigVR;
	eigVR=new float[vecSize];
	
	int ldvl = 1;
	int ldvr = rowSize;
	/* Allocate workspace */
	int lwork = 4*rowSize;
	float* workSpace;
	workSpace = new float[lwork];
	int info;

	/* Calculate the eigenvalues */
	int N = int(rowSize);
    int LDA = int(rowSize);

	sgeev_(&cEigVec1,&cEigvec2,&N,tmpArray,&LDA,eigR,eigI,            
		&eigVV,&ldvl,eigVR,&ldvr,workSpace,&lwork,&info);

	matrix eig(rowSize,rowSize);
		eig=eigVR;

	return ~eig;
}

/* If type == 0, then the column sums are returned as a 1xN vector,
	type == 1 then the row sums are returned in a Nx1 vector */
matrix matrix::Sum(int type)
{
	matrix tmpMtrx;

	if(type == 0)
	{
		tmpMtrx = matrix(1,colSize);
		for(int colC = 0; colC < colSize; colC++)
		{
			for(int rowC = 0; rowC < rowSize; rowC++)
			{
				tmpMtrx.matrixArray[colC] += matrixArray[rowC + colC*rowSize];
			}
		}
	}
	else if(type == 1)
	{
		tmpMtrx = matrix(rowSize,1);
		for(int rowC = 0; rowC < rowSize; rowC++)
		{
			for(int colC = 0; colC < colSize; colC++)
			{
				tmpMtrx.matrixArray[rowC] += matrixArray[rowC + colC*rowSize];
			}
		}
	}
	
	return tmpMtrx;
}


/* Get value of an element */
float matrix::Get(int row, int col)
{


	/* Check that indexes are in the correct range */
	if((row < 1) || (row > rowSize))
	{
		rtError.SetDescription("Row index ot of range","matrix::Get");
		return -1;
	}
	else if ((col < 1) || (col > colSize))
	{
		rtError.SetDescription("Column index out of range",
			"matrix::Get");
		return -1;
	}

	/* Get value */
	return float(matrixArray[(row-1)+rowSize*(col-1)]);
}

/* Prints matrix */
void matrix::Print()
{
	if(rowSize != 0)
	{
		for(int rI = 0; rI < rowSize; rI++)
		{
			for(int cI = 0; cI < colSize; cI++)
			{
				cout << matrixArray[rI+cI*rowSize] << " ";
			}
			cout << endl;
		}
		cout << endl;
	}
	else
	{
		cout << "No values assigned to matrix " << endl;
	}
}

/* Returns the RTError of the matrix object */
RTError matrix::Status()
{
	return rtError;
}

/* Multiplication float*matrix */
const matrix operator*(const float f,const matrix& m)
{
	matrix tmpMtrx(m);

	/* Multiply with m*f */
	tmpMtrx = m*f;

	return tmpMtrx;
}


/* Returns the inverse of matrix m */
const matrix inv(const matrix& m)
{
	matrix tmpMtrx = m.Inv();

	return tmpMtrx;
}

/* Create an eye matrix with given size */
const matrix eye(const int N)
{
	matrix tmpMtrx(N,N);

	for(int inx = 0; inx < N; inx++)
	{
		tmpMtrx.Set(inx+1,inx+1,1.0);
	}

	return tmpMtrx;
}

/** Calculates a multivariate Gaussian distribution. It assumes mean 0 
and saves the value in parameter 'value' */
RTError MultGauss(matrix& epsilon, matrix& cov, float& value)
{
	RTError rtError;
	float pi = 3.14159265f;

	/* Calculate the term ep'*inv(cov)*ep used in the exponent */
	if(cov.rows() == 0)
	{
		rtError.SetDescription("Empty matrix","matrix::MultGauss");
		return rtError;
	}
	matrix tmpMtrx = ~epsilon*inv(cov)*epsilon;
	if(tmpMtrx.Status().Error())
		return tmpMtrx.Status();

	float expvalue = tmpMtrx.Get(1,1);

	float det = cov.PosDefDet();
	if(det == -1)
		return RTError("Error while calculating determinant","matrix::MultGauss");
	float k = float(cov.rows());

	value = 1.0f/(pow(2*pi,k/2.0f)*sqrt(det))*exp(-1.0f*expvalue/2.0f);

	return rtError;
}