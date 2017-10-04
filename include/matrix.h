/** 
	Racetrack
	Automatic Control LiTH

	Filename: matrix.h
	Description:
		This file contains the definition of the matrix class. This class uses LAPACK
		functions. For detailed descriptions see the LAPACK documentation.

	Version:	Date:		Changes:							
		1.0		2011-07-12	First version


	Author/s:
	Isak Nielsen
*/

#ifndef MATRIX_H_
#define MATRIX_H_

//#pragma comment(lib, "C:\Temp\OSAAR\lib\liblapack.lib")
//#pragma comment(lib, "C:\Temp\OSAAR\lib\libblas.lib")

//#pragma comment(lib,"lib\liblapack.lib")
//#pragma comment(lib,"lib\libblas.lib")

#include "globaldefines.h"

class matrix
{
public:
	/** Construct a matrix with single precision, if the size is equal to 0x0 (default)
		no values are set. Otherwise they are initiated to zeroes */
	matrix(int rowS = 0, int colS = 0);

	/** Creates a new copy of the matrix object */
	matrix(const matrix& m);

	/** Destructor. Deallocates the memory */
	~matrix();

	/* Operators */

	/** Check if an operator is successfully performed by using the
		Status() function. This will return a RTError which contains the error if it has
		occured. */
	/**The operator assigns the right value to the left value. Matrices must 
		be of the same sizes. If the left side matrix is not
		initialized (rowSize == 0 and colSize == 0) then the matrix 
		gets the same dimensions as the right side matrix */
	matrix& operator=(const matrix& m);

	/** Assigns an array of floats to the matrix. The array is of the 
		form {a11...a1n a21...a2n...am1...amn}. There is no sanity
		check so make sure the dimensions are OK. */
	matrix& operator=(const float pf[]);

	/** Equal to the operation A = A - B */
	matrix& operator-=(const matrix& m);

	/** Equal to the operation A = A + B */
	matrix& operator+=(const matrix& m);

	/** The standard (element wise) matrix addition */
	const matrix operator+(const matrix& m) const;

	/** The standard (element wise) matrix subtraction */
	const matrix operator-(const matrix& m) const;

	/** The standard matrix multiplication. The number of columns
		in the matrix to the right must equal the number of rows
		in the matrix to the left. */
	const matrix operator*(const matrix& m) const;

	/** Multiplication of matrix times a scalar float */
	const matrix operator*(const float f) const;

	/** Returns the transpose of the matrix */
	const matrix operator~() const;

	/* Functions to manipulate matrix objects */
	
	/** Set the value on element given by row and col. Indicies starts 
		at 1 and moves up to rowSize (colSize) */
	RTError Set(int row, int col, float value);

	/** Returns the inverse of the matrix object. The matrix must 
		be square, i.e. the same number of rows and columns. If the 
		matrix has singular values that are too small 
		(very close to zero) then an error state is set to warn
		the user that the matrix is almost singular and inverse does
		not exist in practice */
	const matrix Inv() const;

	/** Returns the determinant for a positive definite matrix. This is calculated
		as the product of all eigenvalues. This is not a general determinant */
	float PosDefDet();

	/** Returns the eigenvalues for a NxN matrix */
	float* EigVal();

	matrix EigVec();
	/** If type == 0, then the column sums are returned as a 1xN vector,
	type == 1 then the row sums are returned in a Nx1 vector */
	matrix Sum(int type = 0);

	/** Returns a value of an element given by row and col. Indicies 
		starts at 1 and move up to rowSize (colSize) */
	float Get(int row, int col = 1);

	/** Return column size of the matrix */
	int cols(){return colSize;}

	/** Return row size of the matrix */
	int rows(){return rowSize;}

	/** Prints the matrix in command window using cout */
	void Print();

	/** Returns the RTError of the matrix object */
	RTError Status();
	
private:
	/** Number of rows in the matrix. Cannot be changed (from other 
		than 0) after the matrix has been created */
	long int rowSize;
	/** Number of cols in the matrix. Cannot be changed (from other 
		than 0) after the matrix has been created */
	long int colSize;

	/** Array that contains the matrix's values. The ordering of 
		elements correspond to Lapack's (i.e. rows are stored without being split up */
	float* matrixArray;

	/** RTError object that contains the status of the matrix. If
		an operator was not successfully performed this error will
		be in the state 'RTERROR_NOT_OK' */
	RTError rtError;
};

/** Multiplicate a scalar float with a matrix */
const matrix operator*(const float f,const matrix& m);

/** Returns the inverse of the matrix object. The matrix must 
	be square, i.e. the same number of rows and columns. If the 
	matrix has singular values that are too small 
	(very close to zero) then an error state is set to warn
	the user that the matrix is almost singular and inverse does
	not exist in practice */
const matrix inv(const matrix& m);

/** Creates an eye matrix of the specified size (i.e. the diagonal elements are 1 and
	the rest are 0 */
const matrix eye(const int size);

/** Calculates a multivariate Gaussian distribution. It assumes mean 0 and saves the
	value in parameter 'value'. Returns a RTError if it was not properly performed */
RTError MultGauss(matrix& epsilon, matrix& cov, float& value);

#endif