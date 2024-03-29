#pragma once

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include "MathLib/src/Matrix.h"

#define VECTOR_AT(v, i) (*((v->data + (i * v->tda ))))


/*====================================================================================================================================================================*
 | This class will be used to represent matrices of arbitrary sizes (m rows by n columns) that have elements of type double. The underlying data strucutre used by    |
 | this class is gsl's (Gnu Scientific Library) matrix class. This class also makes use of the ATLAS implementation of BLAS for some operations such as matrix-matrix |
 | multiplication. This class is meant to improve performance, not necessarily ease of use.                                                                           |
 *====================================================================================================================================================================*/

class Vector : public Matrix
{
public:
	//constructor - creates an n row vector that is not initialized to any particular values
	Vector(int n);

	//default constructor
	Vector() = delete;
	Vector(const Vector& other) = default;
	~Vector() = default;

	//copy operator - performs a deep copy of the Vector passed in as a parameter.
	Vector& operator=(const Vector& other);

	//copy operator - performs a deep copy of the Vector passed in as a parameter.
	Vector& operator=(const Matrix& other);

	//this method performs a shallow copy of the Vector that is passed in as a parameter.
	void shallow_copy(const Matrix& other);

	//this method performs a deep copy of the vector that is passed in as a paramerer.
	void deep_copy(const Matrix& other);

	//	This method sets the current vector to be equal to one of the products: A * b or A'*b.
	//	The value of transA indicates if A is transposed or not
	void set_to_product_of(const Matrix& A, const Matrix& B, bool transA = false, bool transB = false);


	//This method sets the current vector to be equal to one of the rows of A - shallow column only!
	void set_to_row(const Matrix& A, int row, int start = 0, size_t howMany = -1);

	//This method sets the current vector to be equal to one of the cols of A - shallow column only!
	void set_to_col(const Matrix& A, int col, int start = 0, size_t howMany = -1);

	//This method returns a copy of the value of the matrix at (i,j)
	double get(int i) const;

	//This method sets the value of the matrix at (i,j) to newVal.
	void set(int i, double newVal);

	//Computes the 2-norm squared for the current vector.
	double normSquared()
	{
		int r = getRowCount();
		double result = 0;
		for (int i = 0; i < r; i++)
		{
			double n = MATRIX_AT(matrix, i, 0);
			result += n * n;
		}
		return result;
	}
};
