#include "Vector.h"
#include <gsl/gsl_blas.h>

//constructor - creates an n row vector that is not initialized to any particular values
Vector::Vector(int n) : Matrix(n, 1)
{
}

/**
	copy operator - performs a deep copy of the Vector passed in as a parameter.
*/
Vector& Vector::operator=(const Matrix& other)
{
	if (other.getColumnCount() != 1)
	{
		throw std::logic_error("Can't copy, Matrix must have only one column");
	}
	deepCopy(other);
	return *this;
}

/**
	this method performs a shallow copy of the Vector that is passed in as a parameter.
*/
void Vector::shallow_copy(const Matrix& other)
{
	if (other.getColumnCount() != 1)
	{
		throw std::logic_error("Can't copy, Matrix must have only one column");
	}
	shallowCopy(other);
}

/**
	copy operator - performs a deep copy of the Vector passed in as a parameter.
*/
Vector& Vector::operator=(const Vector& other)
{
	deepCopy(other);
	return *this;
}

/**
	this method performs a deep copy of the vector that is passed in as a paramerer.
*/
void Vector::deep_copy(const Matrix& other)
{
	if (other.getColumnCount() != 1)
	{
		throw std::logic_error("Can't copy, Matrix must have only one column !");
	}
	deepCopy(other);
}

/**
	This method sets the current vector to be equal to one of the products: A * b or A'*b.
	The value of transA indicates if A is transposed or not
*/
void Vector::set_to_product_of(const Matrix& A, const Matrix& B, bool transA, bool transB)
{
	setToProductOf(A, B, transA, transB);
	if (this->matrix->size2 != 1)
	{
		throw std::logic_error("The result isn't a vector !");
	}
}

//This method returns a copy of the value of the matrix at (i,j)
double Vector::get(int i) const
{
	return VECTOR_AT(this->matrix, i);
}

//This method sets the value of the matrix at (i,j) to newVal.
void Vector::set(int i, double newVal)
{
	VECTOR_AT(this->matrix, i) = newVal;
}


/**
	This method sets the current vector to be equal to one of the rows of A - shallow column only!
*/
void Vector::set_to_row(const Matrix& A, int row, int start, size_t howManyCols)
{
	if (this->matrix->owner)
	{
		gsl_block_free(this->matrix->block);
	}

	//make sure that end, if unspecified is equal to the number of columns - i.e. a whole row
	if (howManyCols <= 0)
		howManyCols = A.matrix->size2 - start;

	//this is where it starts
	this->matrix->data = A.matrix->data + row * A.matrix->tda + start;
	//this vector will have as many rows as the row we're copying has columns
	this->matrix->size1 = howManyCols;
	this->matrix->size2 = 1;
	//set this to 1 - as long as on a row of the matrix, elements are packed (i.e. MULTIPLICITY is 1), then this is fine. 
	this->matrix->tda = 1;
	this->matrix->block = A.matrix->block;
	this->matrix->owner = 0;
}

//This method sets the current vector to be equal to one of the cols of A - shallow column only!
void Vector::set_to_col(const Matrix& A, int col, int start, size_t howManyRows)
{
	if (this->matrix->owner)
	{
		gsl_block_free(this->matrix->block);
	}

	//make sure that end, if unspecified is equal to the number of columns - i.e. a whole row
	if (howManyRows <= 0)
		howManyRows = A.matrix->size1 - start;

	//this is where it starts
	this->matrix->data = A.matrix->data + start * A.matrix->tda + col;
	//this vector will have as many rows as the column we're copying has rows
	this->matrix->size1 = howManyRows;
	this->matrix->size2 = 1;
	//set this to 1 - as long as on a row of the matrix, elements are packed (i.e. MULTIPLICITY is 1), then this is fine. 
	this->matrix->tda = A.matrix->tda;
	this->matrix->block = A.matrix->block;
	this->matrix->owner = 0;
}
