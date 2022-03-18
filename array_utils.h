#pragma once

/*
This file contains some vector and matrix operations for C++ arrays instead of std::vectors, or Eigen objects.
Such functions have alerady been used all over kuka_kin class and the controller class.
Pouya P. Niaz
24 July 2021
*/

#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <vector>

using namespace std;

/**
* @brief Makes a dynamically allocated matrix, and initializes it to zeros.
*
* @tparam T: Datatype of the matrix being allocated and initialized.
* @param row: number of rows
* @param col: number of columns
* @return dual pointer to the matrix generated.
*/
template <typename T>
T** allocateMatrix(size_t row, size_t col) {
	T** matrix = new T * [row];
	for (size_t i = 0; i < row; ++i) {
		matrix[i] = new T[col];
		for (size_t j = 0; j < col; j++)
			matrix[i][j] = 0.0;
	}
	return matrix;
}



/**
* @brief Generates a dynamically allocated vector, and initializes it to zeros.
*
* @tparam T: Datatype of the vector being allocated and initialized.
* @param dim: size of vector
* @return pointer to the vector
*/
template<typename T>
T* allocateVector(size_t dim) {
	T* vec = new T[dim];
	for (size_t i = 0; i < dim; i++) {
		vec[i] = 0.0;
	}
	return vec;
}


/**
* @brief Copy the contents of a matrix or vector to another one to avoid aliasing of pointers.
*
* @tparam T: Datatype of the matrix being copied
* @param ret: pointer to matrix whose values will be set.
* @param m: pointer to matrix or vector
* @param dim1: # rows of the original matrix
* @param dim2: # cols of the original matrix
*/
template<typename T>
void copyMatrix(T** ret, T** m, size_t dim1, size_t dim2) {
	for (size_t i = 0; i < dim1; i++) {
		for (size_t j = 0; j < dim2; j++) {
			ret[i][j] = m[i][j];
		}
	}
}


/**
* @brief Copy the contents of a matrix or vector to another one to avoid aliasing of pointers.
*
* @tparam T: Datatype of the vector being copied
* @param ret: pointer to vector whose values will be set by the function.
* @param v: pointer to matrix or vector
* @param dim: size of vector
*/
template<typename T>
void copyVector(T* ret, T* v, size_t dim) {
	for (size_t i = 0; i < dim; i++)
		ret[i] = v[i];
}


/**
* @brief Print a matrix in the form of a dual pointer.
*
* @tparam T: Datatype of the matrix being displayed.
* @param A: matrix dual pointer
* @param rows: number of rows from A to plot
* @param cols: number of columns from A to plot
*/
template<typename T>
void printMatrix(T** A, size_t rows, size_t cols, ostream& os = std::cout) {
	os << endl;
	for (size_t i = 0; i < rows; i++) {
		for (size_t j = 0; j < cols; j++) {
			os << std::setw(10) << A[i][j];
			if (j == cols - 1)
				os << std::endl;
		}
	}
}



/**
* @brief Print a vector in the form of a pointer.
*
* @tparam T: Datatype of the vector being displayed.
* @param V: vector pointer
* @param dim: size of vector
*/
template<typename T>
void printVector(T* V, size_t dim, ostream& os = std::cout) {
	os << "[ ";
	for (size_t i = 0; i < dim; i++) {
		os << V[i];
		if (i < dim - 1)
			os << ", ";
	}
	os << " ]";
}


/**
* @brief Casts the data type of an std::vector from Tsource to Tdest. This function returns the resulting vector.
*
* @tparam Tsource: Datatype of the source vector
* @tparam Tdest: Datatype of the destination vector
* @param in: Vector with the same data type as described in Tsource; i.e. an `std::vector<Tsource>` object.
* 
* @return Vector with values equal to the values of input vector.
*/
template<typename Tsource, typename Tdest>
vector<Tdest> castVector(vector<Tsource> in) {
	vector<Tdest> out = vector <Tdest>(in.size());
	for (size_t i = 0; i < (size_t)in.size(); i++)
		out[i] = static_cast<Tdest>(in[i]);
	return out;
}


/**
* @brief Casts the data type of an std::vector from Tsource to Tdest. 
* This function does not return anything, it only modifies the given vector.
* @tparam Tsource: Datatype of the source vector.
* @tparam Tdest: Datatype of the destination vector
* @param source: The source vector
* @param dest: The destination vector, to be modified by this function.
*/
template<typename Tsource, typename Tdest>
void castVector(vector<Tsource> source, vector<Tdest>& dest) {
	assert(source.size() == dest.size());
	size_t s = (size_t)source.size();
	for (size_t i = 0; i < s; i++)
		dest[i] = static_cast<Tdest>(source[i]);
}


/**
 * @brief Multiply two matrices and store results in an existing matrix.
 * 
 * @tparam T: The datatype/class of the matrices being multiplied with each other.
 * @param ret: The matrix double pointer in which results will be saved.
 * @param m1: The first matrix (left)
 * @param m2: The second matrix (right)
 * @param dim1: # rows of the first matrix
 * @param dim2: # cols of the first matrix, and rows of the second matrix.
 * @param dim3: # cols of the second matrix.
 */
template<typename T>
void multiplyMatrix(T** ret, T** m1, T** m2, size_t dim1, size_t dim2, size_t dim3)
{
	for (size_t i = 0; i < dim1; i++)
		for (size_t j = 0; j < dim3; j++)
				ret[i][j] = 0;

	for (size_t i = 0; i < dim1; i++)
		for (size_t j = 0; j < dim3; j++)
			for (size_t k = 0; k < dim2; k++)
				ret[i][j] += m1[i][k] * m2[k][j];
}


/**
 * @brief Multiply a matrix by a vector and store the result in an existing vector pointer
 * 
 * @tparam T: Datatype of the matrix and vectors
 * @param ret: Destination vector where the results will be stored.
 * @param m: The matrix on the left
 * @param v: The vector on the right.
 * @param dim1: # rows of the matrix.
 * @param dim2: # cols of the matrix, also size of the vector.
 */
template<typename T>
void multiplyVector(T* ret, T** m, T* v, size_t dim1, size_t dim2) {
	for (size_t i = 0; i < dim1; i++) {
		ret[i] = 0;
	}
	for (size_t i = 0; i < dim1; i++)
		for (size_t j = 0; j < dim2; j++)
			ret[i] += m[i][j] * v[j];
}


/**
 * @brief Add two matrices
 * 
 * @tparam T: Datatype of matrices
 * @param dim1: # rows
 * @param dim2: # cols
 * @param ret: Matrix in which to store the results
 * @param m1: First matrix
 * @param m2: Second matrix
 */
template<typename T>
void addMatrix(T** ret, T** m1, T** m2, size_t dim1, size_t dim2) {
	for (size_t i = 0; i < dim1; i++)
		for (size_t j = 0; j < dim2; j++)
			ret[i][j] = m1[i][j] + m2[i][j];
}


/**
 * @brief Add two vectors
 * 
 * @tparam T: Datatype of the vectors
 * @param dim: Dimension of the vectors
 * @param ret: Vector in which to store the results
 * @param v1: First vector
 * @param v2: Second vector
 */
template<typename T>
void addVector(T* ret, T* v1, T* v2, size_t dim) {
	for (size_t i = 0; i < dim; i++)
		ret[i] = v1[i] + v2[i];
}


/**
 * @brief Subtract a matrix from another one.
 * 
 * @tparam T: Datatype of the matrices
 * @param dim1: # rows of the matrices
 * @param dim2: # cols of the matrices
 * @param ret: Matrix in which to store the results
 * @param m1: First matrix
 * @param m2: Second matrix
 */
template<typename T>
void subtract(T** ret, T** m1, T** m2, size_t dim1, size_t dim2) {
	for (size_t i = 0; i < dim1; i++)
		for (size_t j = 0; j < dim2; j++)
			ret[i][j] = m1[i][j] - m2[i][j];
}


/**
 * @brief Subtract a vector from another one
 * 
 * @tparam T: Datatype of the vectors
 * @param dim: Dimension of the vectors
 * @param ret: Vector in which to store the results
 * @param v1: First vector
 * @param v2: Second vector
 */
template<typename T>
void subtract(T* ret, T* v1, T* v2, size_t dim) {
	for (size_t i = 0; i < dim; i++)
		ret[i] = v1[i] - v2[i];
}


/**
 * @brief Cross product of two 3x1 vectors
 * 
 * @tparam T: Datatype of the vectors
 * @param ret: Vector in which to store the results
 * @param a: First vector
 * @param b: Second vector
 */
template<typename T>
void cross(T* ret, T* a, T* b) {
	ret[0] = a[1] * b[2] - a[2] * b[1];
	ret[1] = a[2] * b[0] - a[0] * b[2];
	ret[2] = a[0] * b[1] - a[1] * b[0];
}


/**
 * @brief Calculate norm of a vector
 * 
 * @tparam T: Datatype of ther vector
 * @param dim: Dimension of the vector
 * @param v: Vector
 * @return The norm of the vector
 */
template<typename T>
double calcNorm(T* v, size_t dim) {
	double n = 0.0;
	for (size_t i = 0; i < dim; i++) n += v[i] * v[i];
	n = sqrt(n);
	return n;
}


/**
 * @brief Normalize a vector
 * 
 * @tparam T: Datatype of the vector
 * @param dim: Dimension of the vector
 * @param ret: Vector in which to store the results
 * @param v: Vector to normalize
 */
template<typename T>
void normalize(T* ret, T* v, size_t dim) {
	double n = calcNorm<T>(v, dim);
	for (size_t i = 0; i < dim; i++)
		ret[i] = v[i];
	for (size_t i = 0; i < dim; i++) {
		ret[i] /= n;
	}
}


/**
 * @brief Dot product of two vectors
 * 
 * @tparam T: Datatype of the vectors
 * @param dim: Dimension of the vectors
 * @param v1: First vector
 * @param v2: Second vector
 * @return Dot product of the vectors
 */
template<typename T>
double dot(T* v1, T* v2, size_t dim) {
	double d = 0.0;
	for (size_t i = 0; i < dim; i++) {
		d += v1[i] * v2[i];
	}
	return d;
}


/**
 * @brief Transpose of a matrix
 * 
 * @tparam T: Datatype of the matrix
 * @param numOrigRows: # rows of the original matrix
 * @param numOrigCols: # cols of the original matrix
 * @param ret: Matrix in which to store the results
 * @param m: Matrix to transpose.
 */
template<typename T>
void transpose(T** ret, T** m, size_t numOrigRows, size_t numOrigCols) {
	for (size_t i = 0; i < numOrigCols; i++) {
		for (size_t j = 0; j < numOrigRows; j++) {
			ret[i][j] = m[j][i];
		}
	}
}
