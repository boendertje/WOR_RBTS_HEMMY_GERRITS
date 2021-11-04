#include <cassert>
#include <stdexcept>
#include <numeric>
#include <cmath>
#include <utility>
#include <iomanip>

/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(T value)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at(row).at(column) = value;
		}
	}
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const std::initializer_list<T>& aList)
{
	// Check the arguments
	assert(aList.size() == M * N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column, ++row_iter)
		{
			matrix.at(row).at(column) = *row_iter;
		}
	}
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(
		const std::initializer_list<std::initializer_list<T> >& aList)
{
	// Check the arguments, the static assert assures that there is at least 1 M and 1 N!
	assert(aList.size() == M && (*aList.begin()).size() == N);

	auto row_iter = aList.begin();
	for (std::size_t row = 0; row < aList.size(); ++row, ++row_iter)
	{
		auto column_iter = (*row_iter).begin();
		for (std::size_t column = 0; column < (*row_iter).size();
				++column, ++column_iter)
		{
			matrix.at(row).at(column) = *column_iter;
		}
	}
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const Matrix<T, M, N>& aMatrix) :
		matrix(aMatrix.matrix)
{
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
std::array<T, N>& Matrix<T, M, N>::at(std::size_t aRowIndex)
{
	return matrix.at(aRowIndex);
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
const std::array<T, N>& Matrix<T, M, N>::at(std::size_t aRowIndex) const
{
	return matrix.at(aRowIndex);
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
T& Matrix<T, M, N>::at(std::size_t aRowIndex, std::size_t aColumnIndex)
{
	return matrix.at(aRowIndex).at(aColumnIndex);
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
const T& Matrix<T, M, N>::at(std::size_t aRowIndex,
		std::size_t aColumnIndex) const
{
	return matrix.at(aRowIndex).at(aColumnIndex);
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
std::array<T, N>& Matrix<T, M, N>::operator[](std::size_t aRowIndex)
{
	return matrix[aRowIndex];
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
const std::array<T, N>& Matrix<T, M, N>::operator[](std::size_t aRowIndex) const
{
	return matrix[aRowIndex];
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>& Matrix<T, M, N>::operator=(const Matrix<T, M, N>& rhs)
{
	if (this != &rhs)
	{
		matrix = rhs.matrix;
	}
	return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
bool Matrix<T, M, N>::operator==(const Matrix<T, M, N>& rhs) const
{
	return matrix == rhs.matrix;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
Matrix<T, M, N>& Matrix<T, M, N>::operator*=(const T2& scalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at(row).at(column) *= scalar;
		}
	}
	return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
Matrix<T, M, N> Matrix<T, M, N>::operator*(const T2& scalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");
	Matrix < T, M, N > result(*this);
	return result *= scalar;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
Matrix<T, M, N>& Matrix<T, M, N>::operator/=(const T2& aScalar)
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix.at(row).at(column) /= aScalar;
		}
	}
	return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
Matrix<T, M, N> Matrix<T, M, N>::operator/(const T2& aScalar) const
{
	static_assert( std::is_arithmetic<T2>::value, "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

	Matrix < T, M, N > result(*this);
	return result /= aScalar;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>& Matrix<T, M, N>::operator+=(const Matrix<T, M, N>& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] += rhs.at(row, column);
		}
	}
	return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::operator+(const Matrix<T, M, N>& rhs) const
{
	Matrix < T, M, N > result(*this);
	return result += rhs;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>& Matrix<T, M, N>::operator-=(const Matrix<T, M, N>& rhs)
{
	for (std::size_t row = 0; row < M; ++row)
	{
		for (std::size_t column = 0; column < N; ++column)
		{
			matrix[row][column] -= rhs.at(row, column);
		}
	}
	return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::operator-(const Matrix<T, M, N>& rhs) const
{
	Matrix < T, M, N > result(*this);
	return result -= rhs;
}
/**
 * (M, N) * (N, P) -> (M, P)
 */
template<typename T, std::size_t M, std::size_t N>
template<std::size_t columns>
Matrix<T, M, columns> Matrix<T, M, N>::operator*(
		const Matrix<T, N, columns>& rhs) const
{
	// TODO Implement this function
	Matrix < T, M, columns > result;

	for (std::size_t l_row = 0; l_row < M; ++l_row)
	{

		for (std::size_t r_column = 0; r_column < columns; ++r_column)
		{

			for (std::size_t l_column = 0; l_column < N; ++l_column)
			{

				result.at(l_row, r_column) += at(l_row, l_column)
						* rhs.at(l_column, r_column);

			}

		}
	}

	return result;
}

/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, N, M> Matrix<T, M, N>::transpose() const
{
	Matrix < T, N, M > result;
	for (size_t i = 0; i < M; ++i)
	{
		auto row = this->at(i);
		for (size_t x = 0; x < row.size(); ++x)
		{

			result[x][i] = row[x];
		}
	}
	return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::identity() const
{

	Matrix < T, M, N > result;

	for (size_t i = 0; i < N && i < M; ++i)
	{
		result[i][i] = 1;
	}
	return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::gauss() const
{
	// TODO Implement this function
	Matrix < T, M, N > result = *this;
	for (size_t row_index = 0; row_index < M; ++row_index)
	{

			unsigned int indexHoogte = 0;

			// Zoek de hoogste nummer
			for (size_t i = row_index + 1; i < M; ++i)
			{
				indexHoogte =
						result[i][row_index] > result[indexHoogte][row_index] ?
								i : indexHoogte;
			}

			// swap integer
			if (indexHoogte != 0)
			{
				std::swap(result[row_index], result[indexHoogte]);
			}

		// Deel de eerste rij door zichzelf zodat de eerste niet 0 een 1 wordt;
		double zichzeldelen = result[row_index][row_index];

		for (size_t j = 0; j < N; ++j)
		{
			result[row_index][j] = result[row_index][j] / zichzeldelen;
		}

		/* Doe voor de rest: de eerst niet nul
		 *
		 */
		for (size_t k = row_index + 1; k < M; ++k)
		{
			// Check factor
			double delendoor = result[k][row_index] / 1;
			for (size_t j = 0; j < result.getColumns(); ++j)
			{

				result[k][j] = result[k][j]
						- (result[row_index][j] * delendoor);
			}
		}
	}

	return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::gaussJordan() const
{
	// TODO Implement this function
	Matrix < T, M, N > result = this->gauss();
	for (size_t row_index = this->getRows() -1; row_index > 0; --row_index)
	{

		for (int64_t k = row_index - 1; k >= 0; --k)
		{
					// Check factor

		   double Keer = result[k][row_index] / 1;
					for (size_t j = row_index; j < result.getColumns(); ++j)
					{

						result[k][j] = result[k][j]
								- (result[row_index][j] * Keer);
					}
					if(k == 0)
					{
						break;
					}

		}
	}


	return result;
}
/**
 *

 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, 1> Matrix<T, M, N>::solve() const
{
	Matrix < T, M, 1 > result;
	Matrix < T, M, N > gausJordan = this->gaussJordan();

	for(size_t i = 0 ; i<M;++i)
	{
		result[i][0] = gausJordan.at(i).at(N-1);
	}
	return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N> Matrix<T, M, N>::inverse() const
{
	// TODO Implement this function
	Matrix < T, N, M > result;
	Matrix <T,N,M * 2> tempMatrix;
	Matrix < T,N,M> identity = this->identity();
	for(size_t x = 0 ; x < M;++x)
	{
		for(size_t y = 0 ; y < N; ++y)
		{
			tempMatrix[y][x] = this->at(y,x);
			tempMatrix[y][x + M] = identity[y][x];
		}
	}
	tempMatrix = tempMatrix.gaussJordan();

	for(size_t x = 0 ; x < M;++x)
	{
		for(size_t y = 0 ; y < N; ++y)
			{
				result[y][x] = tempMatrix.at(y,x + M);
			}
		}
	return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
std::string Matrix<T, M, N>::to_string() const
{
	std::string result = "Matrix<columns: " + std::to_string(N) + ",rows" + std::to_string(M)
			+ ">\n{\n";
	for (std::size_t i = 0; i < M; ++i)
	{
		for (std::size_t j = 0; j < N; ++j)
		{
			result += std::to_string(matrix[i][j]) + ",";
		}
		result += "\n";
	}
	result += "}";
	return result;
}
/**
 *
 */
template<typename T, const std::size_t N>
bool equals(const Matrix<T, 1, N>& lhs, const Matrix<T, 1, N>& rhs,
		const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
		const unsigned long aFactor /*= 1*/)
{
	// TODO Implement this function
	for(size_t y = 0; y < N;++y)
			{
				T Lwaarde = lhs[0][y];
				T Rwaarde = rhs[0][y];
				if(!(Lwaarde + (aPrecision * aFactor) >= Rwaarde && Lwaarde - (aPrecision * aFactor) <= Rwaarde))
				{

					return false;
				}

			}

		return true;
}
/**
 *
 */
template<typename T, const std::size_t M>
bool equals(const Matrix<T, M, 1>& lhs, const Matrix<T, M, 1>& rhs,
		const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
		const unsigned long aFactor /*= 1*/)
{
	// TODO Implement this function
		for(size_t x = 0; x < M;++x)
			{
				T Lwaarde = lhs[x][0];
				T Rwaarde = rhs[x][0];
				if(!(Lwaarde + (aPrecision * aFactor) >= Rwaarde && Lwaarde - (aPrecision * aFactor) <= Rwaarde))
				{

					return false;
				}

			}

		return true;
}
template<class T, std::size_t M, std::size_t N>
Matrix<T, 2, 3> Matrix<T, M, N>::makeJacobi(std::vector<double> lengtes,std::vector<double> theta) const
{
	/*
	 *xθ1 = l1 cos(θ1 ) + l2 cos(θ1 + θ2 ) + l3 cos(θ1+θ2+θ3)
	  x02 = l2 cos(θ1 + θ2) + l3 cos(θ1 + θ2 + θ3 )
      x03= 	l3 cos(θ1 + θ2 + θ3)

      yθ1 = −l1 sin(θ 1 ) − l 2 sin(θ 1 + θ 2 ) − l 3 sin(θ 1 + θ 2 + θ 3 )
      yθ2 = −l2 sin(θ 1 + θ 2 ) − l 3 sin(θ 1 + θ 2 + θ 3 )
      y03 = −l3 sin(θ 1 + θ 2 + θ 3)
	 *
	 */

	// TODO Implement this function
	Matrix < T, 2, 3 > result;
	if(lengtes.size() == 3 && theta.size() == 3)
	{
		double x01 = lengtes[0] * cos(theta[0]) + lengtes[1] * cos(theta[0] + theta[1]) + lengtes[2] * cos(theta[0] + theta[1] + theta[2]);
		double x02 = lengtes[1] * cos(theta[0] + theta[1]) + lengtes[2] * cos(theta[0] + theta[1] + theta[2]);
		double x03 = lengtes[2] * cos(theta[0] + theta[1] + theta[2]);

		double y01 = -lengtes[0] * sin(theta[0]) - lengtes[1] * sin(theta[0] + theta[1]) - lengtes[2] * sin(theta[0] + theta[1] + theta[2]);
		double y02 = -lengtes[1] * sin(theta[0] + theta[1]) - lengtes[2] * sin(theta[0] + theta[1] + theta[2]);
		double y03 = -lengtes[2] * sin(theta[0] + theta[1] + theta[2]);

		result[0][0] = x01;
		result[0][1] = x02;
		result[0][2] = x03;

		result[1][0] = y01;
		result[1][1] = y02;
		result[1][2] = y03;
	}
	return result;
}
template<class T, std::size_t M, std::size_t N>
Matrix<T, 3, 2> Matrix<T, M, N>::makeJacobiInverse(std::vector<double> lengtes,std::vector<double> theta) const
{

	Matrix < T, 3, 2 > result;
	Matrix < T, 2, 3 > jacobi;
	if(lengtes.size() == 3 && theta.size() == 3)
	{
		jacobi = jacobi.makeJacobi(lengtes,theta);
		Matrix <T,2,2> jac = jacobi * jacobi.transpose();
		jac = jac.inverse();
		result = jacobi.transpose() * jac;
	}
	return result;
}

/**
 *
 */
template<typename T, const std::size_t M, const std::size_t N>
bool equals(const Matrix<T, M, N>& lhs, const Matrix<T, M, N>& rhs,
		const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
		const unsigned long aFactor /*= 1*/)
{
	// TODO Implement this function

	for(size_t y = 0; y < N;++y)
	{
		for(size_t x = 0; x < M;++x)
		{
			T Lwaarde = lhs[x][y];
			T Rwaarde = rhs[x][y];
			if(!(Lwaarde + (aPrecision * aFactor) >= Rwaarde && Lwaarde - (aPrecision * aFactor) <= Rwaarde))
			{

				return false;
			}

		}

	}
	return true;
}


