
#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <Eigen/Dense>

namespace rapyuta_pose_estimator
{

// Type definitions
typedef Eigen::Matrix<double, 6, 6> Matrix6d; //!< A 6x6 matrix of doubles
typedef Eigen::Matrix<double, 2, 6> Matrix2x6d; //!< A 2x6 matrix of doubles
typedef Eigen::Matrix<double, 3, 4> Matrix3x4d; //!< A 3x4 matrix of doubles
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXYd; //!< A matrix of doubles containing dynamic rows and columns
typedef Eigen::Matrix<unsigned, Eigen::Dynamic, Eigen::Dynamic> MatrixXYu; //!< A matrix of unsigned integers containing dynamic rows and columns
typedef Eigen::Matrix<double, 6, 1> Vector6d; //!< A column vector of 6 elements containing doubles
typedef Eigen::Matrix<unsigned, 3, 1> Vector3u; //!< A column vector of 3 elements containing unsigned integers
typedef Eigen::Matrix<unsigned, 4, 1> Vector4u; //!< A column vector of 4 elements containing unsigned integers
typedef Eigen::Matrix<unsigned, Eigen::Dynamic, 1> VectorXu; //!< A dynamic column vector containing unsigened integers
typedef Eigen::Matrix<unsigned, Eigen::Dynamic, 2> VectorXuPairs; //!< A matrix with a dynamic number of rows and 2 columns containing unsigned integers
typedef Eigen::Matrix<double, 1, Eigen::Dynamic> RowXd; //!< A dynamic row vector containing doubles
typedef Eigen::Matrix<unsigned, 1, Eigen::Dynamic> RowXu; //!< A dynamic row vector containing unsigned integers
typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints; //!< A dynamic column vector containing Vector2D elements. \see Vector2d
typedef Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, 1> List3DPoints; //!< A dynamic column vector containing Vector3D elements. \see Vector3d
typedef Eigen::Matrix<Eigen::Vector4d, Eigen::Dynamic, 1> List4DPoints; //!< A dynamic column vector containing Vector4D elements. \see Vector4d

} // namespace rapyuta_pose_estimator_lib

#endif /* DATATYPES_H_ */
