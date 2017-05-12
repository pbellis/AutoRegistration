#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include <Eigen/Dense>

template <class Scalar>
auto random_translation(Scalar min_x, Scalar max_x) -> Eigen::Matrix<Scalar, 3, 1>
{
    return Eigen::Matrix<Scalar, 3, 1>(Eigen::internal::random<Scalar>(min_x, max_x),
                                       Eigen::internal::random<Scalar>(min_x, max_x),
                                       Eigen::internal::random<Scalar>(min_x, max_x));
}

template <class Scalar>
auto random_quat(Scalar min_angle, Scalar max_angle) -> Eigen::Quaternion<Scalar>
{
    auto u1 = Eigen::internal::random<Scalar>(0, 1);
    auto u2 = Eigen::internal::random<Scalar>(min_angle, max_angle);
    auto u3 = Eigen::internal::random<Scalar>(min_angle, max_angle);

    auto a = std::sqrt(1 - u1);
    auto b = std::sqrt(u1);

    return Eigen::Quaternion<Scalar>(a * std::sin(u2), a * std::cos(u2), b * std::sin(u3), b * std::cos(u3));
}

template <class Scalar>
auto random_angle_axis(Scalar min_angle, Scalar max_angle) -> Eigen::AngleAxis<Scalar>
{
    auto u1 = Eigen::internal::random<Scalar>(0, 1);
    auto u2 = Eigen::internal::random<Scalar>(min_angle, max_angle);
    auto u3 = Eigen::internal::random<Scalar>(min_angle, max_angle);

    auto a = std::sqrt(1 - u1);
    auto b = std::sqrt(u1);

    return Eigen::AngleAxis<Scalar>(a * std::sin(u2), Eigen::Vector3f(a * std::cos(u2), b * std::sin(u3), b * std::cos(u3)));
}

#endif // EIGEN_UTILS_H
