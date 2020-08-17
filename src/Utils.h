#pragma once

#include <BasicLinearAlgebra.h>

#include <array>

template <typename T, size_t N>
T Maximum(const std::array<T, N> &array) {
  T max = array[0];
  for (size_t i = 0; i < N; i++) {
    max = array[i] >= max ? array[i] : max;
  }
  return max;
}

template <typename T, size_t N>
T Minimum(const std::array<T, N> &array) {
  T min = array[0];
  for (size_t i = 0; i < N; i++) {
    min = array[i] <= min ? array[i] : min;
  }
  return min;
}

template <typename T, size_t N>
std::array<T, N> Constrain(std::array<T, N> array, T min, T max) {
  std::array<T, N> out;
  for (size_t i = 0; i < array.size(); i++) {
    out[i] = (array[i] <= min) ? min : (array[i] >= max ? max : array[i]);
  }
  return out;
}

template <class T, size_t N>
std::array<T, N> MaskArray(std::array<T, N> &array,
                           const std::array<bool, N> mask) {
  std::array<T, N> out;
  for (size_t i = 0; i < N; i++) {
    out[i] = mask[i] ? array[i] : T(0);
  }
  return out;
}

template <size_t N>
std::array<int32_t, N> ConvertToFixedPoint(std::array<float, N> in,
                                           float factor) {
  std::array<int32_t, N> out;
  for (size_t i = 0; i < N; i++) {
    out[i] = in[i] * factor;
  }
  return out;
}

// TODO: get this thing to work.
// // Takes a function f and creates an array: [f(0), f(1), f(2), ...]
// template <class T, class MemberClass, size_t N, class IndexType>
// std::array<T, N> GenerateArray(T (MemberClass::*fill_function) (IndexType
// index))
// {
//     std::array<T, N> out;
//     for (size_t i = 0; i<N; i++)
//     {
//         out[i] = fill_function(i);
//     }
//     return out;
// }

/*
Return the infinity norm of a 3-vector
*/
float InfinityNorm3(BLA::Matrix<3> vec) {
  return std::max(std::max(abs(vec(0)), abs(vec(1))), abs(vec(2)));
}

// /*
// Normalizes a vector to have a maximum component of one
// */
// BLA::Matrix<3> Normalize3(BLA::Matrix<3> vec) {
//     float v = max(max(abs(vec(0)), abs(vec(1))), abs(vec(2)));
//     return vec / v;
// }