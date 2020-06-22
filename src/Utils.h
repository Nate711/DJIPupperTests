#include <array>

template <typename T, size_t N>
T Maximum(const std::array<T, N> &array)
{
    T max = array[0];
    for (size_t i = 0; i < N; i++)
    {
        max = array[i] >= max ? array[i] : max;
    }
    return max;
}

template <typename T, size_t N>
T Minimum(const std::array<T, N> &array)
{
    T min = array[0];
    for (size_t i = 0; i < N; i++)
    {
        min = array[i] <= min ? array[i] : min;
    }
    return min;
}

template <typename T, size_t N>
std::array<T, N> Constrain(std::array<T, N> array, T min, T max)
{
    std::array<T, N> out;
    for (size_t i = 0; i < array.size(); i++)
    {
        out[i] = (array[i] <= min) ? min : (array[i] >= max ? max : array[i]);
    }
    return out;
}

template <class T, size_t N>
std::array<T, N> MaskArray(std::array<T, N> &array, const std::array<bool, N> mask)
{
    std::array<T, N> out;
    for (size_t i = 0; i < N; i++)
    {
        out[i] = mask[i] ? array[i] : T(0);
    }
    return out;
}


template <size_t N>
std::array<int32_t, N> ConvertToFixedPoint(std::array<float, N> in, float factor)
{
    std::array<int32_t, N> out;
    for (size_t i = 0; i < N; i++)
    {
        out[i] = in[i] * factor;
    }
    return out;
}