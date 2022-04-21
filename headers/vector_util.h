//
// Created by Megan Finch on 29/03/2022.
//

#ifndef SFM_VECTOR_UTIL_H
#define SFM_VECTOR_UTIL_H

#include <vector>

class VectorUtilities {
public:
    /*
     * Removes all the indices in rm from a vector v
     * @param v - the vector of data
     * @param rm - the SORTED vector of indices to remove, in ascending order
     */
    template<typename INT, typename T>
    static void removeIndicesFromVector(std::vector <T> &v, std::vector <INT> &rm) {
        size_t rm_index = 0;
        v.erase(
                std::remove_if(std::begin(v), std::end(v), [&](T& elem)
                {
                    if (rm.size() != rm_index && &elem - &v[0] == rm[rm_index])
                    {
                        rm_index++;
                        return true;
                    }
                    return false;
                }),
                std::end(v)
        );
    }

    template<typename T>
    static std::vector<std::vector<T>> generateCombinations(std::vector<T> arr, int n, int r) {
        std::vector<std::vector<T>> allCombos;

        std::vector<bool> v(n);
        std::fill(v.begin(), v.begin() + r, true);

        do {
            std::vector<T> combo(r);
            int pos = 0;
            for (int i = 0; i < n; ++i) {
                if (v[i]) {
                    combo[pos] = arr[i];
                    pos++;
                }
            }
            allCombos.push_back(combo);
        } while (std::prev_permutation(v.begin(), v.end()));

        return allCombos;
    }

    /*
     * Computes v1 . v2 for n-dimensional vectors v1, v2
     */
    template <typename T>
    static bool dot(const T* const v1, const T* const v2, size_t n, T* out) {
        T result = v1[0] * v2[0];
        for (size_t i = 1; i < n; i++) {
            result += v1[i] * v2[i];
        }
        out[0] = result;
        return true;
    }

    /*
     * Computes v1 x v2 for 3-vectors v1, v2
     */
    template <typename T>
    static bool cross(const T* const v1, const T* const v2, T* out) {
        // v1 = [x1,y1,z1] and v2 = [x2,y2,z2]
        // i: y1 z2 - y2 z1
        out[0] = v1[1] * v2[2] - v2[1] * v1[2];
        // j: x2 z1 - x1 z2
        out[1] = v2[0] * v1[2] - v1[0] * v2[2];
        // k: x1 y2 - y1 x2
        out[2] = v1[0] * v2[1] - v1[1] * v2[0];
        return true;
    }

    /*
     * Computes v1 - v2 for n-dimensional vectors v1, v2
     */
    template <typename T>
    static bool subtract(const T* const v1, const T* const v2, size_t n, T* out) {
        for (size_t i = 0; i < n; i++) {
            out[i] = v1[i] - v2[i];
        }
        return true;
    }
};

#endif //SFM_VECTOR_UTIL_H
