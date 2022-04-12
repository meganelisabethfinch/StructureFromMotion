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
            std::vector<char> combo(r);
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
};

#endif //SFM_VECTOR_UTIL_H
