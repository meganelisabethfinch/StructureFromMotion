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
};

#endif //SFM_VECTOR_UTIL_H
