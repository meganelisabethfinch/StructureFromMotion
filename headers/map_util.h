//
// Created by Megan Finch on 12/04/2022.
//

#ifndef SFM_MAP_UTIL_H
#define SFM_MAP_UTIL_H

class MapUtilities {
public:
    /*
    * Extracts the set of keys from a given map
    */
    template<typename TK, typename TV>
    static std::set<TK> ExtractKeys(std::map<TK, TV> const& input_map) {
        std::set<TK> retval;
        for (auto const& element : input_map) {
            retval.insert(element.first);
        }
        return retval;
    }

    template<typename TK, typename TV>
    static std::set<TV> ExtractValues(std::map<TK, TV> const& input_map) {
        std::set<TV> retval;
        for (auto const& element : input_map) {
            retval.insert(element.second);
        }
        return retval;
    }
};

#endif //SFM_MAP_UTIL_H
