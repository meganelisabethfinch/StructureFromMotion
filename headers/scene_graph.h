//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_SCENE_GRAPH_H
#define SFM_SCENE_GRAPH_H

#include <vector>
#include "matches.h"

class SceneGraph {
private:
    std::vector<Image> nodes;
    std::vector<std::vector<int>> edges;

public:
    SceneGraph(const std::vector<Image>& images, Matches& matches);

    void toDotFile(const std::string& filename);
};

#endif //SFM_SCENE_GRAPH_H
