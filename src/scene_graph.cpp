//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/scene_graph.h>
#include <vector>
#include <headers/matches.h>
#include <headers/constants.h>
#include <iostream>
#include <fstream>

SceneGraph::SceneGraph(const std::vector<Image>& images, Matches& matches) {
    nodes = images;
    edges.resize(nodes.size(), std::vector<int>(nodes.size())); // zeroes matrix

    for (size_t i = 0; i < nodes.size() - 1; i++) {
        for (size_t j = i + 1; j < nodes.size(); j++) {
            auto ip = ImagePair(i,j);
            Matching2& matching2 = matches.get(ip);
            if (matching2.size() >= SCENE_GRAPH_EDGE_THRESHOLD) {
                // 1 if edge exists, 0 otherwise
                edges[i][j] = 1;
            }
        }
    }
}

void SceneGraph::toDotFile(const std::string& filename) {
    std::cout << "Writing scene graph to .DOT file." << std::endl;

    std::ofstream graph(filename);

    graph << "graph sceneGraph {" << std::endl;

    for (size_t i = 0; i < nodes.size() - 1; i++) {
        for (size_t j = 0; j < nodes.size(); j++) {
            if (edges[i][j] == 1) {
                graph << nodes[i].name << "--" << nodes[j].name << std::endl;
            }
        }
    }

    graph << "}" << std::endl;
    graph.close();
}

