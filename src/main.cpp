#include <headers/types.h>
#include <headers/cli_util.h>
#include <headers/imagecollection.h>
#include <headers/constants.h>
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "----------- Parse Inputs ------------" << std::endl;
    Args args;

    if (not CLIUtilities::ParseInputs(argc, argv, args)) {
        return -1;
    }

    CLIUtilities::Summary(args);

    std::cout << "----------- Open Images ------------" << std::endl;
    ImageCollection images(args.inputImageDir);

    std::cout << "--------- Extract Features ---------" << std::endl;
    auto detector = CLIUtilities::CreateDetector(args.detectorType);
    images.ExtractFeatures(detector);

    if (DEFAULT_VISUAL_DEBUG >= VisualDebugLevel::SHOW_EXAMPLES) {
        images.visualiseKeyPoints(0);
    }

    std::cout << "----------- Find Matches -----------" << std::endl;
    auto matcher = CLIUtilities::CreateMatcher(args.matcherType);
    images.FindMatches(matcher);

    if (DEFAULT_VISUAL_DEBUG >= VisualDebugLevel::SHOW_EXAMPLES) {
        images.visualiseMatches(0, 1);
    }

    auto graph = images.toSceneGraph();
    graph.toDotFile("scene_graph.dot");

    std::cout << "---- Find Baseline Triangulator ---" << std::endl;
    auto triangulator = CLIUtilities::CreateTriangulator(args.triangulatorType);
    auto bundleAdjuster = CLIUtilities::CreateBundleAdjuster(args.bundleAdjusterType);
    auto filters = CLIUtilities::CreateFilters(args.filterTypes);

    if (args.useHomographyOrdering) {
        auto recon = images.toSceneReconstruction(triangulator, bundleAdjuster, filters);
        recon.outputToFiles(args.outputDir, args.outputTypes);
    } else {
        auto recon = images.toSceneReconstruction(triangulator, bundleAdjuster, filters,
                                                  args.baselinePair);
        recon.outputToFiles(args.outputDir, args.outputTypes);
    }

    return 0;
}
