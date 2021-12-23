#include <headers/types.h>
#include <headers/cli.h>
#include <headers/imagecollection.h>

int main(int argc, char** argv) {
    Args args;

    if (not CLIUtilities::ParseInputs(argc, argv, args)) {
        return -1;
    }

    ImageCollection images(args.inputImageDir);

    return 0;
}
