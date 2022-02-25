# :japanese_castle: Structure From Motion

## :keyboard: Command Line Interface
### Mandatory Arguments
* `-i "/path/to/input/directory"`

### Optional Arguments
* `-d "DETECTOR_TYPE"`. The type of feature detector used. Allowed values are `SIFT` and `ORB`.
* `-t "TRIANGULATOR_TYPE"`. The triangulation method used. Allowed values are `LINEAR` and `MIDPOINT`.
* `-a "BUNDLE_ADJUSTER_TYPE"`. The bundle adjustment method used. Allowed values are `OFF`, `BASIC` and `ZHANG`.
    * `OFF`: no bundle adjustment is applied.
    * `BASIC`: bundle adjustment minimises the overall reprojection error.
    * `ZHANG`: bundle adjustment minimises the error as defined by Zhang et al. in [1].

## :books: Sources
[1] J. Zhang, M. Boutin, and D. G. Aliaga, “Robust bundle adjustment for structure from motion,” in 2006 International Conference on Image Processing, pp. 2185–2188, IEEE, 2006.
