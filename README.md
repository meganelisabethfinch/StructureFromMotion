# :japanese_castle: Structure From Motion

❗**Disclaimer:** midpoint triangulation and Zhang bundle adjustment are still a work in progress. Don't expect these methods to work yet!

## :keyboard: Command Line Interface
### Mandatory Arguments
* `--input` or `-i "/path/to/input/directory"`

### Optional Arguments
* `--output` or `-o "/path/to/output/directory"`
* `--output_format` or `-g "OUTPUT_FORMAT"`. The point cloud and cameras positions will be written to a .ply file by default, but additional output formats can be specified. Allowed values are 
  * `PLY_POINT_CLOUD`
  * `PLY_CAMERAS`
  * `PCD_POINT_CLOUD`
  * `VTK_MESH`
* `--baseline` or `-b "IMAGE_INDEX"`. The pair of images to be used as the baseline of the reconstruction.
* `--detector` or `-d "DETECTOR_TYPE"`. The type of feature detector used. Allowed values are
  * `SIFT`
  * `ORB`
* `--triangulator` or `-t "TRIANGULATOR_TYPE"`. The triangulation method used. Allowed values are
  * `LINEAR`
  * `MIDPOINT`
* `--bundle_adjuster` or `-a "BUNDLE_ADJUSTER_TYPE"`. The bundle adjustment method used. Allowed values are
    * `OFF`: no bundle adjustment is applied.
    * `BASIC`: bundle adjustment minimises the overall reprojection error.
    * `ZHANG`: bundle adjustment minimises the error as defined by Zhang et al. in [1].
* `--filter` or `-f "FILTER_TYPE"`. A filter to be applied upon registering each image. Each filter removes certain outliers. Allowed values are
  * `STATISTICAL`
  * `RADIAL`
* `--loss` or `-l "LOSS_TYPE"`. Changes the loss function used during bundle adjustment. Allowed values are
  * `NULL`. The default loss function is used. This is just the squared norm of the residuals.
  * `HUBER`. See [Ceres Solver docs](http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres9HuberLossE).
  * `SOFTLONE`. See [Ceres Solver docs](http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres12SoftLOneLossE).
  * `CAUCHY`. See [Ceres Solver docs](http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres10CauchyLossE).

## :books: Sources
[1] J. Zhang, M. Boutin, and D. G. Aliaga, “Robust bundle adjustment for structure from motion,” in 2006 International Conference on Image Processing, pp. 2185–2188, IEEE, 2006.
