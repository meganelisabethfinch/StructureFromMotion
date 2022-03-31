# :japanese_castle: Structure From Motion

❗**Disclaimer:** midpoint triangulation and Zhang bundle adjustment are still a work in progress. Don't expect these methods to work yet!

## :keyboard: Command Line Interface
### Mandatory Arguments
* `--input` or `-i "/path/to/input/directory"`
* `--output` or `-o "/path/to/output/directory"`

### Optional Arguments
* `--remove_statistical_outliers`. Removes statistical outliers before each bundle adjustment. A statistical inlier is a point for which the average distance to its `k` nearest neighbours is below the computed threshold: `mean + stddev_mult * stddev`.
* `--remove_radial_outliers`. Removes radial outliers before each bundle adjustment. A radial inlier is a point with at least `n` neighbours within a defined search radius.
* `--baseline` or `-b "IMAGE_INDEX"`. The pair of images to be used as the baseline of the reconstruction.
* `--detector` or `-d "DETECTOR_TYPE"`. The type of feature detector used. Allowed values are `SIFT` and `ORB`.
* `--triangulator` or ``-t "TRIANGULATOR_TYPE"`. The triangulation method used. Allowed values are `LINEAR` and `MIDPOINT`.
* `--bundle_adjuster` or `-a "BUNDLE_ADJUSTER_TYPE"`. The bundle adjustment method used. Allowed values are `OFF`, `BASIC` and `ZHANG`.
    * `OFF`: no bundle adjustment is applied.
    * `BASIC`: bundle adjustment minimises the overall reprojection error.
    * `ZHANG`: bundle adjustment minimises the error as defined by Zhang et al. in [1].

## :books: Sources
[1] J. Zhang, M. Boutin, and D. G. Aliaga, “Robust bundle adjustment for structure from motion,” in 2006 International Conference on Image Processing, pp. 2185–2188, IEEE, 2006.
