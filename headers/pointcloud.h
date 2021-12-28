//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_POINTCLOUD_H
#define SFM_POINTCLOUD_H

class PointCloud {
private:
    std::vector<Point> points;

    AddView(); // IN: Features, Matches, Intrinsic, ImageID

    BundleAdjust();

public:
    PointCloud(ImageCollection& images); // constructor

    PointCloud(ImageCollection& images, ImageID baseline1, ImageID baseline2);

    toColmap();

    toPly();
};

#endif //SFM_POINTCLOUD_H
