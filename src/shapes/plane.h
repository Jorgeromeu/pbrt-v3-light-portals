//
// Created by jorge on 5/26/22.
//

#ifndef PBRT_V3_PLANE_H
#define PBRT_V3_PLANE_H

#include "shape.h"

namespace pbrt {

class AAPlane : public Shape {

public:
    AAPlane(const Transform *ObjectToWorld, const Transform *WorldToObject,
            bool reverseOrientation,
            const Point3f &lo,
            const Point3f &hi,
            const int axis,
            const Normal3f &normal)
            : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
              lo(lo),
              hi(hi),
              ax(axis),
              // other two axis depend on value of axis
              ax0(axis == 0 ? 1 : 0),
              ax1(axis == 0 ? 2 : axis == 1 ? 2 : 1),
              n(normal) {}

    // bounds
    Bounds3f ObjectBound() const override;

    // intersection
    bool Intersect(const Ray &ray, Float *tHit,
                           SurfaceInteraction *isect,
                           bool testAlphaTexture = true) const override;

    Float Area() const override;

    // Sample a point on the surface of the shape and return the PDF with
    // respect to area on the surface.
    Interaction Sample(const Point2f &u, Float *pdf) const override;

    Float Pdf(const Interaction &) const override { return 1 / Area(); }

    // portal data
    const Point3f lo;
    const Point3f hi;
    const Normal3f n;

    // axi
    const int ax;
    int ax0;
    int ax1;


};

std::shared_ptr<Shape> CreateAAPlaneShape(const Transform *o2w,
                                          const Transform *w2o,
                                          bool reverseOrientation,
                                          const ParamSet &params);

}


#endif //PBRT_V3_PLANE_H
