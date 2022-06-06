#ifndef PBRT_V3_AA_PLANE_H
#define PBRT_V3_AA_PLANE_H

#include "geometry.h"

namespace pbrt {

class AAPlane {

private:
    Float area;

public:

    AAPlane(const Point3f &lo,
            const Point3f &hi,
            int axis,
            bool facingForwards) :
            lo(lo),
            hi(hi),
            facingFw(facingForwards),
            ax(axis),
            ax0(axis == 2 ? 0 : (axis == 0 ? 1 : 2)),
            ax1(axis == 2 ? 1 : (axis == 1 ? 2 : 0)) {
        area = (hi[ax0] - lo[ax0]) * (hi[ax1] - lo[ax1]);
    }

    bool facingFw;
    Point3f lo;
    Point3f hi;
    const int ax;
    const int ax0;
    const int ax1;

    Point3f V0() const;

    Point3f V1() const;

    Point3f V2() const;

    Point3f V3() const;

    Point3f Sample(const Point2f &u, Float *pdf) const;

    Float Area() const;

    Normal3f Normal() const;

    bool Intersect(const Ray &ray, Float *tHit) const;

    Point2f Uv(Point3f p) const;

    bool InFront(Point3f p) const;

};

}


#endif //PBRT_V3_AA_PLANE_H
