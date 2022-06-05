#ifndef PBRT_V3_POINT_PORTAL_H
#define PBRT_V3_POINT_PORTAL_H

#include "pbrt.h"
#include "geometry.h"

namespace pbrt {

class PointPortal {
public:

    PointPortal(const Float loY, const Float hiY,
                const Float loX, const Float hiX, const Float z,
                bool greater,
                Point3f &pLight);

    bool InFrustum(const Point3f &p);

    bool InFront(const Point3f &p);

private:

    // portal geometry
    Float loY;
    Float hiY;
    Float loX;
    Float hiX;
    Float z;
    bool greater;

    Normal3f n;
    Float area;

    // frustum data
    // point and normal for each frustum plane
    Normal3f fn0;
    Normal3f fn1;
    Normal3f fn2;
    Normal3f fn3;
    Point3f fp0;
    Point3f fp1;
    Point3f fp2;
    Point3f fp3;

};

}


#endif //PBRT_V3_POINT_PORTAL_H
