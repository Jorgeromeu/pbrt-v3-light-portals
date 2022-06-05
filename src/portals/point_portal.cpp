#include "point_portal.h"

namespace pbrt {

PointPortal::PointPortal(const Float loY, const Float hiY,
                         const Float loX, const Float hiX,
                         const Float z, bool greater,
                         Point3f &pLight) :
        loY(loY),
        hiY(hiY),
        hiX(hiX),
        loX(loX),
        z(z),
        greater(greater),

        // compute portal center area and normal here so its cached for future lookups
        n(greater ? Normal3f(0, 0, 1) : Normal3f(0, 0, -1)),
        area(((hiX - loX) * (hiY - loY))) {


    Point3f p0 = Point3f(loX, loY, z);
    Point3f p1 = Point3f(loX, hiY, z);
    Point3f p2 = Point3f(hiX, loY, z);
    Point3f p3 = Point3f(hiX, hiY, z);

    // directions
    auto fd3 = Normalize(p3 - pLight);
    auto fd2 = Normalize(p2 - pLight);
    auto fd1 = Normalize(p1 - pLight);
    auto fd0 = Normalize(p0 - pLight);

    // frustum directions
    LOG(INFO) << "DBG DIR-WHITE:" << p0 << ";" << fd0;
    LOG(INFO) << "DBG DIR-BLUE:" << p1 << ";" << fd1;
    LOG(INFO) << "DBG DIR-GREEN:" << p2 << ";" << fd2;
    LOG(INFO) << "DBG DIR-RED:" << p3 << ";" << fd3;

    // frustum plane normals
    fn0 = Normal3f(Cross(fd1, fd0));
    fn1 = Normal3f(Cross(fd3, fd1));
    fn2 = Normal3f(Cross(fd2, fd3));
    fn3 = Normal3f(Cross(fd0, fd2));

    // frustum plane points
    fp0 = (p0 + p1) / 2;
    fp1 = (p1 + p3) / 2;
    fp2 = (p2 + p3) / 2;
    fp3 = (p0 + p2) / 2;

}

bool PointPortal::InFront(const Point3f &p) {
    if (greater) {
        return p.z > z;
    } else {
        return p.z < z;
    }
}

bool PointPortal::InFrustum(const Point3f &p) {
    bool res = true;
    res &= Dot(fp0 - p, fn0) >= 0;
    res &= Dot(fp1 - p, fn1) >= 0;
    res &= Dot(fp2 - p, fn2) >= 0;
    res &= Dot(fp3 - p, fn3) >= 0;

    return res;
}

}
