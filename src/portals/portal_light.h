#ifndef PBRT_V3_PORTAL_LIGHT_H
#define PBRT_V3_PORTAL_LIGHT_H

#include "spectrum.h"
#include "geometry.h"

namespace pbrt {

class PortalLight {

    // Portal Light interface
    virtual Spectrum EstimateDirect(const Interaction &it,
                                    const Point2f &u1, const Point2f &u2,
                                    const Scene &scene, bool specular) const = 0;

};

}


#endif //PBRT_V3_PORTAL_LIGHT_H
