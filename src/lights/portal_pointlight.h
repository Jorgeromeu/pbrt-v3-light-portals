#ifndef PBRT_V3_PORTAL_POINTLIGHT_H
#define PBRT_V3_PORTAL_POINTLIGHT_H

#include "transform.h"
#include "point.h"
#include "portals/point_portal.h"
#include "portals/portal.h"
#include "portals/portal_light.h"

namespace pbrt {

class PortalPointlight : public PointLight, public PortalLight {
public:

    PortalPointlight(const Transform &LightToWorld,
                     const MediumInterface &mediumInterface, const Spectrum &I,
                     PointPortal &portal)
            : PointLight(LightToWorld, mediumInterface, I),
              portal(portal) {}

    // Portal Light interface
    Spectrum EstimateDirect(const Interaction &it,
                            const Point2f &u1, const Point2f &u2,
                            const Scene &scene, bool specular);

private:

    PointPortal &portal;

};


}

#endif //PBRT_V3_PORTAL_POINTLIGHT_H
