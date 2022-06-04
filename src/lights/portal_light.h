#ifndef PBRT_V3_PORTAL_LIGHT_H
#define PBRT_V3_PORTAL_LIGHT_H

#include "shapes/plane.h"
#include "portals/aaportal.h"
#include "diffuse.h"

namespace pbrt {

enum class PortalStrategy {SampleUniformPortal, SampleUniformLight, SampleProjection};

class PortalLight : public DiffuseAreaLight {

public:

    const AAPortal& portal;
    std::shared_ptr<AAPlane> shape;
    const PortalStrategy strat;

    PortalLight(const Transform &LightToWorld,
                const MediumInterface &mediumInterface, const Spectrum &Le,
                int nSamples,
                const std::shared_ptr<AAPlane> &light,
                const AAPortal &portal,
                const PortalStrategy strategy,
                bool twoSided = false);

    void Preprocess(const Scene &scene) override;

    bool InFrustrum(const Point3f p) const;

    Spectrum EstimateDirect(const Interaction &it,
                            const Point2f &u1, const Point2f &u2,
                            const Scene &scene, bool specular) const;

    Spectrum EstimateDirectLight(const Interaction &it,
                                 const Point2f &u1, const Point2f &u2,
                                 const Scene &scene, bool specular) const;

    Spectrum EstimateDirectPortal(const Interaction &it,
                                 const Point2f &u1, const Point2f &u2,
                                 const Scene &scene, bool specular) const;

    Spectrum EstimateDirectProj(const Interaction &it,
                                const Point2f &u1, const Point2f &u2,
                                const Scene &scene, bool specular) const;


};

std::shared_ptr<PortalLight> CreateAAPortal(
        const Transform &light2world,
        const Medium *medium,
        const ParamSet &paramSet, const std::shared_ptr<AAPlane> &shape);

}

#endif //PBRT_V3_PORTAL_LIGHT_H
