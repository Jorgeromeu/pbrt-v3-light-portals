#ifndef PBRT_V3_PORTAL_AREALIGHT_H
#define PBRT_V3_PORTAL_AREALIGHT_H

#include "shapes/plane.h"
#include "portals/portal_light.h"
#include "portals/aaportal.h"
#include "diffuse.h"
#include "vector"

namespace pbrt {

enum class PortalStrategy {SampleUniformPortal, SampleUniformLight, SampleProjection};

class PortalArealight : public DiffuseAreaLight, public PortalLight {

public:

    const std::vector<AAPortal> portals;
    std::shared_ptr<AAPlaneShape> shape;
    const PortalStrategy strat;
    int selectedPortal;

    PortalArealight(const Transform &LightToWorld,
                    const MediumInterface &mediumInterface, const Spectrum &Le,
                    int nSamples,
                    const std::shared_ptr<AAPlaneShape> &light,
                    std::vector<AAPortal> portals,
                    const PortalStrategy strategy,
                    bool twoSided = false);

    void Preprocess(const Scene &scene) override;

    Spectrum EstimateDirect(const Interaction &it,
                            const Point2f &u1, const Point2f &u2,
                            const Scene &scene, bool specular) override;

private:
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

std::shared_ptr<PortalArealight> CreateAAPortal(
        const Transform &light2world,
        const Medium *medium,
        const ParamSet &paramSet, const std::shared_ptr<AAPlaneShape> &shape);

}

#endif //PBRT_V3_PORTAL_AREALIGHT_H
