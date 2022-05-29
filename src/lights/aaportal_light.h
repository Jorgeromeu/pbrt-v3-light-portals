#ifndef PBRT_V3_AAPORTAL_LIGHT_H
#define PBRT_V3_AAPORTAL_LIGHT_H

#include "shapes/plane.h"
#include "portal.h"
#include "diffuse.h"

namespace pbrt {

enum class PortalStrategy {SampleUniformPortal, SampleUniformLight, SampleProjection};

class AAPortalLight : public DiffuseAreaLight {

public:

    const Portal& portal;
    std::shared_ptr<AAPlane> shape;
    const PortalStrategy strat;

    // frustum data
    const bool useFrustum;
    Normal3f fn0;
    Point3f fp0;

    Normal3f fn1;
    Point3f fp1;

    Normal3f fn2;
    Point3f fp2;

    Normal3f fn3;
    Point3f fp3;


    AAPortalLight(const Transform &LightToWorld,
                  const MediumInterface &mediumInterface, const Spectrum &Le,
                  int nSamples,
                  const std::shared_ptr<AAPlane> &light,
                  const Portal &portal,
                  const PortalStrategy strategy,
                  bool useFrustum,
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

std::shared_ptr<AAPortalLight> CreateAAPortal(
        const Transform &light2world,
        const Medium *medium,
        const ParamSet &paramSet, const std::shared_ptr<AAPlane> &shape);

}

#endif //PBRT_V3_AAPORTAL_LIGHT_H
