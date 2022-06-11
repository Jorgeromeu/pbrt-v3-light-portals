#include "portal_pointlight.h"
#include "point.h"
#include "geometry.h"
#include "spectrum.h"
#include "scene.h"
#include "reflection.h"

namespace pbrt {

Spectrum PortalPointlight::EstimateDirect(const Interaction &it,
                                          const Point2f &u1, const Point2f &u2,
                                          const Scene &scene, bool specular) {

    // reused variables
    BxDFType bsdfFlags = specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    const auto &ref = (const SurfaceInteraction &)it;

    if (portal.InFront(it.p) && !portal.InFrustum(it.p)) {
        return 0;
    }

    // shadow ray
    auto wi = Normalize(pLight - it.p);
    Spectrum Li = 0;
    SurfaceInteraction lightIsect;
    Ray ray = it.SpawnRay(wi);
    if (scene.Intersect(ray, &lightIsect)) {
        Li = lightIsect.Le(-wi);
    }

    if (!Li.IsBlack()) {
        // compute BSDF for sampled direction
        Spectrum f = ref.bsdf->f(ref.wo, wi, bsdfFlags) * AbsDot(wi, ref.shading.n);

        if (!f.IsBlack()) {
            return f * Li;
        }
    }

    return 0;
}

}
