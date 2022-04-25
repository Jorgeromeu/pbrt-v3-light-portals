/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include "integrators/hero_path.h"
#include "bssrdf.h"
#include "camera.h"
#include "film.h"
#include "interaction.h"
#include "paramset.h"
#include "scene.h"
#include "stats.h"

namespace pbrt {

STAT_PERCENT("Integrator/Zero-radiance paths", zeroRadiancePaths, totalPaths);
STAT_INT_DISTRIBUTION("Integrator/Path length", pathLength);

HeroPathIntegrator::HeroPathIntegrator(int maxDepth,
                                       std::shared_ptr<const Camera> camera,
                                       std::shared_ptr<Sampler> sampler,
                                       const Bounds2i &pixelBounds, 
                                       Float rrThreshold)
: HeroSamplerIntegrator(camera, sampler, pixelBounds),
  maxDepth(maxDepth),
  rrThreshold(rrThreshold) {}

Spectrum HeroPathIntegrator::Li(const RayDifferential &r, 
                                const Scene &scene,
                                Sampler &sampler,
                                MemoryArena &arena,
                                int depth) const {
  ProfilePhase p(Prof::SamplerIntegratorLi);
  RayDifferential ray(r);

  /* Tracking values for path computation */
  Spectrum Lo(0.f);             // Exitant radiance along path
  Spectrum beta(1.f);           // Throughput along path
  Float etaScale = 1.f;         // Relative refractive index scaling along path

  /* Status flags for path computations */
  bool isWvlDependent = false;  // Is the path wvl. dependent from some vertex onward?
  
  /* Tracking values for HWSS */
  Vector4f pathWvlPdf(1.f);     // Product of bsdf pdfs along path per wvl
  Vector4i wvlIdx;              // Bin index of the four wavelengths
  Spectrum wvlPdf(1.f);         // Sampling density of the four wavelengths

  /* Initialize HWSS tracking values */
  for (int i = 0; i < nWvls; ++i) {
    wvlIdx[i] = Spectrum::indexFromWavelength(ray.wvls[i]);
    wvlPdf[wvlIdx[i]] = spectralDistribution.Pdf(wvlIdx[i]);
  }

  int bounces;  
  for (bounces = 0;; ++bounces) {
    /* Find next path vertex by raytracing, and store details in _isect_ */
    SurfaceInteraction isect;
    bool foundIntersection = scene.Intersect(ray, &isect);

    /* If no intersection could be found, return radiance from environment emitters */
    if (!foundIntersection) {
      for (const auto &light : scene.infiniteLights) {
        Spectrum Le = light->Le(ray);
        if (Le.IsBlack()) { continue; }

        // Compute energy; wavelength dependency introduces a special case with HWSS
        if (isWvlDependent) {
          Lo += beta * Le / (wvlPdf * Sum(pathWvlPdf));
        } else {
          Lo += beta * Le;
        }
      }

      break; // Terminate path
    }

    /* If an emitter was instead encountered, add the contributed radiance */
    Spectrum Le = isect.Le(-ray.d);
    if (!Le.IsBlack()) {
      // Compute energy; wavelength dependency introduces a special case with HWSS
      if (isWvlDependent) {
        Lo += beta * Le / (wvlPdf * Sum(pathWvlPdf));
      } else {
        Lo += beta * Le;
      } 

      /* TODO: mitsuba returns here instead of adding, but PBRT's path tracer continues bouncing 
         from the emitter onward. Probably a weighting difference somewhere? Blergh. Keeping
         disabled for now. Should compare converged outputs of both renderers. */
      // break; // Terminate path
    }

    /* Terminate path if maximum path depth was reached */
    if (bounces >= maxDepth) { break; }

    /* Compute scattering function and obtain the BSDF at the current intersection */
    isect.ComputeScatteringFunctions(ray, arena, true);
    const BSDF *bsdf = isect.bsdf;

    /* Skip medium boundaries and null objects by just continuing the path */
    if (!bsdf) {
      ray = isect.SpawnRay(ray.d);
      bounces--;
      continue;
    }

    /* Sample a new direction and obtain (non-wavelength-dependent) throughput from the BSDF */
    Float bsdfPdf;
    Vector3f wo = -ray.d, wi;
    BxDFType flags;
    Spectrum f = bsdf->Sample_f(wo, &wi, sampler.Get2D(), &bsdfPdf, BSDF_ALL, &flags);
    if (f.IsBlack() || bsdfPdf == 0.f) { break; }

    /* Wavelength dependency currently occurs only on transmission through dispersive glass */    
    const bool isCurrentWvlDependent = isect.isWvlDependent && (flags & BSDF_TRANSMISSION); 
    
    /* Evaluate the BSDF; wavelength dependency introduces a special case with HWSS */
    if (isWvlDependent || isCurrentWvlDependent) {
      // Zero out all energy except the hero wavelength
      f.zeroAllBinsBut(wvlIdx[0]);
      pathWvlPdf[0] *= bsdfPdf;

      // Evaluate bsdf, pdf for rotated wavelengths
      for (int i = 1; i < nWvls; ++i) {
        const BSDF *_bsdf = &isect.bsdf[isCurrentWvlDependent ? i : 0];
        f[wvlIdx[i]] += _bsdf->f(wo, wi, BSDF_ALL)[wvlIdx[i]];
        pathWvlPdf[i] *= _bsdf->Pdf(wo, wi, BSDF_ALL);
      }
      
      beta *= f * AbsDot(wi, isect.shading.n); // No PDF divide, canceled out by HWSS' MIS weight
    } else {
      beta *= f * AbsDot(wi, isect.shading.n) / bsdfPdf;
    }
    if (beta.IsBlack()) { break; }

    /* Spawn a new ray leading to the next path vertex */
    ray = isect.SpawnRay(wi);

    /* Update ETA scaling for russian roulette */
    if ((flags  & BSDF_SPECULAR) && (flags & BSDF_TRANSMISSION)) {
      Float eta = isect.bsdf->eta;
      etaScale *= (Dot(wo, isect.n) > 0) ? (eta * eta) : 1 / (eta * eta);
    }

    /* Perform russian roulette, possibly terminating the path.
       Factors out radiance scaling due to refraction in rrBeta. */
    Spectrum rrBeta = beta * etaScale;
    if (rrBeta.MaxComponentValue() < rrThreshold && bounces > 3) {
      Float q = std::max((Float).05, 1 - rrBeta.MaxComponentValue());
      if (sampler.Get1D() < q) { break; }
      beta /= 1 - q;
      DCHECK(!std::isinf(beta.y()));
    }

    /* Update status flags */
    isWvlDependent |= isCurrentWvlDependent;
  }

  ReportValue(pathLength, bounces);
  return Lo;
}


HeroPathIntegrator *CreateHeroPathIntegrator(const ParamSet &params,
                                             std::shared_ptr<Sampler> sampler,
                                             std::shared_ptr<const Camera> camera) {
    int maxDepth = params.FindOneInt("maxdepth", 5);
    int np;
    const int *pb = params.FindInt("pixelbounds", &np);
    Bounds2i pixelBounds = camera->film->GetSampleBounds();
    if (pb) {
        if (np != 4)
            Error("Expected four values for \"pixelbounds\" parameter. Got %d.",
                  np);
        else {
            pixelBounds = Intersect(pixelBounds,
                                    Bounds2i{{pb[0], pb[2]}, {pb[1], pb[3]}});
            if (pixelBounds.Area() == 0)
                Error("Degenerate \"pixelbounds\" specified.");
        }
    }
    Float rrThreshold = params.FindOneFloat("rrthreshold", 1.);
    return new HeroPathIntegrator(maxDepth, camera, sampler, pixelBounds, rrThreshold);
}
} // namespace pbrt