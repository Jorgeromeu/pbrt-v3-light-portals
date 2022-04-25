
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


// materials/dispersive_glass.cpp*
#include "materials/dispersive_glass.h"
#include "spectrum.h"
#include "reflection.h"
#include "paramset.h"
#include "texture.h"
#include "interaction.h"

namespace pbrt {

static const Float lambdaMinSq = sampledLambdaStart * sampledLambdaStart;
static const Float lambdaMaxSq = sampledLambdaEnd * sampledLambdaEnd;

// DispersiveGlassMaterial Method Definitions
void DispersiveGlassMaterial::ComputeScatteringFunctions(SurfaceInteraction *si,
                                               MemoryArena &arena,
                                               TransportMode mode,
                                               bool allowMultipleLobes) const {
    // Perform bump mapping with _bumpMap_, if present
    if (bumpMap) Bump(bumpMap, si);
    Float etaMin = indexMin->Evaluate(*si);
    Float etaMax = indexMax->Evaluate(*si);
    Float urough = uRoughness->Evaluate(*si);
    Float vrough = vRoughness->Evaluate(*si);
    Spectrum R = Kr->Evaluate(*si).Clamp();
    Spectrum T = Kt->Evaluate(*si).Clamp();

    // Compute cauchy's equation on the fly
    const Float cauchyB = (lambdaMinSq * etaMax - lambdaMaxSq * etaMin)
                        / (lambdaMinSq - lambdaMaxSq);
    const Float cauchyC = lambdaMinSq * (etaMax - cauchyB);

    const Float t_eta = cauchyB + cauchyC / (sampledLambdaStart * sampledLambdaEnd);
    // std::cout << "Recomputed ETA: " << t_eta << std::endl;

    // Compute ETA for given wavelength packet
    const Vector4f wvl_etas = Vector4f(cauchyB)
                            + Vector4f(cauchyC)
                            / (si->wvls * si->wvls);

    
    // Initialize different scattering components for computed etas
    si->bsdf = (arena.AllocUndeclared<BSDF>(4));
    new(&si->bsdf[0]) BSDF(*si, wvl_etas[0]);
    new(&si->bsdf[1]) BSDF(*si, wvl_etas[1]);
    new(&si->bsdf[2]) BSDF(*si, wvl_etas[2]);
    new(&si->bsdf[3]) BSDF(*si, wvl_etas[3]);

    // No reflective/transmittive components
    if (R.IsBlack() && T.IsBlack()) return;

    // Interacted with a dispersive material; introduce wavelength depencency
    si->isWvlDependent = true;

    // Set up BSDF for each wavelength
    bool isSpecular = urough == 0 && vrough == 0;
    for (int i = 0; i < 4; ++i) {
        const Float eta = wvl_etas[i];
        if (isSpecular && allowMultipleLobes) {
            si->bsdf[i].Add(
                ARENA_ALLOC(arena, FresnelSpecular)(R, T, 1.f, eta, mode));
        } else {
            if (remapRoughness) {
                urough = TrowbridgeReitzDistribution::RoughnessToAlpha(urough);
                vrough = TrowbridgeReitzDistribution::RoughnessToAlpha(vrough);
            }
            MicrofacetDistribution *distrib =
                isSpecular ? nullptr
                        : ARENA_ALLOC(arena, TrowbridgeReitzDistribution)(
                                urough, vrough);
            if (!R.IsBlack()) {
                Fresnel *fresnel = ARENA_ALLOC(arena, FresnelDielectric)(1.f, eta);
                if (isSpecular)
                    si->bsdf[i].Add(
                        ARENA_ALLOC(arena, SpecularReflection)(R, fresnel));
                else
                    si->bsdf[i].Add(ARENA_ALLOC(arena, MicrofacetReflection)(
                        R, distrib, fresnel));
            }
            if (!T.IsBlack()) {
                if (isSpecular)
                    si->bsdf[i].Add(ARENA_ALLOC(arena, SpecularTransmission)(
                        T, 1.f, eta, mode));
                else
                    si->bsdf[i].Add(ARENA_ALLOC(arena, MicrofacetTransmission)(
                        T, distrib, 1.f, eta, mode));
            }
        }
    }
}

DispersiveGlassMaterial *CreateDispersiveGlassMaterial(const TextureParams &mp) {
    std::shared_ptr<Texture<Spectrum>> Kr =
        mp.GetSpectrumTexture("Kr", Spectrum(1.f));
    std::shared_ptr<Texture<Spectrum>> Kt =
        mp.GetSpectrumTexture("Kt", Spectrum(1.f));
    std::shared_ptr<Texture<Float>> etaMin = mp.GetFloatTextureOrNull("etaMin");
    std::shared_ptr<Texture<Float>> etaMax = mp.GetFloatTextureOrNull("etaMax");
    if (!etaMin) etaMin = mp.GetFloatTexture("indexMin", 1.5f);
    if (!etaMax) etaMax = mp.GetFloatTexture("indexMax", 1.5f);
    std::shared_ptr<Texture<Float>> roughu =
        mp.GetFloatTexture("uroughness", 0.f);
    std::shared_ptr<Texture<Float>> roughv =
        mp.GetFloatTexture("vroughness", 0.f);
    std::shared_ptr<Texture<Float>> bumpMap =
        mp.GetFloatTextureOrNull("bumpmap");
    bool remapRoughness = mp.FindBool("remaproughness", false);
    return new DispersiveGlassMaterial(Kr, Kt, roughu, roughv, etaMin, etaMax, bumpMap,
                             remapRoughness);
}

}  // namespace pbrt
