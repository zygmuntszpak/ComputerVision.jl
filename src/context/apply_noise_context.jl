
Base.@kwdef struct ApplyNoiseContext{T₁ <: AbstractNoiseModel, T₂ <: Integer} <: AbstractContext
    perturb::T₁ = HomogeneousGaussianNoise(0.0)
    replications::T₂ = 1
end

function (context::ApplyNoiseContext)(world::AbstractWorld, camera₁::AbstractCamera, camera₂::AbstractCamera)
    # Obtain the ground-truth correspondences
    aquire = AquireImageContext()
    ℳ = aquire(world, camera₁)
    ℳ′ = aquire(world, camera₂)
    N = context.replications
    [Correspondences(tuple(context.perturb(ℳ), context.perturb(ℳ′))) for n = 1:N]
end
