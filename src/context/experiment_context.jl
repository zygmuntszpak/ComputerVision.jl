struct ExperimentContext{T₁ <: SyntheticSceneContext, T₂ <: ApplyNoiseContext, T₃ <: AbstractEstimationContext} <: AbstractContext
    scene_context::T₁
    noise_context::T₂
    estimation_context::T₃
end

function (context::ExperimentContext)(algorithm::AbstractProjectiveEstimationAlgorithm)
    scene_context = context.scene_context
    noise_context = context.noise_context
    estimation_context = context.estimation_context

    world, cameras = scene_context()
    camera₁, camera₂ = cameras

    correspondences = noise_context(world, camera₁, camera₂)
    # TODO Partition the estimation problem (one for each planar structure)
    # Determine which correspondences belong to which planar structure

    #K = length(correspondences)
    #[estimation_context(correspondences[k], algorithm) for k = 1:K]
end
