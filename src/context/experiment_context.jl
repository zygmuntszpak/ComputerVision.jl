struct ExperimentContext{T₁ <: SyntheticScene, T₂ <: AbstractEstimationContext, T₃ <: AbstractVector} <: AbstractContext
    synthetic_scene::T₁
    estimation_context::T₂
    correspondences::T₃
end

# TODO Add constructor which automatically builds the requisite correspondences
ExperimentContext(synthetic_scene::SyntheticScene, estimation_context::AbstractEstimationContext, noise_context::ApplyNoiseContext) = ExperimentContext(synthetic_scene, estimation_context, gather_correspondences(synthetic_scene, noise_context, estimation_context))

function gather_correspondences(synthetic_scene::SyntheticScene, noise_context::ApplyNoiseContext, estimation_context::AbstractEstimationContext)
    world = get_world(synthetic_scene)
    cameras = get_cameras(synthetic_scene)
    camera₁ = cameras[1]
    camera₂ = cameras[2]
    correspondences = noise_context(world, camera₁, camera₂)
end

function (context::ExperimentContext)(algorithm::AbstractProjectiveEstimationAlgorithm)
    synthetic_scene = context.synthetic_scene
    estimation_context = context.estimation_context
    correspondences = context.correspondences
    world = get_world(synthetic_scene)
    # cameras = get_cameras(synthetic_scene)
    # camera₁ = cameras[1]
    # camera₂ = cameras[2]

    span = [get_interval(world.groups[i]) for i = 1:length(world.groups)]

    estimates, runtimes = execute_trials(correspondences, span, algorithm, estimation_context)

    #estimates, runtimes = administer_experiment(world, estimation_context, algorithm)

    return estimates, runtimes


    # TODO Partition the estimation problem (one for each planar structure)
    # Determine which correspondences belong to which planar structure

    #K = length(correspondences)
    #[estimation_context(correspondences[k], algorithm) for k = 1:K]
end


# function administer_experiment(world::AbstractWorld, estimation_context::EstimateHomographyContext, algorithm::AbstractProjectiveEstimationAlgorithm)
#     #R = noise_context.replications
#     #points_per_region = length(world.planes)
#     #correspondences = noise_context(world, camera₁, camera₂)
#     correspondences = context.correspondences
#
#     # Determine which set of points correspond to which planar structure
#     #cummulative = OffsetArray(cumsum(vcat([0],points_per_region)), -1)
#     #span = [ (cummulative[p-1]+1):cummulative[p] for p = 1:P ]
#     # TODO span can be extracted from groups stored in world
#     span = [get_interval(world.groups[i]) for i = 1:length(world.groups)]
#
#     estimates, runtimes = execute_trials(correspondences, span, algorithm, estimation_context)
# end


function execute_trials(correspondences::AbstractVector, span::AbstractVector, algorithm::AbstractProjectiveEstimationAlgorithm, estimation_context::EstimateHomographyContext)

    # Intialise data structure to store the homography estimates and the runtimes.
    replications = length(correspondences)
    estimates = Vector{Vector{SArray{Tuple{3,3},Float64,2,9}}}(undef, replications)
    runtimes = Vector{Float64}(undef, replications)

    for (k, correspondence) in enumerate(correspondences)
        ℳ = correspondence[1]
        ℳ′ = correspondence[2]
        output, timing_info = @timed [algorithm(Correspondences(tuple(ℳ[span[p]], ℳ′[span[p]])), estimation_context)  for p = 1:length(span)]
        estimates[k] =  output
        runtimes[k] = timing_info
    end
    estimates, runtimes
end

# TODO rethink dispatch here to make it more/less specific
function execute_trials(correspondences::AbstractVector, span::AbstractVector, algorithm::ProjectiveEstimationAlgorithm{<: AbstractCost,  <: AbstractConstrainedProjectiveOptimizationScheme}, estimation_context::EstimateHomographyContext)
    # Intialise data structure to store the homography estimates and the runtimes.
    replications = length(correspondences)
    estimates = Vector{Vector{SArray{Tuple{3,3},Float64,2,9}}}(undef, replications)
    runtimes = Vector{Float64}(undef, replications)

    for (k, correspondence) in enumerate(correspondences)
        # This algorithm estimates each homography jointly.
        output, timing_info = @timed algorithm(correspondence, span, estimation_context)
        estimates[k] =  output
        runtimes[k] = timing_info
    end
    estimates, runtimes
end

# estimates = map(correspondences) do correspondence
#     # For each plane, extract the relevant corresponding points and estimate the concomitant homography matrix
#     ℳ = correspondence[1]
#     ℳ′ = correspondence[2]
#     [algorithm(Correspondences(tuple(ℳ[span[p]], ℳ′[span[p]])), estimation_context)  for p = 1:P]
# end
