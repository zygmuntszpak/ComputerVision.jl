abstract type AbstractEstimationContext <: AbstractContext end
struct EstimateHomographyContext <: AbstractEstimationContext end

function (context::EstimateHomographyContext)(data::AbstractCorrespondences, estimator::AbstractProjectiveEstimationAlgorithm)
    H = estimator(data, context)
    #estimate(data, estimator, context)
end

# function estimate(data::AbstractCorrespondences, estimator::DirectLinearTransform, context::EstimateHomographyContext)
#     estimator(data, context)
# end

# TODO add coordinate system information
function (estimator::DirectLinearTransform)(pts₁::AbstractCorrespondences, context::EstimateHomographyContext)::SMatrix{3,3,Float64,9}
    normalize = HartleyNormalizeDataContext(pts₁)
    # Transform data to a Hartley normalised coordinate system.
    pts₂ = normalize(pts₁)
    N = length(pts₂[1])
    if (length(pts₂[1])!= length(pts₂[2]))
          throw(ArgumentError("There should be an equal number of points for each view."))
    end
    𝐀 = moments(pts₂, context)
    λ, h = smallest_eigenpair(Symmetric(𝐀))
    𝐇 = reshape(h,(3,3))
    𝐇 = SMatrix{3,3,Float64,9}(𝐇 / norm(𝐇))
    # Transform estimate back to the original (unnormalised) coordinate system.
    𝒯 = matrices(normalize)
    𝐓 = 𝒯[1]
    𝐓ʹ = 𝒯[2]
    return inv(𝐓ʹ)*𝐇*𝐓
end

function moments(correspondences::AbstractCorrespondences, context::EstimateHomographyContext)
    ℳ = correspondences[1]
    ℳʹ = correspondences[2]
    N = length(ℳ)
    if (N != length(ℳʹ))
           throw(ArgumentError("There should be an equal number of points for each view."))
    end
    𝐀 =  @SMatrix zeros(9,9)
    for n = 1:N
        𝐦  = hom(ℳ[n])
        𝐦ʹ = hom(ℳʹ[n])
        𝐔 = -𝐦 ⊗ vec2antisym(𝐦ʹ)
        𝐀 = 𝐀 + 𝐔*𝐔'
    end
    𝐀/N
end
