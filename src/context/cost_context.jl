abstract type AbstractCostContext end

struct CostContext{T₁ <: AbstractCost, T₂ <: AbstractEstimationContext} <: AbstractCostContext
    cost::T₁
    estimation_context::T₂
end

function (context::CostContext)(Θ::AbstractVector, structure_allotment::AbstractVector,  correspondences::AbstractCorrespondences)
    s = structure_allotment
    [context(Θ[i], extract_subset(correspondences, s[i])) for i = 1:length(s)]
end


# TODO add version which takes covariance matrices as an additional parameter
function (context::CostContext)(𝛉::AbstractVector, correspondences::AbstractCorrespondences)
    N = length(correspondences[1])
    # Construct default covariance matrices (identity)
    Λ = [SMatrix{2,2,Float64,4}(1, 0, 0 ,1) for n = 1:N]
    Λ′ = [SMatrix{2,2,Float64,4}(1, 0, 0 ,1) for n = 1:N]
    covariance_matrices = Correspondences(tuple(Λ, Λ′))
    evaluate_cost(context.cost, context.estimation_context, 𝛉, correspondences, covariance_matrices)
end

function evaluate_cost(cost::ApproximateMaximumLikelihood, estimation_context::EstimateHomographyContext, 𝛉::AbstractVector, correspondences::AbstractCorrespondences, covariance_matrices::AbstractCorrespondences)
    ℳ = correspondences[1]
    ℳ′ = correspondences[2]
    Λ = covariance_matrices[1]
    Λ′ = covariance_matrices[2]
    Jₐₘₗ = 0.0
    N = length(ℳ)
    𝚲ₙ = @MMatrix zeros(4,4)
    𝐞₁ = @SVector [1.0, 0.0, 0.0]
    𝐞₂ = @SVector [0.0, 1.0, 0.0]
    𝐞₁ₓ = vec2antisym(𝐞₁)
    𝐞₂ₓ = vec2antisym(𝐞₂)
    𝐈₃₂ = @SMatrix [1.0  0.0 ; 0.0 1.0 ; 0.0 0.0]
    𝐈₂ = @SMatrix  [1.0  0.0 ; 0.0 1.0]
    index = SVector(1,2)
    @inbounds for n = 1:N
        𝚲ₙ[1:2,1:2] .=  Λ[n][index,index]
        𝚲ₙ[3:4,3:4] .=  Λ′[n][index,index]
        𝐦 = hom(ℳ[n])
        𝐦ʹ= hom(ℳ′[n])
        𝐦ʹₓ = vec2antisym(𝐦ʹ)
        𝐔ₙ = (-𝐦 ⊗ 𝐦ʹₓ)
        𝐕ₙ = 𝐔ₙ * 𝐈₃₂
        ∂ₓ𝐯ₙ = -hcat(vec((𝐞₁ ⊗ 𝐦ʹₓ)*𝐈₃₂), vec((𝐞₂ ⊗ 𝐦ʹₓ)*𝐈₃₂), vec((𝐦 ⊗ 𝐞₁ₓ)*𝐈₃₂), vec((𝐦 ⊗ 𝐞₂ₓ)*𝐈₃₂))
        𝐁ₙ =  ∂ₓ𝐯ₙ * 𝚲ₙ * ∂ₓ𝐯ₙ'
        𝚺ₙ = (𝐈₂ ⊗ 𝛉') * 𝐁ₙ * (𝐈₂ ⊗ 𝛉)
        𝚺ₙ⁻¹ = inv(𝚺ₙ)
        Jₐₘₗ +=  𝛉' * 𝐕ₙ * 𝚺ₙ⁻¹ * 𝐕ₙ' * 𝛉
    end
    Jₐₘₗ
end

# Half the gradient of the approximate maximum likelihood cost function.
# Used throughout the literature on the Fundamental Numerical Scheme.
function evaluate_𝐗(cost::ApproximateMaximumLikelihood, estimation_context::EstimateHomographyContext, 𝛉::AbstractVector, correspondences::AbstractCorrespondences, covariance_matrices::AbstractCorrespondences)
    ℳ = correspondences[1]
    ℳ′ = correspondences[2]
    Λ = covariance_matrices[1]
    Λ′ = covariance_matrices[2]
    N = length(ℳ)
    𝚲ₙ = @MMatrix zeros(4,4)
    𝐞₁ = @SVector [1.0, 0.0, 0.0]
    𝐞₂ = @SVector [0.0, 1.0, 0.0]
    𝐞₁ₓ = vec2antisym(𝐞₁)
    𝐞₂ₓ = vec2antisym(𝐞₂)
    𝐍 = @SMatrix zeros(9,9)
    𝐌 = @SMatrix zeros(9,9)
    𝐈₃₂ = @SMatrix [1.0  0.0 ; 0.0 1.0 ; 0.0 0.0]
    𝐈₂ = @SMatrix  [1.0  0.0 ; 0.0 1.0]
    𝐈₉ = SMatrix{9,9}(1.0I)
    index = SVector(1,2)
    @inbounds for n = 1:N
        𝚲ₙ[1:2,1:2] .=  Λ[n][index,index]
        𝚲ₙ[3:4,3:4] .=  Λ′[n][index,index]
        𝐦 = hom(ℳ[n])
        𝐦ʹ= hom(ℳ′[n])
        𝐦ʹₓ = vec2antisym(𝐦ʹ)
        𝐔ₙ = (-𝐦 ⊗ 𝐦ʹₓ)
        𝐕ₙ = 𝐔ₙ * 𝐈₃₂
        ∂ₓ𝐯ₙ = -hcat(vec((𝐞₁ ⊗ 𝐦ʹₓ)*𝐈₃₂), vec((𝐞₂ ⊗ 𝐦ʹₓ)*𝐈₃₂), vec((𝐦 ⊗ 𝐞₁ₓ)*𝐈₃₂), vec((𝐦 ⊗ 𝐞₂ₓ)*𝐈₃₂))
        𝐁ₙ =  ∂ₓ𝐯ₙ * 𝚲ₙ * ∂ₓ𝐯ₙ'
        𝚺ₙ = (𝐈₂ ⊗ 𝛉') * 𝐁ₙ * (𝐈₂ ⊗ 𝛉)
        𝚺ₙ⁻¹ = inv(𝚺ₙ)
        𝛈ₙ = 𝚺ₙ⁻¹ * 𝐕ₙ' * 𝛉
        𝐍 = 𝐍 + ((𝛈ₙ' ⊗ 𝐈₉) * 𝐁ₙ * (𝛈ₙ ⊗ 𝐈₉))
        𝐌 = 𝐌 + (𝐕ₙ * 𝚺ₙ⁻¹ * 𝐕ₙ')
    end
    𝐗 = 𝐌 - 𝐍
end
