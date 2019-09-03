abstract type AbstractUncertaintyGauge end
abstract type AbstractProjectiveEntityUncertainty end

# struct HomographyVectorUncertainty <: AbstractProjectiveEntityUncertainty end
# struct FundamentalVectorUncertainty <: AbstractProjectiveEntityUncertainty end
#
# struct UnitNormGauge{T <: AbstractProjectiveEntityUncertainty} <: AbstractUncertaintyGauge
#     entity::T
# end

struct UnitNormGauge <: AbstractUncertaintyGauge end

abstract type AbstractUncertaintyContext <: AbstractContext end
Base.@kwdef struct UncertaintyContext{T₁ <: AbstractUncertaintyGauge, T₂ <: AbstractProjectiveEstimationAlgorithm} <: AbstractUncertaintyContext
    gauge::T₁ = UnitNormGauge()
    algorithm::T₂ = ProjectiveEstimationAlgorithm(; objective = AlgebraicLeastSquares(), solver = DirectLinearTransform())
end

function (context::UncertaintyContext)(𝒫::Vector{<: ProjectiveEntity}, structure_allotment::AbstractVector, data::AbstractCorrespondences, data_covariances::AbstractCorrespondences)
    s = structure_allotment
    [evaluate_uncertainty(𝒫[i], context.gauge, context.algorithm, extract_subset(data, s[i]),  extract_subset(data_covariances, s[i])) for i = 1:length(s)]
    #evaluate_uncertainty(𝒫, context.gauge, context.algorithm, data, data_covariances)
end



function (context::UncertaintyContext)(𝒫::ProjectiveEntity, data::AbstractCorrespondences, data_covariances::AbstractCorrespondences)
    evaluate_uncertainty(𝒫, context.gauge, context.algorithm, data, data_covariances)
end

# TODO Tighten the dispatch on the algorithm!
function evaluate_uncertainty(ℋ::HomographyMatrix, gauge::UnitNormGauge, algorithm::AbstractProjectiveEstimationAlgorithm, data::AbstractCorrespondences, data_covariances::AbstractCorrespondences)
    # Transform estimate, data and covariance matrices to the normalized coordinate system.
    𝛉 = vec(matrix(ℋ))
    𝛉 = 𝛉 / norm(𝛉)
    normalize = HartleyNormalizeDataContext(data)
    normalized_data = normalize(data)
    ℳ = normalized_data[1]
    ℳ′ = normalized_data[2]

    𝒯 = matrices(normalize)
    𝐓 = 𝒯[1]
    𝐓ʹ = 𝒯[2]
    Λ₁ = transform_covariance(data_covariances[1], 𝐓)
    Λ₂ = transform_covariance(data_covariances[2], 𝐓ʹ)

    # Map estimate to the normalized coordinate system.
    𝛉₁ = (inv(𝐓') ⊗ 𝐓ʹ) * 𝛉
    𝛉₁ = 𝛉₁ / norm(𝛉₁)

    𝚲 = evaluate_homography_uncertainty(𝛉₁, gauge, algorithm.solver, ℳ, ℳ′, Λ₁, Λ₂)

    𝛉₀ = (𝐓' ⊗ inv(𝐓ʹ)) * 𝛉₁
    𝛉₀ = 𝛉₀ / norm(𝛉₀)

    # Jacobian of the unit normalisation transformation: 𝛉 / norm(𝛉)
    ∂𝛉 = (1/norm(𝛉₀)) * (Matrix{Float64}(I, 9, 9) - ((𝛉₀*𝛉₀') / norm(𝛉₀)^2) )
    F = ∂𝛉*((𝐓' ⊗ inv(𝐓ʹ)))
    F * 𝚲 * F'
end

# function (:UncertaintyContext)(ℋ::HomographyMatrix, data::AbstractCorrespondences, data_covariances::AbstractCorrespondences)
#     # Add decision to normalise or not normalise data
#     ℳ, ℳ′ = data
#     Λ₁, Λ₂ = data_covariances
#     #TODO
# end

function evaluate_homography_uncertainty(𝛉::AbstractVector, gauge::UnitNormGauge, solver::FundamentalNumericalScheme, ℳ::AbstractVector, ℳ′::AbstractVector, Λ::AbstractVector, Λʹ::AbstractVector)
    N = length(ℳ)
    𝐈₉ = SMatrix{9,9}(1.0I)
    𝐈₃₂ = @SMatrix [1.0  0.0 ; 0.0 1.0 ; 0.0 0.0]
    𝐈₂ = @SMatrix  [1.0  0.0 ; 0.0 1.0]
    𝚲ₙ = @MMatrix zeros(4,4)
    𝐞₁ = @SMatrix [1.0; 0.0; 0.0]
    𝐞₂ = @SMatrix [0.0; 1.0; 0.0]
    𝐞₁ₓ = vec2antisym(𝐞₁)
    𝐞₂ₓ = vec2antisym(𝐞₂)
    index = SVector(1,2)
    𝐌 = fill(0.0,(9,9))
    for n = 1:N
        𝚲ₙ[1:2,1:2] .=  Λ[n][index,index]
        𝚲ₙ[3:4,3:4] .=  Λʹ[n][index,index]
        𝐦 = hom(ℳ[n])
        𝐦ʹ= hom(ℳ′[n])
        𝐦ʹₓ = vec2antisym(𝐦ʹ)
        𝐔ₙ = -𝐦 ⊗ 𝐦ʹₓ
        𝐕ₙ = 𝐔ₙ * 𝐈₃₂
        ∂ₓ𝐯ₙ = -hcat(vec((𝐞₁ ⊗ 𝐦ʹₓ)*𝐈₃₂), vec((𝐞₂ ⊗ 𝐦ʹₓ)*𝐈₃₂), vec((𝐦 ⊗ 𝐞₁ₓ)*𝐈₃₂), vec((𝐦 ⊗ 𝐞₂ₓ)*𝐈₃₂))
        𝐁ₙ =  ∂ₓ𝐯ₙ * 𝚲ₙ * ∂ₓ𝐯ₙ'
        𝚺ₙ = (𝐈₂ ⊗ 𝛉') * 𝐁ₙ * (𝐈₂ ⊗ 𝛉)
        𝚺ₙ⁻¹ = inv(𝚺ₙ)
        𝐌 = 𝐌 + (𝐕ₙ * 𝚺ₙ⁻¹ * 𝐕ₙ')
    end
    d = length(𝛉)
    𝐏 = Matrix{Float64}(I, d, d) - norm(𝛉)^-2 * (𝛉*𝛉') # TODO StaticArray?
    U,S,V = svd(𝐌)
    S = SizedArray{Tuple{9}}(S)
    for i = 1:d-1
        S[i] = 1/S[i]
    end
    S[d] = 0.0
    𝐌⁻¹ = U*Diagonal(S)*V'
    𝐏 * 𝐌⁻¹ * 𝐏
end

function evaluate_homography_uncertainty(𝛉::AbstractVector, gauge::UnitNormGauge, solver::DirectLinearTransform, ℳ::AbstractVector, ℳ′::AbstractVector, Λ::AbstractVector, Λʹ::AbstractVector)
    N = length(ℳ)
    𝐈₉ = SMatrix{9,9}(1.0I)
    𝐈₃₂ = @SMatrix [1.0  0.0 ; 0.0 1.0 ; 0.0 0.0]
    𝐈₂ = @SMatrix  [1.0  0.0 ; 0.0 1.0]
    𝚲ₙ = @MMatrix zeros(4,4)
    𝐞₁ = @SMatrix [1.0; 0.0; 0.0]
    𝐞₂ = @SMatrix [0.0; 1.0; 0.0]
    𝐞₁ₓ = vec2antisym(𝐞₁)
    𝐞₂ₓ = vec2antisym(𝐞₂)
    index = SVector(1,2)
    𝐌 = fill(0.0,(9,9))
    𝐃 = fill(0.0,(9,9))
    for n = 1:N
        𝚲ₙ[1:2,1:2] .=  Λ[n][index,index]
        𝚲ₙ[3:4,3:4] .=  Λʹ[n][index,index]
        𝐦 = hom(ℳ[n])
        𝐦ʹ= hom(ℳ′[n])
        𝐦ʹₓ = vec2antisym(𝐦ʹ)
        𝐔ₙ = -𝐦 ⊗ 𝐦ʹₓ
        𝐕ₙ = 𝐔ₙ * 𝐈₃₂
        ∂ₓ𝐯ₙ = -hcat(vec((𝐞₁ ⊗ 𝐦ʹₓ)*𝐈₃₂), vec((𝐞₂ ⊗ 𝐦ʹₓ)*𝐈₃₂), vec((𝐦 ⊗ 𝐞₁ₓ)*𝐈₃₂), vec((𝐦 ⊗ 𝐞₂ₓ)*𝐈₃₂))
        𝐁ₙ =  ∂ₓ𝐯ₙ * 𝚲ₙ * ∂ₓ𝐯ₙ'
        𝚺ₙ = (𝐈₂ ⊗ 𝛉') * 𝐁ₙ * (𝐈₂ ⊗ 𝛉)
        𝚺ₙ⁻¹ = inv(𝚺ₙ)
        # TODO Verify veracity of M
        𝐌 = 𝐌 + (𝐕ₙ * 𝚺ₙ⁻¹ * 𝐕ₙ')
        𝐃 = 𝐃 + (𝐕ₙ * 𝚺ₙ * 𝐕ₙ')
    end
    d = length(𝛉)
    𝐏 = Matrix{Float64}(I, d, d) - norm(𝛉)^-2 * (𝛉*𝛉') # TODO StaticArray?
    U,S,V = svd(𝐌)
    S = SizedArray{Tuple{9}}(S)
    for i = 1:d-1
        S[i] = 1/S[i]
    end
    S[d] = 0.0
    𝐌⁻¹ = U*Diagonal(S)*V'
    𝚲₀ = 𝐌⁻¹ * 𝐃 *  𝐌⁻¹
    𝐏 * 𝚲₀ * 𝐏
end

# TODO Fix dispatch here (change to solver)
function evaluate_fundamental_matrix_uncertainty(gauge::UnitNormGauge, algorithm::FundamentalNumericalScheme, ℱ::FundamentalMatrix, data::AbstractCorrespondences, data_covariances::AbstractCorrespondences)
    # TODO Switch based on whether the estimation is done in a normalised or unnormalised coordinate system.
    𝛉 = vec(matrix(ℱ))
    𝛉 = 𝛉 / norm(𝛉)
    # TODO Fix the splatting of data...
    ℳ, ℳ′ = data
    Λ₁, Λ₂ = data_covariances
    N = length(ℳ)
    𝚲ₙ = @MMatrix zeros(4,4)
    𝐞₁ = @SMatrix [1.0; 0.0; 0.0]
    𝐞₂ = @SMatrix [0.0; 1.0; 0.0]
    index = SVector(1,2)
    𝐌 = fill(0.0,(9,9))
    for n = 1:N
        𝚲ₙ[1:2,1:2] .=  Λ₁[n][index,index]
        𝚲ₙ[3:4,3:4] .=  Λ₂[n][index,index]
        𝐦 = ℳ[n]
        𝐦ʹ= ℳʹ[n]
        𝐔ₙ = (𝐦 ⊗ 𝐦ʹ)
        𝐀 = 𝐔ₙ*𝐔ₙ'
        ∂ₓ𝐮ₙ =  [(𝐞₁ ⊗ 𝐦ʹ) (𝐞₂ ⊗ 𝐦ʹ) (𝐦 ⊗ 𝐞₁) (𝐦 ⊗ 𝐞₂)]
        𝐁ₙ =  ∂ₓ𝐮ₙ * 𝚲ₙ * ∂ₓ𝐮ₙ'
        𝐌 = 𝐌 + 𝐀/(𝛉'*𝐁ₙ*𝛉)
    end
     d = length(𝛉)
     𝐏 = Matrix{Float64}(I, d, d) - norm(𝛉)^-2 * (𝛉*𝛉')
     U,S,V = svd(𝐌)
     S = SizedArray{Tuple{9}}(S)
     for i = 1:d-1
         S[i] = 1/S[i]
     end
     S[d] = 0.0
     𝐌⁻¹ = U*diagm(S)*V'
     𝐏 * 𝐌⁻¹ * 𝐏
end

function transform_covariance(Λ::AbstractVector, 𝐓::AbstractMatrix)
    Λ₂ = map(Λ) do 𝚲
       # Lift the covariance matrix so that it correspond to homeogenous 2D coordinates.
       # This way the requisite transformation can be computed by multiply with a matrix 𝐓.
       𝚲₀ =  hcat(𝚲, SVector(0.0, 0.0))
       𝚲₁ = vcat(𝚲₀, transpose(SVector(0.0, 0.0, 0.0)))
       𝚲₂ =  (𝐓 * 𝚲₁ * 𝐓')
       𝚲′ = SMatrix{2,2,Float64,4}(𝚲₂[1], 𝚲₂[2], 𝚲₂[4], 𝚲₂[5])
    end
end
