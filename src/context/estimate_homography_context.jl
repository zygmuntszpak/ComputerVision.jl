abstract type AbstractEstimationContext <: AbstractContext end
struct EstimateHomographyContext <: AbstractEstimationContext end

abstract type AbstractFitModel end
abstract type AbstractFitJacobian end

Base.@kwdef struct HomographyFitModel{T₁ <: AbstractCost} <: AbstractFitModel
    cost::T₁ = ReprojectionError()
end
Base.@kwdef struct HomographyFitJacobian{T₁ <: AbstractCost} <: AbstractFitJacobian
    cost::T₁ = ReprojectionError()
end

Base.@kwdef struct ImplicitConsistentHomographyFitModel{T₁ <: AbstractCost, T₂ <: AbstractVector{<: UnitRange}} <: AbstractFitModel
    cost::T₁ = ReprojectionError()
    structure_allotment::T₂
end

Base.@kwdef struct ImplicitConsistentHomographyFitJacobian{T₁ <: AbstractCost, T₂  <: AbstractVector{<: UnitRange}} <: AbstractFitModel
    cost::T₁ = ReprojectionError()
    structure_allotment::T₂
end

Base.@kwdef struct ExplicitConsistentHomographyFitModel{T₁ <: AbstractCost, T₂ <:  AbstractVector{<: UnitRange}} <: AbstractFitModel
    cost::T₁ = Mahalanobis()
    structure_allotment::T₂
end

function (context::EstimateHomographyContext)(data::AbstractCorrespondences, estimator::AbstractProjectiveEstimationAlgorithm)
    H = estimator(data, context)
    #estimate(data, estimator, context)
end

function (context::EstimateHomographyContext)(data::AbstractCorrespondences, structure_allotment::AbstractVector, estimator::AbstractProjectiveEstimationAlgorithm)
    H = estimator(data, structure_allotment, context)
    #estimate(data, estimator, context)
end

# function estimate(data::AbstractCorrespondences, estimator::DirectLinearTransform, context::EstimateHomographyContext)
#     estimator(data, context)
# end

# # TODO add coordinate system information
# function (estimator::DirectLinearTransform)(pts₁::AbstractCorrespondences, context::EstimateHomographyContext)::SMatrix{3,3,Float64,9}
#     normalize = HartleyNormalizeDataContext(pts₁)
#     # Transform data to a Hartley normalised coordinate system.
#     pts₂ = normalize(pts₁)
#     N = length(pts₂[1])
#     if (length(pts₂[1])!= length(pts₂[2]))
#           throw(ArgumentError("There should be an equal number of points for each view."))
#     end
#     𝐀 = moments(pts₂, context)
#     λ, h = smallest_eigenpair(Symmetric(𝐀))
#     𝐇 = reshape(h,(3,3))
#     𝐇 = SMatrix{3,3,Float64,9}(𝐇 / norm(𝐇))
#     # Transform estimate back to the original (unnormalised) coordinate system.
#     𝒯 = matrices(normalize)
#     𝐓 = 𝒯[1]
#     𝐓ʹ = 𝒯[2]
#     return inv(𝐓ʹ)*𝐇*𝐓
# end

# TODO add coordinate system information
function (estimator::ProjectiveEstimationAlgorithm{<: AlgebraicLeastSquares, <:DirectLinearTransform})(pts₁::AbstractCorrespondences, context::EstimateHomographyContext)::SMatrix{3,3,Float64,9}
    if (length(pts₁[1])!= length(pts₁[2]))
          throw(ArgumentError("There should be an equal number of points for each view."))
    elseif estimator.apply_normalisation
        normalize = HartleyNormalizeDataContext(pts₁)
        # Transform data to a Hartley normalised coordinate system.
        𝐇 = direct_linear_transform(normalize(pts₁), context)
        # Transform estimate back to the original (unnormalised) coordinate system.
        𝒯 = matrices(normalize)
        𝐓 = 𝒯[1]
        𝐓ʹ = 𝒯[2]
        return inv(𝐓ʹ)*𝐇*𝐓
    else
        𝐇 = direct_linear_transform(pts₁, context)
        return 𝐇
    end
end

function direct_linear_transform(pts::AbstractCorrespondences, context::EstimateHomographyContext)
    𝐀 = moments(pts, context)
    λ, h = smallest_eigenpair(Symmetric(𝐀))
    𝐇 = reshape(h,(3,3))
    𝐇 = SMatrix{3,3,Float64,9}(𝐇 / norm(𝐇))
end

function (estimator::ProjectiveEstimationAlgorithm{<: ApproximateMaximumLikelihood,
                                                   <: FundamentalNumericalScheme{
                                                   <: AbstractVariant,
                                                   <: Number,
                                                   <: ProjectiveEstimationAlgorithm{
                                                   <: AlgebraicLeastSquares,
                                                   <: DirectLinearTransform         }
                                                                                 }
                                                   })(correspondences::AbstractCorrespondences,
                                                      context::EstimateHomographyContext)::SMatrix{3,3,Float64,9}
    N = length(correspondences[1])
    # Construct default covariance matrices for the corresponding points.
    Λ = [SMatrix{2,2,Float64,4}(1, 0, 0 ,1) for n = 1:N]
    Λ′ = [SMatrix{2,2,Float64,4}(1, 0, 0 ,1) for n = 1:N]
    covariance_matrices = Correspondences(tuple(Λ, Λ′))
    estimator(correspondences, covariance_matrices, context)
end

function (estimator::ProjectiveEstimationAlgorithm{<: ApproximateMaximumLikelihood,
                                                   <: FundamentalNumericalScheme
                                                   })(correspondences::AbstractCorrespondences,
                                                      covariance_matrices::AbstractCorrespondences,
                                                      estimation_context::EstimateHomographyContext)::SMatrix{3,3,Float64,9}
    solver = estimator.solver
    if (length(correspondences[1])!= length(correspondences[2]))
          throw(ArgumentError("There should be an equal number of points for each view."))
    elseif estimator.apply_normalisation
        # Determine an initial estimate
        𝐇 = estimation_context(correspondences, solver.seed_estimator)
        𝛉 = vec(𝐇)
        𝛉 = 𝛉 / norm(𝛉)
        # Transform the initial estimate, correspondences and covariance matrices
        # to the normalized coordinate system.
        normalize = HartleyNormalizeDataContext(correspondences)
        normalized_correspondences = normalize(correspondences)
        ℳ = normalized_correspondences[1]
        ℳ′ = normalized_correspondences[2]
        # Transform estimate, data and covariance matrices to the normalized coordinate system.
        # Construct default covariance matrices (identity)
        𝒯 = matrices(normalize)
        𝐓 = 𝒯[1]
        𝐓ʹ = 𝒯[2]
        Λ = transform_covariance(covariance_matrices[1], 𝐓)
        Λ′ = transform_covariance(covariance_matrices[2], 𝐓ʹ)
        normalized_covariance_matrices = Correspondences(tuple(Λ, Λ′))
        # Map initial estimate to the normalized coordinate system.
        𝛉₁ = (inv(𝐓') ⊗ 𝐓ʹ) * 𝛉
        𝛉₁ = 𝛉₁ / norm(𝛉₁)
        # Find the minimizer of the approximate maximum likelihood cost function.
        𝛉⁺ = solver(𝛉₁, normalized_correspondences, normalized_covariance_matrices, estimation_context, estimator.objective)
        #𝛉⁺ = 𝛉₁
        # Transform AML estimate back to unnormalised coordinate system
        𝛉₀ = (𝐓' ⊗ inv(𝐓ʹ)) * 𝛉⁺
        𝛉₀ = 𝛉₀ / norm(𝛉₀)
        𝐇⁺ = SMatrix{3,3,Float64,9}(reshape(𝛉₀, (3, 3)))
        return 𝐇⁺
    else
        # Determine an initial estimate
        𝐇 = estimation_context(correspondences, estimator.seed_estimator)
        𝛉 = vec(𝐇)
        𝛉 = 𝛉 / norm(𝛉)
        𝛉⁺ = solver(𝛉, correspondences, covariance_matrices, estimation_context, estimator.objective)
        𝐇⁺ = SMatrix{3,3,Float64,9}(reshape(𝛉⁺, (3, 3)))
        return 𝐇⁺
    end
end

# TODO move to separate estimation.jl file
function (fns::FundamentalNumericalScheme{<: UndampedVariant, <: Number})(𝛉::AbstractVector, correspondences::AbstractCorrespondences, covariance_matrices::AbstractCorrespondences, estimation_context::AbstractEstimationContext, objective::ApproximateMaximumLikelihood)
    I = fns.max_iter
    # TODO add stopping critria based on parameter difference
    for i = 1:I
        𝐗 = evaluate_𝐗(objective, estimation_context, 𝛉, correspondences, covariance_matrices)
        λ, 𝛉⁺ = smallest_eigenpair(Symmetric(𝐗))
        # TODO check type stability
        𝛉 = vec(𝛉⁺)
    end
    return 𝛉
end

# function fundamental_numerical_scheme(𝛉::AbstractVector, correspondences::AbstractCorrespondences, covariance_matrices::AbstractCorrespondences, estimation_context::EstimateHomographyContext, objective::ApproximateMaximumLikelihood)
#     # TODO construct X
#     # Extract stopping criterion from "estimator"
#     max_iter =
#     ℳ = correspondences[1]
#     ℳʹ = correspondences[2]
#     Λ = covariance_matrices[1]
#     Λ′ = covariance_matrices[2]
#     cost_context = CostContex(objective, context)
#     𝐗 = evaluate_𝐗(objective, estimation_context, 𝛉, correspondencs, covariance_matrices)
# end

#evaluate_𝐗(cost::ApproximateMaximumLikelihood, estimation_context::EstimateHomographyContext, 𝛉::AbstractVector, correspondences::AbstractCorrespondences, covariance_matrices::AbstractCorrespondences)


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

function (estimator::ProjectiveEstimationAlgorithm{<: ReprojectionError,
                                                   <: BundleAdjustment{
                                                   <: ProjectiveEstimationAlgorithm{
                                                   <: AlgebraicLeastSquares,
                                                   <: DirectLinearTransform         }
                                                                       }
                                                    })(correspondences::AbstractCorrespondences,
                                                      context::EstimateHomographyContext)::SMatrix{3,3,Float64,9}
   solver = estimator.solver
   seed = context(correspondences, solver.seed_estimator)
   ℳ = correspondences[1]
   ℳʹ = correspondences[2]
   N = length(ℳ)
   𝐈 = SMatrix{3,3}(1.0I)
   # Construct a length-(9+2*N) vector consisting of the homography matrix
   # (the first nine dimensions), as well as N two-dimensional points in the
   # first view (the remaining dimensions).
   𝛉 = pack(HomographyMatrix(seed), ℳ)

   index₁ = SVector(1,2)
   index₂ = SVector(3,4)
   pts = Matrix{Float64}(undef,4,N)
   for n = 1:N
       pts[index₁,n] = ℳ[n][index₁]
       pts[index₂,n] = ℳʹ[n][index₁]
   end

   cost_type = ReprojectionError()
   fit = curve_fit(HomographyFitModel(cost_type), HomographyFitJacobian(cost_type),  𝐈, reshape(reinterpret(Float64,vec(pts)),(4*N,)) , 𝛉; show_trace = false)
   #fit = curve_fit(model_homography, 𝐈, reshape(reinterpret(Float64,vec(pts)),(4*N,)) , 𝛉; show_trace = false)

   𝐇₊  = SMatrix{3,3,Float64,9}(reshape(fit.param[1:9],(3,3)))
   𝐇₊
end

function (::HomographyFitModel{<:ReprojectionError})(𝐈,𝛉)
    # Nine parameters for the projection matrix, and 2 parameters per 2D point.
    N = Int((length(𝛉) - 9) / 2)
    index₁ = SVector(1,2)
    index₂ = SVector(3,4)
    reprojections = Matrix{Float64}(undef,4,N)
    𝛉v = @view 𝛉[1:9]
    𝐇 = SMatrix{3,3,Float64,9}(reshape(𝛉v,(3,3)))
    i = 10
    for n = 1:N
        # Extract 2D point and convert to homogeneous coordinates
        𝐦 = hom(SVector{2,Float64}(𝛉[i],𝛉[i+1]))
        reprojections[index₁,n] = hom⁻¹(𝐦)
        reprojections[index₂,n] = hom⁻¹(𝐇 * 𝐦)
        i = i + 2
    end
    reshape(reinterpret(Float64,vec(reprojections)),(4*N,))
end

function (::HomographyFitJacobian{<:ReprojectionError})(𝐈,𝛉)
    # Nine parameters for the homography matrix, and 2 parameters per 2D point.
    N = Int((length(𝛉) - 9) / 2)
    index₁ = SVector(1,2)
    index₂ = SVector(3,4)
    reprojections = Matrix{Float64}(undef,4,N)
    𝛉v = @view 𝛉[1:9]
    𝐇 = SMatrix{3,3,Float64,9}(reshape(𝛉v,(3,3)))
    𝐉 = zeros(4*N,9+2*N)
    # Create a view of the jacobian matrix 𝐉 and reshape it so that
    # it will be more convenient to index into the appropriate entries
    # whilst looping over all of the data points.
    𝐉v = reshape(reinterpret(Float64,𝐉), 4, N, 9+2*N)
    𝐀 = SMatrix{2,3,Float64,6}(1,0,0,1,0,0)
    𝐈₃ = SMatrix{3,3}(1.0I)
    i = 10
    for n = 1:N
        # Extract 3D point and convert to homogeneous coordinates.
        𝐦 = hom(SVector{2,Float64}(𝛉[i], 𝛉[i+1]))

        # Derivative of residual in first and second image w.r.t 2D point in the
        # first image.
        ∂𝐫₁_d𝐦 = 𝐀 * 𝐈₃
        ∂𝐫₂_d𝐦 = 𝐀 * ∂hom⁻¹(𝐇 * 𝐦) * 𝐇

        # Derivative of residual in second image w.r.t homography martix.
        # ∂𝐫₁_d𝐇 is the zero vector.
        ∂𝐫₂_d𝐇 = 𝐀 * ∂hom⁻¹(𝐇  * 𝐦) * (𝐦' ⊗ 𝐈₃)
        𝐉v[index₂,n,1:9] = ∂𝐫₂_d𝐇
        𝐉v[index₁,n,i:i+1] = ∂𝐫₁_d𝐦[:,1:2]
        𝐉v[index₂,n,i:i+1] = ∂𝐫₂_d𝐦[:,1:2]
        i = i + 2
    end
    𝐉
end

# Construct a parameter vector consisting of a homography matrix and 2D points.
function pack(H::HomographyMatrix, ℳ::AbstractArray)
    𝐇 = matrix(H)
    N = length(ℳ)
    𝛉 = Vector{Float64}(undef, 9 + N*2)
    𝛉[1:9] = Array(vec(𝐇))
    i = 10
    for n = 1:N
        𝛉[i:i+1] = ℳ[n][1:2]
        i = i + 2
    end
    𝛉
end

# Construct a parameter vector consisting of latent variables parametrising a set of consistent
# homography matrices, as well as a set of 2D points in the first view.
function pack(L::LatentVariables,  ℳ::AbstractArray)
    𝐥 = variables(L)
    M = length(𝐥)
    N = length(ℳ)
    𝛉 = Vector{Float64}(undef, M + N*2)
    𝛉[1:M] = 𝐥
    i = M + 1
    for n = 1:N
        𝛉[i:i+1] = ℳ[n][1:2]
        i = i + 2
    end
    𝛉
end


# <: AlgebraicLeastSquares,
# <: DirectLinearTransform        }
function (estimator::ProjectiveEstimationAlgorithm{<: Mahalanobis,
                                                   <: ConstrainedMahalanobis{
                                                   <: ProjectiveEstimationAlgorithm{
                                                   <: T₁,
                                                   <: T₂        }
                                                                            }
                                                   })(correspondences::AbstractCorrespondences,
                                                      structure_allotment::AbstractVector,
                                                      estimation_context::EstimateHomographyContext) where {T₁ <: AbstractCost, T₂ <: AbstractProjectiveOptimizationScheme} #::Vector{<:SMatrix{3,3,Float64,9}}

    # Transform data to a Hartley normalised coordinate system.
    normalize = HartleyNormalizeDataContext(correspondences)
    correspondences_normalized = normalize(correspondences)
    ℳ = correspondences_normalized[1]
    ℳʹ = correspondences_normalized[2]
    N = length(ℳ)
    S = length(structure_allotment)
    ℋ = [@SMatrix zeros(3,3) for s = 1:S]
    ℬ =  [@SMatrix zeros(9,9) for s = 1:S]
    cost_context = CostContext(ApproximateMaximumLikelihood(), estimation_context)
    uncertainty_context = UncertaintyContext(UnitNormGauge(), ProjectiveEstimationAlgorithm(; objective = T₁(), solver = T₂()))
    solver = estimator.solver
    for s = 1:S
        𝒪 = ℳ[structure_allotment[s]]
        𝒪′ = ℳʹ[structure_allotment[s]]
        correspondencesₛ = Correspondences(tuple(𝒪, 𝒪′))
        ℋ[s] = estimation_context(correspondencesₛ, solver.seed_estimator)
        𝐇 = ℋ[s]
        𝛉 = vec(𝐇)
        Nₛ = length(𝒪)
        cost = cost_context(𝛉, correspondencesₛ)
        # An estimate of the noise level based on the approximate maximum likelihood cost function.
        σ = sqrt(cost / (Nₛ - 8))
        σ² = σ^2
        # Construct default covariance matrices using the estimated noise level.
        Λ = [SMatrix{2,2,Float64,4}(σ², 0, 0 ,σ²) for n = 1:Nₛ]
        Λ′ = [SMatrix{2,2,Float64,4}(σ², 0, 0 ,σ²) for n = 1:Nₛ]
        covariance_matricesₛ = Correspondences(tuple(Λ, Λ′))
        𝐂 = uncertainty_context(HomographyMatrix(𝐇), correspondencesₛ, covariance_matricesₛ)
        # The information matrix is the pseudo-inverse of the covariance matrix.
        𝚲⁺ = pinv(𝐂)
        # We work with the non-negative definite suare root because we express
        # our cost function as a sum of squares.
        ℬ[s] = SMatrix{9,9,Float64,81}(non_negative_definite_square_root(𝚲⁺))
    end
    # Construct a length-(4I+12) vector consisting of latent variables that
    # enforce consistency between the I homography matrices.
    #𝛈 = pack(HomographyMatrices(HomographyMatrix.(ℋ)))
    𝛈 = initialise_latent_variables(ℋ)
    cost_type = Mahalanobis(ℬ)
    model = ImplicitConsistentHomographyFitModel(cost_type, [1:S])
    jacobian = ImplicitConsistentHomographyFitJacobian(cost_type, structure_allotment)
    𝐈 = SMatrix{3,3}(1.0I)
    # TODO: Implement in-place version of model and Jacobian
    #fit = curve_fit(model,  𝐈, zeros(9*S), 𝛈 ; show_trace = false)
    fit = curve_fit(model, jacobian,  𝐈, zeros(9*S), 𝛈 ; show_trace = false)
    𝛈_est = fit.param

    ℋ′ = matrices(compose(HomographyMatrices, 𝛈_est))
    # Transform estimate back to the original (unnormalised) coordinate system.
    𝒯 = matrices(normalize)
    𝐓 = 𝒯[1]
    𝐓ʹ = 𝒯[2]
    ℋ₊ = map(ℋ′) do 𝐇
          inv(𝐓ʹ)*𝐇*𝐓
    end
    ℋ₊
end

function non_negative_definite_square_root(𝐀::AbstractMatrix)
    U,S,V = svd(𝐀)
    U*Diagonal(sqrt.(S))*V'
end#

function (param::ImplicitConsistentHomographyFitModel{<:Mahalanobis, <: AbstractVector})(𝐈,𝛉)
    # Vector of non-negative square root of covariance matrices.
    ℬ = param.cost.ℬ
    structure_allotment = param.structure_allotment
    I = length(ℬ)
    # TODO Revisit this so that we reduce the unnecessary memory allocations.
    ℋ = matrices(compose(HomographyMatrices, 𝛉))
    residual = zeros(9,I)
    for i = 1:I
        𝛉ᵢ = vec(ℋ[i])
        𝐁ᵢ = ℬ[i]
        residual[:,i] = (𝛉ᵢ' * 𝐁ᵢ) * (norm(𝛉ᵢ)^-1)
    end
    reshape(reinterpret(Float64,vec(residual)),(9*I,))
end

function (param::ImplicitConsistentHomographyFitJacobian{ <:Mahalanobis, <: AbstractVector})(𝐈,𝛈)
    # Vector of non-negative square root of covariance matrices.
    ℬ = param.cost.ℬ
    structure_allotment = param.structure_allotment
    # Total number of homographies.
    K = length(ℬ)
    # Determine the total number of latent variables.
    M = length(𝛈)
    # TODO Revisit this so that we reduce the unnecessary memory allocations.
    ℋ = matrices(compose(HomographyMatrices, 𝛈))
    𝐉 = zeros(9*K, length(𝛈))

    # Create a view of the jacobian matrix 𝐉 and reshape it so that
    # it will be more convenient to index into the appropriate entries
    # whilst looping over all of the data points.
    𝐉v = reshape(reinterpret(Float64,𝐉), 9, K, length(𝛈))
    for k = 1:K
        πₖ = vec(ℋ[k])
        𝐁ₖ = ℬ[k]
        𝐏 = UniformScaling(1) - norm(πₖ)^-2 * (πₖ*πₖ')
        ∂𝐫ₖ_θₖ = norm(πₖ)^-1 * 𝐁ₖ * 𝐏
        ∂𝐫ₖ_d𝐚 = ∂𝐫ₖ_θₖ * ∂ₐ𝛑(𝛈, k)
        ∂𝐫ₖ_d𝐛 = ∂𝐫ₖ_θₖ * ∂ᵦ𝛑(𝛈, k)
        ∂𝐫ₖ_d𝐯 = ∂𝐫ₖ_θₖ * ∂ᵥ𝛑(𝛈, k)
        ∂𝐫ₖ_d𝐰 = ∂𝐫ₖ_θₖ * ∂ₛ𝛑(𝛈, k)
        ∂𝐫ₖ_d𝛈 = hcat(∂𝐫ₖ_d𝐚, ∂𝐫ₖ_d𝐛, ∂𝐫ₖ_d𝐯, ∂𝐫ₖ_d𝐰)
        𝐉v[:, k, 1:M] = ∂𝐫ₖ_d𝛈
    end
    𝐉
end


function (estimator::ProjectiveEstimationAlgorithm{<: ReprojectionError,
                                                   <: ConstrainedBundleAdjustment{
                                                   <: ProjectiveEstimationAlgorithm{
                                                   <: AlgebraicLeastSquares,
                                                   <: DirectLinearTransform        },
                                                   <: ImplicitChojnackiSzpak      }
                                                  })(correspondences::AbstractCorrespondences,
                                                     structure_allotment::AbstractVector,
                                                     context::EstimateHomographyContext)#::Vector{<:SMatrix{3,3,Float64,9}}

   # ℳ = correspondences[1]
   # ℳʹ = correspondences[2]
   solver = estimator.solver
   normalize = HartleyNormalizeDataContext(correspondences)
   # Transform data to a Hartley normalised coordinate system.
   correspondences_normalized = normalize(correspondences)
   ℳ = correspondences_normalized[1]
   ℳʹ = correspondences_normalized[2]
   N = length(ℳ)
   S = length(structure_allotment)
   ℋ = [@SMatrix zeros(3,3) for s = 1:S]
   for s = 1:S
       𝒪 = ℳ[structure_allotment[s]]
       𝒪′ = ℳʹ[structure_allotment[s]]
       ℋ[s] = context(Correspondences(tuple(𝒪, 𝒪′)), solver.seed_estimator)
   end

   𝛈ₕ = initialise_latent_variables(ℋ)
   # Construct a length-((4I+12)+2*N) vector consisting of the I homography matrices
   # (the first 4I+12 dimensions), as well as N two-dimensional points in the
   # first view (the remaining dimensions).
   𝛉 = pack(LatentVariables(𝛈ₕ), ℳ)
   𝐈 = SMatrix{3,3}(1.0I)
   index₁ = SVector(1,2)
   index₂ = SVector(3,4)
   pts = Matrix{Float64}(undef,4,N)
   for n = 1:N
       pts[index₁,n] = ℳ[n][index₁]
       pts[index₂,n] = ℳʹ[n][index₁]
   end
   cost_type = ReprojectionError()
   model = ImplicitConsistentHomographyFitModel(cost_type, structure_allotment)
   jacobian = ImplicitConsistentHomographyFitJacobian(cost_type, structure_allotment)

   # TODO: Implement in-place version of model and Jacobian
   #fit = curve_fit(model,  𝐈, reshape(reinterpret(Float64,vec(pts)),(4*N,)) , 𝛉; show_trace = false)
   fit = curve_fit(model, jacobian, 𝐈, reshape(reinterpret(Float64,vec(pts)),(4*N,)) , 𝛉; show_trace = false)

   𝛈_est = fit.param[1:length(𝛈ₕ)]
   ℋ′ = matrices(compose(HomographyMatrices, fit.param[1:length(𝛈ₕ)]))

   # Transform estimate back to the original (unnormalised) coordinate system.
   𝒯 = matrices(normalize)
   𝐓 = 𝒯[1]
   𝐓ʹ = 𝒯[2]
   ℋ₊ = map(ℋ′) do 𝐇
         inv(𝐓ʹ)*𝐇*𝐓
   end
   ℋ₊
   #fit
end

function (param::ImplicitConsistentHomographyFitModel{ <:ReprojectionError, <: AbstractVector})(𝐈,𝛉)
    structure_allotment = param.structure_allotment
    # The value of the last range tells us the total number of 2D points.
    N = structure_allotment[end][end]
    # Determine the total number of latent variables.
    M = length(𝛉) - N*2
    index₁ = SVector(1,2)
    index₂ = SVector(3,4)
    #reprojections = Matrix{Float64}(undef,4,N)
    reprojections = zeros(4,N)
    𝛉v = @view 𝛉[1:M]
    # TODO Revisit this so that we reduce the unnecessary memory allocations.
    ℋ = matrices(compose(HomographyMatrices, 𝛉v))
    i = M + 1
    for (k, span) in enumerate(structure_allotment)
        𝐇ₖ = ℋ[k]
        #display(𝐇ₖ/ norm(𝐇ₖ))
        for n in span
            # Extract 2D point and convert to homogeneous coordinates
            𝐦 = hom(SVector{2,Float64}(𝛉[i],𝛉[i+1]))
            reprojections[index₁,n] = hom⁻¹(𝐦)
            reprojections[index₂,n] = hom⁻¹(𝐇ₖ  * 𝐦)
            i = i + 2
        end
    end
    reshape(reinterpret(Float64,vec(reprojections)),(4*N,))
end


function (param::ImplicitConsistentHomographyFitJacobian{ <:ReprojectionError, <: AbstractVector})(𝐈,𝛉)
    structure_allotment = param.structure_allotment
    # The value of the last range tells us the total number of 2D points.
    N = structure_allotment[end][end]
    # Determine the total number of latent variables.
    M = length(𝛉) - N*2
    index₁ = SVector(1,2)
    index₂ = SVector(3,4)
    reprojections = Matrix{Float64}(undef, 4, N)
    𝛈 = @view 𝛉[1:M]
    𝐉 = zeros(4*N, length(𝛉))
    # Create a view of the jacobian matrix 𝐉 and reshape it so that
    # it will be more convenient to index into the appropriate entries
    # whilst looping over all of the data points.
    𝐉v = reshape(reinterpret(Float64,𝐉), 4, N, length(𝛉))
    𝐀 = SMatrix{2,3,Float64,6}(1,0,0,1,0,0)
    𝐈₃ = SMatrix{3,3}(1.0I)

    # TODO Revisit this so that we reduce the unnecessary memory allocations.
    i = M + 1
    for (k, span) in enumerate(structure_allotment)
        #𝐇ₖ = ℋ[k]
        𝐇ₖ = 𝛑(𝛈, k)
        #display(𝐇ₖ/ norm(𝐇ₖ))
        for n in span
            # Extract 2D point and convert to homogeneous coordinates
            𝐦 = hom(SVector{2,Float64}(𝛉[i],𝛉[i+1]))

            # Derivative of residual in first and second image w.r.t 2D point in the
            # first image.
            ∂𝐫₁_d𝐦 = 𝐀 * 𝐈₃
            ∂𝐫₂_d𝐦 = 𝐀 * ∂hom⁻¹(𝐇ₖ  * 𝐦) * 𝐇ₖ

            # 9 x 3
            # Derivative of residual in second image w.r.t homography martix.
            # ∂𝐫₁_d𝛈  is the zero vector.
            ∂𝐫₂_d𝐚 = ∂ₐ𝛑(𝛈, k)
            ∂𝐫₂_d𝐛 = ∂ᵦ𝛑(𝛈, k)
            ∂𝐫₂_d𝐯 = ∂ᵥ𝛑(𝛈, k)
            ∂𝐫₂_d𝐰 = ∂ₛ𝛑(𝛈, k)
            ∂𝐫₂_d𝛈 = hcat(∂𝐫₂_d𝐚, ∂𝐫₂_d𝐛, ∂𝐫₂_d𝐯, ∂𝐫₂_d𝐰)
            ∂𝐫₂_d𝐇ₖ = 𝐀 * ∂hom⁻¹(𝐇ₖ   * 𝐦) * (𝐦' ⊗ 𝐈₃) * ∂𝐫₂_d𝛈
            𝐉v[index₂,n, 1:M] = ∂𝐫₂_d𝐇ₖ
            𝐉v[index₁,n,i:i+1] = ∂𝐫₁_d𝐦[:,1:2]
            𝐉v[index₂,n,i:i+1] = ∂𝐫₂_d𝐦[:,1:2]
            i = i + 2
        end
    end
    𝐉
end

# TODO make this a type of HomographyMatrices
function initialise_latent_variables(ℋ::AbstractVector)
    N = length(ℋ)
    if N < 2
        throw(ArgumentError("Please supply at least two homography matrices."))
    end
    𝐗₁ = ℋ[1]
    𝛍 = zeros(N)
    𝐉 = Array{Float64}(undef,(3,(N-1)*6))
    i₁ = range(1, step = 6, length = N - 1)
    i₂ = range(6, step = 6, length = N - 1)
    for n = 2:N
        𝐗ₙ = ℋ[n]
        # TODO deal with complex results...
        e₁, e₂ = find_nearest_eigenvalues(real.(eigvals(Array(𝐗₁), Array(𝐗ₙ))))
        #e₁, e₂ = find_nearest_eigenvalues(eigvals(Array(𝐗₁), Array(𝐗ₙ)))
        𝐘 = hcat(e₁ * 𝐗ₙ - 𝐗₁, e₂ * 𝐗ₙ - 𝐗₁)
        μ = (e₁ + e₂) / 2
        𝛍[n] = real(μ)
        𝐉[:,i₁[n-1]:i₂[n-1]] .= 𝐘
    end
    𝛈 = initialisation_procedure(𝐉, 𝛍, ℋ)
end

function compose(type::Type{HomographyMatrices}, 𝛈::AbstractVector)
        N = div(length(𝛈) - 12,  4)
        𝐚  = @view 𝛈[1:9]
        𝐀 = reshape(𝐚, (3,3))
        𝐛 = @view 𝛈[10:12]
        𝐰 = @view 𝛈[end-(N-1):end]
        r = range(13, step = 3, length = N+1)
        𝐯 = reshape(view(𝛈,first(r):last(r)-1), (3,N))
        #ℋ = Array{Array{Float64,2},1}(undef,(N,))
        #ℋ = Array{SMatrix{Tuple{3,3},Float64,2,9},1}(undef,(N,))
        #ℋ = Vector{HomographyMatrix}(undef,N)
        ℋ = [HomographyMatrix(SMatrix{3,3,Float64,9}(𝐰[n]*𝐀 + 𝐛*𝐯[:,n]')) for n = 1:N]
        # for n = 1:N
        #     𝐇 = SMatrix{3,3,Float64,9}(𝐰[n]*𝐀 + 𝐛*𝐯[:,n]')
        #     ℋ[n] = HomographyMatrix(𝐇)
        #     #ℋ[n] = HomographyMatrix(𝐰[n]*𝐀 + 𝐛*𝐯[:,n]')
        # end
        HomographyMatrices(ℋ)
end

# Maps latent variables to homography matrices
function 𝛑(𝛈::AbstractVector, n::Integer)
    N = div(length(𝛈) - 12,  4)
    𝐚  = @view 𝛈[1:9]
    𝐀 = reshape(𝐚, (3,3))
    𝐛 = @view 𝛈[10:12]
    𝐰 = @view 𝛈[end-(N-1):end]
    r = range(13, step = 3, length = N+1)
    𝐯 = reshape(view(𝛈,first(r):last(r)-1), (3,N))
    out = SMatrix{3,3,Float64,9}(𝐰[n]*𝐀 + 𝐛*𝐯[:,n]')
end

function ∂ₐ𝛑(𝛈::AbstractVector, n::Integer)
    N = div(length(𝛈) - 12,  4)
    𝐰 = @view 𝛈[end-(N-1):end]
    𝐈₉ = SMatrix{9,9}(1.0I)
    out = 𝐰[n] * 𝐈₉
end

function ∂ᵦ𝛑(𝛈::AbstractVector, n::Integer)
    N = div(length(𝛈) - 12,  4)
    r = range(13, step = 3, length = N+1)
    𝐯 = reshape(view(𝛈,first(r):last(r)-1), (3,N))
    𝐈₃ = SMatrix{3,3}(1.0I)
    out = 𝐯[:,n] ⊗ 𝐈₃
end

function ∂ᵥ𝛑(𝛈::AbstractVector, n::Integer)
    N = div(length(𝛈) - 12,  4)
    𝐛 = @view 𝛈[10:12]
    𝐈₃ = SMatrix{3,3}(1.0I)
    out = zeros(9, 3 * N)
    s = (n-1) * 3 + 1
    e = s + 2
    out[:,s:e] .= 𝐈₃ ⊗ 𝐛
    out
end

function ∂ₛ𝛑(𝛈::AbstractVector, n::Integer)
    N = div(length(𝛈) - 12,  4)
    𝐚  = @view 𝛈[1:9]
    out = zeros(9, N)
    out[:, n] .= 𝐚
    out
end

# TODO make this take a type of HomographyMatrices
function initialisation_procedure(𝐉::AbstractArray, 𝛍::AbstractArray, ℋ::AbstractVector)
    N = length(ℋ)
    if N < 2
        throw(ArgumentError("Please supply at least two homography matrices."))
    end
    F = svd(𝐉)
    𝛈 = zeros(9 + 3 + (N*3) + N)
    𝐛 = real(F.U[:,1])
    𝐗₁ = ℋ[1]
    𝐀 = 𝐗₁
    𝐯₁ = SVector(0,0,0)
    wₙ = 1
    # pack 𝛈 = [𝐚,𝐛, 𝐯₁,...,𝐯ₙ, w₁, ..., wₙ]
    𝛈[1:9] .= vec(𝐀)
    𝛈[10:12] .= 𝐛
    for (n,i) in enumerate(range(13, step = 3, length = N))
        if n == 1
            𝛈[i:i+2] .= 𝐯₁
        else
            𝐗ₙ = ℋ[n]
            𝛈[i:i+2] .= 𝐯₁ +  (𝛍[n] * 𝐗ₙ - 𝐗₁)' * 𝐛 / (norm(𝐛)^2)
        end
    end
    𝛈[end-(N-1):end] .= wₙ
    𝛈
end

function find_nearest_eigenvalues(e::AbstractArray)
    #display(e) # TODO remove
    dist = SVector(abs(e[1]-e[2]), abs(e[1]-e[3]), abs(e[2]-e[3]))
    minval, index = findmin(dist)
    if index == 3
        i₁ = 2
        i₂ = 3
    elseif index == 2
        i₁ = 1
        i₂ = 3
    else
        i₁ = 1
        i₂ = 2
    end
    e[i₁], e[i₂]
end

# TODO Finish this implementation
function (estimator::ProjectiveEstimationAlgorithm{<: ReprojectionError,
                                                   <: ConstrainedBundleAdjustment{
                                                   <: ProjectiveEstimationAlgorithm{
                                                   <: AlgebraicLeastSquares,
                                                   <: DirectLinearTransform         },
                                                   <: ExplicitChojnackiSzpak      }
                                                   })(correspondences::AbstractCorrespondences,
                                                      structure_allotment::AbstractVector,
                                                      context::EstimateHomographyContext)#::Vector{<:SMatrix{3,3,Float64,9}}

   ℳ = correspondences[1]
   ℳʹ = correspondences[2]
   # normalize = HartleyNormalizeDataContext(correspondences)
   # # Transform data to a Hartley normalised coordinate system.
   # correspondences_normalized = normalize(correspondences)
   # ℳ = correspondences_normalized[1]
   # ℳʹ = correspondences_normalized[2]
   N = length(ℳ)
   S = length(structure_allotment)
   ℋ = [HomographyMatrix(@SMatrix zeros(3,3)) for s = 1:S]
   for s = 1:S
       𝒪 = ℳ[structure_allotment[s]]
       𝒪′ = ℳʹ[structure_allotment[s]]
       ℋ[s] = HomographyMatrix(context(Correspondences(tuple(𝒪, 𝒪′)), estimator.seed_estimator))
   end

   # Construct a length-((I*9)+2*N) vector consisting of the I homography matrices
   # (the first I*9 dimensions), as well as N two-dimensional points in the
   # first view (the remaining dimensions).
   𝛉 = pack(HomographyMatrices(ℋ), ℳ)
   𝐈 = SMatrix{3,3}(1.0I)
   index₁ = SVector(1,2)
   index₂ = SVector(3,4)
   pts = Matrix{Float64}(undef,4,N)
   for n = 1:N
       pts[index₁,n] = ℳ[n][index₁]
       pts[index₂,n] = ℳʹ[n][index₁]
   end
   cost_type = ReprojectionError()
   model = ExplicitConsistentHomographyFitModel(cost_type, structure_allotment)
   #jacobian = ImplicitConsistentHomographyFitJacobian(structure_allotment)

   # TODO: Implement in-place version of model and Jacobian
   fit = curve_fit(model,  𝐈, reshape(reinterpret(Float64,vec(pts)),(4*N,)) , 𝛉; show_trace = false)
   #fit = curve_fit(model, jacobian, 𝐈, reshape(reinterpret(Float64,vec(pts)),(4*N,)) , 𝛉; show_trace = false)

   𝛉_est = fit.param[1:(9*S)]

   # Apply post-correction scheme
   𝐉, dummy = form_rank_1_constraint(𝛉_est)
   𝐉ᵪ = enforce_rankone(𝐉)

   D𝐉 = x-> ForwardDiff.jacobian(vec_rank_1_constraint, x)
   D𝛟 = x-> ForwardDiff.jacobian(gather_constraints, x)
   #q = x-> ForwardDiff.hessian(vec_rank_1_constraint, x)
   #r = ForwardDiff.gradient(vec_rank_1_constraint, rand(9))
   #display(r)
   D𝐉⁺ = D𝐉(𝛉_est)
   δ′θ = -pinv(D𝐉(𝛉_est))*(vec(𝐉) - vec(𝐉ᵪ))

   #δ′θ = -pinv(D𝐉(𝛉_est))( vec(𝐉) - vec(𝐉ᵪ))
   display(δ′θ )
   #o = q(𝛉_est)
   #display(A)
   @show "phi"
   B = D𝛟(𝛉_est)
   #display(pinv(A))
   display(B)
   𝛟 = gather_constraints(𝛉_est)
   display(𝛟)
   #display(o)
   #display(g(rand(9)))

   ℋ′ = unpack_homographies(𝛉_est)

   # Transform estimate back to the original (unnormalised) coordinate system.
   # 𝒯 = matrices(normalize)
   # 𝐓 = 𝒯[1]
   # 𝐓ʹ = 𝒯[2]
   # ℋ₊ = map(ℋ′) do 𝐇
   #       inv(𝐓ʹ)*𝐇*𝐓
   # end
   # ℋ₊
   #fit
   ℋ′
end

# TODO Finish this implementation
function (param::ExplicitConsistentHomographyFitModel{<: ReprojectionError, <: AbstractVector})(𝐈,𝛉)
    structure_allotment = param.structure_allotment
    # The value of the last range tells us the total number of 2D points.
    N = structure_allotment[end][end]
    # Determine the total number of homography parameters
    M = length(𝛉) - N*2
    index₁ = SVector(1,2)
    index₂ = SVector(3,4)
    #reprojections = Matrix{Float64}(undef,4,N)
    reprojections = zeros(4,N)
    𝛉v = @view 𝛉[1:M]
    # TODO Revisit this so that we reduce the unnecessary memory allocations.
    ℋ = unpack_homographies(𝛉v)
    i = M + 1
    for (k, span) in enumerate(structure_allotment)
        𝐇ₖ = ℋ[k]
        for n in span
            # Extract 2D point and convert to homogeneous coordinates
            𝐦 = hom(SVector{2,Float64}(𝛉[i],𝛉[i+1]))
            reprojections[index₁,n] = hom⁻¹(𝐦)
            reprojections[index₂,n] = hom⁻¹(𝐇ₖ  * 𝐦)
            i = i + 2
        end
    end
    reshape(reinterpret(Float64,vec(reprojections)),(4*N,))
end

# Explicit Constraints with Mahalnobis distance
function (estimator::ProjectiveEstimationAlgorithm{<: Mahalanobis,
                                                   <: ConstrainedMahalanobis{
                                                   <: ProjectiveEstimationAlgorithm{
                                                   <: T₁,
                                                   <: T₂                            },
                                                   <: ExplicitChojnackiSzpak}
                                                   })(correspondences::AbstractCorrespondences,
                                                      structure_allotment::AbstractVector,
                                                      estimation_context::EstimateHomographyContext) where {T₁ <: AbstractCost, T₂ <: AbstractProjectiveOptimizationScheme} #::Vector{<:SMatrix{3,3,Float64,9}}

    # Transform data to a Hartley normalised coordinate system.
    normalize = HartleyNormalizeDataContext(correspondences)
    correspondences_normalized = normalize(correspondences)
    ℳ = correspondences_normalized[1]
    ℳʹ = correspondences_normalized[2]
    N = length(ℳ)
    S = length(structure_allotment)
    ℋ = [@SMatrix zeros(3,3) for s = 1:S]
    ℬ =  [@SMatrix zeros(9,9) for s = 1:S]
    cost_context = CostContext(ApproximateMaximumLikelihood(), estimation_context)
    uncertainty_context = UncertaintyContext(UnitNormGauge(), ProjectiveEstimationAlgorithm(; objective = T₁(), solver = T₂()))
    solver = estimator.solver
    for s = 1:S
        𝒪 = ℳ[structure_allotment[s]]
        𝒪′ = ℳʹ[structure_allotment[s]]
        correspondencesₛ = Correspondences(tuple(𝒪, 𝒪′))
        ℋ[s] = estimation_context(correspondencesₛ, solver.seed_estimator)
        𝐇 = ℋ[s]
        𝛉 = vec(𝐇)
        Nₛ = length(𝒪)
        cost = cost_context(𝛉, correspondencesₛ)
        # An estimate of the noise level based on the approximate maximum likelihood cost function.
        σ = sqrt(cost / (Nₛ - 8))
        σ² = σ^2
        # Construct default covariance matrices using the estimated noise level.
        Λ = [SMatrix{2,2,Float64,4}(σ², 0, 0 ,σ²) for n = 1:Nₛ]
        Λ′ = [SMatrix{2,2,Float64,4}(σ², 0, 0 ,σ²) for n = 1:Nₛ]
        covariance_matricesₛ = Correspondences(tuple(Λ, Λ′))
        𝐂 = uncertainty_context(HomographyMatrix(𝐇), correspondencesₛ, covariance_matricesₛ)
        # The information matrix is the pseudo-inverse of the covariance matrix.
        𝚲⁺ = pinv(𝐂)
        # We work with the non-negative definite suare root because we express
        # our cost function as a sum of squares.
        ℬ[s] = SMatrix{9,9,Float64,81}(non_negative_definite_square_root(𝚲⁺))
    end
    # Construct a length-(4I+12) vector consisting of latent variables that
    # enforce consistency between the I homography matrices.
    #𝛈 = pack(HomographyMatrices(HomographyMatrix.(ℋ)))
    # 𝛈 = initialise_latent_variables(ℋ)

    𝛉 = Vector{Float64}(undef, S*9)
    r = 1:9:(S*9)
    for (i, k) in enumerate(r)
        𝐇 = ℋ[i]
        𝛉[k:(k+8)] = vec(𝐇)
    end



    𝐉, dummy = form_rank_1_constraint(𝛉)
    𝐉ᵪ = enforce_rankone(𝐉)

    D𝐉 = x-> ForwardDiff.jacobian(vec_rank_1_constraint, x)
    D𝛟 = x-> ForwardDiff.jacobian(gather_constraints, x)
    δ′θ = -pinv(D𝐉(𝛉))*(vec(𝐉) - vec(𝐉ᵪ))
    𝐏 = pinv(D𝛟(𝛉)) * D𝛟(𝛉)
    𝐐 = UniformScaling(1) - 𝐏
    #D𝐉⁺ = D𝐉(𝛉_est)


    #B = D𝛟(𝛉)
    #display(pinv(A))
    𝛟 = gather_constraints(𝛉)


    cost_type = Mahalanobis(ℬ)
    model = ExplicitConsistentHomographyFitModel(cost_type, [1:S])
    #jacobian = ImplicitConsistentHomographyFitJacobian(cost_type, structure_allotment)
    𝐈 = SMatrix{3,3}(1.0I)
    #Z = model(𝐈,𝛉)
    ∇J = x-> ForwardDiff.gradient(z -> model(𝐈, z), x)
    ∇²J = x-> ForwardDiff.hessian(z -> model(𝐈, z), x)
    #
    #@show "Z"
    #Z = ∇²J(𝛉)
    ∇J𝛉 = ∇J(𝛉)
    ∇²J𝛉 = ∇²J(𝛉)
    δ′′θ = -pinv(𝐐 * ∇²J𝛉 * 𝐐) * (∇J𝛉 + ∇²J𝛉 * δ′θ)

    δθ = δ′θ + δ′′θ

    α = 1e-1
    m = x-> norm(gather_constraints(x))^2 + norm(𝐐 * ∇J𝛉)^2
    @show "merit"
    display(m(𝛉))
    display(m(𝛉 + α*δθ))
    𝛉_est = 𝛉 + α*δθ
    #display(∇J𝛉)
    #display(∇²J𝛉)


    # TODO: Implement in-place version of model and Jacobian
    #fit = curve_fit(model,  𝐈, zeros(9*S), 𝛈 ; show_trace = false)
    #fit = curve_fit(model, jacobian,  𝐈, zeros(9*S), 𝛈 ; show_trace = false)
    #𝛈_est = fit.param
    ℋ′ =  unpack_homographies(𝛉_est)
    # Transform estimate back to the original (unnormalised) coordinate system.
    𝒯 = matrices(normalize)
    𝐓 = 𝒯[1]
    𝐓ʹ = 𝒯[2]
    ℋ₊ = map(ℋ′) do 𝐇
          inv(𝐓ʹ)*𝐇*𝐓
    end
    ℋ₊
end


function (param::ExplicitConsistentHomographyFitModel{<:Mahalanobis, <: AbstractVector})(𝐈,𝛉)
    # Vector of non-negative square root of covariance matrices.
    ℬ = param.cost.ℬ
    structure_allotment = param.structure_allotment
    I = length(ℬ)
    # TODO Revisit this so that we reduce the unnecessary memory allocations.
    #ℋ = unpack_homographies(𝛉)
    r = 1:9:(I*9)
    #ℋ = Vector{HomographyMatrix{SMatrix{3,3,Float64,9}}}(undef,I)
    ℋ = Vector(undef,I)
    for (i,k) in enumerate(r)
        𝛉v = @view 𝛉[k:(k+8)]
        𝐇 = reshape(𝛉v,(3,3))
        ℋ[i] = 𝐇
    end
    ####################



    residual = zeros(eltype(𝛉),9,I)
    for i = 1:I
        𝛉ᵢ = vec(ℋ[i])
        𝐁ᵢ = ℬ[i]
        residual[:,i] = (𝛉ᵢ' * 𝐁ᵢ) * (norm(𝛉ᵢ)^-1)
    end
    #display(residual)
    # Todo remove collect once we don't need to use  ForwardDiff
    #collect(reshape(reinterpret(Float64,vec(residual)),(9*I,)))
    z = vec(residual)
    dot(z,z)
end


# Construct a parameter vector consisting of a homography matrix and 2D points.
function pack(H::HomographyMatrices, ℳ::AbstractArray)
    ℋ = matrices(H)
    N = length(ℳ)
    I = length(ℋ)
    r = 1:9:(I*9)
    𝛉 = Vector{Float64}(undef, I*9 + N*2)
    for (i, k) in enumerate(r)
        𝐇 = ℋ[i]
        𝛉[k:(k+8)] = vec(𝐇)
    end
    k = I*9 + 1
    for n = 1:N
        𝛉[k:k+1] = ℳ[n][1:2]
        k = k + 2
    end
    𝛉
end

function pack(H::HomographyMatrices)
    ℋ = matrices(H)
    I = length(ℋ)
    r = 1:9:(I*9)
    𝛉 = Vector{Float64}(undef, I*9)
    for (i, k) in enumerate(r)
        𝐇 = ℋ[i]
        𝛉[k:(k+8)] = vec(𝐇)
    end
    𝛉
end

function unpack_homographies(𝛉::AbstractVector)
    I = div(length(𝛉),  9)
    r = 1:9:(I*9)
    #ℋ = Vector{HomographyMatrix{SMatrix{3,3,Float64,9}}}(undef,I)
    ℋ = Vector{SMatrix{3,3,Float64,9}}(undef,I)
    for (i,k) in enumerate(r)
        𝛉v = @view 𝛉[k:(k+8)]
        𝐇 = SMatrix{3,3,Float64,9}(reshape(𝛉v,(3,3)))
        ℋ[i] = 𝐇
    end
    ℋ
end


function characteristic_polynomial(𝐀::AbstractArray, 𝐁::AbstractArray)
    𝐚₁ = @view 𝐀[:,1]
    𝐚₂ = @view 𝐀[:,2]
    𝐚₃ = @view 𝐀[:,3]
    𝐛₁ = @view 𝐁[:,1]
    𝐛₂ = @view 𝐁[:,2]
    𝐛₃ = @view 𝐁[:,3]

    c₀ = det(𝐀)
    c₁ = det(hcat(𝐛₁, 𝐚₂, 𝐚₃)) + det(hcat(𝐚₁, 𝐛₂, 𝐚₃)) + det(hcat(𝐚₁, 𝐚₂, 𝐛₃))
    c₂ = det(hcat(𝐚₁, 𝐛₂, 𝐛₃)) + det(hcat(𝐛₁, 𝐚₂, 𝐛₃)) + det(hcat(𝐛₁, 𝐛₂, 𝐚₃))
    c₃ = det(𝐁)
    return c₀, c₁, c₂, c₃
end

function ω(𝐇ᵢ::AbstractArray, 𝐇₁::AbstractArray)
    c₀, c₁, c₂, c₃ = characteristic_polynomial(𝐇ᵢ, 𝐇₁)
    σ = c₁ * c₂ - 9*c₀*c₃
    τ = 2*(c₂^2 - 3*c₁*c₃)
    return σ / τ
end

function form_rank_1_constraint(𝛉::AbstractVector)
    I = div(length(𝛉),  9)
    r = 1:9:(I*9)
    # # #ℋ = Vector{HomographyMatrix{SMatrix{3,3,Float64,9}}}(undef,I)
    # # ℋ = Vector(undef,I)
    # # for (i,k) in enumerate(r)
    # #     𝛉v = @view 𝛉[k:(k+8)]
    # #     𝐇 = reshape(𝛉v,(3,3))
    # #     ℋ[i] = 𝐇
    # # end
    #
    ℋ = [Array(reshape(𝛉[k:(k+8)],(3,3))) for k in r]
    # # #I = length(ℋ)
    # # # TODO preallocate instead of hcat
    𝐉 = [ℋ[i] - ω(ℋ[i], ℋ[1]) * ℋ[1] for i = 2:I]
    #vec(hcat(𝐉...))
    hcat(𝐉...), ℋ
end

function vec_rank_1_constraint(𝛉::AbstractVector)
    𝐉, ℋ = form_rank_1_constraint(𝛉)
    vec(𝐉)
end

# function reshape_homographies(𝛉::AbstractVector)
#     I = div(length(𝛉),  9)
#     r = 1:9:(I*9)
#     ℋ = [Array(reshape(𝛉[k:(k+8)],(3,3))) for k in r]
# end

function gather_constraints(𝛉::AbstractVector)
    I = div(length(𝛉),  9)
    total_constraints = binomial(3, 2) * binomial(3*I-3, 2)
    𝛟 = zeros(eltype(𝛉), total_constraints)
    𝐉, ℋ = form_rank_1_constraint(𝛉)
    k = 1
    index₁ = [0, 0]
    index₂ = [0, 0]
    for a = 1:3
        for b = 1:3
            for c = 1:(3*I - 3)
                for d = 1:(3*I - 3)
                    if (a < b) && (c < d)
                        ic = ceil(Int,c / 3) + 1
                        id = ceil(Int,d / 3) + 1
                        H_ic = ℋ[ic]
                        H_id = ℋ[id]
                        index₁[1] = a
                        index₁[2] = b
                        index₂[1] = c
                        index₂[2] = d
                        𝛟[k] = phi_constraint(H_ic, H_id, 𝐉, index₁, index₂)
                        k = k + 1
                    end
                end
            end
        end
    end
    𝛟
end

function phi_constraint(H_ic::AbstractMatrix, H_id::AbstractMatrix, 𝐉::AbstractMatrix, index₁::AbstractVector, index₂::AbstractVector)
    𝐉v = view(𝐉, index₁, index₂)
    norm(H_ic)^-1 * norm(H_id)^-1 * det(𝐉v)
end

function enforce_rankone(𝐉::AbstractArray)
    # Enforce the rank-1 constraint.
    U,S,V = svd(𝐉)
    S[end-1] = 0.0
    S[end] = 0.0
    U*Matrix(Diagonal(S))*V'
end
