abstract type AbstractProjectiveOptimizationScheme end
abstract type AbstractFundamentalNumericalScheme <: AbstractProjectiveOptimizationScheme  end
abstract type AbstractConstrainedProjectiveOptimizationScheme end
abstract type ConstrainedFundamentalNumericalScheme <: AbstractConstrainedProjectiveOptimizationScheme end

abstract type AbstractVariant end
struct UndampedVariant <: AbstractVariant end
struct DampedVariant <: AbstractVariant end

Base.@kwdef struct FundamentalNumericalScheme{T₁ <: AbstractVariant, T₂ <: Number} <: AbstractProjectiveOptimizationScheme
    variant::T₁ = UndampedVariant()
    Δθ_tol::T₂ = 1e-8
    max_iter::Int = 8
end

function (fns::FundamentalNumericalScheme{<: UndampedVariant, <: Number})(𝛉::AbstractVector, correspondences::AbstractCorrespondences, covariance_matrices::AbstractCorrespondences, estimation_context::AbstractEstimationContext, objective::ApproximateMaximumLikelihood)
    I = fns.max_iter
    ℳ = correspondences[1]
    ℳʹ = correspondences[2]
    Λ = covariance_matrices[1]
    Λ′ = covariance_matrices[2]
    cost_context = CostContex(objective, context)
    # TODO add stopping critria based on parameter difference
    for i = 1:I
        𝐗 = evaluate_𝐗(objective, estimation_context, 𝛉, correspondencs, covariance_matrices)
        λ, 𝛉⁺ = smallest_eigenpair(Symmetric(𝐗))
        # TODO check type stability
        𝛉 = vec(𝛉⁺)
    end
    return 𝛉
end

Base.@kwdef  struct DirectLinearTransform  <: AbstractProjectiveOptimizationScheme
    apply_normalisation::Bool = true
end

struct ManualEstimation <: AbstractProjectiveOptimizationScheme end

Base.@kwdef  struct BundleAdjustment{T <: AbstractProjectiveEstimationAlgorithm}  <:  AbstractProjectiveOptimizationScheme
    seed_estimator::T = ProjectiveEstimationAlgorithm(AlgebraicLeastSquares(), DirectLinearTransform())
end

Base.@kwdef  struct ConstrainedBundleAdjustment{T₁ <: AbstractProjectiveEstimationAlgorithm, T₂ <: AbstractProjectiveConstraints}  <: AbstractConstrainedProjectiveOptimizationScheme
    seed_estimator::T₁ = ProjectiveEstimationAlgorithm(AlgebraicLeastSquares(), DirectLinearTransform())
    constraints::T₂ = ImplicitChojnackiSzpak()
    #TODO add option for choice of initialisation algorithm
end

Base.@kwdef  struct ConstrainedMahalanobis{T₁ <: AbstractProjectiveEstimationAlgorithm, T₂ <: AbstractProjectiveConstraints}  <: AbstractConstrainedProjectiveOptimizationScheme
    seed_estimator::T₁ = ProjectiveEstimationAlgorithm(AlgebraicLeastSquares(), DirectLinearTransform())
    constraints::T₂ = ImplicitChojnackiSzpak()
    #TODO add option for choice of initialisation algorithm
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
