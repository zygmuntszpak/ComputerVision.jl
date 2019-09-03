abstract type AbstractCostFunction end


abstract type AbstractProjectiveConstraints end
abstract type AbstractImplicitConstraints <: AbstractProjectiveConstraints end
abstract type AbstractExplicitConstraints <: AbstractProjectiveConstraints end
abstract type AbstractImplicitHomographyConstraints <: AbstractImplicitConstraints end
abstract type AbstractExplicitHomographyConstraints <: AbstractExplicitConstraints end
struct ExplicitChojnackiSzpak <: AbstractExplicitHomographyConstraints end
struct ImplicitChojnackiSzpak <: AbstractImplicitHomographyConstraints end

abstract type AbstractEstimationAlgorithm end
abstract type AbstractProjectiveEstimationAlgorithm <: AbstractEstimationAlgorithm end
abstract type AbstractConstrainedProjectiveEstimationAlgorithm <: AbstractProjectiveEstimationAlgorithm end

abstract type AbstractProjectiveOptimizationScheme end
abstract type AbstractFundamentalNumericalScheme <: AbstractProjectiveOptimizationScheme  end
abstract type AbstractConstrainedProjectiveOptimizationScheme <: AbstractProjectiveOptimizationScheme end
abstract type ConstrainedFundamentalNumericalScheme <: AbstractConstrainedProjectiveOptimizationScheme end

abstract type AbstractVariant end
struct UndampedVariant <: AbstractVariant end
struct DampedVariant <: AbstractVariant end

Base.@kwdef  struct ProjectiveEstimationAlgorithm{T₁ <: AbstractCost, T₂ <: AbstractProjectiveOptimizationScheme}  <: AbstractProjectiveEstimationAlgorithm
    objective::T₁ = AlgebraicLeastSquares()
    solver::T₂ = DirectLinearTransform()
    apply_normalisation::Bool = true
end



Base.@kwdef struct FundamentalNumericalScheme{T₁ <: AbstractVariant, T₂ <: Number, T₃ <: AbstractProjectiveEstimationAlgorithm} <: AbstractProjectiveOptimizationScheme
    variant::T₁ = UndampedVariant()
    Δθ_tol::T₂ = 1e-8
    max_iter::Int = 8
    seed_estimator::T₃ = ProjectiveEstimationAlgorithm(; objective = AlgebraicLeastSquares(), solver = DirectLinearTransform())
end



Base.@kwdef  struct DirectLinearTransform  <: AbstractProjectiveOptimizationScheme
    apply_normalisation::Bool = true
end

struct ManualEstimation <: AbstractProjectiveOptimizationScheme end

Base.@kwdef  struct BundleAdjustment{T <: AbstractProjectiveEstimationAlgorithm}  <:  AbstractProjectiveOptimizationScheme
    seed_estimator::T = ProjectiveEstimationAlgorithm(; objective = AlgebraicLeastSquares(), solver = DirectLinearTransform())
end

Base.@kwdef  struct ConstrainedBundleAdjustment{T₁ <: AbstractProjectiveEstimationAlgorithm, T₂ <: AbstractProjectiveConstraints}  <: AbstractConstrainedProjectiveOptimizationScheme
    seed_estimator::T₁ = ProjectiveEstimationAlgorithm(; objective = AlgebraicLeastSquares(), solver = DirectLinearTransform())
    constraints::T₂ = ImplicitChojnackiSzpak()
    #TODO add option for choice of initialisation algorithm
end

Base.@kwdef  struct ConstrainedMahalanobis{T₁ <: AbstractProjectiveEstimationAlgorithm, T₂ <: AbstractProjectiveConstraints}  <: AbstractConstrainedProjectiveOptimizationScheme
    seed_estimator::T₁ = ProjectiveEstimationAlgorithm(; objective = AlgebraicLeastSquares(), solver = DirectLinearTransform())
    constraints::T₂ = ImplicitChojnackiSzpak()
    #TODO add option for choice of initialisation algorithm
end


#
# Base.@kwdef  struct DirectLinearTransform  <: AbstractProjectiveEstimationAlgorithm
#     apply_normalisation::Bool = true
# end
#
# Base.@kwdef  struct ApproximateMaximumLikelihood{T₁ <: AbstractProjectiveEstimationAlgorithm, T₂ <: }  <: AbstractProjectiveEstimationAlgorithm
#     seed_estimator::T₁ = DirectLinearTransform()
#     solver::T = FundamentalNumericalScheme()
#     apply_normalisation::Bool = true
# end
#
# struct ManualEstimation <: AbstractProjectiveEstimationAlgorithm end
# Base.@kwdef  struct BundleAdjustment{T <: AbstractProjectiveEstimationAlgorithm}  <: AbstractProjectiveEstimationAlgorithm
#     seed_estimator::T = DirectLinearTransform()
# end
#
# Base.@kwdef  struct ConstrainedBundleAdjustment{T₁ <: AbstractProjectiveEstimationAlgorithm, T₂ <: AbstractProjectiveConstraints}  <: AbstractConstrainedProjectiveEstimationAlgorithm
#     seed_estimator::T₁ = DirectLinearTransform()
#     constraints::T₂ = ImplicitChojnackiSzpak()
#     #TODO add option for choice of initialisation algorithm
# end
#
# Base.@kwdef  struct ConstrainedMahalanobis{T₁ <: AbstractProjectiveEstimationAlgorithm, T₂ <: AbstractProjectiveConstraints}  <: AbstractConstrainedProjectiveEstimationAlgorithm
#     seed_estimator::T₁ = DirectLinearTransform()
#     constraints::T₂ = ImplicitChojnackiSzpak()
#     #TODO add option for choice of initialisation algorithm
# end




#BundleAdjustment() =
