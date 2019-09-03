abstract type AbstractSimulationSetup end

# Base.@kwdef struct VaryPlanesSimulationSetup{T₁ <: AbstractVector, T₂ <: AbstractVector, T₃ <: AbstractVector, T₄ <: AbstractVector, T₅ <: Integer, T₆ <: Integer} <: AbstractSimulationSetup
#     planes::T₁ = [1, 2, 3, 4, 5]
#     roi_dimensions::T₂ = [[200.0], [200.0, 200.0], [200.0, 200.0, 200.0], [200.0, 200.0, 200.0, 200.0], [200.0, 200.0, 200.0, 200.0, 200.0]]
#     noise_per_roi::T₃ = [[0.01], [0.01, 0.01], [0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01], [0.01, 0.01, 0.01, 0.01, 0.01]]
#     points_per_roi::T₄ = [[24], [24, 24], [24, 24, 24], [24, 24, 24, 24], [24, 24, 24, 24, 24]]
#     replications::T₅ = 10
#     trials::T₆ = 15
# end

# Base.@kwdef struct VaryPlanesSimulationSetup{T₁ <: AbstractVector, T₂ <: AbstractVector, T₃ <: AbstractVector, T₄ <: AbstractVector, T₅ <: Integer, T₆ <: Integer} <: AbstractSimulationSetup
#     planes::T₁ = [1, 2, 3, 4, 5]
#     roi_dimensions::T₂ = [[200.0], [200.0, 200.0], [200.0, 200.0, 200.0], [200.0, 200.0, 200.0, 200.0], [200.0, 200.0, 200.0, 200.0, 200.0]]
#     noise_per_roi::T₃ = [[1], [1, 1], [1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1, 1]]
#     points_per_roi::T₄ = [[24], [24, 24], [24, 24, 24], [24, 24, 24, 24], [24, 24, 24, 24, 24]]
#     replications::T₅ = 10
#     trials::T₆ = 15
# end

# Base.@kwdef struct VaryPlanesSimulationSetup{T₁ <: AbstractVector, T₂ <: AbstractVector, T₃ <: AbstractVector, T₄ <: AbstractVector, T₅ <: Integer, T₆ <: Integer} <: AbstractSimulationSetup
#     planes::T₁ = [2, 3, 4, 5]
#     roi_dimensions::T₂ = [ [200.0, 200.0], [200.0, 200.0, 200.0], [200.0, 200.0, 200.0, 200.0], [200.0, 200.0, 200.0, 200.0, 200.0]]
#     noise_per_roi::T₃ = [[1, 1], [1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1, 1]]
#     points_per_roi::T₄ = [[24, 24], [24, 24, 24], [24, 24, 24, 24], [24, 24, 24, 24, 24]]
#     replications::T₅ = 10
#     trials::T₆ = 15
# end

# Base.@kwdef struct VaryPlanesSimulationSetup{T₁ <: AbstractVector, T₂ <: AbstractVector, T₃ <: AbstractVector, T₄ <: AbstractVector, T₅ <: Integer, T₆ <: Integer} <: AbstractSimulationSetup
#     planes::T₁ = [2, 3, 4, 5]
#     roi_dimensions::T₂ = [ [20.0, 200.0], [20.0, 40.0, 200.0], [20.0, 40.0, 100.0, 200.0], [20.0, 40.0, 100.0, 150.0, 200.0]]
#     noise_per_roi::T₃ = [[1, 1], [1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1, 1]]
#     points_per_roi::T₄ = [[24, 24], [24, 24, 24], [24, 24, 24, 24], [24, 24, 24, 24, 24]]
#     replications::T₅ = 10
#     trials::T₆ = 15
# end

Base.@kwdef struct VaryPlanesSimulationSetup{T₁ <: AbstractVector, T₂ <: AbstractVector, T₃ <: AbstractVector, T₄ <: AbstractVector, T₅ <: Integer, T₆ <: Integer} <: AbstractSimulationSetup
    planes::T₁ = [2]
    roi_dimensions::T₂ = [ [200.0, 200.0]]
    noise_per_roi::T₃ = [[0.3, 0.3]]
    points_per_roi::T₄ = [[24, 24]]
    replications::T₅ = 10
    trials::T₆ = 15
end
