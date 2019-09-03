abstract type AbstractNoiseModel end

struct HomogeneousGaussianNoise <: AbstractNoiseModel
    σ::Float64
end

function (noise::HomogeneousGaussianNoise)(ℳ::AbstractVector)
    σ = noise.σ
    𝒪 = map(ℳ) do 𝐦
        δ₁ = first(randn(1)) * σ
        δ₂ = first(randn(1)) * σ
        Δ = SVector(δ₁, δ₂)
        𝐦 + Δ
    end
end
