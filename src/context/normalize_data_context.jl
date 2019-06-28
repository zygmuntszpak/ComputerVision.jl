abstract type NormalizeDataContext <: AbstractContext end
struct HartleyNormalizeDataContext{N,T <:AbstractMatrix} <: NormalizeDataContext
    transformations::NTuple{N,T}
end

function matrices(context::HartleyNormalizeDataContext)
    context.transformations
end

HartleyNormalizeDataContext(data::Correspondences) = HartleyNormalizeDataContext(construct_normalization_context(data))

function construct_normalization_context(data::Correspondences{N,T}) where {N,T}
    _construct_normalization_context(data, Val(N))
end

function _construct_normalization_context(data, ::Val{2})
    return hartley_transformation(data[1]), hartley_transformation(data[2])
end

function (normalize_data::HartleyNormalizeDataContext)(data::Correspondences{N,T}) where {N,T}
    transformations = matrices(normalize_data)
    _normalize_data(data, transformations, Val(N))
end

function _normalize_data(data, transformations, ::Val{2})
    𝐓₁ = transformations[1]
    𝐓₂ = transformations[2]
    ℳ = data[1]
    ℳ′ = data[2]
    return Correspondences((hartley_normalization(ℳ, 𝐓₁), hartley_normalization(ℳ′, 𝐓₂)))
end

function hartley_transformation(ℳ::Vector{<:AbstractArray})
    if isempty(ℳ)
        throw(ArgumentError("Array cannot be empty."))
    end
    npts = length(ℳ)
    ndim = length(ℳ[1])
    𝐜 = centroid(ℳ)
    σ = root_mean_square(ℳ, 𝐜)
    σ⁻¹ = 1 / σ
    𝐓 = SMatrix{ndim+1,ndim+1,Float64, (ndim+1)^2}([σ⁻¹*Matrix{Float64}(I,ndim,ndim) -σ⁻¹*𝐜 ; zeros(1,ndim) 1.0])
end

function centroid(positions::Vector{T}) where T <: AbstractArray
    x = zeros(T)
    for pos ∈ positions
        x = x + pos
    end
    return x / length(positions)
end

function root_mean_square(ℳ::Vector{T}, 𝐜::T ) where  T <: AbstractArray
    total = 0.0
    npts = length(ℳ)
    ndim = length(ℳ[1])
    for 𝐦 ∈ ℳ
         total  = total + ∑((𝐦-𝐜).^2)
    end
    σ = √( (1/(npts*ndim)) * total)
end

function hartley_normalization(ℳ::Vector{<:AbstractArray}, 𝐓::AbstractMatrix)
    𝒪 = hartley_normalization!(copy(ℳ), 𝐓)
end

function hartley_normalization!(ℳ::Vector{<:AbstractArray}, 𝐓::AbstractMatrix)
    map!(ℳ , ℳ) do 𝐦
         hom⁻¹(𝐓 * hom(𝐦))
    end
     ℳ
end


function hartley_normalization(ℳ::Vector{<:AbstractArray})
    𝒪, 𝐓 = hartley_normalization!(copy(ℳ))
end

function hartley_normalization!(ℳ::Vector{<:AbstractArray})
    𝐓 = hartley_transformation(ℳ)
    map!(ℳ , ℳ) do 𝐦
         hom⁻¹(𝐓 * hom(𝐦))
    end
     ℳ, 𝐓
end
