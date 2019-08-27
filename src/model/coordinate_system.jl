abstract type AbstractCoordinateSystem end
abstract type AbstractPlanarCoordinateSystem <: AbstractCoordinateSystem end

Base.@kwdef struct PlanarCartesianSystem{T₁ <: AbstractVector, T₂ <: AbstractVector} <: AbstractPlanarCoordinateSystem
    𝐨::T₁ = Vec(0.0, 0.0)
    𝐞₁::T₂ = Vec(-1.0, 0.0)
    𝐞₂::T₂ = Vec(0.0, 1.0)
end

Base.@kwdef struct CartesianSystem{T₁ <: AbstractVector, T₂ <: AbstractVector} <: AbstractCoordinateSystem
    𝐨::T₁ = Vec(0.0, 0.0, 0.0)
    𝐞₁::T₂ = Vec(1.0, 0.0, 0.0)
    𝐞₂::T₂ = Vec(0.0, 1.0, 0.0)
    𝐞₃::T₂ = Vec(0.0, 0.0, 1.0)
end

Base.@kwdef struct RasterSystem{T₁ <: AbstractVector, T₂ <: AbstractVector} <: AbstractPlanarCoordinateSystem
    𝐨::T₁ = Vec(0.0, 0.0)
    𝐞₁::T₂ = Vec(-1.0, 0.0)
    𝐞₂::T₂ = Vec(0.0, -1.0)
end

Base.@kwdef struct OpticalSystem{T₁ <: AbstractVector, T₂ <: AbstractVector} <: AbstractPlanarCoordinateSystem
    𝐨::T₁ = Vec(0.0, 0.0)
    𝐞₁::T₂ = Vec(-1.0, 0.0)
    𝐞₂::T₂ = Vec(0.0, -1.0)
end

# TODO depractate get_ convention
function get_origin(param::AbstractCoordinateSystem)
    param.𝐨
end

function origin(param::AbstractCoordinateSystem)
    param.𝐨
end

function get_e₁(param::AbstractCoordinateSystem)
    param.𝐞₁
end

function get_e₂(param::AbstractCoordinateSystem)
    param.𝐞₂
end

function get_e₃(param::AbstractCoordinateSystem)
    param.𝐞₃
end
