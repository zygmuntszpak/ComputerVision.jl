abstract type AbstractVisualProperties end

Base.@kwdef struct  MakieVisualProperties{T₁ <: Real, T₂ <: Number, T₃ <: Number} <: AbstractVisualProperties
    scale::T₁ = 100.0f0
    linewidth::T₂  = 4
    markersize::T₃ = 5
end

Base.@kwdef struct PGFPlotsVisualProperties{T₁ <: Real, T₂ <: Number, T₃ <: Number} <: AbstractVisualProperties
    scale::T₁ = 100.0f0
    linewidth::T₂  = 4
    markersize::T₃ = 5
end

function get_scale(properties::AbstractVisualProperties)
    properties.scale
end

function set_scale!(properties::AbstractVisualProperties, scale::Real)
    properties.scale = scale
end

function get_linewidth(properties::AbstractVisualProperties)
    properties.linewidth
end

function set_linewidth!(properties::AbstractVisualProperties, linewidth::Number)
    properties.linewidth = linewidth
end

function get_markersize(properties::AbstractVisualProperties)
    properties.markersize
end

function set_markersize!(properties::AbstractVisualProperties, markersize::Number)
    properties.markersize = markersize
end
