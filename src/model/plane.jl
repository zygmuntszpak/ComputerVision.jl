abstract type AbstractPlane end


Base.@kwdef struct Plane{T₁ <: Vec3, T₂ <: Real} <: AbstractPlane
    normal::T₁
    distance::T₂
end

function get_normal(plane::Plane)
    plane.normal
end

function get_distance(plane::Plane)
    plane.distance
end

Base.@kwdef struct PlaneSegment{T₁ <: Plane, T₂ <: AbstractVector} <: AbstractPlane
    plane::T₁
    segment::T₂
end

function get_plane(plane_segment::PlaneSegment)
    plane_segment.plane
end

function set_plane!(plane_segment::PlaneSegment, plane::Plane)
    plane_segment.plane = plane
end

function get_normal(plane_segment::PlaneSegment)
    get_normal(get_plane(plane_segment))
end

function get_distance(plane_segment::PlaneSegment)
    get_distance(get_plane(plane_segment))
end

function get_segment(plane_segment::PlaneSegment)
    plane_segment.segment
end

function on_plane(𝐗::AbstractVector, plane::Plane; tol::Number = 1e-10)
    𝐧 = get_normal(plane)
    d = get_distance(plane)
    abs(dot(𝐗,𝐧) - d) < tol ? true : false
end
