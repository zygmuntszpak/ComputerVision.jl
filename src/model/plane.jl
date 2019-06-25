Base.@kwdef struct Plane{T₁ <: Vec3, T₂ <: Real}
    normal::T₁
    distance::T₂
end

function get_normal(plane::Plane)
    plane.normal
end

function get_distance(plane::Plane)
    plane.distance
end
