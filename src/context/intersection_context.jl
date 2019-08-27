struct IntersectionContext <: AbstractContext end

function (context::IntersectionContext)(plane::EuclideanPlane3D, line::Line3D)
    # Based on http://paulbourke.net/geometry/pointlineplane/
    𝐩₁ = line.𝐩₁
    𝐩₂ = line.𝐩₂
    𝐩₃ = origin(plane)
    𝐧 = normal(plane)
    v₁ = dot(𝐧, 𝐩₃ - 𝐩₁)
    v₂ = dot(𝐧, 𝐩₂ - 𝐩₁)
    # If the denominator is 0 then the normal to the plane is perpendicular to
    # the line. Thus the line is either parallel to the plane and there are no
    # solutions or the line is on the plane in which case there are an infinite
    # number of solutions.
    unique_intersection = v₂ == 0.0 ? false  : true
    if unique_intersection
        u = unique_intersection ? v₁ / v₂ : 0.0
        𝐩 = 𝐩₁ + u * (𝐩₂ - 𝐩₁)
        return 𝐩
    else
        return nothing
    end
end
