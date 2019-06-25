struct Homography{T₁ <: AbstractMatrix} <: ProjectiveEntity
    𝐇::AbstractMatrix
end

function to_matrix(entity::Homography)
    entity.𝐇
end
