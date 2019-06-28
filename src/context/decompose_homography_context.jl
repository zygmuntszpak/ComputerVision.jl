abstract type AbstractHomographyDecomposition end

struct MalisVargasDecomposition <: AbstractHomographyDecomposition end
struct FaugerasDecomposition <: AbstractHomographyDecomposition end


struct PoseFromSingleHomographyContext{T₁ <: IntrinsicParameters, T₂ <: AbstractAnalogueImage, T₃ <: AbstractHomographyDecomposition} <: AbstractContext
     intrinsics::T₁
     image_type::T₂
     algorithm::T₃
end

function (context::PoseFromSingleHomographyContext)(homography::HomographyMatrix)
    𝐊 = to_matrix(context.intrinsics, get_coordinate_system(context.image_type))
    𝐇 = matrix(homography)
    context.algorithm(𝐇, 𝐊)
    #estimate(data, estimator, context)
end

function (context::PoseFromSingleHomographyContext)(homography::HomographyMatrix, correspondences::AbstractCorrespondences)
    𝐊 = to_matrix(context.intrinsics, get_coordinate_system(context.image_type))
    𝐇 = matrix(homography)
    poses = context.algorithm(𝐇, 𝐊)
    two_poses = apply_reference_point_visibility_constraint(poses, 𝐊 , correspondences)
    return two_poses
end

function apply_reference_point_visibility_constraint(putative_poses::AbstractVector, 𝐊::AbstractMatrix, correspondences::AbstractCorrespondences)
    # We ought to be able to reduce 4 putative solutions down to 2 putative solutions
    # based on the constraint that the points and planar surface need to be in front of the cameras.
    mask = [false, false, false, false]
    ℳ = correspondences[1]
    ℳ′ = correspondences[2]
    N = length(ℳ)
    𝐊⁻¹ = inv(𝐊)
    for (k, pose) in enumerate(putative_poses)
        𝐧 = last(pose)
        v = [dot(𝐊⁻¹ * hom(ℳ[i]), 𝐧) > 0  for i = 1:N]
        mask[k] = all(v)
    end
    return putative_poses[mask]
end

function (algorithm::MalisVargasDecomposition)(𝐇′::AbstractArray, 𝐊::AbstractArray)
    𝐇 =  construct_euclidean_homography(𝐇′, 𝐊)
    # TODO Check if 𝐇 is a rotation matrix and handle this special case.
    𝐈 = UniformScaling(1)
    𝐒 = 𝐇'*𝐇 - 𝐈
    i  = argmax(abs.(SVector(𝐒[1,1], 𝐒[2,2], 𝐒[3,3])))

    𝐧ₐ′, 𝐧ᵦ′, i = develop_normals(𝐒, i)
    𝐧ₐ = 𝐧ₐ′ / norm(𝐧ₐ′)
    𝐧ᵦ = 𝐧ᵦ′ / norm(𝐧ᵦ′)

    M𝐬₁₁ = M𝐬ᵢⱼ(𝐒, 1, 1)
    M𝐬₂₂ = M𝐬ᵢⱼ(𝐒, 2, 2)
    M𝐬₃₃ = M𝐬ᵢⱼ(𝐒, 3, 3)
    ν = 2 * sqrt(1 + tr(𝐒) - M𝐬₁₁ - M𝐬₂₂ - M𝐬₃₃)
    ϵ = sign₀(𝐒[i,i])

    𝐭ₐ′, 𝐭ᵦ′ = develop_translations(𝐒, 𝐧ₐ, 𝐧ᵦ, ν, ϵ)

    𝐑ₐ = 𝐇 * (𝐈 - (2/ν) * (𝐭ₐ′ * 𝐧ₐ'))
    𝐑ᵦ = 𝐇 * (𝐈 - (2/ν) * (𝐭ᵦ′ * 𝐧ᵦ'))

    # Ensure that we have a proper rotation (det(R)  = 1 )
    𝐑ₐ = 𝐑ₐ * sign(det(𝐑ₐ))
    𝐑ᵦ = 𝐑ᵦ * sign(det(𝐑ᵦ))

    𝐭ₐ = 𝐑ₐ * 𝐭ₐ′
    𝐭ᵦ = 𝐑ᵦ * 𝐭ᵦ′

    [(𝐑ₐ, 𝐭ₐ, 𝐧ₐ), (𝐑ᵦ, 𝐭ᵦ, 𝐧ᵦ), (𝐑ₐ, -𝐭ₐ, -𝐧ₐ), (𝐑ᵦ, -𝐭ᵦ, -𝐧ᵦ)]
end

function construct_euclidean_homography(𝐇′::AbstractArray, 𝐊::AbstractArray)
    𝐇 = inv(𝐊) * 𝐇′ * 𝐊
    F = svd(𝐇)
    γ = F.S[2]
    return 𝐇 * (1 / γ)
end

function develop_normals(𝐒::AbstractArray, i::Number)
    s₁₁ = 𝐒[1,1]
    s₁₂ = 𝐒[1,2]
    s₁₃ = 𝐒[1,3]
    s₂₂ = 𝐒[2,2]
    s₂₃ = 𝐒[2,3]
    s₃₃ = 𝐒[3,3]
    if i == 1
        M𝐬₃₃ = M𝐬ᵢⱼ(𝐒, 3, 3)
        M𝐬₂₂ = M𝐬ᵢⱼ(𝐒, 2, 2)
        M𝐬₂₃ = M𝐬ᵢⱼ(𝐒, 2, 3)
        ϵ₂₃ = sign₀(M𝐬₂₃)
        𝐧ₐ = SVector(s₁₁, s₁₂ + √M𝐬₃₃, s₁₃ + ϵ₂₃*√M𝐬₂₂)
        𝐧ᵦ = SVector(s₁₁, s₁₂ - √M𝐬₃₃, s₁₃ - ϵ₂₃*√M𝐬₂₂)
        return 𝐧ₐ, 𝐧ᵦ, i
    elseif i == 2
        M𝐬₃₃ = M𝐬ᵢⱼ(𝐒, 3, 3)
        M𝐬₁₁ = M𝐬ᵢⱼ(𝐒, 1, 1)
        M𝐬₁₃ = M𝐬ᵢⱼ(𝐒, 1, 3)
        ϵ₁₃ = sign₀(M𝐬₁₃)
        𝐧ₐ = SVector(s₁₂ + √M𝐬₃₃, s₂₂, s₂₃ - ϵ₁₃*√M𝐬₁₁)
        𝐧ᵦ = SVector(s₁₂ - √M𝐬₃₃, s₂₂, s₂₃ + ϵ₁₃*√M𝐬₁₁)
        return 𝐧ₐ, 𝐧ᵦ, i
    else
        M𝐬₁₂ = M𝐬ᵢⱼ(𝐒, 1, 2)
        M𝐬₂₂ = M𝐬ᵢⱼ(𝐒, 2, 2)
        M𝐬₁₁ = M𝐬ᵢⱼ(𝐒, 1, 1)
        ϵ₁₂ = sign₀(M𝐬₁₂)
        𝐧ₐ = SVector(s₁₃ + ϵ₁₂*√M𝐬₂₂, s₂₃ + √M𝐬₁₁, s₃₃)
        𝐧ᵦ = SVector(s₁₃ - ϵ₁₂*√M𝐬₂₂, s₂₃ - √M𝐬₁₁, s₃₃)
        return 𝐧ₐ, 𝐧ᵦ, i
    end
end

function develop_translations(𝐒::AbstractArray, 𝐧ₐ::AbstractArray, 𝐧ᵦ::AbstractArray, ν::Number, ϵ::Number)
    tₑ = sqrt(2 + tr(𝐒) - ν)
    ρ = sqrt(2 + tr(𝐒) + ν)
    𝐭ₐ′ =  (tₑ/2) * (ϵ * ρ * 𝐧ᵦ -  tₑ*𝐧ₐ)
    𝐭ᵦ′ =  (tₑ/2) * (ϵ * ρ * 𝐧ₐ -  tₑ*𝐧ᵦ)
    return 𝐭ₐ′, 𝐭ᵦ′
end


function M𝐬ᵢⱼ(𝐒, i ,j)
    -det(minor(𝐒, i , j))
end

function sign₀(x::Number)
    x >= 0 ? 1 : -1
end

# function (algorithm::FaugerasDecomposition)(𝐇::AbstractArray, 𝐊::AbstractArray)
#
# end
