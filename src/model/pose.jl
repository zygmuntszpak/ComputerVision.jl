abstract type AbstractPose end

struct RelativePose{T₁ <: AbstractMatrix, T₂ <: AbstractVector} <: AbstractPose
    rotation::T₁
    translation::T₂
end

function rotation(pose::AbstractPose)
    pose.rotation
end

function translation(pose::AbstractPose)
    pose.translation
end


RelativePose(camera₁::AbstractCamera, camera₂::AbstractCamera) = RelativePose(construct_relative_pose(camera₁, camera₂)...)


function construct_relative_pose(camera₁, camera₂)
    camera_model₁ = get_model(camera₁)
    camera_model₂ = get_model(camera₂)
    extrinsics₁ = get_extrinsics(camera_model₁)
    extrinsics₂ = get_extrinsics(camera_model₂)
    camera_system₁ = get_coordinate_system(extrinsics₁)
    camera_system₂ = get_coordinate_system(extrinsics₂)
    𝐞₁ = get_e₁(camera_system₁)
    𝐞₂ = get_e₂(camera_system₁)
    𝐞₃ = get_e₃(camera_system₁)
    𝐞₁′ = get_e₁(camera_system₂)
    𝐞₂′ = get_e₂(camera_system₂)
    𝐞₃′ = get_e₃(camera_system₂)
    𝐭 = get_centroid(extrinsics₂) - get_centroid(extrinsics₁)
    𝐑 = inv(hcat(𝐞₁, 𝐞₂, 𝐞₃)) * hcat(𝐞₁′, 𝐞₂′, 𝐞₃′)
    𝐑, 𝐭
end
