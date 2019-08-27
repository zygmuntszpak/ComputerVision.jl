abstract type AbstractPose end
abstract type AbstractCoordinateTransformation end

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
RelativePose(coordinate_system₁::CartesianSystem, coordinate_system₂::CartesianSystem) = RelativePose(construct_relative_pose(coordinate_system₁, coordinate_system₂)...)


function construct_relative_pose(camera₁::AbstractCamera, camera₂::AbstractCamera)
    camera_model₁ = get_model(camera₁)
    camera_model₂ = get_model(camera₂)
    extrinsics₁ = get_extrinsics(camera_model₁)
    extrinsics₂ = get_extrinsics(camera_model₂)
    camera_system₁ = get_coordinate_system(extrinsics₁)
    camera_system₂ = get_coordinate_system(extrinsics₂)
    construct_relative_pose(camera_system₁, camera_system₂)
    # 𝐞₁ = get_e₁(camera_system₁)
    # 𝐞₂ = get_e₂(camera_system₁)
    # 𝐞₃ = get_e₃(camera_system₁)
    # 𝐞₁′ = get_e₁(camera_system₂)
    # 𝐞₂′ = get_e₂(camera_system₂)
    # 𝐞₃′ = get_e₃(camera_system₂)
    # 𝐭 = get_origin(camera_system₁) - get_origin(camera_system₂)
    # 𝐑 = inv(hcat(𝐞₁, 𝐞₂, 𝐞₃)) * hcat(𝐞₁′, 𝐞₂′, 𝐞₃′)
    # 𝐑, 𝐭
end

# system 2 with respect to system 1
function construct_relative_pose(coordinate_system₁::AbstractCoordinateSystem,  coordinate_system₂::AbstractCoordinateSystem)
    𝐞₁ = get_e₁(coordinate_system₁)
    𝐞₂ = get_e₂(coordinate_system₁)
    𝐞₃ = get_e₃(coordinate_system₁)
    𝐞₁′ = get_e₁(coordinate_system₂)
    𝐞₂′ = get_e₂(coordinate_system₂)
    𝐞₃′ = get_e₃(coordinate_system₂)
    # Mistake, change order TODO
    𝐭 = get_origin(coordinate_system₂) - get_origin(coordinate_system₁)
    𝐑 = inv(hcat(𝐞₁, 𝐞₂, 𝐞₃)) * hcat(𝐞₁′, 𝐞₂′, 𝐞₃′)
    𝐑, 𝐭
end

Base.@kwdef struct CoordinateTransformation{T₁ <: AbstractCoordinateSystem, T₂ <: AbstractCoordinateSystem} <: AbstractCoordinateTransformation
    source::T₁ = CartesianSystem()
    target::T₂ = CartesianSystem()
    relative_pose = RelativePose(source, target)
end

#function target(CoordinateTransformation)

function rotation(transformation::CoordinateTransformation)
    rotation(transformation.relative_pose)
end

function translation(transformation::CoordinateTransformation)
    translation(transformation.relative_pose)
end



# function get_transformation()
#
# function get_rotation()
# end
#
# function get_translation()
# end
