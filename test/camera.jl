using GeometryTypes
using ComputerVision
using Test

digital_camera = DigitalCamera()

camera_model = get_model(digital_camera)
pictures = get_pictures(digital_camera)
@test typeof(camera_model) <: AbstractCameraModel
@test isnothing(pictures)

intrinsics = get_intrinsics(camera_model)
@test typeof(intrinsics) <: AbstractIntrinsicParameters
f = get_focal_length(intrinsics)
@test f == 50
@test @inferred get_focal_length(intrinsics) == 50
