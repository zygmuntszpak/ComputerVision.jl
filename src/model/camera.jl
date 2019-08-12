
abstract type AbstractCamera end

abstract type AbstractCameraModel end

abstract type AbstractIntrinsicParameters end
abstract type AbstractExtrinsicParameters end

# TODO revisit principal point and origin
Base.@kwdef mutable struct IntrinsicParameters <: AbstractIntrinsicParameters
    focal_length::Float64 = 50
    width::Int = 1000
    height::Int = 1000
    # Origin of the picture plane (the image) is the optical axis.
    #origin::Point{2,Float64} = Point(0.0, 0.0)
    # The principal point offset with respect to te Optical Axis coordinate system
    principal_point::Point{2,Float64} = Point(0.0, 0.0)
    # Basis vectors that characterise the coordinate system of the
    # picture plane (the image).
    coordinate_system = OpticalSystem()
    # 𝐞₁::Vec{2,Float64} = Vec(-1.0, 0.0)
    # 𝐞₂::Vec{2,Float64} = Vec(0.0, -1.0)
end

function to_matrix(intrinsics::IntrinsicParameters)
    f = get_focal_length(intrinsics)
    𝐩 = get_principal_point(intrinsics)
    𝐊 = @SMatrix [f 0 𝐩[1]; 0 f 𝐩[2]; 0 0 1]
end

function get_focal_length(param::IntrinsicParameters)
    param.focal_length
end

function set_focal_length!(param::IntrinsicParameters, focal_length::Float64)
    param.focal_length = focal_length
end

function get_width(param::IntrinsicParameters)
     param.width
end

function set_width!(param::IntrinsicParameters, width::Int)
    param.width = width
end

function get_height(param::IntrinsicParameters)
     param.height
end

function set_height!(param::IntrinsicParameters, height::Int)
    param.height = height
end

# function get_origin(param::IntrinsicParameters)
#     param.origin
# end
#
# function set_origin!(param::IntrinsicParameters, origin::Point{2,Float64})
#     param.origin = origin
# end

function get_principal_point(param::IntrinsicParameters)
    param.principal_point
end

function set_principal_point!(param::IntrinsicParameters, principal_point::Point{2,Float64})
    param.principal_point = principal_point
end

function get_coordinate_system(param::IntrinsicParameters)
    param.coordinate_system
end

function set_coordinate_system!(param::IntrinsicParameters, coordinate_system::AbstractPlanarCoordinateSystem)
    param.coordinate_system = coordinate_system
end

#
# function get_e₁(param::IntrinsicParameters)
#     param.e₁
# end
#
# function set_e₁!(param::IntrinsicParameters, e₁::Vec{2,Float64})
#     param.e₁ = e₁
# end
#
# function get_e₂(param::IntrinsicParameters)
#     param.e₂
# end
#
# function set_e₂!(param::IntrinsicParameters, e₂::Vec{2,Float64})
#     param.e₂ = e₂
# end

Base.@kwdef mutable struct ExtrinsicParameters <: AbstractExtrinsicParameters
    # Center of projection.
    #centroid::Point{3,Float64} = Point(0.0, 0.0, 0.0)
    # Basis vectors that characterise the pose of the camera
    coordinate_system = CartesianSystem(Point(0.0, 0.0, 0.0), Vec(-1.0, 0.0, 0.0), Vec(0.0, -1.0, 0.0), Vec(0.0, 0.0, 1.0))
    # 𝐞₁::Vec{3,Float64} = Vec(-1.0, 0.0, 0.0)
    # 𝐞₂::Vec{3,Float64} = Vec(0.0, -1.0, 0.0)
    # 𝐞₃::Vec{3,Float64} = Vec(0.0, 0.0, 1.0)
end


function get_origin(param::ExtrinsicParameters)
    get_origin(param.coordinate_system)
end

# function set_centroid!(param::ExtrinsicParameters, centroid::Point{3,Float64})
#     𝐞₁ = get_e₁(param.coordinate_system)
#     𝐞₂ = get_e₂(param.coordinate_system)
#     𝐞₃ = get_e₃(param.coordinate_system)
#     set_coordinate_system!(param, CartesianSystem(centroid, 𝐞₁, 𝐞₂, 𝐞₃))
# end

# function set_basis_vectors!(param::ExtrinsicParameters, 𝐞₁::Vec{3,Float64}, 𝐞₂::Vec{3,Float64}, 𝐞₃::Vec{3,Float64})
#     centroid = get_origin(param)
#     set_coordinate_system!(param, CartesianSystem(centroid, 𝐞₁, 𝐞₂, 𝐞₃))
# end
#
function get_basis_vectors(param::ExtrinsicParameters)
    𝐞₁ = get_e₁(param.coordinate_system)
    𝐞₂ = get_e₂(param.coordinate_system)
    𝐞₃ = get_e₃(param.coordinate_system)
    return 𝐞₁, 𝐞₂, 𝐞₃
end

function get_coordinate_system(param::ExtrinsicParameters)
    param.coordinate_system
end

function set_coordinate_system!(param::ExtrinsicParameters, coordinate_system::AbstractCoordinateSystem)
    param.coordinate_system = coordinate_system
end

#
# function get_e₁(param::ExtrinsicParameters)
#     param.e₁
# end
#
# function set_e₁!(param::ExtrinsicParameters, e₁::Vec{3,Float64})
#     param.e₁ = e₁
# end
#
# function get_e₂(param::ExtrinsicParameters)
#     param.e₂
# end
#
# function set_e₂!(param::ExtrinsicParameters, e₂::Vec{3,Float64})
#     param.e₂ = e₂
# end
#
# function get_e₃(param::ExtrinsicParameters)
#     param.e₃
# end
#
# function set_e₃!(param::ExtrinsicParameters, e₃::Vec{3,Float64})
#     param.e₃ = e₃
# end

Base.@kwdef mutable struct  Pinhole{T₁ <: AbstractIntrinsicParameters, T₂ <: AbstractExtrinsicParameters} <: AbstractCameraModel
    intrinsics::T₁ = IntrinsicParameters()
    extrinsics::T₂ = ExtrinsicParameters()
end

function get_intrinsics(model::AbstractCameraModel)
    model.intrinsics
end

function set_intrinsics!(model::AbstractCameraModel, intrinsics::AbstractIntrinsicParameters)
    model.intrinsics = intrinsics
end

function get_extrinsics(model::AbstractCameraModel)
    model.extrinsics
end

function set_extrinsics!(model::AbstractCameraModel, extrinsics::AbstractExtrinsicParameters)
    model.extrinsics = extrinsics
end

Base.@kwdef mutable struct Camera{T₁ <: AbstractCameraModel, T₂ <: AbstractImage} <: AbstractCamera
    model::T₁ = Pinhole()
    image_type::T₂ = AnalogueImage()
end

function get_model(camera::Camera)
    camera.model
end

function set_model!(camera::Camera, model::AbstractCameraModel)
    camera.model = model
end

function get_image_type(camera::Camera)
    camera.image_type
end

function set_image_type!(camera::Camera, image_type::AbstractImage)
    camera.image_type = image_type
end


function rotate!(camera::Camera,  𝐑::AbstractArray)
    model = get_model(camera)
    extrinsics = get_extrinsics(model)
    # coordinate_system = get_coordinate_system(extrinsics)
    # 𝐞₁ = get_e₁(coordinate_system)
    # 𝐞₂ = get_e₂(coordinate_system)
    # 𝐞₃ = get_e₃(coordinate_system)
    𝐞₁, 𝐞₂, 𝐞₃ = get_basis_vectors(extrinsics)
    𝐞₁′ = 𝐑*𝐞₁
    𝐞₂′ = 𝐑*𝐞₂
    𝐞₃′ = 𝐑*𝐞₃
    𝐨 = get_origin(extrinsics)
    set_coordinate_system!(extrinsics, CartesianSystem(𝐨, 𝐞₁′,𝐞₂′,𝐞₃′))
end

function translate!(camera::Camera, 𝐭::AbstractArray)
    model = get_model(camera)
    extrinsics = get_extrinsics(model)
    𝐨 = get_origin(extrinsics)
    𝐞₁, 𝐞₂, 𝐞₃ = get_basis_vectors(extrinsics)
    set_coordinate_system!(extrinsics, CartesianSystem(𝐨 + 𝐭, 𝐞₁, 𝐞₂, 𝐞₃))
    #𝐜 = get_centroid(extrinsics)
    #set_centroid!(extrinsics, 𝐜 + 𝐭)
end

function relocate!(camera::Camera, 𝐑::AbstractArray, 𝐭::AbstractArray)
    model = get_model(camera)
    extrinsics = get_extrinsics(model)
    𝐞₁, 𝐞₂, 𝐞₃ = get_basis_vectors(extrinsics)
    𝐞₁′ = 𝐑*𝐞₁
    𝐞₂′ = 𝐑*𝐞₂
    𝐞₃′ = 𝐑*𝐞₃
    𝐨 = get_origin(extrinsics)
    set_coordinate_system!(extrinsics, CartesianSystem(𝐨 + 𝐭, 𝐞₁′,𝐞₂′,𝐞₃′))
    #rotate!(camera, 𝐑)
    #translate!(camera, 𝐭)
end#
