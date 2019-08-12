struct Projection{T₁ <: AbstractMatrix} <: ProjectiveEntity
    𝐏::T₁
end

function to_matrix(entity::Projection)
    entity.𝐏
end

function matrix(entity::Projection)
    entity.𝐏
end

Projection(camera::AbstractCamera) = Projection(camera, CartesianSystem(Point(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
Projection(camera::AbstractCamera, reference_system::AbstractCoordinateSystem) = Projection(construct_projection(camera, reference_system))


#Projection(model::AbstractCameraModel) = Projection(model, CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)), PlanarCartesianSystem(Vec(-1.0, 0.0), Vec(0.0, -1.0)))
#Projection(model::AbstractCameraModel, reference_system::AbstractCoordinateSystem, image_system::AbstractPlanarCoordinateSystem) = Projection(construct_projection(model, reference_system, image_system))

function project(P::Projection, 𝒳::Vector{<: AbstractVector})
    𝐏 = to_matrix(P)
    ℳ = map(𝒳) do 𝐗
        𝐦 = hom⁻¹(𝐏 * hom(𝐗))
    end
    return ℳ
end

function construct_projection(camera::AbstractCamera, reference_system::AbstractCoordinateSystem)
    model = get_model(camera)
    image_type = get_image_type(camera)
    image_system = get_coordinate_system(image_type)
    construct_projection(model, reference_system, image_system)
end

function construct_projection(model::AbstractCameraModel, reference_system::AbstractCoordinateSystem, image_system::AbstractPlanarCoordinateSystem)
    intrinsics = get_intrinsics(model)
    extrinsics = get_extrinsics(model)
    𝐊 = to_matrix(intrinsics, image_system)
    𝐄 = to_matrix(extrinsics, reference_system)
    𝐏 = 𝐊 * 𝐄
end

function to_matrix(intrinsics::IntrinsicParameters, image_system::AbstractPlanarCoordinateSystem)
    f = get_focal_length(intrinsics)
    optical_system = get_coordinate_system(intrinsics)
    𝐞₁ = get_e₁(image_system)
    𝐞₂ = get_e₂(image_system)
    𝐞₁′ = get_e₁(optical_system)
    𝐞₂′ = get_e₂(optical_system)
    # TODO Fix this so that we don't assume that the principal point is at position (0,0)
    # 𝐩 = get_principal_point(intrinsics)
    𝐭 = determine_translation(intrinsics, image_system)
    𝐑 = inv(hcat(𝐞₁, 𝐞₂)) * hcat(𝐞₁′ , 𝐞₂′)
    𝐊 = SMatrix{3,3,Float64,9}(f, 0.0, 0.0, 0.0, f, 0.0, 0.0, 0.0 , 1)
    𝐊′ = vcat(hcat(𝐑', -𝐑'*𝐭), SMatrix{1,3,Float64}(0,0,1)) * 𝐊
end

function to_matrix(extrinsics::ExtrinsicParameters, reference_system::CartesianSystem)
    𝐑, 𝐭 = ascertain_pose(extrinsics, reference_system)
    𝐄 = [𝐑' -𝐑'*𝐭]
end

function ascertain_pose(camera::AbstractCamera, reference_system::CartesianSystem)
    model = get_model(camera)
    ascertain_pose(get_extrinsics(model), reference_system)
end

function ascertain_pose(extrinsics::ExtrinsicParameters, reference_system::CartesianSystem)
    camera_system = get_coordinate_system(extrinsics)
    𝐞₁ = get_e₁(reference_system)
    𝐞₂ = get_e₂(reference_system)
    𝐞₃ = get_e₃(reference_system)
    𝐞₁′ = get_e₁(camera_system)
    𝐞₂′ = get_e₂(camera_system)
    𝐞₃′ = get_e₃(camera_system)
    𝐭 = get_origin(camera_system) - get_origin(reference_system)
    𝐑 = inv(hcat(𝐞₁, 𝐞₂, 𝐞₃)) * hcat(𝐞₁′, 𝐞₂′, 𝐞₃′)
    𝐑, 𝐭
end

# TODO Incorporate information about the origin of the image coordinate system
function determine_translation(intrinsics::IntrinsicParameters, system::PlanarCartesianSystem)
    width = get_width(intrinsics)
    height = get_height(intrinsics)
    𝐭 = Point(-width / 2, height / 2)
end

function determine_translation(intrinsics::IntrinsicParameters, system::OpticalSystem)
    𝐭 = Point(0, 0)
end

function determine_translation(intrinsics::IntrinsicParameters, system::RasterSystem)
    width = get_width(intrinsics)
    height = get_height(intrinsics)
    𝐭 = Point(-width / 2, -height / 2)
end
