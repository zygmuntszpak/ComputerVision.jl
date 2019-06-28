struct EssentialMatrix{T₁ <: AbstractMatrix} <: ProjectiveEntity
    𝐄::T₁
end

function matrix(entity::EssentialMatrix)
    entity.𝐄
end

EssentialMatrix(camera₁::AbstractCamera, camera₂::AbstractCamera) = EssentialMatrix(camera₁, camera₂, CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
EssentialMatrix(camera₁::AbstractCamera, camera₂::AbstractCamera, world_system::AbstractCoordinateSystem) = EssentialMatrix(construct_essential_matrix(camera₁, camera₂, world_system))

#FundamentalMatrix(model₁::AbstractCameraModel, model₂::AbstractCameraModel) = FundamentalMatrix(model₁, model₂, CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
#FundamentalMatrix(model₁::AbstractCameraModel, model₂::AbstractCameraModel, world_system::AbstractCoordinateSystem) = FundamentalMatrix(construct_fundamental_matrix(model₁, model₂, world_system))

#EssentialMatrix(P₁::Projection, P₂::Projection) = EssentialMatrix(construct_essential_matrix(P₁, P₂))

function construct_essential_matrix(camera₁::AbstractCamera, camera₂::AbstractCamera,  world_system::AbstractCoordinateSystem)
    model₁ = get_model(camera₁)
    model₂ = get_model(camera₂)
    image_type₁ = get_image_type(camera₁)
    image_system₁ = get_coordinate_system(image_type₁)
    image_type₂ = get_image_type(camera₂)
    image_system₂ = get_coordinate_system(image_type₂)
    intrinsics₁ = get_intrinsics(model₁)
    intrinsics₂ = get_intrinsics(model₂)
    𝐊₁ = to_matrix(intrinsics₁, image_system₁)
    𝐊₂ = to_matrix(intrinsics₂, image_system₂)
    𝐅 = matrix(FundamentalMatrix(camera₁, camera₂, world_system))
    𝐄 = 𝐊₂'*𝐅*𝐊₁
end
