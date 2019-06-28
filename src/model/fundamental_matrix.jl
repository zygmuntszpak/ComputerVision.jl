struct FundamentalMatrix{T₁ <: AbstractMatrix} <: ProjectiveEntity
    𝐅::T₁
end

function matrix(entity::FundamentalMatrix)
    entity.𝐅
end

FundamentalMatrix(camera₁::AbstractCamera, camera₂::AbstractCamera) = FundamentalMatrix(camera₁, camera₂, CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
FundamentalMatrix(camera₁::AbstractCamera, camera₂::AbstractCamera, world_system::AbstractCoordinateSystem) = FundamentalMatrix(construct_fundamental_matrix(camera₁, camera₂, world_system))

#FundamentalMatrix(model₁::AbstractCameraModel, model₂::AbstractCameraModel) = FundamentalMatrix(model₁, model₂, CartesianSystem(Vec(1.0, 0.0, 0.0), Vec(0.0, 1.0, 0.0), Vec(0.0, 0.0, 1.0)))
#FundamentalMatrix(model₁::AbstractCameraModel, model₂::AbstractCameraModel, world_system::AbstractCoordinateSystem) = FundamentalMatrix(construct_fundamental_matrix(model₁, model₂, world_system))

FundamentalMatrix(P₁::Projection, P₂::Projection) = FundamentalMatrix(construct_fundamental_matrix(P₁, P₂))

function construct_fundamental_matrix(camera₁::AbstractCamera, camera₂::AbstractCamera,  world_system::AbstractCoordinateSystem)
    model₁ = get_model(camera₁)
    model₂ = get_model(camera₂)
    image_type₁ = get_image_type(camera₁)
    image_system₁ = get_coordinate_system(image_type₁)
    image_type₂ = get_image_type(camera₂)
    image_system₂ = get_coordinate_system(image_type₂)
    construct_fundamental_matrix(model₁, model₂,  world_system, image_system₁, image_system₂)
end

function construct_fundamental_matrix(model₁::AbstractCameraModel, model₂::AbstractCameraModel,  world_system::AbstractCoordinateSystem, image_system₁::AbstractPlanarCoordinateSystem, image_system₂::AbstractPlanarCoordinateSystem)
    @show world_system

    intrinsics₁ = get_intrinsics(model₁)
    𝐊₁ = to_matrix(intrinsics₁, image_system₁)
    extrinsics₁ = get_extrinsics(model₁)
    𝐑₁′, 𝐭₁′ = ascertain_pose(extrinsics₁, world_system)
    𝐑₁ = transpose(𝐑₁′)
    𝐭₁ = 𝐭₁′
    # Our projection matrix should decompose as [𝐑 -𝐑*𝐭]

    intrinsics₂ = get_intrinsics(model₂)
    𝐊₂ = to_matrix(intrinsics₂, image_system₂)
    extrinsics₂ = get_extrinsics(model₂)
    𝐑₂′, 𝐭₂′ = ascertain_pose(extrinsics₂, world_system)
    # Our projection matrix should decompose as [𝐑 -𝐑*𝐭]
    𝐑₂ = transpose(𝐑₂′)
    𝐭₂ = 𝐭₂′

    𝐅 = vec2antisym(𝐊₂*𝐑₂*(𝐭₁ - 𝐭₂))*𝐊₂*𝐑₂/𝐑₁/𝐊₁
end

function construct_fundamental_matrix(P₁::Projection, P₂::Projection)
    𝐏₁ = to_matrix(P₁)
    𝐏₂ = to_matrix(P₂)
    𝐜₁ = SVector{4,Float64}(nullspace(Array(𝐏₁)))
    𝐞₂ = 𝐏₂*𝐜₁
    𝐅 = vec2antisym(𝐞₂)*𝐏₂*pinv(𝐏₁)
    SMatrix{3,3,Float64,3*3}(𝐅)
end

# 𝐏₁ = Projection(model₁, world_system, image_system)
# 𝐏₂ = Projection(model₁, world_system, image_system)
