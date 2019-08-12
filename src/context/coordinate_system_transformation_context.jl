abstract type AbstractCoordinateTransformationContext <: AbstractContext end


struct WorldCoordinateTransformationContext{T <: CoordinateTransformation} <: AbstractCoordinateTransformationContext
     coordinate_transformation::T
end

function (context::WorldCoordinateTransformationContext)(camera::AbstractCamera)
    𝐑 = rotation(context.coordinate_transformation)
    𝐭 = translation(context.coordinate_transformation)
    relocate!(camera, 𝐑',𝐭)
end
