abstract type AbstractContext end
struct TwoViewContext <: AbstractContext end
struct TwoViewExperimentContext <: TwoViewContext end
struct AquireImageContext <: AbstractContext end

function (aquire::AquireImageContext)(world::PrimitiveWorld, camera::AbstractCamera)
    @show "Project world into camera"
end
