module ComputerVision

using StaticArrays, GeometryTypes, LinearAlgebra, Optim

include("util.jl")
include("model/image.jl")
include("model/camera.jl")
include("model/plane.jl")
include("model/world.jl")

export AbstractAnalgoueImage,
       AbstractDigitalImage,
       AbstractImage,
       AnalogueImage

#export  get_data

export  AbstractCamera,
        AbstractCameraModel,
        AbstractIntrinsicParameters,
        AbstractExtrinsicParameters,
        IntrinsicParameters,
        ExtrinsicParameters,
        Camera

export Plane,
       AbstractWorld,
       PrimitiveWorld


export  get_xaxis,
        set_xaxis!,
        get_yaxis,
        set_yaxis!,
        get_zaxis,
        set_zaxis!,
        get_points,
        set_points!,
        get_planes,
        set_planes!,
        get_normal,
        get_distance

export  hom⁻¹,
        hom

export  get_focal_length,
        set_focal_length!,
        get_width,
        set_width!,
        get_height!,
        set_height!,
        get_centroid,
        set_centroid,
        get_intrinsics,
        set_intrinsics,
        get_extrinsics,
        set_extrinsics,
        get_e₁,
        set_e₁!,
        get_e₂,
        set_e₂!,
        get_e₃,
        set_e₃!,
        get_origin,
        set_origin,
        get_model,
        set_model!,
        get_image_type,
        set_image_type!








end # module
