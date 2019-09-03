module ComputerVision

using StaticArrays, GeometryTypes, LinearAlgebra, Optim, Makie, PGFPlotsX, Colors, Statistics, OffsetArrays
using InplaceOps, LsqFit, ForwardDiff

abstract type ProjectiveEntity end

include("util.jl")
include("rotations.jl")
include("model/correspondence.jl")
include("model/coordinate_system.jl")
include("model/allotment.jl")
include("model/image.jl")
include("model/camera.jl")
include("model/plane.jl")
include("model/geometry.jl")
include("model/world.jl")
include("model/projection.jl")
include("model/pose.jl")
include("model/latent_variables.jl")
include("model/fundamental_matrix.jl")
include("model/essential_matrix.jl")
include("model/homography_matrix.jl")
include("model/cost.jl")
include("model/estimators.jl")
include("model/noise.jl")
include("model/synthetic_scene.jl")
include("model/simulation.jl")
#include("model/solver.jl")
include("view/visualize_properties.jl")
include("context/context.jl")
include("context/intersection_context.jl")
include("context/aquire_context.jl")
include("context/normalize_data_context.jl")
include("context/estimate_homography_context.jl")
include("context/decompose_homography_context.jl")
include("context/triangulate_context.jl")
include("context/coordinate_system_transformation_context.jl")
# include("context/synthetic_scene_context.jl") #TODO remove
include("context/apply_noise_context.jl")
include("context/experiment_context.jl")
include("context/analyze_experiment_context.jl")
include("context/visualize_context.jl")
include("context/summarize_experiment_context.jl")
include("context/evaluate_uncertainty_context.jl")
include("context/cost_context.jl")
include("simulation/simulation_util.jl")

export SummarizeExperimentContext

export UncertaintyContext,
       UnitNormGauge

export CostContext

export ApproximateMaximumLikelihood,
       ReprojectionError,
       AlgebraicLeastSquares,
       Mahalanobis

export AbstractSimulationSetup,
       VaryPlanesSimulationSetup,
       construct_simulation_data,
       execute_simulation,
       analyze_simulation

export AbstractCoordinateTransformationContext,
       WorldCoordinateTransformationContext,
       IntersectionContext

export ExperimentContext,
       ExperimentAnalysisContext,
       ReconstructionErrorAnalysis,
       ReprojectionErrorAnalysis,
       ParameterErrorAnalysis


# export SyntheticSceneContext,
#        PlanarSyntheticScene,
#        AbstractSyntheticScene
export SyntheticScene,
       PlanarSyntheticScene,
       AbstractSyntheticScene,
       AbstractSceneParameters,
       PlanarSceneParameters



export HomogeneousGaussianNoise,
       ApplyNoiseContext

export AbstractAllotment,
       IntervalAllotment,
       get_allotment,
       get_interval

export AbstractCorrespondences,
       Correspondences

export hartley_transformation,
       hartley_normalization

export FundamentalMatrix,
       EssentialMatrix,
       HomographyMatrix,
       HomographyMatrices,
       to_matrix,
       matrix,
       matrices

export TriangulateContext,
       AbstractTriangulationAlgorithm,
       DirectLinearTriangulation

export FundamentalNumericalScheme,
       UndampedVariant,
       DampedVariant

export AbstractPose,
       RelativePose,
       CoordinateTransformation,
       AbstractCoordinateTransformation,
       rotation,
       translation

export PoseFromSingleHomographyContext,
       MalisVargasDecomposition


export Projection,
       AquireImageContext,
       AbstractContext,
       NormalizeDataContext,
       HartleyNormalizeDataContext,
       AbstractEstimationContext,
       EstimateHomographyContext,
       VisualizeWorldContext,
       MakieVisualProperties,
       PGFPlotsVisualProperties,
       AbstractVisualProperties

export DirectLinearTransform,
       ImplicitChojnackiSzpak,
       ExplicitChojnackiSzpak,
       BundleAdjustment,
       ConstrainedBundleAdjustment,
       ConstrainedMahalanobis,
       ProjectiveEstimationAlgorithm

export project,
       back_project

export get_visualize_properties,
       set_scene!,
       get_scene,
       get_linewidth,
       set_linewidth!,
       get_markersize,
       set_markersize!,
       get_scale,
       set_scale!

export AbstractCoordinateSystem,
       AbstractPlanarCoordinateSystem,
       PlanarCartesianSystem,
       CartesianSystem,
       RasterSystem,
       OpticalSystem

export get_coordinate_system

export AbstractAnalgoueImage,
       AbstractDigitalImage,
       AbstractImage,
       AnalogueImage

#export  get_data

export LatentVariables

export  AbstractCamera,
        AbstractCameraModel,
        Pinhole,
        AbstractIntrinsicParameters,
        AbstractExtrinsicParameters,
        IntrinsicParameters,
        ExtrinsicParameters,
        Camera

export AbstractPlane,
       Plane,
       Line3D,
       PlaneSegment,
       EuclideanPlane3D,
       AbstractWorld,
       PrimitiveWorld,
       PlanarWorld


export  get_points,
        set_points!,
        get_planes,
        set_planes!,
        get_groups,
        set_groups,
        get_normal,
        get_distance,
        get_plane,
        set_plane,
        get_segment,
        on_plane,
        on_line
        distance,
        normal,
        origin

export  hom⁻¹,
        hom,
        vec2antisym,
        smallest_eigenpair

export  get_focal_length,
        set_focal_length!,
        get_width,
        set_width!,
        get_height!,
        set_height!,
        #get_centroid,
        #set_centroid!,
        get_origin,
        get_basis_vectors,
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
        rotate!,
        translate!,
        relocate!,
        rotxyz,
        #get_origin,
        #set_origin,
        get_model,
        set_model!,
        get_image_type,
        set_image_type!,
        ascertain_pose







end # module
