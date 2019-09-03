using ComputerVision
using Test
using StaticArrays
using GeometryTypes
using LinearAlgebra
using PGFPlotsX
using Makie
using Colors


# Test synthetic planar scene generation
# rois = [HyperRectangle(Vec(0.0, 0.0), Vec(50.0, 50.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(80.0, 50.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(50.0, 80.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0)),
#         HyperRectangle(Vec(0.0, 0.0), Vec(80.0, 80.0))]
rois = [HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0)),
        HyperRectangle(Vec(0.0, 0.0), Vec(180.0, 150.0)),
        HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 180.0)),
        HyperRectangle(Vec(0.0, 0.0), Vec(250.0, 250.0)),
        HyperRectangle(Vec(0.0, 0.0), Vec(80.0, 80.0))]
#pts = [8, 12, 16, 20, 24]
pts = [24, 20, 16, 12, 8]
planar_scene = PlanarSyntheticScene(total_planes = 5, regions_of_interest = rois, points_per_region = pts)
synthetic_scene_context = SyntheticSceneContext(planar_scene)

# world, cameras = synthetic_scene_context()
# camera₁, camera₂ = cameras


# axis = Scene()
# visualize = VisualizeWorldContext(; scene = axis, visual_properties = MakieVisualProperties(scale = 150, markersize = 25))
# visualize(world, cameras)
# scene = get_scene(visualize)
# display(scene)

#axis₂ = @pgf PGFPlotsX.Axis({axis_equal="true", view="{10}{45}", axis_equal_image="false"});
# axis₂ = @pgf PGFPlotsX.Axis({plot_box_ratio = "1 1 1", unit_vector_ratio="1 1 1", unit_rescale_keep_size="true", width="500", height="500",view="{15}{-15}"});
# visualize₂ = VisualizeWorldContext(; scene = axis₂, visual_properties = PGFPlotsVisualProperties(scale = 150, markersize = 100))
# visualize₂(world, [camera₁, camera₂])
# scene₂ = get_scene(visualize₂)
# display(scene₂)

apply_noise_context = ApplyNoiseContext()
correspondences = apply_noise_context(world, camera₁, camera₂)

conduct_experiment = ExperimentContext(synthetic_scene_context, apply_noise_context, EstimateHomographyContext())
estimates, runtimes = conduct_experiment(DirectLinearTransform())

analyze_experiment = ExperimentAnalysisContext(ReconstructionErrorAnalysis())

analyze_experiment(conduct_experiment, DirectLinearTransform(), estimates, runtimes)

#(context::ExperimentAnalysisContext)(experiment_context::ExperimentContext, algorithm::AbstractProjectiveEstimationAlgorithm, estimates::AbstractVector, runtimes::AbstractVector)
apply_noise_context
