

function construct_simulation_data(setup::VaryPlanesSimulationSetup)
    control_variable = Vector(undef, length(setup.planes))
    for k in 1:length(setup.planes)
        P = setup.planes[k]
        trials = Vector(undef, setup.trials)
        for t = 1:setup.trials
            dimensions = setup.roi_dimensions[k]
            rois = [HyperRectangle(Vec(0.0, 0.0), Vec(dimensions[s], dimensions[s])) for s = 1:length(dimensions)]
            pts = setup.points_per_roi[k]
            noise = setup.noise_per_roi[k]
            planar_scene_parameters = PlanarSceneParameters(total_planes = P, regions_of_interest = rois, points_per_region = pts)
            synthetic_scene = SyntheticScene(planar_scene_parameters)
            # TODO Extend ApplyNoiseContext so that it can apply different noise levels to different regions of interest in the image
            apply_noise_context = ApplyNoiseContext(perturb = HomogeneousGaussianNoise(noise[1]), replications = setup.replications)
            experiment_context = ExperimentContext(synthetic_scene, EstimateHomographyContext(), apply_noise_context)
            trials[t] = experiment_context
        end
        control_variable[k] = trials
    end
    control_variable
end

function execute_simulation(experiment_conditions::AbstractVector, algorithm::AbstractEstimationAlgorithm)
    results_per_condition = Vector(undef, length(experiment_conditions))

    for (i, condition) in enumerate(experiment_conditions)
        results_per_trial = Vector(undef, length(condition))
        for (j, trial) in enumerate(condition)
            outcome = trial(algorithm)
            results_per_trial[j] = outcome
        end
        results_per_condition[i] = results_per_trial
    end
    results_per_condition
end

function analyze_simulation(experiment_conditions::AbstractVector, experiment_estimates::AbstractVector, error_analysis::AbstractErrorAnalysis)
    errors_per_condition = Vector(undef, length(experiment_conditions))

    for ((i₀, condition₀), (i₁, condition₁)) in zip(enumerate(experiment_conditions), enumerate(experiment_estimates))
         errors_per_trial = Vector(undef, length(condition₁))
        for ((j₀, experiment_context), (j₁, outcomes)) in zip(enumerate(condition₀), enumerate(condition₁))
            analyze_experiment = ExperimentAnalysisContext(experiment_context.synthetic_scene, error_analysis)
            errors = analyze_experiment(outcomes...)
            errors_per_trial[j₀] = errors
        end
        errors_per_condition[i₀] = errors_per_trial
    end
    errors_per_condition
end

# rois = [HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0)), HyperRectangle(Vec(0.0, 0.0), Vec(150.0, 150.0))]
# #pts = [8, 12, 16, 20, 24]
# pts = [24, 24]
#
# #planar_scene = PlanarSyntheticScene(total_planes = 5, regions_of_interest = rois, points_per_region = pts)
# planar_scene_parameters = PlanarSceneParameters(total_planes = 2, regions_of_interest = rois, points_per_region = pts)
#
# synthetic_scene = SyntheticScene(planar_scene_parameters)
#
# apply_noise_context = ApplyNoiseContext(perturb = HomogeneousGaussianNoise(0.1), replications = 10)
# conduct_experiment_context = ExperimentContext(synthetic_scene, apply_noise_context, EstimateHomographyContext())
# estimates, runtimes = conduct_experiment_context(DirectLinearTransform())
# analyze_experiment = ExperimentAnalysisContext(synthetic_scene, apply_noise_context, ReconstructionErrorAnalysis())
# results = analyze_experiment(DirectLinearTransform(), estimates, runtimes)

# total_scenes = 15
# replications_per_scene = 10
# noise_level = 0.0
# total_planes = 2
# roi_sizes = [250.0, 200.0, 150.0, 100.0, 50.0]
# total_pts = 24
