Base.@kwdef mutable struct SummarizeExperimentContext{T₀ <: Union{AbstractPlotting.Scene, PGFPlotsX.Axis}} <: AbstractContext
    scene::T₀ = Scene()
end

function get_scene(context::SummarizeExperimentContext)
    context.scene
end

function set_scene!(context::SummarizeExperimentContext, scene::AbstractPlotting.Scene)
    context.scene = scene
end


function (summarize::SummarizeExperimentContext)(setup::VaryPlanesSimulationSetup, experiment_results::AbstractVector, error_analysis::AbstractErrorAnalysis)
    # Determine the total number of conditions
    C = length(setup.planes)
    mean_mean_mean_rmse_per_condition = Vector{Float64}(undef,C)
    for c = 1:C
        condition = experiment_results[c]
        T = length(condition)
        mean_mean_rmse_per_trial = Vector{Float64}(undef,T)
        for t = 1:T
            mean_mean_rmse_per_trial[t] = mean(mean(condition[t]))
        end
        mean_mean_mean_rmse_per_condition[c] = mean(mean_mean_rmse_per_trial)
    end
    mean_mean_mean_rmse_per_condition
end
