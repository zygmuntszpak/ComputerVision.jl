struct LatentVariables{T <: AbstractVector}
    variables::T
end

function variables(L::LatentVariables)
    L.variables
end
