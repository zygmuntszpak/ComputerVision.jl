function hom⁻¹(v::StaticVector)
    if isapprox(v[end], 0.0; atol = 1e-14)
        pop(v)
    else
        pop(v / v[end])
    end
end

function hom(v::StaticVector)
    push(v,1)
end
