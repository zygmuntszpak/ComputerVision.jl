const  √ = sqrt
const  ∑ = sum
const  ⊗ = kron

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

function vec2antisym(v::AbstractArray)
    if length(v) != 3
         throw(ArgumentError("The operation is only defined for a length-3 vector."))
    end
    𝐒  = @SMatrix [   0  -v[3]    v[2] ;
                    v[3]    0    -v[1] ;
                   -v[2]  v[1]      0]

end

function smallest_eigenpair(A::AbstractArray)
    F = eigen(A)
    index = argmin(F.values)
    (F.values[index], F.vectors[:,index])
end

function smallest_eigenpair(A::AbstractArray,B::AbstractArray)
    F = eigfact(A,B)
    index = indmin(F[:values])
    (F[:values][index], F[:vectors][:,index])
end

function minor(A, i, j)
    m, n = size(A)
    B = similar(A, m-1, n-1)
    for j′=1:j-1, i′=1:i-1; B[i′,j′] = A[i′,j′]; end
    for j′=1:j-1, i′=i+1:m; B[i′-1,j′]= A[i′,j′]; end
    for j′=j+1:n, i′=1:i-1; B[i′,j′-1] = A[i′,j′]; end
    for j′=j+1:n, i′=i+1:m; B[i′-1,j′-1] = A[i′,j′]; end
    return B
end
