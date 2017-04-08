

# constrained maximization exercises

## portfolio choice problem

module HW_constrained
Pkg.add("NLopt")
Pkg.add("JuMP")
Pkg.add("DataFrames")
	using JuMP, NLopt, DataFrames

	export data, table_NLopt, table_JuMP

	function dt(a)
	n=3
	p = [1,1,1]
	e = [2,0,0]
	u(c) = -exp(-a*c)
	u_prime= a*exp(-a*c)
	z[1]=(1.0,1.0,1.0)
	z[2]=(0.72,0.92,1.12,1.32)
	z[3]=(0.86,0.96,1.06,1.16)
	S=([0.72,0.86],[0.72,0.96],[0.72,1.06],[0.72,1.16],[0.92,0.86],[0.92,0.96],[0.92,1.06],[0.92,1.16],[1.12,0.86],[1.12,0.96],[1.12,1.06],[1.12,1.16],[1.32,0.86],[1.32,0.96],[1.32,1.06],[1.32,1.16])
	dict=Dict("a"=>a,"utility"=>u,"uprime"=>u_prime,"nasset"=>n,"prices"=>p,"endwts"=>e,"states"=>S)
	return dict
	end


	function max_JuMP(a)
	m=Model(solver = IpoptSolver())
	n=3
	@variable(m,c>=0)
	@variable(m,omega[1:n])
	pi=1/16
	@NLobjective(m,Max,-exp(-a*c)+pi*sum(-exp(-a*(omega[1]+omega[2]*dt["states"][s][1]+omega[3]*dt["states"][s][2])) for s in 1:16))
	@NLconstraint(m, c+sum(omega[i]- dta["endwts"][i] for i in 1:n)==0)
	print(m)
	status=solve(m)
	v=[getobjectivevalue(m);getvalue(c);getvalue(omega)[1];getvalue(omega)[2];getvalue(omega)[3]]
  return v
	end

	function table_JuMP()
	dfMP=DataFrame()
  dfMP[:a]=[0.5;1;5]
  dfMP[:c]=[max_JuMP(0.5)[2];max_JuMP(1)[2];max_JuMP(5)[2]]
  dfMP[:omega1]=[max_JuMP(0.5)[3];max_JuMP(1)[3];max_JuMP(5)[3]]
  dfMP[:omega2]=[max_JuMP(0.5)[4];max_JuMP(1)[4];max_JuMP(5)[4]]
  dfMP[:omega3]=[max_JuMP(0.5)[5];max_JuMP(1)[5];max_JuMP(5)[5]]
  dfMP[:fvalue]=[max_JuMP(0.5)[1];max_JuMP(1)[1];max_JuMP(5)[1]]
 return dfMP
	end


	function obj(x::Vector,grad::Vector,dt::Dict)
	if lenght(grad)>0
		grad[1]=dt["a"]exp(-dt["a"]*x[1])
		grad[2]=1/16(sum(dt["a"]*exp(-dt["a"]*(x[2]+x[3]*dt["states"]s[1]+x[4]*dt["states"]s[2]))for S=1:16))
		grad[3]=1/16(sum(dt["states"]s[1](dt["a"]*exp(-dt["a"]*(x[2]+x[3]*dt["states"]s[1]+x[4]*dt["states"]s[2])))for S=1:16))
		grad[4]=1/16(sum(dt["states"]s[2](dt["a"]*exp(-dt["a"]*(x[2]+x[3]*dt["states"]s[1]+x[4]*dt["states"]s[2])))for S=1:16))
  end
		return  -exp(-dta["a"]*x[1])+(1/16)*sum((-exp(-dta["a"]*(x[2]+x[3]*dt["states"][s][1]+x[4]*dt["states"][s][2]))) for s=1:16)
	end

	function constr(x::Vector,grad::Vector,dt::Dict)
	if length(grad) > 0
    grad[1] = 1
    grad[2] = 1
    grad[3] = 1
    grad[4] = 1
	end
    return x[1] + sum(x[i]- dt["endwts"][i-1] for i in 2:4)
	end

	function max_NLopt(a)
 opt = Opt(:LD_MMA, 4)
 data = dt(a)
 xtol_rel!(opt,1e-4)
 max_objective!(opt, (x,grad) -> obj(x, grad, data))
 inequality_constraint!(opt, (x,grad) -> constr(x,grad,data))
	(optf,optx,ret) = optimize(opt, [1, -1.5, 1, 1.5])
	end

	function table_NLopt()
		dfNL=DataFrame()
    dfNL[:a]=[0.5;1;5]
	  dfNL[:c]=[max_NLopt(0.5)[2][1];max_NLopt(1)[2][1];max_NLopt(5)[2][1]]
	  dfNL[:omega1]=[max_NLopt(0.5)[2][2];max_NLopt(1)[2][2];max_NLopt(5)[2][2]]
	  dfNL[:omega2]=[max_NLopt(0.5)[2][3];max_NLopt(1)[2][3];max_NLopt(5)[2][3]]
	  dfNL[:omega3]=[max_NLopt(0.5)[2][4];max_NLopt(1)[2][4];max_NLopt(5)[2][4]]
	  dfNL[:fvalue]=[max_NLopt(0.5)[1];max_NLopt(1)[1];max_NLopt(5)[1]]
	end

	# function `f` is for the NLopt interface, i.e.
	# it has 2 arguments `x` and `grad`, where `grad` is
	# modified in place
	# if you want to call `f` with more than those 2 args, you need to
	# specify an anonymous function as in
	# other_arg = 3.3
	# test_finite_diff((x,g)->f(x,g,other_arg), x )
	# this function cycles through all dimensions of `f` and applies
	# the finite differencing to each. it prints some nice output.
	function test_finite_diff(f::Function,x::Vector{Float64},tol=1e-6)

	end

	# do this for each dimension of x
	# low-level function doing the actual finite difference
	function finite_diff(f::Function,x::Vector)

	end

	function runAll()
		println("running tests:")
		include("test/runtests.jl")
		println("")
		println("JumP:")
		table_JuMP()
		println("")
		println("NLopt:")
		table_NLopt()
		ok = input("enter y to close this session.")
		if ok == "y"
			quit()
		end
	end


end
