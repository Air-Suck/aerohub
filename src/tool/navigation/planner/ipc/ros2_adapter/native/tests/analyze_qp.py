"""Inspect a dumped QP independently; this program never supplies flight commands."""
import argparse,json,re
from pathlib import Path
import numpy as np
from scipy.optimize import linprog,minimize,LinearConstraint
p=argparse.ArgumentParser();p.add_argument('problem',type=Path);p.add_argument('--conflicts',action='store_true',help='Find an irreducible infeasible row set by independent LP checks');a=p.parse_args()
d=json.loads(re.sub(r'-?\bnan\b', 'NaN', a.problem.read_text()));H=np.asarray(d['H']);P=np.triu(H)+np.triu(H,1).T;q=np.asarray(d['q']).ravel();A=np.asarray(d['A']);lower=np.asarray(d['lower']).ravel();upper=np.asarray(d['upper']).ravel()
nonfinite={k:int((~np.isfinite(np.asarray(d[k],dtype=float))).sum()) for k in ('H','q','A','lower','upper','state')}
if any(nonfinite.values()):
 print(json.dumps({'scope':'offline QP diagnostic; no optimization or commands','nonfinite_entries':nonfinite,'valid_problem':False},indent=2));raise SystemExit(0)
lower[lower<=-1e20]=-np.inf;upper[upper>=1e20]=np.inf
finite_u=np.isfinite(upper);finite_l=np.isfinite(lower)
feas=linprog(np.zeros(len(q)),A_ub=np.vstack([A[finite_u],-A[finite_l]]),b_ub=np.r_[upper[finite_u],-lower[finite_l]],bounds=[(None,None)]*len(q),method='highs')
def violation(x):return float(max(0,np.max(lower-A@x),np.max(A@x-upper)))
eigen=np.linalg.eigvalsh(P);norms=np.linalg.norm(A,axis=1)
result={'scope':'independent offline QP diagnostics','shape':list(A.shape),'hessian_eigen_min':float(eigen.min()),'hessian_eigen_max':float(eigen.max()),'hessian_asymmetry_norm':float(np.linalg.norm(H-H.T)),'A_row_norm_min':float(norms.min()),'A_row_norm_max':float(norms.max()),'max_abs_q':float(abs(q).max()),'zero_violation':violation(np.zeros(len(q))),'linear_feasible':bool(feas.success),'linear_message':feas.message}
if feas.success:
 r=minimize(lambda x:.5*x@P@x+q@x,feas.x,jac=lambda x:P@x+q,constraints=LinearConstraint(A,lower,upper),method='SLSQP',options={'maxiter':500,'ftol':1e-9})
 result.update(slsqp_success=bool(r.success),slsqp_message=r.message,slsqp_iterations=r.nit,slsqp_objective=float(r.fun),slsqp_max_violation=violation(r.x))
print(json.dumps(result,indent=2))

n=len(q)
if a.conflicts and not feas.success:
 def feasible_rows(rows):
  matrix=A[rows];lo=lower[rows];hi=upper[rows]
  fu=np.isfinite(hi);fl=np.isfinite(lo)
  check=linprog(np.zeros(n),A_ub=np.vstack([matrix[fu],-matrix[fl]]),b_ub=np.r_[hi[fu],-lo[fl]],bounds=[(None,None)]*n,method='highs')
  if check.status not in (0,2):raise RuntimeError('LP did not establish feasibility/infeasibility: '+check.message)
  return check.success
 rows=list(range(len(lower)))
 for row in list(rows):
  trial=[r for r in rows if r!=row]
  if not feasible_rows(trial):rows=trial
 # Group names apply only to this native MPC layout: [jerk, acceleration,
 # velocity, pruned corridors]. Refuse to label an unrelated dump this way.
 native_layout=len(lower)>=3*n and n%3==0 and np.allclose(A[:n],np.eye(n))
 def describe(row):
  group=('jerk' if row<n else 'acceleration' if row<2*n else 'velocity' if row<3*n else 'corridor') if native_layout else 'unknown'
  return {'row':row,'group':group,'step_zero_based':row%n//3 if native_layout and row<3*n else None,
          'axis':('xyz'[row%3] if native_layout and row<3*n else None),
          'lower':float(lower[row]) if np.isfinite(lower[row]) else None,'upper':float(upper[row]) if np.isfinite(upper[row]) else None,
          'coefficients':[[int(j),float(A[row,j])] for j in np.flatnonzero(A[row])]}
 print(json.dumps({'scope':'offline conflict diagnosis; no flight parameter recommendation',
                   'system_constraints_feasible':bool(feasible_rows(list(range(3*n)))) if native_layout else None,
                   'irreducible_infeasible_rows':[describe(row) for row in rows],
                   'limitations':'Irreducible under the independent LP tolerance, not necessarily a minimum-cardinality or unique conflict. Corridor row numbers refer to the pruned dump, not original planes.'},indent=2))
if np.allclose(A[:n],np.eye(n)):
 lo=lower[:n];hi=upper[:n]
 maximum=np.where(A>=0,A*hi,A*lo).sum(axis=1)
 minimum=np.where(A>=0,A*lo,A*hi).sum(axis=1)
 dominated=(maximum<=upper-1e-9)&(minimum>=lower+1e-9)
 dominated[:3*n]=False
 keep=~dominated
 if feas.success:
  reduced=minimize(lambda x:.5*x@P@x+q@x,feas.x,jac=lambda x:P@x+q,constraints=LinearConstraint(A[keep],lower[keep],upper[keep]),method='SLSQP',options={'maxiter':500,'ftol':1e-9})
  print(json.dumps({'input_box_implied_corridor_rows_removed':int(dominated.sum()),'remaining_rows':int(keep.sum()),'reduced_success':bool(reduced.success),'objective_difference':float(reduced.fun-r.fun),'reduced_solution_original_violation':violation(reduced.x)},indent=2))
