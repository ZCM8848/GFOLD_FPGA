#!/usr/bin/env python3
"""
numpy re-assembler of the G-FOLD SOCP for ARBITRARY time-of-flight.

Ports gfold-core/src/assemble.rs (+ derive.rs, config.rs) so we can rebuild
A, b, q for any tof. Verified against software/problem.json at tof=44.63.

Row order (matches assemble()):
  [0:706] zero cone equalities
  [706:1007] nonneg = glide(100) + thrust_upper(100)+dry(1) + pointing(100)
  [1007:1407] velocity SOC (100 x dim4)
  [1407:1807] thrust_slack SOC (100 x dim4)
  [1807:2107] thrust_lower SOC (100 x dim3)
"""
import numpy as np
import scipy.sparse as sp

# ---- defaults from config.rs ----
WET=2000.0; FUEL=1700.0; REAL_MAX_T=24000.0; MIN_PCT=0.2; MAX_PCT=0.8
MAX_VEL=1000.0; INIT_POS=[450.0,-330.0,2400.0]; INIT_VEL=[-40.0,10.0,-10.0]
TGT_VEL=[0.0,0.0,0.0]; TGT_POS=[0.0,0.0,0.0]; FUEL_CONS=5e-4
GRAV=[0.0,0.0,-3.71]; GLIDE_ANGLE=0.0; MAX_ANGLE=90.0
N=100

def log_wet_mass(): return np.log(WET)
def log_dry_mass(): return np.log(WET-FUEL)
def min_thrust(): return REAL_MAX_T*MIN_PCT
def max_thrust(): return REAL_MAX_T*MAX_PCT
def dt(tof): return tof/N

def derive(tof):
    n=N; a=FUEL_CONS; max_t=max_thrust(); min_t=min_thrust(); wet=WET
    d=dt(tof)
    z0=[]; max_exp=[]; min_exp=[]
    for i in range(n):
        z=np.log(wet - a*d*max_t*i)
        e=np.exp(-z)
        z0.append(z); max_exp.append(1.0/(e*max_t))
        min_exp.append(np.inf if min_t==0 else 1.0/(e*min_t))
    return np.array(z0), np.array(max_exp), np.array(min_exp)

def x(i,c): return 6*i+c
def u(i,c): return 6*N+3*i+c
def s(i): return 9*N+i
def z(i): return 10*N+i

def assemble(tof):
    n=N
    nvars=11*n
    d=dt(tof); d2=d*d; a_dt=FUEL_CONS*d
    g=GRAV
    z0,max_exp,min_exp=derive(tof)
    # rows: list of (coeffs dict, b)
    rows=[]; b=[]
    def add(coeffs, bb):
        rows.append(coeffs); b.append(bb)
    # equality_rows
    for c in range(3): add({x(0,c):1.0}, INIT_POS[c])
    for c in range(3): add({x(0,c+3):1.0}, INIT_VEL[c])
    add({z(0):1.0}, log_wet_mass())
    for i in range(n-1):
        for c in range(3):
            add({x(i+1,c):1.0, x(i,c):-1.0, x(i,c+3):-d, u(i,c):-d2/3.0, u(i+1,c):-d2/6.0}, g[c]*d2/2.0)
        for c in range(3):
            add({x(i+1,c+3):1.0, x(i,c+3):-1.0, u(i,c):-d/2.0, u(i+1,c):-d/2.0}, g[c]*d)
        add({z(i+1):1.0, z(i):-1.0, s(i):a_dt/2.0, s(i+1):a_dt/2.0}, 0.0)
    for c in range(3): add({x(n-1,c):1.0}, TGT_POS[c])
    for c in range(3): add({x(n-1,c+3):1.0}, TGT_VEL[c])
    # nonneg: glide(n) + thrust_upper(n)+dry(1) + pointing(n)
    sin_g=np.sin(np.radians(GLIDE_ANGLE))
    for i in range(n): add({x(i,2):-1.0}, 0.0)          # glide (angle 0 -> x[i,2]>=0)
    for i in range(n): add({z(i):1.0, s(i):max_exp[i]}, 1.0+z0[i])  # thrust upper
    add({z(n-1):-1.0}, -log_dry_mass())                  # dry mass
    cos_a=np.cos(np.radians(MAX_ANGLE))
    for i in range(n): add({s(i):cos_a, u(i,2):-1.0}, 0.0)  # pointing
    # SOC blocks (t, v...) -- each returns rows with b
    # velocity (dim4)
    vel_rows=[]
    for i in range(n):
        blk=[({}, MAX_VEL)]
        for c in range(3): blk.append(({x(i,c+3):-1.0}, 0.0))
        vel_rows.append(blk)
    # thrust_slack (dim4)
    ts_rows=[]
    for i in range(n):
        blk=[({s(i):-1.0}, 0.0)]
        for c in range(3): blk.append(({u(i,c):-1.0}, 0.0))
        ts_rows.append(blk)
    # thrust_lower (dim3)
    tl_rows=[]
    for i in range(n):
        m=min_exp[i]; zz=z0[i]
        blk=[({s(i):-2*m, z(i):-2.0}, -2*zz-1.0),
             ({z(i):-2.0}, -2*zz),
             ({s(i):-2*m, z(i):-2.0}, -2*zz-3.0)]
        tl_rows.append(blk)
    soc_rows = vel_rows + ts_rows + tl_rows
    # build CSC A
    nrows=len(rows)+sum(1 for _ in b for _ in [0])  # equalities+nonneg = len(rows)
    # count: equalities 706 + nonneg 301 = 1007
    n_base=len(rows)   # = 1007
    n_soc=sum(len(blk) for blk in soc_rows)  # 1100
    total=n_base+n_soc
    # assemble
    colptr=[0]*(nvars+1); nzval=[]; rowval=[]
    def add_entry(cidx, r, val):
        nzval.append(val); rowval.append(r)
    # use CSR-like build: for each row, for each col
    Arows=[[] for _ in range(total)]
    Ab=[0.0]*total
    r=0
    for coeffs,bb in zip(rows,b):
        Ab[r]=bb
        for cc,vv in coeffs.items(): Arows[r].append((cc,vv))
        r+=1
    for blk in soc_rows:
        for coeffs,bb in blk:
            Ab[r]=bb
            for cc,vv in coeffs.items(): Arows[r].append((cc,vv))
            r+=1
    # to CSC
    data=[]; indices=[]; indptr=[0]
    for cc in range(nvars):
        for rr in range(total):
            pass
    # build via scipy COO
    from collections import defaultdict
    rr_list=[]; cc_list=[]; vv_list=[]
    for ri in range(total):
        for ci,val in Arows[ri]:
            rr_list.append(ri); cc_list.append(ci); vv_list.append(val)
    A=sp.coo_matrix((vv_list,(rr_list,cc_list)),shape=(total,nvars)).tocsc()
    b_arr=np.array(Ab)
    # q
    q=np.zeros(nvars); q[z(n-1)]=-1.0
    return A,b_arr,q,total,nvars

if __name__=="__main__":
    import json
    D=json.load(open("problem.json"))
    A,b,q,nrows,nvars=assemble(44.63)
    # compare to problem.json
    Aref=sp.csc_matrix((np.array(D["nzval"]),np.array(D["rowval"]),np.array(D["colptr"])),shape=(D["nrows"],D["nvars"])).tocsr()
    dA=(A-Aref).toarray()
    print("nvars",nvars,"vs",D["nvars"],"| nrows",nrows,"vs",D["nrows"])
    print("max|A diff|=",np.abs(dA).max())
    print("max|b diff|=",np.abs(b-np.array(D["b"])).max())
    print("max|q diff|=",np.abs(q-np.array(D["q"])).max())
    print("A equal:", np.allclose(A.toarray(),Aref.toarray()), " b equal:", np.allclose(b,D["b"]), " q equal:",np.allclose(q,D["q"]))
