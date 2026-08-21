"""Generate test vectors for root_plus RTL: r,g,p,mu (len L), eta -> expected tau."""
import sys, os, json, struct, random
import numpy as np

def root_plus_py(r, g, p, mu, eta, TAU=10.0):
    gg = float(np.sum(g*g*r)); mug = float(np.sum(mu*g*r))
    pg = float(np.sum(p*g*r)); pp = float(np.sum(p*p*r)); pmu = float(np.sum(p*mu*r))
    a = TAU + gg; b = mug - 2*pg - eta*TAU; cc = pp - pmu
    rad = b*b - 4*a*cc
    if rad < 0: return -b/(2*a)
    sq = float(np.sqrt(rad))
    if b <= 0: return (-b+sq)/(2*a)
    q = -0.5*(b+sq)
    return cc/q if q != 0 else 0.0

def f2h(x):
    return format(struct.unpack('>Q', struct.pack('>d', float(x)))[0], '016x')

def main():
    ncases = int(sys.argv[1]) if len(sys.argv)>1 else 24
    L = int(sys.argv[2]) if len(sys.argv)>2 else 8
    rng = np.random.default_rng(1234)
    cases = []
    for c in range(ncases):
        r  = rng.uniform(0.5, 2.0, L)
        g  = rng.normal(0, 1, L)
        p  = rng.normal(0, 1, L)
        mu = rng.normal(0, 1, L)
        eta = rng.uniform(-2, 2)
        tau = root_plus_py(r, g, p, mu, eta)
        cases.append(dict(r=r, g=g, p=p, mu=mu, eta=eta, tau=tau))

    # branch distribution check
    br = []
    for c in cases:
        gg=float(np.sum(c['g']*c['g']*c['r'])); mug=float(np.sum(c['mu']*c['g']*c['r']))
        pg=float(np.sum(c['p']*c['g']*c['r'])); pp=float(np.sum(c['p']*c['p']*c['r']))
        pmu=float(np.sum(c['p']*c['mu']*c['r']))
        a=10+gg; b=mug-2*pg-c['eta']*10; cc=pp-pmu; rad=b*b-4*a*cc
        if rad<0: br.append('rad<0')
        elif b<=0: br.append('b<=0')
        else: br.append('b>0')
    print("branch counts:", {k:br.count(k) for k in set(br)})

    os.makedirs('rtl/data', exist_ok=True)
    # write per-case files: hex rows of r,g,p,mu (L each), then eta line, then expected tau
    for ci, c in enumerate(cases):
        with open(f'rtl/data/rp_{ci}.hex','w') as f:
            for i in range(L):
                f.write(f2h(c['r'][i])+' '+f2h(c['g'][i])+' '+f2h(c['p'][i])+' '+f2h(c['mu'][i])+'\n')
            f.write(f2h(c['eta'])+' '+f2h(c['tau'])+'\n')   # last line: eta, expected
    with open('rtl/data/rp_manifest.json','w') as f:
        json.dump(dict(ncases=ncases, L=L, fname_prefix='rp_'), f)
    print(f"wrote {ncases} cases (L={L}) to rtl/data/rp_*.hex + manifest")

if __name__ == '__main__':
    main()
