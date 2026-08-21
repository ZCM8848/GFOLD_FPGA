from fixed_bfp import solve_bfp
for nl, tag in [(0.0, "float64 baseline"), (1e-3, "wide-f32-32 (1e-3)"), (1e-4, "1e-4")]:
    r = solve_bfp(50000, 32, track=10000, kkt_noise=nl)
    print(f"noise={nl:g} ({tag}) @50k: rel_x={r['relx']:.4e} obj={r['obj']:.6f} "
          f"scale={r['scale']:.2e} upd={r['n_scale_updates']}")
    for it, rx, o in r['hist']:
        print(f"   {it}: rel_x={rx:.3e} obj={o:.6f}")
