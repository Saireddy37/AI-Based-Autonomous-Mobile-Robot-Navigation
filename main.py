"""
Main Entrypoint for Autonomous Mobile Robot Navigation System
Supports Web GUI Dashboard mode, Automated Benchmark mode, and Mathematical Verification mode.
"""

import sys
import argparse

if hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except Exception:
        pass

from robot_model.symbolic_verifier import verify_symbolic_dynamics
from simulation.benchmark import BenchmarkSuite

def run_verification():
    print("=" * 60)
    print("🤖 SYMBOLIC & NUMERICAL DYNAMICS VERIFICATION")
    print("=" * 60)
    res = verify_symbolic_dynamics()
    print(f"Lagrangian == Newton-Euler Symbolic Proof: {'SUCCESS ✅' if res['overall_symbolic_proof'] else 'FAILED ❌'}")
    print(f"Kinetic Energy T: {res['Kinetic_Energy']}")
    print(f"v-equation matching: {res['v_equation_match']}")
    print(f"omega-equation matching: {res['omega_equation_match']}")
    print("=" * 60)

def run_cli_benchmark():
    print("=" * 60)
    print("📊 RUNNING AUTOMATED BENCHMARK SUITE")
    print("=" * 60)
    suite = BenchmarkSuite(num_trials=20)
    results = suite.run_benchmarks()
    for name, metrics in results.items():
        print(f"\n▶ [{name}]")
        print(f"  • Success Rate:             {metrics['success_rate_percent']:.1f}%")
        print(f"  • Mean Path Length:        {metrics['mean_path_length_m']:.2f} m")
        print(f"  • Mean Smoothness:         {metrics['mean_smoothness']:.4f}")
        print(f"  • Mean Convergence Time:   {metrics['mean_convergence_time_s']:.2f} s")
        print(f"  • Mean Step Computation:   {metrics['mean_comp_time_per_step_ms']:.2f} ms")
        print(f"  • Mean Torque Energy:      {metrics['mean_torque_energy_J']:.2f} J")
    print("=" * 60)

def main():
    parser = argparse.ArgumentParser(description="AI-Based Autonomous Mobile Robot Navigation System")
    parser.add_argument("--benchmark", action="store_true", help="Run CLI benchmark suite")
    parser.add_argument("--verify", action="store_true", help="Run symbolic dynamics proof")
    parser.add_argument("--self-test", action="store_true", help="Run fast full-execution smoke tests")
    parser.add_argument("--port", type=int, default=5050, help="Web server port (default 5050)")
    args = parser.parse_args()

    if args.verify:
        run_verification()
    elif args.self_test:
        import unittest
        print("=" * 60)
        print("🧪 RUNNING FULL EXECUTION SELF-TEST")
        print("=" * 60)
        suite = unittest.defaultTestLoader.loadTestsFromName("tests.test_full_execution")
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        raise SystemExit(0 if result.wasSuccessful() else 1)
    elif args.benchmark:
        run_cli_benchmark()
    else:
        run_verification()
        print(f"\n🌐 Starting Web GUI Dashboard on http://localhost:{args.port} ...")
        from web.app import app
        app.run(host="0.0.0.0", port=args.port, debug=False)

if __name__ == "__main__":
    main()
