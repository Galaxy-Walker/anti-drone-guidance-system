import sys
from pathlib import Path

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pn_nmpc_guidance import DEFAULT_40CM_CONFIG, ProportionalNavigation3D_VariableSpeed


def main():
    print("=" * 60)
    print("3D比例导引 - PN/NMPC 可变速度控制仿真")
    print("=" * 60)

    sim = ProportionalNavigation3D_VariableSpeed.from_config(DEFAULT_40CM_CONFIG)

    sim.run_simulation()
    sim.plot_results()


if __name__ == "__main__":
    main()
