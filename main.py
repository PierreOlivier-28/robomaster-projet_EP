"""Point d'entree : python main.py s1|s2|s3|mission|sim [--real]

  s1 / s2 / s3 / mission : scenarios (faux robot en console, ou vrai robot avec --real)
  sim                    : simulateur 2D interactif (fenetre), pour voir ce que fait le code
"""
import argparse
import os


def main(argv=None):
    parser = argparse.ArgumentParser(description="Projet RoboMaster")
    parser.add_argument("scenario", choices=["s1", "s2", "s3", "mission", "sim"])
    parser.add_argument("--real", action="store_true",
                        help="utiliser le vrai robot (sinon : simulation)")
    args = parser.parse_args(argv)

    # le mode doit etre fixe AVANT d'importer config / les scenarios
    os.environ["ROBOT_MODE"] = "real" if args.real else "sim"

    if args.scenario == "sim":
        from sim import demo
        demo.main()
        return
    if args.scenario == "s1":
        from scenarios import s1_navigation as sc
    elif args.scenario == "s2":
        from scenarios import s2_tracking as sc
    elif args.scenario == "s3":
        from scenarios import s3_manipulation as sc
    else:
        from scenarios import s4_mission_complete as sc
    sc.main()


if __name__ == "__main__":
    main()
