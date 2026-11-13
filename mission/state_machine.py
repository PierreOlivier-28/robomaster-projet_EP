"""Machine a etats : INIT -> CHERCHER -> APPROCHER -> SAISIR -> NAVIGUER -> DEPOSER -> FIN (+ ERREUR)."""


class Mission:
    def run(self):
        raise NotImplementedError
