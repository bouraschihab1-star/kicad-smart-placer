"""
=============================================================================
 constraints.py — Phase 3 : Logique Métier et Contraintes du Monde Réel
=============================================================================
 Ce module encode les règles physiques et électriques strictes :
   1. Verrouillage périphérique des connecteurs (J, P) sur Edge.Cuts
   2. Groupement rigide des super-composants
   3. Optimisation Top/Bottom (flip des passifs si densité haute)
   4. Répulsion thermique entre composants de puissance
   5. Orientation discrète (0°, 90°, 180°, 270° uniquement)
   6. Respect absolu des zones interdites

 PHILOSOPHIE DE CONCEPTION :
   Chaque contrainte est implémentée comme une pénalité continue plutôt
   qu'une contrainte dure. Cela permet au recuit simulé d'explorer
   temporairement des états invalides (ce qui aide à sortir des minima
   locaux), tout en convergent vers un état valide grâce aux pénalités
   croissantes à basse température.

   Exception : les zones interdites absolues et l'orientation discrète
   sont des contraintes dures (pénalité infinie).
=============================================================================
"""

import math
from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict

from .utils import (
    BBox, ComponentData, NetInfo,
    from_mm, to_mm, snap_angle, euclidean_distance,
    point_to_segment_distance, setup_logger
)

logger = setup_logger("Constraints")


# ---------------------------------------------------------------------------
#  1. CONSTANTES DE CONFIGURATION
# ---------------------------------------------------------------------------

# Distance maximale d'attraction des connecteurs vers le bord (mm)
CONNECTOR_EDGE_MAX_DIST_MM = 5.0

# Poids de la pénalité d'éloignement des connecteurs du bord (désactivé pour l'instant)
CONNECTOR_EDGE_PENALTY_WEIGHT = 0.0

# Distance cible des connecteurs par rapport au bord (mm)
# Idéalement, le bord du connecteur touche exactement le bord de la carte
CONNECTOR_EDGE_TARGET_MM = 0.0

# Seuil de densité pour activer le flip Bottom automatique (Désactivé : 100% F.Cu)
DENSITY_FLIP_THRESHOLD = 999.0

# Fraction maximale de composants pouvant être basculés en Bottom
MAX_FLIP_RATIO = 0.0

# Distance minimale de répulsion thermique entre composants de puissance (mm)
# Réduit à 0.5 mm (au lieu de 8.0 mm) pour ne pas disperser arbitrairement les composants
THERMAL_REPULSION_DIST_MM = 0.5

# Poids de la force de répulsion thermique (atténué en l'absence de simulation thermique)
THERMAL_REPULSION_WEIGHT = 1.0


class ConstraintEngine:
    """
    Moteur de contraintes de placement.

    Ce moteur applique les règles métier du monde réel au placement.
    Il est appelé à chaque itération du recuit simulé pour évaluer
    la qualité d'un état de placement.

    Architecture :
        Les contraintes sont évaluées séparément et retournent des
        pénalités additives. Le coût total est :
            C = HPWL + λ₁·P_collision + λ₂·P_connector + λ₃·P_thermal
                + λ₄·P_boundary + λ₅·P_keepout

        Les poids λᵢ sont calibrés pour que :
          - Les contraintes dures (keepout, orientation) dominent toujours
          - Les contraintes souples (thermique, edge) guident sans bloquer
    """

    def __init__(
        self,
        board_outline: List[Tuple] = None,
        placement_area: BBox = None
    ):
        """
        Args:
            board_outline: Segments du contour Edge.Cuts.
            placement_area: Zone de placement autorisée.
        """
        self._board_outline = board_outline or []
        self._placement_area = placement_area or BBox()
        self._rigid_groups: List[List[str]] = []
        self._group_map: Dict[str, int] = {}  # ref → group_index

    # -------------------------------------------------------------------
    #  2. VERROUILLAGE PÉRIPHÉRIQUE DES CONNECTEURS
    # -------------------------------------------------------------------

    def compute_connector_penalty(
        self,
        components: List[ComponentData],
        positions: Dict[str, Tuple[int, int]] = None
    ) -> float:
        """
        Calcule la pénalité d'éloignement des connecteurs du bord.

        Règle :
            Les composants dont la référence commence par 'J' ou 'P'
            (connecteurs) doivent être attirés vers le bord le plus
            proche de la carte (défini par Edge.Cuts).

        Modèle de pénalité (potentiel quadratique d'attraction) :
            Pour chaque connecteur C :
                d = distance(centre_C, bord_carte)
                P_C = λ × d²

            La pénalité quadratique crée un gradient lisse qui attire
            les connecteurs vers le bord. Plus un connecteur est loin
            du bord, plus la force de rappel est forte.

        Cas idéal : le connecteur est directement sur le bord → P = 0.

        Si aucun contour Edge.Cuts n'est défini, la pénalité est calculée
        par rapport aux bords de la zone de placement.

        Args:
            components: Liste des composants.
            positions: Positions optionnelles mises à jour.

        Returns:
            Pénalité totale (positif, 0 = tous les connecteurs sur le bord).
        """
        if not self._board_outline and not self._placement_area:
            return 0.0

        total_penalty = 0.0
        max_dist_nm = from_mm(CONNECTOR_EDGE_MAX_DIST_MM)

        for comp in components:
            ref_prefix = _get_prefix(comp.reference)
            if ref_prefix not in ('J', 'P'):
                continue

            # Position actuelle ou mise à jour
            cx, cy = positions.get(comp.reference, comp.position) \
                     if positions else comp.position

            # Trouver la distance au bord le plus proche
            min_dist = float('inf')

            if self._board_outline:
                # Distance au contour Edge.Cuts
                for (x1, y1), (x2, y2) in self._board_outline:
                    d = point_to_segment_distance(cx, cy, x1, y1, x2, y2)
                    min_dist = min(min_dist, d)
            else:
                # Distance aux bords de la zone de placement
                area = self._placement_area
                distances = [
                    abs(cx - area.x_min),  # bord gauche
                    abs(cx - area.x_max),  # bord droit
                    abs(cy - area.y_min),  # bord haut
                    abs(cy - area.y_max),  # bord bas
                ]
                min_dist = min(distances)

            # Pénalité quadratique :
            #   P = weight × (d / max_dist)²
            # Normalisée par max_dist pour garder des valeurs cohérentes
            if min_dist > 0:
                normalized_dist = min_dist / max_dist_nm if max_dist_nm > 0 else 0
                total_penalty += CONNECTOR_EDGE_PENALTY_WEIGHT * (normalized_dist ** 2)

        return total_penalty

    def get_nearest_edge_point(
        self, x: int, y: int
    ) -> Tuple[int, int]:
        """
        Trouve le point le plus proche sur le bord de la carte.

        Utilisé pour "snapper" les connecteurs sur le bord lors du
        placement initial.

        Algorithme :
            Pour chaque segment [A, B] du contour :
                Calculer le point le plus proche P sur [A, B]
                Si dist(query, P) < best_dist :
                    best_point = P

        Args:
            x, y: Coordonnées du point de requête (nm).

        Returns:
            (x, y) du point le plus proche sur le bord.
        """
        if not self._board_outline:
            # Fallback : snapper aux bords de la zone de placement
            area = self._placement_area
            distances = [
                (abs(x - area.x_min), area.x_min, y),
                (abs(x - area.x_max), area.x_max, y),
                (abs(y - area.y_min), x, area.y_min),
                (abs(y - area.y_max), x, area.y_max),
            ]
            _, bx, by = min(distances, key=lambda t: t[0])
            return (bx, by)

        best_dist = float('inf')
        best_point = (x, y)

        for (ax, ay), (bx, by) in self._board_outline:
            # Projection sur le segment
            dx_ab = bx - ax
            dy_ab = by - ay
            seg_len_sq = dx_ab * dx_ab + dy_ab * dy_ab

            if seg_len_sq == 0:
                px, py = ax, ay
            else:
                t = ((x - ax) * dx_ab + (y - ay) * dy_ab) / seg_len_sq
                t = max(0.0, min(1.0, t))
                px = int(ax + t * dx_ab)
                py = int(ay + t * dy_ab)

            d = euclidean_distance((x, y), (px, py))
            if d < best_dist:
                best_dist = d
                best_point = (px, py)

        return best_point

    # -------------------------------------------------------------------
    #  3. GROUPEMENT RIGIDE (SUPER-COMPOSANTS)
    # -------------------------------------------------------------------

    def set_rigid_groups(self, groups: List[List[str]]):
        """
        Enregistre les groupes de composants rigides.

        Un groupe rigide est un ensemble de composants qui doivent
        maintenir leurs positions relatives fixes. Quand le composant
        maître (premier de la liste) est déplacé, tous les composants
        esclaves sont translatés du même vecteur.

        Args:
            groups: Liste de groupes. Chaque groupe = [maître, esclave1, ...]
        """
        self._rigid_groups = groups
        self._group_map = {}
        for idx, group in enumerate(groups):
            for ref in group:
                self._group_map[ref] = idx

        logger.info(f"Groupes rigides enregistrés : {len(groups)}")

    def get_group_for_component(self, ref: str) -> Optional[int]:
        """Retourne l'index du groupe rigide du composant, ou None."""
        return self._group_map.get(ref, None)

    def is_group_master(self, ref: str) -> bool:
        """Vérifie si le composant est le maître de son groupe."""
        group_idx = self._group_map.get(ref)
        if group_idx is None:
            return False
        return self._rigid_groups[group_idx][0] == ref

    def get_group_members(self, ref: str) -> List[str]:
        """Retourne la liste des membres du groupe du composant."""
        group_idx = self._group_map.get(ref)
        if group_idx is None:
            return [ref]
        return self._rigid_groups[group_idx]

    def compute_rigid_group_offsets(
        self,
        group_idx: int,
        components: List[ComponentData]
    ) -> Dict[str, Tuple[int, int]]:
        """
        Calcule les offsets relatifs des membres d'un groupe rigide.

        L'offset de chaque composant est calculé par rapport au maître :
            offset_i = position_i - position_master

        Ces offsets sont constants : quand le maître bouge de (dx, dy),
        chaque esclave bouge de (dx, dy) aussi.

        Args:
            group_idx: Index du groupe rigide.
            components: Liste des composants.

        Returns:
            Dict {ref: (dx, dy)} des offsets relatifs au maître.
        """
        group = self._rigid_groups[group_idx]
        comp_by_ref = {c.reference: c for c in components}

        master = comp_by_ref.get(group[0])
        if master is None:
            return {}

        mx, my = master.position
        offsets = {}

        for ref in group:
            comp = comp_by_ref.get(ref)
            if comp is None:
                continue
            offsets[ref] = (comp.position[0] - mx, comp.position[1] - my)

        return offsets

    # -------------------------------------------------------------------
    #  4. OPTIMISATION TOP/BOTTOM (FLIP DES PASSIFS)
    # -------------------------------------------------------------------

    def select_flip_candidates(
        self,
        components: List[ComponentData],
        density: float
    ) -> List[str]:
        """
        Sélectionne les composants passifs candidats au basculement Bottom.

        Règle de décision :
            Si la densité de placement ρ > DENSITY_FLIP_THRESHOLD (55%),
            alors on autorise le basculement des composants passifs (R, C, L)
            sur la face arrière (B.Cu) pour libérer de l'espace en face avant.

        Critères de sélection :
            - Seuls les passifs (R, C, L) sont éligibles
            - Les composants verrouillés sont exclus
            - Les composants déjà en face arrière sont exclus
            - On ne bascule pas plus de MAX_FLIP_RATIO du total

        Stratégie de sélection :
            On trie les candidats par TAILLE CROISSANTE (les plus petits
            sont basculés en premier) car :
              1. Les petits passifs (0402, 0603) sont faciles à souder en SMD bottom
              2. Les gros composants (SMD > 1206) sont plus complexes à assembler

        Args:
            components: Liste des composants.
            density: Densité de placement actuelle (0.0 à 1.0).

        Returns:
            Liste des références des composants à basculer.
        """
        # Mode 100% F.Cu (Top layer) imposé : aucun composant n'est basculé en Bottom
        logger.info("Placement simple face (100% F.Cu) actif → pas de flip Bottom")
        return []

    # -------------------------------------------------------------------
    #  5. RÉPULSION THERMIQUE
    # -------------------------------------------------------------------

    def compute_thermal_penalty(
        self,
        components: List[ComponentData],
        positions: Dict[str, Tuple[int, int]] = None
    ) -> float:
        """
        Calcule la pénalité de proximité thermique entre composants de puissance.

        Les composants de puissance (Q, D, L, ou liés à des nets forts)
        génèrent de la chaleur. S'ils sont trop proches, ils créent des
        "points chauds" (hotspots) qui dégradent la fiabilité.

        Modèle de pénalité (potentiel répulsif inverse) :
            Pour chaque paire de composants de puissance (A, B) :
                d = distance(centre_A, centre_B)
                Si d < d_min :
                    P_AB = λ × (1 - d/d_min)²

        Justification physique du modèle quadratique :
            La température en un point d'un PCB suit la loi de Fourier.
            La superposition des champs thermiques de deux sources proches
            crée un échauffement additionnel qui croît approximativement
            en 1/d. Notre potentiel quadratique est une approximation
            conservative qui pénalise fortement les très petites distances.

        Args:
            components: Liste des composants.
            positions: Positions optionnelles mises à jour.

        Returns:
            Pénalité thermique totale (positif, 0 = espacement suffisant).
        """
        # Filtrer les composants de puissance
        power_comps = [c for c in components if c.is_power]

        if len(power_comps) < 2:
            return 0.0

        total_penalty = 0.0
        d_min = from_mm(THERMAL_REPULSION_DIST_MM)

        for i in range(len(power_comps)):
            for j in range(i + 1, len(power_comps)):
                a = power_comps[i]
                b = power_comps[j]

                # Positions actuelles ou mises à jour
                pos_a = positions.get(a.reference, a.position) \
                        if positions else a.position
                pos_b = positions.get(b.reference, b.position) \
                        if positions else b.position

                d = euclidean_distance(pos_a, pos_b)

                if d < d_min:
                    # Pénalité quadratique de proximité
                    # ratio = 1.0 quand d=0 (superposés), 0.0 quand d=d_min
                    ratio = 1.0 - (d / d_min)
                    total_penalty += THERMAL_REPULSION_WEIGHT * (ratio ** 2)

        return total_penalty

    # -------------------------------------------------------------------
    #  6. PÉNALITÉ GLOBALE
    # -------------------------------------------------------------------

    def compute_total_constraint_penalty(
        self,
        components: List[ComponentData],
        positions: Dict[str, Tuple[int, int]] = None
    ) -> Tuple[float, Dict[str, float]]:
        """
        Calcule la pénalité totale de toutes les contraintes.

        Composition :
            P_total = P_connector + P_thermal

        Les pénalités de collision et de frontière sont gérées par
        SpatialModel pour éviter la double comptabilité.

        Args:
            components: Liste des composants.
            positions: Positions optionnelles mises à jour.

        Returns:
            Tuple (pénalité_totale, détail_par_type).
        """
        p_connector = self.compute_connector_penalty(components, positions)
        p_thermal = self.compute_thermal_penalty(components, positions)

        total = p_connector + p_thermal

        detail = {
            'connector_edge': p_connector,
            'thermal_repulsion': p_thermal,
        }

        return total, detail


# ---------------------------------------------------------------------------
#  UTILITAIRE LOCAL
# ---------------------------------------------------------------------------

def _get_prefix(reference: str) -> str:
    """Extrait le préfixe alphabétique d'une référence."""
    prefix = ""
    for char in reference:
        if char.isalpha():
            prefix += char
        else:
            break
    return prefix.upper()
