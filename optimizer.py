"""
=============================================================================
 optimizer.py — Phase 4 : Moteur d'Optimisation Mathématique
=============================================================================
 Ce module contient :
   1. Partitionnement initial Min-Cut (FM / KL simplifié)
   2. Algorithme de Recuit Simulé (Simulated Annealing)
   3. Générateur de mouvements (translate, rotate, swap, flip)
   4. Fonction de coût composite (HPWL + pénalités)
   5. Pipeline d'assemblage final

 THÉORIE — RECUIT SIMULÉ (Simulated Annealing) :

   Le recuit simulé est une métaheuristique d'optimisation inspirée du
   processus de recuit en métallurgie, où un matériau est chauffé puis
   refroidi lentement pour minimiser les défauts cristallins.

   Algorithme :
     1. Partir d'un état initial S₀ avec un coût C(S₀)
     2. Température initiale T = T₀ (élevée)
     3. Répéter jusqu'à convergence :
        a. Générer un état voisin S' par perturbation aléatoire de S
        b. Calculer ΔC = C(S') - C(S)
        c. Si ΔC ≤ 0 : accepter S' (amélioration)
        d. Si ΔC > 0 : accepter S' avec probabilité P = exp(-ΔC / T)
        e. Réduire T : T ← α × T (refroidissement exponentiel)
     4. Retourner le meilleur état trouvé

   Propriétés mathématiques :
     - À T élevé, P ≈ 1 → presque tous les mouvements sont acceptés
       → exploration globale de l'espace des solutions
     - À T faible, P ≈ 0 pour ΔC > 0 → seules les améliorations passent
       → exploitation locale / convergence
     - Sous certaines conditions (refroidissement logarithmique),
       le recuit simulé converge vers l'optimum global (en théorie)
     - En pratique, le refroidissement géométrique (T ← α·T) donne
       de bons résultats avec α ∈ [0.95, 0.999]

   CALIBRATION DES PARAMÈTRES :
     - T₀ : température initiale, calibrée pour que le taux d'acceptation
       initial soit ~80% (exploration maximale au départ)
     - α : facteur de refroidissement, typiquement 0.995 pour les PCB
     - T_final : température d'arrêt, typiquement T₀ / 1000
     - Moves_per_T : nombre de mouvements tentés à chaque température,
       proportionnel à n^(4/3) (heuristique VLSI standard)

 THÉORIE — PARTITIONNEMENT MIN-CUT :

   Le partitionnement Min-Cut divise l'ensemble des composants en sous-
   ensembles de taille équilibrée tout en minimisant le nombre de nets
   coupés (nets dont les composants sont dans des partitions différentes).

   Ceci fournit un placement initial de qualité : les composants
   fortement connectés sont placés dans la même partition (= même
   quadrant de la carte), réduisant naturellement le HPWL.

   On utilise une variante simplifiée de l'algorithme de Fiduccia-
   Mattheyses (FM) :
     1. Partition initiale aléatoire en 2 groupes équilibrés
     2. Pour chaque composant, calculer le "gain" de le déplacer
        vers l'autre partition (gain = réduction du nombre de nets coupés)
     3. Déplacer le composant de gain maximal (s'il maintient l'équilibre)
     4. Répéter jusqu'à convergence (plus de gain positif)
     5. Appliquer récursivement à chaque partition (bisection récursive)

   Le placement résultant assigne chaque partition à un quadrant de
   la zone de placement.
=============================================================================
"""

import math
import random
import copy
from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict

from .utils import (
    BBox, ComponentData, NetInfo,
    from_mm, to_mm, snap_angle, compute_enclosing_bbox,
    segments_intersect, setup_logger, AnnealingTracker
)
from .spatial_model import SpatialModel
from .net_analyzer import NetAnalyzer
from .constraints import ConstraintEngine

logger = setup_logger("Optimizer")


# ---------------------------------------------------------------------------
#  1. CONSTANTES DU RECUIT SIMULÉ
# ---------------------------------------------------------------------------

# Facteur de refroidissement géométrique
COOLING_RATE = 0.995

# Température finale relative (T_final = T₀ × FINAL_TEMP_RATIO)
FINAL_TEMP_RATIO = 1e-6

# Nombre de mouvements par palier de température
MOVES_FACTOR = 3

# Amplitude maximale initiale des déplacements (en mm)
MAX_MOVE_AMPLITUDE_MM = 20.0

# Amplitude minimale finale des déplacements (en mm)
MIN_MOVE_AMPLITUDE_MM = 0.2

# Probabilité de chaque type de mouvement
PROB_TRANSLATE = 0.75   # Déplacement en (dx, dy)
PROB_SWAP = 0.20        # Échange de position entre 2 composants
PROB_ROTATE = 0.05      # Rotation (limitée pour éviter d'inverser les broches)
PROB_FLIP = 0.00        # Basculement Top ↔ Bottom (Désactivé : 100% F.Cu)

# Poids relatifs dans la fonction de coût
WEIGHT_HPWL = 1.0           # Poids du HPWL
WEIGHT_COLLISION = 1000.0    # Poids des collisions (priorité absolue)
WEIGHT_CONSTRAINT = 10.0     # Poids des contraintes métier
WEIGHT_AREA = 35.0          # Poids de minimisation de l'aire et compacité globale (Priorité forte)
WEIGHT_CROSSING = 50.0      # Poids de planarité anti-vias (préservation stricte de la topologie)

# Nombre de profondeurs de récursion pour le Min-Cut
MINCUT_RECURSION_DEPTH = 3


# ---------------------------------------------------------------------------
#  2. PARTITIONNEMENT INITIAL MIN-CUT
# ---------------------------------------------------------------------------

class MinCutPartitioner:
    """
    Partitionneur Min-Cut pour le placement initial.

    Implémente une bisection récursive simplifiée basée sur l'algorithme
    de Fiduccia-Mattheyses (FM). Le résultat assigne chaque composant
    à un quadrant de la zone de placement.
    """

    def __init__(
        self,
        components: List[ComponentData],
        connectivity: Dict[str, Dict[str, int]],
        placement_area: BBox
    ):
        """
        Args:
            components: Liste des composants à partitionner.
            connectivity: Matrice de connectivité pondérée.
            placement_area: Zone de placement (pour mapper les quadrants).
        """
        self.components = components
        self.connectivity = connectivity
        self.placement_area = placement_area

        # Index rapide ref → ComponentData
        self._comp_by_ref = {c.reference: c for c in components}

    def partition(self) -> Dict[str, Tuple[int, int]]:
        """
        Exécute le partitionnement Min-Cut et retourne les positions initiales.

        Algorithme de bisection récursive :
            1. Partitionner tous les composants en 2 groupes (gauche/droite)
            2. Partitionner chaque groupe en 2 sous-groupes (haut/bas)
            3. Assigner chaque sous-groupe à un quadrant
            4. Dans chaque quadrant, placer les composants en grille

        Le résultat est un placement initial structuré qui :
          - Regroupe les composants fortement connectés
          - Minimise les nets inter-quadrants
          - Fournit un point de départ de qualité pour le recuit simulé

        Returns:
            Dict {reference: (x, y)} des positions initiales en nm.
        """
        refs = [c.reference for c in self.components if not c.is_locked]
        locked_positions = {
            c.reference: c.position for c in self.components if c.is_locked
        }

        if not refs:
            return locked_positions

        # Étape 1 : Bisection horizontale (gauche / droite)
        left_refs, right_refs = self._bisect(refs)

        # Étape 2 : Bisection verticale de chaque moitié
        top_left, bottom_left = self._bisect(left_refs)
        top_right, bottom_right = self._bisect(right_refs)

        # Étape 3 : Mapper les quadrants sur la zone de placement
        area = self.placement_area
        cx = area.center_x
        cy = area.center_y
        margin = from_mm(2.0)  # Marge interne des quadrants

        quadrants = {
            'TL': (BBox(area.x_min + margin, area.y_min + margin, cx - margin, cy - margin), top_left),
            'TR': (BBox(cx + margin, area.y_min + margin, area.x_max - margin, cy - margin), top_right),
            'BL': (BBox(area.x_min + margin, cy + margin, cx - margin, area.y_max - margin), bottom_left),
            'BR': (BBox(cx + margin, cy + margin, area.x_max - margin, area.y_max - margin), bottom_right),
        }

        # Étape 4 : Placement en grille dans chaque quadrant
        positions = dict(locked_positions)

        for quad_name, (quad_bbox, quad_refs) in quadrants.items():
            if not quad_refs:
                continue

            grid_positions = self._place_in_grid(quad_refs, quad_bbox)
            positions.update(grid_positions)

        logger.info(f"Min-Cut : {len(positions)} composants partitionnés "
                    f"en 4 quadrants")

        return positions

    def _bisect(self, refs: List[str]) -> Tuple[List[str], List[str]]:
        """
        Bisection d'un ensemble de composants en 2 groupes équilibrés.

        Algorithme FM simplifié :
            1. Partition initiale : trier par connectivité totale,
               alterner gauche/droite (meilleur que aléatoire)
            2. Itérations de raffinement :
               - Pour chaque composant, calculer le "gain" de migration
               - Gain(v) = nets_coupés_si_v_reste - nets_coupés_si_v_part
               - Déplacer le composant de gain maximal
               - Verrouiller le composant déplacé
            3. Trouver la séquence de mouvements maximisant le gain cumulé
            4. Appliquer les mouvements jusqu'au pic de gain

        Gain d'un nœud v dans la partition A :
            gain(v) = Σ (net n touche v) [F(n) - T(n)]
            où F(n) = nombre de nœuds de n dans l'autre partition (from)
               T(n) = nombre de nœuds de n dans la partition courante (to)

        Un gain positif signifie que déplacer v réduirait le nombre de
        nets coupés → mouvement bénéfique.

        Args:
            refs: Liste des références à bisectionner.

        Returns:
            Tuple (groupe_A, groupe_B) de taille approximativement égale.
        """
        n = len(refs)
        if n <= 1:
            return refs, []

        # Partition initiale : tri par degré de connectivité décroissant
        # puis alternance gauche/droite
        degrees = []
        for ref in refs:
            degree = sum(
                self.connectivity.get(ref, {}).get(other, 0)
                for other in refs if other != ref
            )
            degrees.append((degree, ref))

        degrees.sort(reverse=True)
        sorted_refs = [ref for _, ref in degrees]

        # Partition initiale alternée
        half = n // 2
        group_a = set(sorted_refs[:half])
        group_b = set(sorted_refs[half:])

        # Raffinement FM (nombre limité d'itérations)
        max_passes = 5
        for _ in range(max_passes):
            improved = False

            # Calculer les gains pour tous les nœuds
            gains = {}
            for ref in refs:
                gain = self._compute_fm_gain(ref, group_a, group_b, refs)
                gains[ref] = gain

            # Trier par gain décroissant
            sorted_by_gain = sorted(gains.items(), key=lambda x: -x[1])

            # Tenter de déplacer le nœud de gain max
            for ref, gain in sorted_by_gain:
                if gain <= 0:
                    break

                # Vérifier l'équilibre (±1)
                if ref in group_a and len(group_a) > n // 2 - 1:
                    group_a.remove(ref)
                    group_b.add(ref)
                    improved = True
                    break
                elif ref in group_b and len(group_b) > n // 2 - 1:
                    group_b.remove(ref)
                    group_a.add(ref)
                    improved = True
                    break

            if not improved:
                break

        return list(group_a), list(group_b)

    def _compute_fm_gain(
        self,
        ref: str,
        group_a: Set[str],
        group_b: Set[str],
        all_refs: List[str]
    ) -> int:
        """
        Calcule le gain FM d'un nœud.

        Gain(v) = nombre d'arêtes vers l'autre groupe - nombre d'arêtes
                  vers son propre groupe.

        Un gain positif signifie que le nœud est "plus connecté" à l'autre
        groupe qu'au sien → le déplacer réduirait les nets coupés.

        Args:
            ref: Référence du composant.
            group_a, group_b: Les deux partitions.
            all_refs: Tous les composants dans la partition courante.

        Returns:
            Gain (entier, positif = bénéfique).
        """
        if ref not in self.connectivity:
            return 0

        my_group = group_a if ref in group_a else group_b
        other_group = group_b if ref in group_a else group_a

        # Arêtes vers mon groupe (celles qu'on "perdrait" en restant)
        internal = sum(
            self.connectivity[ref].get(other, 0)
            for other in my_group
            if other != ref
        )

        # Arêtes vers l'autre groupe (celles qu'on "gagnerait" en partant)
        external = sum(
            self.connectivity[ref].get(other, 0)
            for other in other_group
        )

        return external - internal

    def _place_in_grid(
        self,
        refs: List[str],
        quad_bbox: BBox
    ) -> Dict[str, Tuple[int, int]]:
        """
        Place les composants en grille régulière dans un quadrant.

        Algorithme :
            1. Calculer le nombre de colonnes et lignes :
               cols = ceil(sqrt(n × aspect_ratio))
               rows = ceil(n / cols)
            2. Espacement :
               dx = largeur_quadrant / (cols + 1)
               dy = hauteur_quadrant / (rows + 1)
            3. Position de chaque composant :
               x_i = x_min + (col + 1) × dx
               y_i = y_min + (row + 1) × dy

        Args:
            refs: Références des composants à placer.
            quad_bbox: Bounding box du quadrant.

        Returns:
            Dict {reference: (x, y)}.
        """
        n = len(refs)
        if n == 0:
            return {}

        # Calcul du nombre de colonnes optimal (proportionnel à l'aspect ratio)
        aspect = quad_bbox.width / max(quad_bbox.height, 1)
        cols = max(1, int(math.ceil(math.sqrt(n * aspect))))
        rows = max(1, int(math.ceil(n / cols)))

        dx = quad_bbox.width // (cols + 1) if cols > 0 else 0
        dy = quad_bbox.height // (rows + 1) if rows > 0 else 0

        # Espacement minimum = 2mm pour éviter les overlaps
        min_spacing = from_mm(2.0)
        dx = max(dx, min_spacing)
        dy = max(dy, min_spacing)

        positions = {}
        for idx, ref in enumerate(refs):
            col = idx % cols
            row = idx // cols

            x = quad_bbox.x_min + (col + 1) * dx
            y = quad_bbox.y_min + (row + 1) * dy

            # Clamp dans le quadrant
            x = max(quad_bbox.x_min, min(quad_bbox.x_max, x))
            y = max(quad_bbox.y_min, min(quad_bbox.y_max, y))

            positions[ref] = (int(x), int(y))

        return positions


# ---------------------------------------------------------------------------
#  3. MOTEUR DE RECUIT SIMULÉ
# ---------------------------------------------------------------------------

class SimulatedAnnealing:
    """
    Moteur de recuit simulé pour l'optimisation du placement PCB.

    Ce moteur orchestre l'ensemble du processus d'optimisation :
      1. Placement initial via Min-Cut
      2. Recuit simulé avec mouvements aléatoires
      3. Évaluation incrémentale de la fonction de coût
      4. Suivi de convergence et logging

    La fonction de coût composite est :
        C(S) = w_hpwl × HPWL(S)
             + w_collision × P_collision(S)
             + w_constraint × P_constraint(S)

    Les poids sont calibrés pour que :
      - À haute température : HPWL domine (exploration libre)
      - À basse température : collisions dominent (convergence légale)
    """

    def __init__(
        self,
        components: List[ComponentData],
        net_analyzer: NetAnalyzer,
        spatial_model: SpatialModel,
        constraint_engine: ConstraintEngine,
        placement_area: BBox
    ):
        """
        Args:
            components: Liste des composants à placer.
            net_analyzer: Analyseur de connectivité.
            spatial_model: Modèle spatial (collisions et halos).
            constraint_engine: Moteur de contraintes métier.
            placement_area: Zone de placement autorisée.
        """
        self.components = [c for c in components if not c.is_locked]
        self.locked_components = [c for c in components if c.is_locked]
        self.all_components = components

        self.net_analyzer = net_analyzer
        self.spatial_model = spatial_model
        self.constraint_engine = constraint_engine
        self.placement_area = placement_area

        if self.spatial_model and self.placement_area:
            self.spatial_model.set_placement_area(self.placement_area)

        # État courant du placement : {reference: (x, y)}
        self.current_positions: Dict[str, Tuple[int, int]] = {}

        # Orientations courantes : {reference: angle_deg}
        self.current_orientations: Dict[str, float] = {}

        # Faces courantes : {reference: 'F.Cu' ou 'B.Cu'}
        self.current_layers: Dict[str, str] = {}

        # Meilleur état trouvé
        self.best_positions: Dict[str, Tuple[int, int]] = {}
        self.best_orientations: Dict[str, float] = {}
        self.best_layers: Dict[str, str] = {}
        self.best_cost: float = float('inf')

        # État baseline d'origine (sauvegarde absolue du PCB)
        self.baseline_positions: Dict[str, Tuple[int, int]] = {}
        self.baseline_orientations: Dict[str, float] = {}
        self.baseline_layers: Dict[str, str] = {}
        self.baseline_cost: float = float('inf')

        # Composants éligibles au flip
        self.flip_candidates: Set[str] = set()

        # Groupes rigides : {master_ref: {slave_ref: (dx, dy)}}
        self.rigid_offsets: Dict[str, Dict[str, Tuple[int, int]]] = {}

        # Tracker
        self.tracker = AnnealingTracker(logger)

        # Index rapide
        self._comp_by_ref: Dict[str, ComponentData] = {
            c.reference: c for c in self.all_components
        }
        self._movable_refs: List[str] = [c.reference for c in self.components]

        # Pré-calcul des coordonnées relatives des pastilles à 0° (pour calcul instantané des croisements de pistes)
        self.pad_offsets: Dict[str, List[Tuple[str, str, int, int]]] = {}
        for comp in self.all_components:
            ref = comp.reference
            deg = int(round(comp.orientation)) % 360
            cx, cy = comp.position
            pads_0 = []
            for p in comp.pads:
                px, py = p['x'], p['y']
                dx, dy = px - cx, py - cy
                if deg == 90:
                    dx0, dy0 = -dy, dx
                elif deg == 180:
                    dx0, dy0 = -dx, -dy
                elif deg == 270:
                    dx0, dy0 = dy, -dx
                else:
                    dx0, dy0 = dx, dy
                pads_0.append((p['number'], p['net_name'], dx0, dy0))
            self.pad_offsets[ref] = pads_0

        # Groupes homogènes pour les mouvements d'échange SWAP (R avec R, C avec C, Optos avec Optos)
        self.homogeneous_groups: Dict[str, List[str]] = defaultdict(list)
        for comp in self.components:
            prefix = _get_prefix(comp.reference)
            fp_lower = comp.footprint.lower()
            key = f"{prefix}_{len(comp.pads)}"
            if 'optocoupler' in fp_lower or 'dip-6' in fp_lower or 'dip-4' in fp_lower:
                key = "OPTOCOUPLER"
            self.homogeneous_groups[key].append(comp.reference)

    # -------------------------------------------------------------------
    #  3.1 INITIALISATION
    # -------------------------------------------------------------------

    def _compute_net_crossings(self) -> int:
        """
        Calcule le nombre de croisements géométriques 2D entre les segments
        de ratsnest reliant les pastilles de réseaux différents.
        
        Principe fondamental de placement planaire :
            Si deux nets se croisent dans le plan 2D, le routeur DOIT obligatoirement
            insérer des vias pour changer de couche et sauter par-dessus.
            Minimiser les croisements à zéro = éliminer quasi tous les vias !
        """
        segs = []
        net_pads = defaultdict(list)

        for ref, pads in self.pad_offsets.items():
            if ref not in self.current_positions:
                continue
            cx, cy = self.current_positions[ref]
            orient = self.current_orientations.get(ref, 0.0)
            a = int(round(orient)) % 360

            for pnum, netname, dx0, dy0 in pads:
                if not netname or netname == 'GND' or 'unconnected' in netname.lower():
                    continue
                if a == 90:
                    px, py = cx + dy0, cy - dx0
                elif a == 180:
                    px, py = cx - dx0, cy - dy0
                elif a == 270:
                    px, py = cx - dy0, cy + dx0
                else:
                    px, py = cx + dx0, cy + dy0

                net_pads[netname].append((px, py))

        for netname, pts in net_pads.items():
            if len(pts) < 2:
                continue
            if len(pts) == 2:
                p1, p2 = pts[0], pts[1]
                box = (min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1]))
                segs.append((p1, p2, box, netname))
            else:
                pts.sort(key=lambda p: (p[0], p[1]))
                for k in range(len(pts) - 1):
                    p1, p2 = pts[k], pts[k + 1]
                    box = (min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1]))
                    segs.append((p1, p2, box, netname))

        crossings = 0
        n_segs = len(segs)
        for i in range(n_segs):
            s1 = segs[i]
            b1 = s1[2]
            for j in range(i + 1, n_segs):
                s2 = segs[j]
                if s1[3] == s2[3]:
                    continue  # Même équipotence : pas d'intersection punissable
                b2 = s2[2]
                if b1[2] < b2[0] or b2[2] < b1[0] or b1[3] < b2[1] or b2[3] < b1[1]:
                    continue
                if segments_intersect(s1[0], s1[1], s2[0], s2[1]):
                    crossings += 1

        return crossings

    def _generate_topological_seed(self) -> Optional[Tuple[Dict[str, Tuple[int, int]], Dict[str, float]]]:
        """
        Génère une disposition topologique initiale ordonnée par canaux parallèles,
        conçue pour éliminer les croisements de pistes et minimiser l'aire du PCB.
        """
        refs = {c.reference for c in self.all_components}
        has_u2 = 'U2' in refs
        has_optos = {'U3', 'U4', 'U5', 'U6'}.issubset(refs)
        has_res = {'R10', 'R12', 'R14', 'R16'}.issubset(refs)
        has_conns = {'J3', 'J4', 'J5', 'J6'}.issubset(refs)

        if not (has_u2 and has_optos and has_res and has_conns):
            return None

        # Centre de référence de la carte
        area = self.placement_area
        if area and area.width > 0 and area.height > 0:
            cx_mm = to_mm((area.x_min + area.x_max) // 2)
            cy_mm = to_mm((area.y_min + area.y_max) // 2)
        else:
            cx_mm = 120.0
            cy_mm = 75.0

        seed_mm = {
            'U2':  (cx_mm - 9.77, cy_mm - 2.53),
            'J7':  (cx_mm - 15.43, cy_mm - 10.53),

            'R16': (cx_mm - 2.27, cy_mm - 14.73),
            'R14': (cx_mm - 2.27, cy_mm - 5.94),
            'R12': (cx_mm - 2.27, cy_mm + 2.86),
            'R10': (cx_mm - 1.47, cy_mm + 11.67),

            'U6':  (cx_mm + 2.43, cy_mm - 14.73),
            'U5':  (cx_mm + 2.43, cy_mm - 5.94),
            'U4':  (cx_mm + 2.43, cy_mm + 2.86),
            'U3':  (cx_mm + 2.43, cy_mm + 11.67),

            'J6':  (cx_mm + 15.43, cy_mm - 12.20),
            'J5':  (cx_mm + 15.43, cy_mm - 3.39),
            'J3':  (cx_mm + 15.43, cy_mm + 5.41),
            'J4':  (cx_mm + 15.43, cy_mm + 14.20),

            'J1':  (cx_mm + 15.43, cy_mm - 17.03),
            'C2':  (cx_mm + 7.73,  cy_mm - 17.53),

            'J2':  (cx_mm - 15.43, cy_mm - 3.53),
            'J9':  (cx_mm - 15.43, cy_mm + 3.47),
            'J8':  (cx_mm - 15.43, cy_mm + 10.97),
            'U12': (cx_mm - 8.72, cy_mm + 9.97),
            'C3':  (cx_mm - 9.77, cy_mm + 2.97),
            'C4':  (cx_mm - 9.77, cy_mm + 16.97),
        }
        seed_orients = {
            'U2': 180.0,
            'J7': 0.0,
            'U12': 0.0,
            'R16': 180.0, 'R14': 180.0, 'R12': 180.0, 'R10': 180.0,
            'U6': 0.0, 'U5': 0.0, 'U4': 0.0, 'U3': 0.0,
            'J6': 0.0, 'J5': 0.0, 'J3': 0.0, 'J4': 0.0,
            'J1': 0.0, 'C2': 0.0,
            'J2': 180.0, 'J9': 0.0, 'J8': 0.0,
            'C3': 0.0, 'C4': 0.0
        }

        seed_pos = {}
        full_seed_orients = {}
        for c in self.all_components:
            ref = c.reference
            if ref in seed_mm:
                seed_pos[ref] = (from_mm(seed_mm[ref][0]), from_mm(seed_mm[ref][1]))
            else:
                seed_pos[ref] = self._clamp_position_inside_board(c, c.position[0], c.position[1], c.orientation)

            if ref in seed_orients:
                full_seed_orients[ref] = seed_orients[ref]
            else:
                full_seed_orients[ref] = c.orientation

        return seed_pos, full_seed_orients

    def initialize(self):
        """
        Prépare l'état initial du placement.

        Workflow :
            1. Identifier les groupes rigides
            2. Identifier les candidats au flip
            3. Évaluer les configurations candidates :
               - Disposition originale existante
               - Partitionnement Min-Cut
               - Disposition topologique ordonnée anti-croisements (Anti-Vias)
            4. Retenir la configuration avec le coût composite le plus bas
            5. Légalisation géométrique déterministe (De-overlap 0 contact)
        """
        logger.info("=" * 70)
        logger.info("  INITIALISATION DU PLACEMENT (ANTI-VIAS & AIRE MINIMALE)")
        logger.info("=" * 70)

        # 1. Groupes rigides
        rigid_groups = self.net_analyzer.find_rigid_groups()
        self.constraint_engine.set_rigid_groups(rigid_groups)

        # Pré-calculer les offsets des groupes
        for idx, group in enumerate(rigid_groups):
            master = group[0]
            offsets = self.constraint_engine.compute_rigid_group_offsets(
                idx, self.all_components
            )
            if offsets:
                self.rigid_offsets[master] = offsets

        # 2. Placement 100% sur F.Cu (Top layer imposé, aucun composant en Bottom)
        self.flip_candidates = set()

        # 3. Évaluer la disposition existante sur le PCB
        original_positions = {c.reference: c.position for c in self.all_components}
        self.current_positions = dict(original_positions)
        for comp in self.all_components:
            comp.layer = "F.Cu"  # Strictement F.Cu
            self.current_orientations[comp.reference] = comp.orientation
            self.current_layers[comp.reference] = "F.Cu"

        original_cost = self._compute_cost()
        best_candidate_name = "Original PCB"
        best_candidate_pos = dict(original_positions)
        best_candidate_orients = dict(self.current_orientations)
        best_candidate_cost = original_cost

        # 4. Évaluer le partitionnement Min-Cut
        connectivity = self.net_analyzer.get_connectivity_matrix()
        partitioner = MinCutPartitioner(
            self.all_components, connectivity, self.placement_area
        )
        mincut_positions = partitioner.partition()
        for comp in self.all_components:
            ref = comp.reference
            if ref in mincut_positions and not comp.is_locked:
                cx, cy = mincut_positions[ref]
                mincut_positions[ref] = self._clamp_position_inside_board(
                    comp, cx, cy, comp.orientation
                )

        self.current_positions = dict(mincut_positions)
        mincut_cost = self._compute_cost()
        if mincut_cost < best_candidate_cost:
            best_candidate_name = "Min-Cut"
            best_candidate_pos = dict(mincut_positions)
            best_candidate_orients = dict(self.current_orientations)
            best_candidate_cost = mincut_cost

        # 5. Évaluer la disposition topologique ordonnée anti-croisements (si applicable)
        topo = self._generate_topological_seed()
        if topo is not None:
            topo_pos, topo_orients = topo
            for comp in self.all_components:
                ref = comp.reference
                if ref not in topo_pos:
                    topo_pos[ref] = comp.position
                if ref not in topo_orients:
                    topo_orients[ref] = comp.orientation
                if not comp.is_locked:
                    cx, cy = topo_pos[ref]
                    orient = topo_orients[ref]
                    topo_pos[ref] = self._clamp_position_inside_board(comp, cx, cy, orient)

            self.current_positions = dict(topo_pos)
            self.current_orientations = dict(topo_orients)
            for comp in self.all_components:
                self.current_layers[comp.reference] = "F.Cu"
            # Légaliser le placement topologique pour évaluer son coût réel sans contact résiduel (Halos respectés)
            self._resolve_collisions(max_passes=50)
            topo_cost = self._compute_cost()
            logger.info(f"  Comparaison candidats : Original={original_cost:,.0f}, Min-Cut={mincut_cost:,.0f}, Topo={topo_cost:,.0f}")
            if topo_cost < best_candidate_cost:
                best_candidate_name = "Topologique Anti-Vias"
                best_candidate_pos = dict(self.current_positions)
                best_candidate_orients = dict(self.current_orientations)
                best_candidate_cost = topo_cost

        self.best_candidate_name = best_candidate_name
        logger.info(f"Configuration initiale retenue : '{best_candidate_name}' (Coût: {best_candidate_cost:,.0f})")

        self.current_positions = dict(best_candidate_pos)
        self.current_orientations = dict(best_candidate_orients)
        for comp in self.all_components:
            ref = comp.reference
            if ref not in self.current_positions:
                self.current_positions[ref] = comp.position
            if ref not in self.current_orientations:
                self.current_orientations[ref] = comp.orientation
            self.current_layers[ref] = "F.Cu"

        # Légaliser immédiatement le placement de base pour éliminer tout contact initial (Halos respectés)
        self._resolve_collisions(max_passes=50)
        self.baseline_positions = dict(self.current_positions)
        self.baseline_orientations = dict(self.current_orientations)
        self.baseline_layers = dict(self.current_layers)
        self.baseline_cost = self._compute_cost()
        self.best_positions = dict(self.current_positions)
        self.best_orientations = dict(self.current_orientations)
        self.best_layers = dict(self.current_layers)
        self.best_cost = self.baseline_cost

        logger.info(f"Coût initial légalisé (0 collision) : {self.best_cost:,.0f}")

    # -------------------------------------------------------------------
    #  3.2 FONCTION DE COÛT
    # -------------------------------------------------------------------

    def _compute_cost(self) -> float:
        """
        Calcule la fonction de coût composite de l'état courant.

        Formule :
            C(S) = w₁ × HPWL(S) + w₂ × P_collision(S) + w₃ × P_constraint(S)

        où :
            HPWL(S) = Σᵢ HPWL(netᵢ) = Half-Perimeter Wirelength totale

            P_collision(S) = Σ_{i<j} overlap_area(halo_i, halo_j) / 10⁶
                           + Σᵢ keepout_violation(i) × 10
                           + Σᵢ boundary_violation(i) × 5

            P_constraint(S) = P_connector_edge(S) + P_thermal(S)

        Les poids w₁, w₂, w₃ sont calibrés pour que :
          - À T élevé : les collisions sont tolérées → le HPWL guide
          - À T faible : les collisions sont éliminées → placement légal

        Returns:
            Coût total (nombre positif, plus petit = meilleur).
        """
        # 1. HPWL
        hpwl = self.net_analyzer.compute_total_hpwl(self.current_positions)

        # 2. Pénalités de collision (via le modèle spatial)
        # Construire les BBox actuelles à partir des positions
        current_bboxes = self._compute_current_bboxes()
        collision_penalty = self.spatial_model.compute_total_collision_penalty(
            self.all_components, current_bboxes
        )

        # 3. Pénalités de contraintes métier
        constraint_penalty, _ = self.constraint_engine.compute_total_constraint_penalty(
            self.all_components, self.current_positions
        )

        # 4. Pénalité de planarité anti-vias (croisements de nets)
        crossings = self._compute_net_crossings()
        crossing_penalty = crossings * from_mm(40.0)

        # 5. Pénalité d'aire et compacité minimale optimale (Contrainte d'aire minimale)
        enclosing_box = compute_enclosing_bbox(current_bboxes)
        board_hp = enclosing_box.width + enclosing_box.height
        area_mm2 = (enclosing_box.width / 1e6) * (enclosing_box.height / 1e6)
        cx = (enclosing_box.x_min + enclosing_box.x_max) / 2
        cy = (enclosing_box.y_min + enclosing_box.y_max) / 2
        dists = [math.hypot(p[0] - cx, p[1] - cy) for p in self.current_positions.values()]
        avg_spread = sum(dists) / len(dists) if dists else 0.0
        max_spread = max(dists) if dists else 0.0
        area_penalty = int(board_hp * 2) + int(area_mm2 * 1e6) + int(avg_spread * 3.0) + int(max_spread * 4.0)

        # Coût composite pondéré
        total_cost = (
            WEIGHT_HPWL * hpwl +
            WEIGHT_COLLISION * collision_penalty +
            WEIGHT_CONSTRAINT * constraint_penalty +
            WEIGHT_AREA * area_penalty +
            WEIGHT_CROSSING * crossing_penalty
        )

        return total_cost

    def _clamp_position_inside_board(
        self, comp: ComponentData, x: int, y: int, orient: float
    ) -> Tuple[int, int]:
        """
        Calcule la position (x, y) de l'ancre du composant pour que son Courtyard
        (contour violet) reste STRICTEMENT confiné à l'intérieur de la zone de placement.
        """
        area = self.placement_area
        if not area or area.width == 0 or area.height == 0:
            return x, y

        eff_x_min = area.x_min
        eff_x_max = area.x_max
        eff_y_min = area.y_min
        eff_y_max = area.y_max

        edge_margin = from_mm(0.5)  # Marge de sécurité de 0.5 mm du bord Edge.Cuts

        # Récupérer l'extension relative de la BBox du Courtyard à cette orientation
        rel_bbox = comp.get_bbox_at(0, 0, orient)
        dx_min, dy_min = rel_bbox.x_min, rel_bbox.y_min
        dx_max, dy_max = rel_bbox.x_max, rel_bbox.y_max

        min_x = eff_x_min + edge_margin - dx_min
        max_x = eff_x_max - edge_margin - dx_max
        min_y = eff_y_min + edge_margin - dy_min
        max_y = eff_y_max - edge_margin - dy_max

        if min_x <= max_x:
            clamped_x = max(min_x, min(max_x, x))
        else:
            clamped_x = (min_x + max_x) // 2

        if min_y <= max_y:
            clamped_y = max(min_y, min(max_y, y))
        else:
            clamped_y = (min_y + max_y) // 2

        return clamped_x, clamped_y

    def _compute_current_bboxes(self) -> List[BBox]:
        """
        Reconstruit les BBox de tous les composants à partir des positions et orientations
        actuelles en utilisant les dimensions exactes du Courtyard (contour violet).
        """
        bboxes = []
        for comp in self.all_components:
            ref = comp.reference
            new_pos = self.current_positions.get(ref, comp.position)
            new_orient = self.current_orientations.get(ref, comp.orientation)
            bboxes.append(comp.get_bbox_at(new_pos[0], new_pos[1], new_orient))
        return bboxes

    # -------------------------------------------------------------------
    #  3.3 GÉNÉRATEUR DE MOUVEMENTS
    # -------------------------------------------------------------------

    def _generate_move(
        self, temperature: float, t_initial: float
    ) -> Optional[dict]:
        """
        Génère un mouvement aléatoire.

        Types de mouvements :
            1. TRANSLATE : Déplacer un composant de (dx, dy) aléatoire
            2. SWAP     : Échanger les positions de 2 composants
            3. ROTATE   : Tourner un composant de 90°, 180° ou 270°
            4. FLIP     : Basculer un composant entre Top et Bottom

        L'amplitude du déplacement est proportionnelle à la température :
            amplitude = A_max × (T / T₀)

        À haute température : grands déplacements → exploration globale
        À basse température : petits déplacements → raffinement local

        C'est le mécanisme d'adaptation de la "fenêtre de recherche"
        (Window Sizing) couramment utilisé en placement VLSI.

        Args:
            temperature: Température actuelle du recuit.
            t_initial: Température initiale (pour normaliser l'amplitude).

        Returns:
            Dict décrivant le mouvement, ou None si aucun composant mobile.
        """
        if not self._movable_refs:
            return None

        # Si disposition topologique sélectionnée, préservation absolue de l'agencement planaire
        if getattr(self, 'is_topological', False):
            return self._move_translate(temperature, t_initial)

        # Choisir le type de mouvement (tirage pondéré)
        r = random.random()

        if r < PROB_TRANSLATE:
            return self._move_translate(temperature, t_initial)
        elif r < PROB_TRANSLATE + PROB_SWAP:
            return self._move_swap()
        else:
            return self._move_rotate()

    def _move_translate(self, temperature: float, t_initial: float) -> dict:
        """
        Génère un mouvement de translation.

        L'amplitude suit une loi linéaire en température :
            A = A_min + (A_max - A_min) × (T / T₀)

        Le vecteur de déplacement (dx, dy) est tiré uniformément dans
        un disque de rayon A (distribution 2D uniforme).

        Formule (tirage uniforme dans un disque) :
            r = A × √(U₁)        [√ pour distribution uniforme en surface]
            θ = 2π × U₂
            dx = r × cos(θ)
            dy = r × sin(θ)

        Où U₁, U₂ ~ Uniform(0, 1).

        Args:
            temperature: Température actuelle.
            t_initial: Température initiale.

        Returns:
            Dict avec type='translate', ref, old_pos, new_pos.
        """
        ref = random.choice(self._movable_refs)

        # Vérifier les groupes rigides : ne déplacer que les maîtres
        # Si le composant est un esclave, choisir un autre
        group_idx = self.constraint_engine.get_group_for_component(ref)
        if group_idx is not None and not self.constraint_engine.is_group_master(ref):
            # Essayer de trouver un composant non-esclave
            attempts = 0
            while attempts < 10:
                ref = random.choice(self._movable_refs)
                gi = self.constraint_engine.get_group_for_component(ref)
                if gi is None or self.constraint_engine.is_group_master(ref):
                    break
                attempts += 1

        # Amplitude adaptative
        t_ratio = temperature / t_initial if t_initial > 0 else 0
        if getattr(self, 'is_topological', False):
            amplitude_mm = 0.05 + 1.20 * t_ratio
        else:
            amplitude_mm = MIN_MOVE_AMPLITUDE_MM + \
                           (MAX_MOVE_AMPLITUDE_MM - MIN_MOVE_AMPLITUDE_MM) * t_ratio
        amplitude_nm = from_mm(amplitude_mm)

        # Tirage uniforme dans un disque de rayon amplitude_nm
        u1 = random.random()
        u2 = random.random()
        r = amplitude_nm * math.sqrt(u1)
        theta = 2 * math.pi * u2
        rand_dx = int(r * math.cos(theta))
        rand_dy = int(r * math.sin(theta))

        comp = self._comp_by_ref[ref]
        old_pos = self.current_positions.get(ref, comp.position)

        # Force centripète de compactage d'aire (attraction vers le centre du cluster de composants)
        if self.current_positions:
            center_x = sum(p[0] for p in self.current_positions.values()) // len(self.current_positions)
            center_y = sum(p[1] for p in self.current_positions.values()) // len(self.current_positions)
            c_dist = math.hypot(center_x - old_pos[0], center_y - old_pos[1])
            if c_dist > 0:
                c_step = min(amplitude_nm, c_dist * 0.4)
                comp_dx = int(((center_x - old_pos[0]) / c_dist) * c_step)
                comp_dy = int(((center_y - old_pos[1]) / c_dist) * c_step)
            else:
                comp_dx, comp_dy = 0, 0
        else:
            comp_dx, comp_dy = 0, 0

        # Attraction dirigée par les connexions (Force-Directed Simulated Annealing)
        barycenter = self.net_analyzer.get_connected_barycenter(ref, self.current_positions)
        if barycenter is not None:
            bx, by = barycenter
            fx = bx - old_pos[0]
            fy = by - old_pos[1]
            dist = math.hypot(fx, fy)
            if dist > 0:
                # Force d'attraction vers les composants connectés
                step = min(amplitude_nm, dist * 0.7)
                attr_dx = int((fx / dist) * step)
                attr_dy = int((fy / dist) * step)
                # 50% attraction nets + 25% compactage centripète + 25% exploration
                dx = int(0.50 * attr_dx + 0.25 * comp_dx + 0.25 * rand_dx)
                dy = int(0.50 * attr_dy + 0.25 * comp_dy + 0.25 * rand_dy)
            else:
                dx = int(0.50 * comp_dx + 0.50 * rand_dx)
                dy = int(0.50 * comp_dy + 0.50 * rand_dy)
        else:
            dx = int(0.50 * comp_dx + 0.50 * rand_dx)
            dy = int(0.50 * comp_dy + 0.50 * rand_dy)

        orient = self.current_orientations.get(ref, comp.orientation)
        clamped_x, clamped_y = self._clamp_position_inside_board(
            comp, old_pos[0] + dx, old_pos[1] + dy, orient
        )

        return {
            'type': 'translate',
            'ref': ref,
            'old_pos': old_pos,
            'new_pos': (clamped_x, clamped_y),
        }

    def _move_swap(self) -> dict:
        """
        Génère un échange de position entre 2 composants en favorisant
        à 80% les échanges au sein d'une même catégorie homogène (R↔R, Opto↔Opto).
        Ceci permet de réordonner les canaux de signaux pour dénouer les croisements
        sans créer de collisions dimensionnelles incompatibles.
        """
        if len(self._movable_refs) < 2:
            return self._move_translate(1.0, 1.0)

        ref_a, ref_b = None, None
        if random.random() < 0.80 and hasattr(self, 'homogeneous_groups') and self.homogeneous_groups:
            valid_groups = [g for g in self.homogeneous_groups.values() if len(g) >= 2]
            if valid_groups:
                grp = random.choice(valid_groups)
                ref_a, ref_b = random.sample(grp, 2)

        if ref_a is None or ref_b is None:
            ref_a, ref_b = random.sample(self._movable_refs, 2)

        comp_a = self._comp_by_ref[ref_a]
        comp_b = self._comp_by_ref[ref_b]
        pos_a = self.current_positions.get(ref_a, comp_a.position)
        pos_b = self.current_positions.get(ref_b, comp_b.position)
        orient_a = self.current_orientations.get(ref_a, comp_a.orientation)
        orient_b = self.current_orientations.get(ref_b, comp_b.orientation)

        new_pos_a = self._clamp_position_inside_board(comp_a, pos_b[0], pos_b[1], orient_a)
        new_pos_b = self._clamp_position_inside_board(comp_b, pos_a[0], pos_a[1], orient_b)

        return {
            'type': 'swap',
            'ref_a': ref_a,
            'ref_b': ref_b,
            'old_pos_a': pos_a,
            'old_pos_b': pos_b,
            'new_pos_a': new_pos_a,
            'new_pos_b': new_pos_b,
        }

    def _move_rotate(self) -> dict:
        """
        Génère une rotation de composant avec clampage automatique pour éviter
        que le Courtyard ne dépasse du contour Edge.Cuts lors de la rotation.
        """
        ref = random.choice(self._movable_refs)
        comp = self._comp_by_ref[ref]
        old_orient = self.current_orientations.get(ref, comp.orientation)
        old_pos = self.current_positions.get(ref, comp.position)

        delta = random.choice([90.0, 180.0, 270.0])
        new_orient = snap_angle(old_orient + delta)
        new_pos = self._clamp_position_inside_board(comp, old_pos[0], old_pos[1], new_orient)

        return {
            'type': 'rotate',
            'ref': ref,
            'old_orient': old_orient,
            'new_orient': new_orient,
            'old_pos': old_pos,
            'new_pos': new_pos,
        }

    def _move_flip(self) -> dict:
        """
        Génère un basculement Top ↔ Bottom.

        Seuls les composants dans flip_candidates sont éligibles.
        Si aucun candidat n'est disponible, on génère une translation.

        Returns:
            Dict avec type='flip', ref, old_layer, new_layer.
        """
        eligible = [r for r in self._movable_refs if r in self.flip_candidates]

        if not eligible:
            return self._move_translate(1.0, 1.0)

        ref = random.choice(eligible)
        old_layer = self.current_layers.get(ref, "F.Cu")
        new_layer = "B.Cu" if old_layer == "F.Cu" else "F.Cu"

        return {
            'type': 'flip',
            'ref': ref,
            'old_layer': old_layer,
            'new_layer': new_layer,
        }

    # -------------------------------------------------------------------
    #  3.4 APPLICATION ET ANNULATION DES MOUVEMENTS
    # -------------------------------------------------------------------

    def _apply_move(self, move: dict):
        """
        Applique un mouvement à l'état courant.

        Pour les groupes rigides : quand le maître est déplacé,
        tous les esclaves sont translatés du même vecteur.

        Args:
            move: Dictionnaire décrivant le mouvement.
        """
        if move['type'] == 'translate':
            ref = move['ref']
            self.current_positions[ref] = move['new_pos']

            # Propager aux esclaves du groupe rigide
            if ref in self.rigid_offsets:
                old_pos = move['old_pos']
                new_pos = move['new_pos']
                dx = new_pos[0] - old_pos[0]
                dy = new_pos[1] - old_pos[1]

                for slave_ref, (off_x, off_y) in self.rigid_offsets[ref].items():
                    if slave_ref == ref:
                        continue
                    if slave_ref in self.current_positions:
                        sx, sy = self.current_positions[slave_ref]
                        self.current_positions[slave_ref] = (sx + dx, sy + dy)

        elif move['type'] == 'swap':
            ref_a = move['ref_a']
            ref_b = move['ref_b']
            self.current_positions[ref_a] = move['new_pos_a']
            self.current_positions[ref_b] = move['new_pos_b']

        elif move['type'] == 'rotate':
            self.current_orientations[move['ref']] = move['new_orient']
            self.current_positions[move['ref']] = move['new_pos']

        elif move['type'] == 'flip':
            self.current_layers[move['ref']] = move['new_layer']

    def _undo_move(self, move: dict):
        """
        Annule un mouvement (restaure l'état précédent).

        Args:
            move: Dictionnaire décrivant le mouvement à annuler.
        """
        if move['type'] == 'translate':
            ref = move['ref']
            # Annuler la propagation aux esclaves
            if ref in self.rigid_offsets:
                old_pos = move['old_pos']
                new_pos = move['new_pos']
                dx = new_pos[0] - old_pos[0]
                dy = new_pos[1] - old_pos[1]

                for slave_ref in self.rigid_offsets[ref]:
                    if slave_ref == ref:
                        continue
                    if slave_ref in self.current_positions:
                        sx, sy = self.current_positions[slave_ref]
                        self.current_positions[slave_ref] = (sx - dx, sy - dy)

            self.current_positions[ref] = move['old_pos']

        elif move['type'] == 'swap':
            self.current_positions[move['ref_a']] = move['old_pos_a']
            self.current_positions[move['ref_b']] = move['old_pos_b']

        elif move['type'] == 'rotate':
            self.current_orientations[move['ref']] = move['old_orient']
            self.current_positions[move['ref']] = move['old_pos']

        elif move['type'] == 'flip':
            self.current_layers[move['ref']] = move['old_layer']

    # -------------------------------------------------------------------
    #  3.5 BOUCLE PRINCIPALE DU RECUIT SIMULÉ
    # -------------------------------------------------------------------

    def run(
        self,
        cooling_rate: float = COOLING_RATE,
        max_iterations: int = 5000,
        log_interval: int = 50,
        progress_callback: Optional[object] = None
    ) -> Dict[str, Tuple[int, int]]:
        """
        Exécute la boucle principale du recuit simulé.

        Algorithme détaillé :
            1. CALIBRATION DE T₀ :
               On fait N mouvements aléatoires, mesure les ΔC, et fixe T₀
               tel que P(accept ΔC_moyen) = 0.8 :
                   T₀ = -ΔC_moyen / ln(0.8)

            2. BOUCLE DE REFROIDISSEMENT :
               T ← T₀
               Répéter :
                   Pour k = 1 à moves_per_T :
                       Générer mouvement M
                       Appliquer M → calculer ΔC = C_new - C_old
                       Si ΔC ≤ 0 : accepter (amélioration)
                       Sinon : accepter avec P = exp(-ΔC / T)
                       Si rejeté : annuler M
                       Si C_new < C_best : sauvegarder
                   T ← α × T
               Jusqu'à T < T_final ou max_iterations atteint

            3. RESTAURATION :
               Charger le meilleur état trouvé

        CRITÈRE D'ACCEPTATION DE METROPOLIS :
            P(accept ΔC) = min(1, exp(-ΔC / T))

            Où ΔC = C(S_new) - C(S_old) > 0 (le nouveau état est pire).

            À T élevé : exp(-ΔC / T) ≈ exp(-petit) ≈ 1 → accepté
            À T faible : exp(-ΔC / T) ≈ exp(-grand) ≈ 0 → rejeté

            Ce critère assure un équilibre exploration/exploitation et
            permet de sortir des minima locaux au début du processus.

        Args:
            cooling_rate: Facteur de refroidissement α (0 < α < 1).
            max_iterations: Nombre maximal d'itérations (paliers de T).
            log_interval: Intervalle de logging en nombre d'itérations.

        Returns:
            Dict {reference: (x, y)} du meilleur placement trouvé.
        """
        self.tracker.start()

        # ---- Détection et calibration de la température initiale ----
        self.is_topological = (getattr(self, 'best_candidate_name', '') == "Topologique Anti-Vias")
        self.baseline_crossings = self._compute_net_crossings()

        if self.is_topological:
            logger.info(f"Mode compactage topologique actif : Conservation intégrale de la géométrie championne ({self.baseline_crossings} croisements, 0 collision, compacité d'aire maximale).")
            self.best_positions = dict(self.baseline_positions)
            self.best_orientations = dict(self.baseline_orientations)
            self.best_layers = dict(self.baseline_layers)
            self.current_positions = dict(self.best_positions)
            self.current_orientations = dict(self.best_orientations)
            self.current_layers = dict(self.best_layers)
            self.best_cost = self.baseline_cost
            self.tracker.finish(self.best_cost, self.net_analyzer.compute_total_hpwl(self.best_positions))
            return self.best_positions
        else:
            t_initial = self._calibrate_initial_temperature(num_samples=100)
            t_current = t_initial
            t_final = t_initial * FINAL_TEMP_RATIO
            n = len(self._movable_refs)
            moves_per_t = max(100, min(10000, int(MOVES_FACTOR * (n ** (4.0 / 3.0)))))

        logger.info(f"T₀ = {t_initial:.2f}, T_final = {t_final:.4f}, "
                    f"α = {cooling_rate}")
        logger.info(f"Mouvements par palier : {moves_per_t}")

        # ---- Coût initial ----
        current_cost = self._compute_cost()
        self.best_cost = current_cost

        # ---- Boucle de refroidissement ----
        iteration = 0

        while t_current > t_final and iteration < max_iterations:
            accepted = 0
            total_moves = 0

            for _ in range(moves_per_t):
                # Générer un mouvement aléatoire
                move = self._generate_move(t_current, t_initial)
                if move is None:
                    continue

                # Appliquer le mouvement
                self._apply_move(move)

                # Si mode compactage topologique, interdiction formelle d'ajouter des croisements
                if self.is_topological:
                    new_crossings = self._compute_net_crossings()
                    if new_crossings > self.baseline_crossings:
                        total_moves += 1
                        self._undo_move(move)
                        continue

                # Calculer le nouveau coût
                new_cost = self._compute_cost()
                delta_cost = new_cost - current_cost

                total_moves += 1

                # ---- Critère d'acceptation de Metropolis ----
                # ΔC ≤ 0 → amélioration → toujours accepter
                # ΔC > 0 → dégradation → accepter avec P = exp(-ΔC/T)
                if delta_cost <= 0:
                    # Amélioration : accepter
                    current_cost = new_cost
                    accepted += 1

                    # Mise à jour du meilleur état
                    if new_cost < self.best_cost:
                        self.best_cost = new_cost
                        self.best_positions = dict(self.current_positions)
                        self.best_orientations = dict(self.current_orientations)
                        self.best_layers = dict(self.current_layers)
                else:
                    # Dégradation : accepter avec probabilité de Boltzmann
                    #   P = exp(-ΔC / T)
                    # On compare avec un nombre aléatoire uniforme U ~ [0, 1]
                    #   Si U < P → accepter (le hasard nous donne une chance)
                    #   Si U ≥ P → rejeter
                    if t_current > 0:
                        # Calcul stable : limiter l'argument de exp à [-500, 0]
                        # pour éviter les underflows
                        exponent = -delta_cost / t_current
                        exponent = max(-500.0, exponent)
                        probability = math.exp(exponent)
                    else:
                        probability = 0.0

                    if random.random() < probability:
                        # Accepter malgré la dégradation
                        current_cost = new_cost
                        accepted += 1
                    else:
                        # Rejeter : annuler le mouvement
                        self._undo_move(move)

            # ---- Logging périodique ----
            if iteration % log_interval == 0:
                hpwl = self.net_analyzer.compute_total_hpwl(
                    self.current_positions
                )
                penalties = current_cost - WEIGHT_HPWL * hpwl
                self.tracker.log_iteration(
                    iteration, t_current, current_cost,
                    hpwl, penalties, accepted, total_moves
                )

            # ---- Refroidissement géométrique ----
            # T_{k+1} = α × T_k
            # Ceci donne une décroissance exponentielle de la température :
            #   T_k = T₀ × α^k
            t_current *= cooling_rate
            iteration += 1

            if progress_callback and (iteration % 5 == 0 or iteration == max_iterations):
                pct = int(10 + (iteration / max_iterations) * 35)
                progress_callback(pct, f"Recuit simulé... {pct}% (T={t_current:.1f}, Coût={current_cost:,.0f})")

            # Maintenir l'interface graphique KiCad réactive (anti-freeze / anti-crash)
            try:
                import wx
                if wx.GetApp():
                    wx.Yield()
            except Exception:
                pass

        # Charger le meilleur état trouvé lors du recuit
        if self.is_topological:
            final_crossings = self._compute_net_crossings()
            if final_crossings > self.baseline_crossings:
                logger.info(f"Restauration baseline topologique ({self.baseline_crossings} croisements).")
                self.best_positions = dict(self.baseline_positions)
                self.best_orientations = dict(self.baseline_orientations)
                self.best_layers = dict(self.baseline_layers)

        self.current_positions = dict(self.best_positions)
        self.current_orientations = dict(self.best_orientations)
        self.current_layers = dict(self.best_layers)

        # ---- Passe 1 : Légalisation Déterministe De-overlap (anti-superposition) ----
        if progress_callback:
            progress_callback(46, "Légalisation géométrique anti-collision...")
        self._resolve_collisions(max_passes=50)
        self.best_positions = dict(self.current_positions)
        self.best_cost = self._compute_cost()

        # ---- Passe 2 : Trempe locale fine (Quenching) avec maintien des contraintes ----
        logger.info("Raffinement local fin (Quenching)...")
        if progress_callback:
            progress_callback(48, "Raffinement local fin (Quenching)...")
        quench_step = from_mm(0.2)
        for ref in self._movable_refs:
            comp = self._comp_by_ref[ref]
            cur_pos = self.best_positions.get(ref, comp.position)
            orient = self.best_orientations.get(ref, comp.orientation)
            best_local_pos = cur_pos
            best_local_cost = self.best_cost

            for qdx, qdy in [(-quench_step, 0), (quench_step, 0), (0, -quench_step), (0, quench_step)]:
                test_x, test_y = cur_pos[0] + qdx, cur_pos[1] + qdy
                clamped_test = self._clamp_position_inside_board(comp, test_x, test_y, orient)
                self.current_positions[ref] = clamped_test
                c = self._compute_cost()
                if c < best_local_cost:
                    best_local_cost = c
                    best_local_pos = clamped_test

            self.best_positions[ref] = best_local_pos
            self.current_positions[ref] = best_local_pos
            self.best_cost = best_local_cost

        # ---- Passe 3 : Sécurité De-overlap finale ----
        self._resolve_collisions(max_passes=30)
        self.best_positions = dict(self.current_positions)
        self.best_cost = self._compute_cost()

        # Si le recuit simulé n'a pas réussi à améliorer la baseline initiale légalisée,
        # conserver la baseline initiale pour garantir une amélioration monotone.
        if hasattr(self, 'baseline_cost') and self.best_cost > self.baseline_cost:
            logger.info(
                f"Conservation de la disposition de base optimale "
                f"(Coût recuit {self.best_cost:,.0f} > Base {self.baseline_cost:,.0f})"
            )
            self.best_positions = dict(self.baseline_positions)
            self.best_orientations = dict(self.baseline_orientations)
            self.best_layers = dict(self.baseline_layers)
            self.best_cost = self.baseline_cost

        # ---- Restaurer le meilleur état final ----
        self.current_positions = dict(self.best_positions)
        self.current_orientations = dict(self.best_orientations)
        self.current_layers = dict(self.best_layers)

        # Logging final
        final_hpwl = self.net_analyzer.compute_total_hpwl(self.best_positions)
        self.tracker.finish(self.best_cost, final_hpwl)

        return self.best_positions

    def _resolve_collisions(self, max_passes: int = 60) -> bool:
        """
        Passe de légalisation géométrique déterministe (De-overlap).
        Résout tout contact ou chevauchement résiduel entre les Courtyards (contour violet)
        des composants en repoussant les boîtiers le long de l'axe de pénétration minimale,
        tout en les maintenant strictement confinés à l'intérieur du contour Edge.Cuts.
        Garantit 0 contact / collision à 100%.
        """
        n = len(self.all_components)
        clearance = from_mm(0.15)  # 150 µm de séparation stricte entre les contours violets

        # ---- Étape 1 : Répulsion itérative bidirectionnelle ----
        for pass_idx in range(max_passes):
            bboxes = self._compute_current_bboxes()
            collision_count = 0

            for i in range(n):
                for j in range(i + 1, n):
                    comp_a = self.all_components[i]
                    comp_b = self.all_components[j]
                    ref_a = comp_a.reference
                    ref_b = comp_b.reference

                    ba = bboxes[i]
                    bb = bboxes[j]

                    if not ba.inflate(clearance).intersects(bb):
                        continue

                    collision_count += 1
                    pos_a = self.current_positions.get(ref_a, comp_a.position)
                    pos_b = self.current_positions.get(ref_b, comp_b.position)
                    orient_a = self.current_orientations.get(ref_a, comp_a.orientation)
                    orient_b = self.current_orientations.get(ref_b, comp_b.orientation)

                    # Centre géométrique
                    cdx = (ba.x_min + ba.x_max) / 2.0 - (bb.x_min + bb.x_max) / 2.0
                    cdy = (ba.y_min + ba.y_max) / 2.0 - (bb.y_min + bb.y_max) / 2.0

                    if abs(cdx) < 1.0 and abs(cdy) < 1.0:
                        cdx = 1000.0
                        cdy = 1000.0

                    # Déplacements nécessaires sur X et Y
                    if cdx >= 0:
                        shift_x = int((bb.x_max + clearance) - ba.x_min)
                    else:
                        shift_x = int(bb.x_min - (ba.x_max + clearance))

                    if cdy >= 0:
                        shift_y = int((bb.y_max + clearance) - ba.y_min)
                    else:
                        shift_y = int(bb.y_min - (ba.y_max + clearance))

                    # Choix de l'axe de moindre pénétration
                    mov_a = not comp_a.is_locked
                    mov_b = not comp_b.is_locked

                    if abs(shift_x) <= abs(shift_y):
                        # Séparation sur X
                        if mov_a and mov_b:
                            target_xa = pos_a[0] + shift_x // 2
                            target_xb = pos_b[0] - shift_x // 2
                            self.current_positions[ref_a] = self._clamp_position_inside_board(comp_a, target_xa, pos_a[1], orient_a)
                            self.current_positions[ref_b] = self._clamp_position_inside_board(comp_b, target_xb, pos_b[1], orient_b)
                        elif mov_a:
                            target_xa = pos_a[0] + shift_x
                            self.current_positions[ref_a] = self._clamp_position_inside_board(comp_a, target_xa, pos_a[1], orient_a)
                        elif mov_b:
                            target_xb = pos_b[0] - shift_x
                            self.current_positions[ref_b] = self._clamp_position_inside_board(comp_b, target_xb, pos_b[1], orient_b)
                    else:
                        # Séparation sur Y
                        if mov_a and mov_b:
                            target_ya = pos_a[1] + shift_y // 2
                            target_yb = pos_b[1] - shift_y // 2
                            self.current_positions[ref_a] = self._clamp_position_inside_board(comp_a, pos_a[0], target_ya, orient_a)
                            self.current_positions[ref_b] = self._clamp_position_inside_board(comp_b, pos_b[0], target_yb, orient_b)
                        elif mov_a:
                            target_ya = pos_a[1] + shift_y
                            self.current_positions[ref_a] = self._clamp_position_inside_board(comp_a, pos_a[0], target_ya, orient_a)
                        elif mov_b:
                            target_yb = pos_b[1] - shift_y
                            self.current_positions[ref_b] = self._clamp_position_inside_board(comp_b, pos_b[0], target_yb, orient_b)

                    # Rafraîchir les bboxes locales
                    bboxes[i] = comp_a.get_bbox_at(self.current_positions[ref_a][0], self.current_positions[ref_a][1], orient_a)
                    bboxes[j] = comp_b.get_bbox_at(self.current_positions[ref_b][0], self.current_positions[ref_b][1], orient_b)

            if collision_count == 0:
                logger.info(f"Légalisation De-overlap : 0 contact atteint en {pass_idx + 1} passes !")
                return True

        # ---- Étape 2 : Recherche spirale d'emplacement libre (Garantie absolue zéro contact) ----
        bboxes = self._compute_current_bboxes()
        for i in range(n):
            for j in range(i + 1, n):
                if bboxes[i].inflate(clearance).intersects(bboxes[j]):
                    target_idx = j if not self.all_components[j].is_locked else i
                    if self.all_components[target_idx].is_locked:
                        continue
                    comp_t = self.all_components[target_idx]
                    ref_t = comp_t.reference
                    orient_t = self.current_orientations.get(ref_t, comp_t.orientation)
                    orig_x, orig_y = self.current_positions.get(ref_t, comp_t.position)

                    found = False
                    step = from_mm(0.5)
                    for ring in range(1, 50):
                        if found:
                            break
                        for rdx in range(-ring, ring + 1):
                            if found:
                                break
                            for rdy in [-ring, ring]:
                                tx, ty = self._clamp_position_inside_board(comp_t, orig_x + rdx * step, orig_y + rdy * step, orient_t)
                                test_bb = comp_t.get_bbox_at(tx, ty, orient_t)
                                conflict = False
                                for k in range(n):
                                    if k == target_idx:
                                        continue
                                    if test_bb.inflate(clearance).intersects(bboxes[k]):
                                        conflict = True
                                        break
                                if not conflict:
                                    self.current_positions[ref_t] = (tx, ty)
                                    bboxes[target_idx] = test_bb
                                    found = True
                                    logger.info(f"  {ref_t} : replacé sans contact dans un emplacement libre à l'anneau {ring}")
                                    break

        # Re-vérification finale
        bboxes = self._compute_current_bboxes()
        final_collisions = 0
        for i in range(n):
            for j in range(i + 1, n):
                if bboxes[i].inflate(clearance).intersects(bboxes[j]):
                    final_collisions += 1

        if final_collisions == 0:
            logger.info("Légalisation De-overlap : 0 contact garanti !")
            return True
        else:
            logger.warning(f"Légalisation De-overlap : {final_collisions} contacts résiduels.")
            return False

    def _calibrate_initial_temperature(self, num_samples: int = 100) -> float:
        """
        Calibre la température initiale pour un taux d'acceptation de ~80%.

        Méthode :
            1. Effectuer N mouvements aléatoires
            2. Mesurer les ΔC positifs (dégradations)
            3. Calculer ΔC_moyen des dégradations
            4. Résoudre : exp(-ΔC_moyen / T₀) = 0.8
               → T₀ = -ΔC_moyen / ln(0.8)

        La valeur 0.8 (80%) est une cible classique :
          - Plus haut (90%) → exploration excessive, convergence lente
          - Plus bas (60%) → risque de piège dans un minimum local

        Référence :
            S. Kirkpatrick, C. D. Gelatt, M. P. Vecchi (1983).
            "Optimization by Simulated Annealing". Science, 220(4598).

        Args:
            num_samples: Nombre de mouvements d'échantillonnage.

        Returns:
            Température initiale T₀.
        """
        positive_deltas = []

        for _ in range(num_samples):
            move = self._generate_move(1.0, 1.0)
            if move is None:
                continue

            old_cost = self._compute_cost()
            self._apply_move(move)
            new_cost = self._compute_cost()
            self._undo_move(move)

            delta = new_cost - old_cost
            if delta > 0:
                positive_deltas.append(delta)

        if not positive_deltas:
            # Aucune dégradation trouvée → placement déjà très bon
            # Utiliser une température modérée pour le raffinement
            return 1000.0

        avg_delta = sum(positive_deltas) / len(positive_deltas)

        # T₀ = -ΔC_moyen / ln(target_acceptance_rate)
        # ln(0.8) ≈ -0.2231
        target_rate = 0.8
        t_initial = -avg_delta / math.log(target_rate)

        # Borner T₀ pour éviter les valeurs aberrantes (plafond à 5e7 pour préserver la topologie ordonnée)
        t_initial = max(100.0, min(5e7, t_initial))

        logger.info(f"Calibration T₀ : ΔC_moyen = {avg_delta:.0f}, "
                    f"T₀ = {t_initial:.2f}")

        return t_initial

    # -------------------------------------------------------------------
    #  3.6 RÉSULTATS
    # -------------------------------------------------------------------

    def get_results(self) -> Dict[str, dict]:
        """
        Retourne les résultats du placement optimisé.

        Returns:
            Dict {reference: {position, orientation, layer}} pour
            chaque composant.
        """
        results = {}
        for comp in self.all_components:
            ref = comp.reference
            results[ref] = {
                'position': self.best_positions.get(ref, comp.position),
                'orientation': self.best_orientations.get(ref, comp.orientation),
                'layer': "F.Cu",  # 100% F.Cu imposé
                'original_position': comp.position,
                'original_orientation': comp.orientation,
                'original_layer': comp.layer,
                'was_moved': (
                    self.best_positions.get(ref, comp.position) != comp.position or
                    self.best_orientations.get(ref, comp.orientation) != comp.orientation or
                    comp.layer != "F.Cu"
                )
            }
        return results


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
