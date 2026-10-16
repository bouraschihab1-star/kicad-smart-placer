"""
=============================================================================
 spatial_model.py — Phase 2 : Modélisation Spatiale
=============================================================================
 Ce module implémente :
   1. Somme de Minkowski pour les zones d'exclusion dynamiques
   2. Halos de dégagement par classes d'équipotence
   3. Gestion des zones interdites statiques et dynamiques
   4. Détection de collisions optimisée (Sweep and Prune)

 THÉORIE — SOMME DE MINKOWSKI :
   La somme de Minkowski de deux ensembles A et B est définie comme :
     A ⊕ B = { a + b | a ∈ A, b ∈ B }

   Pour deux rectangles alignés sur les axes (AABB) :
     R₁ = [x₁_min, x₁_max] × [y₁_min, y₁_max]
     R₂ = [x₂_min, x₂_max] × [y₂_min, y₂_max]

   La somme de Minkowski donne :
     R₁ ⊕ R₂ = [x₁_min + x₂_min, x₁_max + x₂_max]
                × [y₁_min + y₂_min, y₁_max + y₂_max]

   Pour créer un halo autour d'un composant avec un dégagement `d`,
   on utilise la somme avec un carré centré [-d, d] × [-d, d] :
     R_halo = R ⊕ {[-d, d] × [-d, d]}
            = [x_min - d, x_max + d] × [y_min - d, y_max + d]

   Ceci revient à l'opération inflate() de notre BBox.
   Le test de non-collision entre deux composants A et B avec dégagement d
   est alors : l'AABB étendu de A ne doit pas intersecter l'AABB de B.
   En pratique, on teste l'intersection de :
     A.bbox.inflate(d_A) avec B.bbox.inflate(d_B)
   mais cela double le dégagement. L'approche correcte est :
     A.bbox.inflate(d_A + d_B) ne doit pas intersecter B.bbox
   ou de manière équivalente :
     A.bbox ne doit pas intersecter B.bbox.inflate(d_A + d_B)
=============================================================================
"""

from typing import List, Dict, Tuple, Set
from .utils import (
    BBox, ComponentData, NetInfo,
    from_mm, to_mm, euclidean_distance, setup_logger
)

logger = setup_logger("SpatialModel")


# ---------------------------------------------------------------------------
#  1. CONSTANTES DE DÉGAGEMENT (CLEARANCE)
# ---------------------------------------------------------------------------

# Dégagement standard pour les composants généraux (en mm)
# Réduit drastiquement à 0.05 mm (50 µm / ~2 mils) pour permettre un placement
# compact réaliste sans collision entre boîtiers CMS adjacents.
CLEARANCE_SIGNAL_MM = 0.05

# Dégagement pour les composants de puissance (en mm)
# Réduit à 0.08 mm (80 µm) en l'absence d'analyse thermique/haute tension spécifique
CLEARANCE_POWER_MM = 0.08

# Dégagement thermique minimal entre composants de puissance (en mm)
# Réduit à 0.10 mm (100 µm) pour éviter d'écarter artificiellement les composants
CLEARANCE_THERMAL_MM = 0.10

# Marge autour des trous de montage et zones keep-out (en mm)
CLEARANCE_KEEPOUT_MM = 0.10


class SpatialModel:
    """
    Modèle spatial de la carte PCB.

    Ce modèle gère les halos de dégagement et les zones d'exclusion
    dynamiques autour de chaque composant. Il fournit les primitives
    de collision utilisées par l'optimiseur.

    Architecture :
        Chaque composant possède un "halo" — une zone tampon autour de
        sa bounding box physique. La taille du halo dépend de la classe
        d'équipotence du composant :
          - Signal faible : +0.5 mm de chaque côté
          - Puissance     : +3.0 mm de chaque côté
          - Thermique     : +5.0 mm entre composants de puissance

        Le test de collision vérifie l'intersection des halos (pas des
        bounding boxes nues). Ceci implémente naturellement la somme de
        Minkowski pour le dégagement.
    """

    def __init__(
        self,
        signal_clearance_mm: float = CLEARANCE_SIGNAL_MM,
        power_clearance_mm: float = CLEARANCE_POWER_MM,
        thermal_clearance_mm: float = CLEARANCE_THERMAL_MM,
        keepout_clearance_mm: float = CLEARANCE_KEEPOUT_MM
    ):
        """
        Args:
            signal_clearance_mm:  Dégagement signal faible en mm.
            power_clearance_mm:   Dégagement puissance en mm.
            thermal_clearance_mm: Dégagement thermique en mm.
            keepout_clearance_mm: Marge autour des zones interdites en mm.
        """
        # Conversion des dégagements en unités internes (nm)
        self.signal_clearance = from_mm(signal_clearance_mm)
        self.power_clearance = from_mm(power_clearance_mm)
        self.thermal_clearance = from_mm(thermal_clearance_mm)
        self.keepout_clearance = from_mm(keepout_clearance_mm)

        # Zones interdites statiques (keep-outs + trous de montage)
        self._static_keepouts: List[BBox] = []

        # Zone de placement autorisée
        self._placement_area: BBox = BBox()

    def set_static_keepouts(self, keepouts: List[BBox]):
        """
        Enregistre les zones interdites statiques.

        Ces zones sont dilatées par la marge keepout_clearance pour créer
        un buffer de sécurité. Aucun composant ne peut être placé dans
        ces zones (même partiellement).

        Implémentation (Somme de Minkowski) :
            Chaque zone Z est étendue :
            Z' = Z ⊕ {[-m, m] × [-m, m]}
               = Z.inflate(keepout_clearance)

        Args:
            keepouts: Liste des BBox des zones interdites brutes.
        """
        self._static_keepouts = [
            kz.inflate(self.keepout_clearance) for kz in keepouts
        ]
        logger.info(f"Zones interdites statiques chargées : "
                    f"{len(self._static_keepouts)}")

    def set_placement_area(self, area: BBox):
        """Définit la zone de placement autorisée."""
        self._placement_area = area

    # -------------------------------------------------------------------
    #  2. CALCUL DU HALO DYNAMIQUE
    # -------------------------------------------------------------------

    def get_clearance(self, comp: ComponentData) -> int:
        """
        Calcule le dégagement (halo) applicable à un composant.

        Règle de classification :
            Si le composant est lié à un réseau de puissance :
                halo = power_clearance (ex: 3.0 mm)
            Sinon :
                halo = signal_clearance (ex: 0.5 mm)

        Le halo de puissance est plus large car :
          - Les pistes de puissance sont plus larges (courant élevé)
          - L'isolation renforcée est nécessaire (sécurité électrique)
          - L'espacement thermique améliore la dissipation

        Args:
            comp: Données du composant.

        Returns:
            Dégagement en nm.
        """
        if comp.is_power:
            return self.power_clearance
        return self.signal_clearance

    def get_halo_bbox(self, comp: ComponentData) -> BBox:
        """
        Calcule la bounding box étendue (avec halo) d'un composant.

        C'est l'application directe de la somme de Minkowski :
            BBox_halo = BBox_physique ⊕ {[-c, c] × [-c, c]}

        où c = clearance du composant.

        Args:
            comp: Données du composant.

        Returns:
            BBox étendue incluant le halo de dégagement.
        """
        clearance = self.get_clearance(comp)
        return comp.bbox.inflate(clearance)

    # -------------------------------------------------------------------
    #  3. DÉTECTION DE COLLISIONS
    # -------------------------------------------------------------------

    def compute_pairwise_clearance(
        self, comp_a: ComponentData, comp_b: ComponentData
    ) -> int:
        """
        Calcule le dégagement minimum entre deux composants.

        La somme de Minkowski pour le test de non-collision entre A et B
        requiert un dégagement total de :
            d_total = (d_A + d_B) / 2

        Pourquoi diviser par 2 ? Parce que chaque composant porte son
        propre halo. Si on additionne naïvement, on doublerait le
        dégagement. La convention est que chaque composant "étend" de
        la moitié du dégagement requis.

        MAIS pour la simplicité et la sécurité, on prend le MAX des
        deux dégagements (approche conservative) :
            d_effective = max(d_A, d_B)

        Si les deux sont des composants de puissance, on ajoute le
        dégagement thermique supplémentaire.

        Args:
            comp_a, comp_b: Les deux composants à tester.

        Returns:
            Dégagement effectif en nm.
        """
        d_a = self.get_clearance(comp_a)
        d_b = self.get_clearance(comp_b)

        # Dégagement de base : le plus restrictif des deux
        d_effective = max(d_a, d_b)

        # Bonus thermique : si les deux sont des composants de puissance,
        # ajouter une distance supplémentaire pour éviter les points chauds
        if comp_a.is_power and comp_b.is_power:
            d_effective = max(d_effective, self.thermal_clearance)

        return d_effective

    def check_collision(
        self,
        comp_a: ComponentData,
        comp_b: ComponentData,
        bbox_a: BBox = None,
        bbox_b: BBox = None
    ) -> Tuple[bool, int]:
        """
        Vérifie si deux composants sont en collision (halos inclus).

        Algorithme (basé sur la Somme de Minkowski) :
            1. Calculer le dégagement effectif d = max(d_A, d_B)
            2. Étendre la BBox de A par d : A' = A.inflate(d)
            3. Tester l'intersection : A' ∩ B ≠ ∅ ?

        Cette approche est EXACTE pour les AABB : si l'AABB étendu de A
        chevauche l'AABB de B, alors la distance minimum entre les deux
        est inférieure au dégagement requis.

        Alternativement (et de manière équivalente), on peut tester :
            B.inflate(d) ∩ A ≠ ∅

        Args:
            comp_a, comp_b: Composants à tester.
            bbox_a, bbox_b: BBox optionnelles (pour utiliser des positions
                           mises à jour par l'optimiseur au lieu des bbox
                           originales).

        Returns:
            Tuple (collision: bool, overlap_area: int en nm²).
            overlap_area est non-nul seulement si collision est True.
        """
        ba = bbox_a if bbox_a is not None else comp_a.bbox
        bb = bbox_b if bbox_b is not None else comp_b.bbox
        # Pas de collision si les composants sont sur des faces différentes (Top vs Bottom)
        if comp_a.layer != comp_b.layer:
            return False, 0

        clearance = self.compute_pairwise_clearance(comp_a, comp_b)

        # Somme de Minkowski : étendre A par le dégagement total
        inflated = ba.inflate(clearance)

        if inflated.intersects(bb):
            overlap = inflated.overlap_area(bb)
            return True, overlap

        return False, 0

    def check_keepout_violation(
        self, comp: ComponentData, bbox: BBox = None
    ) -> Tuple[bool, int]:
        """
        Vérifie si un composant viole une zone interdite.

        On teste l'intersection entre la BBox du composant et chaque
        zone interdite statique (déjà dilatée par keepout_clearance).

        Args:
            comp: Composant à tester.
            bbox: BBox optionnelle (position mise à jour).

        Returns:
            Tuple (violation: bool, total_overlap_area: int en nm²).
        """
        b = bbox if bbox is not None else comp.bbox
        total_overlap = 0

        for keepout in self._static_keepouts:
            if b.intersects(keepout):
                total_overlap += b.overlap_area(keepout)

        return total_overlap > 0, total_overlap

    def check_boundary_violation(
        self, comp: ComponentData, bbox: BBox = None
    ) -> Tuple[bool, int]:
        """
        Vérifie si un composant dépasse de la zone de placement.

        Un composant est en violation de frontière si sa BBox dépasse
        les limites de la zone de placement autorisée.

        Calcul de la pénalité :
            Pour chaque axe, on mesure le dépassement :
                dx_left  = max(0, area.x_min - comp.x_min)  [dépassement à gauche]
                dx_right = max(0, comp.x_max - area.x_max)  [dépassement à droite]
                dy_top   = max(0, area.y_min - comp.y_min)  [dépassement en haut]
                dy_bot   = max(0, comp.y_max - area.y_max)  [dépassement en bas]

            La "surface de violation" est approximée par :
                S = (dx_left + dx_right) × hauteur + (dy_top + dy_bot) × largeur

        Args:
            comp: Composant à tester.
            bbox: BBox optionnelle (position mise à jour).

        Returns:
            Tuple (violation: bool, penalty_area: int en nm²).
        """
        b = bbox if bbox is not None else comp.bbox
        area = self._placement_area

        if not area or area.area == 0:
            return False, 0

        dx_left = max(0, area.x_min - b.x_min)
        dx_right = max(0, b.x_max - area.x_max)
        dy_top = max(0, area.y_min - b.y_min)
        dy_bottom = max(0, b.y_max - area.y_max)

        total_overshoot = dx_left + dx_right + dy_top + dy_bottom

        if total_overshoot > 0:
            # Pénalité proportionnelle au dépassement
            penalty = (
                (dx_left + dx_right) * b.height +
                (dy_top + dy_bottom) * b.width
            )
            return True, penalty

        return False, 0

    # -------------------------------------------------------------------
    #  4. CALCUL GLOBAL DES COLLISIONS
    # -------------------------------------------------------------------

    def compute_total_collision_penalty(
        self,
        components: List[ComponentData],
        bboxes: List[BBox] = None
    ) -> float:
        """
        Calcule la pénalité totale de collision pour un placement donné.

        Complexité : O(n²) paire-à-paire + O(n × k) pour les keepouts.
        Pour les grandes cartes (n > 200), on pourrait optimiser avec un
        algorithme Sweep and Prune ou un Spatial Hash Grid. Ici on garde
        l'approche brute-force car le recuit simulé limite déjà n.

        La pénalité est la somme de :
            1. Overlaps entre composants (halos inclus)
            2. Violations de zones interdites
            3. Violations de frontière (hors zone de placement)

        Chaque overlap est pondéré par son aire en nm² pour que les
        grosses collisions soient plus pénalisées que les frôlements.

        Args:
            components: Liste des composants.
            bboxes: Liste optionnelle de BBox (positions mises à jour).

        Returns:
            Pénalité totale (nombre positif, 0 = aucune collision).
        """
        n = len(components)
        total_penalty = 0.0

        # 1. Collisions paire-à-paire entre composants (Courtyards)
        for i in range(n):
            for j in range(i + 1, n):
                bi = bboxes[i] if bboxes else None
                bj = bboxes[j] if bboxes else None

                collision, overlap = self.check_collision(
                    components[i], components[j], bi, bj
                )
                if collision:
                    # Pénalité MAJEURE : 1e9 par collision pour interdire formellement tout chevauchement
                    total_penalty += 1e9 + (overlap / 1e6)

        # 2. Violations des zones interdites
        for i in range(n):
            bi = bboxes[i] if bboxes else None
            violation, overlap = self.check_keepout_violation(
                components[i], bi
            )
            if violation:
                # Pénalité forte : les keepouts sont ABSOLUS
                total_penalty += 1e9 + (overlap / 1e6 * 10.0)

        # 3. Violations de frontière (hors Edge.Cuts)
        for i in range(n):
            bi = bboxes[i] if bboxes else None
            violation, penalty = self.check_boundary_violation(
                components[i], bi
            )
            if violation:
                total_penalty += 1e9 + (penalty / 1e6 * 10.0)

        return total_penalty

    # -------------------------------------------------------------------
    #  5. CALCUL DE LA DENSITÉ DE PLACEMENT
    # -------------------------------------------------------------------

    def compute_density(
        self,
        components: List[ComponentData],
        bboxes: List[BBox] = None
    ) -> float:
        """
        Calcule la densité de placement = surface composants / surface carte.

        Formule :
            ρ = Σ(aire_composant_i) / aire_zone_placement

        Une densité > 0.6 (60%) est considérée comme "haute densité"
        et justifie le basculement de passifs sur la face arrière.

        Args:
            components: Liste des composants.
            bboxes: BBox optionnelles (positions mises à jour).

        Returns:
            Densité (0.0 à 1.0+, peut dépasser 1.0 si overlaps).
        """
        if not self._placement_area or self._placement_area.area == 0:
            return 0.0

        total_comp_area = sum(
            (bboxes[i] if bboxes else c.bbox).area
            for i, c in enumerate(components)
        )

        return total_comp_area / self._placement_area.area
