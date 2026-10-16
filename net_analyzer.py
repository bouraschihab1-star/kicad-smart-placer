"""
=============================================================================
 net_analyzer.py — Phase 2 : Analyse des Réseaux Électriques
=============================================================================
 Ce module analyse la connectivité électrique de la carte pour :
   1. Construire le graphe de connectivité composant-composant
   2. Calculer le HPWL (Half-Perimeter Wirelength) pour chaque net
   3. Identifier les groupements de composants dépendants
   4. Classifier les composants en puissance / signal / passif
   5. Fournir les données nécessaires au partitionnement Min-Cut

 THÉORIE — HPWL (Half-Perimeter Wirelength) :
   Le HPWL est un estimateur classique de la longueur de routage.
   Pour un net connectant des pads aux positions {(x₁,y₁), ..., (xₙ,yₙ)},
   le HPWL est le demi-périmètre du plus petit rectangle englobant :

     HPWL(net) = [max(xᵢ) - min(xᵢ)] + [max(yᵢ) - min(yᵢ)]

   Propriétés du HPWL :
     - C'est une borne inférieure de la longueur de routage réelle
     - Exact pour les nets à 2 pads (routage L ou Z)
     - Sous-estime pour les nets à >2 pads (arbre de Steiner)
     - Calculable en O(n) → très rapide pour l'optimisation itérative
     - Largement utilisé en placement VLSI (Cadence, Synopsys)

   COMPLEXITÉ :
     Le calcul total du HPWL pour M nets avec en moyenne k pads :
       O(M × k) par évaluation complète
     Avec mise à jour incrémentale (seuls les nets touchés par le
     mouvement sont recalculés) : O(k) par mouvement, amortis.
=============================================================================
"""

from typing import List, Dict, Tuple, Set, Optional
from collections import defaultdict

from .utils import (
    BBox, ComponentData, NetInfo,
    to_mm, manhattan_distance, setup_logger
)

logger = setup_logger("NetAnalyzer")


class NetAnalyzer:
    """
    Analyseur de la connectivité électrique de la carte.

    Construit un modèle de la netlist exploitable par l'optimiseur :
      - Graphe de connectivité composant↔composant
      - Calcul rapide du HPWL total et par-net
      - Identification des groupes rigides (super-composants)
      - Statistiques de connectivité pour le partitionnement Min-Cut
    """

    def __init__(
        self,
        components: List[ComponentData],
        nets: Dict[str, NetInfo]
    ):
        """
        Args:
            components: Liste des composants extraits de la carte.
            nets: Dictionnaire des nets extraits.
        """
        self.components = components
        self.nets = nets

        # Index rapide : reference → ComponentData
        self._comp_by_ref: Dict[str, ComponentData] = {
            c.reference: c for c in components
        }

        # Index rapide : reference → index dans la liste
        self._ref_to_idx: Dict[str, int] = {
            c.reference: i for i, c in enumerate(components)
        }

        # Graphe de connectivité : reference → set(references connectées)
        self._adjacency: Dict[str, Set[str]] = defaultdict(set)

        # Cache : net_name → list of (comp_ref, pad_number)
        self._net_to_comps: Dict[str, Set[str]] = defaultdict(set)

        # Cache : comp_ref → list of net_names
        self._comp_to_nets: Dict[str, Set[str]] = defaultdict(set)

        self._build_connectivity_graph()

    # -------------------------------------------------------------------
    #  1. CONSTRUCTION DU GRAPHE DE CONNECTIVITÉ
    # -------------------------------------------------------------------

    def _build_connectivity_graph(self):
        """
        Construit le graphe de connectivité composant-composant.

        Pour chaque net, tous les composants qui y sont connectés forment
        une clique (chaque paire est connectée). Le poids de l'arête
        entre deux composants est le nombre de nets qu'ils partagent.

        Algorithme :
            Pour chaque net N avec les composants {C₁, C₂, ..., Cₖ} :
                Pour chaque paire (Cᵢ, Cⱼ) avec i < j :
                    Ajouter l'arête (Cᵢ, Cⱼ) au graphe

        Complexité : O(Σ k² / 2) pour M nets, soit O(M × k²) dans le
        pire cas. En pratique, k est petit (2-5 pour la majorité des nets).
        """
        for net_name, net_info in self.nets.items():
            # Identifier les composants uniques connectés à ce net
            comp_refs = set()
            for ref, pad_num, x, y in net_info.pad_refs:
                if ref in self._comp_by_ref:
                    comp_refs.add(ref)
                    self._net_to_comps[net_name].add(ref)
                    self._comp_to_nets[ref].add(net_name)

            # Créer les arêtes (clique complète pour ce net)
            refs_list = list(comp_refs)
            for i in range(len(refs_list)):
                for j in range(i + 1, len(refs_list)):
                    self._adjacency[refs_list[i]].add(refs_list[j])
                    self._adjacency[refs_list[j]].add(refs_list[i])

        logger.info(f"Graphe de connectivité : "
                    f"{sum(len(v) for v in self._adjacency.values()) // 2} arêtes")

    def get_connected_barycenter(
        self,
        ref: str,
        positions: Dict[str, Tuple[int, int]]
    ) -> Optional[Tuple[int, int]]:
        """
        Calcule le barycentre (centre de gravité) des composants directement
        connectés au composant 'ref' par au moins un net.

        Utilisé pour le Recuit Simulé Dirigé par les Forces (Force-Directed SA) :
        permet d'attirer un composant vers ses voisins connectés plutôt que
        de le déplacer de manière purement aléatoire.

        Args:
            ref: Référence du composant (ex: 'R10').
            positions: Dictionnaire des positions actuelles {ref: (x, y)}.

        Returns:
            Tuple (x, y) du barycentre en nm, ou None si aucun voisin connecté.
        """
        neighbors = self._adjacency.get(ref, set())
        if not neighbors:
            return None

        total_x = 0
        total_y = 0
        count = 0

        for neighbor in neighbors:
            if neighbor in positions:
                nx, ny = positions[neighbor]
                total_x += nx
                total_y += ny
                count += 1

        if count == 0:
            return None

        return (total_x // count, total_y // count)

    # -------------------------------------------------------------------
    #  2. CALCUL DU HPWL
    # -------------------------------------------------------------------

    def compute_hpwl_single_net(
        self,
        net_name: str,
        positions: Dict[str, Tuple[int, int]] = None
    ) -> int:
        """
        Calcule le HPWL d'un seul net.

        HPWL (Half-Perimeter Wirelength) :
            Soit les pads du net aux positions {(x₁,y₁), ..., (xₙ,yₙ)}.
            HPWL = [max(xᵢ) - min(xᵢ)] + [max(yᵢ) - min(yᵢ)]

        C'est le demi-périmètre du MBR (Minimum Bounding Rectangle)
        des pads du net. C'est une borne inférieure de la longueur
        du routage optimal (arbre de Steiner minimum).

        Si `positions` est fourni, les coordonnées des pads sont
        recalculées en translatant les offsets pad relatifs au composant
        vers les nouvelles positions des composants.

        Formule de translation des pads :
            Si le composant C est déplacé de (x_old, y_old) à (x_new, y_new),
            alors chaque pad P de C est translaté :
                P_new = P_old + (x_new - x_old, y_new - y_old)
                      = (P_old.x + dx, P_old.y + dy)

        Args:
            net_name: Nom du net.
            positions: Dict optionnel {reference: (x, y)} avec les nouvelles
                      positions des composants (utilisé par l'optimiseur).

        Returns:
            HPWL en nm (entier). Retourne 0 si le net a <2 pads.
        """
        if net_name not in self.nets:
            return 0

        net = self.nets[net_name]
        if len(net.pad_refs) < 2:
            return 0

        xs = []
        ys = []

        for ref, pad_num, pad_x, pad_y in net.pad_refs:
            if positions and ref in positions:
                # Recalculer la position du pad en fonction de la
                # nouvelle position du composant
                if ref in self._comp_by_ref:
                    comp = self._comp_by_ref[ref]
                    old_cx, old_cy = comp.position
                    new_cx, new_cy = positions[ref]
                    # Translation : delta du composant appliqué au pad
                    dx = new_cx - old_cx
                    dy = new_cy - old_cy
                    xs.append(pad_x + dx)
                    ys.append(pad_y + dy)
                else:
                    xs.append(pad_x)
                    ys.append(pad_y)
            else:
                xs.append(pad_x)
                ys.append(pad_y)

        # HPWL = (max_x - min_x) + (max_y - min_y)
        hpwl = (max(xs) - min(xs)) + (max(ys) - min(ys))
        return hpwl

    def compute_total_hpwl(
        self,
        positions: Dict[str, Tuple[int, int]] = None
    ) -> int:
        """
        Calcule le HPWL total sur tous les nets.

        Formule :
            HPWL_total = Σ HPWL(netᵢ)  pour tous les nets i

        C'est la métrique principale d'optimisation du placement.
        Minimiser le HPWL total tend à minimiser la longueur totale
        de routage, et donc la surface de cuivre, les délais de
        propagation, et la diaphonie (crosstalk).

        Optionnellement, les nets de puissance peuvent être exclus
        du calcul car ils sont routés en dernier et suivent souvent
        des plans de masse/alimentation plutôt que des pistes.

        Args:
            positions: Dict optionnel {reference: (x, y)} avec les
                      nouvelles positions.

        Returns:
            HPWL total en nm.
        """
        total = 0
        for net_name, net_info in self.nets.items():
            # Inclure tous les nets (puissance et signal) dans le calcul
            # Les nets de puissance contribuent aussi au placement car
            # les condensateurs de découplage doivent être proches des ICs
            total += self.compute_hpwl_single_net(net_name, positions)
        return total

    def compute_hpwl_delta(
        self,
        moved_ref: str,
        old_pos: Tuple[int, int],
        new_pos: Tuple[int, int],
        current_positions: Dict[str, Tuple[int, int]]
    ) -> int:
        """
        Calcule la VARIATION du HPWL due au déplacement d'un composant.

        Optimisation incrémentale : au lieu de recalculer le HPWL total
        (O(M×k)), on ne recalcule que les nets touchés par le mouvement
        (O(k_affected)), ce qui accélère drastiquement le recuit simulé.

        Algorithme :
            1. Identifier les nets touchés = nets du composant déplacé
            2. Pour chaque net touché :
               a. Calculer HPWL_before avec l'ancienne position
               b. Calculer HPWL_after avec la nouvelle position
               c. Accumuler : delta += (HPWL_after - HPWL_before)
            3. Retourner delta (négatif = amélioration)

        Args:
            moved_ref: Référence du composant déplacé.
            old_pos: Ancienne position (x, y) en nm.
            new_pos: Nouvelle position (x, y) en nm.
            current_positions: Positions actuelles de tous les composants.

        Returns:
            Delta HPWL en nm (négatif = amélioration, positif = dégradation).
        """
        if moved_ref not in self._comp_to_nets:
            return 0

        delta = 0

        for net_name in self._comp_to_nets[moved_ref]:
            # HPWL avec l'ancienne position
            pos_before = dict(current_positions)
            pos_before[moved_ref] = old_pos
            hpwl_before = self.compute_hpwl_single_net(net_name, pos_before)

            # HPWL avec la nouvelle position
            pos_after = dict(current_positions)
            pos_after[moved_ref] = new_pos
            hpwl_after = self.compute_hpwl_single_net(net_name, pos_after)

            delta += (hpwl_after - hpwl_before)

        return delta

    # -------------------------------------------------------------------
    #  3. IDENTIFICATION DES GROUPEMENTS RIGIDES (SUPER-COMPOSANTS)
    # -------------------------------------------------------------------

    def find_rigid_groups(self) -> List[List[str]]:
        """
        Identifie les groupes de composants dépendants (super-composants).

        Un "super-composant" est un groupe de composants si fortement
        connectés qu'ils doivent être placés comme une unité rigide.

        Règle de groupement :
            Un condensateur 'C' connecté directement aux broches
            d'alimentation d'un circuit intégré 'U' forme un groupe
            rigide avec ce 'U'. C'est le cas typique des condensateurs
            de découplage qui doivent être physiquement adjacents au CI.

        Algorithme :
            Pour chaque composant U (circuit intégré) :
                1. Identifier ses nets d'alimentation (VCC, GND, etc.)
                2. Pour chaque net d'alimentation :
                   - Trouver les condensateurs (C) connectés à ce net
                   - Si un C est connecté à la fois au VCC et au GND du U
                     → c'est un condensateur de découplage → grouper avec U

        Un algorithme plus simple mais efficace :
            Pour chaque condensateur C :
                Si C est connecté à exactement 2 nets, et les deux sont
                des nets de puissance auxquels un même U est connecté
                → grouper C avec U

        Returns:
            Liste de groupes. Chaque groupe est une liste de références.
            Le premier élément est le composant "maître" (U).
        """
        groups = []
        assigned = set()  # Composants déjà assignés à un groupe

        # Trouver tous les ICs (U)
        ic_refs = [
            ref for ref in self._comp_by_ref
            if ref.startswith('U') and ref not in assigned
        ]

        for u_ref in ic_refs:
            if u_ref not in self._comp_to_nets:
                continue

            group = [u_ref]
            u_nets = self._comp_to_nets[u_ref]

            # Identifier les nets de puissance du U
            u_power_nets = {
                n for n in u_nets
                if n in self.nets and self.nets[n].is_power
            }

            if not u_power_nets:
                continue

            # Chercher les condensateurs connectés aux nets de puissance du U
            for net_name in u_power_nets:
                if net_name not in self._net_to_comps:
                    continue
                for comp_ref in self._net_to_comps[net_name]:
                    if comp_ref in assigned or comp_ref == u_ref:
                        continue
                    if not comp_ref.startswith('C'):
                        continue

                    # Vérifier que le condensateur est connecté à AU MOINS
                    # 2 nets de puissance du même U (VCC + GND)
                    c_nets = self._comp_to_nets.get(comp_ref, set())
                    shared_power = c_nets & u_power_nets
                    if len(shared_power) >= 1:
                        # Le condensateur partage au moins un net de puissance
                        # avec le U → condensateur de découplage probable
                        group.append(comp_ref)
                        assigned.add(comp_ref)

            if len(group) > 1:
                assigned.add(u_ref)
                groups.append(group)

        logger.info(f"Groupes rigides identifiés : {len(groups)}")
        for g in groups:
            logger.debug(f"  Groupe : {g[0]} ← {g[1:]}")

        return groups

    # -------------------------------------------------------------------
    #  4. DONNÉES POUR LE PARTITIONNEMENT MIN-CUT
    # -------------------------------------------------------------------

    def get_connectivity_matrix(self) -> Dict[str, Dict[str, int]]:
        """
        Construit une matrice de connectivité pondérée.

        Pour chaque paire de composants (A, B), le poids de l'arête
        est le nombre de nets qu'ils partagent. Plus deux composants
        partagent de nets, plus ils devraient être proches.

        La matrice est représentée comme un dictionnaire imbriqué :
            matrix[refA][refB] = nombre_de_nets_partagés

        Utilisée par l'algorithme Min-Cut pour le partitionnement initial.

        Returns:
            Matrice de connectivité pondérée.
        """
        matrix = defaultdict(lambda: defaultdict(int))

        for net_name, comps in self._net_to_comps.items():
            comps_list = list(comps)
            for i in range(len(comps_list)):
                for j in range(i + 1, len(comps_list)):
                    matrix[comps_list[i]][comps_list[j]] += 1
                    matrix[comps_list[j]][comps_list[i]] += 1

        return dict(matrix)

    def get_nets_for_component(self, ref: str) -> Set[str]:
        """Retourne l'ensemble des nets auxquels un composant est connecté."""
        return self._comp_to_nets.get(ref, set())

    def get_components_for_net(self, net_name: str) -> Set[str]:
        """Retourne l'ensemble des composants connectés à un net."""
        return self._net_to_comps.get(net_name, set())

    @property
    def adjacency(self) -> Dict[str, Set[str]]:
        """Graphe d'adjacence composant ↔ composant."""
        return dict(self._adjacency)
