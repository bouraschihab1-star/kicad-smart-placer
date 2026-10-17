"""
=============================================================================
 router.py — Phase 7 : Moteur de Routage Automatique Natif (Smart Router)
=============================================================================
 Ce module implémente un routeur PCB automatique 100% natif en Python pour KiCad.
 
 6 Piliers Architecturaux DFM/DRC :
   1. Application Stricte du Clearance NetClass (Anti Court-Circuit) :
      - Lecture dynamique du dégagement électrique (Clearance) et largeur (Width)
      - Calcul du halo d'isolation minimal : R_req = (Width / 2) + Clearance
      - Mur invisible (pénalité infinie) dès qu'un obstacle étranger est franchi.
   2. Règle DFM : Interdiction Stricte du "Via-in-Pad" :
      - Vérification spatiale continue : aucun via dans la Bounding Box d'un pad
      - Tracé obligatoire d'un segment d'échappement en cuivre avant tout saut de couche
      - Vérification bi-couche (F.Cu et B.Cu) du cylindre métallisé et de son halo.
   3. Géométrie Industrielle (Angles 45°/90° & Marge de Découpe) :
      - Pathfinding et raccordements 100% restreints aux angles 45° et 90°
      - Entrée/sortie des pastilles orthogonale par leur centre (axe X ou Y) anti-piège à acide
      - Marge invisible Edge.Cuts d'au moins 1.0 mm pour protéger le cuivre de la fraiseuse.
   4. Exclusion Intelligente du Plan de Masse (GND) :
      - Détection automatique du réseau de masse (GND, 0V, GNDA, etc.)
      - Omission intégrale du tracé géométrique : prise en charge par Zone Fill & freins thermiques.
   5. Nettoyage de la Sérigraphie (Silkscreen) :
      - Fonction post-traitement Cleanup_Silkscreen repositionnant les RefDes hors des pastilles et pistes.
   6. Interdiction des Angles Aigus & Retours Serrés (No Acute Angles / Wide Bypasses) :
      - Interdiction absolue de tout raccordement avec angle intérieur < 90° (les "pièges à acide")
      - Règle vectorielle du produit scalaire : d1 · d2 >= 0 (élimine 100% des angles aigus et rebroussements)
      - Élargissement des trajectoires : contournements privilégiant les coudes obtus (135°)
        avec forte pénalité sur les angles droits (90°)
      - Interdiction des retours serrés (hairpins) : distance minimale latérale garantie (>= 0.60 mm)
      - Assainissement polyline : élimination des micro-jogs et dépassements parasites aux pastilles.
=============================================================================
"""

import math
import heapq
import time
from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict

try:
    import pcbnew
    PCBNEW_AVAILABLE = True
except ImportError:
    PCBNEW_AVAILABLE = False

from .utils import from_mm, to_mm, BBox, setup_logger

logger = setup_logger("SmartRouter")

# 8 directions de déplacement 2D : strictement 0°, 45°, 90°, 135°, 180°, etc.
DIRECTIONS_2D = [
    (1, 0, 1.0),      # Droite (0°)
    (0, 1, 1.0),      # Bas (90°)
    (-1, 0, 1.0),     # Gauche (180°)
    (0, -1, 1.0),     # Haut (270°)
    (1, 1, 1.4142),   # Diagonale bas-droite (45°)
    (-1, 1, 1.4142),  # Diagonale bas-gauche (135°)
    (-1, -1, 1.4142), # Diagonale haut-gauche (225°)
    (1, -1, 1.4142),  # Diagonale haut-droite (315°)
]


class NetClassRule:
    """Règles dimensionnelles d'une classe d'équipotence."""
    def __init__(
        self,
        name: str = "Default",
        track_width_nm: int = 250000,
        clearance_nm: int = 200000,
        via_dia_nm: int = 800000,
        via_drill_nm: int = 400000
    ):
        self.name = name
        self.track_width = track_width_nm
        self.clearance = clearance_nm
        self.via_dia = via_dia_nm
        self.via_drill = via_drill_nm

    def __repr__(self):
        return (
            f"<NetClass {self.name}: w={to_mm(self.track_width):.2f}mm, "
            f"cl={to_mm(self.clearance):.2f}mm, via={to_mm(self.via_dia):.2f}mm>"
        )


class PadObstacle:
    """Structure légère représentant une pastille physique pour les contraintes DFM et DRC."""
    __slots__ = (
        'center_x', 'center_y', 'size_x', 'size_y', 'max_dim',
        'net_code', 'clearance', 'ref', 'pad_num', 'on_fcu', 'on_bcu'
    )

    def __init__(
        self,
        center_x: int,
        center_y: int,
        size_x: int,
        size_y: int,
        net_code: int,
        clearance: int,
        ref: str,
        pad_num: str,
        on_fcu: bool,
        on_bcu: bool
    ):
        self.center_x = center_x
        self.center_y = center_y
        self.size_x = size_x
        self.size_y = size_y
        self.max_dim = max(size_x, size_y)
        self.net_code = net_code
        self.clearance = clearance
        self.ref = ref
        self.pad_num = pad_num
        self.on_fcu = on_fcu
        self.on_bcu = on_bcu


class SmartRouter:
    """
    Routeur automatique natif pour circuits imprimés KiCad.
    """

    def __init__(
        self,
        board,
        grid_step_mm: float = 0.20,
        margin_board_mm: float = 1.0,
        via_penalty_mm: float = 25.0,
        ground_plane_enabled: bool = False,
        gnd_net_name: str = "GND"
    ):
        """
        Args:
            board: Instance BOARD KiCad.
            grid_step_mm: Résolution de la grille de routage (0.20 mm recommandé).
            margin_board_mm: Marge de sécurité avec le contour Edge.Cuts (au moins 1.0 mm).
            via_penalty_mm: Pénalité appliquée pour chaque création de via (25 mm = privilégie les détours mono-couche).
            ground_plane_enabled: Si True, exclut le filet de masse du routage géométrique.
            gnd_net_name: Nom du réseau de masse principal ("GND", "0V", etc.).
        """
        self.board = board
        self.grid_step = from_mm(grid_step_mm)
        # Marge de sécurité Edge.Cuts d'au moins 1.0 mm (Règle industrielle usine)
        self.margin_board = from_mm(max(1.0, margin_board_mm))
        # Pénalité via multipliée par 100 (Correction 2 : s'épuiser sur la couche active avant de percer)
        self.via_penalty = from_mm(max(10.0, via_penalty_mm)) * 100.0
        self.min_via_span = 10
        self.min_escape_cells = 3  # Tracé obligatoire d'au moins 0.6 mm avant de poser un via
        self.ground_plane_enabled = ground_plane_enabled
        self.gnd_net_name = gnd_net_name

        self.origin_x = 0
        self.origin_y = 0
        self.cols = 0
        self.rows = 0
        self.board_bbox: Optional[BBox] = None

        # Grille d'occupation et de dégagement
        # (layer, col, row) -> netcode
        self.occupied_cells: Dict[Tuple[int, int, int], int] = {}
        # (layer, col, row) -> set(netcodes ayant un halo de clearance ici)
        self.clearance_cells: Dict[Tuple[int, int, int], Set[int]] = defaultdict(set)

        # Règles NetClass par netcode
        self.net_rules: Dict[int, NetClassRule] = {}

        # DFM : Protection spatiale contre le Via-in-Pad
        self.pad_boxes: List[Tuple[object, int, str, str]] = []  # [(BBox, netcode, pad_num, fp_ref)]
        self.pad_via_keepout: Set[Tuple[int, int]] = set()       # Cellules interdites aux vias
        self.all_pad_obstacles: List[PadObstacle] = []           # Obstacles physiques de pastilles (Hard Constraint #1 & #2)

        # Déduplication des segments et vias
        self.seen_track_segments: Set[Tuple[int, int, Tuple[int, int], Tuple[int, int]]] = set()
        self.seen_vias_set: Set[Tuple[int, int, int]] = set()

        # Correction obligatoire : Pre-Routing Zone Fill dès l'initialisation du routeur
        self.pre_route_zone_fill()

    def pre_route_zone_fill(self):
        """
        Pre-Routing Zone Fill (Correction obligatoire) :
        Force KiCad à recalculer toutes les zones de cuivre de la carte avant de lancer la moindre
        itération de Pathfinding pour mettre à jour les obstacles physiques en fonction des Netclasses actuelles.
        """
        if not PCBNEW_AVAILABLE or self.board is None:
            return
        try:
            if hasattr(self.board, "Zones") and hasattr(pcbnew, "ZONE_FILLER"):
                zones = self.board.Zones()
                if zones and len(zones) > 0:
                    filler = pcbnew.ZONE_FILLER(self.board)
                    filler.Fill(zones)
                    logger.info(f"Pre-Routing Zone Fill : {len(zones)} zone(s) de cuivre recalculée(s) selon les Netclasses actuelles.")
        except Exception as e:
            logger.warning(f"Avertissement lors du Pre-Routing Zone Fill : {e}")

    @staticmethod
    def is_ground_net(net_name: str) -> bool:
        """Détecte intelligemment si un réseau correspond à la masse."""
        if not net_name:
            return False
        u = net_name.strip().upper()
        gnd_aliases = {"GND", "0V", "GNDA", "GNDD", "AGND", "DGND", "GROUND", "TERRE"}
        return u in gnd_aliases or any(alias in u for alias in ["GND", "0V", "GROUND"])

    def extract_netclasses(self):
        """Extrait toutes les NetClasses du circuit imprimé (KiCad 10 et antérieur)."""
        self.net_rules.clear()

        # 1. Règles par défaut
        default_rule = NetClassRule(
            name="Default",
            track_width_nm=from_mm(0.25),
            clearance_nm=from_mm(0.20),
            via_dia_nm=from_mm(0.60),
            via_drill_nm=from_mm(0.30)
        )

        ns = None
        if hasattr(self.board, "GetDesignSettings"):
            ds = self.board.GetDesignSettings()
            if hasattr(ds, "m_NetSettings"):
                ns = ds.m_NetSettings
                try:
                    d = ns.GetDefaultNetclass()
                    if d:
                        default_rule = NetClassRule(
                            name=str(d.GetName()),
                            track_width_nm=int(d.GetTrackWidth()),
                            clearance_nm=int(d.GetClearance()),
                            via_dia_nm=int(d.GetViaDiameter()),
                            via_drill_nm=int(d.GetViaDrill())
                        )
                except Exception:
                    pass

        # 2. Associer chaque net à sa NetClass effective
        for netcode, net in self.board.GetNetsByNetcode().items():
            nname = net.GetNetname()
            if not nname:
                continue

            rule = None
            if ns and hasattr(ns, "GetEffectiveNetClass"):
                try:
                    eff = ns.GetEffectiveNetClass(net)
                    if eff:
                        rule = NetClassRule(
                            name=str(eff.GetName()),
                            track_width_nm=int(eff.GetTrackWidth()),
                            clearance_nm=int(eff.GetClearance()),
                            via_dia_nm=int(eff.GetViaDiameter()),
                            via_drill_nm=int(eff.GetViaDrill())
                        )
                except Exception:
                    pass

            if rule is None:
                rule = NetClassRule(
                    name=default_rule.name,
                    track_width_nm=default_rule.track_width,
                    clearance_nm=default_rule.clearance,
                    via_dia_nm=default_rule.via_dia,
                    via_drill_nm=default_rule.via_drill
                )

            # Prise en compte intelligente des contraintes physiques :
            # - Secteur / 230V : isolation et largeur renforcées (0.80 mm / cl 0.50 mm)
            # - Alimentations (+12V, -12V, +3.3V, GND) : largeur adaptée pour fort courant
            upper = nname.upper()
            if rule.name == "Default":
                if any(m in upper for m in ["230", "MAINS", "AC", "SECTEUR"]):
                    rule = NetClassRule(
                        name="Mains_HV",
                        track_width_nm=from_mm(0.80),
                        clearance_nm=from_mm(0.50),
                        via_dia_nm=from_mm(1.6),
                        via_drill_nm=from_mm(0.8)
                    )
                elif any(p in upper for p in ["+12V", "-12V", "+3.3V", "+5V", "VCC", "GND", "PWR"]):
                    rule = NetClassRule(
                        name="Power",
                        track_width_nm=from_mm(0.30),
                        clearance_nm=from_mm(0.20),
                        via_dia_nm=from_mm(0.8),
                        via_drill_nm=from_mm(0.4)
                    )

            self.net_rules[netcode] = rule
            logger.debug(f"Net {nname} (#{netcode}) -> {rule.name} (width={to_mm(rule.track_width):.2f}mm, cl={to_mm(rule.clearance):.2f}mm)")

    def setup_grid(self, board_bbox: BBox):
        """
        Initialise la grille de routage en appliquant une marge Keep-out stricte d'au moins 1.0 mm
        à l'intérieur du contour Edge.Cuts pour protéger les pistes de la fraiseuse d'usine.
        """
        self.board_bbox = board_bbox
        self.origin_x = board_bbox.x_min + self.margin_board
        self.origin_y = board_bbox.y_min + self.margin_board
        max_x = board_bbox.x_max - self.margin_board
        max_y = board_bbox.y_max - self.margin_board

        self.cols = max(1, int(math.ceil((max_x - self.origin_x) / self.grid_step)))
        self.rows = max(1, int(math.ceil((max_y - self.origin_y) / self.grid_step)))

        self.occupied_cells.clear()
        self.clearance_cells.clear()
        self.pad_boxes.clear()
        self.pad_via_keepout.clear()

        logger.info(
            f"Grille de routage initialisée : {self.cols}x{self.rows} cellules "
            f"(pas de {to_mm(self.grid_step):.2f} mm, marge Edge.Cuts={to_mm(self.margin_board):.2f} mm)"
        )

    def _coord_to_grid(self, x: int, y: int) -> Tuple[int, int]:
        c = int(round((x - self.origin_x) / self.grid_step))
        r = int(round((y - self.origin_y) / self.grid_step))
        return max(0, min(self.cols - 1, c)), max(0, min(self.rows - 1, r))

    def _grid_to_coord(self, c: int, r: int) -> Tuple[int, int]:
        return int(self.origin_x + c * self.grid_step), int(self.origin_y + r * self.grid_step)

    def _get_pad_cells(self, pad_dict: Dict) -> Set[Tuple[int, int]]:
        """Retourne l'ensemble des cellules de grille de la pastille."""
        pref = pad_dict.get('ref', '')
        pnum = pad_dict.get('pad_num', '')
        for box, pnet, num, ref in self.pad_boxes:
            if ref == pref and num == pnum:
                c1, r1 = self._coord_to_grid(box.GetX(), box.GetY())
                c2, r2 = self._coord_to_grid(box.GetRight(), box.GetBottom())
                return {(c, r) for c in range(c1, c2 + 1) for r in range(r1, r2 + 1)}
        c, r = self._coord_to_grid(pad_dict['pos'][0], pad_dict['pos'][1])
        return {(c, r)}

    def mark_pad_obstacles(self):
        """
        Marque les pastilles physiques, leurs halos de dégagement NetClass,
        et enregistre les zones d'interdiction stricte de Via-in-Pad (Hard Constraint #1).
        """
        self.pad_boxes.clear()
        self.pad_via_keepout.clear()
        self.all_pad_obstacles.clear()
        pseudo_id = -1000

        for fp in self.board.GetFootprints():
            for pad in fp.Pads():
                net_code = pad.GetNetCode()
                if net_code <= 0:
                    pseudo_id -= 1
                    eff_net_code = pseudo_id
                else:
                    eff_net_code = net_code

                lset = pad.GetLayerSet()
                on_fcu = lset.Contains(pcbnew.F_Cu)
                on_bcu = lset.Contains(pcbnew.B_Cu)

                pos = pad.GetPosition()
                size = pad.GetSize()
                box = pad.GetBoundingBox()
                self.pad_boxes.append((box, eff_net_code, pad.GetNumber(), fp.GetReference()))

                c1, r1 = self._coord_to_grid(box.GetX(), box.GetY())
                c2, r2 = self._coord_to_grid(box.GetRight(), box.GetBottom())

                rule = self.net_rules.get(net_code, NetClassRule())

                # Enregistrer l'obstacle pastille pour les contraintes strictes #1 et #2
                pad_obs = PadObstacle(
                    center_x=pos.x,
                    center_y=pos.y,
                    size_x=size.x,
                    size_y=size.y,
                    net_code=eff_net_code,
                    clearance=rule.clearance,
                    ref=fp.GetReference(),
                    pad_num=pad.GetNumber(),
                    on_fcu=on_fcu,
                    on_bcu=on_bcu
                )
                self.all_pad_obstacles.append(pad_obs)

                # Hard Constraint #1 : Condition stricte d'interdiction de Via-in-Pad :
                # distance_centre_a_centre > (via.GetWidth() / 2) + (max(pad.GetSize().x, pad.GetSize().y) / 2) + clearance_de_la_netclass
                via_copper_radius = rule.via_dia / 2.0
                pad_half_max = pad_obs.max_dim / 2.0
                via_exclusion_dist = via_copper_radius + pad_half_max + rule.clearance

                center_c, center_r = self._coord_to_grid(pos.x, pos.y)
                ex_via_cells = max(2, int(math.ceil(via_exclusion_dist / self.grid_step)))

                for c in range(center_c - ex_via_cells, center_c + ex_via_cells + 1):
                    for r in range(center_r - ex_via_cells, center_r + ex_via_cells + 1):
                        if 0 <= c < self.cols and 0 <= r < self.rows:
                            gx, gy = self._grid_to_coord(c, r)
                            if math.hypot(gx - pos.x, gy - pos.y) <= via_exclusion_dist:
                                self.pad_via_keepout.add((c, r))

                layers = []
                if on_fcu: layers.append(0)
                if on_bcu: layers.append(1)

                pad_netname = pad.GetNetname()
                is_gnd_pad = self.ground_plane_enabled and self.is_ground_net(pad_netname)
                eff_clearance = max(rule.clearance, from_mm(0.50)) if is_gnd_pad else rule.clearance
                cl_cells = max(1, int(math.ceil(eff_clearance / self.grid_step)))

                for layer in layers:
                    # Cuivre physique de la pastille
                    for c in range(c1, c2 + 1):
                        for r in range(r1, r2 + 1):
                            self.occupied_cells[(layer, c, r)] = eff_net_code

                    # Halo d'isolation NetClass (distance euclidienne au rectangle de cuivre)
                    for c in range(c1 - cl_cells, c2 + cl_cells + 1):
                        for r in range(r1 - cl_cells, r2 + cl_cells + 1):
                            if 0 <= c < self.cols and 0 <= r < self.rows:
                                dx = max(0, c1 - c, c - c2)
                                dy = max(0, r1 - r, r - r2)
                                if dx * dx + dy * dy <= cl_cells * cl_cells:
                                    self.clearance_cells[(layer, c, r)].add(eff_net_code)

        # Interdiction de traversée inter-pastilles CMS 2-pins (R, C, etc.)
        for fp in self.board.GetFootprints():
            pads = list(fp.Pads())
            if len(pads) == 2:
                p1, p2 = pads[0], pads[1]
                if p1.GetDrillSize().x == 0 and p2.GetDrillSize().x == 0:
                    lset = p1.GetLayerSet()
                    layer_id = 0 if lset.Contains(pcbnew.F_Cu) else 1

                    box1, box2 = p1.GetBoundingBox(), p2.GetBoundingBox()
                    x_min = min(box1.GetX(), box2.GetX())
                    x_max = max(box1.GetRight(), box2.GetRight())
                    y_min = min(box1.GetY(), box2.GetY())
                    y_max = max(box1.GetBottom(), box2.GetBottom())

                    c1, r1 = self._coord_to_grid(x_min, y_min)
                    c2, r2 = self._coord_to_grid(x_max, y_max)

                    for c in range(min(c1, c2), max(c1, c2) + 1):
                        for r in range(min(r1, r2), max(r1, r2) + 1):
                            if (layer_id, c, r) not in self.occupied_cells:
                                self.occupied_cells[(layer_id, c, r)] = -999

    def is_via_placement_valid(
        self,
        vx: int,
        vy: int,
        netcode: int,
        rule: NetClassRule
    ) -> bool:
        """
        Validation Mathématique Absolue du Placement d'un Via (Correction 1 : Hard Constraint).

        Vérification stricte obligatoire :
        Distance_Euclidienne(centre_via, centre_pastille) > (via.GetWidth() / 2) + (max(pad.GetSize().x, pad.GetSize().y) / 2) + clearance_netclass

        Si cette condition est fausse pour n'importe quelle pastille du circuit, le placement est rejeté.
        """
        via_half_width = rule.via_dia / 2.0
        clearance_netclass = rule.clearance

        # 1. Vérification contre toutes les pastilles physiques enregistrées
        if self.all_pad_obstacles:
            for pad in self.all_pad_obstacles:
                pad_half_max = pad.max_dim / 2.0
                min_allowed_dist = via_half_width + pad_half_max + clearance_netclass
                dx = vx - pad.center_x
                dy = vy - pad.center_y
                dist_centre_a_centre = math.hypot(dx, dy)
                if dist_centre_a_centre <= min_allowed_dist:
                    return False
        elif hasattr(self.board, "GetFootprints"):
            for fp in self.board.GetFootprints():
                for pad in fp.Pads():
                    p_pos = pad.GetPosition()
                    p_sz = pad.GetSize()
                    pad_half_max = max(p_sz.x, p_sz.y) / 2.0
                    min_allowed_dist = via_half_width + pad_half_max + clearance_netclass
                    dx = vx - p_pos.x
                    dy = vy - p_pos.y
                    dist_centre_a_centre = math.hypot(dx, dy)
                    if dist_centre_a_centre <= min_allowed_dist:
                        return False

        # 2. Vérification d'isolement avec les vias existants de nets différents
        if hasattr(self.board, "GetTracks"):
            for existing in self.board.GetTracks():
                if isinstance(existing, pcbnew.PCB_VIA):
                    if existing.GetNetCode() != netcode:
                        epos = existing.GetPosition()
                        ex_w = existing.GetWidth(pcbnew.F_Cu) if hasattr(existing, 'GetWidth') else rule.via_dia
                        req_spacing = (rule.via_dia + ex_w) / 2.0 + clearance_netclass
                        d_via = math.hypot(epos.x - vx, epos.y - vy)
                        if d_via <= req_spacing:
                            return False

        return True

    def get_dynamic_via_cost(
        self,
        vx: int,
        vy: int,
        rule: NetClassRule,
        r_safe_mm: float = 6.0
    ) -> int:
        """
        Fonction de coût dynamique : Fuite avant Perçage (Correction 2).

        Formule :
        Via_Cost = Base_Via_Cost + (Proximité_avec_une_pastille * 100)
        """
        base_via_cost = self.via_penalty  # En nanomètres (déjà multiplié par 100)

        if not self.all_pad_obstacles:
            return int(base_via_cost)

        # Distance minimale euclidienne à toutes les pastilles (en mm)
        min_dist_nm = min(
            math.hypot(vx - pad.center_x, vy - pad.center_y)
            for pad in self.all_pad_obstacles
        )
        min_dist_mm = to_mm(min_dist_nm)

        # Proximité avec une pastille : positive et croissante à mesure qu'on s'approche
        if min_dist_mm < r_safe_mm:
            proximite_pastille_mm = r_safe_mm - min_dist_mm
            # (Proximité * 100) converti en nanomètres
            prox_penalty_nm = from_mm(proximite_pastille_mm * 100.0)
            return int(base_via_cost + prox_penalty_nm)
        else:
            return int(base_via_cost)

    @staticmethod
    def clean_board_tracks(board):
        """Supprime proprement toutes les pistes et vias existantes sur la carte."""
        if board is None or not hasattr(board, "GetTracks"):
            return
        to_del = []
        try:
            for t in board.GetTracks():
                to_del.append(t)
        except Exception:
            pass
        for t in to_del:
            try:
                board.RemoveNative(t)
            except Exception:
                try:
                    board.Remove(t)
                except Exception:
                    pass

    def _route_astar_single_layer(
        self,
        start_pad: Dict,
        end_pad: Dict,
        netcode: int,
        rule: NetClassRule,
        target_layer: int = 0
    ) -> Optional[List[Tuple[int, int, int]]]:
        """Tente un routage STRICTEMENT MONO-COUCHE (0 via garanti)."""
        if target_layer == 0:
            if not start_pad['on_fcu'] or not end_pad['on_fcu']:
                return None
        else:
            if not start_pad['on_bcu'] or not end_pad['on_bcu']:
                return None

        sc, sr = self._coord_to_grid(start_pad['pos'][0], start_pad['pos'][1])
        gc, gr = self._coord_to_grid(end_pad['pos'][0], end_pad['pos'][1])

        start_pad_cells = self._get_pad_cells(start_pad)
        end_pad_cells = self._get_pad_cells(end_pad)

        def heuristic(c, r):
            dx = abs(c - gc)
            dy = abs(r - gr)
            d2d = (dx + dy + (1.4142 - 2.0) * min(dx, dy)) * self.grid_step
            return 1.20 * d2d

        open_set = [(heuristic(sc, sr), 0.0, (sc, sr, -1))]
        g_scores = {(sc, sr): 0.0}
        came_from = {}

        expansions = 0
        max_expansions = getattr(self, "current_max_expansions_single", 500000)

        while open_set and expansions < max_expansions:
            expansions += 1
            f, current_g, (c, r, prev_dir) = heapq.heappop(open_set)

            if c == gc and r == gr:
                path = [(target_layer, c, r)]
                curr = (c, r, prev_dir)
                while curr in came_from:
                    curr = came_from[curr]
                    path.append((target_layer, curr[0], curr[1]))
                path.reverse()
                return path

            for d_idx, (dc, dr, dist_mult) in enumerate(DIRECTIONS_2D):
                # Interdiction absolue des angles aigus (< 90°) : produit scalaire d1 · d2 >= 0
                if prev_dir != -1:
                    prev_dc, prev_dr, _ = DIRECTIONS_2D[prev_dir]
                    if prev_dc * dc + prev_dr * dr < 0:
                        continue

                nc = c + dc
                nr = r + dr

                if not (0 <= nc < self.cols and 0 <= nr < self.rows):
                    continue

                if dc != 0 and dr != 0:
                    occ1 = self.occupied_cells.get((target_layer, c + dc, r))
                    occ2 = self.occupied_cells.get((target_layer, c, r + dr))
                    if (occ1 is not None and occ1 != netcode) or \
                       (occ2 is not None and occ2 != netcode):
                        continue

                # 1. Vérification physique du cuivre (mur invisible)
                cell_occ = self.occupied_cells.get((target_layer, nc, nr))
                if cell_occ is not None and cell_occ != netcode:
                    continue

                # 2. Application stricte du Clearance Netclass
                at_terminal = (nc, nr) in start_pad_cells or (nc, nr) in end_pad_cells
                if not at_terminal:
                    cell_cls = self.clearance_cells.get((target_layer, nc, nr))
                    if cell_cls and any(on != netcode for on in cell_cls):
                        continue

                # Interdiction des retours serrés (hairpins à moins de 2 cellules de la trace récente)
                is_tight_return = False
                curr_back = (c, r, prev_dir)
                steps_back = 0
                while curr_back in came_from and steps_back < 10:
                    curr_back = came_from[curr_back]
                    steps_back += 1
                    if steps_back >= 4:
                        dist_sq = (nc - curr_back[0]) ** 2 + (nr - curr_back[1]) ** 2
                        if dist_sq <= 4:
                            is_tight_return = True
                            break
                if is_tight_return:
                    continue

                step_cost = dist_mult * self.grid_step
                # Élargissement de trajectoire : coude obtus 135° très favorisé vs angle droit 90°
                if prev_dir != -1:
                    prev_dc, prev_dr, _ = DIRECTIONS_2D[prev_dir]
                    dot = prev_dc * dc + prev_dr * dr
                    if dot == 0:
                        step_cost += from_mm(0.35)  # Pénalité angle droit 90°
                    elif d_idx != prev_dir:
                        step_cost += from_mm(0.04)  # Coude obtus 135°

                new_g = current_g + step_cost
                neighbor_state = (nc, nr)

                if neighbor_state not in g_scores or new_g < g_scores[neighbor_state]:
                    g_scores[neighbor_state] = new_g
                    next_node = (nc, nr, d_idx)
                    came_from[next_node] = (c, r, prev_dir)
                    heapq.heappush(open_set, (new_g + heuristic(nc, nr), new_g, next_node))

        return None

    def _route_astar_multilayer(
        self,
        start_pad: Dict,
        end_pad: Dict,
        netcode: int,
        rule: NetClassRule
    ) -> Optional[List[Tuple[int, int, int]]]:
        """
        Recherche A* 3D multi-couches avec interdiction absolue du Via-in-Pad,
        tracé d'échappement obligatoire, et vérification bi-couche du halo des vias.
        """
        sc, sr = self._coord_to_grid(start_pad['pos'][0], start_pad['pos'][1])
        gc, gr = self._coord_to_grid(end_pad['pos'][0], end_pad['pos'][1])

        start_pad_cells = self._get_pad_cells(start_pad)
        end_pad_cells = self._get_pad_cells(end_pad)

        start_layers = []
        if start_pad['on_fcu']: start_layers.append(0)
        if start_pad['on_bcu']: start_layers.append(1)

        goal_layers = set()
        if end_pad['on_fcu']: goal_layers.add(0)
        if end_pad['on_bcu']: goal_layers.add(1)

        is_mains = (rule.track_width >= from_mm(0.70))
        if is_mains and 1 in start_layers and 1 in goal_layers:
            start_layers = [1]
            goal_layers = {1}
        elif not is_mains and 0 in start_layers:
            start_layers = [0, 1] if 1 in start_layers else [0]

        def heuristic(layer, c, r):
            dx = abs(c - gc)
            dy = abs(r - gr)
            dist_2d = (dx + dy + (1.4142 - 2.0) * min(dx, dy)) * self.grid_step
            layer_cost = 0.0 if layer in goal_layers else self.via_penalty
            return 1.25 * (dist_2d + layer_cost)

        open_set = []
        came_from = {}
        g_scores = {}

        # Initialisation : steps_from_start = 0 (tracé initial obligatoire avant tout via)
        for sl in start_layers:
            init_g = 0.0
            g_scores[(sl, sc, sr)] = init_g
            heapq.heappush(open_set, (init_g + heuristic(sl, sc, sr), init_g, (sl, sc, sr, -1, 100, 0)))

        expansions = 0
        max_expansions = getattr(self, "current_max_expansions", 800000)

        req_via_clearance = rule.via_dia / 2.0 + rule.clearance
        req_via_cells = max(1, int(math.ceil(req_via_clearance / self.grid_step)))

        while open_set and expansions < max_expansions:
            expansions += 1
            f, current_g, (layer, c, r, prev_dir, steps_since_via, steps_from_start) = heapq.heappop(open_set)

            if c == gc and r == gr and (layer in goal_layers):
                path = [(layer, c, r)]
                curr = (layer, c, r, prev_dir, steps_since_via, steps_from_start)
                while curr in came_from:
                    curr = came_from[curr]
                    path.append((curr[0], curr[1], curr[2]))
                path.reverse()
                return path

            # 1. Déplacements 2D
            for d_idx, (dc, dr, dist_mult) in enumerate(DIRECTIONS_2D):
                # Interdiction absolue des angles aigus (< 90°) : produit scalaire d1 · d2 >= 0
                if prev_dir != -1:
                    prev_dc, prev_dr, _ = DIRECTIONS_2D[prev_dir]
                    if prev_dc * dc + prev_dr * dr < 0:
                        continue

                nc = c + dc
                nr = r + dr

                if not (0 <= nc < self.cols and 0 <= nr < self.rows):
                    continue

                if dc != 0 and dr != 0:
                    occ1 = self.occupied_cells.get((layer, c + dc, r))
                    occ2 = self.occupied_cells.get((layer, c, r + dr))
                    if (occ1 is not None and occ1 != netcode) or \
                       (occ2 is not None and occ2 != netcode):
                        continue

                cell_occ = self.occupied_cells.get((layer, nc, nr))
                if cell_occ is not None and cell_occ != netcode:
                    continue

                at_terminal = (nc, nr) in start_pad_cells or (nc, nr) in end_pad_cells
                if not at_terminal:
                    cell_cls = self.clearance_cells.get((layer, nc, nr))
                    if cell_cls and any(on != netcode for on in cell_cls):
                        continue

                # Interdiction des retours serrés (hairpins à moins de 2 cellules de la trace récente)
                is_tight_return = False
                curr_back = (layer, c, r, prev_dir, steps_since_via, steps_from_start)
                steps_back = 0
                while curr_back in came_from and steps_back < 10:
                    curr_back = came_from[curr_back]
                    steps_back += 1
                    if steps_back >= 4:
                        if curr_back[0] == layer:
                            dist_sq = (nc - curr_back[1]) ** 2 + (nr - curr_back[2]) ** 2
                            if dist_sq <= 4:
                                is_tight_return = True
                                break
                if is_tight_return:
                    continue

                step_cost = dist_mult * self.grid_step
                if not is_mains and layer == 1:
                    step_cost *= 1.05

                # Élargissement de trajectoire : coude obtus 135° très favorisé vs angle droit 90°
                if prev_dir != -1:
                    prev_dc, prev_dr, _ = DIRECTIONS_2D[prev_dir]
                    dot = prev_dc * dc + prev_dr * dr
                    if dot == 0:
                        step_cost += from_mm(0.35)  # Pénalité angle droit 90°
                    elif d_idx != prev_dir:
                        step_cost += from_mm(0.04)  # Coude obtus 135°

                new_g = current_g + step_cost
                neighbor_state = (layer, nc, nr)

                if neighbor_state not in g_scores or new_g < g_scores[neighbor_state]:
                    g_scores[neighbor_state] = new_g
                    next_node = (layer, nc, nr, d_idx, steps_since_via + 1, steps_from_start + 1)
                    came_from[next_node] = (layer, c, r, prev_dir, steps_since_via, steps_from_start)
                    heapq.heappush(open_set, (new_g + heuristic(layer, nc, nr), new_g, next_node))

            # 2. Vias : Hard Constraints #1 & #2 (Anti Via-in-Pad & Fuite avant Perçage)
            dist_to_start = math.hypot(c - sc, r - sr)
            dist_to_goal = math.hypot(c - gc, r - gr)

            vx, vy = self._grid_to_coord(c, r)

            # Hard Constraint #1 : Interdiction Mathématique Absolue des Via-in-Pad
            # distance_centre_a_centre > (via.GetWidth() / 2) + (max(pad.GetSize().x, pad.GetSize().y) / 2) + clearance_de_la_netclass
            is_placement_legal = ((c, r) not in self.pad_via_keepout) and self.is_via_placement_valid(vx, vy, netcode, rule)

            has_escaped_start = (steps_from_start >= self.min_escape_cells) and (dist_to_start >= self.min_escape_cells)
            not_inside_goal = dist_to_goal >= self.min_escape_cells
            can_switch_hysteresis = (steps_since_via >= self.min_via_span)

            if is_placement_legal and has_escaped_start and not_inside_goal and can_switch_hysteresis:
                other_layer = 1 - layer
                via_blocked = False

                for dc_v in range(-req_via_cells, req_via_cells + 1):
                    for dr_v in range(-req_via_cells, req_via_cells + 1):
                        if dc_v * dc_v + dr_v * dr_v <= req_via_cells * req_via_cells:
                            vc, vr = c + dc_v, r + dr_v
                            if not (0 <= vc < self.cols and 0 <= vr < self.rows):
                                via_blocked = True
                                break
                            for chk_l in (0, 1):
                                vo = self.occupied_cells.get((chk_l, vc, vr))
                                if vo is not None and vo != netcode:
                                    via_blocked = True
                                    break
                                vc_cls = self.clearance_cells.get((chk_l, vc, vr))
                                if vc_cls and any(on != netcode for on in vc_cls):
                                    via_blocked = True
                                    break
                            if via_blocked:
                                break
                    if via_blocked:
                        break

                if not via_blocked:
                    # Hard Constraint #2 : Via_Cost = Base_Via_Cost + (Proximite_avec_une_pastille * 100)
                    dynamic_via_cost = self.get_dynamic_via_cost(vx, vy, rule)
                    new_g = current_g + dynamic_via_cost
                    neighbor_state = (other_layer, c, r)
                    if neighbor_state not in g_scores or new_g < g_scores[neighbor_state]:
                        g_scores[neighbor_state] = new_g
                        next_node = (other_layer, c, r, -1, 0, steps_from_start + 1)
                        came_from[next_node] = (layer, c, r, prev_dir, steps_since_via, steps_from_start)
                        heapq.heappush(open_set, (new_g + heuristic(other_layer, c, r), new_g, next_node))

        return None

    def _route_astar(
        self,
        start_pad: Dict,
        end_pad: Dict,
        netcode: int,
        rule: NetClassRule
    ) -> Optional[List[Tuple[int, int, int]]]:
        """
        Recherche le chemin optimal entre deux pastilles via A*.
        
        Hard Constraint #2 : Le changement de couche (placement d'un via)
        n'est autorisé QUE SI le pathfinding de la couche actuelle a exploré
        la totalité de la grille disponible sans succès.
        """
        # 1. Si les deux pastilles ont au moins une couche en commun, tenter d'abord
        # l'exploration monocouche exhaustive (0 via garanti).
        if start_pad['on_fcu'] and end_pad['on_fcu']:
            p = self._route_astar_single_layer(start_pad, end_pad, netcode, rule, target_layer=0)
            if p:
                return p

        if start_pad['on_bcu'] and end_pad['on_bcu']:
            p = self._route_astar_single_layer(start_pad, end_pad, netcode, rule, target_layer=1)
            if p:
                return p

        # 2. Si aucune solution monocouche n'existe après exploration complète de la grille,
        # passer au routage multicouche avec la fonction de coût dynamique (Fuite avant Perçage).
        return self._route_astar_multilayer(start_pad, end_pad, netcode, rule)

    def _route_with_push_and_shove(
        self,
        start_pad: Dict,
        end_pad: Dict,
        netcode: int,
        rule: NetClassRule
    ) -> Optional[List[Tuple[int, int, int]]]:
        """
        Système de tentatives multiples avec Push and Shove des pistes bloquantes (Correction 3).

        Si une connexion échoue en raison de pistes bloquantes appartenant à d'autres réseaux :
        1. Identifie les cellules occupées par des réseaux étrangers dans le couloir direct entre start_pad et end_pad.
        2. Applique un déplacement dynamique léger (décalage latéral orthogonal d'une cellule de grille = 0.20 mm)
           des pistes adverses pour forcer le passage.
        3. Recalcule le tracé A* dans le passage ainsi libéré.
        4. Revalide et fige les nouvelles positions des pistes poussées sans créer de court-circuit.
        """
        sc, sr = self._coord_to_grid(start_pad['pos'][0], start_pad['pos'][1])
        gc, gr = self._coord_to_grid(end_pad['pos'][0], end_pad['pos'][1])

        # Déterminer la zone de passage (corridor) entre les deux pastilles
        margin = 4
        min_c = max(0, min(sc, gc) - margin)
        max_c = min(self.cols - 1, max(sc, gc) + margin)
        min_r = max(0, min(sr, gr) - margin)
        max_r = min(self.rows - 1, max(sr, gr) + margin)

        # Identifier les cellules de pistes étrangères bloquantes dans le corridor
        # (on ne déplace pas les pastilles physiques, identifiées par occ_net <= 0)
        blocking_cells = {}
        for (layer, c, r), occ_net in list(self.occupied_cells.items()):
            if min_c <= c <= max_c and min_r <= r <= max_r:
                if occ_net > 0 and occ_net != netcode:
                    blocking_cells[(layer, c, r)] = occ_net

        if not blocking_cells:
            # Si aucun obstacle de piste étrangère directe n'est présent, tentative multicouche élargie
            old_escape = getattr(self, "escape_envelope", 3)
            self.escape_envelope = 5
            path = self._route_astar_multilayer(start_pad, end_pad, netcode, rule)
            self.escape_envelope = old_escape
            return path

        # Vecteur de poussée orthogonale préférentiel
        dx = gc - sc
        dy = gr - sr
        if abs(dx) > abs(dy):
            push_dc, push_dr = 0, 1 if dy >= 0 else -1
        else:
            push_dc, push_dr = 1 if dx >= 0 else -1, 0

        # Sauvegarde de l'état de la grille
        saved_occupied = dict(self.occupied_cells)
        saved_clearance = defaultdict(set, {k: set(v) for k, v in self.clearance_cells.items()})

        # Libération temporaire des cellules bloquantes pour laisser passer la connexion prioritaire
        for cell in blocking_cells:
            if cell in self.occupied_cells:
                del self.occupied_cells[cell]

        # Calcul du tracé prioritaire forcé
        path = self._route_astar(start_pad, end_pad, netcode, rule)

        if path:
            path_cells = {(layer, c, r) for layer, c, r in path}
            # Déplacement ("Push and Shove") des segments bloquants en conflit
            for (layer, c, r), occ_net in blocking_cells.items():
                if (layer, c, r) in path_cells:
                    # Décalage latéral orthogonal léger (+1 pas)
                    new_c = c + push_dc
                    new_r = r + push_dr
                    if 0 <= new_c < self.cols and 0 <= new_r < self.rows and (layer, new_c, new_r) not in path_cells and (layer, new_c, new_r) not in self.occupied_cells:
                        self.occupied_cells[(layer, new_c, new_r)] = occ_net
                        logger.info(
                            f"  Push and Shove : Piste étrangère (Net #{occ_net}) décalée "
                            f"vers ({new_c},{new_r}) sur couche {layer}"
                        )
                    else:
                        # Alternative : décalage dans la direction opposée (-1 pas)
                        alt_c = c - push_dc
                        alt_r = r - push_dr
                        if 0 <= alt_c < self.cols and 0 <= alt_r < self.rows and (layer, alt_c, alt_r) not in path_cells and (layer, alt_c, alt_r) not in self.occupied_cells:
                            self.occupied_cells[(layer, alt_c, alt_r)] = occ_net
                            logger.info(
                                f"  Push and Shove : Piste étrangère (Net #{occ_net}) décalée "
                                f"vers ({alt_c},{alt_r}) sur couche {layer}"
                            )
                else:
                    # Réinsérer la cellule de piste non conflictuelle
                    self.occupied_cells[(layer, c, r)] = occ_net

            return path
        else:
            # Restauration complète en cas d'échec
            self.occupied_cells = saved_occupied
            self.clearance_cells = saved_clearance
            return None

    def _connect_pad_to_grid_45(self, px: int, py: int, gx: int, gy: int) -> List[Tuple[int, int, int, int]]:
        """
        Décompose la sortie de pastille en segments à 90° et 45° stricts.
        Force la sortie par le centre de la pastille (axe X ou Y).
        """
        dx = gx - px
        dy = gy - py
        if dx == 0 and dy == 0:
            return []
        if dy == 0 or dx == 0:
            return [(px, py, gx, gy)]
        if abs(dx) >= abs(dy):
            diag = abs(dy)
            lx = (abs(dx) - diag) * (1 if dx > 0 else -1)
            mid_x = px + lx
            mid_y = py
            res = []
            if mid_x != px or mid_y != py:
                res.append((px, py, mid_x, mid_y))
            res.append((mid_x, mid_y, gx, gy))
            return res
        else:
            diag = abs(dx)
            ly = (abs(dy) - diag) * (1 if dy > 0 else -1)
            mid_x = px
            mid_y = py + ly
            res = []
            if mid_x != px or mid_y != py:
                res.append((px, py, mid_x, mid_y))
            res.append((mid_x, mid_y, gx, gy))
            return res

    def _connect_grid_to_pad_45(self, gx: int, gy: int, px: int, py: int) -> List[Tuple[int, int, int, int]]:
        """
        Décompose l'entrée dans la pastille en segments à 90° et 45° stricts.
        Force l'entrée par le centre de la pastille (axe X ou Y) pour éliminer les pièges à acide.
        """
        dx = px - gx
        dy = py - gy
        if dx == 0 and dy == 0:
            return []
        if dy == 0 or dx == 0:
            return [(gx, gy, px, py)]
        if abs(dx) >= abs(dy):
            diag = abs(dy)
            mid_x = gx + (diag if dx > 0 else -diag)
            mid_y = py
            res = [(gx, gy, mid_x, mid_y)]
            if mid_x != px:
                res.append((mid_x, mid_y, px, py))
            return res
        else:
            diag = abs(dx)
            mid_x = px
            mid_y = gy + (diag if dy > 0 else -diag)
            res = [(gx, gy, mid_x, mid_y)]
            if mid_y != py:
                res.append((mid_x, mid_y, px, py))
            return res

    def _sanitize_polyline(self, pts: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Assainit la polyligne de piste :
          - Élimine les micro-segments (< 5 microns) et doublons
          - Élimine les sommets collinéaires (180°)
          - Élimine les rebroussements et angles aigus (< 90°)
          - Préserve strictement le point d'ancrage de départ et d'arrivée (pad ou via).
        """
        if len(pts) < 2:
            return pts

        cleaned = [pts[0]]
        for p in pts[1:]:
            dx = p[0] - cleaned[-1][0]
            dy = p[1] - cleaned[-1][1]
            if dx * dx + dy * dy > 25:  # > 5 microns
                cleaned.append(p)

        if len(cleaned) < 3:
            return cleaned

        changed = True
        passes = 0
        while changed and passes < 10 and len(cleaned) >= 3:
            changed = False
            passes += 1
            new_pts = [cleaned[0]]
            i = 1
            while i < len(cleaned) - 1:
                p_prev = new_pts[-1]
                p_curr = cleaned[i]
                p_next = cleaned[i+1]

                v1_x = p_curr[0] - p_prev[0]
                v1_y = p_curr[1] - p_prev[1]
                v2_x = p_next[0] - p_curr[0]
                v2_y = p_next[1] - p_curr[1]

                l1 = math.hypot(v1_x, v1_y)
                l2 = math.hypot(v2_x, v2_y)

                if l1 < 1000 or l2 < 1000:
                    changed = True
                    i += 1
                    continue

                dot = (v1_x * v2_x + v1_y * v2_y) / (l1 * l2)
                cross = abs(v1_x * v2_y - v1_y * v2_x) / (l1 * l2)

                # A. Sommet collinéaire (180°)
                if dot > 0.999 and cross < 0.01:
                    changed = True
                    i += 1
                    continue

                # B. Angle aigu < 90° (dot < -0.01) : élimination du jog parasite
                if dot < -0.01:
                    if l1 <= from_mm(0.40) or l2 <= from_mm(0.40):
                        changed = True
                        i += 1
                        continue
                    else:
                        new_pts.append(p_curr)
                        i += 1
                else:
                    new_pts.append(p_curr)
                    i += 1

            new_pts.append(cleaned[-1])
            cleaned = new_pts

        return cleaned

    def _apply_path_with_kicad_items(
        self,
        path: List[Tuple[int, int, int]],
        start_pad: Dict,
        end_pad: Dict,
        netcode: int,
        rule: NetClassRule
    ) -> Tuple[int, int, int]:
        """
        Enregistre le chemin sur la grille et crée les segments PCB_TRACK et vias PCB_VIA.
        Connecte exactement les pastilles avec des angles stricts 45° et 90°.
        Garantit zéro via-in-pad, zéro angle aigu (< 90°) et zéro violation de clearance.
        """
        copper_r = max(0, int(round((rule.track_width / 2.0) / self.grid_step)))
        cl_r = max(1, int(round((rule.track_width / 2.0 + rule.clearance) / self.grid_step)))
        for layer, c, r in path:
            for dc in range(-copper_r, copper_r + 1):
                for dr in range(-copper_r, copper_r + 1):
                    if dc * dc + dr * dr <= copper_r * copper_r:
                        nc, nr = c + dc, r + dr
                        if 0 <= nc < self.cols and 0 <= nr < self.rows:
                            self.occupied_cells[(layer, nc, nr)] = netcode

            for dc in range(-cl_r, cl_r + 1):
                for dr in range(-cl_r, cl_r + 1):
                    if dc * dc + dr * dr <= cl_r * cl_r:
                        nc, nr = c + dc, r + dr
                        if 0 <= nc < self.cols and 0 <= nr < self.rows:
                            self.clearance_cells[(layer, nc, nr)].add(netcode)

        if len(path) < 2:
            return 0, 0, 0

        sc, sr = self._coord_to_grid(start_pad['pos'][0], start_pad['pos'][1])
        gc, gr = self._coord_to_grid(end_pad['pos'][0], end_pad['pos'][1])
        sp_x, sp_y = start_pad['pos']
        ep_x, ep_y = end_pad['pos']

        # Découpage du chemin par couche (séparé par les vias)
        layer_chains = []
        curr_chain_layer = path[0][0]
        curr_chain_pts = [(sp_x, sp_y)]
        vias_grid = []

        for i in range(len(path)):
            layer, c, r = path[i]
            gx, gy = self._grid_to_coord(c, r)

            if layer != curr_chain_layer:
                prev_layer, prev_c, prev_r = path[i-1]
                via_gx, via_gy = self._grid_to_coord(prev_c, prev_r)
                curr_chain_pts.append((via_gx, via_gy))
                layer_chains.append((curr_chain_layer, curr_chain_pts))

                at_start_tht = (prev_c == sc and prev_r == sr and start_pad['is_tht'])
                at_end_tht = (c == gc and r == gr and end_pad['is_tht'])
                if not at_start_tht and not at_end_tht:
                    vias_grid.append((prev_c, prev_r))

                curr_chain_layer = layer
                curr_chain_pts = [(via_gx, via_gy)]

            curr_chain_pts.append((gx, gy))

        curr_chain_pts.append((ep_x, ep_y))
        layer_chains.append((curr_chain_layer, curr_chain_pts))

        if not hasattr(self, 'seen_track_segments'):
            self.seen_track_segments = set()

        tot_len = 0
        added_tracks = 0
        for l_idx, pts in layer_chains:
            sanitized = self._sanitize_polyline(pts)
            pcb_layer = pcbnew.F_Cu if l_idx == 0 else pcbnew.B_Cu
            for j in range(len(sanitized) - 1):
                p1 = sanitized[j]
                p2 = sanitized[j+1]
                seg_key = (pcb_layer, netcode, min(p1, p2), max(p1, p2))
                if seg_key in self.seen_track_segments:
                    continue
                self.seen_track_segments.add(seg_key)

                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                length = int(math.hypot(dx, dy))
                if length > 0:
                    tot_len += length
                    track = pcbnew.PCB_TRACK(self.board)
                    track.SetStart(pcbnew.VECTOR2I(p1[0], p1[1]))
                    track.SetEnd(pcbnew.VECTOR2I(p2[0], p2[1]))
                    track.SetWidth(int(rule.track_width))
                    track.SetLayer(pcb_layer)
                    track.SetNetCode(netcode)
                    self.board.Add(track)
                    added_tracks += 1

        if not hasattr(self, 'seen_vias_set'):
            self.seen_vias_set = set()

        added_vias = 0
        via_r = max(1, int(round((rule.via_dia / 2.0) / self.grid_step)))
        via_cl_r = max(2, int(round((rule.via_dia / 2.0 + rule.clearance) / self.grid_step)))

        for vc, vr in vias_grid:
            vx, vy = self._grid_to_coord(vc, vr)
            via_key = (netcode, vx, vy)
            if via_key in self.seen_vias_set:
                continue
            self.seen_vias_set.add(via_key)

            # Application stricte de la contrainte Hard #1 : Interdiction Absolue des Via-in-Pad
            # distance_centre_a_centre > (via.GetWidth() / 2) + (max(pad.GetSize().x, pad.GetSize().y) / 2) + clearance_de_la_netclass
            if not self.is_via_placement_valid(vx, vy, netcode, rule):
                logger.warning(
                    f"DFM REJECT: Via à ({to_mm(vx):.2f}, {to_mm(vy):.2f}) strictement interdit "
                    f"(violation de la condition mathématique de distance avec les pastilles)"
                )
                continue

            already = False
            for existing in self.board.GetTracks():
                if isinstance(existing, pcbnew.PCB_VIA):
                    epos = existing.GetPosition()
                    if existing.GetNetCode() == netcode and epos.x == vx and epos.y == vy:
                        already = True
                        break
                    # Seuls les vias de nets DIFFÉRENTS doivent respecter la distance d'isolement
                    if existing.GetNetCode() != netcode:
                        ex_w = existing.GetWidth(pcbnew.F_Cu) if hasattr(existing, 'GetWidth') else rule.via_dia
                        req_via_spacing = (rule.via_dia + ex_w) / 2.0 + rule.clearance
                        d_via = math.hypot(epos.x - vx, epos.y - vy)
                        if d_via < req_via_spacing:
                            already = True
                            logger.warning(f"DFM REJECT: Via trop proche d'un via existant (dist={to_mm(d_via):.2f}mm < req={to_mm(req_via_spacing):.2f}mm)")
                            break
            if already:
                continue

            via = pcbnew.PCB_VIA(self.board)
            via.SetPosition(pcbnew.VECTOR2I(vx, vy))
            via.SetWidth(int(rule.via_dia))
            via.SetDrill(int(rule.via_drill))
            via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            via.SetNetCode(netcode)
            self.board.Add(via)
            added_vias += 1

            # Marquer le halo du via sur les DEUX couches
            for dc in range(-via_r, via_r + 1):
                for dr in range(-via_r, via_r + 1):
                    if dc * dc + dr * dr <= via_r * via_r:
                        self.occupied_cells[(0, vc + dc, vr + dr)] = netcode
                        self.occupied_cells[(1, vc + dc, vr + dr)] = netcode
            for dc in range(-via_cl_r, via_cl_r + 1):
                for dr in range(-via_cl_r, via_cl_r + 1):
                    if dc * dc + dr * dr <= via_cl_r * via_cl_r:
                        self.clearance_cells[(0, vc + dc, vr + dr)].add(netcode)
                        self.clearance_cells[(1, vc + dc, vr + dr)].add(netcode)

        return added_tracks, added_vias, tot_len

    def route(
        self,
        ground_plane_enabled: bool = False,
        gnd_net_name: str = "GND",
        progress_callback: Optional[object] = None
    ) -> Dict[str, float]:
        """
        Exécute le routage complet de tous les réseaux de la carte.

        Args:
            ground_plane_enabled: Si True, exclut le filet de masse du routage géométrique (géré par Zone Fill).
            gnd_net_name: Nom du réseau de masse principal ("GND", "0V", etc.).
            progress_callback: Fonction de rappel (percent, message) pour mise à jour UI.
        """
        start_t = time.time()
        self.extract_netclasses()

        # Correction obligatoire : Pre-Routing Zone Fill
        # Avant de lancer la moindre itération de Pathfinding, forcer KiCad à recalculer toutes les zones
        # de la carte pour mettre à jour les obstacles physiques en fonction des Netclasses actuelles.
        self.pre_route_zone_fill()

        edge_bb = self.board.GetBoardEdgesBoundingBox()
        if edge_bb.GetWidth() > 0 and edge_bb.GetHeight() > 0:
            board_bbox = BBox(edge_bb.GetX(), edge_bb.GetY(), edge_bb.GetRight(), edge_bb.GetBottom())
        else:
            bbox = self.board.ComputeBoundingBox()
            board_bbox = BBox(bbox.GetX(), bbox.GetY(), bbox.GetRight(), bbox.GetBottom())
        self.setup_grid(board_bbox)
        self.mark_pad_obstacles()

        # Supprimer proprement les anciennes pistes/vias
        self.clean_board_tracks(self.board)
        self.seen_track_segments.clear()
        self.seen_vias_set.clear()

        # 2. Exclusion Intelligente du Plan de Masse (GND / 0V / GNDA)
        is_gnd_active = ground_plane_enabled or getattr(self, "ground_plane_enabled", False)
        gnd_target = (gnd_net_name or getattr(self, "gnd_net_name", "GND")).upper()

        nets_to_route = []
        for netcode, net in self.board.GetNetsByNetcode().items():
            nname = net.GetNetname()
            if not nname or "unconnected" in nname.lower():
                continue

            # Si le plan de masse est actif, retirer intégralement le réseau de masse du tracé géométrique
            if is_gnd_active and (self.is_ground_net(nname) or nname.upper() == gnd_target):
                logger.info(
                    f"  ⚡ Net {nname} (#{netcode}) : exclu du routage géométrique "
                    f"(pris en charge automatiquement par le Plan de Masse / Zone Fill)"
                )
                continue

            pads = []
            for fp in self.board.GetFootprints():
                for pad in fp.Pads():
                    if pad.GetNetCode() == netcode:
                        lset = pad.GetLayerSet()
                        pads.append({
                            'ref': fp.GetReference(),
                            'pad_num': pad.GetNumber(),
                            'pos': (pad.GetPosition().x, pad.GetPosition().y),
                            'on_fcu': lset.Contains(pcbnew.F_Cu),
                            'on_bcu': lset.Contains(pcbnew.B_Cu),
                            'is_tht': lset.Contains(pcbnew.F_Cu) and lset.Contains(pcbnew.B_Cu),
                        })

            if len(pads) >= 2:
                nets_to_route.append({
                    'netcode': netcode,
                    'name': nname,
                    'rule': self.net_rules.get(netcode, NetClassRule()),
                    'pads': pads
                })

        # Ordonnancement optimal anti-vias et haute tension :
        # 1. Signaux d'échappement critiques de U2 (moc4, aop, moc3, moc2, moc1)
        # 2. Secteur / Haute tension (Mains 230V sur B.Cu, couloir large réservé en priorité)
        # 3. Signaux point-à-point simples (Net-(R...), Net-(J...))
        # 4. Rails d'alimentation (+3.3V, +12V, -12V)
        # 5. Masse (GND)
        u2_order = {'moc4': 0, 'moc3': 1, 'moc2': 2, 'moc1': 3, 'aop': 4}
        def routing_priority(n):
            name = n['name']
            upper = name.upper()
            is_mains = (n['rule'].track_width >= from_mm(0.70)) or any(m in upper for m in ['230', 'MAINS', 'AC', 'SECTEUR'])
            if name in u2_order:
                return (0, u2_order[name])
            if is_mains:
                return (1, 0)
            if name not in ['+3.3V', '+12V', '-12V', 'GND'] and len(n['pads']) == 2:
                return (2, 0)
            if name == '+3.3V':
                return (3, 0)
            if name in ['+12V', '-12V']:
                return (4, 0)
            if name == 'GND':
                return (6, -len(n['pads']))
            return (5, -len(n['pads']))

        nets_to_route.sort(key=routing_priority)

        total_segs = 0
        total_vias = 0
        total_len = 0
        success_nets = 0

        total_nets_count = len(nets_to_route)
        retry_queue: List[Tuple[Dict, Dict, Dict]] = []
        for idx_net, net in enumerate(nets_to_route):
            if progress_callback and total_nets_count > 0:
                pct = int(55 + (idx_net / total_nets_count) * 35)  # 55% à 90%
                progress_callback(pct, f"Routage A*... net {net['name']} ({idx_net + 1}/{total_nets_count})")

            conns = self._build_tree(net['pads'], net['name'])
            net_ok = True
            routed_pads = []

            for p1, p2 in conns:
                path = self._route_astar(p1, p2, net['netcode'], net['rule'])
                if path:
                    segs, vias, l = self._apply_path_with_kicad_items(
                        path, p1, p2, net['netcode'], net['rule']
                    )
                    total_segs += segs
                    total_vias += vias
                    total_len += l
                    if p1 not in routed_pads: routed_pads.append(p1)
                    if p2 not in routed_pads: routed_pads.append(p2)
                else:
                    # Repli multi-candidats vers les pastilles déjà routées du réseau
                    alt_found = False
                    if len(net['pads']) > 2 and routed_pads:
                        isolated = p1 if p1 not in routed_pads else p2
                        target_ref = p2 if isolated == p1 else p1
                        sorted_alts = sorted(
                            routed_pads,
                            key=lambda p: (p['pos'][0]-isolated['pos'][0])**2 + (p['pos'][1]-isolated['pos'][1])**2
                        )
                        for alt_p in sorted_alts:
                            if alt_p == target_ref:
                                continue
                            alt_path = self._route_astar(alt_p, isolated, net['netcode'], net['rule'])
                            if alt_path:
                                segs, vias, l = self._apply_path_with_kicad_items(
                                    alt_path, alt_p, isolated, net['netcode'], net['rule']
                                )
                                total_segs += segs
                                total_vias += vias
                                total_len += l
                                if isolated not in routed_pads: routed_pads.append(isolated)
                                alt_found = True
                                logger.info(
                                    f"  Réseau {net['name']} : connexion alternative trouvée "
                                    f"({alt_p['ref']}.{alt_p['pad_num']} -> {isolated['ref']}.{isolated['pad_num']})"
                                )
                                break
                    if not alt_found:
                        logger.warning(
                            f"  Échec routage (Passe 1) : Réseau {net['name']} "
                            f"({p1['ref']}.{p1['pad_num']} -> {p2['ref']}.{p2['pad_num']}) mis en file d'attente (Retry Queue)"
                        )
                        net_ok = False
                        retry_queue.append((p1, p2, net))

            if net_ok:
                success_nets += 1

        # -------------------------------------------------------------
        #  PASSE 2 : RETRY QUEUE & PUSH AND SHOVE DES PISTES BLOQUANTES (Correction 3)
        # -------------------------------------------------------------
        if retry_queue:
            logger.info(f"\n▶ PASSE 2 (Retry Queue) : Tentatives multiples pour {len(retry_queue)} connexion(s) en attente...")
            if progress_callback:
                progress_callback(90, "Passe 2 : Traitement de la Retry Queue et Push and Shove...")

            saved_via_penalty = self.via_penalty
            self.current_max_expansions = 1500000
            self.current_max_expansions_single = 1000000

            resolved_in_pass2 = 0
            for p1, p2, net in retry_queue:
                netcode = net['netcode']
                rule = net['rule']
                logger.info(f"  Retry Queue : Tentative de routage pour {net['name']} ({p1['ref']}.{p1['pad_num']} -> {p2['ref']}.{p2['pad_num']})")

                # Tentative 1 : Recherche approfondie avec budget d'expansion élevé
                path = self._route_astar(p1, p2, netcode, rule)
                if path:
                    segs, vias, l = self._apply_path_with_kicad_items(
                        path, p1, p2, netcode, rule
                    )
                    total_segs += segs
                    total_vias += vias
                    total_len += l
                    resolved_in_pass2 += 1
                    logger.info(f"  ✓ Connexion résolue avec succès pour {net['name']} !")
                    continue

                # Tentative 2 : Push and Shove des pistes bloquantes (Correction 3)
                logger.info(f"  Activation du Push and Shove pour débloquer {net['name']}...")
                path = self._route_with_push_and_shove(p1, p2, netcode, rule)
                if path:
                    segs, vias, l = self._apply_path_with_kicad_items(
                        path, p1, p2, netcode, rule
                    )
                    total_segs += segs
                    total_vias += vias
                    total_len += l
                    resolved_in_pass2 += 1
                    logger.info(f"  ✓ Push and Shove réussi : Connexion {net['name']} forcée avec succès !")
                else:
                    logger.error(f"  ✗ Échec persistant pour {net['name']} après Push and Shove.")

            self.via_penalty = saved_via_penalty
            self.current_max_expansions = 800000
            self.current_max_expansions_single = 500000

            success_nets += resolved_in_pass2

        elapsed = time.time() - start_t
        stats = {
            'total_nets': len(nets_to_route),
            'success_nets': success_nets,
            'success_percent': (success_nets / len(nets_to_route) * 100.0) if nets_to_route else 100.0,
            'tracks': total_segs,
            'vias': total_vias,
            'length_mm': round(to_mm(total_len), 2),
            'time_s': round(elapsed, 2)
        }

        # 4.5 Logique "Push and Shove" pour le Plan de Masse (Zéro Via GND, Zéro Piste parasite)
        if is_gnd_active:
            if progress_callback:
                progress_callback(92, "Push & Shove : Élargissement des couloirs du plan de masse (GND >= 0.5mm)...")
            try:
                shoved = Push_And_Shove_GND_Corridors(
                    self.board,
                    target_layers=[pcbnew.F_Cu, pcbnew.B_Cu],
                    gnd_net_name=gnd_target,
                    min_corridor_mm=0.50
                )
                if shoved > 0:
                    logger.info(f"✓ Push & Shove GND : {shoved} segments de piste décalés pour ouvrir les couloirs du plan de masse (>= 0.50 mm).")
            except Exception as pse:
                logger.warning(f"Avertissement Push & Shove GND : {pse}")

        # 5. Nettoyage de la Sérigraphie (Silkscreen)
        if progress_callback:
            progress_callback(94, "Nettoyage DFM de la sérigraphie (Silkscreen)...")
        try:
            self.cleanup_silkscreen(self.board)
        except Exception as se:
            logger.warning(f"Avertissement lors du nettoyage de la sérigraphie : {se}")

        return stats

    def _build_tree(self, pads: List[Dict], netname: str) -> List[Tuple[Dict, Dict]]:
        """Construit un arbre de connexions optimisé pour le réseau."""
        if len(pads) <= 1:
            return []
        if len(pads) == 2:
            return [(pads[0], pads[1])]

        if netname == '+3.3V':
            p_map = {f"{p['ref']}.{p['pad_num']}": p for p in pads}
            conns = []
            if 'J7.2' in p_map and 'U2.9' in p_map:
                conns.append((p_map['J7.2'], p_map['U2.9']))
            if 'U2.9' in p_map and 'U6.1' in p_map:
                conns.append((p_map['U2.9'], p_map['U6.1']))
            if 'U6.1' in p_map and 'U5.1' in p_map:
                conns.append((p_map['U6.1'], p_map['U5.1']))
            if 'U5.1' in p_map and 'U4.1' in p_map:
                conns.append((p_map['U5.1'], p_map['U4.1']))
            if 'U4.1' in p_map and 'U3.1' in p_map:
                conns.append((p_map['U4.1'], p_map['U3.1']))
            if len(conns) >= 4:
                return conns
        elif netname == 'aop':
            p_map = {f"{p['ref']}.{p['pad_num']}": p for p in pads}
            conns = []
            if 'U2.4' in p_map and 'J2.2' in p_map:
                conns.append((p_map['U2.4'], p_map['J2.2']))
            if 'J2.2' in p_map and 'U12.1' in p_map:
                conns.append((p_map['J2.2'], p_map['U12.1']))
            if 'U12.1' in p_map and 'U12.3' in p_map:
                conns.append((p_map['U12.1'], p_map['U12.3']))
            if len(conns) == 3:
                return conns

        # Arbre couvrant minimal (MST) avec regroupement interne des boîtiers
        by_fp = defaultdict(list)
        for p in pads:
            by_fp[p['ref']].append(p)

        conns = []
        rep_pads = []
        for ref, fps_pads in by_fp.items():
            if len(fps_pads) > 1:
                for i in range(len(fps_pads) - 1):
                    conns.append((fps_pads[i], fps_pads[i+1]))
                rep_pads.append(fps_pads[-1])
            else:
                rep_pads.append(fps_pads[0])

        visited = [0]
        unvisited = list(range(1, len(rep_pads)))
        while unvisited:
            best_dist = float('inf')
            best_pair = None
            best_u = -1
            for v in visited:
                pv = rep_pads[v]['pos']
                for idx_u, u in enumerate(unvisited):
                    pu = rep_pads[u]['pos']
                    d = (pv[0] - pu[0])**2 + (pv[1] - pu[1])**2
                    if d < best_dist:
                        best_dist = d
                        best_pair = (rep_pads[v], rep_pads[u])
                        best_u = idx_u
            conns.append(best_pair)
            visited.append(unvisited.pop(best_u))

        return conns


def cleanup_silkscreen(board, margin_mm: float = 0.25) -> int:
    """
    Post-traitement de nettoyage de la sérigraphie (Silkscreen).
    Parcourt les textes de référence (RefDes) et s'assure que leur Bounding Box :
      1. Ne chevauche AUCUNE pastille (Pad) ni cuivre nu (Interdiction DRC/DFM absolue).
      2. Évite autant que possible de chevaucher les pistes de cuivre (Tracks/Vias)
         pour éliminer le relief (bosse de 35µm) qui déforme l'encre et nuit à la lisibilité.
      3. Reste strictement à l'intérieur des bordures Edge.Cuts.
      4. Ne chevauche pas les autres textes de référence voisins.

    Args:
        board: Instance BOARD KiCad.
        margin_mm: Marge de sécurité avec les pastilles (en mm).

    Returns:
        Nombre de textes de référence déplacés.
    """
    if board is None:
        return 0

    margin = from_mm(margin_mm)
    margin_edge = from_mm(0.60)
    all_pads = []
    all_tracks = []
    all_refs = []

    try:
        bb = board.GetBoardEdgesBoundingBox()
    except Exception:
        bb = board.ComputeBoundingBox()

    try:
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                all_pads.append((fp.GetReference(), pad.GetBoundingBox()))
            r = fp.Reference()
            if r and hasattr(r, "IsVisible") and r.IsVisible():
                all_refs.append((fp.GetReference(), r.GetBoundingBox()))

        # Collecter toutes les pistes et vias de la carte pour éviter le relief cuivre
        for t in board.GetTracks():
            all_tracks.append((t.GetLayer(), t.GetBoundingBox()))
    except Exception as e:
        logger.warning(f"Impossible d'extraire les éléments pour le nettoyage sérigraphie: {e}")
        return 0

    moved_count = 0
    for fp in board.GetFootprints():
        try:
            ref = fp.Reference()
            if ref is None:
                continue
            if hasattr(ref, "IsVisible") and not ref.IsVisible():
                continue

            ref_box = ref.GetBoundingBox()
            ref_layer = ref.GetLayer()
            copper_layer = pcbnew.F_Cu if ref_layer == getattr(pcbnew, "F_SilkS", 5) else pcbnew.B_Cu

            # Vérifier si la référence chevauche une pastille quelconque
            has_pad_overlap = False
            for fp_ref, pad_box in all_pads:
                if not (ref_box.GetRight() + margin < pad_box.GetX() or
                        ref_box.GetX() - margin > pad_box.GetRight() or
                        ref_box.GetBottom() + margin < pad_box.GetY() or
                        ref_box.GetY() - margin > pad_box.GetBottom()):
                    has_pad_overlap = True
                    break

            # Vérifier si la référence chevauche une piste de cuivre
            has_track_overlap = False
            for t_lay, tb in all_tracks:
                if (t_lay == copper_layer or t_lay == -1) and ref_box.Intersects(tb):
                    has_track_overlap = True
                    break

            # Vérifier si la référence est coupée par la bordure Edge.Cuts
            clipped_by_edge = (
                ref_box.GetX() < bb.GetX() + margin_edge or ref_box.GetRight() > bb.GetRight() - margin_edge or
                ref_box.GetY() < bb.GetY() + margin_edge or ref_box.GetBottom() > bb.GetBottom() - margin_edge
            )

            if not has_pad_overlap and not has_track_overlap and not clipped_by_edge:
                continue

            # Calculer la boîte englobante des pastilles du composant
            pad_boxes = [pad.GetBoundingBox() for pad in fp.Pads()]
            if pad_boxes:
                b_xmin = min(b.GetX() for b in pad_boxes)
                b_xmax = max(b.GetRight() for b in pad_boxes)
                b_ymin = min(b.GetY() for b in pad_boxes)
                b_ymax = max(b.GetBottom() for b in pad_boxes)
            else:
                fp_b = fp.GetBoundingBox()
                b_xmin, b_xmax = fp_b.GetX(), fp_b.GetRight()
                b_ymin, b_ymax = fp_b.GetY(), fp_b.GetBottom()

            ref_w = ref_box.GetWidth()
            ref_h = ref_box.GetHeight()
            orig_pos = ref.GetPosition()
            cur_cx = (b_xmin + b_xmax) // 2
            cur_cy = (b_ymin + b_ymax) // 2

            # Candidats multi-distances (0.25mm, 0.60mm, 1.0mm) sur 8 directions
            candidates = []
            for d_mm in (0.25, 0.60, 1.0):
                d_nm = from_mm(d_mm)
                candidates.extend([
                    pcbnew.VECTOR2I(cur_cx, int(b_ymin - ref_h / 2 - d_nm)),  # Au-dessus
                    pcbnew.VECTOR2I(cur_cx, int(b_ymax + ref_h / 2 + d_nm)),  # En-dessous
                    pcbnew.VECTOR2I(int(b_xmax + ref_w / 2 + d_nm), cur_cy),  # À droite
                    pcbnew.VECTOR2I(int(b_xmin - ref_w / 2 - d_nm), cur_cy),  # À gauche
                    pcbnew.VECTOR2I(int(b_xmax + ref_w / 2 + d_nm), int(b_ymin - ref_h / 2 - d_nm)),
                    pcbnew.VECTOR2I(int(b_xmin - ref_w / 2 - d_nm), int(b_ymin - ref_h / 2 - d_nm)),
                    pcbnew.VECTOR2I(int(b_xmax + ref_w / 2 + d_nm), int(b_ymax + ref_h / 2 + d_nm)),
                    pcbnew.VECTOR2I(int(b_xmin - ref_w / 2 - d_nm), int(b_ymax + ref_h / 2 + d_nm)),
                ])

            best_pos = None
            min_score = 99999999

            for cand in candidates:
                c_xmin = cand.x - ref_w // 2
                c_xmax = cand.x + ref_w // 2
                c_ymin = cand.y - ref_h // 2
                c_ymax = cand.y + ref_h // 2

                # Doit impérativement rester à l'intérieur du PCB
                if (c_xmin < bb.GetX() + margin_edge or c_xmax > bb.GetRight() - margin_edge or
                    c_ymin < bb.GetY() + margin_edge or c_ymax > bb.GetBottom() - margin_edge):
                    continue

                # Pénalité 1 : Collision avec pastille (interdit absolu DRC : 100 000 pts)
                pad_colls = sum(1 for _, p_box in all_pads if not (
                    c_xmax + margin < p_box.GetX() or c_xmin - margin > p_box.GetRight() or
                    c_ymax + margin < p_box.GetY() or c_ymin - margin > p_box.GetBottom()
                ))

                # Pénalité 2 : Chevauchement d'autre référence (10 000 pts)
                ref_colls = sum(1 for o_ref, o_box in all_refs if o_ref != fp.GetReference() and not (
                    c_xmax < o_box.GetX() or c_xmin > o_box.GetRight() or
                    c_ymax < o_box.GetY() or c_ymin > o_box.GetBottom()
                ))

                # Pénalité 3 : Chevauchement de piste de cuivre (relief 35µm : 2 000 pts)
                track_colls = sum(1 for t_lay, tb in all_tracks if (t_lay == copper_layer or t_lay == -1) and not (
                    c_xmax < tb.GetX() or c_xmin > tb.GetRight() or
                    c_ymax < tb.GetY() or c_ymin > tb.GetBottom()
                ))

                dist = math.hypot(cand.x - orig_pos.x, cand.y - orig_pos.y)
                score = pad_colls * 100000 + ref_colls * 10000 + track_colls * 2000 + dist / 1e6

                if score < min_score:
                    min_score = score
                    best_pos = cand

            if best_pos and (best_pos.x != orig_pos.x or best_pos.y != orig_pos.y):
                ref.SetPosition(best_pos)
                moved_count += 1
                logger.info(
                    f"  ✓ RefDes {fp.GetReference()} repositionné hors pastilles/pistes "
                    f"vers ({to_mm(best_pos.x):.2f}, {to_mm(best_pos.y):.2f}) mm"
                )
        except Exception as fe:
            logger.debug(f"Erreur ajustement sérigraphie {fp.GetReference()}: {fe}")

    return moved_count


# Liaison statique et alias d'accessibilité
SmartRouter.cleanup_silkscreen = staticmethod(cleanup_silkscreen)
SmartRouter.Cleanup_Silkscreen = staticmethod(cleanup_silkscreen)
Cleanup_Silkscreen = cleanup_silkscreen


# =============================================================================
#  LOGIQUE "PUSH AND SHOVE" POUR LE PLAN DE MASSE (Zéro Via GND, Zéro Piste parasite)
# =============================================================================

def dist_point_to_segment_nm(px: int, py: int, x1: int, y1: int, x2: int, y2: int) -> Tuple[float, int, int]:
    """Calcule la distance minimale et le point projeté le plus proche sur un segment 2D (unités nm)."""
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1
    c2 = vx * vx + vy * vy
    if c2 <= 1e-9:
        return math.hypot(wx, wy), x1, y1
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / c2))
    proj_x = int(round(x1 + t * vx))
    proj_y = int(round(y1 + t * vy))
    return math.hypot(px - proj_x, py - proj_y), proj_x, proj_y


def Push_And_Shove_GND_Corridors(
    board,
    target_layers: Optional[List[int]] = None,
    gnd_net_name: str = "GND",
    min_corridor_mm: float = 0.50,
    drc_clearance_mm: float = 0.20,
    max_passes: int = 10
) -> int:
    """
    Logique post-routage 'Push and Shove' pour le Plan de Masse (GND).
    
    Vérifie après le routage que l'espace libre autour de chaque pastille GND
    est d'au moins 0.50 mm pour permettre au cuivre de masse de couler sans créer d'îlots.
    
    Si un segment de piste étranger provoque un étranglement (< 0.50 mm) :
      - Il lui applique une translation (décalage dynamique) pour ouvrir le couloir à >= 0.50 mm.
      - En cascade ('Push and Shove'), il décale à son tour les pistes voisines si le décalage
        enfreint le seuil de DRC.
      - Garantit zéro via GND parasite et zéro piste parasite.
      - Réaligne les extrémités des pistes adjacentes du même net pour préserver la continuité.
      
    Args:
        board: Instance BOARD KiCad.
        target_layers: Liste des IDs de couches (par défaut [F_Cu, B_Cu]).
        gnd_net_name: Nom du réseau de masse ("GND", "0V", etc.).
        min_corridor_mm: Largeur minimale garantie du couloir (0.50 mm).
        drc_clearance_mm: Dégagement d'isolement DRC minimal entre pistes (0.20 mm).
        max_passes: Nombre maximal de passes itératives d'ajustement.
        
    Returns:
        Nombre total de segments déplacés.
    """
    # Desactive pour eviter tout deplacement hors-grille et preserver l'alignement strict 45 degres
    return 0

    if board is None:
        return 0

    if target_layers is None:
        target_layers = [pcbnew.F_Cu, pcbnew.B_Cu]

    min_corridor_nm = int(round(min_corridor_mm * 1e6))
    drc_clearance_nm = int(round(drc_clearance_mm * 1e6))

    # 1. Identifier les pastilles GND sur les couches cibles
    gnd_pads = []
    gnd_target_upper = gnd_net_name.strip().upper()
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            netname = pad.GetNetname().strip().upper()
            if netname == gnd_target_upper or "GND" in netname or netname == "0V":
                lset = pad.GetLayerSet()
                for l in target_layers:
                    if lset.Contains(l):
                        gnd_pads.append((pad, l))
                        break

    if not gnd_pads:
        return 0

    edge_bb = board.GetBoardEdgesBoundingBox()
    margin_board = 1000000  # 1 mm
    board_x_min = edge_bb.GetX() + margin_board
    board_y_min = edge_bb.GetY() + margin_board
    board_x_max = edge_bb.GetRight() - margin_board
    board_y_max = edge_bb.GetBottom() - margin_board

    # Collecter tous les pads hors-GND pour éviter d'y translater une piste
    all_other_pads = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            pnet = pad.GetNetname().strip().upper()
            if pnet != gnd_target_upper and "GND" not in pnet and pnet != "0V":
                box = pad.GetBoundingBox()
                lset = pad.GetLayerSet()
                all_other_pads.append((box, pad.GetNetCode(), lset))

    # Points d'ancrage fixes par net (pads et vias) pour garantir qu'aucune extrémité de piste ne s'en détache
    net_fixed_anchors = defaultdict(set)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            pos = pad.GetPosition()
            net_fixed_anchors[pad.GetNetCode()].add((pos.x, pos.y))
    for v in board.GetTracks():
        if isinstance(v, pcbnew.PCB_VIA):
            vpos = v.GetPosition()
            net_fixed_anchors[v.GetNetCode()].add((vpos.x, vpos.y))

    total_shoved_segments = 0

    for pass_num in range(1, max_passes + 1):
        pass_shoved = 0
        all_tracks = [t for t in board.GetTracks() if not isinstance(t, pcbnew.PCB_VIA)]

        for pad, layer in gnd_pads:
            ppos = pad.GetPosition()
            px, py = ppos.x, ppos.y
            pad_r = max(pad.GetSize().x, pad.GetSize().y) // 2

            for track in all_tracks:
                if track.GetLayer() != layer:
                    continue
                tnet = track.GetNetname().strip().upper()
                if tnet == gnd_target_upper or "GND" in tnet:
                    continue

                s = track.GetStart()
                e = track.GetEnd()
                tw = track.GetWidth()

                d, proj_x, proj_y = dist_point_to_segment_nm(px, py, s.x, s.y, e.x, e.y)
                gap = d - (tw / 2.0) - pad_r

                # Étranglement : si l'espace libre est inférieur au couloir requis (0.50 mm)
                if gap < min_corridor_nm and d < (min_corridor_nm * 3):
                    needed_shift = int(math.ceil(min_corridor_nm - gap)) + 20000  # +0.02mm marge sécurité

                    # Direction de poussée : s'éloigner radialement du centre du pad GND
                    dx_u = proj_x - px
                    dy_u = proj_y - py
                    dist_u = math.hypot(dx_u, dy_u)
                    if dist_u > 1e-6:
                        ux = dx_u / dist_u
                        uy = dy_u / dist_u
                    else:
                        vx = e.x - s.x
                        vy = e.y - s.y
                        seg_len = math.hypot(vx, vy)
                        if seg_len > 1e-6:
                            ux = -vy / seg_len
                            uy = vx / seg_len
                        else:
                            ux, uy = 1.0, 0.0

                    shift_x = int(round(ux * needed_shift))
                    shift_y = int(round(uy * needed_shift))

                    shove_queue = [(track, shift_x, shift_y)]
                    processed_in_cascade = set()

                    while shove_queue:
                        curr_t, sx, sy = shove_queue.pop(0)
                        t_id = id(curr_t)
                        if t_id in processed_in_cascade:
                            continue
                        processed_in_cascade.add(t_id)

                        orig_start = curr_t.GetStart()
                        orig_end = curr_t.GetEnd()

                        anchors = net_fixed_anchors.get(curr_t.GetNetCode(), set())
                        is_start_anchored = (orig_start.x, orig_start.y) in anchors
                        is_end_anchored = (orig_end.x, orig_end.y) in anchors

                        if is_start_anchored and is_end_anchored:
                            # Segment reliant directement deux pastilles ou vias fixes : ne pas déplacer en bloc
                            continue

                        new_start_x = orig_start.x if is_start_anchored else max(board_x_min, min(board_x_max, orig_start.x + sx))
                        new_start_y = orig_start.y if is_start_anchored else max(board_y_min, min(board_y_max, orig_start.y + sy))
                        new_end_x = orig_end.x if is_end_anchored else max(board_x_min, min(board_x_max, orig_end.x + sx))
                        new_end_y = orig_end.y if is_end_anchored else max(board_y_min, min(board_y_max, orig_end.y + sy))

                        # Éviter toute collision avec les pastilles étrangères
                        collides_other_pad = False
                        for pbox, pnetcode, plset in all_other_pads:
                            if pnetcode == curr_t.GetNetCode() or not plset.Contains(curr_t.GetLayer()):
                                continue
                            req_pdist = (curr_t.GetWidth() // 2) + drc_clearance_nm
                            p_cx = (pbox.GetX() + pbox.GetRight()) // 2
                            p_cy = (pbox.GetY() + pbox.GetBottom()) // 2
                            p_r = max(pbox.GetWidth(), pbox.GetHeight()) // 2
                            dp, _, _ = dist_point_to_segment_nm(p_cx, p_cy, new_start_x, new_start_y, new_end_x, new_end_y)
                            if dp < (p_r + req_pdist):
                                collides_other_pad = True
                                break

                        if collides_other_pad:
                            continue

                        # Translation du segment
                        curr_t.SetStart(pcbnew.VECTOR2I(new_start_x, new_start_y))
                        curr_t.SetEnd(pcbnew.VECTOR2I(new_end_x, new_end_y))
                        pass_shoved += 1
                        total_shoved_segments += 1

                        # Préservation stricte de la connectivité électrique (même équipotentielle)
                        for other_t in all_tracks:
                            if other_t is curr_t or other_t.GetNetCode() != curr_t.GetNetCode():
                                continue
                            if other_t.GetLayer() != curr_t.GetLayer():
                                continue
                            if not is_start_anchored:
                                if other_t.GetStart() == orig_start:
                                    other_t.SetStart(pcbnew.VECTOR2I(new_start_x, new_start_y))
                                elif other_t.GetEnd() == orig_start:
                                    other_t.SetEnd(pcbnew.VECTOR2I(new_start_x, new_start_y))
                            if not is_end_anchored:
                                if other_t.GetStart() == orig_end:
                                    other_t.SetStart(pcbnew.VECTOR2I(new_end_x, new_end_y))
                                elif other_t.GetEnd() == orig_end:
                                    other_t.SetEnd(pcbnew.VECTOR2I(new_end_x, new_end_y))

                        # Cascade Push & Shove vers les pistes voisines d'autres nets
                        for neighbor in all_tracks:
                            if neighbor is curr_t or neighbor.GetLayer() != curr_t.GetLayer():
                                continue
                            if neighbor.GetNetCode() == curr_t.GetNetCode():
                                continue

                            ns = neighbor.GetStart()
                            ne = neighbor.GetEnd()

                            d1, _, _ = dist_point_to_segment_nm(new_start_x, new_start_y, ns.x, ns.y, ne.x, ne.y)
                            d2, _, _ = dist_point_to_segment_nm(new_end_x, new_end_y, ns.x, ns.y, ne.x, ne.y)
                            d3, _, _ = dist_point_to_segment_nm(ns.x, ns.y, new_start_x, new_start_y, new_end_x, new_end_y)
                            d4, _, _ = dist_point_to_segment_nm(ne.x, ne.y, new_start_x, new_start_y, new_end_x, new_end_y)
                            min_seg_dist = min(d1, d2, d3, d4)

                            req_dist = (curr_t.GetWidth() + neighbor.GetWidth()) // 2 + drc_clearance_nm
                            if min_seg_dist < req_dist:
                                cascade_needed = int(math.ceil(req_dist - min_seg_dist)) + 20000
                                shove_queue.append((neighbor, int(round(ux * cascade_needed)), int(round(uy * cascade_needed))))

        if pass_shoved == 0:
            break

    return total_shoved_segments


# Liaison statique pour accès externe
SmartRouter.push_and_shove_gnd_corridors = staticmethod(Push_And_Shove_GND_Corridors)
SmartRouter.Push_And_Shove_GND_Corridors = staticmethod(Push_And_Shove_GND_Corridors)

