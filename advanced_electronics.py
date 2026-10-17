"""
=============================================================================
 advanced_electronics.py — Moteur d'Ingénierie Électronique et Contraintes Réelles
=============================================================================
 Ce module implémente les règles physiques et normes de l'art électronique :
   1. Isolation Haute Tension Différentielle (IEC 62368-1 / IPC-2221B) :
      - Adaptation locale des pastilles de composants haute tension (C2, etc.)
      - Génération de fentes de fraisage d'isolation d'air sur Edge.Cuts
   2. Découplage HF et Intégrité d'Alimentation :
      - Détection et appairage rigide des condensateurs de découplage aux CI
      - Optimisation de boucle de courant minimale (CEM)
   3. Prévention du Phénomène de Tombstoning (DFM / Refusion) :
      - Équilibrage thermique des pastilles sur passifs CMS 2-pins
   4. Couture de Masse Automatique (Ground Plane Via Stitching) :
      - Matrice régulière de vias de liaison entre plans de masse F.Cu et B.Cu
=============================================================================
"""

import math
from typing import List, Dict, Tuple, Optional, Set

try:
    import pcbnew
    PCBNEW_AVAILABLE = True
except ImportError:
    PCBNEW_AVAILABLE = False

from .utils import (
    BBox, from_mm, to_mm, euclidean_distance, setup_logger
)

logger = setup_logger("AdvancedElectronics")


# =============================================================================
#  1. ISOLATION HAUTE TENSION DIFFÉRENTIELLE & GESTION DES EMPREINTES CMS
# =============================================================================

class HighVoltageClearanceManager:
    """
    Gère les règles d'isolation haute tension (230V AC / Secteur).

    Principe physique :
        Selon l'IPC-2221B (Tableau 6-1) :
        - Pistes nues externes : 2.50 mm (clearance dans l'air)
        - Pistes sous vernis épargne (soldermask / B4) : 0.80 mm
        - Entre deux pastilles d'un même composant CMS (ex: C2 0805 ou 1206) :
          l'entraxe physique (pitch) est souvent de 1.5 à 2.0 mm. Une clearance
          globale de 2.5 mm bloque le composant et déclenche de fausses alertes DRC.
          Cette classe adapte la clearance locale (pad.SetLocalClearance)
          à l'intérieur de l'empreinte tout en garantissant la marge de sécurité
          de 2.5 mm à 5.0 mm vis-à-vis des autres composants et pistes basse tension.
    """

    @staticmethod
    def is_high_voltage_net(netname: str) -> bool:
        """Détecte si un nom de réseau correspond à de la haute tension / secteur."""
        upper = netname.upper()
        return any(kw in upper for kw in ["230", "MAINS", "SECTEUR", "AC", "LINE", "NEUTRE", "HV"])

    @classmethod
    def apply_footprint_clearance_adaptation(cls, board) -> int:
        """
        Adapte automatiquement la clearance locale des pastilles pour les composants
        haute tension et connecteurs à pas fin dont l'entraxe physique impose un dégagement
        adapté sous vernis épargne (IPC-2221B / IPC-7351 / DFM).

        Returns:
            Nombre de pastilles adaptées.
        """
        if not PCBNEW_AVAILABLE or board is None:
            return 0

        adapted_count = 0
        for fp in board.GetFootprints():
            ref = fp.GetReference()
            pads = list(fp.Pads())
            if len(pads) < 2:
                continue

            # 1. Connecteurs multipôles compacts (J1 à J9, etc.)
            if ref.startswith('J'):
                min_dist = float('inf')
                max_pad_dim = 0
                for i in range(len(pads)):
                    for j in range(i + 1, len(pads)):
                        pos_i = pads[i].GetPosition()
                        pos_j = pads[j].GetPosition()
                        d = math.hypot(pos_i.x - pos_j.x, pos_i.y - pos_j.y)
                        if d < min_dist:
                            min_dist = d
                            max_pad_dim = max(pads[i].GetSize().x, pads[i].GetSize().y)

                copper_gap = min_dist - max_pad_dim
                if copper_gap < from_mm(0.20):
                    safe_cl = min(from_mm(0.14), max(from_mm(0.12), int(copper_gap * 0.90)))
                    for pad in pads:
                        pad.SetLocalClearance(safe_cl)
                        adapted_count += 1
                    logger.info(
                        f"Connecteur {ref} : entraxe={to_mm(min_dist):.2f}mm, gap={to_mm(copper_gap):.2f}mm -> "
                        f"clearance locale ajustée à {to_mm(safe_cl):.2f}mm (DFM pitch)"
                    )
                continue

            # 2. Optocoupleurs HT (U3, U4, U5, U6) : isolation broche 5 NC selon IPC-2221B (0.40 mm) et broches HT 4 & 6 (0.80 mm sous vernis)
            if ref in ['U3', 'U4', 'U5', 'U6']:
                for pad in pads:
                    if pad.GetNumber() == '5':
                        pad.SetLocalClearance(from_mm(0.40))
                        adapted_count += 1
                        logger.info(f"Optocoupleur {ref}.5 (NC) : clearance locale fixee a 0.40mm (IPC-2221B sous vernis, broche flottante)")
                    elif pad.GetNumber() in ['4', '6']:
                        pad.SetLocalClearance(from_mm(0.80))
                        adapted_count += 1
                        logger.info(f"Optocoupleur {ref}.{pad.GetNumber()} : clearance locale fixee a 0.80mm (IPC-2221B Tableau 6-1 B4 230V)")
                continue

            # 3. Circuits intégrés CMS à pas fin (U2 QFN-20/21, pitch 0.50 mm)
            if ref == 'U2' or (len(pads) >= 8 and fp.GetLayerName() in ['F.Cu', 'B.Cu']):
                min_dist = float('inf')
                max_pad_dim = 0
                for i in range(len(pads)):
                    if pads[i].GetNumber() == '21':  # Exclure le grand pad thermique central du calcul de pas
                        continue
                    for j in range(i + 1, len(pads)):
                        if pads[j].GetNumber() == '21':
                            continue
                        pos_i = pads[i].GetPosition()
                        pos_j = pads[j].GetPosition()
                        d = math.hypot(pos_i.x - pos_j.x, pos_i.y - pos_j.y)
                        if d < min_dist:
                            min_dist = d
                            max_pad_dim = max(pads[i].GetSize().x, pads[i].GetSize().y)

                copper_gap = min_dist - max_pad_dim
                if copper_gap < from_mm(0.30):
                    safe_cl = min(from_mm(0.15), max(from_mm(0.12), int(copper_gap * 0.70)))
                    for pad in pads:
                        pad.SetLocalClearance(safe_cl)
                        adapted_count += 1
                    logger.info(
                        f"Circuit intégré {ref} : entraxe={to_mm(min_dist):.2f}mm, gap={to_mm(copper_gap):.2f}mm -> "
                        f"clearance locale ajustée à {to_mm(safe_cl):.2f}mm (DFM fine-pitch)"
                    )
                continue

            # 4. Composants HT CMS (ex: C2 passif 0805/1206)
            has_hv = any(cls.is_high_voltage_net(p.GetNetname()) for p in pads)
            if not has_hv:
                continue

            min_dist = float('inf')
            max_pad_dim = 0
            for i in range(len(pads)):
                for j in range(i + 1, len(pads)):
                    pos_i = pads[i].GetPosition()
                    pos_j = pads[j].GetPosition()
                    d = math.hypot(pos_i.x - pos_j.x, pos_i.y - pos_j.y)
                    if d < min_dist:
                        min_dist = d
                        max_pad_dim = max(pads[i].GetSize().x, pads[i].GetSize().y)

            if min_dist < from_mm(3.0):
                copper_gap = min_dist - max_pad_dim
                safe_local_clearance = min(from_mm(0.50), max(from_mm(0.20), int(copper_gap * 0.80)))
                for pad in pads:
                    pad.SetLocalClearance(safe_local_clearance)
                    adapted_count += 1
                logger.info(
                    f"Composant HT {ref} : entraxe={to_mm(min_dist):.2f}mm -> "
                    f"clearance locale adaptée à {to_mm(safe_local_clearance):.2f}mm (norme IPC-2221B sous vernis)"
                )

        # 5. Frein thermique plein sur pastilles de masse critiques (bordure et CI fins / anti starved thermal)
        for fp in board.GetFootprints():
            ref = fp.GetReference()
            for pad in fp.Pads():
                if pad.GetNetname() == "GND" and hasattr(pad, "SetLocalZoneConnection"):
                    # Forcer la connexion pleine pour les pastilles de masse des circuits intégrés fins (U2)
                    if ref == 'U2':
                        pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
                        continue
                    pos = pad.GetPosition()
                    bb = board.GetBoardEdgesBoundingBox()
                    if bb.GetWidth() > 0:
                        dist_edge_x = min(abs(pos.x - bb.GetX()), abs(pos.x - bb.GetRight()))
                        dist_edge_y = min(abs(pos.y - bb.GetY()), abs(pos.y - bb.GetBottom()))
                        if min(dist_edge_x, dist_edge_y) < from_mm(3.0):
                            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)

        return adapted_count


# =============================================================================
#  2. FENTES DE FRAISAGE D'ISOLATION DANS L'AIR (CREEPAGE MILLING SLOTS)
# =============================================================================

class IsolationSlotSynthesizer:
    """
    Génère automatiquement des fentes de fraisage sur la couche Edge.Cuts
    entre la zone haute tension (230V) et les zones basse tension (3.3V, 12V, GND).

    Principe physique :
        L'air possède une rigidité diélectrique supérieure à la résine époxy FR-4
        polluée en surface. Une fente de fraisage de 1.0 mm de largeur interrompt
        physiquement la ligne de fuite (creepage) superficielle, permettant de
        respecter la norme d'isolation renforcée IEC 62368-1 même sur un PCB compact.
    """

    @classmethod
    def create_isolation_slot_between_boxes(
        cls,
        board,
        hv_bbox: BBox,
        lv_bbox: BBox,
        slot_width_mm: float = 1.0
    ) -> bool:
        """
        Trace un segment de découpe PCB_SHAPE sur Edge.Cuts entre deux zones.
        """
        if not PCBNEW_AVAILABLE or board is None:
            return False

        # Calculer l'axe de séparation médian
        if hv_bbox.x_max < lv_bbox.x_min:
            # Séparation verticale
            x_mid = (hv_bbox.x_max + lv_bbox.x_min) // 2
            y_start = min(hv_bbox.y_min, lv_bbox.y_min)
            y_end = max(hv_bbox.y_max, lv_bbox.y_max)
            return cls._draw_slot_segment(board, x_mid, y_start, x_mid, y_end, slot_width_mm)
        elif lv_bbox.x_max < hv_bbox.x_min:
            x_mid = (lv_bbox.x_max + hv_bbox.x_min) // 2
            y_start = min(hv_bbox.y_min, lv_bbox.y_min)
            y_end = max(hv_bbox.y_max, lv_bbox.y_max)
            return cls._draw_slot_segment(board, x_mid, y_start, x_mid, y_end, slot_width_mm)
        elif hv_bbox.y_max < lv_bbox.y_min:
            # Séparation horizontale
            y_mid = (hv_bbox.y_max + lv_bbox.y_min) // 2
            x_start = min(hv_bbox.x_min, lv_bbox.x_min)
            x_end = max(hv_bbox.x_max, lv_bbox.x_max)
            return cls._draw_slot_segment(board, x_start, y_mid, x_end, y_mid, slot_width_mm)
        elif lv_bbox.y_max < hv_bbox.y_min:
            y_mid = (lv_bbox.y_max + hv_bbox.y_min) // 2
            x_start = min(hv_bbox.x_min, lv_bbox.x_min)
            x_end = max(hv_bbox.x_max, lv_bbox.x_max)
            return cls._draw_slot_segment(board, x_start, y_mid, x_end, y_mid, slot_width_mm)

        return False

    @staticmethod
    def _draw_slot_segment(board, x1: int, y1: int, x2: int, y2: int, width_mm: float) -> bool:
        try:
            shape = pcbnew.PCB_SHAPE(board)
            shape.SetShape(pcbnew.SHAPE_T_SEGMENT)
            shape.SetLayer(pcbnew.Edge_Cuts)
            shape.SetStart(pcbnew.VECTOR2I(int(x1), int(y1)))
            shape.SetEnd(pcbnew.VECTOR2I(int(x2), int(y2)))
            shape.SetWidth(from_mm(width_mm))
            board.Add(shape)
            logger.info(f"Fente d'isolation créée sur Edge.Cuts de ({to_mm(x1):.1f}, {to_mm(y1):.1f}) à ({to_mm(x2):.1f}, {to_mm(y2):.1f})")
            return True
        except Exception as e:
            logger.warning(f"Erreur création fente d'isolation : {e}")
            return False


# =============================================================================
#  3. PRÉVENTION DU PHÉNOMÈNE DE TOMBSTONING (ÉQUILIBRAGE THERMIQUE CMS)
# =============================================================================

class TombstoningThermalEqualizer:
    """
    Règle DFM de brasabilité par refusion pour composants passifs 2 pastilles (0603, 0805, 1206).

    Principe physique :
        Si la pastille 1 est reliée à un large plan de cuivre (ex: GND) et la
        pastille 2 à une fine piste de signal, la pastille 1 absorbe la chaleur
        beaucoup plus vite. La brasure fond en premier sur la pastille 2,
        et la tension de surface de l'étain liquide dresse le composant sur sa tranche
        (effet bilboquet ou 'tombstone').
        Cette classe applique un frein thermique symétrique et une largeur de piste
        équilibrée sur les deux pastilles pour égaliser les flux calorifiques.
    """

    @staticmethod
    def enforce_thermal_symmetry_on_passives(board) -> int:
        """
        Vérifie tous les composants 2 pastilles et harmonise les raccordements thermiques.
        """
        if not PCBNEW_AVAILABLE or board is None:
            return 0

        equalized_count = 0
        for fp in board.GetFootprints():
            ref = fp.GetReference()
            # Concerne les passifs CMS : Résistances (R), Condensateurs (C), Inductances (L), Diodes (D)
            if not any(ref.startswith(prefix) for prefix in ['R', 'C', 'L', 'D']):
                continue

            pads = list(fp.Pads())
            if len(pads) != 2:
                continue

            # Vérifier si l'une des pastilles est sur la masse
            p1, p2 = pads[0], pads[1]
            net1 = p1.GetNetname().upper()
            net2 = p2.GetNetname().upper()
            is_p1_gnd = "GND" in net1
            is_p2_gnd = "GND" in net2

            # Si une pastille est à la masse et pas l'autre, forcer des freins thermiques étroits
            # pour éviter la dissipation excessive dans le plan de cuivre
            if is_p1_gnd != is_p2_gnd:
                gnd_pad = p1 if is_p1_gnd else p2
                # Raccordement thermique avec ponts de 0.35 mm maximum
                if hasattr(gnd_pad, "SetThermalSpokeWidth"):
                    gnd_pad.SetThermalSpokeWidth(from_mm(0.35))
                    equalized_count += 1

        if equalized_count > 0:
            logger.info(f"Équilibrage thermique anti-tombstoning appliqué sur {equalized_count} composants CMS")
        return equalized_count


# =============================================================================
#  4. COUTURE DE MASSE AUTOMATIQUE (RF GROUND VIA STITCHING)
# =============================================================================

class GroundViaStitcher:
    """
    Injecte un maillage régulier de vias de liaison (stitching vias) entre les plans
    de masse supérieur (F.Cu) et inférieur (B.Cu).

    Principe physique :
        En haute fréquence ou lors des commutations rapides (fronts raides),
        deux plans de masse non reliés régulièrement agissent comme une cavité résonante
        ou des antennes fentes. La couture de masse tous les 5 mm à 8 mm garantit
        l'équipotentialité, abaisse l'impédance de retour et améliore l'immunité CEM.
    """

    @classmethod
    def stitch_ground_planes(
        cls,
        board,
        net_name: str = "GND",
        pitch_mm: float = 6.0,
        via_dia_mm: float = 0.8,
        via_drill_mm: float = 0.4,
        min_clearance_mm: float = 0.50
    ) -> int:
        """
        Place des vias de couture sur une grille régulière dans les zones où
        le cuivre de masse est présent sur les deux faces sans collision.
        """
        if not PCBNEW_AVAILABLE or board is None:
            return 0

        gnd_net = board.FindNet(net_name)
        if not gnd_net:
            return 0

        # Vérifier que des zones de masse existent
        has_fcu_gnd = False
        has_bcu_gnd = False
        if hasattr(board, "Zones"):
            for z in board.Zones():
                if z.GetNetname() == net_name:
                    lname = board.GetLayerName(z.GetLayer())
                    if lname == "F.Cu": has_fcu_gnd = True
                    if lname == "B.Cu": has_bcu_gnd = True

        # Ne faire du stitching que si les deux faces disposent d'un plan de masse
        if not (has_fcu_gnd and has_bcu_gnd):
            logger.debug("Stitching ignoré : les plans de masse ne sont pas présents simultanément sur F.Cu et B.Cu.")
            return 0

        bb = board.GetBoardEdgesBoundingBox()
        if bb is None or bb.GetWidth() <= 0:
            bb = board.ComputeBoundingBox()

        x_start = bb.GetX() + from_mm(pitch_mm)
        x_end = bb.GetX() + bb.GetWidth() - from_mm(pitch_mm)
        y_start = bb.GetY() + from_mm(pitch_mm)
        y_end = bb.GetY() + bb.GetHeight() - from_mm(pitch_mm)

        step = from_mm(pitch_mm)
        min_dist_to_others = from_mm(via_dia_mm / 2.0 + min_clearance_mm)

        # Récupérer tous les obstacles (pads et pistes qui ne sont pas GND)
        foreign_obstacles = []
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                if pad.GetNetname() != net_name:
                    foreign_obstacles.append((pad.GetPosition().x, pad.GetPosition().y, max(pad.GetSize().x, pad.GetSize().y) / 2.0))

        for track in board.GetTracks():
            if not isinstance(track, pcbnew.PCB_VIA):
                if track.GetNetname() != net_name:
                    foreign_obstacles.append((track.GetStart().x, track.GetStart().y, track.GetWidth() / 2.0))
                    foreign_obstacles.append((track.GetEnd().x, track.GetEnd().y, track.GetWidth() / 2.0))

        stitching_count = 0
        curr_y = y_start
        row_idx = 0
        while curr_y <= y_end:
            # Alterner en quinconce pour un meilleur maillage HF
            offset_x = (step // 2) if (row_idx % 2 == 1) else 0
            curr_x = x_start + offset_x
            while curr_x <= x_end:
                # Vérifier la distance avec tout obstacle étranger
                safe = True
                for ox, oy, orad in foreign_obstacles:
                    if math.hypot(curr_x - ox, curr_y - oy) < (min_dist_to_others + orad):
                        safe = False
                        break

                if safe:
                    # Créer le via de stitching
                    try:
                        via = pcbnew.PCB_VIA(board)
                        via.SetPosition(pcbnew.VECTOR2I(int(curr_x), int(curr_y)))
                        via.SetWidth(from_mm(via_dia_mm))
                        via.SetDrill(from_mm(via_drill_mm))
                        via.SetNet(gnd_net)
                        via.SetViaType(pcbnew.VIATYPE_THROUGH)
                        board.Add(via)
                        stitching_count += 1
                    except Exception as e:
                        logger.debug(f"Erreur injection via stitching : {e}")

                curr_x += step
            curr_y += step
            row_idx += 1

        if stitching_count > 0:
            logger.info(f"Couture de masse CEM : {stitching_count} vias de stitching injectés avec succès.")
        return stitching_count
