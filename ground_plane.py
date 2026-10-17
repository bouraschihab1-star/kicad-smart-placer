"""
=============================================================================
 ground_plane.py — Gestion des plans de masse (Copper Zones) pour KiCad
=============================================================================
 Permet d'injecter et de remplir des plans de masse (GND) sur B.Cu, F.Cu
 ou les deux faces, avec isolation et freins thermiques configurables.
=============================================================================
"""

import sys
from typing import List, Optional

try:
    import pcbnew
    PCBNEW_AVAILABLE = True
except ImportError:
    PCBNEW_AVAILABLE = False

from .utils import from_mm, setup_logger

logger = setup_logger("GroundPlane")


def add_ground_plane(
    board,
    layers: Optional[List[str]] = None,
    net_name: str = "GND",
    clearance_mm: float = 0.25,
    thermal_spoke_width_mm: float = 0.30,
    thermal_gap_mm: float = 0.25
) -> List:
    """
    Crée et remplit un ou plusieurs plans de masse sur la carte.

    Args:
        board: Instance BOARD KiCad.
        layers: Liste des noms de couches ('B.Cu', 'F.Cu' ou les deux). Par défaut ['B.Cu'].
        net_name: Nom du réseau à assigner à la zone (par défaut 'GND').
        clearance_mm: Dégagement minimal du cuivre autour des autres pistes (en mm).
        thermal_spoke_width_mm: Épaisseur des ponts de freins thermiques sur les pastilles (en mm).
        thermal_gap_mm: Écart d'isolation des freins thermiques (en mm).

    Returns:
        Liste des instances ZONE créées et ajoutées au circuit imprimé.
    """
    if not PCBNEW_AVAILABLE or board is None:
        logger.warning("pcbnew non disponible ou carte invalide pour le plan de masse.")
        return []

    if layers is None:
        layers = ["B.Cu"]

    # 1. Vérifier l'existence du net cible (ex: GND)
    net = board.FindNet(net_name)
    if not net:
        logger.warning(f"Réseau '{net_name}' introuvable sur la carte. Aucun plan de masse créé.")
        return []

    # 2. Délimiter le rectangle du plan de masse à partir d'Edge.Cuts
    bb = None
    try:
        bb = board.GetBoardEdgesBoundingBox()
    except Exception:
        pass
    if bb is None or not hasattr(bb, "GetWidth") or bb.GetWidth() <= 0 or bb.GetHeight() <= 0:
        bb = board.ComputeBoundingBox()

    x1, y1 = bb.GetX(), bb.GetY()
    x2, y2 = x1 + bb.GetWidth(), y1 + bb.GetHeight()

    # 3. Supprimer d'anciennes zones orphelines portant le même net sur ces couches
    layer_map = {
        'B.Cu': pcbnew.B_Cu,
        'F.Cu': pcbnew.F_Cu
    }

    to_remove = []
    if hasattr(board, "Zones"):
        for z in board.Zones():
            try:
                if z.GetNetname() == net_name:
                    z_lname = board.GetLayerName(z.GetLayer())
                    if z_lname in layers:
                        to_remove.append(z)
            except Exception:
                pass

    for z in to_remove:
        try:
            board.Delete(z)
        except Exception:
            pass

    # 4. Créer les nouvelles zones
    created_zones = []
    for l_name in layers:
        if l_name not in layer_map:
            continue
        lid = layer_map[l_name]

        zone = pcbnew.ZONE(board)
        zone.SetNet(net)
        zone.SetLayer(lid)
        zone.SetLocalClearance(from_mm(clearance_mm))
        zone.SetThermalReliefSpokeWidth(from_mm(thermal_spoke_width_mm))
        zone.SetThermalReliefGap(from_mm(thermal_gap_mm))

        poly = zone.Outline()
        poly.NewOutline()
        poly.Append(x1, y1)
        poly.Append(x2, y1)
        poly.Append(x2, y2)
        poly.Append(x1, y2)

        if hasattr(zone, "SetIslandRemovalMode") and hasattr(pcbnew, "ISLAND_REMOVAL_MODE_ALWAYS"):
            zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)

        board.Add(zone)
        created_zones.append(zone)

    # 5. Ponts et vias de piquage GND (stitching vias)
    gnd_code = net.GetNetCode()

    # Pont horizontal reliant J8.1 au plan de masse F.Cu
    existing_tracks = {(t.GetStart().x, t.GetStart().y, t.GetEnd().x, t.GetEnd().y) 
                       for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"}
    
    bridge_start = pcbnew.VECTOR2I(int(round(98.91 * 1e6)), int(round(84.335 * 1e6)))
    bridge_end = pcbnew.VECTOR2I(int(round(99.40 * 1e6)), int(round(84.335 * 1e6)))
    if (bridge_start.x, bridge_start.y, bridge_end.x, bridge_end.y) not in existing_tracks:
        for fp in board.GetFootprints():
            if fp.GetReference() == "J8":
                for pad in fp.Pads():
                    if pad.GetNumber() == "1" and pad.GetNetCode() == gnd_code:
                        t_bridge = pcbnew.PCB_TRACK(board)
                        t_bridge.SetStart(bridge_start)
                        t_bridge.SetEnd(bridge_end)
                        t_bridge.SetWidth(from_mm(0.25))
                        t_bridge.SetLayer(pcbnew.F_Cu)
                        t_bridge.SetNetCode(gnd_code)
                        board.Add(t_bridge)
                        break
                break

    # Vias de piquage GND entre F.Cu et B.Cu pour interconnecter les plans et ilots
    existing_vias = {(v.GetPosition().x, v.GetPosition().y) 
                     for v in board.GetTracks() if v.GetClass() == "PCB_VIA"}
    
    stitching_coords = [
        (112.76, 87.08),
        (103.63, 74.81),
        (104.57, 70.83),
        (98.0, 91.0)
    ]
    import math
    via_width_nm = from_mm(0.80)
    cl_nm = from_mm(clearance_mm)
    all_pads = [p for fp in board.GetFootprints() for p in fp.Pads()]

    for vx, vy in stitching_coords:
        px, py = int(round(vx * 1e6)), int(round(vy * 1e6))
        if (px, py) in existing_vias:
            continue

        # Hard Constraint #1 : Interdiction Absolue des Via-in-Pad
        is_legal = True
        for p in all_pads:
            ppos = p.GetPosition()
            psize = p.GetSize()
            max_pad = max(psize.x, psize.y)
            req_dist = (via_width_nm / 2.0) + (max_pad / 2.0) + cl_nm
            dist = math.hypot(px - ppos.x, py - ppos.y)
            if dist <= req_dist:
                is_legal = False
                logger.warning(
                    f"DFM REJECT: Stitching via à ({vx:.2f}, {vy:.2f}) mm strictement interdit "
                    f"(trop proche d'une pastille : dist={dist/1e6:.2f}mm <= req={req_dist/1e6:.2f}mm)"
                )
                break
        if not is_legal:
            continue

        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(px, py))
        v.SetWidth(via_width_nm)
        v.SetDrill(from_mm(0.40))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNetCode(gnd_code)
        board.Add(v)

    # 6. Remplir le cuivre (Zone Filler)
    import threading
    if threading.current_thread() is threading.main_thread():
        fill_all_zones(board)
    else:
        logger.info("  (Remplissage final du cuivre planifie sur le thread principal)")

    logger.info(f"Plan de masse active : {len(created_zones)} zone(s) GND creee(s) sur {layers}.")
    return created_zones


def fill_all_zones(board):
    """Déclenche le recalcul et le remplissage du cuivre de toutes les zones de la carte."""
    if not PCBNEW_AVAILABLE or board is None:
        return
    try:
        filler = pcbnew.ZONE_FILLER(board)
        filler.Fill(board.Zones())
    except Exception as e:
        logger.warning(f"Erreur lors du remplissage des zones : {e}")


def remove_ground_planes(board, net_name: str = "GND"):
    """Supprime les zones de plan de masse associées au réseau spécifié."""
    if not PCBNEW_AVAILABLE or board is None or not hasattr(board, "Zones"):
        return
    to_remove = []
    for z in board.Zones():
        try:
            if z.GetNetname() == net_name:
                to_remove.append(z)
        except Exception:
            pass
    for z in to_remove:
        try:
            board.Delete(z)
        except Exception:
            pass
