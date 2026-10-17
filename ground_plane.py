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

    # 5. Routine de secours pour pastilles GND isolées et vias de piquage (Correction 2)
    existing_vias = {
        (v.GetPosition().x, v.GetPosition().y) 
        for v in board.GetTracks() if v.GetClass() == "PCB_VIA"
    }

    import math
    via_width_nm = from_mm(0.60)
    via_drill_nm = from_mm(0.30)
    cl_nm = from_mm(clearance_mm)

    all_pads = [(fp.GetReference(), p) for fp in board.GetFootprints() for p in fp.Pads()]
    smd_gnd_pads = [
        (ref, p) for ref, p in all_pads 
        if p.GetNetCode() == gnd_code and p.GetDrillSize().x == 0 and p.GetLayerSet().Contains(pcbnew.F_Cu)
    ]

    edge_bb = board.GetBoardEdgesBoundingBox()
    margin_nm = from_mm(1.0)

    # Pour chaque pastille GND CMS isolée sur F.Cu, poser un via de secours ET relier le via à la pastille
    for ref, pad in smd_gnd_pads:
        ppos = pad.GetPosition()
        psz = pad.GetSize()
        max_pad = max(psz.x, psz.y)

        # Condition stricte absolue :
        # Distance_Euclidienne > (via.GetWidth() / 2) + (max(pad.GetSize().x, pad.GetSize().y) / 2) + clearance_netclass
        req_dist = (via_width_nm / 2.0) + (max_pad / 2.0) + cl_nm

        best_candidate = None
        for extra_d in [0.15, 0.30, 0.50, 0.80]:
            search_dist = req_dist + from_mm(extra_d)
            for deg in [0, 45, 90, 135, 180, 225, 270, 315]:
                rad = math.radians(deg)
                vx = int(round(ppos.x + search_dist * math.cos(rad)))
                vy = int(round(ppos.y + search_dist * math.sin(rad)))

                if not (edge_bb.GetX() + margin_nm <= vx <= edge_bb.GetRight() - margin_nm and
                        edge_bb.GetY() + margin_nm <= vy <= edge_bb.GetBottom() - margin_nm):
                    continue

                # Vérification stricte anti-collision et anti-via-in-pad
                is_legal = True
                for o_ref, o_pad in all_pads:
                    o_pos = o_pad.GetPosition()
                    o_sz = o_pad.GetSize()
                    o_max = max(o_sz.x, o_sz.y)
                    o_req = (via_width_nm / 2.0) + (o_max / 2.0) + cl_nm
                    if math.hypot(vx - o_pos.x, vy - o_pos.y) <= o_req:
                        is_legal = False
                        break

                if is_legal:
                    for ex_vx, ex_vy in existing_vias:
                        if math.hypot(vx - ex_vx, vy - ex_vy) <= (via_width_nm + cl_nm):
                            is_legal = False
                            break

                if is_legal:
                    best_candidate = (vx, vy)
                    break
            if best_candidate:
                break

        if best_candidate:
            vx, vy = best_candidate

            # 1. Instanciation du Via de secours
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(vx, vy))
            v.SetWidth(via_width_nm)
            v.SetDrill(via_drill_nm)
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetNetCode(gnd_code)
            board.Add(v)
            existing_vias.add((vx, vy))

            # 2. Correction 2 : Instanciation explicite du segment de piste reliant le via au centre de la pastille
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(pad.GetPosition())
            track.SetEnd(v.GetPosition())
            track.SetLayer(pcbnew.F_Cu)
            track.SetWidth(from_mm(0.25))
            track.SetNetCode(gnd_code)
            board.Add(track)
            logger.info(
                f"Via GND de secours et piste de liaison créés : {ref}.{pad.GetNumber()} -> "
                f"Via ({vx/1e6:.2f}, {vy/1e6:.2f}) mm sur F.Cu"
            )

    # Liaison directe de la pastille centrale thermique QFN U2.21 vers U2.8
    u2_pads = {p.GetNumber(): p for ref, p in smd_gnd_pads if ref == "U2"}
    if "21" in u2_pads and "8" in u2_pads:
        t_qfn = pcbnew.PCB_TRACK(board)
        t_qfn.SetStart(u2_pads["21"].GetPosition())
        t_qfn.SetEnd(u2_pads["8"].GetPosition())
        t_qfn.SetLayer(pcbnew.F_Cu)
        t_qfn.SetWidth(from_mm(0.25))
        t_qfn.SetNetCode(gnd_code)
        board.Add(t_qfn)

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
