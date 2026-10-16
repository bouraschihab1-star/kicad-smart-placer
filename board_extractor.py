"""
=============================================================================
 board_extractor.py — Phase 1 : Extraction des Données Carte
=============================================================================
 Ce module est l'interface directe avec l'API pcbnew de KiCad.
 Il extrait TOUTES les informations nécessaires au placement :
   - Liste des composants avec leurs propriétés physiques
   - Contour de la carte (Edge.Cuts)
   - Trous de montage et zones d'exclusion (Keep-out)
   - Zone de placement autorisée

 IMPORTANT : Ce module est le SEUL à importer pcbnew directement.
 Tous les autres modules travaillent avec les structures de données
 définies dans utils.py, ce qui permet le test unitaire sans KiCad.
=============================================================================
"""

import math
from typing import List, Tuple, Optional, Dict

try:
    import pcbnew
    PCBNEW_AVAILABLE = True
except ImportError:
    PCBNEW_AVAILABLE = False

from .utils import (
    BBox, ComponentData, NetInfo,
    from_mm, to_mm, snap_angle, setup_logger
)

logger = setup_logger("BoardExtractor")


# ---------------------------------------------------------------------------
#  Constantes pour la détection des réseaux de puissance
# ---------------------------------------------------------------------------

# Motifs de noms de nets typiquement associés à la puissance/alimentation
POWER_NET_PATTERNS = [
    'VCC', 'VDD', 'V3V3', 'V5V', 'V12V', 'VBUS', 'VBAT', 'VSYS',
    'GND', 'AGND', 'DGND', 'PGND', 'GNDA', 'GNDD',
    'VIN', 'VOUT', 'VREG', 'VSS', 'AVCC', 'DVCC',
    '+3V3', '+5V', '+12V', '+24V', '+48V',
    '-5V', '-12V', '-15V',
    '3V3', '5V', '12V', '24V', '48V',
]

# Préfixes de référence indiquant des composants de puissance
POWER_COMPONENT_PREFIXES = ('Q', 'D', 'L')

# Préfixes de référence indiquant des connecteurs (placement périphérique)
CONNECTOR_PREFIXES = ('J', 'P')

# Préfixes de référence indiquant des composants passifs (candidats au flip)
PASSIVE_PREFIXES = ('R', 'C', 'L')


def _is_power_net(net_name: str) -> bool:
    """
    Détermine si un nom de réseau correspond à un réseau de puissance.

    Stratégie de détection multi-niveaux :
      1. Correspondance exacte avec les motifs connus (insensible à la casse)
      2. Le nom contient un motif de puissance comme sous-chaîne
      3. Le nom commence par '+' ou '-' suivi de chiffres (convention KiCad)

    Args:
        net_name: Nom du réseau (ex: 'VCC', 'Net-(U1-Pad3)', '+3V3').

    Returns:
        True si le réseau est identifié comme un réseau de puissance.
    """
    if not net_name:
        return False

    upper = net_name.upper().strip()

    # Cas 1 : Correspondance exacte
    if upper in POWER_NET_PATTERNS:
        return True

    # Cas 2 : Le nom contient un motif de puissance
    for pattern in POWER_NET_PATTERNS:
        if pattern in upper:
            return True

    # Cas 3 : Convention +/- suivie de voltage (ex: +5V, -12V)
    if len(upper) >= 2 and upper[0] in ('+', '-') and any(c.isdigit() for c in upper):
        return True

    return False


def _get_ref_prefix(reference: str) -> str:
    """
    Extrait le préfixe alphabétique d'une référence de composant.

    Exemples :
        'U1'  → 'U'
        'C12' → 'C'
        'R1A' → 'R'
        'Q3'  → 'Q'
        'J1'  → 'J'

    L'algorithme parcourt la chaîne caractère par caractère tant qu'il
    rencontre des lettres, puis s'arrête au premier chiffre.

    Args:
        reference: Référence du composant (RefDes).

    Returns:
        Préfixe alphabétique (en majuscules).
    """
    prefix = ""
    for char in reference:
        if char.isalpha():
            prefix += char
        else:
            break
    return prefix.upper()


def _get_footprint_pads(fp) -> list:
    """
    Récupère la liste des pads d'un footprint de manière compatible
    avec toutes les versions de KiCad (KiCad 6, 7, 8, 9, 10).

    En KiCad 8/9/10, fp.Pads() retourne la deque/collection de tous les pads,
    alors que fp.GetPads(aPadNumber) requiert un numéro de pad.
    """
    if hasattr(fp, "Pads"):
        return list(fp.Pads())
    elif hasattr(fp, "GetPads"):
        try:
            return list(fp.GetPads())
        except TypeError:
            pass
    return []


class BoardExtractor:
    """
    Extracteur de données PCB depuis l'API pcbnew.

    Cette classe encapsule toute l'interaction avec KiCad et produit
    des structures de données sérialisables (ComponentData, NetInfo, BBox)
    utilisées par le reste du pipeline d'optimisation.

    Workflow :
        1. extract_components() → Liste[ComponentData]
        2. extract_nets()       → Dict[str, NetInfo]
        3. extract_board_outline() → Liste de segments (contour Edge.Cuts)
        4. extract_keepouts()   → Liste[BBox] (zones interdites)
        5. compute_placement_area() → BBox (zone de placement autorisée)
    """

    def __init__(self, board=None):
        """
        Args:
            board: Objet pcbnew.BOARD. Si None, utilise GetBoard() (carte
                   actuellement ouverte dans l'éditeur KiCad).
        """
        if board is None and PCBNEW_AVAILABLE:
            self.board = pcbnew.GetBoard()
        else:
            self.board = board

        self._components: List[ComponentData] = []
        self._nets: Dict[str, NetInfo] = {}
        self._board_outline_segments: List[Tuple] = []
        self._keepout_zones: List[BBox] = []
        self._mounting_holes: List[BBox] = []
        self._board_bbox: Optional[BBox] = None

    # -------------------------------------------------------------------
    #  Phase 1.1 : Extraction des Composants
    # -------------------------------------------------------------------

    def extract_components(self) -> List[ComponentData]:
        """
        Extrait tous les composants (footprints) de la carte.

        Pour chaque footprint, on récupère :
          - Référence (RefDes) via GetReference()
          - Position (x, y) via GetPosition()
          - Orientation via GetOrientation()
          - Bounding Box via GetBoundingBox()
          - Face (Top/Bottom) via IsFlipped()
          - Liste des pads avec leurs nets associés
          - État verrouillé via IsLocked()

        Les trous de montage (référence commençant par 'MH' ou 'H',
        ou footprint contenant 'MountingHole') sont exclus du placement
        et stockés séparément dans self._mounting_holes.

        Returns:
            Liste de ComponentData pour tous les composants mobiles.
        """
        if not PCBNEW_AVAILABLE or self.board is None:
            logger.error("pcbnew non disponible ou carte non chargée")
            return []

        self._components = []
        self._mounting_holes = []

        for fp in self.board.GetFootprints():
            ref = fp.GetReference()
            prefix = _get_ref_prefix(ref)

            # ---- Extraction de la position ----
            pos = fp.GetPosition()
            x, y = int(pos.x), int(pos.y)

            # ---- Extraction de l'orientation ----
            # KiCad 7+ : GetOrientation() retourne un EDA_ANGLE
            # KiCad 6  : GetOrientation() retourne des décidegrés (int)
            try:
                orientation = fp.GetOrientationDegrees()
            except AttributeError:
                # Fallback KiCad 6 : décidegrés → degrés
                orientation = fp.GetOrientation() / 10.0

            orientation = snap_angle(orientation)

            # ---- Extraction du Courtyard (contour violet) ramené à l'angle 0° ----
            try:
                orig_angle = fp.GetOrientationDegrees()
            except AttributeError:
                orig_angle = fp.GetOrientation() / 10.0

            try:
                fp.SetOrientationDegrees(0.0)
            except AttributeError:
                fp.SetOrientation(0)

            base_pos = fp.GetPosition()

            # Recherche du contour Courtyard officiel (F.CrtYd ou B.CrtYd)
            crt = fp.GetCourtyard(pcbnew.F_CrtYd) if hasattr(fp, 'GetCourtyard') else None
            if not crt or crt.OutlineCount() == 0:
                if hasattr(fp, 'GetCourtyard'):
                    crt = fp.GetCourtyard(pcbnew.B_CrtYd)

            if crt and crt.OutlineCount() > 0:
                raw_bbox = crt.BBox()
            else:
                try:
                    raw_bbox = fp.GetBoundingBox(False, False)
                except TypeError:
                    raw_bbox = fp.GetBoundingBox()

            crt_dx_min = int(raw_bbox.GetX() - base_pos.x)
            crt_dy_min = int(raw_bbox.GetY() - base_pos.y)
            crt_dx_max = int(raw_bbox.GetX() + raw_bbox.GetWidth() - base_pos.x)
            crt_dy_max = int(raw_bbox.GetY() + raw_bbox.GetHeight() - base_pos.y)

            # Restaurer l'angle initial immédiatement
            try:
                fp.SetOrientationDegrees(orig_angle)
            except AttributeError:
                fp.SetOrientation(int(orig_angle * 10))

            # ---- Face de montage ----
            layer = "B.Cu" if fp.IsFlipped() else "F.Cu"

            # ---- Détection des trous de montage ----
            fp_name = fp.GetFPID().GetUniStringLibItemName()
            if (prefix in ('MH', 'H') or
                'MountingHole' in fp_name or
                'mounting' in fp_name.lower()):
                # Stocker comme zone interdite, pas comme composant mobile
                mh_comp = ComponentData(
                    reference=ref,
                    position=(x, y),
                    orientation=orientation,
                    crt_dx_min=crt_dx_min,
                    crt_dy_min=crt_dy_min,
                    crt_dx_max=crt_dx_max,
                    crt_dy_max=crt_dy_max
                )
                self._mounting_holes.append(mh_comp.get_bbox_at(x, y, orientation))
                logger.debug(f"  Trou de montage détecté : {ref}")
                continue

            # ---- Extraction des pads et de leurs nets ----
            pads_data = []
            component_nets = set()
            is_power = False

            for pad in _get_footprint_pads(fp):
                pad_pos = pad.GetPosition()
                net = pad.GetNet()
                net_name = net.GetNetname() if net else ""
                net_code = net.GetNetCode() if net else 0

                pads_data.append({
                    'number': pad.GetNumber(),
                    'x': int(pad_pos.x),
                    'y': int(pad_pos.y),
                    'net_name': net_name,
                    'net_code': net_code,
                })

                if net_name:
                    component_nets.add(net_name)

            if prefix in POWER_COMPONENT_PREFIXES:
                is_power = True

            # ---- Construction de la structure de données ----
            comp = ComponentData(
                reference=ref,
                footprint=fp_name,
                position=(x, y),
                orientation=orientation,
                layer=layer,
                pads=pads_data,
                is_locked=fp.IsLocked(),
                net_classes=component_nets,
                is_power=is_power,
                fp_ref=fp,
                crt_dx_min=crt_dx_min,
                crt_dy_min=crt_dy_min,
                crt_dx_max=crt_dx_max,
                crt_dy_max=crt_dy_max
            )
            comp.bbox = comp.get_bbox_at(x, y, orientation)

            self._components.append(comp)

        logger.info(f"Composants extraits : {len(self._components)} "
                    f"(+ {len(self._mounting_holes)} trous de montage)")

        return self._components

    # -------------------------------------------------------------------
    #  Phase 1.2 : Extraction des Nets (Chevelu)
    # -------------------------------------------------------------------

    def extract_nets(self) -> Dict[str, NetInfo]:
        """
        Extrait la netlist complète de la carte.

        Un Net (réseau) connecte les pastilles de plusieurs composants.
        Pour chaque net, on collecte :
          - Nom et code du réseau
          - Liste des pads connectés avec leurs positions
          - Classification puissance / signal

        La structure résultante est utilisée pour :
          - Calculer le HPWL (Half-Perimeter Wirelength)
          - Déterminer les classes d'équipotence
          - Identifier les composants de puissance

        Returns:
            Dictionnaire {nom_net: NetInfo} avec toutes les connexions.
        """
        if not PCBNEW_AVAILABLE or self.board is None:
            return {}

        self._nets = {}

        # Parcourir tous les footprints et collecter les pads par net
        for fp in self.board.GetFootprints():
            ref = fp.GetReference()
            prefix = _get_ref_prefix(ref)

            # Ignorer les trous de montage
            fp_name = fp.GetFPID().GetUniStringLibItemName()
            if (prefix in ('MH', 'H') or 'MountingHole' in fp_name):
                continue

            for pad in _get_footprint_pads(fp):
                net = pad.GetNet()
                if net is None:
                    continue

                net_name = net.GetNetname()
                net_code = net.GetNetCode()

                # Ignorer le net vide (code 0 = non connecté)
                if net_code == 0 or not net_name:
                    continue

                pad_pos = pad.GetPosition()

                if net_name not in self._nets:
                    self._nets[net_name] = NetInfo(
                        name=net_name,
                        code=net_code,
                        is_power=_is_power_net(net_name)
                    )

                self._nets[net_name].pad_refs.append((
                    ref,
                    pad.GetNumber(),
                    int(pad_pos.x),
                    int(pad_pos.y)
                ))

        # Filtrer les nets à un seul pad (pas d'intérêt pour le HPWL)
        self._nets = {
            name: info for name, info in self._nets.items()
            if len(info.pad_refs) >= 2
        }

        n_power = sum(1 for n in self._nets.values() if n.is_power)
        logger.info(f"Nets extraits : {len(self._nets)} "
                    f"({n_power} puissance, {len(self._nets) - n_power} signal)")

        return self._nets

    # -------------------------------------------------------------------
    #  Phase 1.3 : Extraction du Contour de Carte (Edge.Cuts)
    # -------------------------------------------------------------------

    def extract_board_outline(self) -> List[Tuple]:
        """
        Extrait les segments formant le contour de la carte (couche Edge.Cuts).

        Le contour est défini par des éléments graphiques sur la couche
        Edge.Cuts : segments de droite, arcs, et cercles. On les convertit
        en une liste de segments de droite pour simplifier les calculs
        de distance (Phase 3, verrouillage périphérique).

        Chaque segment est un tuple ((x1, y1), (x2, y2)) en nm.

        Pour les arcs, on les approxime en N segments de droite (N=16).

        Returns:
            Liste de tuples ((x1, y1), (x2, y2)) représentant les segments
            du contour de la carte.
        """
        if not PCBNEW_AVAILABLE or self.board is None:
            return []

        self._board_outline_segments = []

        # Méthode 1 : via GetBoardPolygonOutlines (méthode native officielle KiCad 7/8/9/10)
        try:
            poly = pcbnew.SHAPE_POLY_SET()
            self.board.GetBoardPolygonOutlines(poly, True)
            if poly.OutlineCount() > 0:
                bbox = poly.BBox()
                if bbox.GetWidth() > 0 and bbox.GetHeight() > 0:
                    self._board_bbox = BBox(
                        x_min=int(bbox.GetX()),
                        y_min=int(bbox.GetY()),
                        x_max=int(bbox.GetX() + bbox.GetWidth()),
                        y_max=int(bbox.GetY() + bbox.GetHeight())
                    )
                    for outline_idx in range(poly.OutlineCount()):
                        outline = poly.Outline(outline_idx)
                        pts_count = outline.PointCount()
                        for p_idx in range(pts_count):
                            p1 = outline.CPoint(p_idx)
                            p2 = outline.CPoint((p_idx + 1) % pts_count)
                            self._board_outline_segments.append(
                                ((int(p1.x), int(p1.y)), (int(p2.x), int(p2.y)))
                            )
        except Exception as e:
            logger.debug(f"Polygon outlines non disponible : {e}")

        # Méthode 1b : via les Zones sur Edge.Cuts (très fréquent lorsque le contour est défini par un polygone de zone)
        if not self._board_outline_segments:
            for zone in self.board.Zones():
                try:
                    lname = self.board.GetLayerName(zone.GetLayer())
                    if lname == "Edge.Cuts" or zone.GetLayer() == pcbnew.Edge_Cuts:
                        poly = zone.Outline()
                        if poly and poly.OutlineCount() > 0:
                            bbox = poly.BBox()
                            if bbox.GetWidth() > 0 and bbox.GetHeight() > 0:
                                self._board_bbox = BBox(
                                    x_min=int(bbox.GetX()),
                                    y_min=int(bbox.GetY()),
                                    x_max=int(bbox.GetX() + bbox.GetWidth()),
                                    y_max=int(bbox.GetY() + bbox.GetHeight())
                                )
                                for outline_idx in range(poly.OutlineCount()):
                                    outline = poly.Outline(outline_idx)
                                    pts_count = outline.PointCount()
                                    for p_idx in range(pts_count):
                                        p1 = outline.CPoint(p_idx)
                                        p2 = outline.CPoint((p_idx + 1) % pts_count)
                                        self._board_outline_segments.append(
                                            ((int(p1.x), int(p1.y)), (int(p2.x), int(p2.y)))
                                        )
                except Exception as e:
                    logger.debug(f"Erreur extraction contour zone Edge.Cuts : {e}")

        # Méthode 2 : via GetDrawings (fallback)
        if not self._board_outline_segments:
            drawings_list = []
            if hasattr(self.board, "GetDrawings"):
                drawings_list.extend(list(self.board.GetDrawings()))
            if hasattr(self.board, "Drawings"):
                try:
                    drawings_list.extend(list(self.board.Drawings()))
                except Exception:
                    pass

            for drawing in drawings_list:
                try:
                    lname = self.board.GetLayerName(drawing.GetLayer())
                    if lname != "Edge.Cuts" and drawing.GetLayer() != pcbnew.Edge_Cuts:
                        continue
                    if hasattr(drawing, 'GetStart') and hasattr(drawing, 'GetEnd'):
                        start = drawing.GetStart()
                        end = drawing.GetEnd()
                        segment = (
                            (int(start.x), int(start.y)),
                            (int(end.x), int(end.y))
                        )
                        self._board_outline_segments.append(segment)
                except Exception as e:
                    logger.warning(f"Erreur extraction contour dessin : {e}")

        logger.info(f"Contour carte : {len(self._board_outline_segments)} segments")
        return self._board_outline_segments

    # -------------------------------------------------------------------
    #  Phase 1.4 : Extraction des Zones d'Exclusion (Keep-out)
    # -------------------------------------------------------------------

    def extract_keepouts(self) -> List[BBox]:
        """
        Extrait les zones d'exclusion définies sur la carte.

        Les zones d'exclusion (Rule Areas) dans KiCad interdisent le
        placement de composants dans certaines régions. On les convertit
        en BBox pour les intégrer dans le calcul de collision.

        On combine :
          - Les Rule Areas (zones d'exclusion KiCad)
          - Les trous de montage (détectés en Phase 1.1)

        Returns:
            Liste de BBox représentant les zones interdites.
        """
        if not PCBNEW_AVAILABLE or self.board is None:
            return self._mounting_holes

        self._keepout_zones = []

        for zone in self.board.Zones():
            try:
                # IMPORTANT : Ne jamais traiter le contour Edge.Cuts comme une zone interdite !
                if zone.GetLayer() == pcbnew.Edge_Cuts:
                    continue
                # Seules les zones interdisant expressément les composants (footprints) sont des keepouts
                if hasattr(zone, "GetDoNotAllowFootprints") and not zone.GetDoNotAllowFootprints():
                    continue

                if zone.GetIsRuleArea():
                    outline = zone.Outline()
                    zone_bbox = BBox(
                        x_min=int(outline.BBox().GetX()),
                        y_min=int(outline.BBox().GetY()),
                        x_max=int(outline.BBox().GetX() + outline.BBox().GetWidth()),
                        y_max=int(outline.BBox().GetY() + outline.BBox().GetHeight())
                    )
                    self._keepout_zones.append(zone_bbox)
            except Exception as e:
                logger.warning(f"Erreur extraction keep-out : {e}")

        # Ajouter les trous de montage comme zones interdites
        self._keepout_zones.extend(self._mounting_holes)

        logger.info(f"Zones interdites : {len(self._keepout_zones)} "
                    f"(dont {len(self._mounting_holes)} trous de montage)")

        return self._keepout_zones

    # -------------------------------------------------------------------
    #  Phase 1.5 : Calcul de la Zone de Placement
    # -------------------------------------------------------------------

    def compute_placement_area(self) -> BBox:
        """
        Calcule la zone de placement autorisée.

        Si un contour Edge.Cuts existe, la zone de placement est le
        bounding box de ce contour. Sinon, on utilise le bounding box
        de l'ensemble des composants existants avec une marge minimale.

        Returns:
            BBox définissant la zone de placement autorisée.
        """
        if self._board_bbox is not None and self._board_bbox.width > 0:
            pass
        elif self._board_outline_segments:
            all_x = []
            all_y = []
            for (x1, y1), (x2, y2) in self._board_outline_segments:
                all_x.extend([x1, x2])
                all_y.extend([y1, y2])

            self._board_bbox = BBox(
                x_min=min(all_x),
                y_min=min(all_y),
                x_max=max(all_x),
                y_max=max(all_y)
            )
        elif hasattr(self, "board") and self.board is not None and any(self.board.Zones()):
            # Vérifier si une zone Edge.Cuts ou F.Cu délimite la carte
            for zone in self.board.Zones():
                try:
                    lname = self.board.GetLayerName(zone.GetLayer())
                    if lname in ["Edge.Cuts", "F.Cu"]:
                        bb = zone.GetBoundingBox()
                        if bb.GetWidth() > 0 and bb.GetHeight() > 0:
                            self._board_bbox = BBox(
                                x_min=int(bb.GetX()),
                                y_min=int(bb.GetY()),
                                x_max=int(bb.GetX() + bb.GetWidth()),
                                y_max=int(bb.GetY() + bb.GetHeight())
                            )
                            break
                except Exception:
                    pass

        if self._board_bbox is None or self._board_bbox.width == 0:
            if self._components:
                bboxes = [c.bbox for c in self._components]
                enclosing = BBox(
                    x_min=min(b.x_min for b in bboxes),
                    y_min=min(b.y_min for b in bboxes),
                    x_max=max(b.x_max for b in bboxes),
                    y_max=max(b.y_max for b in bboxes)
                )
                margin = from_mm(5.0)  # Marge fixe de 5mm pour éviter de disperser
                self._board_bbox = enclosing.inflate(margin)
            else:
                half = from_mm(50)
                self._board_bbox = BBox(-half, -half, half, half)

        logger.info(
            f"Zone de placement : {to_mm(self._board_bbox.width):.1f} × "
            f"{to_mm(self._board_bbox.height):.1f} mm"
        )

        return self._board_bbox

    # -------------------------------------------------------------------
    #  Accesseurs pour les données extraites
    # -------------------------------------------------------------------

    @property
    def components(self) -> List[ComponentData]:
        return self._components

    @property
    def nets(self) -> Dict[str, NetInfo]:
        return self._nets

    @property
    def board_outline(self) -> List[Tuple]:
        return self._board_outline_segments

    @property
    def keepouts(self) -> List[BBox]:
        return self._keepout_zones

    @property
    def placement_area(self) -> Optional[BBox]:
        return self._board_bbox
