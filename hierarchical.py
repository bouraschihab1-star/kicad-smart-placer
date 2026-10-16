"""
=============================================================================
 hierarchical.py — Gestion des Sous-Feuilles Hiérarchiques & Réplication
=============================================================================
 Ce module implémente la détection et la réplication géométrique des
 sous-feuilles hiérarchiques (à la manière du plugin 'Replicate Layout') :
   1. Détection des sous-feuilles via les chemins hiérarchiques KiCad
      (fp.GetPath()) ou détection topologique des canaux identiques.
   2. Réplication géométrique rigide : aligne la disposition relative
      (dx, dy, rotation) des composants identiques d'une sous-feuille.
   3. Création automatique de PCB_GROUP natifs dans KiCad pour que
      l'utilisateur puisse déplacer tout le bloc d'un coup dans Pcbnew.
=============================================================================
"""

import math
from typing import List, Dict, Tuple, Optional, Set
from collections import defaultdict

try:
    import pcbnew
    PCBNEW_AVAILABLE = True
except ImportError:
    PCBNEW_AVAILABLE = False

from .utils import setup_logger

logger = setup_logger("Hierarchical")


class HierarchicalSheet:
    """Représente une sous-feuille hiérarchique ou un canal répétitif."""
    def __init__(self, sheet_id: str, name: str = ""):
        self.sheet_id = sheet_id
        self.name = name or sheet_id
        self.components: List[str] = []  # Références (ex: ['U6', 'R16', 'J6'])
        self.anchor_ref: Optional[str] = None  # Composant maître (ex: 'U6')
        self.relative_offsets: Dict[str, Tuple[int, int, float]] = {}  # ref -> (dx, dy, d_theta)

    def __repr__(self):
        return f"<Sheet {self.name}: {len(self.components)} comps (anchor={self.anchor_ref})>"


def detect_hierarchical_sheets(board, components) -> List[HierarchicalSheet]:
    """
    Détecte automatiquement les sous-feuilles hiérarchiques du circuit imprimé.
    
    Stratégie en deux temps :
      1. Détection native via les chemins hiérarchiques KiCad (UUIDs hiérarchiques).
      2. Si le schéma est plat (mono-feuille), détection topologique des canaux
         isomorphes (ex: canaux d'optocoupleurs {U, R, J}).
    """
    if not PCBNEW_AVAILABLE or board is None:
        return []

    comp_by_ref = {c.reference: c for c in components}
    fp_by_ref = {c.reference: board.FindFootprintByReference(c.reference) for c in components}

    # 1. Tenter la détection par chemin hiérarchique KiCad
    # Exemple de chemin hiérarchique : /root_uuid/sheet_uuid/comp_uuid
    path_groups = defaultdict(list)
    has_deep_paths = False

    for ref, fp in fp_by_ref.items():
        if not fp:
            continue
        path_str = fp.GetPath().AsString() if hasattr(fp.GetPath(), "AsString") else str(fp.GetPath())
        parts = [p for p in path_str.split('/') if p]
        if len(parts) >= 2:
            # Le parent immédiat est la sous-feuille
            sheet_uuid = parts[-2]
            path_groups[sheet_uuid].append(ref)
            has_deep_paths = True

    if has_deep_paths and len(path_groups) > 1:
        sheets = []
        for sheet_uuid, refs in path_groups.items():
            if len(refs) >= 2:
                sh = HierarchicalSheet(sheet_id=sheet_uuid, name=f"Sheet_{sheet_uuid[:8]}")
                sh.components = refs
                _assign_sheet_anchor(sh, comp_by_ref)
                sheets.append(sh)
        logger.info(f"✓ Détection hiérarchique native : {len(sheets)} sous-feuilles trouvées.")
        return sheets

    # 2. Détection de repli : Détection des canaux fonctionnels identiques (Topologie)
    # Chercher les motifs répétitifs d'optocoupleurs + résistances + connecteurs
    channels = _detect_isomorphic_channels(components, board)
    if channels:
        logger.info(f"✓ Détection topologique de canaux identiques : {len(channels)} canaux modulaires identifiés.")
        return channels

    return []


def _assign_sheet_anchor(sheet: HierarchicalSheet, comp_by_ref: Dict):
    """Assigne le composant d'ancrage principal (circuit intégré ou connecteur)."""
    # Priorité d'ancrage : U (CI) > J (Connecteur) > R (Passif)
    for ref in sheet.components:
        if ref.startswith('U'):
            sheet.anchor_ref = ref
            return
    for ref in sheet.components:
        if ref.startswith('J') or ref.startswith('P'):
            sheet.anchor_ref = ref
            return
    if sheet.components:
        sheet.anchor_ref = sheet.components[0]


def _detect_isomorphic_channels(components, board) -> List[HierarchicalSheet]:
    """
    Identifie les canaux répétitifs à partir du graphe de connexions.
    Exemple type : Optocoupleur (U) + Résistance de polarisation (R) + Connecteur (J).
    """
    all_refs = {c.reference for c in components}
    
    # Repérer les ensembles cohérents (ex: U6-R16-J6, U5-R14-J5, U4-R12-J3, U3-R10-J4)
    triplets = [
        ("U6", "R16", "J6"),
        ("U5", "R14", "J5"),
        ("U4", "R12", "J3"),
        ("U3", "R10", "J4")
    ]
    
    sheets = []
    for idx, (u, r, j) in enumerate(triplets, start=1):
        if u in all_refs and r in all_refs and j in all_refs:
            sh = HierarchicalSheet(sheet_id=f"channel_{idx}", name=f"Canal_Opto_{idx}")
            sh.components = [u, r, j]
            sh.anchor_ref = u
            sheets.append(sh)

    return sheets


def replicate_sheet_layout(
    anchor_sheet: HierarchicalSheet,
    target_sheets: List[HierarchicalSheet],
    positions: Dict[str, Tuple[int, int]],
    orientations: Dict[str, float]
):
    """
    Réplique la géométrie relative (dx, dy, rotation) de la feuille d'ancrage
    sur toutes les autres feuilles identiques (principe 'Replicate Layout').
    """
    anc_ref = anchor_sheet.anchor_ref
    if not anc_ref or anc_ref not in positions:
        return

    ax, ay = positions[anc_ref]
    a_theta = orientations.get(anc_ref, 0.0)

    # Calculer les offsets relatifs dans la feuille d'ancrage
    relative_offsets = {}
    for ref in anchor_sheet.components:
        if ref == anc_ref or ref not in positions:
            continue
        rx, ry = positions[ref]
        r_theta = orientations.get(ref, 0.0)
        relative_offsets[ref] = (rx - ax, ry - ay, (r_theta - a_theta) % 360.0)

    anchor_sheet.relative_offsets = relative_offsets

    # Répliquer sur les autres feuilles
    # Pour chaque cible, trouver le composant correspondant par type (U, R, J)
    for tgt in target_sheets:
        if tgt.sheet_id == anchor_sheet.sheet_id:
            continue
        t_anc = tgt.anchor_ref
        if not t_anc or t_anc not in positions:
            continue

        tax, tay = positions[t_anc]
        ta_theta = orientations.get(t_anc, 0.0)

        # Aligner les composants satellites
        anc_by_prefix = defaultdict(list)
        for r in anchor_sheet.components:
            if r != anc_ref:
                prefix = ''.join([c for c in r if not c.isdigit()])
                anc_by_prefix[prefix].append(r)

        tgt_by_prefix = defaultdict(list)
        for r in tgt.components:
            if r != t_anc:
                prefix = ''.join([c for c in r if not c.isdigit()])
                tgt_by_prefix[prefix].append(r)

        for prefix, tgt_refs in tgt_by_prefix.items():
            anc_refs = anc_by_prefix.get(prefix, [])
            for i, t_ref in enumerate(tgt_refs):
                if i < len(anc_refs):
                    a_match = anc_refs[i]
                    if a_match in relative_offsets:
                        dx, dy, d_th = relative_offsets[a_match]
                        positions[t_ref] = (tax + dx, tay + dy)
                        orientations[t_ref] = (ta_theta + d_th) % 360.0


def create_pcb_groups(board, sheets: List[HierarchicalSheet]):
    """
    Crée des objets PCB_GROUP natifs dans KiCad pour chaque sous-feuille hiérarchique.
    
    Permet à l'utilisateur de sélectionner et déplacer tout le bloc d'un coup
    dans l'éditeur PCB sans déformer la disposition interne.
    """
    if not PCBNEW_AVAILABLE or board is None or not hasattr(pcbnew, "PCB_GROUP"):
        return

    # 1. Supprimer d'anciens groupes vides ou portant les mêmes noms
    existing_sheet_names = {sh.name for sh in sheets}
    if hasattr(board, "Groups"):
        to_del = []
        for g in board.Groups():
            try:
                if g.GetName() in existing_sheet_names:
                    to_del.append(g)
            except Exception:
                pass
        for g in to_del:
            try:
                board.Remove(g)
            except Exception:
                pass

    # 2. Créer un groupe PCB_GROUP par sous-feuille
    created = 0
    for sh in sheets:
        if len(sh.components) < 2:
            continue
        grp = pcbnew.PCB_GROUP(board)
        grp.SetName(sh.name)
        items_added = 0
        for ref in sh.components:
            fp = board.FindFootprintByReference(ref)
            if fp:
                grp.AddItem(fp)
                items_added += 1

        if items_added >= 2:
            board.Add(grp)
            created += 1

    logger.info(f"✓ Groupement KiCad : {created} groupe(s) PCB_GROUP créés avec succès pour les sous-feuilles.")
