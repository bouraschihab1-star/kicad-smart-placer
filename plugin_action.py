"""
=============================================================================
 plugin_action.py — Phase 5 : Plugin KiCad Action Plugin
=============================================================================
 Ce fichier implémente l'Action Plugin KiCad qui :
   1. Ajoute un bouton dans l'interface de l'éditeur PCB
   2. Orchestre tout le pipeline de placement automatique
   3. Applique les résultats de l'optimisation à la carte
   4. Gère les conversions d'unités et le rafraîchissement de l'affichage

 ARCHITECTURE DU PLUGIN KICAD :
   KiCad charge les plugins Python depuis le dossier de scripts utilisateur.
   Un Action Plugin doit :
     - Hériter de pcbnew.ActionPlugin
     - Implémenter defaults() pour l'icône et la description
     - Implémenter Run() pour la logique principale
   Le plugin apparaît dans le menu Outils → Scripts d'action externe.

 INSTALLATION :
   Copier le dossier kicad_smart_placer/ dans :
     - Windows : %APPDATA%\KiCad\<version>\scripting\plugins\
     - Linux   : ~/.local/share/kicad/<version>/scripting/plugins/
     - macOS   : ~/Library/Preferences/kicad/<version>/scripting/plugins/

   Ou via le gestionnaire de plugins KiCad (PCM).
=============================================================================
"""

import os
import sys
import traceback
import time
import math
from typing import List, Dict, Tuple, Optional, Any

try:
    import pcbnew
    PCBNEW_AVAILABLE = True
except ImportError:
    PCBNEW_AVAILABLE = False

# Ajouter le dossier parent au path pour les imports relatifs
plugin_dir = os.path.dirname(os.path.abspath(__file__))
if plugin_dir not in sys.path:
    sys.path.insert(0, os.path.dirname(plugin_dir))

from .utils import (
    BBox, ComponentData,
    from_mm, to_mm, snap_angle,
    setup_logger, AnnealingTracker
)
from .board_extractor import BoardExtractor
from .spatial_model import SpatialModel
from .net_analyzer import NetAnalyzer
from .constraints import ConstraintEngine
from .optimizer import SimulatedAnnealing
from .gui_dialog import SmartPlacerDialog, DEFAULT_SETTINGS, prompt_user_settings
from .ground_plane import add_ground_plane, fill_all_zones
from .hierarchical import (
    detect_hierarchical_sheets,
    replicate_sheet_layout,
    create_pcb_groups
)

logger = setup_logger("SmartPlacer")


class SmartPlacerPlugin(pcbnew.ActionPlugin if PCBNEW_AVAILABLE else object):
    """
    Plugin KiCad de placement automatique intelligent.

    Ce plugin utilise un algorithme de recuit simulé pour optimiser
    le placement des composants sur une carte PCB. Il minimise :
      - La longueur totale du chevelu (HPWL)
      - Les collisions entre composants
      - Les violations de contraintes physiques et électriques

    Fonctionnalités :
      ✓ Partitionnement Min-Cut pour le placement initial
      ✓ Recuit simulé avec mouvements adaptatifs
      ✓ Halos de dégagement (Somme de Minkowski)
      ✓ Classes d'équipotence (puissance vs signal)
      ✓ Verrouillage périphérique des connecteurs
      ✓ Groupement rigide (IC + condensateurs de découplage)
      ✓ Flip Top/Bottom automatique si densité haute
      ✓ Répulsion thermique entre composants de puissance
      ✓ Orientation discrète (0°/90°/180°/270°)
      ✓ Respect des zones interdites et trous de montage
    """

    def defaults(self):
        """
        Définit les métadonnées du plugin pour l'interface KiCad.

        Appelé par KiCad lors du chargement du plugin pour :
          - Afficher le nom dans le menu
          - Afficher la description dans la tooltip
          - Assigner une catégorie de menu
          - Définir le chemin de l'icône
        """
        self.name = "Smart Placer & AutoRouter"
        self.category = "Placement & Routing"
        self.description = (
            "Placement et routage automatique intelligent des composants PCB.\n"
            "Optimise le placement par recuit simulé (HPWL + Courtyard De-overlap),\n"
            "redimensionne automatiquement le contour Edge.Cuts,\n"
            "et route 100% nativement les pistes en respectant les NetClasses."
        )
        self.show_toolbar_button = True

        # Icône (optionnel)
        icon_path = os.path.join(plugin_dir, "icon.png")
        if os.path.exists(icon_path):
            self.icon_file_name = icon_path

    def Run(self, board=None):
        """
        Point d'entrée principal du plugin KiCad.
        Affiche l'interface graphique native (wx.Dialog) "Zéro Configuration"
        et délègue l'exécution à un Worker Thread pour supprimer tout gel sous Windows.
        """
        if board is None and PCBNEW_AVAILABLE:
            board = pcbnew.GetBoard()

        try:
            import wx
            if wx.GetApp():
                dlg = SmartPlacerDialog(
                    parent=None,
                    board=board,
                    pipeline_runner=self.run_pipeline
                )
                dlg.ShowModal()
                dlg.Destroy()
                return
        except Exception as e:
            logger.warning(f"Interface graphique non disponible ({e}), exécution directe.")

        # Exécution directe headless / mode batch
        self.run_pipeline(board=board, settings=None, progress_callback=None)

    def run_pipeline(
        self,
        board=None,
        settings: Optional[Dict[str, Any]] = None,
        progress_callback: Optional[object] = None
    ) -> Dict[str, Any]:
        """
        Orchestration complète du pipeline exécutée de manière non-bloquante :
          Phase 1 : Extraction des données de la carte
          Phase 2 : Modélisation spatiale et électrique
          Phase 3 : Application des contraintes métier DFM
          Phase 4 : Optimisation par recuit simulé (placement)
          Phase 5 : Application des résultats à la carte
          Phase 6 : Ajustement automatique du contour Edge.Cuts
          Phase 7 : Routage automatique natif (Smart Router A*)
          Phase 8 : Boucle de compaction réactive post-routage
          Phase 9 : Plan de masse et finalisation
        """
        if settings is None:
            settings = dict(DEFAULT_SETTINGS)

        try:
            logger.info("=" * 70)
            logger.info("  SMART PLACER & AUTOROUTER — Pipeline d'Optimisation")
            logger.info("=" * 70)
            start_time = time.time()

            if progress_callback:
                progress_callback(3, "Initialisation de la carte...")

            if board is None:
                if PCBNEW_AVAILABLE:
                    board = pcbnew.GetBoard()
                else:
                    raise RuntimeError("pcbnew n'est pas disponible et aucune instance de BOARD n'a été fournie.")

            # Extraction des réglages utilisateur / industriels (Mode Extrême par défaut)
            effort_level = settings.get("placement_effort_level", 3)
            if effort_level == 0:
                cooling_rate = 0.92
                max_iterations = 100
            elif effort_level == 1:
                cooling_rate = 0.95
                max_iterations = 250
            elif effort_level == 2:
                cooling_rate = 0.97
                max_iterations = 500
            else:
                cooling_rate = 0.985
                max_iterations = 800

            margin_mm = float(settings.get("margin_edge_cuts_mm", settings.get("margin_mm", 1.0)))
            enable_gnd = bool(settings.get("enable_ground_plane", True))
            gnd_layers = settings.get("ground_plane_layers", ["B.Cu", "F.Cu"])
            gnd_net = settings.get("ground_net_name", "GND")
            do_auto_route = bool(settings.get("auto_route", True))
            do_clean_tracks = bool(settings.get("clean_tracks", True))
            enable_hier = bool(settings.get("enable_hierarchical_groups", True))
            apply_netclasses = bool(settings.get("apply_netclasses", True))
            via_penalty_mm = float(settings.get("via_penalty_mm", 25.0))

            # =============================================================
            #  PHASE 1 : Extraction des données de la carte
            # =============================================================
            logger.info("\n▶ PHASE 1 : Extraction des données de la carte")
            if progress_callback:
                progress_callback(6, "Extraction des composants et des filets...")

            extractor = BoardExtractor(board)
            components = extractor.extract_components()
            if not components:
                logger.error("Aucun composant trouvé sur la carte !")
                return {"success_percent": 0.0, "error": "Aucun composant"}

            sheets = []
            if enable_hier:
                try:
                    sheets = detect_hierarchical_sheets(board, components)
                except Exception as he:
                    logger.warning(f"Erreur détection sous-feuilles : {he}")

            nets = extractor.extract_nets()
            board_outline = extractor.extract_board_outline()
            keepouts = extractor.extract_keepouts()
            placement_area = extractor.compute_placement_area()

            eff_placement_area = placement_area

            logger.info(f"  → {len(components)} composants, {len(nets)} nets, {len(keepouts)} zones interdites")

            # =============================================================
            #  PHASE 2 : Modélisation spatiale et électrique
            # =============================================================
            logger.info("\n▶ PHASE 2 : Modélisation spatiale et électrique")
            if progress_callback:
                progress_callback(12, "Modélisation spatiale et connectivité...")

            spatial_model = SpatialModel()
            spatial_model.set_static_keepouts(keepouts)
            spatial_model.set_placement_area(eff_placement_area)

            net_analyzer = NetAnalyzer(components, nets)
            initial_hpwl = net_analyzer.compute_total_hpwl()
            logger.info(f"  → HPWL initial : {to_mm(initial_hpwl):.2f} mm")

            # =============================================================
            #  PHASE 3 : Configuration des contraintes
            # =============================================================
            logger.info("\n▶ PHASE 3 : Configuration des contraintes métier DFM")
            if progress_callback:
                progress_callback(16, "Configuration des contraintes DFM...")

            constraint_engine = ConstraintEngine(
                board_outline=board_outline,
                placement_area=eff_placement_area
            )
            rigid_groups = net_analyzer.find_rigid_groups()
            constraint_engine.set_rigid_groups(rigid_groups)

            # =============================================================
            #  PHASE 4 : Optimisation par recuit simulé
            # =============================================================
            logger.info("\n▶ PHASE 4 : Optimisation par recuit simulé")
            if progress_callback:
                progress_callback(20, "Démarrage du recuit simulé...")

            optimizer = SimulatedAnnealing(
                components=components,
                net_analyzer=net_analyzer,
                spatial_model=spatial_model,
                constraint_engine=constraint_engine,
                placement_area=eff_placement_area
            )
            optimizer.initialize()

            if enable_hier and sheets and len(sheets) > 1:
                try:
                    replicate_sheet_layout(sheets[0], sheets[1:], optimizer.current_positions, optimizer.current_orientations)
                    logger.info(f"  ✓ Réplication de disposition appliquée sur {len(sheets)-1} sous-feuille(s).")
                except Exception as he:
                    logger.warning(f"Erreur réplication sous-feuilles : {he}")

            best_positions = optimizer.run(
                cooling_rate=cooling_rate,
                max_iterations=max_iterations,
                log_interval=10,
                progress_callback=progress_callback
            )

            # =============================================================
            #  PHASE 5 : Application des résultats à la carte
            # =============================================================
            logger.info("\n▶ PHASE 5 : Application des résultats à la carte")
            if progress_callback:
                progress_callback(50, "Application des positions optimisées...")

            results = optimizer.get_results()
            self._apply_results_to_board(board, results, components)

            # =============================================================
            #  PHASE 6 : Ajustement automatique du contour Edge.Cuts
            # =============================================================
            logger.info("\n▶ PHASE 6 : Ajustement automatique du contour Edge.Cuts")
            if progress_callback:
                progress_callback(53, f"Ajustement contour Edge.Cuts (marge {margin_mm} mm)...")

            new_edge_cuts = self._update_or_create_edge_cuts(
                board=board,
                components=components,
                results=results,
                margin_mm=margin_mm
            )

            # =============================================================
            #  PHASE 7 : Routage automatique natif (Smart Router)
            # =============================================================
            routing_stats = None
            if do_auto_route:
                logger.info("\n▶ PHASE 7 : Routage automatique natif (Smart Router)")
                if progress_callback:
                    progress_callback(55, "Lancement du Smart Router A*...")

                try:
                    from .router import SmartRouter
                    if do_clean_tracks:
                        SmartRouter.clean_board_tracks(board)
                    router = SmartRouter(
                        board,
                        grid_step_mm=0.20,
                        margin_board_mm=margin_mm,
                        via_penalty_mm=via_penalty_mm,
                        ground_plane_enabled=enable_gnd,
                        gnd_net_name=gnd_net
                    )
                    routing_stats = router.route(
                        ground_plane_enabled=enable_gnd,
                        gnd_net_name=gnd_net,
                        progress_callback=progress_callback
                    )
                    logger.info(
                        f"  Routage initial : {routing_stats['success_nets']}/{routing_stats['total_nets']} filets "
                        f"({routing_stats['success_percent']:.1f}%), {routing_stats['tracks']} pistes, "
                        f"{routing_stats['vias']} vias en {routing_stats['time_s']:.2f}s"
                    )

                    # =============================================================
                    #  PHASE 8 : Validation de la Surface Optimale Minimale
                    # =============================================================
                    if routing_stats and routing_stats.get('success_percent') == 100.0 and new_edge_cuts:
                        logger.info("\n▶ PHASE 8 : Validation de la Surface Optimale Minimale")
                        if progress_callback:
                            progress_callback(88, "Validation de la compacite d'aire...")

                        init_w = new_edge_cuts[2] - new_edge_cuts[0]
                        init_h = new_edge_cuts[3] - new_edge_cuts[1]
                        init_area = init_w * init_h
                        logger.info(
                            f"  Surface optimale validee : {init_w:.1f}x{init_h:.1f} mm "
                            f"({init_area:.1f} mm2), 100% route avec {routing_stats['vias']} vias."
                        )

                except Exception as re:
                    logger.error(f"Erreur durant le routage automatique : {re}", exc_info=True)

            # =============================================================
            #  PHASE 8.5 : Groupement natif KiCad PCB_GROUP
            # =============================================================
            if enable_hier and sheets:
                logger.info(f"\n▶ PHASE 8.5 : Groupement natif KiCad PCB_GROUP ({len(sheets)} sous-feuilles)")
                if progress_callback:
                    progress_callback(93, "Création des groupes PCB_GROUP natifs...")
                try:
                    create_pcb_groups(board, sheets)
                except Exception as ge:
                    logger.warning(f"Erreur création groupes PCB_GROUP : {ge}")

            # =============================================================
            #  PHASE 9 : Génération du Plan de Masse & Push and Shove
            # =============================================================
            if enable_gnd:
                logger.info(f"\n▶ PHASE 9 : Optimisation Push & Shove et Génération du plan de masse cuivre ({gnd_net}) sur {gnd_layers}")
                if progress_callback:
                    progress_callback(95, "Push & Shove : Élargissement des couloirs de masse (>= 0.50 mm)...")
                try:
                    from .router import Push_And_Shove_GND_Corridors
                    layer_ids = [pcbnew.F_Cu if l == 'F.Cu' else pcbnew.B_Cu for l in gnd_layers]
                    shoved_cnt = Push_And_Shove_GND_Corridors(board, target_layers=layer_ids, gnd_net_name=gnd_net, min_corridor_mm=0.50)
                    if shoved_cnt > 0:
                        logger.info(f"  ✓ Push & Shove final : {shoved_cnt} segment(s) décalé(s) pour ouvrir les couloirs GND >= 0.50 mm.")
                except Exception as pse:
                    logger.warning(f"Avertissement Push & Shove GND : {pse}")

                if progress_callback:
                    progress_callback(97, f"Génération plan de masse ({gnd_net}) sur {gnd_layers}...")
                try:
                    add_ground_plane(board, layers=gnd_layers, net_name=gnd_net)
                except Exception as gnd_err:
                    logger.warning(f"Erreur lors de la création du plan de masse : {gnd_err}")

            elapsed = time.time() - start_time
            final_hpwl = net_analyzer.compute_total_hpwl(best_positions)
            improvement = (1 - final_hpwl / initial_hpwl) * 100 if initial_hpwl > 0 else 0

            logger.info("\n" + "=" * 70)
            logger.info("  PLACEMENT & ROUTAGE TERMINÉS AVEC SUCCÈS")
            logger.info(f"  Temps total     : {elapsed:.1f}s")
            logger.info(f"  HPWL initial    : {to_mm(initial_hpwl):.2f} mm")
            logger.info(f"  HPWL final      : {to_mm(final_hpwl):.2f} mm")
            logger.info(f"  Amélioration    : {improvement:.1f}%")
            if routing_stats:
                logger.info(f"  Routage filets  : {routing_stats['success_nets']}/{routing_stats['total_nets']} ({routing_stats['success_percent']:.1f}%)")
                logger.info(f"  Pistes / Vias   : {routing_stats['tracks']} pistes, {routing_stats['vias']} vias")
            logger.info("=" * 70)

            if progress_callback:
                progress_callback(100, "Placement & Routage terminés avec succès !")

            return routing_stats or {"success_percent": 100.0, "time_s": round(elapsed, 2)}

        except Exception as e:
            logger.error(f"ERREUR : {str(e)}")
            logger.error(traceback.format_exc())
            raise e

    # -------------------------------------------------------------------
    #  APPLICATION DES RÉSULTATS À LA CARTE
    # -------------------------------------------------------------------

    def _apply_results_to_board(
        self,
        board,
        results: dict,
        components: list
    ):
        """
        Applique les positions, orientations et faces optimisées à la carte.

        Pour chaque composant modifié :
          1. Mettre à jour la position via SetPosition()
          2. Mettre à jour l'orientation via SetOrientationDegrees()
          3. Si flip nécessaire, appeler Flip()
          4. Logger le changement

        GESTION DES UNITÉS :
            Les positions sont déjà en nanomètres (unités internes KiCad).
            Aucune conversion n'est nécessaire pour SetPosition().
            Les orientations sont en degrés.

        Args:
            board: Objet pcbnew.BOARD.
            results: Dict {ref: {position, orientation, layer, ...}}.
            components: Liste des ComponentData originaux.
        """
        moved_count = 0

        for comp in components:
            ref = comp.reference
            if ref not in results:
                continue

            result = results[ref]
            if not result.get('was_moved', False):
                continue

            fp = None
            if hasattr(board, 'FindFootprintByReference'):
                try:
                    fp = board.FindFootprintByReference(ref)
                except Exception:
                    pass
            if fp is None:
                fp = comp.fp_ref

            if fp is None:
                logger.warning(f"  {ref} : référence footprint manquante, ignoré")
                continue

            if comp.is_locked:
                logger.debug(f"  {ref} : verrouillé, ignoré")
                continue

            # 1. Appliquer la nouvelle position
            new_x, new_y = result['position']
            try:
                fp.SetPosition(pcbnew.VECTOR2I(int(new_x), int(new_y)))
            except TypeError:
                # Fallback pour les anciennes versions de KiCad
                fp.SetPosition(pcbnew.wxPoint(int(new_x), int(new_y)))

            # 2. Appliquer la nouvelle orientation
            new_orient = result['orientation']
            try:
                fp.SetOrientationDegrees(new_orient)
            except AttributeError:
                # Fallback KiCad 6 : décidegrés
                fp.SetOrientation(int(new_orient * 10))

            # 3. Forcer la couche F.Cu (Top) : si le composant est sur B.Cu (flipped), le remettre sur F.Cu !
            new_layer = result.get('layer', 'F.Cu')
            if fp.IsFlipped():
                try:
                    center = fp.GetPosition()
                    fp.Flip(center, False)
                    logger.info(f"  {ref} : remis sur F.Cu (Top layer)")
                except Exception as e:
                    logger.warning(f"  {ref} : erreur remise sur F.Cu : {e}")

            moved_count += 1
            old_x_mm = to_mm(result['original_position'][0])
            old_y_mm = to_mm(result['original_position'][1])
            new_x_mm = to_mm(new_x)
            new_y_mm = to_mm(new_y)

            logger.info(
                f"  {ref:8s} : ({old_x_mm:7.2f}, {old_y_mm:7.2f}) → "
                f"({new_x_mm:7.2f}, {new_y_mm:7.2f}) mm, "
                f"θ={new_orient:.0f}°, face={new_layer}"
            )

        logger.info(f"  → {moved_count} composants déplacés")

    def _update_or_create_edge_cuts(
        self,
        board,
        components: List[ComponentData],
        results: Dict[str, dict],
        margin_mm: float = 2.5
    ) -> Optional[Tuple[float, float, float, float]]:
        """
        Crée ou ajuste un contour Edge.Cuts rectangulaire, propre et parfaitement
        dimensionné autour des composants placés.

        Args:
            board: Instance BOARD KiCad.
            components: Liste des ComponentData.
            results: Résultats du placement {ref: {position, orientation, ...}}.
            margin_mm: Marge de sécurité entre les composants et le bord de carte (en mm).

        Returns:
            Tuple (x1_mm, y1_mm, x2_mm, y2_mm) du nouveau rectangle Edge.Cuts, ou None.
        """
        if not PCBNEW_AVAILABLE or board is None:
            return None

        # 1. Calculer la bounding box englobante de tous les composants placés
        x_mins, x_maxs, y_mins, y_maxs = [], [], [], []
        for comp in components:
            ref = comp.reference
            res = results.get(ref)
            if not res:
                continue
            pos = res['position']
            orient = res['orientation']
            bb = comp.get_bbox_at(pos[0], pos[1], orient)
            x_mins.append(bb.x_min)
            x_maxs.append(bb.x_max)
            y_mins.append(bb.y_min)
            y_maxs.append(bb.y_max)

        if not x_mins:
            return None

        min_x, max_x = min(x_mins), max(x_maxs)
        min_y, max_y = min(y_mins), max(y_maxs)
        w_comp = max_x - min_x
        h_comp = max_y - min_y

        # 2. Dimensions avec marge de sécurité, au dixième de millimètre pour compacité maximale
        margin = from_mm(margin_mm)
        board_w_mm = round(to_mm(w_comp + 2 * margin), 1)
        board_h_mm = round(to_mm(h_comp + 2 * margin), 1)
        board_w = from_mm(board_w_mm)
        board_h = from_mm(board_h_mm)

        # 3. Centrage parfait du rectangle autour des composants
        cx = (min_x + max_x) / 2.0
        cy = (min_y + max_y) / 2.0
        x1 = int(cx - board_w / 2.0)
        x2 = int(cx + board_w / 2.0)
        y1 = int(cy - board_h / 2.0)
        y2 = int(cy + board_h / 2.0)

        # 4. Nettoyage des anciens tracés sur Edge.Cuts
        drawings_to_remove = []
        if hasattr(board, "GetDrawings"):
            for d in board.GetDrawings():
                try:
                    lname = board.GetLayerName(d.GetLayer())
                    if lname == "Edge.Cuts" or d.GetLayer() == pcbnew.Edge_Cuts:
                        drawings_to_remove.append(d)
                except Exception:
                    pass
        for d in drawings_to_remove:
            try:
                board.Remove(d)
            except Exception:
                pass

        zones_to_remove = []
        if hasattr(board, "Zones"):
            for z in board.Zones():
                try:
                    lname = board.GetLayerName(z.GetLayer())
                    if lname == "Edge.Cuts" or z.GetLayer() == pcbnew.Edge_Cuts:
                        zones_to_remove.append(z)
                except Exception:
                    pass
        for z in zones_to_remove:
            try:
                board.Remove(z)
            except Exception:
                pass

        # 5. Création des 4 segments fermés sur Edge.Cuts
        p1 = pcbnew.VECTOR2I(x1, y1)
        p2 = pcbnew.VECTOR2I(x2, y1)
        p3 = pcbnew.VECTOR2I(x2, y2)
        p4 = pcbnew.VECTOR2I(x1, y2)

        line_width = from_mm(0.15)  # 0.15 mm : largeur de trait standard KiCad Edge.Cuts

        for start_pt, end_pt in [(p1, p2), (p2, p3), (p3, p4), (p4, p1)]:
            try:
                seg = pcbnew.PCB_SHAPE(board)
                seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
                seg.SetLayer(pcbnew.Edge_Cuts)
                seg.SetStart(start_pt)
                seg.SetEnd(end_pt)
                seg.SetWidth(int(line_width))
                board.Add(seg)
            except Exception as e:
                logger.warning(f"Erreur création segment Edge.Cuts : {e}")

        logger.info(
            f"  ✓ Contour Edge.Cuts ajusté : {board_w_mm:.1f} × {board_h_mm:.1f} mm "
            f"(Marge: {margin_mm:.1f} mm, BBox: [{to_mm(x1):.2f}, {to_mm(y1):.2f}] → [{to_mm(x2):.2f}, {to_mm(y2):.2f}])"
        )

        return to_mm(x1), to_mm(y1), to_mm(x2), to_mm(y2)
