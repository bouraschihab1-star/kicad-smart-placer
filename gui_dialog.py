"""
=============================================================================
 gui_dialog.py — Interface Utilisateur Professionnelle Électronique (wxPython)
=============================================================================
 Interface 100% "Zéro Configuration" ergonomique et épurée :
   - Thème sombre industriel "Pro Élec" (évite l'éblouissement sur fond blanc)
   - Menus déroulants (wx.Choice) concis : B.Cu, F.Cu, F.Cu + B.Cu, etc.
   - Champ de saisie direct pour la marge Edge.Cuts (1.0 mm par défaut)
   - Paramètres superflus supprimés (règles DFM strictes et Netclasses auto)
   - Plan de masse sur F.Cu par défaut
   - Statut dynamique affiché uniquement après le clic sur "Commencer"
   - Architecture Anti-Freeze / Non-bloquante avec Worker Thread et wx.CallAfter
=============================================================================
"""

import sys
import os
import threading
import traceback
from typing import Optional, Dict, Any, Callable

try:
    import wx
    WX_AVAILABLE = True
except ImportError:
    WX_AVAILABLE = False

try:
    import pcbnew
    PCBNEW_AVAILABLE = True
except ImportError:
    PCBNEW_AVAILABLE = False

from .utils import setup_logger

logger = setup_logger("SmartPlacerGUI")


# =============================================================================
#  VALEURS PAR DÉFAUT INDUSTRIELLES
# =============================================================================
DEFAULT_SETTINGS: Dict[str, Any] = {
    # Onglet 1 : Placement (Mode Extrême par défaut)
    "placement_effort": "Extrême",
    "placement_effort_level": 3,
    "placement_grid": "0.5 mm",
    "placement_grid_mm": 0.5,
    "allow_bcu": False,
    "allow_rotations_45": True,

    # Onglet 2 : Fabrication & Bords
    "margin_edge_cuts_mm": 1.0,
    "forbid_routing_between_smd2_pads": True,
    "forbid_via_in_pad": True,
    "halo_clearance_multiplier": 1.0,

    # Onglet 3 : Routage Automatique
    "apply_netclasses": True,
    "routing_strategy": "Optimal",
    "routing_strategy_bias": 50,
    "force_orthogonal_layers": True,

    # Onglet 4 : Plan de Masse (GND)
    "enable_ground_plane": True,
    "ground_plane_target": "F.Cu",               # F.Cu par défaut !
    "ground_plane_layers": ["F.Cu"],             # F.Cu par défaut !
    "ground_net_name": "GND"
}


if WX_AVAILABLE:

    # -------------------------------------------------------------------------
    #  Couleurs du Bandeau d'En-tête (évite l'éblouissement du fond blanc)
    # -------------------------------------------------------------------------
    HEADER_BG_COLOR   = wx.Colour(30, 41, 59)     # Slate foncé élégant (#1E293B)
    HEADER_TITLE_COLOR= wx.Colour(255, 255, 255)  # Blanc net
    HEADER_SUB_COLOR  = wx.Colour(148, 163, 184)  # Gris ardoise clair (#94A3B8)
    COLOR_SUCCESS     = wx.Colour(22, 101, 52)    # Vert sombre lisible sur fond standard

    # -------------------------------------------------------------------------
    #  Onglet 1 : Placement — Menus déroulants concis (Thème natif KiCad)
    # -------------------------------------------------------------------------
    class PlacementTabPanel(wx.Panel):
        GRID_CHOICES = ["0.1 mm", "0.25 mm", "0.5 mm", "1.0 mm", "2.54 mm"]
        LAYER_CHOICES = ["F.Cu", "F.Cu + B.Cu"]

        def __init__(self, parent):
            super().__init__(parent)
            self._init_ui()

        def _init_ui(self):
            sizer = wx.BoxSizer(wx.VERTICAL)

            # 2. Grille de placement
            box_grid = wx.StaticBox(self, label="Grille")
            sz_grid = wx.StaticBoxSizer(box_grid, wx.HORIZONTAL)

            lbl_grid = wx.StaticText(box_grid, label="Pas de grille :")
            sz_grid.Add(lbl_grid, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)

            self.choice_grid = wx.Choice(box_grid, choices=self.GRID_CHOICES)
            self.choice_grid.SetSelection(2)  # "0.5 mm" par défaut
            sz_grid.Add(self.choice_grid, 1, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)
            sizer.Add(sz_grid, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)

            # 3. Couche composants
            box_layer = wx.StaticBox(self, label="Couche composants")
            sz_layer = wx.StaticBoxSizer(box_layer, wx.HORIZONTAL)

            lbl_layer = wx.StaticText(box_layer, label="Face(s) :")
            sz_layer.Add(lbl_layer, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)

            self.choice_layer = wx.Choice(box_layer, choices=self.LAYER_CHOICES)
            self.choice_layer.SetSelection(0)  # "F.Cu" par défaut
            sz_layer.Add(self.choice_layer, 1, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)
            sizer.Add(sz_layer, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)
            self.SetSizer(sizer)

        def get_values(self) -> Dict[str, Any]:
            effort_str = "Extrême"
            effort_idx = 3

            grid_str = self.choice_grid.GetStringSelection()
            try:
                grid_val = float(grid_str.replace(" mm", ""))
            except ValueError:
                grid_val = 0.5

            allow_bcu = (self.choice_layer.GetSelection() == 1)

            return {
                "placement_effort": effort_str,
                "placement_effort_level": effort_idx,
                "placement_grid": grid_str,
                "placement_grid_mm": grid_val,
                "allow_bcu": allow_bcu,
                "allow_rotations_45": True,
            }

    # -------------------------------------------------------------------------
    #  Onglet 2 : Fabrication & Bords — Case de saisie & isolation
    # -------------------------------------------------------------------------
    class DFMTabPanel(wx.Panel):
        HALO_CHOICES = ["1.0x", "1.2x", "1.5x", "2.0x"]

        def __init__(self, parent):
            super().__init__(parent)
            self._init_ui()

        def _init_ui(self):
            sizer = wx.BoxSizer(wx.VERTICAL)

            # 1. Marge Edge.Cuts (Case de saisie libre)
            box_margin = wx.StaticBox(self, label="Frontière PCB (Edge.Cuts)")
            sz_margin = wx.StaticBoxSizer(box_margin, wx.HORIZONTAL)

            lbl_m = wx.StaticText(box_margin, label="Marge Edge.Cuts (mm) :")
            sz_margin.Add(lbl_m, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)

            self.txt_margin = wx.TextCtrl(box_margin, value="1.0")  # "1.0" mm par défaut
            self.txt_margin.SetToolTip("Distance minimale en millimètres entre les composants et le bord de la carte.")
            sz_margin.Add(self.txt_margin, 1, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)
            sizer.Add(sz_margin, 0, wx.EXPAND | wx.ALL, 6)

            # 2. Multiplicateur de Halo d'isolation
            box_halo = wx.StaticBox(self, label="Isolation pistes & vias")
            sz_halo = wx.StaticBoxSizer(box_halo, wx.HORIZONTAL)

            lbl_halo = wx.StaticText(box_halo, label="Facteur d'isolation :")
            sz_halo.Add(lbl_halo, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)

            self.choice_halo = wx.Choice(box_halo, choices=self.HALO_CHOICES)
            self.choice_halo.SetSelection(0)  # "1.0x" par défaut
            sz_halo.Add(self.choice_halo, 1, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)
            sizer.Add(sz_halo, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)

            self.SetSizer(sizer)

        def get_values(self) -> Dict[str, Any]:
            txt = self.txt_margin.GetValue().strip().replace(",", ".")
            try:
                margin = float(txt)
            except ValueError:
                margin = 1.0

            halo_map = [1.0, 1.2, 1.5, 2.0]
            halo_idx = self.choice_halo.GetSelection()
            halo_mult = halo_map[halo_idx] if 0 <= halo_idx < len(halo_map) else 1.0

            return {
                "margin_edge_cuts_mm": max(0.1, margin),
                "forbid_routing_between_smd2_pads": True,  # Règle industrielle automatique
                "forbid_via_in_pad": True,                 # Règle industrielle automatique
                "halo_clearance_multiplier": halo_mult,
            }

    # -------------------------------------------------------------------------
    #  Onglet 3 : Routage Automatique — Topologie & Normes
    # -------------------------------------------------------------------------
    class RouterTabPanel(wx.Panel):
        ORTHO_CHOICES = ["Orthogonale (H: F.Cu / V: B.Cu)", "Libre"]

        def __init__(self, parent):
            super().__init__(parent)
            self._init_ui()

        def _init_ui(self):
            sizer = wx.BoxSizer(wx.VERTICAL)

            box_ortho = wx.StaticBox(self, label="Topologie des pistes")
            sz_ortho = wx.StaticBoxSizer(box_ortho, wx.HORIZONTAL)

            lbl_ortho = wx.StaticText(box_ortho, label="Orientation :")
            sz_ortho.Add(lbl_ortho, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)

            self.choice_ortho = wx.Choice(box_ortho, choices=self.ORTHO_CHOICES)
            self.choice_ortho.SetSelection(0)  # Orthogonale par défaut
            sz_ortho.Add(self.choice_ortho, 1, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)
            sizer.Add(sz_ortho, 0, wx.EXPAND | wx.ALL, 6)

            self.SetSizer(sizer)

        def get_values(self) -> Dict[str, Any]:
            force_ortho = (self.choice_ortho.GetSelection() == 0)

            return {
                "apply_netclasses": True,        # Respect strict des largeurs KiCad
                "routing_strategy": "Optimal",   # Équilibre optimal automatique longueur/vias
                "routing_strategy_bias": 50,
                "force_orthogonal_layers": force_ortho,
            }

    # -------------------------------------------------------------------------
    #  Onglet 4 : Plan de Masse (GND) — F.Cu par défaut, libellés concis
    # -------------------------------------------------------------------------
    class GNDTabPanel(wx.Panel):
        TARGET_CHOICES = ["F.Cu", "B.Cu", "F.Cu + B.Cu"]
        THERMAL_CHOICES = ["Freins thermiques", "Plein cuivre"]

        def __init__(self, parent):
            super().__init__(parent)
            self._init_ui()

        def _init_ui(self):
            sizer = wx.BoxSizer(wx.VERTICAL)

            box_gnd = wx.StaticBox(self, label="Plan de masse (GND)")
            sz_gnd = wx.StaticBoxSizer(box_gnd, wx.VERTICAL)

            # 1. Case à cocher : Coché par défaut !
            self.cb_enable_gnd = wx.CheckBox(box_gnd, label="Générer un plan de masse automatique")
            self.cb_enable_gnd.SetValue(True)  # Coché par défaut !
            self.cb_enable_gnd.Bind(wx.EVT_CHECKBOX, self._on_toggle_gnd)
            sz_gnd.Add(self.cb_enable_gnd, 0, wx.ALL, 8)

            # 2. Couche cible : F.Cu par défaut !
            row_layers = wx.BoxSizer(wx.HORIZONTAL)
            self.lbl_layers = wx.StaticText(box_gnd, label="Couche cible :")
            row_layers.Add(self.lbl_layers, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)

            self.choice_layers = wx.Choice(box_gnd, choices=self.TARGET_CHOICES)
            self.choice_layers.SetSelection(0)  # "F.Cu" par défaut !
            row_layers.Add(self.choice_layers, 1, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)
            sz_gnd.Add(row_layers, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)

            # 3. Raccordement thermique
            row_thermal = wx.BoxSizer(wx.HORIZONTAL)
            self.lbl_thermal = wx.StaticText(box_gnd, label="Pastilles GND :")
            row_thermal.Add(self.lbl_thermal, 0, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)

            self.choice_thermal = wx.Choice(box_gnd, choices=self.THERMAL_CHOICES)
            self.choice_thermal.SetSelection(0)  # Freins thermiques par défaut
            row_thermal.Add(self.choice_thermal, 1, wx.ALIGN_CENTER_VERTICAL | wx.ALL, 6)
            sz_gnd.Add(row_thermal, 0, wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 6)

            self._update_state()
            sizer.Add(sz_gnd, 0, wx.EXPAND | wx.ALL, 6)
            self.SetSizer(sizer)

        def _on_toggle_gnd(self, event):
            self._update_state()

        def _update_state(self):
            enabled = self.cb_enable_gnd.GetValue()
            self.lbl_layers.Enable(enabled)
            self.choice_layers.Enable(enabled)
            self.lbl_thermal.Enable(enabled)
            self.choice_thermal.Enable(enabled)

        def get_values(self) -> Dict[str, Any]:
            enable_gnd = self.cb_enable_gnd.GetValue()
            sel = self.choice_layers.GetSelection()
            if sel == 0:
                layers = ["F.Cu"]
                target_str = "F.Cu"
            elif sel == 1:
                layers = ["B.Cu"]
                target_str = "B.Cu"
            else:
                layers = ["F.Cu", "B.Cu"]
                target_str = "F.Cu + B.Cu"

            return {
                "enable_ground_plane": enable_gnd,
                "ground_plane_target": target_str,
                "ground_plane_layers": layers,
                "ground_net_name": "GND"
            }

    # -------------------------------------------------------------------------
    #  Boîte de dialogue Principale (wx.Dialog)
    # -------------------------------------------------------------------------
    class SmartPlacerDialog(wx.Dialog):
        """
        Boîte de dialogue native non-bloquante pour KiCad.
        Intégration propre au thème système avec bandeau d'en-tête anti-éblouissement.
        """
        def __init__(self, parent=None, board=None, pipeline_runner: Optional[Callable] = None):
            super().__init__(
                parent,
                title="Smart Placer & AutoRouter — KiCad",
                style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER,
                size=(520, 480)
            )
            self.board = board
            self.pipeline_runner = pipeline_runner
            self.worker_thread: Optional[threading.Thread] = None
            self.is_running = False
            self.execution_success = False
            self.collected_settings: Dict[str, Any] = dict(DEFAULT_SETTINGS)

            self.SetMinSize((480, 440))
            self._init_ui()
            self.Centre()
            self.Bind(wx.EVT_CLOSE, self._on_close_window)

        def _init_ui(self):
            main_sizer = wx.BoxSizer(wx.VERTICAL)

            # En-tête avec fond sombre élégant (remplace le fond blanc éblouissant)
            header_panel = wx.Panel(self)
            header_panel.SetBackgroundColour(HEADER_BG_COLOR)
            header_sizer = wx.BoxSizer(wx.VERTICAL)

            title_lbl = wx.StaticText(header_panel, label="Smart Placer & AutoRouter")
            f_title = title_lbl.GetFont()
            f_title.SetPointSize(f_title.GetPointSize() + 2)
            f_title.SetWeight(wx.FONTWEIGHT_BOLD)
            title_lbl.SetFont(f_title)
            title_lbl.SetForegroundColour(HEADER_TITLE_COLOR)
            header_sizer.Add(title_lbl, 0, wx.LEFT | wx.RIGHT | wx.TOP, 10)

            subtitle_lbl = wx.StaticText(
                header_panel,
                label="Placement & Routage Automatique Intelligent"
            )
            subtitle_lbl.SetForegroundColour(HEADER_SUB_COLOR)
            header_sizer.Add(subtitle_lbl, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 10)
            header_panel.SetSizer(header_sizer)
            main_sizer.Add(header_panel, 0, wx.EXPAND)

            # Onglets (Thème natif système)
            self.notebook = wx.Notebook(self)

            self.tab_placement = PlacementTabPanel(self.notebook)
            self.tab_dfm = DFMTabPanel(self.notebook)
            self.tab_router = RouterTabPanel(self.notebook)
            self.tab_gnd = GNDTabPanel(self.notebook)

            self.notebook.AddPage(self.tab_placement, "Placement")
            self.notebook.AddPage(self.tab_dfm, "Fabrication")
            self.notebook.AddPage(self.tab_router, "Routage")
            self.notebook.AddPage(self.tab_gnd, "Plan de Masse")
            main_sizer.Add(self.notebook, 1, wx.EXPAND | wx.ALL, 8)

            # Zone de statut textuel (VIDE au départ, affichée uniquement après clic sur Commencer)
            box_status = wx.BoxSizer(wx.VERTICAL)
            self.status_lbl = wx.StaticText(self, label="")  # Vide au départ !
            f_status = self.status_lbl.GetFont()
            f_status.SetWeight(wx.FONTWEIGHT_BOLD)
            self.status_lbl.SetFont(f_status)
            self.status_lbl.SetForegroundColour(COLOR_SUCCESS)
            box_status.Add(self.status_lbl, 0, wx.LEFT | wx.RIGHT | wx.TOP | wx.EXPAND, 8)

            # Boutons d'Action (Natif)
            btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
            btn_sizer.AddStretchSpacer(1)

            self.btn_start = wx.Button(self, id=wx.ID_OK, label="Commencer")
            self.btn_start.SetDefault()
            f_btn = self.btn_start.GetFont()
            f_btn.SetWeight(wx.FONTWEIGHT_BOLD)
            self.btn_start.SetFont(f_btn)
            self.btn_start.Bind(wx.EVT_BUTTON, self._on_start_clicked)
            btn_sizer.Add(self.btn_start, 0, wx.ALL, 6)

            self.btn_close = wx.Button(self, id=wx.ID_CANCEL, label="Fermer")
            self.btn_close.Bind(wx.EVT_BUTTON, self._on_close_clicked)
            btn_sizer.Add(self.btn_close, 0, wx.ALL, 6)

            box_status.Add(btn_sizer, 0, wx.EXPAND)
            main_sizer.Add(box_status, 0, wx.EXPAND | wx.ALL, 6)

            self.SetSizer(main_sizer)
            self.Layout()

        def get_settings(self) -> Dict[str, Any]:
            settings = dict(DEFAULT_SETTINGS)
            settings.update(self.tab_placement.get_values())
            settings.update(self.tab_dfm.get_values())
            settings.update(self.tab_router.get_values())
            settings.update(self.tab_gnd.get_values())
            return settings

        def _on_start_clicked(self, event):
            if self.is_running:
                return

            self.collected_settings = self.get_settings()
            logger.info(f"Paramètres validés : {self.collected_settings}")

            if self.pipeline_runner is None:
                self.EndModal(wx.ID_OK)
                return

            self.is_running = True
            self.btn_start.Enable(False)
            self.btn_close.SetLabel("Annuler / Masquer")
            self.notebook.Enable(False)
            self.status_lbl.SetLabel("Statut : [0%] Initialisation du moteur...")

            self.worker_thread = threading.Thread(
                target=self._worker_entry_point,
                name="SmartPlacerWorker",
                daemon=True
            )
            self.worker_thread.start()

        def _on_close_clicked(self, event):
            if self.is_running:
                res = wx.MessageBox(
                    "Un calcul est en cours d'exécution en arrière-plan.\n"
                    "Voulez-vous fermer la fenêtre ?",
                    "Opération en cours",
                    wx.YES_NO | wx.ICON_QUESTION, self
                )
                if res != wx.YES:
                    return
            self.EndModal(wx.ID_OK if self.execution_success else wx.ID_CANCEL)

        def _on_close_window(self, event):
            if self.is_running:
                event.Veto()
                return
            event.Skip()

        def _worker_entry_point(self):
            try:
                stats = self.pipeline_runner(
                    board=self.board,
                    settings=self.collected_settings,
                    progress_callback=self._report_progress
                )
                wx.CallAfter(self._on_worker_completed, True, "Succès", stats)
            except Exception as e:
                err_msg = str(e)
                logger.error(f"Erreur Worker Thread : {traceback.format_exc()}")
                wx.CallAfter(self._on_worker_completed, False, err_msg, None)

        def _report_progress(self, percent: int, message: str):
            wx.CallAfter(self._update_progress_gui, percent, message)

        def _update_progress_gui(self, percent: int, message: str):
            clamped_pct = max(0, min(100, int(percent)))
            self.status_lbl.SetLabel(f"Statut : [{clamped_pct}%] {message}")
            self.Layout()
            try:
                wx.SafeYield(self, onlyIfNeeded=True)
            except Exception:
                pass

        def _on_worker_completed(self, success: bool, message: str, stats: Optional[dict] = None):
            self.is_running = False
            self.execution_success = success
            self.notebook.Enable(True)
            self.btn_close.SetLabel("Fermer")
            self.btn_close.Enable(True)

            if success:
                summary_msg = "Statut : [100%] Terminé avec succès !"
                if stats and 'vias' in stats and 'tracks' in stats:
                    summary_msg = (
                        f"Statut : [100%] Terminé ! {stats['tracks']} pistes, "
                        f"{stats['vias']} vias ({stats.get('time_s', 0)}s)."
                    )
                self.status_lbl.SetLabel(summary_msg)
                self.btn_start.SetLabel("Recommencer")
                self.btn_start.Enable(True)

                # Rafraîchissement sécurisé sur le thread UI principal uniquement
                try:
                    if self.board and hasattr(pcbnew, "ZONE_FILLER") and hasattr(self.board, "Zones"):
                        pcbnew.ZONE_FILLER(self.board).Fill(self.board.Zones())
                except Exception:
                    pass
                try:
                    if PCBNEW_AVAILABLE and hasattr(pcbnew, "Refresh"):
                        pcbnew.Refresh()
                except Exception:
                    pass
                try:
                    if PCBNEW_AVAILABLE and hasattr(pcbnew, "UpdateUserInterface"):
                        pcbnew.UpdateUserInterface()
                except Exception:
                    pass
            else:
                self.status_lbl.SetLabel(f"Statut : Échec ({message})")
                self.btn_start.SetLabel("Réessayer")
                self.btn_start.Enable(True)
                wx.MessageBox(
                    f"Une erreur est survenue pendant l'exécution :\n\n{message}",
                    "Erreur Smart Placer", wx.OK | wx.ICON_ERROR, self
                )


def prompt_user_settings(parent=None, board=None, pipeline_runner=None) -> Optional[Dict[str, Any]]:
    if not WX_AVAILABLE:
        return dict(DEFAULT_SETTINGS)

    app = wx.GetApp()
    if not app:
        return dict(DEFAULT_SETTINGS)

    dlg = SmartPlacerDialog(parent=parent, board=board, pipeline_runner=pipeline_runner)
    result = dlg.ShowModal()
    settings = None
    if result == wx.ID_OK or dlg.execution_success:
        settings = dlg.collected_settings
    dlg.Destroy()
    return settings
