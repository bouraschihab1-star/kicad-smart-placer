"""
=============================================================================
 __init__.py — Point d'Entrée du Plugin KiCad Smart Placer
=============================================================================
 Ce fichier est le point d'entrée chargé par KiCad lors du démarrage.
 Il enregistre l'Action Plugin dans le système de plugins KiCad.

 MÉCANISME DE CHARGEMENT KICAD :
   1. KiCad parcourt les dossiers de plugins au démarrage
   2. Pour chaque dossier contenant un __init__.py, il l'importe
   3. Il cherche des appels à pcbnew.ActionPlugin.register()
   4. Les plugins enregistrés apparaissent dans le menu Outils

 STRUCTURE DU DOSSIER :
   kicad_smart_placer/
   ├── __init__.py          ← CE FICHIER (enregistrement)
   ├── plugin_action.py     ← Classe ActionPlugin principale
   ├── board_extractor.py   ← Extraction des données carte
   ├── spatial_model.py     ← Modélisation spatiale (Minkowski, halos)
   ├── net_analyzer.py      ← Analyse des réseaux électriques
   ├── constraints.py       ← Contraintes de placement
   ├── optimizer.py         ← Recuit simulé + Min-Cut
   └── utils.py             ← Utilitaires (conversions, logging, géométrie)

 INSTALLATION :
   Copier ce dossier dans le répertoire de plugins KiCad :
     Windows : %APPDATA%\KiCad\8.0\scripting\plugins\
     Linux   : ~/.local/share/kicad/8.0/scripting/plugins/
     macOS   : ~/Library/Preferences/kicad/8.0/scripting/plugins/
=============================================================================
"""

try:
    import pcbnew
    import wx
    from .plugin_action import SmartPlacerPlugin

    # Ne s'enregistrer que si l'application KiCad GUI est active
    if wx.GetApp() is not None:
        try:
            SmartPlacerPlugin().register()
        except Exception as e:
            pass
except Exception:
    pass
