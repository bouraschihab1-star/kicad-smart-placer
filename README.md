# Smart Placer & AutoRouter pour KiCad

[![KiCad](https://img.shields.io/badge/KiCad-7.0%20%7C%208.0%20%7C%209.0%20%7C%2010.0-blue.svg)](https://kicad.org)
[![Python](https://img.shields.io/badge/Python-3.9%2B-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Smart Placer & AutoRouter** est une extension complète pour l'éditeur de circuits imprimés (`pcbnew`) de **KiCad**, intégrant un algorithme de placement automatique par **recuit simulé** et un **routeur géométrique DFM** haute performance avec **Push & Shove post-routage pour le plan de masse**.

---

## 🚀 Fonctionnalités Clés

### 1. Interface Graphique "Zéro Configuration" & Anti-Freeze
- **Architecture Non-Bloquante :** Exécution du moteur mathématique dans un `Worker Thread` dédié avec délégation sûre (`wx.Yield()` / `wx.CallAfter`), éliminant l'erreur *"Ne répond plus"* sous Windows.
- **Interface Épurée :** Paramètres pré-remplis avec des valeurs industrielles optimales (pas de curseurs superflus, sélecteurs déroulants pour les couches et champs directs).
- **Suivi en temps réel :** Barre de progression fluide (`wx.Gauge`) et messages de phase clairs.

### 2. Moteur de Placement par Recuit Simulé (Simulated Annealing)
- **Topologie & Hiérarchie :** Partitionnement Min-Cut (Kernighan-Lin) et détection des groupes rigides (cristaux de quartz, condensateurs de découplage, sous-feuilles hiérarchiques).
- **Minimisation HPWL :** Longueur totale des équipotentielles optimisée avec préservation de l'orientation (0°, 90°, 180°, 270°).
- **Modèle spatial physique :** Barrière infranchissable sur le contour `Edge.Cuts` (marge paramétrable de 1.0 mm) et anti-chevauchement des boîtes d'encombrement (AABB).

### 3. Routeur Géométrique A* & Règles DFM Industrielles
- **Angles 45° et 90° Stricts :** Sortie et entrée orthogonales par le centre des pastilles, raccordements 45° anti-pièges à acide (acid traps).
- **Pénalité Drastique des Vias (80.0 mm) :** L'algorithme privilégie agressivement les longs détours sur la même couche plutôt que de percer inutilement le PCB.
- **Détection Stricte de la Collerette de Cuivre des Vias :** Calcul basé sur le diamètre total de cuivre `GetWidth()` (et non le perçage) avec formule d'exclusion DFM :
  $$\text{Zone d'Exclusion} = \left(\frac{\text{via.GetWidth()}}{2}\right) + \text{Clearance}_{\text{Netclass}}$$
- **Respect Automatique des NetClasses :** Prise en compte dynamique des largeurs de pistes et isolements (Secteur/230V, Alimentations, Signaux).

### 4. Logique "Push & Shove" pour le Plan de Masse (GND)
- **Zéro Via GND & Zéro Piste Parasite :** Détection automatique des pastilles de masse (GND / 0V) sans forcer d'artifices de secours.
- **Boucle d'Élargissement des Couloirs :** Détection post-routage des étranglements (< 0.50 mm) et translation vectorielle des segments responsables pour ouvrir la voie au cuivre coulé.
- **Cascade Push & Shove :** Décalage des pistes voisines avec préservation stricte de la continuité électrique et respect des règles DRC.

### 5. Nettoyage Automatique de la Sérigraphie (Silkscreen DFM)
- Détection et repositionnement automatique des identificateurs de composants (`RefDes`) pour éviter tout chevauchement avec les pastilles de cuivre et les zones nues.

---

## 📦 Installation

### Méthode 1 : Copie Directe (Recommandée)
Téléchargez ou clonez ce dépôt, puis copiez le dossier `kicad_smart_placer` dans le répertoire des plugins de votre version de KiCad :

- **Windows :**
  ```text
  %APPDATA%\kicad\10.0\scripting\plugins\
  ou
  %USERPROFILE%\Documents\KiCad\10.0\3rdparty\plugins\
  ```
- **Linux :**
  ```text
  ~/.local/share/kicad/10.0/scripting/plugins/
  ```
- **macOS :**
  ```text
  ~/Library/Preferences/kicad/10.0/scripting/plugins/
  ```

### Méthode 2 : Archive ZIP
Un fichier d'installation tout-en-un `kicad_smart_placer.zip` est fourni dans les *Releases* ou générable via `create_zip.py`.

---

## 🛠️ Utilisation dans KiCad

1. Ouvrez votre projet dans l'éditeur de circuit imprimé (`pcbnew`).
2. Cliquez sur l'icône **Smart Placer & AutoRouter** dans la barre d'outils supérieure (ou via le menu *Outils > Scripts externes*).
3. Ajustez les paramètres si nécessaire (Plan de masse `B.Cu`, `F.Cu` ou `B.Cu + F.Cu`, marge du bord).
4. Cliquez sur **Commencer** :
   - Phase 1-5 : Extraction et placement optimisé par recuit simulé.
   - Phase 6-8 : Routage A* multicouche DFM sans croisement.
   - Phase 9 : Push & Shove des couloirs de masse et création du plan de masse.
   - Phase 10 : Nettoyage de la sérigraphie et sauvegarde.

---

## 📄 Licence

Ce projet est distribué sous la licence [MIT](LICENSE).

Auteur : **Bouras Chihabedinne** (<bouraschihab1@gmail.com>)
