# Smart Placer & AutoRouter pour KiCad

Extension pour l'éditeur de circuits imprimés (pcbnew) de KiCad 7, 8, 9 et 10.
Le projet combine un algorithme de placement automatisé sous contraintes physiques (recuit simulé) et un moteur de routage géométrique A* multicouche intégrant les règles de fabricabilité industrielle (DFM) et une logique de déplacement post-routage (Push and Shove) pour le plan de masse.

---

## Architecture Mathématique et Contraintes Implémentées

### 1. Moteur de Placement sous Contraintes (Recuit Simulé)
Le moteur optimise la position spatiale et l'orientation des composants en minimisant une fonction de coût multi-critères :

- **Minimisation HPWL (Half-Perimeter Wire Length) :** Évaluation globale de la longueur théorique du chevelu (ratsnest) calculée à partir des coordonnées exactes de chaque pastille.
- **Partitionnement Min-Cut (Kernighan-Lin) :** Regroupement topologique des composants partageant une forte densité d'interconnexions afin de minimiser le croisement des équipotentielles avant le routage.
- **Groupes Rigides et Hiérarchie :** Verrouillage géométrique des composants sensibles (quartz d'horloge au plus près des broches oscillateur, condensateurs de découplage accolés aux broches d'alimentation, regroupement par sous-feuilles schématiques).
- **Contraintes Spatiales Strictes (AABB) :** Détection d'interpénétration par boîtes englobantes alignées sur les axes. Pénalisation infinie en cas de chevauchement de cuivre ou de corps de composant.
- **Marge d'Usinage (Edge.Cuts) :** Barrière infranchissable maintenant tous les composants à au moins 1.0 mm du bord du PCB pour protéger les pastilles lors du détourage à la fraiseuse.
- **Discrétisation Angulaire :** Snapping strict des rotations autorisées aux multiples orthogonaux (0°, 90°, 180°, 270°).

---

### 2. Routeur Géométrique A* et Règles DFM Industrielles
Le routeur convertit le chevelu en pistes de cuivre sans court-circuit, en résolvant net par net sur une grille discrète d'unités internes (nanomètres) :

- **Angles 45° et 90° Stricts (Anti-Acid Traps) :** Élimination absolue des angles intérieurs aigus (< 90°) susceptibles de piéger les agents de gravure chimique et de provoquer des coupures de pistes à la fabrication.
- **Raccordement Orthogonal des Pastilles :** Les pistes entrent et sortent obligatoirement par le centre géométrique de la pastille sur l'axe X ou Y avant d'entamer une diagonale à 45°.
- **Pénalité Drastique des Vias (80.0 mm) :** Le coût de transition de couche dans la fonction de coût A* (g-score) et l'heuristique (h-score) est fixé à 80.0 mm. Le solveur privilégie mathématiquement un long détour géométrique sur la même couche plutôt que la création d'un via traversant.
- **Correction DFM du Diamètre des Vias (Collerette de Cuivre) :** Le test de collision n'utilise pas le diamètre de perçage mécanique (GetDrillValue), mais le diamètre extérieur de la collerette de cuivre (GetWidth). La zone d'exclusion autour d'un via est strictement vérifiée selon la formule :
  
  Distance_Exclusion = (via.GetWidth() / 2) + Clearance_Netclass
  
  Cette vérification s'applique à la fois sur la grille de routage et lors de la pose physique pour interdire tout effleurement avec les pastilles CMS.
- **Respect Automatique des NetClasses :** Application automatique des règles d'isolement et de largeur selon la nature du signal (haute tension 230V, alimentations +12V/-12V/+3.3V/GND, signaux logiques).

---

### 3. Logique Push and Shove pour le Plan de Masse (GND)
Pour garantir la continuité du plan de masse sans recourir à des pistes parasites ni à des vias de couture, le moteur exécute une passe de déplacement dynamique post-routage :

- **Détection des Étranglements :** Analyse de la distance libre entre les pastilles de masse et les segments de piste environnants. Un étranglement est détecté dès que l'espace libre est inférieur à 0.50 mm.
- **Translation Dynamique :** Le segment de piste provoquant le blocage subit une translation radiale vers l'extérieur du pad GND afin d'ouvrir un couloir d'au moins 0.50 mm permettant au cuivre de masse de s'écouler librement.
- **Cascade Push and Shove :** Si le déplacement d'un segment réduit l'écart avec une piste voisine d'un autre net sous le seuil d'isolement DRC, cette voisine est à son tour décalée de manière récursive.
- **Préservation Stricte des Connexions (Ancrages) :** Les extrémités des pistes en contact direct avec une pastille ou un via restent ancrées à leur position d'origine. Seul le point intermédiaire ou le corps du segment est translaté, et les segments contigus du même réseau sont automatiquement réalignés pour préserver 100% de la continuité électrique sans générer d'éléments déconnectés.

---

### 4. Dégagement de la Sérigraphie (Silkscreen)
Passe de nettoyage DFM analysant les boîtes englobantes des identificateurs de composants (RefDes) pour les décaler hors des pastilles de cuivre et des pistes nues, évitant tout rejet de fabrication lors du vernis épargne.

---

## Installation

Copier le dossier `kicad_smart_placer` dans le répertoire des extensions de votre installation KiCad :

- Windows : `%APPDATA%\kicad\10.0\scripting\plugins\` ou `%USERPROFILE%\Documents\KiCad\10.0\3rdparty\plugins\`
- Linux : `~/.local/share/kicad/10.0/scripting/plugins/`
- macOS : `~/Library/Preferences/kicad/10.0/scripting/plugins/`

Une archive prête à l'emploi `kicad_smart_placer.zip` est également disponible dans la section Releases du dépôt.

---

## Auteur

Bouras Chihabedinne (bouraschihab1@gmail.com)
