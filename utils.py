"""
=============================================================================
 utils.py — Module Utilitaire du Smart Placer
=============================================================================
 Ce module fournit :
   1. Les fonctions de conversion d'unités internes KiCad (nanomètres ↔ mm).
   2. Un système de logging structuré pour tracer l'évolution du recuit simulé.
   3. Des primitives géométriques réutilisées dans tout le projet.

 CONVENTION D'UNITÉS KICAD :
   KiCad utilise en interne des **nanomètres** (nm) comme unité de base.
   1 mm = 1 000 000 nm. Les fonctions pcbnew.FromMM() et pcbnew.ToMM()
   convertissent entre ces deux espaces.
   Toute la logique mathématique interne de ce plugin opère en **nanomètres**
   pour éviter les erreurs d'arrondi. La conversion en mm n'est faite que
   pour l'affichage utilisateur dans les logs.
=============================================================================
"""

import json
import logging
import math
import os
import sys
import time
from datetime import datetime
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

# ---------------------------------------------------------------------------
#  1. CONSTANTES DE CONVERSION
# ---------------------------------------------------------------------------

# Facteur de conversion : 1 mm = 1 000 000 unités internes (nm)
NM_PER_MM = 1_000_000


def to_mm(nm_value: int) -> float:
    """
    Convertit une valeur en nanomètres (unités internes KiCad) en millimètres.

    Formule :
        mm = nm / 1 000 000

    Args:
        nm_value: Valeur en nanomètres (entier, unité interne KiCad).

    Returns:
        Valeur en millimètres (flottant).
    """
    return nm_value / NM_PER_MM


def from_mm(mm_value: float) -> int:
    """
    Convertit une valeur en millimètres vers les unités internes KiCad (nm).

    Formule :
        nm = round(mm × 1 000 000)

    On arrondit à l'entier le plus proche car KiCad travaille en coordonnées
    entières (pas de sous-nanomètres).

    Args:
        mm_value: Valeur en millimètres (flottant).

    Returns:
        Valeur en nanomètres (entier).
    """
    return int(round(mm_value * NM_PER_MM))


# ---------------------------------------------------------------------------
#  2. STRUCTURES DE DONNÉES GÉOMÉTRIQUES
# ---------------------------------------------------------------------------

@dataclass
class BBox:
    """
    Rectangle Englobant Aligné sur les Axes (Axis-Aligned Bounding Box).

    Représentation canonique d'un rectangle 2D par ses bornes :
        (x_min, y_min) = coin supérieur-gauche
        (x_max, y_max) = coin inférieur-droit

    Toutes les coordonnées sont en unités internes KiCad (nanomètres).

    NOTE SUR LE SYSTÈME D'AXES KICAD :
        L'axe Y est **inversé** : Y augmente vers le bas.
        Cela n'affecte pas les calculs de surface ou d'intersection
        car on travaille toujours avec min/max.
    """
    x_min: int = 0
    y_min: int = 0
    x_max: int = 0
    y_max: int = 0

    @property
    def width(self) -> int:
        """Largeur du rectangle (dimension en X), en nm."""
        return self.x_max - self.x_min

    @property
    def height(self) -> int:
        """Hauteur du rectangle (dimension en Y), en nm."""
        return self.y_max - self.y_min

    @property
    def area(self) -> int:
        """
        Surface du rectangle en nm².
        Formule : S = largeur × hauteur
        """
        return self.width * self.height

    @property
    def center_x(self) -> int:
        """Coordonnée X du centre géométrique."""
        return (self.x_min + self.x_max) // 2

    @property
    def center_y(self) -> int:
        """Coordonnée Y du centre géométrique."""
        return (self.y_min + self.y_max) // 2

    @property
    def center(self) -> Tuple[int, int]:
        """Tuple (x, y) du centre géométrique."""
        return (self.center_x, self.center_y)

    def inflate(self, margin: int) -> 'BBox':
        """
        Expansion isotrope du rectangle (Somme de Minkowski avec un carré).

        Mathématiquement, la somme de Minkowski d'un rectangle R avec un
        disque de rayon r donne un rectangle arrondi. Pour simplifier
        (puisque nous travaillons en AABB), on approxime par une expansion
        uniforme de `margin` nm sur chaque côté :

            R_inflated = {(x, y) : dist(p, R) ≤ margin, pour tout p}

        En AABB, cela revient à :
            x_min' = x_min - margin
            y_min' = y_min - margin
            x_max' = x_max + margin
            y_max' = y_max + margin

        Args:
            margin: Marge d'expansion en nm (peut être négatif pour rétrécir).

        Returns:
            Nouveau BBox étendu.
        """
        return BBox(
            x_min=self.x_min - margin,
            y_min=self.y_min - margin,
            x_max=self.x_max + margin,
            y_max=self.y_max + margin
        )

    def intersects(self, other: 'BBox') -> bool:
        """
        Test d'intersection de deux AABB.

        Théorème du test de séparation d'axes (SAT) simplifié pour les AABB :
        Deux rectangles NE SE CHEVAUCHENT PAS si et seulement si
        l'un est entièrement à gauche, à droite, au-dessus ou en-dessous
        de l'autre.

        Autrement dit, ils S'INTERSECTENT si :
            x_min_A < x_max_B  ET  x_max_A > x_min_B
            ET
            y_min_A < y_max_B  ET  y_max_A > y_min_B

        Returns:
            True si les deux rectangles se chevauchent (même partiellement).
        """
        return (self.x_min < other.x_max and self.x_max > other.x_min and
                self.y_min < other.y_max and self.y_max > other.y_min)

    def overlap_area(self, other: 'BBox') -> int:
        """
        Calcule l'aire de la zone d'intersection entre deux AABB.

        L'intersection de deux AABB est elle-même un AABB défini par :
            ix_min = max(A.x_min, B.x_min)
            iy_min = max(A.y_min, B.y_min)
            ix_max = min(A.x_max, B.x_max)
            iy_max = min(A.y_max, B.y_max)

        L'aire est :
            S = max(0, ix_max - ix_min) × max(0, iy_max - iy_min)

        Si l'intersection est vide (les rectangles ne se chevauchent pas),
        au moins l'un des termes max(0, ...) sera nul → S = 0.

        Returns:
            Aire d'intersection en nm² (0 si pas d'intersection).
        """
        ix_min = max(self.x_min, other.x_min)
        iy_min = max(self.y_min, other.y_min)
        ix_max = min(self.x_max, other.x_max)
        iy_max = min(self.y_max, other.y_max)

        overlap_w = max(0, ix_max - ix_min)
        overlap_h = max(0, iy_max - iy_min)
        return overlap_w * overlap_h

    def contains_point(self, x: int, y: int) -> bool:
        """Vérifie si un point (x, y) est à l'intérieur du rectangle."""
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass
class ComponentData:
    """
    Structure de données représentant un composant PCB extrait de la carte.

    Champs :
        reference   : Identifiant du composant (ex: 'U1', 'C12', 'J3').
        footprint   : Nom du footprint (ex: 'SOIC-8_3.9x4.9mm').
        position    : Tuple (x, y) en nm, position du centre du composant.
        orientation : Angle de rotation en degrés (0, 90, 180, 270).
        bbox        : Bounding Box physique du composant (en nm).
        layer       : Face de montage ('F.Cu' = dessus, 'B.Cu' = dessous).
        pads        : Liste des pastilles avec leurs nets associés.
        is_locked   : Si True, le composant ne doit pas être déplacé.
        net_classes  : Ensemble des classes de net auxquelles ce composant
                       est connecté (pour déterminer le halo de dégagement).
        is_power    : True si le composant est lié à des réseaux de puissance.
        fp_ref      : Référence vers l'objet pcbnew.FOOTPRINT original
                       (pour appliquer les modifications finales).
    """
    reference: str = ""
    footprint: str = ""
    position: Tuple[int, int] = (0, 0)
    orientation: float = 0.0
    bbox: BBox = field(default_factory=BBox)
    layer: str = "F.Cu"
    pads: List[dict] = field(default_factory=list)
    is_locked: bool = False
    net_classes: set = field(default_factory=set)
    is_power: bool = False
    fp_ref: object = None  # Référence vers pcbnew.FOOTPRINT
    # Offsets relatifs du Courtyard (contour violet) par rapport à l'ancre (position) à 0°
    crt_dx_min: int = 0
    crt_dy_min: int = 0
    crt_dx_max: int = 0
    crt_dy_max: int = 0

    def get_bbox_at(self, x: int, y: int, orientation: float) -> BBox:
        """
        Calcule la BBox EXACTE du Courtyard (contour violet) à une position (x, y)
        et une orientation données, en respectant la cinématique d'ancrage KiCad.
        """
        a = int(round(orientation)) % 360
        if a == 90:
            x_min = x + self.crt_dy_min
            y_min = y - self.crt_dx_max
            x_max = x + self.crt_dy_max
            y_max = y - self.crt_dx_min
        elif a == 180:
            x_min = x - self.crt_dx_max
            y_min = y - self.crt_dy_max
            x_max = x - self.crt_dx_min
            y_max = y - self.crt_dy_min
        elif a == 270:
            x_min = x - self.crt_dy_max
            y_min = y + self.crt_dx_min
            x_max = x - self.crt_dy_min
            y_max = y + self.crt_dx_max
        else:  # 0°
            x_min = x + self.crt_dx_min
            y_min = y + self.crt_dy_min
            x_max = x + self.crt_dx_max
            y_max = y + self.crt_dy_max
        return BBox(x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max)


@dataclass
class NetInfo:
    """
    Informations sur un réseau (Net) électrique.

    Un Net connecte plusieurs pastilles (pads) de différents composants.
    Le HPWL (Half-Perimeter Wirelength) est calculé sur la base de la
    position des pads appartenant à ce net.

    Champs :
        name       : Nom du réseau (ex: 'VCC', 'GND', 'NET-U1-3').
        code       : Code numérique interne du net dans KiCad.
        pad_refs   : Liste de tuples (reference_composant, pad_number, x, y).
        is_power   : True si c'est un réseau de puissance (VCC, GND, etc.).
    """
    name: str = ""
    code: int = 0
    pad_refs: List[Tuple[str, str, int, int]] = field(default_factory=list)
    is_power: bool = False


# ---------------------------------------------------------------------------
#  3. FONCTIONS GÉOMÉTRIQUES UTILITAIRES
# ---------------------------------------------------------------------------

def euclidean_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
    """
    Distance euclidienne entre deux points.

    Formule :
        d(P₁, P₂) = √((x₂ - x₁)² + (y₂ - y₁)²)

    On travaille en nm mais le résultat est en flottant pour la précision.

    Args:
        p1, p2: Points sous forme de tuples (x, y) en nm.

    Returns:
        Distance en nm (flottant).
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.sqrt(dx * dx + dy * dy)


def manhattan_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> int:
    """
    Distance de Manhattan entre deux points.

    Formule :
        d_manhattan(P₁, P₂) = |x₂ - x₁| + |y₂ - y₁|

    La distance de Manhattan est utilisée en routage PCB car les pistes
    suivent généralement un routage orthogonal (horizontal/vertical).
    Elle fournit une borne inférieure plus réaliste que la distance
    euclidienne pour estimer la longueur de piste nécessaire.

    Args:
        p1, p2: Points sous forme de tuples (x, y) en nm.

    Returns:
        Distance en nm (entier).
    """
    return abs(p2[0] - p1[0]) + abs(p2[1] - p1[1])


def snap_angle(angle_deg: float) -> float:
    """
    Restreint un angle aux valeurs discrètes autorisées : 0, 90, 180, 270.

    Algorithme :
        1. Normalise l'angle dans [0, 360[
        2. Trouve l'angle discret le plus proche parmi {0, 90, 180, 270}

    C'est une contrainte physique : en placement PCB, les composants
    sont quasi-systématiquement placés à ces angles pour faciliter
    le routage sur une grille orthogonale.

    Args:
        angle_deg: Angle en degrés (quelconque).

    Returns:
        Angle discret le plus proche parmi {0, 90, 180, 270}.
    """
    VALID_ANGLES = [0.0, 90.0, 180.0, 270.0]
    # Normalisation dans [0, 360[
    normalized = angle_deg % 360.0
    if normalized < 0:
        normalized += 360.0

    # Trouver l'angle valide le plus proche (distance angulaire minimale)
    best = min(VALID_ANGLES, key=lambda a: min(
        abs(normalized - a),
        360.0 - abs(normalized - a)  # Gérer le wrap-around 350° → 0°
    ))
    return best


def compute_enclosing_bbox(bboxes: List[BBox]) -> BBox:
    """
    Calcule le plus petit rectangle englobant une liste de BBox.

    C'est l'union des AABB : on prend les bornes extrêmes.
        x_min = min(b.x_min pour b dans bboxes)
        y_min = min(b.y_min pour b dans bboxes)
        x_max = max(b.x_max pour b dans bboxes)
        y_max = max(b.y_max pour b dans bboxes)

    Utilisé pour calculer la surface totale occupée par le placement.

    Args:
        bboxes: Liste de BBox à englober.

    Returns:
        BBox englobant toutes les entrées.
    """
    if not bboxes:
        return BBox()
    return BBox(
        x_min=min(b.x_min for b in bboxes),
        y_min=min(b.y_min for b in bboxes),
        x_max=max(b.x_max for b in bboxes),
        y_max=max(b.y_max for b in bboxes)
    )


def point_to_segment_distance(
    px: int, py: int,
    ax: int, ay: int,
    bx: int, by: int
) -> float:
    """
    Distance minimale d'un point P à un segment [A, B].

    Algorithme basé sur la projection orthogonale :
        1. On paramétrise le segment : S(t) = A + t·(B - A), t ∈ [0, 1]
        2. On projette P sur la droite portant le segment :
           t = ((P - A) · (B - A)) / |B - A|²
        3. On clamp t dans [0, 1] pour rester sur le segment
        4. Le point le plus proche est S(t_clamped)
        5. La distance est |P - S(t_clamped)|

    Cas dégénéré : si A = B (segment de longueur nulle), on retourne
    simplement la distance P→A.

    Args:
        px, py: Coordonnées du point P.
        ax, ay: Coordonnées de l'extrémité A du segment.
        bx, by: Coordonnées de l'extrémité B du segment.

    Returns:
        Distance minimale en nm (flottant).
    """
    # Vecteur AB
    dx_ab = bx - ax
    dy_ab = by - ay

    # Longueur au carré du segment |AB|²
    seg_len_sq = dx_ab * dx_ab + dy_ab * dy_ab

    if seg_len_sq == 0:
        # Segment dégénéré (point) : distance P→A
        return euclidean_distance((px, py), (ax, ay))

    # Paramètre t de la projection (produit scalaire normalisé)
    # t = dot(AP, AB) / |AB|²
    t = ((px - ax) * dx_ab + (py - ay) * dy_ab) / seg_len_sq

    # Clamp t dans [0, 1] pour rester sur le segment
    t = max(0.0, min(1.0, t))

    # Point le plus proche sur le segment
    closest_x = ax + t * dx_ab
    closest_y = ay + t * dy_ab

    return euclidean_distance((px, py), (int(closest_x), int(closest_y)))


def ccw(A: Tuple[float, float], B: Tuple[float, float], C: Tuple[float, float]) -> bool:
    """Test d'orientation CCW (Counter-Clockwise) de 3 points 2D."""
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def segments_intersect(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
    p4: Tuple[float, float]
) -> bool:
    """
    Détermine si deux segments 2D [p1, p2] et [p3, p4] s'intersectent strictement.
    Les extrémités partagées ne sont pas considérées comme une intersection.
    """
    if p1 == p3 or p1 == p4 or p2 == p3 or p2 == p4:
        return False
    return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) and (ccw(p1, p2, p3) != ccw(p1, p2, p4))


# ---------------------------------------------------------------------------
#  4. SYSTÈME DE LOGGING STRUCTURÉ SÉCURISÉ POUR ENVIRONNEMENT GUI
# ---------------------------------------------------------------------------

class SafeStreamHandler(logging.Handler):
    """
    Handler de logging sécurisé pour l'environnement GUI KiCad.
    Évite l'erreur 'NoneType has no attribute write' lorsque sys.stdout/sys.stderr sont None.
    Écrit également dans un fichier de log local pour permettre l'inspection par l'utilisateur.
    """
    def __init__(self, log_file: Optional[str] = None):
        super().__init__()
        self._log_file = log_file

    def emit(self, record):
        try:
            msg = self.format(record)
            # Écrire vers stdout ou stderr si disponibles
            written = False
            for stream in (sys.stdout, sys.stderr):
                if stream is not None and hasattr(stream, 'write'):
                    try:
                        stream.write(msg + '\n')
                        if hasattr(stream, 'flush'):
                            stream.flush()
                        written = True
                        break
                    except Exception:
                        pass
            # Toujours écrire dans le fichier de log si configuré
            if self._log_file:
                try:
                    with open(self._log_file, 'a', encoding='utf-8') as f:
                        f.write(msg + '\n')
                except Exception:
                    pass
        except Exception:
            self.handleError(record)


def setup_logger(name: str = "SmartPlacer", level: int = logging.INFO) -> logging.Logger:
    """
    Configure un logger structuré pour le suivi du recuit simulé.

    Le format inclut le timestamp, le niveau, et le message :
        [HH:MM:SS] [LEVEL] message

    Args:
        name: Nom du logger.
        level: Niveau de logging (DEBUG, INFO, WARNING, ERROR).

    Returns:
        Logger configuré.
    """
    logger = logging.getLogger(name)

    # Réinitialiser les handlers pour garantir un état propre
    logger.handlers.clear()

    logger.setLevel(level)

    # Chemin du fichier de log dans le dossier utilisateur
    log_dir = os.path.expanduser("~")
    log_file = os.path.join(log_dir, "smart_placer.log")

    # Handler sécurisé (console si dispo + fichier)
    safe_handler = SafeStreamHandler(log_file=log_file)
    safe_handler.setLevel(level)

    # Format compact et lisible
    formatter = logging.Formatter(
        '[%(asctime)s] [%(levelname)-7s] %(message)s',
        datefmt='%H:%M:%S'
    )
    safe_handler.setFormatter(formatter)
    logger.addHandler(safe_handler)

    return logger


class AnnealingTracker:
    """
    Traqueur d'exécution du recuit simulé.

    Enregistre l'historique complet de l'optimisation pour analyse :
        - Température à chaque itération
        - Score de coût (HPWL + pénalités)
        - Taux d'acceptation des mouvements
        - Temps écoulé

    L'historique est stocké en mémoire et peut être exporté pour
    tracer les courbes de convergence.
    """

    def __init__(self, logger: logging.Logger):
        self.logger = logger
        self.start_time: float = 0.0
        self.history: List[dict] = []
        self.total_moves: int = 0
        self.accepted_moves: int = 0

    def start(self):
        """Démarre le chronomètre."""
        self.start_time = time.time()
        self.logger.info("=" * 70)
        self.logger.info("  SMART PLACER — Recuit Simulé Démarré")
        self.logger.info("=" * 70)

    def log_iteration(self, iteration: int, temperature: float,
                      cost: float, hpwl: float, penalties: float,
                      accepted: int, total: int):
        """
        Enregistre et affiche les métriques d'une itération.

        Args:
            iteration:   Numéro de l'itération courante.
            temperature: Température actuelle du recuit (sans unité).
            cost:        Coût total = HPWL + pénalités.
            hpwl:        Composante HPWL du coût (en nm).
            penalties:   Somme des pénalités (collisions + thermiques).
            accepted:    Nombre de mouvements acceptés à cette itération.
            total:       Nombre total de mouvements tentés.
        """
        elapsed = time.time() - self.start_time
        accept_rate = (accepted / total * 100) if total > 0 else 0.0

        record = {
            'iteration': iteration,
            'temperature': temperature,
            'cost': cost,
            'hpwl_nm': hpwl,
            'hpwl_mm': to_mm(hpwl),
            'penalties': penalties,
            'accept_rate': accept_rate,
            'elapsed_s': elapsed
        }
        self.history.append(record)

        # Affichage condensé toutes les N itérations pour ne pas noyer la console
        self.logger.info(
            f"Iter {iteration:5d} | T={temperature:10.2f} | "
            f"Coût={cost:12.0f} | HPWL={to_mm(hpwl):8.2f}mm | "
            f"Pén={penalties:10.0f} | Accept={accept_rate:5.1f}% | "
            f"t={elapsed:.1f}s"
        )

    def finish(self, final_cost: float, final_hpwl: float):
        """Affiche le résumé final de l'optimisation."""
        elapsed = time.time() - self.start_time
        self.logger.info("=" * 70)
        self.logger.info(f"  OPTIMISATION TERMINÉE en {elapsed:.1f}s")
        self.logger.info(f"  Coût final     : {final_cost:,.0f}")
        self.logger.info(f"  HPWL finale    : {to_mm(final_hpwl):.2f} mm")
        self.logger.info(f"  Itérations     : {len(self.history)}")
        self.logger.info("=" * 70)


def save_run_trace(trace_data: dict) -> str:
    """
    Enregistre les données télémétriques d'une session de placement
    dans un dossier dédié pour analyse et boucle de retour.

    Dossier : ~/.kicad_smart_placer_traces/
    Fichier : trace_YYYYMMDD_HHMMSS.json

    Args:
        trace_data: Dictionnaire contenant toutes les métriques de la session.

    Returns:
        Chemin absolu du fichier de trace JSON créé.
    """
    traces_dir = os.path.join(os.path.expanduser("~"), ".kicad_smart_placer_traces")
    try:
        os.makedirs(traces_dir, exist_ok=True)
    except Exception:
        pass

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = os.path.join(traces_dir, f"trace_{timestamp_str}.json")

    try:
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(trace_data, f, indent=2, ensure_ascii=False)
    except Exception as e:
        logging.getLogger("SmartPlacer").warning(f"Impossible d'écrire la trace : {e}")

    return file_path
