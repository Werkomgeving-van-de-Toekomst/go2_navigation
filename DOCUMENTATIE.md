# SLAM Exploration - Unitree Go2 Documentatie

## Overzicht

Dit project implementeert autonome exploratie voor de Unitree Go2 robot met behulp van de originele C++ SDK2. De robot kan zelfstandig een gebied verkennen terwijl het obstakels vermijdt.

**Status: Volledig werkend met C++ native RealSense camera integratie en Wander Mode**

---

## Laatste Update: 2024-02-20

### Werkende Configuratie (Wander Mode)
Na uitgebreid testen is de **Wander Mode** de beste oplossing gebleken:

```bash
./slam_exploration --interface enP8p1s0 --duration 180 --area small --camera
```

**Belangrijkste instellingen:**
- Obstakel thresholds: voor 40cm (draaien), 30cm (stop)
- Stuck detection: alleen na 2 seconden echt vastzitten (< 1cm beweging)
- Korte escape manoeuvre (0.6s reverse + 0.6s draaien)
- Geen return-to-home (veroorzaakt problemen)

---

## Bestanden

| Bestand | Beschrijving |
|---------|-------------|
| `src/slam_exploration.cpp` | Hoofdprogramma - alles in C++ |
| `src/CMakeLists.txt` | Build configuratie met RealSense |
| `slam_exploration.sh` | Shell script voor gecombineerde SLAM |
| `camera_obstacle_service.py` | (Optioneel) Python camera service - niet meer nodig |

---

## Wat Werkt

### 1. Wander Mode (AANBEVOLEN)
- **Vrije exploratie** zonder vaste waypoints
- Robot kiest willekeurige richtingen
- Veel robuuster dan waypoint navigatie
- Beste resultaten in tests

### 2. Basis Robot Controle
- Robot opstaan (`StandUp()`)
- Balans modus (`BalanceStand()`)
- Bewegen met `Move(vx, vy, vyaw)` - inclusief zijwaarts!

### 2. Obstakel Vermijding
- SDK obstakel vermijding via `ObstaclesAvoidClient`
- Automatisch inschakelen bij start
- Werkt goed in combinatie met eigen navigatie

### 3. Waypoint Navigatie
- Genereert waypoints in een grid patroon (lawnmower pattern)
- Navigeert naar elk waypoint
- Gaat door naar volgend waypoint wanneer bereikt (< 0.4m)

### 4. RealSense Camera Obstakel Detectie (C++ Native)
- **Volledig in C++ geïntegreerd** - geen Python nodig
- Real-time diepte analyse op 640x480 @ 30fps
- Verdeelt camera beeld in 3 regio's: links, voor, rechts
- Median filtering voor robuuste waarden
- Thread-based - blokkeert niet de hoofdlus

### 5. Vastlopen Detectie & Escape Manoeuvre
- Detecteert wanneer robot niet vooruit komt (< 2cm beweging)
- **4-fase escape manoeuvre:**
  1. **Achteruit** (1.0-1.2s) - Ruimte creëren
  2. **Draaien** (1.0s) - Opening zoeken
  3. **Zijwaarts** (0.6s) - Obstakel ontwijken
  4. **Vooruit** (0.6s) - Nieuwe richting op

### 6. Krappe Ruimte Detectie
- Detecteert wanneer alle kanten geblokkeerd zijn (F, L, R < 35cm)
- Langer achteruit lopen in krappe ruimtes (1.2s)
- Versnelde stuck detection

### 7. Slimme Richting Keuze
- Kiest draairichting op basis van camera data
- Draait naar kant met meer open ruimte
- Wisselt af als geen voorkeur

### 8. Vloeiende Beweging
- Velocity smoothing voorkomt schokkerige bewegingen
- Smooth factor van 0.3
- Proactief zijwaarts bewegen bij obstakels

---

## Architectuur

```
┌─────────────────────────────────────────────────────────────┐
│                    SLAMExplorer (C++)                        │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ SportClient │  │ Obstacles    │  │ StateSubscriber  │   │
│  │             │  │ AvoidClient  │  │                  │   │
│  └─────────────┘  └──────────────┘  └──────────────────┘   │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              RealSenseCamera (Thread)                 │  │
│  │  ┌─────────────────────────────────────────────────┐ │  │
│  │  │  rs2::pipeline → Depth Analysis → CameraObstacles│ │  │
│  │  └─────────────────────────────────────────────────┘ │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                  Escape State Machine                 │  │
│  │  Phase 1: REVERSE → Phase 2: TURN → Phase 3: SIDE   │  │
│  │           → Phase 4: FORWARD                          │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## Code Structuur

### Hoofd Klassen

```cpp
// RealSense Camera Thread
class RealSenseCamera {
    rs2::pipeline pipeline;
    std::thread cameraThread;
    std::mutex dataMutex;
    CameraObstacles obstacles;

    bool init();              // Initialiseer camera
    void start();             // Start background thread
    void stop();              // Stop thread
    CameraObstacles getObstacles();  // Thread-safe data access
};

// Main Explorer
class SLAMExplorer {
    SportClient sportClient;
    ObstaclesAvoidClient oaClient;
    RealSenseCamera camera;

    void Explore(int duration, string areaSize, bool useCamera);
    void Stop();
    std::vector<Waypoint> GenerateWaypoints(string areaSize);
};
```

### Belangrijke Variabelen

```cpp
// Velocity smoothing
double currentVx = 0, currentVy = 0, currentVyaw = 0;
const double smoothFactor = 0.3;

// Stuck detection
int stuckCount = 0;           // Teller voor vastlopen
int escapePhase = 0;          // 0=normal, 1=reverse, 2=turn, 3=side, 4=forward
int escapeTimer = 0;          // Tijdsduur huidige fase
int turnDirection = 1/-1;     // Links of rechts

// Tight space detection
int tightSpaceCount = 0;
bool inTightSpace = camObs.front < 0.35 && camObs.left < 0.35 && camObs.right < 0.35;

// Camera thresholds
float frontThreshold = 0.30f;   // 30cm
float sideThreshold = 0.25f;    // 25cm
```

### Camera Data Structuur

```cpp
struct CameraObstacles {
    float front = 999.0f;      // Afstand voor in meters
    float left = 999.0f;       // Afstand links
    float right = 999.0f;      // Afstand rechts
    float rear = 999.0f;       // Niet gebruikt (voor camera)
    bool frontBlocked;         // < 30cm
    bool leftBlocked;          // < 25cm
    bool rightBlocked;         // < 25cm
    bool valid;                // Data beschikbaar
};
```

---

## Escape Manoeuvre Details

### Fase 1: REVERSE (1.0-1.2 seconden)
```cpp
currentVx = -0.30;           // Achteruit
currentVy = 0;
currentVyaw = 0.2 * turnDirection;  // Licht draaien
```

### Fase 2: TURN (1.0 seconden)
```cpp
currentVx = 0;
currentVy = 0;
currentVyaw = 0.9 * turnDirection;  // Snel draaien
```

### Fase 3: SIDESTEP (0.6 seconden)
```cpp
currentVx = 0;
currentVy = 0.35 * turnDirection;   // Zijwaarts
currentVyaw = 0.3 * turnDirection;  // Draaien
```

### Fase 4: FORWARD (0.6 seconden)
```cpp
currentVx = 0.25;            // Vooruit
currentVy = 0.15 * turnDirection;
currentVyaw = 0.2 * turnDirection;
```

---

## Hoe Te Gebruiken

### Quick Start (Aanbevolen)

```bash
# Bouw
cd /home/unitree/sdk2_navigation/src/build
make slam_exploration -j$(nproc)

# Start Wander Mode (3 minuten)
export LD_LIBRARY_PATH=/home/unitree/unitree_sdk2/thirdparty/lib/aarch64:$LD_LIBRARY_PATH
./slam_exploration --interface enP8p1s0 --duration 180 --area small --camera
```

### Bouwen

```bash
cd /home/unitree/sdk2_navigation/src
mkdir -p build && cd build
cmake ..
make slam_exploration -j$(nproc)
```

### Starten

```bash
export LD_LIBRARY_PATH=/home/unitree/unitree_sdk2/thirdparty/lib/aarch64:$LD_LIBRARY_PATH

# Standaard (60 seconden, klein gebied)
./slam_exploration --interface enP8p1s0 --duration 60 --area small

# Met RealSense camera (AANBEVOLEN)
./slam_exploration --interface enP8p1s0 --duration 60 --area small --camera

# Groter gebied
./slam_exploration --interface enP8p1s0 --duration 120 --area medium --camera

# Zonder obstakel vermijding
./slam_exploration --interface enP8p1s0 --duration 60 --area small --no-oa

# Zonder camera
./slam_exploration --interface enP8p1s0 --duration 60 --area small
```

### Parameters

| Parameter | Standaard | Beschrijving |
|-----------|-----------|--------------|
| `--interface` | enP8p1s0 | Netwerk interface |
| `--duration` | 60 | Duur in seconden |
| `--area` | medium | Gebied grootte (small/medium/large) |
| `--no-oa` | - | Schakel obstakel vermijding uit |
| `--camera` | - | Schakel RealSense camera in (C++ native) |
| `--help` | - | Toon hulp |

---

## Status Output

De robot toont real-time status:

```
[45s/60s] WP:1/10 | Pos:(+1.2,+0.9) | Vel:(-0.30,0.00,0.20) | Cam:(F0.82,L0.76,R0.24) [REV!]
```

### Status Indicators

| Indicator | Betekenis |
|-----------|-----------|
| `[REV!]` | Achteruit bewegen (Fase 1) |
| `[ESC]` | Escape manoeuvre bezig (Fase 2-4) |
| `[TIGHT]` | Krappe ruimte gedetecteerd |
| `[STUCK?]` | Vastloop detectie actief |

### Camera Waarden

- `F` = Front (voor)
- `L` = Left (links)
- `R` = Right (rechts)
- `999.00` = Geen obstakel gedetecteerd

---

## Test Resultaten

### Test: 60 seconden, klein gebied, met camera

| Metric | Waarde |
|--------|--------|
| Start positie | (+0.4, -0.0) |
| Eind positie | (+0.7, +1.5) |
| Totale beweging | ~1.6m |
| Escape manoeuvres | 11 |
| Camera bereik | 0.4 - 6.6m |

**Resultaat:** Robot verkent actief de ruimte, manoeuvreert zich uit krappe situaties.

---

## Experimenten & Wat NIET Werkte

### 1. Waypoint-gebaseerde Navigatie
**Probleem:** De robot bleef op één plek hangen en bereikte geen waypoints.

**Oorzaken:**
- Positie-odometrie drift - robot denkt dat hij op (+1.5, +1.3) is, maar is elders
- Stuck detection te gevoelig - triggt escape voordat waypoint bereikt wordt
- Robot draait continu zonder vooruit te gaan

**Geprobeerd:**
- Minder gevoelige stuck detection (100 vs 50 cycles)
- Lagere obstakel thresholds (20cm ipv 30cm)
- Meer agressieve beweging (0.50 m/s)
- Eerder draaien bij obstakels (60cm)

**Resultaat:** Nog steeds vastlopen op zelfde positie.

**Oplossing:** Wander mode - robot zwervet vrij rond zonder vaste waypoints.

---

### 2. Return-to-Home Functionaliteit
**Probleem:** Robot komt niet terug naar startpositie, blijft oscilleren.

**Oorzaken:**
- Odometrie drift maakt "home" positie onnauwkeurig
- Escape manoeuvres bewegen robot weg van home
- Robot komt dichtbij (1.5m) maar raakt weer vast

**Geprobeerd:**
- Escape manoeuvres tijdens return-home
- Alleen draaien als voor geblokkeerd
- Achteruit + draaien combinatie
- Spin-to-find-opening strategie

**Resultaat:** Robot blijft heen-en-weer bewegen op ~2m van start.

**Oplossing:** Return-to-home uitgeschakeld - robot stopt na exploratie.

---

### 3. Te Gevoelige Stuck Detection
**Probleem:** Robot trekt terug terwijl er genoeg ruimte is.

**Oorzaak:** Stuck count too low (50 cycles = 1 seconde), triggering escapes when robot is just moving slowly.

**Geprobeerd:**
- stuckCount > 50 (1 seconde) → teveel escapes
- stuckCount > 80 (1.6 seconde) → nog steeds gevoelig
- stuckCount > 100 (2 seconden) → beter resultaat

**Oplossing:** Stuck threshold op 100, alleen triggeren bij < 1cm beweging.

---

### 4. Te Agressieve Obstakel Vermijding
**Probleem:** Robot draait/draait terwijl obstakels op 80cm+ zijn.

**Oorzaak:** Obstacle avoidance thresholds te hoog (50-60cm).

**Geprobeerd:**
- front < 60cm → draaien
- front < 50cm → draaien
- front < 40cm → draaien (huidig)

**Oplossing:** Alleen draaien bij < 40cm, stoppen bij < 30cm.

---

### 5. Complex Escape Manoeuvre (4-fase)
**Probleem:** 4-fase escape duurde te lang (3.2 seconden).

**Oorspronkelijk:**
1. Reverse 1.0s
2. Turn 1.0s
3. Sidestep 0.6s
4. Forward 0.6s

**Probleem:** Robot vast in escape-lus, beweegt niet vooruit.

**Oplossing:** Vereenvoudigd naar 2-fase (0.6s reverse + 0.6s turn).

---

### 6. Velocity Smoothing
**Probleem:** Te veel smoothing (0.3) maakt robot traag in reactie.

**Geprobeerd:**
- smoothFactor = 0.3 → traag
- smoothFactor = 0.5 → beter (huidig)
- Geen smoothing → te schokkerig

**Oplossing:** smoothFactor = 0.5 voor betere responsiviteit.

---

## Werkende Parameters (Finale Configuratie)

```cpp
// Obstakel thresholds
if (camObs.front < 0.40f) {  // 40cm - begin te draaien
    targetVx = 0.15;
    targetVyaw = (camObs.left > camObs.right) ? 0.6 : -0.6;
}
if (camObs.front < 0.30f) {  // 30cm - stop en draai
    targetVx = 0;
    targetVyaw = (camObs.left > camObs.right) ? 0.8 : -0.8;
}

// Stuck detection
if (moved < 0.01 && targetVx > 0.3) {  // < 1cm beweging
    stuckCount++;
}
if (stuckCount > 100) {  // 2 seconden vast
    // Trigger escape
}

// Escape manoeuvre
currentVx = -0.30;        // 0.6s achteruit
currentVyaw = 0.9 * dir;  // 0.6s draaien

// Basis snelheid
double targetVx = 0.40;   // 40 cm/s vooruit
```

---

## Test Resultaat: Wander Mode (180 seconden)

| Metric | Waarde |
|--------|--------|
| Start positie | (+2.3, +2.3) |
| Maximale afstand | (+5.3, +2.1) |
| Eind positie | (+3.7, +2.7) |
| Escape manoeuvres | 43 |
| Looptijd | 180 seconden |

**Resultaat:** Robot verkent actief, beweegt door omgeving, vermijdt obstakels succesvol.

---

### 1. Interne Obstakel Sensoren
- `range_obstacle` array geeft onbetrouwbare data
- Gebruik RealSense camera in plaats

### 2. LiDAR Mapping Integratie
- Livox Mid-360 niet direct geïntegreerd
- Vereist apart proces

### 3. Positionering Nauwkeurigheid
- `state.position()` niet erg nauwkeurig
- Relatieve beweging wel betrouwbaar

---

## Bekende Limitaties

### Hardware
- RealSense camera kijkt alleen naar voren
- Geen achteruit camera (rear blijft 999.0)

### Software
- Geen echte SLAM (geen map building)
- Geen path planning algoritme
- **Geen terugkeer naar start positie** (odometrie drift maakt dit onbetrouwbaar)
- Geen battery level monitoring
- Waypoint navigatie werkt niet goed (gebruik wander mode)

### Odometrie
- `state.position()` drift over tijd
- Niet betrouwbaar voor nauwkeurige navigatie
- Wel bruikbaar voor relatieve beweging detectie

---

## Debug Tips

### Robot beweegt niet
1. Controleer of robot in balance stand is
2. Wacht 3 seconden na StandUp() en BalanceStand()
3. Controleer obstakel vermijding status

### Veel vastlopen
1. Gebruik `--camera` optie voor betere detectie
2. Controleer of camera werkt (rs-enumerate-devices)
3. Vergroot area size parameter

### Camera niet beschikbaar
```bash
# Check camera
rs-enumerate-devices --short

# Kill blocking processes
fuser /dev/video* -k
```

### Crashes bij startup
1. Initialiseer ChannelFactory VOORDAT je clients maakt
2. Controleer LD_LIBRARY_PATH
3. Controleer netwerk interface naam

---

## Aanbevolen Verbeteringen

### Korte Termijn
- [ ] Battery monitoring
- [ ] Achteruit camera toevoegen
- [ ] betere odometrie (visual odometry / IMU fusion)

### Middellange Termijn
- [ ] LiDAR integratie voor SLAM
- [ ] A* path planning
- [ ] RViz visualisatie
- [ ] Return-to-home met betere positionering

### Lange Termijn
- [ ] Multi-robot coordinatie
- [ ] Autonoom opladen
- [ ] Semantic mapping

---

## Lessons Learned

1. **Waypoints werken niet** - Odometrie drift maakt nauwkeurige navigatie onmogelijk
2. **Wander mode is robuuster** - Robot kan zichzelf niet "verliezen"
3. **Minder gevoelig = beter** - Te veel escapes zorgen voor trillingen
4. **Return-to-home onbetrouwbaar** - Startpositie niet nauwkeurig genoeg
5. **Korte escapes werken beter** - Lange escapes maken robot "verward"
6. **Camera essentieel** - Zonder camera geen betrouwbare obstakel detectie

---

## Versie Geschiedenis

| Versie | Datum | Wijzigingen |
|--------|-------|-------------|
| 1.0 | 2024-02 | Initiële implementatie |
| 1.1 | 2024-02 | Velocity smoothing |
| 1.2 | 2024-02 | Stuck detection |
| 1.3 | 2024-02 | Lateral movement (vy) |
| 1.4 | 2024-02 | 3-fase escape manoeuvre |
| 1.5 | 2024-02 | Achteruit botsing detectie |
| 1.6 | 2024-02 | Python camera service |
| 2.0 | 2024-02 | C++ native RealSense integratie |
| 2.1 | 2024-02 | 4-fase escape met reverse |
| 2.2 | 2024-02 | Krappe ruimte detectie |
| 2.3 | 2024-02 | Slimme richting keuze |
| 2.4 | 2024-02 | Return-to-home toegevoegd |
| 2.5 | 2024-02 | Return-to-home verwijderd (werkte niet) |
| **3.0** | **2024-02-20** | **Wander Mode - beste resultaten** |
| 3.1 | 2024-02-20 | Vereenvoudigde escape (2-fase) |
| 3.2 | 2024-02-20 | Minder gevoelige stuck detection |
| 3.3 | 2024-02-20 | Lagere obstakel thresholds |

---

## Contact & Ondersteuning

Voor vragen of problemen, raadpleeg:
- Unitree SDK2: https://github.com/unitreerobotics/unitree_sdk2
- RealSense SDK: https://github.com/IntelRealSense/librealsense

---

## Samenvatting: Werkende Configuratie

### Commando
```bash
./slam_exploration --interface enP8p1s0 --duration 180 --area small --camera
```

### Belangrijkste Parameters
| Parameter | Waarde | Reden |
|-----------|--------|-------|
| Stuck threshold | 100 cycles (2s) | Minder vals alarm |
| Obstacle threshold | 40cm draaien, 30cm stop | Niet te voorzichtig |
| Escape duur | 0.6s + 0.6s | Kort en effectief |
| Basis snelheid | 0.40 m/s | Vloeiend bewegen |
| Wander mode | Ja | Robuuster dan waypoints |
| Return home | Nee | Odometrie onbetrouwbaar |

### Wat te vermijden
- Waypoint navigatie (odometrie drift)
- Return-to-home (werkt niet)
- Te gevoelige stuck detection (< 80 cycles)
- Te hoge obstacle thresholds (> 50cm)
- Lange escape manoeuvres (> 1s per fase)
