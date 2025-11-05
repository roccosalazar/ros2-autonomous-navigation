
# ROS2 Autonomous Navigation

Progetto ROS 2 (Humble) per mappatura SLAM e navigazione autonoma basata su Nav2.  
Include setup Docker/Compose e workspace colcon per eseguire mapping e navigation in ambiente simulato (Gazebo).

---

## Struttura del progetto

```

ros2-autonomous-navigation/
├─ Dockerfile
├─ docker-compose.yml
├─ entrypoint.sh
├─ bashrc
├─ .dockerignore
├─ .gitignore
├─ README.md
└─ workspace/
    ├─ src/                # pacchetti ROS2
    └─ requirements.txt    # dipendenze Python opzionali

````

---

## Quickstart

### Opzione 1 – Esecuzione temporanea (container distrutto all'uscita)

```bash
git clone https://github.com/<tuo-username>/ros2-autonomous-navigation.git
cd ros2-autonomous-navigation

# Costruisci l'immagine Docker
docker compose build

# Avvia un container interattivo temporaneo
docker compose run --rm ros2_gazebo
````

Dentro il container:

```bash
# (già sorgente nel .bashrc, ma puoi rilanciare per sicurezza)
source /opt/ros/humble/setup.bash

cd /workspace
colcon build

# Lancia Gazebo (nuova versione Ignition)
ros2 launch ros_gz_sim gz_sim.launch.py
```

Il container si chiuderà automaticamente quando esci con `exit`.

---

### Opzione 2 – Esecuzione persistente (per aprire più terminali)

```bash
git clone https://github.com/roccosalazar/ros2-autonomous-navigation.git
cd ros2-autonomous-navigation

# Costruisci l'immagine Docker
docker compose build

# Avvia il container in background
docker compose up -d

# Apri una shell interattiva nel container
docker compose exec ros2_gazebo bash
```

Puoi aprire più terminali contemporaneamente con lo stesso comando `docker compose exec ros2_gazebo bash`.
Per terminare e rimuovere il container:

```bash
docker compose down
```

---

## Note

* Il workspace locale (`./workspace`) è montato in `/workspace` nel container.
* ROS 2 e Gazebo sono già configurati nell'ambiente (`.bashrc` fa il source automatico).
* I comandi `ign gazebo` e `ros2 launch ros_gz_sim gz_sim.launch.py` usano la nuova versione di Gazebo (Ignition).
* Per usare Gazebo Classic, installare manualmente `ros-humble-gazebo-ros-pkgs`.

---

## Requisiti

* Docker >= 24
* Docker Compose V2
* X11 configurato per permettere l'accesso grafico (`xhost +local:` se necessario)

---

## Troubleshooting

* **`cannot find name for group ID 992`**: messaggio innocuo, indica che nel container non esiste il nome del gruppo host.
* **`XDG_RUNTIME_DIR not set`**: avviso di Qt; Gazebo funziona comunque.
---

