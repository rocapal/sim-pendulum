# Simulación de Péndulo Invertido con MuJoCo

<p align="left">
  <img src="https://img.shields.io/badge/OS-Linux-FCC624?style=flat-square&logo=linux&logoColor=black" alt="Linux">
  <img src="https://img.shields.io/badge/Python-3.x-3776AB?style=flat-square&logo=python&logoColor=white" alt="Python 3">
  <img src="https://img.shields.io/badge/Simulator-MuJoCo-233445?style=flat-square" alt="MuJoCo">
  <img src="https://img.shields.io/badge/Domain-Robotics-FF6F00?style=flat-square" alt="Robotics">
</p>

Este proyecto configura un entorno de experimentación en Linux para simular la física de un péndulo invertido (Cart-Pole) utilizando MuJoCo. La simulación emplea un modelo de concurrencia y multihilo para separar el bucle de la simulación física (motor MuJoCo) y el bucle de control (LQR).


## Estructura del Repositorio

- `pendulum.py`: Script principal de la simulación. Lanza los procesos de física y de control de manera concurrente.
- `config.json`: Archivo de configuración global (duración, frecuencia de control base, configuración de perturbaciones).
- `cart_pole.xml`: Definición del modelo físico del carro-péndulo (Cart-Pole) para MuJoCo.
- `get_lqr.py`: Script auxiliar para calcular las ganancias óptimas del controlador LQR extrayendo y linearizando el modelo desde MuJoCo.
- `setup_env.sh`: Script bash para automatizar la configuración del entorno virtual e instalación de dependencias.

## Configuración del Entorno

Para ejecutar este proyecto, necesitas Python 3. Se proporciona el script `setup_env.sh` para facilitar la creación de un entorno virtual aislado (`venv`) e instalar dependencias automáticas como `mujoco`, `numpy`, `scipy`, etc.

```bash
# 1. Dar permisos de ejecución al script
chmod +x setup_env.sh

# 2. Ejecutar el script para instalar y configurar el entorno virtual
./setup_env.sh

# 3. Activar el entorno virtual para empezar a usar el simulador
source venv/bin/activate
```

## Ejecución de la Simulación

<p align="center">
  <video src="img/pole.mp4" width="800" controls autoplay loop muted></video>
</p>

Una vez que el entorno virtual esté activo, puedes lanzar la simulación ejecutando el script principal `pendulum.py`. Por defecto, se ejecutará en modo *headless* (sin ventana) y utilizará los parámetros de `config.json`.

```bash
python3 pendulum.py
```

### Parámetros en Línea de Comandos

El script `pendulum.py` acepta una serie de argumentos para modificar su comportamiento en tiempo de ejecución:

- `--hz <valor>`: Sobreescribe la frecuencia del ciclo de control en Hz. Si no se especifica, se toma de `config.json`.
- `--render`: Despliega el visor 3D interactivo de MuJoCo, permitiéndote ver la simulación gráficamente.
- `--perturb`: Activa la inyección de perturbaciones periódicas. El péndulo recibirá un empuje aleatorio de forma regular para evaluar la robustez del controlador ante imprevistos.



**Ejemplos de uso:**

Ejecutar la simulación visualizando en 3D y con las perturbaciones activadas:
```bash
python3 pendulum.py --render --perturb
```

Ejecutar el controlador forzado a una frecuencia de 500 Hz:
```bash
python3 pendulum.py --hz 500
```

## Configuración (`config.json`)

Varios parámetros internos de funcionamiento residen en `config.json`:

- `control_hz`: Frecuencia de muestreo y ejecución (en Hz) por defecto del proceso de control.
- `simulation_duration_s`: Tiempo en segundos de duración máxima de la simulación. Pasado este tiempo se apagará de forma automática.
- `perturbation`: Configuración sobre el empuje cuando se activa con `--perturb`.
  - `interval_s`: Intervalo en segundos entre cada perturbación aleatoria.
  - `min_rads` / `max_rads`: Margen inferior y superior de la intensidad aleatoria (velocidad angular inyectada al péndulo).
