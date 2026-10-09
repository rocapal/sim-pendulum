#!/bin/bash

# Nombre del directorio para el entorno virtual
ENV_DIR="venv"

echo "=============================================="
echo "    Configuración del Entorno de Python"
echo "=============================================="

# 1. Comprobar si existe el entorno virtual y crearlo si no es así
if [ ! -d "$ENV_DIR" ]; then
    echo "[*] Creando el entorno virtual '$ENV_DIR'..."
    python3 -m venv $ENV_DIR
    if [ $? -ne 0 ]; then
        echo "[!] Error al crear el entorno virtual. Asegúrate de tener python3-venv instalado."
        exit 1
    fi
else
    echo "[*] El entorno virtual '$ENV_DIR' ya existe. Usando el existente..."
fi

# 2. Activar el entorno virtual
echo "[*] Activando el entorno virtual..."
source $ENV_DIR/bin/activate
if [ $? -ne 0 ]; then
    echo "[!] Error al activar el entorno virtual."
    exit 1
fi

# 3. Actualizar pip por buenas prácticas
echo "[*] Actualizando pip..."
pip install --upgrade pip > /dev/null

# 4. Instalar las dependencias identificadas
echo "[*] Instalando dependencias necesarias (mujoco, numpy, scipy, pandas, matplotlib)..."
# Explicación de dependencias:
# - mujoco: Simulador físico principal.
# - numpy: Cálculos matriciales y numéricos.
# - scipy: Utilizado para la función de control LQR (scipy.linalg).
# - pandas: Tratamiento de datos (CSV) para gráficas en plot_results.py.
# - matplotlib: Generación de las gráficas de resultados.
pip install mujoco numpy scipy pandas matplotlib

if [ $? -eq 0 ]; then
    echo "=============================================="
    echo "[✔] ¡Entorno configurado e instalado con éxito!"
    echo "=============================================="
    echo "Para activar este entorno de forma manual en el futuro, ejecuta:"
    echo "    source $ENV_DIR/bin/activate"
    echo ""
else
    echo "[!] Ha ocurrido un error durante la instalación de paquetes."
    exit 1
fi
