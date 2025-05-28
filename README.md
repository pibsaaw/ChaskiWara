![Mapa conceptual - P�gina 1 (14)](https://github.com/user-attachments/assets/d5e65ca3-f912-41d3-b574-e5982c79e921)# Plataforma de Telemetría para Globos Estratosféricos con LoRa y MQTT

Este repositorio contiene el proyecto de grado para optar al título de Licenciatura en Ingeniería Electrónica y de Sistemas de la Universidad Privada del Valle: **"Desarrollo de una Plataforma de Telemetría para la Carga Útil de Globos Estratosféricos Soportada en Redes Inalámbricas de Baja Potencia y Largo Alcance"**.

**Autor:** Rodrigo Andres Murillo Murillo
**Tutor:** Ing. Msc. Eynar Calle Viles
**URL de la plataforma (demo):** [chaskiwara.online](http://chaskiwara.online) (si sigue activa)

## 📝 Resumen del Proyecto

Este proyecto presenta el diseño e implementación de una plataforma de telemetría completa para monitorizar la carga útil de globos estratosféricos. Utiliza redes inalámbricas de baja potencia y largo alcance (LPWAN) como LoRa para la comunicación entre la carga útil (Estación Espacial) y una estación base (Estación Terrena). Los datos son luego transmitidos a un servidor en la nube (VPS Oracle) mediante MQTT para su almacenamiento, procesamiento y visualización en tiempo real a través de una interfaz web interactiva.

El sistema permite el seguimiento de variables atmosféricas y físicas, la orientación del CubeSat, y la gestión de lanzamientos, ofreciendo una solución flexible y modular para la investigación atmosférica y pruebas de concepto.

## ✨ Características Principales

*   **Adquisición de Datos:** Recopilación de datos de múltiples sensores en la carga útil (Estación Espacial):
    *   **MPU6050:** Aceleración (X, Y, Z) y giroscopio (Yaw, Pitch, Roll).
    *   **MPL3115A2:** Altitud, presión atmosférica y temperatura.
    *   **BMM150:** Orientación magnética (brújula).
    *   **GPS NEO-7M:** Coordenadas geográficas (latitud, longitud).
*   **Comunicación LoRa:** Transmisión de datos desde la Estación Espacial a la Estación Terrena.
*   **Comunicación MQTT:** Envío de datos desde la Estación Terrena al broker EMQX en el servidor VPS.
*   **Almacenamiento de Datos:**
    *   **Local (Datalogger):** Respaldo de datos en una tarjeta MicroSD en la Estación Espacial.
    *   **Nube:** Persistencia de datos en una base de datos MySQL en el servidor VPS.
*   **Visualización Web Interactiva:**
    *   **Dashboard en Tiempo Real:** Muestra los últimos datos recibidos vía MQTT, incluyendo un modelo 3D del CubeSat (Three.js) que refleja su orientación y ubicación.
    *   **Dashboards Históricos:** Gráficos (Chart.js) y mapas (Leaflet.js) para analizar datos almacenados de sensores específicos (MPU, MPL, GPS).
    *   Filtrado de datos por rango y selección de lanzamientos.
*   **Gestión de Plataforma:**
    *   Autenticación de usuarios y gestión de roles (Administrador, Trabajador, Visitante).
    *   Creación, selección y administración de lanzamientos.
*   **Arquitectura Modular:**
    *   **Estación Espacial (Carga Útil):** Basada en TTGO LoRa Promini v02 y Arduino Nano, con subsistemas de orientación, telemetría, datalogger y energía.
    *   **Estación Terrena:** Basada en TTGO T-Beam ESP32, actúa como gateway LoRa-MQTT.
    *   **Servidor VPS (Oracle Cloud):** Aloja el broker MQTT (EMQX), servidor web (Apache/Nginx), PHP y base de datos MySQL.

## 🛠️ Tecnologías Utilizadas

*   **Hardware:**
    *   Estación Espacial: TTGO LoRa Promini v02, Arduino Nano, MPU6050, MPL3115A2, BMM150, GPS NEO-7M, Módulo MicroSD, Baterías 18650.
    *   Estación Terrena: TTGO LoRa T-Beam ESP32.
*   **Comunicación:**
    *   LoRa (915 MHz)
    *   MQTT (con broker EMQX)
    *   Wi-Fi (para conexión de Estación Terrena a Internet)
    *   I2C, SPI, Serial (para comunicación interna de sensores)
*   **Software y Plataforma:**
    *   **Backend:** PHP, MySQL.
    *   **Frontend:** HTML5, CSS3, JavaScript, Bootstrap 4, Three.js, Chart.js, Leaflet.js, MQTT.js (WebSocket).
    *   **Servidor:** Oracle Cloud VPS (Ubuntu 22.04 LTS), Hestia Control Panel.
    *   **Desarrollo Embebido:** Arduino IDE.
*   **Metodología:** Scrum.

## 🏗️ Arquitectura del Sistema

El sistema se divide en tres bloques principales:

1.  **Bloque I - Estación Espacial (Carga Útil):**
    *   Subsistema de Orientación: Lee los sensores (MPU6050, MPL3115A2, BMM150, GPS).
    *   Subsistema de Telemetría: Procesa los datos y los envía vía LoRa y al Datalogger.
    *   Subsistema Datalogger: Almacena los datos en una tarjeta MicroSD.
    *   Subsistema de Energía: Gestiona la alimentación de la carga útil.

2.  **Bloque II - Estación Terrena:**
    *   Recepción de datos LoRa desde la Estación Espacial.
    *   Procesamiento y formateo de datos.
    *   Publicación de datos al broker MQTT (EMQX) vía Wi-Fi.

3.  **Bloque III - Servidor VPS Oracle:**
    *   Broker EMQX: Recibe y distribuye los mensajes MQTT.
    *   Aplicación Web (PHP/MySQL):
        *   Procesa los tópicos MQTT y almacena los datos en MySQL.
        *   Sirve la interfaz gráfica para visualización y administración.
    *   Base de Datos MySQL: Almacena datos de sensores, usuarios y lanzamientos.

![ArquitecturaChaskiWara](https://github.com/user-attachments/assets/bde2e7fe-5594-493b-9edc-d022a8029df1)


## 🚀 Cómo Funciona (Flujo de Datos)

1.  Los sensores en la **Estación Espacial** recopilan datos.
2.  El microcontrolador de la Estación Espacial procesa los datos, los envía al **Datalogger** (MicroSD) y los transmite vía **LoRa**.
3.  La **Estación Terrena** recibe los datos LoRa.
4.  La Estación Terrena se conecta a Internet vía Wi-Fi y publica los datos en tópicos específicos al **broker EMQX** en el VPS.
5.  El **Backend** de la plataforma web (Node.js/PHP escuchando MQTT o scripts) se suscribe a los tópicos, procesa los mensajes y los almacena en la base de datos **MySQL**.
6.  El **Frontend** de la plataforma web se suscribe (vía WebSockets a EMQX) para mostrar datos en tiempo real y consulta la base de datos MySQL para datos históricos.
7.  Los **usuarios** interactúan con la plataforma web para monitorizar misiones y administrar la plataforma.

## 🖼️ Vistas Previas (Screenshots)

*   Dashboard de Datos en Vivo
*   Modelo 3D del CubeSat
*   Gráfico de Aceleración (MPU6050)
*   Gráfico de Altitud/Presión/Temperatura (MPL3115A2)
*   Mapa de Trayectoria (GPS)
*   Panel de Administración de Lanzamientos
