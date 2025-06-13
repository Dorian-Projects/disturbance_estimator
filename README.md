# Estimación de Fuerzas Perturbadoras en Drones PX4

**Autores:** Dorian Miguel Flores Bonilla / Daniel González Zuñiga  
**Centro:** IES Martínez Montañés  
**Fecha:** Junio 2025

---

## Índice

1. [Introducción](#1-introducción)  
2. [Objetivo del módulo](#2-objetivo-del-módulo)  
3. [Entorno de desarrollo](#3-entorno-de-desarrollo)  
4. [Estructura del proyecto](#4-estructura-del-proyecto)  
5. [Filtro de estimación](#5-filtro-de-estimación)  
6. [Fragmento de código relevante](#6-fragmento-de-código-relevante)  
7. [Simulación y resultados](#7-simulación-y-resultados)  
8. [Comunicación entre módulos](#8-comunicación-entre-módulos)  
9. [Conclusión](#9-conclusión)  

---

## 1. Introducción

Este proyecto desarrolla un módulo en C++ para PX4 Autopilot que estima en tiempo real las fuerzas perturbadoras que afectan al dron durante el vuelo, como viento, fallos en actuadores o cargas imprevistas. Se prueba mediante simulación en Gazebo y se visualiza en QGroundControl.

---

## 2. Objetivo del módulo

Estimar la fuerza perturbadora \( F_{pert} \) que afecta al dron, usando la ecuación:

```
F_total = m * a = F_thrust + F_pert
```

---

## 3. Entorno de desarrollo

- **SO:** Ubuntu 22.04.5 LTS  
- **PX4 versión:** 1.14  
- **Simulador:** Gazebo Classic  
- **Lenguaje:** C++  
- **Herramientas:** QGroundControl, MAVSDK, VSCode, Git

---

## 4. Estructura del proyecto

Ruta del módulo:  
```
src/modules/disturbance_estimator/
```

Archivo principal:  
```
disturbance_estimator.cpp
```

El módulo accede a los tópicos:

- `sensor_combined` – Aceleración lineal  
- `vehicle_attitude` – Orientación  
- `vehicle_thrust_setpoint` – Empuje deseado

---

## 5. Filtro de estimación

Se utiliza un filtro de media móvil para estabilizar los datos de aceleración y reducir el ruido.

```cpp
history.push_back(a_measured);
if (history.size() > N) history.pop_front();
Vector3f a_filtered(0.0f, 0.0f, 0.0f);
for (const auto& a : history) a_filtered += a;
a_filtered /= history.size();
Vector3f F_pert = mass * a_filtered - thrust_vector;
```

---

## 6. Fragmento de código relevante

```cpp
Vector3f a_measured = sensor.accelerometer_m_s2;
Vector3f thrust_vector = compute_thrust_vector(thrust_sp, attitude);
Vector3f F_pert = mass * a_measured - thrust_vector;

PX4_INFO("Fuerza perturbadora: [%.2f, %.2f, %.2f] N",
         (double)F_pert(0), (double)F_pert(1), (double)F_pert(2));
```

---

## 7. Simulación y resultados

- Modelo usado: `iris` en Gazebo Classic  
- Se introducen ráfagas de viento  
- Visualización en QGroundControl mediante uORB/MAVLink

Resultados: el estimador detecta y reporta perturbaciones correctamente.

---

## 8. Comunicación entre módulos

La fuerza estimada se publica en un tópico `uORB`, permitiendo que otros módulos (como el de los rotores) reaccionen a perturbaciones.

### Publicación

```cpp
struct disturbance_estimate_s msg = {};
msg.timestamp = hrt_absolute_time();
msg.force[0] = F_pert(0);
msg.force[1] = F_pert(1);
msg.force[2] = F_pert(2);
_dist_pub.publish(msg);
```

### Recepción

```cpp
if (_dist_sub.update(&msg)) {
    PX4_INFO("F_pert recibida: [%.2f, %.2f, %.2f]",
             (double)msg.force[0], (double)msg.force[1], (double)msg.force[2]);
}
```

---

## 9. Conclusión

El módulo permite visualizar y registrar en tiempo real las fuerzas externas que afectan al dron. Su integración con otros módulos lo hace útil para control adaptativo, diagnóstico o aprendizaje automático.

---
