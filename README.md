# Proyecto de Estimación de Fuerzas en Drones PX4

**Autores:** Dorian Miguel Flores Bonilla / Daniel Gonzalez Zuñiga  
**Centro:** Universidad de Sevilla  
**Repositorio:** [disturbance_estimator](https://github.com/Dorian-Projects/disturbance_estimator)  
**Fecha:** Junio 2025

## Resumen

Este proyecto desarrolla un módulo personalizado en C++ para la plataforma PX4 Autopilot, capaz de estimar en tiempo real las fuerzas perturbadoras que afectan a un dron durante el vuelo. La solución incluye simulación en Gazebo y monitorización mediante QGroundControl. El objetivo es dotar al sistema de una mayor capacidad de detección de perturbaciones externas como viento, cargas imprevistas o fallos en actuadores.

## Entorno de Desarrollo

- **Sistema Operativo:** Ubuntu 22.04.5 LTS  
- **PX4 versión:** 1.14  
- **Simulador:** Gazebo Classic  
- **Lenguaje:** C++  
- **Herramientas adicionales:** QGroundControl, MAVSDK, VSCode, Git  

## Estructura del Proyecto

El módulo se encuentra en la ruta:

```
src/modules/disturbance_estimator/
```

El archivo principal es `disturbance_estimator.cpp`, que contiene la lógica de adquisición de datos y estimación de perturbaciones.

## Objetivo del Módulo

Estimar la fuerza perturbadora `F_pert` actuando sobre el dron, partiendo de la ecuación de movimiento:

```
F_total = m * a = F_thrust + F_pert
```

El módulo accede a los siguientes tópicos de uORB:

- `sensor_combined` – Aceleración lineal  
- `vehicle_attitude` – Orientación  
- `vehicle_thrust_setpoint` – Empuje deseado  

## Fragmento de Código Relevante

```cpp
Vector3f a_measured = sensor.accelerometer_m_s2;
Vector3f thrust_vector = compute_thrust_vector(thrust_sp, attitude);
Vector3f F_pert = mass * a_measured - thrust_vector;
PX4_INFO("Fuerza perturbadora: [%.2f, %.2f, %.2f] N",
         (double)F_pert(0), (double)F_pert(1), (double)F_pert(2));
```

## Simulación y Resultados

La simulación se realizó en Gazebo con el modelo estándar `iris`. Se introdujeron ráfagas de viento para comprobar la sensibilidad del estimador. Los resultados se visualizaron mediante PX4 `uORB` y MAVLink Inspector en QGroundControl.

## Conclusión

El módulo desarrollado permite estimar y visualizar perturbaciones externas en vuelo de forma efectiva. Esta funcionalidad puede integrarse en controladores adaptativos o sistemas de diagnóstico para aumentar la resiliencia del dron.

## Implementación del Filtro de Estimación

Se implementó un filtro de media móvil sobre la aceleración para mejorar la estabilidad de la estimación.

```cpp
history.push_back(a_measured);
if (history.size() > N) history.pop_front();
Vector3f a_filtered(0.0f, 0.0f, 0.0f);
for (const auto& a : history) a_filtered += a;
a_filtered /= history.size();

Vector3f F_pert = mass * a_filtered - thrust_vector;
```

## Registro de Estimaciones y Comunicación entre Módulos

La fuerza estimada se registra y publica en un tópico personalizado `disturbance_estimator`.

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

## Integración con Módulo de Control de Actitud

En `mc_att_control`, se suscribe al tópico `disturbance_estimator` y compensa el empuje con la fuerza perturbadora:

```cpp
if (disturbance_sub.update(&dist)) {
    const float drone_mass = 1.5f;
    _thrust_setpoint_body(0) += dist.force_x / drone_mass;
    _thrust_setpoint_body(1) += dist.force_y / drone_mass;
    _thrust_setpoint_body(2) += dist.force_z / drone_mass;
}
```

Esto permite una respuesta adaptativa del controlador ante perturbaciones externas, mejorando la estabilidad del dron.
