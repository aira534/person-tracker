# person-tracker
Este paquete sigue a una persona cuando sale del campo de visión de la cámara y la reconoce cuando vuelve a entrar.

*English version: [README_en.md](README_en.md)*

Proyecto desarrollado en un ordenador con una unidad de procesamiento gráfico *(GPU) NVIDIA RTX 4060*, con la cámara `oak_d_camera` de *Luxonis* y el LiDAR *RPLIDAR*.

## Funcionamiento
El sistema combina la cámara y el LiDAR mediante varios nodos:
- **DetectionTo3DfromDepthNode:** recibe las detecciones de YOLO, las pasa a 3D con la imagen de profundidad y las transforma a `odom`. Publica en `/detections_3d_odom`.
- **ObjectProcessingNode:** segmenta el scan del LiDAR en objetos y publica sus centros en `odom` en `/lidar_objects`.
- **PersonTrackerNode:** fusiona ambas fuentes. Solo la cámara crea entidades (personas) y el LiDAR las actualiza, también fuera del campo de visión de la cámara, prediciendo su posición con la velocidad. Publica en `/tracked_entities`.
- **TrackerDebugNode:** publica los marcadores para RViz en `/debug/tracked_entities_markers`.

**COMANDOS**
Activar `debug` para visualizar las entidades en RViz.
```bash
ros2 launch person_tracker person_tracker.launch.py debug:=true
```

## Ramas
- **cambios-viejos:** copia de seguridad de mi enfoque original (commit `64d82c5`), en el que el LiDAR creaba una entidad por cada objeto detectado.
- **main:** contiene mi boceto de cambios (`05f2347`, que no compilaba) y la versión corregida que explico a continuación.

## Troubleshooting
### Problema de enfoque
En mi enfoque original (rama `cambios-viejos`) los nodos estaban conectados así:
- **ObjectProcessingNode:** segmentaba el scan del LiDAR y publicaba los centros de los objetos en `/lidar_objects`.
- **LidarTrackerNode:** seguía todos los objetos del LiDAR. Creaba una entidad nueva por cada centro que no estuviera asociado, la asociaba en los siguientes scans con el punto más cercano (a menos de 0.6 m), predecía su posición con la velocidad y la borraba tras `max_misses` scans sin verla. Publicaba las entidades en `/tracked_entities`.
- **DetectionTo3DfromDepthNode:** pasaba las detecciones de YOLO a 3D en `odom` y las publicaba en `/detections_3d_odom`.
- **PersonTrackerNode (`sensor_fusion`):** la idea era sincronizar `/tracked_entities` y `/detections_3d_odom` con `message_filters` (`ApproximateTime`) y decidir qué entidad del LiDAR era cada persona detectada por YOLO. Este nodo no llegué a implementarlo.

Al probarlo me di cuenta de que este enfoque era inviable. El LiDAR detecta también paredes, patas de sillas, columnas o piernas separadas, y en los bags daba entre 20 y 30 objetos por scan. Como el LiDAR creaba una entidad por cada uno, había tantas entidades trackeándose a la vez, muchas de ellas muy cerca unas de otras, que al asociar por cercanía los IDs se intercambiaban todo el rato, y además aparecían puntos fantasma. Con IDs del LiDAR que cambiaban constantemente, el nodo de fusión no podía saber qué entidad era cada persona, así que no tenía sentido emparejarlas después.

Para solucionarlo cambié la arquitectura: ahora solo la cámara crea entidades, ya que solo YOLO sabe que algo es una persona, y el LiDAR únicamente actualiza las entidades que ya existen. Los objetos del LiDAR que no se asocian a ninguna entidad se ignoran.

### Otros cambios
Además del cambio de arquitectura, realicé los siguientes cambios respecto a mi enfoque original (`64d82c5`). Todos los justifiqué midiendo con los bags de `bags/corrected_bags`.

#### Tracker
- **Asociación greedy global:** antes cada entidad, por orden, cogía el punto libre más cercano, por lo que la primera de la lista podía "robarle" el punto a otra. Creé la función `associate_closest`, que calcula todas las parejas entidad–punto a menos del margen, las ordena de más cerca a más lejos y las asigna sin repetir. La usan el LiDAR (margen de 0.6 m) y la cámara (1.0 m). Con varias personas cerca se cruzaban los IDs, y con dos personas juntas la segunda no llegaba a crearse.
- **La cámara mantiene viva una entidad que el LiDAR ha perdido:** si la cámara ve a una persona cuya entidad tiene `misses > 0`, le asigna la posición de la cámara y pone `misses = 0`. Si no, alguien que la cámara ve pero el LiDAR no (lejos, pegado a otro objeto o tapado por otra persona) se borraba tras `max_misses` scans.

  Para comprobar si esta regla hacía falta, pasé los 9 bags con y sin ella (una ejecución por versión). Sin la regla salieron 68 IDs en total, frente a 40 con ella, y ningún bag mejoró. Las mayores diferencias aparecieron con oclusiones, porque el LiDAR solo ve un plano a la altura de las piernas y se tapa con facilidad, mientras que la cámara ve a la persona entera:
  - **duo_reid:** 14 IDs sin la regla frente a 4 con ella.
  - **trio_partial_oclusion:** 18 IDs sin la regla frente a 6 con ella.
  - **Resto de bags:** diferencias de 0 a 2 IDs.
- **Velocidad solo entre posiciones del LiDAR:** añadí el campo `position_from_lidar` para calcular la velocidad solo entre dos posiciones del LiDAR. Restar una posición de la cámara y otra del LiDAR produce picos falsos, porque cada sensor tiene su propio error.
- **Filtro de velocidad:** añadí una media móvil exponencial `v = 0.3·v_medida + 0.7·v_anterior`, que se fía un 30 % de la velocidad nueva y un 70 % de la anterior. Lo hice así porque la velocidad calculada directamente entre dos posiciones del LiDAR tenía a veces picos muy grandes, por ejemplo por el parpadeo de algunos centroides, y con ellos la predicción se iba lejos de la persona y se cambiaba el ID.
- **Predicción durante las oclusiones:** antes solo predecía si la entidad se había visto en el scan anterior (`misses == 0`) y durante menos de 0.4 s, y al perderla ponía la velocidad a 0. Lo había hecho así para evitar los saltos de velocidad, pero los resolví de otra forma con la media móvil exponencial (ver el filtro de velocidad). Además, conceptualmente no tenía sentido para mi enfoque, ya que la idea es predecir justo cuando no se ve a la persona. Ahora predigo mientras la entidad exista (`dt > 0`), el límite lo marca `max_misses`, y la velocidad ya no se pone a 0.

#### Otros nodos
- **Profundidades inválidas:** en `DetectionTo3DfromDepthNode`, además de `NaN`, ahora descarto las profundidades de 0 (píxel sin dato) y las mayores de 8 m. Generaban detecciones encima del robot o a 30 m que creaban fantasmas, sobre todo con la persona en el borde de la imagen.

#### Parámetros
- **max_dist de 0.50 a 0.30:** en `config/params.yaml`, es la distancia máxima entre dos puntos consecutivos del scan para considerarlos el mismo objeto. Con 0.5, una persona delante de una pared se fundía con ella en un único objeto demasiado ancho, que se descartaba por superar `max_obj_size`.
- **max_misses de 5 a 8:** en `config/params.yaml`, es el número de scans seguidos sin ver una entidad tras los que se borra. Con 5 las entidades se borraban demasiado pronto cuando el LiDAR perdía a la persona un momento, y al volver recibía un ID nuevo. Probé con 10 y se fragmentaban menos, pero a veces la entidad sobrevivía a la pérdida de la persona y se enganchaba a un objeto fijo cercano que el LiDAR ve en cada scan, quedándose ahí para siempre. Lo dejé en 8 como compromiso entre los dos casos.

#### Reorganización
Moví todo el código del tracker a `PersonTrackerNode` (nodo `person_tracker_node`) y eliminé `LidarTrackerNode` y `lidar_tracker_main`. En `params.yaml` renombré la sección `detections_tracking_node` a `person_tracker_node`.

### Observaciones
- **La cámara y el LiDAR llegan en momentos distintos:** por eso no sincronizo los dos sensores en el tracker, ya que al esperar a tener un mensaje de cada uno las posiciones estarían desactualizadas. Cada sensor actualiza las entidades en cuanto llega su mensaje. Mi hipótesis es que las detecciones de la cámara llegan más tarde por el procesamiento de YOLO y de la profundidad.
- **Variación entre ejecuciones:** los resultados cambian de una ejecución a otra probablemente porque el tiempo de procesamiento de YOLO no es siempre el mismo. Para comparar versiones hay que repetir cada bag varias veces.

## Metodología
He desarrollado varios scripts en [scripts/eval](/scripts/eval) para evaluar el tracker con los bags:
- **[run_bags.sh](/scripts/eval/run_bags.sh):** lanza YOLO y el tracker con cada bag, graba los topics y al terminar muestra una tabla con el número total de IDs de cada bag frente al número real de personas. Usa `ROS_DOMAIN_ID=77` para no interferir con otros procesos.
- **[ids.py](/scripts/eval/ids.py):** muestra cuándo y dónde nace y muere cada ID, y en qué porcentaje de scans lo vio el LiDAR.

**COMANDOS**
```bash
source ~/TFG_ws/install/setup.bash
src/person_tracker/scripts/eval/run_bags.sh <carpeta_resultados> [bags]
```

## Limitaciones conocidas
- **Pérdida por ambos sensores:** si la cámara y el LiDAR pierden a la persona, al volver recibe un ID nuevo.
- **Cruces fuera de la cámara:** si la persona seguida pasa junto a otra persona quieta, o a un objeto fijo, fuera del campo de visión de la cámara, el ID puede saltar a ese otro objeto (bag `cruzar`).
- **Fantasmas:** un error de profundidad puntual puede crear un ID que desaparece enseguida.
