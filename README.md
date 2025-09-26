Пакет `clover_payload` предназначен для моделирования доставки и сброса грузов с помощью дрона **Clover** в среде симуляции **Gazebo**. Он полностью интегрируется с официальной симуляцией `clover_simulation` и **не требует модификации URDF-модели дрона**.

## 🔧 Возможности

- **Автоматический спавн груза** с привязкой к дрону при запуске симуляции.  
- **Физическое прикрепление груза** к дрону с помощью плагина [`gazebo_ros_link_attacher`](https://github.com/pal-robotics/gazebo_ros_link_attacher).  
- **Управление через ROS-сервисы**:
  - `/release_load` — отсоединяет и сбрасывает груз.  
  - `/reset_delivery` — возвращает груз под дрон и повторно прикрепляет его (работает даже во время полёта).  
- **Позиционирование без использования TF** — используется прямой запрос состояния модели из Gazebo, что обеспечивает высокую скорость и надёжность.  
- **Полная совместимость** с пакетом `clover_simulation`.

## 📦 Установка

### 1. Установка зависимостей

Перед использованием необходимо установить плагин `gazebo_ros_link_attacher`:

```bash
cd ~/catkin_ws/src
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
cd ..
catkin_make --only-pkg-with-deps gazebo_ros_link_attacher
```

Добавьте путь к Gazebo-плагинам в вашу среду:

```bash
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/catkin_ws/devel/lib' >> ~/.bashrc
source ~/.bashrc
```

> 💡 Убедитесь, что в `~/.bashrc` уже прописаны `source /opt/ros/noetic/setup.bash` и `source ~/catkin_ws/devel/setup.bash`.

### 2. Установка пакета `clover_payload`

Установите `clover_payload` и соберите рабочее пространство:

```bash
cd ~/catkin_ws/src
git clone https://github.com/petayyyy/clover_payload.git
cd ..
catkin_make
source devel/setup.bash
```

## 🚀 Использование

### Запуск симуляции с грузом

Для запуска выберите нужный launch-файл в зависимости от типа груза:

- **Телефон с поддержкой камеры**:
  ```bash
  roslaunch clover_payload payload_phone.launch
  ```

- **Телефон без поддержки камеры**:
  ```bash
  roslaunch clover_payload payload_phone_no_cam.launch
  ```

- **Коробка**:
  ```bash
  roslaunch clover_payload payload_box.launch
  ```

### Основные команды

- **Сбросить груз**:
  ```bash
  rosservice call /release_load
  ```

- **Вернуть груз в исходное положение**:
  ```bash
  rosservice call /reset_delivery
  ```

Оба сервиса возвращают статус выполнения (`success: true/false`) и текстовое сообщение.

## 🧩 Интеграция с симулятором Clover

### 1. Модификация launch-файла

Откройте файл `simulator.launch` из пакета `clover_simulation` и добавьте в конец тега `<launch>`:

```xml
<include file="$(find clover_payload)/launch/payload_phone.launch"/>
```

> 💡 Вы можете создать собственный launch-файл (за основу можно взять `payload_box.launch`), если используете несколько типов грузов.

### 2. Модификация мира Gazebo

Откройте ваш `.world`-файл (например, `clover_aruco.world`) и **внутри тега `<world>`**, **перед закрывающим `</world>`**, добавьте:

```xml
<plugin name="link_attacher" filename="libgazebo_ros_link_attacher.so"/>
```

Пример:
```xml
<world name="default">
  <!-- ... другие настройки ... -->
  <plugin name="link_attacher" filename="libgazebo_ros_link_attacher.so"/>
</world>
```

## 💡 Советы

- Масса груза по умолчанию — **1 грамм**, чтобы не влиять на динамику полёта дрона.  
- Смещение груза задаётся параметром `offset` в launch-файле: `[x, y, z]` в системе координат дрона (`base_link`).  
- Для добавления нового типа груза поместите его модель в папку `models` и укажите имя модели в соответствующем launch-файле.