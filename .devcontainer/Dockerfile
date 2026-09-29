# syntax=docker/dockerfile:1

# ROS 2 Jazzy desktop як базовий devcontainer-образ.
# Це лишає Ubuntu 24.04 під капотом, але одразу дає /opt/ros/jazzy,
# rviz2, rqt-інструменти і системні залежності ROS 2.
FROM osrf/ros:jazzy-desktop

# Неінтерактивний frontend, щоб apt не зависнув чекаючи відповіді на
# питання (часовий пояс, клавіатура, kernel upgrade prompt). У Dockerfile
# інтерактивного stdin немає - без цього build може зависнути назавжди.
ARG DEBIAN_FRONTEND="noninteractive"

# Пакети зі списку apt-packages.in (по одному на рядок, коментарі через #).
#
# Прапори й чистка:
# - --no-install-recommends пропускає необов'язкові залежні пакети
#   (документацію, десктопні інтеграції). Економить 200-500 MB і час.
# - rm /var/lib/apt/lists/* + apt-get clean чистять кеш apt після
#   install. Без цього layer тягне сотні MB зайвого.
# - locale-gen генерує en_US.UTF-8 щоб C++ std::cout коректно виводив
#   non-ASCII (кирилиця, UTF-8 JSON). Без цього setlocale повертає NULL
#   і stdout/stderr ламаються на кирилиці.
COPY apt-packages.in /tmp/apt.in
RUN apt update && \
  grep -vE '^\s*#' /tmp/apt.in | xargs apt install --no-install-recommends -y && \
  rm -rf /var/lib/apt/lists/* && apt-get clean -y && rm /tmp/apt.in && \
  sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
  locale-gen

# Конфіги C++ тулів кладемо в корінь /, не в home. clangd, clang-format,
# cmake-format шукають такі файли в батьківських директоріях від точки
# запуску - тобто вони знайдуться незалежно від того, де всередині
# контейнера змонтовано проєкт.
COPY .clang-format /.clang-format
COPY .clang-tidy /.clang-tidy
COPY .clangd /.clangd
COPY .cmake-format.json /.cmake-format.json
COPY cyclonedds.xml /etc/cyclonedds.xml

# Змінні середовища:
# - TZ: UTC як нейтральний дефолт. Логи завжди в UTC, без суперечок з
#   літнім часом різних регіонів.
# - LC_ALL / LANG / LANGUAGE: en_US.UTF-8 щоб stdout/stderr коректно
#   виводили UTF-8 текст (кирилиця, emoji, JSON з non-ASCII ключами).
# - RMW_IMPLEMENTATION: CycloneDDS як default middleware для ROS 2 demo.
#   Це робить discovery стабільнішим у host-network devcontainer.
# - CYCLONEDDS_URI: явний DDS config для локального devcontainer graph.
#   Config лишає autodetect interface і multicast loopback для host network.
ENV TZ=Etc/UTC
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# sudo без пароля для dev-контейнера: зручно для apt install на ходу,
# debug tools, etc. У прод-образах ніколи - там контейнер має жорстко
# визначений набір прав.
RUN echo "ALL ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# CONTAINER_USER і WORKSPACE_FOLDER приходять з devcontainer.json:
#   CONTAINER_USER   <- ${localEnv:USER}
#   WORKSPACE_FOLDER <- ${localEnv:HOME}/${localWorkspaceFolderBasename}
# Локально VS Code підставляє host username і path. У CI
# devcontainers/ci підставляє `runner` і `/home/runner/<repo>`.
ARG CONTAINER_USER=developer
ARG WORKSPACE_FOLDER=/workspace

# Noble base постачається з дефолтним ubuntu на UID 1000. Видаляємо
# його до створення CONTAINER_USER, інакше remap UID ламається на WSL
# та будь-якому хості, де host user має UID 1000.
#
# Чому: VS Code і devcontainers/ci на старті контейнера підлаштовують
# UID нашого користувача під host через механізм updateRemoteUserUID.
# Він відмовляється перепризначати UID, якщо цільовий уже зайнятий
# кимось у /etc/passwd - просто друкує "User with UID exists" і
# пропускає remap. На WSL (host UID = 1000) цільовий remap-а це 1000,
# і коли ubuntu@1000 на місці, remap скіпається, наш CONTAINER_USER
# лишається з UID 1001, а ownership bind-mount не збігається з host -
# файли створені в контейнері з хоста виглядають як чужі.
#
# Звільнивши слот 1000, дозволяємо useradd без -u взяти його. Далі
# remap або робить no-op (host UID = 1000), або переносить наш UID
# на інший вільний слот (host UID != 1000).
#
# `if id ubuntu` - щоб майбутній noble без дефолтного ubuntu не ламав build.
#
# PROMPT_COMMAND з history -a; history -n скидає історію bash у файл
# після кожної команди. У devcontainer.json цей файл змонтований з
# хоста - тому історія переживає перезапуск контейнера.
RUN if id ubuntu >/dev/null 2>&1; then \
      userdel -r ubuntu && groupdel ubuntu 2>/dev/null || true; \
    fi && \
    useradd -s /bin/bash -m ${CONTAINER_USER} && \
    printf '\nsource /opt/ros/jazzy/setup.bash\nexport PROMPT_COMMAND="history -a; history -n"\n' >> /home/${CONTAINER_USER}/.bashrc

# Всі подальші команди у контейнері (включно з процесами VS Code remote
# extension) виконуються від CONTAINER_USER, не від root.
USER ${CONTAINER_USER}
