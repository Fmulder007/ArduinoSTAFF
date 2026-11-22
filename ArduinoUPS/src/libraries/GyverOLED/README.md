# GyverOLED — локальная копия для ArduinoUPS

- Библиотека: GyverOLED
- Автор: AlexGyver
- Версия: 1.6.4
- Лицензия: MIT (оригинальный файл LICENSE сохранён рядом)
- Оригинальный репозиторий: https://github.com/GyverLibs/GyverOLED
- Зафиксированный upstream commit: 0fc70748b1bbffc325263a8f38198371e310f9a9

В этой папке хранится используемая проектом копия библиотеки. Не требуется устанавливать GyverOLED через Library Manager.

## Состав

- `src/GyverOLED.h` — исходный заголовочный файл библиотеки
- `src/charMap.h` — таблица символов, включая русские буквы и «ё»
- `library.properties` — метаданные версии

## Использование в ArduinoUPS

Скетч подключает заголовок по относительному пути из каталога проекта:

```cpp
#include "src/libraries/GyverOLED/src/GyverOLED.h"
```

Применяется SSD1306 128×32 с буфером `OLED_BUFFER`. Библиотека использует стандартный `Wire` и не требует microWire.
