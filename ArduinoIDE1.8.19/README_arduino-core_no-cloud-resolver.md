# Arduino IDE 1.8.19 — arduino-core.jar без облачного определения платы

## Для какой версии

- **Arduino IDE:** 1.8.19
- **Платформа:** Windows x64
- **Изменённый компонент:** `arduino-core.jar`

Этот JAR собран из исходников Arduino IDE 1.8.19 с небольшим изменением в `SerialDiscovery.java`.

## Что изменено

В файле

`arduino-core/src/cc/arduino/packages/discoverers/serial/SerialDiscovery.java`

отключён автоматический облачный запрос `BoardCloudResolver` для USB-устройств с неизвестными VID/PID.

Раньше при обнаружении неизвестного USB VID/PID Arduino IDE могла обращаться к адресу вида:

`https://builder.arduino.cc/builder/v1/boards/0x<VID>/0x<PID>`

В этой сборке удалены:

- импорт `processing.app.helpers.BoardCloudResolver`;
- создание объекта `BoardCloudResolver`;
- вызов `boardCloudResolver.getBoardBy(...)` при обнаружении неизвестного устройства.

Локальное обнаружение последовательных портов и локальное сопоставление известных VID/PID оставлены без изменений.

**Важно:** это изменение отключает именно указанный облачный запрос при последовательном обнаружении устройств. Оно не отключает все сетевые функции Arduino IDE: Boards Manager, Library Manager, обновление индексов и сетевое обнаружение устройств остаются отдельными механизмами.

## Куда положить

Закрой Arduino IDE и помести файл `arduino-core.jar` в каталог:

`C:\Program Files (x86)\Arduino\lib\`

Полный путь к файлу:

`C:\Program Files (x86)\Arduino\lib\arduino-core.jar`

Перед заменой штатный файл был сохранён под именем:

`C:\Program Files (x86)\Arduino\lib\arduino-core_old.jar`

После замены запусти Arduino IDE обычным способом.

## Как вернуть исходный файл

1. Закрой Arduino IDE.
2. Удали или переименуй изменённый `arduino-core.jar`.
3. Переименуй `arduino-core_old.jar` обратно в `arduino-core.jar`.
4. Запусти Arduino IDE.

## Как собран файл

Сборка выполнена из исходников Arduino IDE 1.8.19 в каталоге:

`D:\temp\Arduino-1.8.19`

Использовались:

- Amazon Corretto JDK 8.512 (`D:\temp\jdk1.8.0_512`);
- Apache Ant 1.10.18 (`D:\temp\apache-ant-1.10.18`).

Команда сборки:

```cmd
cd /d D:\temp\Arduino-1.8.19\arduino-core
ant build
```

Результат сборки:

`D:\temp\Arduino-1.8.19\arduino-core\arduino-core.jar`

Ant завершил сборку сообщением `BUILD SUCCESSFUL`.

---

**Назначение файла:** локальная сборка `arduino-core.jar` для Arduino IDE 1.8.19 с отключённым облачным поиском платы по неизвестным USB VID/PID.
