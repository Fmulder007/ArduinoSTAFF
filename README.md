# ArduinoSTAFF

Набор сохранённых инструментов и локальных настроек для работы с Arduino на Windows. Репозиторий содержит не полную Arduino IDE, а отдельные исполняемые файлы, заменяемые компоненты и скрипты сборки.

## Содержимое репозитория

```text
ArduinoSTAFF/
├── ArduinoCLI/
│   ├── arduino-cli.exe
│   └── arduino-cli-offline.cmd
├── ArduinoIDE1.8.19/
│   ├── arduino-core.jar
│   └── README_arduino-core_no-cloud-resolver.md
└── BuildScripts/
    └── Notepad++/
        └── Arduino_GyverCore_F7_NppExec.txt
```

### Arduino CLI

- `ArduinoCLI/arduino-cli.exe` — сохранённая сборка Arduino CLI для Windows.
- `ArduinoCLI/arduino-cli-offline.cmd` — командный файл для запуска расположенного рядом `arduino-cli.exe`.

Командный файл не является сетевым блокировщиком: он только запускает локальный CLI и передаёт ему аргументы. Сетевые команды CLI по-прежнему могут обращаться к серверам. Скрипт сборки из Notepad++ выполняет компиляцию уже установленной платформой и не запускает обновление индексов или загрузку плат.

Чтобы использовать CLI по пути, указанному в скрипте Notepad++, скопируйте:

`ArduinoCLI/arduino-cli.exe` → `C:\ArduinoCLI\arduino-cli.exe`

Создайте каталог `C:\ArduinoCLI`, если его ещё нет. При желании скопируйте туда и `arduino-cli-offline.cmd`.

Проверить версию установленного файла можно командой:

```cmd
C:\ArduinoCLI\arduino-cli.exe version
```

Если CLI хранится в другом месте, измените путь к `arduino-cli.exe` в NppExec-скрипте.

### Arduino IDE 1.8.19

В `ArduinoIDE1.8.19` находится изменённый `arduino-core.jar`. Он собран из исходников **Arduino IDE 1.8.19** с отключённым облачным поиском платы по неизвестным USB VID/PID.

**Куда подменить файл:**

1. Полностью закройте Arduino IDE.
2. Откройте каталог установленной IDE. Для стандартной установки это:

   `C:\Program Files (x86)\Arduino\lib\`

3. Сохраните штатный файл, например переименуйте:

   `arduino-core.jar` → `arduino-core_old.jar`

4. Скопируйте из репозитория файл:

   `ArduinoIDE1.8.19/arduino-core.jar`

   в каталог `C:\Program Files (x86)\Arduino\lib\`, чтобы полный путь был:

   `C:\Program Files (x86)\Arduino\lib\arduino-core.jar`

5. Запустите Arduino IDE.

Отключён только облачный запрос `BoardCloudResolver` при обнаружении неизвестного USB VID/PID. Локальное обнаружение COM-портов и распознавание известных плат не удалялись. Boards Manager, Library Manager и другие сетевые функции IDE этим изменением не отключаются.

**Откат:** закройте IDE, удалите изменённый `arduino-core.jar` и переименуйте `arduino-core_old.jar` обратно в `arduino-core.jar`.

Подробности и сведения о сборке находятся в [README компонента Arduino IDE](ArduinoIDE1.8.19/README_arduino-core_no-cloud-resolver.md).

## Сборка скетча из Notepad++

Скрипт вынесен отдельно от CLI:

`BuildScripts/Notepad++/Arduino_GyverCore_F7_NppExec.txt`

Он предназначен для Notepad++ с установленным плагином **NppExec** и платформой **GyverCore 2.0.4**. Скрипт компилирует активный скетч, но **не загружает прошивку в плату** и не обращается к COM-порту.

### Что требуется заранее

- Notepad++ с плагином NppExec;
- CLI установлен по пути `C:\ArduinoCLI\arduino-cli.exe`;
- платформа GyverCore 2.0.4 и её инструменты установлены в Arduino data directory текущего пользователя;
- скетч сохранён на диск.

Скрипт использует FQBN:

```text
GyverCore:avr:nano:boot=no,clock=external_16,serial=gyverUart,compiler_version=avrgcc8,timers=yes_millis,init=enable,es=enabled,bod=bod_2_7,co=disabled
```

Это конкретная конфигурация платы и параметров GyverCore. Для другой платформы или платы необходимо изменить параметр `--fqbn` в скрипте.

### Первый запуск в Notepad++

1. Откройте в Notepad++ основной файл скетча `.ino`.
2. Убедитесь, что файл сохранён. Активной должна быть вкладка именно главного `.ino`, а не заголовочный файл или вкладка другого проекта.
3. Проверьте, что имя главного файла совпадает с именем папки скетча — это стандартное требование Arduino CLI.
4. Откройте файл `BuildScripts/Notepad++/Arduino_GyverCore_F7_NppExec.txt` из этого репозитория.
5. Скопируйте его содержимое целиком.
6. В Notepad++ откройте **Плагины → NppExec → Execute…** (обычно клавиша **F6**) и вставьте текст скрипта в окно выполнения.
7. Запустите скрипт кнопкой **OK/Execute** в этом окне.

Для повторного использования сохраните текст в NppExec как отдельный скрипт, например `Arduino_GyverCore_F7_Build` (кнопка **Save…** в окне Execute, если она доступна в установленной версии плагина). После сохранения его можно запускать из меню **Плагины → NppExec → Execute…** или из списка сохранённых скриптов в меню NppExec.

### Что создаётся

Файлы сборки сохраняются в папке самого скетча:

- `.hex` — прошивка;
- `.elf` — файл с отладочной информацией;
- `.lst` — дизассемблированный листинг с исходным кодом;
- `.sym` — таблица символов.

Если компиляция или создание листинга/таблицы символов завершится ошибкой, скрипт остановится и покажет сообщение в консоли NppExec.

### Пути, которые можно изменить

В начале скрипта указан путь к CLI:

`C:\ArduinoCLI\arduino-cli.exe`

Два вызова инструментов `avr-objdump.exe` и `avr-nm.exe` используют переменную Windows `%LOCALAPPDATA%`, поэтому не зависят от имени пользователя Windows. В пути зафиксирована версия пакета GyverCore **2.0.4**:

`%LOCALAPPDATA%\Arduino15\packages\GyverCore\hardware\avr\2.0.4\tools\avr-gcc\bin\`

Если установлена другая версия GyverCore или пакет находится в другом каталоге, поправьте оба пути к инструментам в скрипте.

---

**Важно:** в репозитории хранятся отдельные компоненты и скрипты. Полную Arduino IDE, платформу GyverCore, библиотеки и конфигурацию Arduino CLI он не заменяет.
