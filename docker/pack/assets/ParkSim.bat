@echo off
REM ============================================================================
REM  ParkSim.bat —— Windows 侧一键入口
REM
REM  作用：
REM    1) 检查 WSL 是否可用
REM    2) 检查包是否已 install（WSL 里有没有 /media/step/data/Yccc7/ParkSim-JTH）
REM    3) 在 WSL 里拉起 run.sh（新开一个窗口显示服务日志）
REM    4) 轮询 http://127.0.0.1:8099/ 直到 200，然后自动打开浏览器
REM    5) 超时没起来 → 打印排查指引，并给 WSL IP 兜底
REM
REM  用法：双击即可。也可以命令行运行看更完整的输出。
REM  注意：本文件按 UTF-8 保存（下一行 chcp 65001 负责切换代码页）。
REM ============================================================================
chcp 65001 >nul
setlocal EnableDelayedExpansion

set "WSL_ROOT=/media/step/data/Yccc7/ParkSim-JTH"
set "PORT=8099"
set "URL=http://127.0.0.1:%PORT%/"
set "MAXTRY=40"

echo.
echo  ============================================
echo   ParkSim JTH - Windows 一键入口
echo  ============================================
echo.

REM ---------------------------------------------------------------- 1) WSL ? --
echo [1/4] 检查 WSL ...
where wsl.exe >nul 2>nul
if errorlevel 1 (
  echo.
  echo  [X] 没找到 wsl.exe —— 本机似乎没装 WSL。
  echo.
  echo  请先安装 WSL2 + Ubuntu 20.04（管理员 PowerShell 里一条命令）：
  echo      wsl --install -d Ubuntu-20.04
  echo  装完重启电脑，然后在 Ubuntu 窗口里执行本包的安装脚本：
  echo      cd /mnt/c/Users/^<你^>/Downloads/parksim-jth-portable-^<date^>
  echo      sudo bash install.sh
  echo.
  echo  也可以直接双击本包里的 ParkSim.html 看图文版指引。
  goto :FAIL
)

wsl.exe -l -q >nul 2>nul
if errorlevel 1 (
  echo  [!] wsl.exe 存在但列不出发行版，尝试继续 ...
) else (
  echo  [OK] WSL 可用。已安装的发行版：
  for /f "usebackq delims=" %%D in (`wsl.exe -l -q 2^>nul`) do echo       %%D
)

REM ------------------------------------------------------------ 2) 已安装 ? --
echo.
echo [2/4] 检查 WSL 里是否已经 install ...
wsl.exe -- test -d "%WSL_ROOT%/_ParkSim" >nul 2>nul
if errorlevel 1 (
  echo.
  echo  [X] WSL 里还没有 %WSL_ROOT%
  echo.
  echo  请先在 Ubuntu（WSL）窗口里执行：
  echo      1^) 把本包拷进 WSL，例如放到 /mnt/c/Users/^<你^>/Downloads/ 后在 WSL 里 cd 过去
  echo      2^) tar -xzf parksim-jth-portable-^<date^>.tar.gz
  echo      3^) cd parksim-jth-portable-^<date^>
  echo      4^) sudo bash install.sh
  echo.
  echo  装完再双击本文件。
  goto :FAIL
)
echo  [OK] 已找到 %WSL_ROOT%

REM -------------------------------------------------------- 3) 端口已占用 ? --
echo.
echo [3/4] 检查 %URL% 是否已经在服务 ...
call :PROBE
if "!PROBE_OK!"=="1" (
  echo  [OK] 服务已在运行，直接开页面。
  goto :OPEN
)

echo  未在服务，准备启动 ...
echo  （会新开一个窗口跑 run.sh，那个窗口就是服务日志，别关它）
start "ParkSim JTH Server (WSL)" wsl.exe -- bash "%WSL_ROOT%/run.sh"

REM ------------------------------------------------------------- 4) 轮询 ----
echo.
echo [4/4] 等待服务就绪（最多 %MAXTRY% 次，约 80 秒）...
set "TRY=0"
:WAIT
set /a TRY+=1
call :PROBE
if "!PROBE_OK!"=="1" (
  echo.
  echo  [OK] 服务就绪。
  goto :OPEN
)
if !TRY! GEQ %MAXTRY% (
  echo.
  echo  [X] 等了 %MAXTRY% 次还没起来。
  goto :TIMEOUT
)
<nul set /p "=  ."
timeout /t 2 /nobreak >nul
goto :WAIT

:OPEN
echo.
echo  正在打开 %URL% ...
start "" "%URL%"
echo.
echo  完成。关掉那个 "ParkSim JTH Server" 窗口即可停服务。
echo  （若页面空白，强制走 IPv4：%URL% —— 已经是 IPv4；也可试 http://localhost:%PORT%/）
goto :END

:TIMEOUT
echo.
echo  ------------------------------------------------------------
echo   排查指引
echo  ------------------------------------------------------------
echo   1) 看那个 "ParkSim JTH Server (WSL)" 窗口里有没有 Python 报错。
echo      最常见的两条：
echo        · ModuleNotFoundError  —— 没跑完 install.sh 的 pip 安装阶段
echo        · 资产缺失 ERROR        —— 解包不完整，重新解压
echo   2) 在 WSL 里手动跑一次，直接看日志：
echo        wsl -- bash %WSL_ROOT%/run.sh
echo   3) 确认监听的是 0.0.0.0 而不是 127.0.0.1（WSL2 是 NAT，绑 127.0.0.1
echo      Windows 侧访问不到）。日志里应该有：
echo        serving on http://0.0.0.0:%PORT%/
echo   4) localhost 转发可能失效（快速启动/休眠之后常见）。
echo      Windows PowerShell 里执行：  wsl --shutdown
echo      然后重新双击本文件。
echo   5) 兜底：直接用 WSL 的 IP。在 WSL 里执行：
echo        ip -4 addr show eth0
echo      拿到 IP（例如 172.20.x.x），浏览器开 http://^<那个IP^>:%PORT%/
echo.
echo  也可以双击 ParkSim.html，它会自己探测并给同样的指引。
echo.
set "WSLIP="
for /f "usebackq delims=" %%I in (`wsl.exe -- hostname -I 2^>nul`) do set "WSLIP=%%I"
if defined WSLIP (
  for /f "tokens=1" %%A in ("!WSLIP!") do (
    echo  检测到 WSL IP：%%A —— 可试 http://%%A:%PORT%/
  )
)
goto :FAIL

:PROBE
set "PROBE_OK=0"
curl.exe -s -o NUL --max-time 3 "%URL%" >nul 2>nul
if not errorlevel 1 set "PROBE_OK=1"
exit /b 0

:FAIL
echo.
echo  没能把服务拉起来。按任意键关闭 ...
pause >nul
endlocal
exit /b 1

:END
echo.
echo  按任意键关闭本窗口（服务窗口请单独关）...
pause >nul
endlocal
exit /b 0
