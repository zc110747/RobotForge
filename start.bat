@echo off
rem ===========================================================================
rem  RobotForge v0.1 - one-click launcher
rem ---------------------------------------------------------------------------
rem  What it does
rem    1. preflight: locate python / node, create or reuse .venv, install
rem       python deps, install frontend deps (vite and friends)
rem    2. start backend (uvicorn on 8000) and frontend (vite on 5173)
rem    3. print how to reach the app, then wait
rem
rem  Usage
rem    start.bat              full run, pauses at the end
rem    start.bat --no-pause   for scripted runs: never pause
rem    start.bat --check      preflight only, starts nothing
rem    start.bat --no-install never install, fail instead if deps are missing
rem    start.bat --help       this text (does not pause)
rem
rem  NOTE ON ENCODING - do not "fix" this later
rem    This file is plain ASCII on purpose. cmd.exe parses .bat in the OEM
rem    codepage (cp936 here). A UTF-8 Chinese byte stream gets split into
rem    mojibake AND eats the quote characters after it, which corrupts if/for
rem    parenthesis pairing - so the reported error line never matches the
rem    real one. Keep every byte of this file in the ASCII range.
rem
rem  NOTE ON CAPTURING SUBPROCESS OUTPUT - the reason for :capture
rem    `for /f "usebackq" ... in (`"C:\path with.exe" args`)` DOES NOT WORK.
rem    With a leading quote, cmd hands the whole string to the command
rem    interpreter as a single quoted program name, fails to find it, and
rem    `for /f` iterates ZERO times without any error. The variable stays
rem    empty, which is indistinguishable from "the command ran and printed
rem    nothing".
rem      => every capture in this script goes through :capture, which
rem         redirects to a file first and then reads the file.
rem    Verified on Windows 10 cmd.exe, 2026-09-16. See
rem    .workbuddy/skills/robotforge-phase-gate/references/frontend-checks.md 9.3
rem ===========================================================================

setlocal EnableExtensions EnableDelayedExpansion

rem  pause is on by default; --no-pause turns it off. --help also turns it off,
rem  because "print usage and exit" is something you call from another script
rem  and a pause there just looks like a hang.
set "PAUSE_ON=1"
set "DO_INSTALL=1"
set "CHECK_ONLY=0"

:parse_args
if "%~1"=="" goto args_done
if /i "%~1"=="--no-pause"   set "PAUSE_ON=0"   & shift & goto parse_args
if /i "%~1"=="-n"           set "PAUSE_ON=0"   & shift & goto parse_args
if /i "%~1"=="--check"      set "CHECK_ONLY=1" & shift & goto parse_args
if /i "%~1"=="--install"    set "DO_INSTALL=1" & shift & goto parse_args
if /i "%~1"=="--no-install" set "DO_INSTALL=0" & shift & goto parse_args
if /i "%~1"=="-h"           set "PAUSE_ON=0"   & goto usage
if /i "%~1"=="--help"       set "PAUSE_ON=0"   & goto usage
if /i "%~1"=="/?"           set "PAUSE_ON=0"   & goto usage
echo [ERROR] unknown option: %~1
goto usage

:args_done

rem ---------------------------------------------------------------------------
rem  Paths
rem ---------------------------------------------------------------------------
set "HERE=%~dp0"
rem  %~dp0 always ends with a backslash; strip it. A trailing backslash inside
rem  a quoted "C:\dir\" confuses several cmd builtins (it escapes the quote).
if "%HERE:~-1%"=="\" set "HERE=%HERE:~0,-1%"

set "VENV_PY=%HERE%\.venv\Scripts\python.exe"
set "FE_DIR=%HERE%\frontend"
set "NODE_MODULES=%FE_DIR%\node_modules"
set "VITE_JS=%NODE_MODULES%\vite\bin\vite.js"
set "LOG_DIR=%HERE%\.workbuddy\scratch"
set "PID_FILE=%LOG_DIR%\start-pids.txt"
set "TMP_OUT=%LOG_DIR%\start-probe.txt"

set "BACKEND_PORT=8000"
set "FRONTEND_PORT=5173"

set "BACKEND_LOG=%LOG_DIR%\start-backend.log"
set "FRONTEND_LOG=%LOG_DIR%\start-frontend.log"
set "PREFLIGHT_LOG=%LOG_DIR%\start-preflight.log"

rem  One-shot results of the preflight phase.
set "PY_CMD="
set "PY_DESC="
set "NODE_EXE="
set "NPM_KIND="
set "NPM_TARGET="
set "PORT_BUSY="

if not exist "%LOG_DIR%" mkdir "%LOG_DIR%" >nul 2>nul

echo ===========================================================================
echo   RobotForge v0.1 - one-click launcher
echo ===========================================================================
echo   repo     : %HERE%
echo   backend  : http://127.0.0.1:%BACKEND_PORT%
echo   frontend : http://127.0.0.1:%FRONTEND_PORT%
if "%CHECK_ONLY%"=="1" echo   mode     : preflight check only ^(nothing is started^)
echo.

rem ---------------------------------------------------------------------------
rem  preflight
rem ---------------------------------------------------------------------------
echo [1/2] environment check
call :check_python
if errorlevel 1 goto failed
call :check_node
if errorlevel 1 goto failed

rem  The single-instance guard MUST run before check_ports - otherwise
rem  check_ports "frees" the ports by killing the already-running instance,
rem  and the guard can never fire. Measured before this reorder: a second
rem  start.bat silently terminated the first one's backend and carried on.
if "%CHECK_ONLY%"=="0" (
  call :already_running
  if not errorlevel 1 goto ports_checked
  echo ===========================================================================
  echo   Another RobotForge launcher is already running.
  echo   It owns port %BACKEND_PORT% / %FRONTEND_PORT%.
  echo   Close that window first ^(closing it stops the servers^), then retry.
  echo   Use "start.bat --check" if you only wanted the environment check.
  echo ===========================================================================
  call :pause_if_needed
  exit /b 1
)
:ports_checked
call :check_ports
rem  In --check mode :check_ports prints its own complete verdict and pauses
rem  before returning 1, so we must NOT print the generic banner on top of it.
rem
rem  Deliberately written as a plain "if errorlevel 1 goto" followed by a
rem  separate second test, NOT as a nested parenthesised block: an "exit /b"
rem  inside a block after a "call" returned gave a wrong exit code (measured
rem  exit 0 where the check had clearly failed).
if errorlevel 1 goto ports_failed
goto ports_ok
:ports_failed
if "%CHECK_ONLY%"=="1" exit /b 1
goto failed
:ports_ok

echo.
echo [2/2] dependencies
call :ensure_python_deps
if errorlevel 1 goto failed
call :ensure_frontend_deps
if errorlevel 1 goto failed

echo.
echo   environment OK
echo.

if "%CHECK_ONLY%"=="1" (
  echo ===========================================================================
  echo   Preflight passed. Nothing was started because --check was given.
  echo   Run start.bat without --check to launch the app.
  echo ===========================================================================
  call :pause_if_needed
  exit /b 0
)

rem ---------------------------------------------------------------------------
rem  launch
rem ---------------------------------------------------------------------------
echo Launching...
echo.

rem  Backend uses tools.serve_mujoco, NOT the default app factory.
rem  The default factory wires MockBackend, whose "physics" is just "advance one
rem  step per tick". It serves fine, but it is not the Sim2Sim chain this whole
rem  project is built around. See tools/serve_mujoco.py.
echo   [1] backend  127.0.0.1:%BACKEND_PORT%  ^(MuJoCoBackend^)
pushd "%HERE%"
start "RobotForge backend" /min cmd /c ""%VENV_PY%" -m uvicorn tools.serve_mujoco:app --host 127.0.0.1 --port %BACKEND_PORT% --log-level info > "%BACKEND_LOG%" 2>&1"
popd

call :wait_port %BACKEND_PORT% 45
if errorlevel 1 (
  echo.
  echo ===========================================================================
  echo   [FAILED] backend did not listen on %BACKEND_PORT% within 45s.
  echo   Log: %BACKEND_LOG%
  echo   Last lines:
  echo   ---------------------------------------------------------------------
  call :tail_log "%BACKEND_LOG%"
  echo   ---------------------------------------------------------------------
  echo ===========================================================================
  call :cleanup
  call :pause_if_needed
  exit /b 1
)
echo   [ OK ] backend is up  ^(log: %BACKEND_LOG%^)
call :remember_pid %BACKEND_PORT%

rem  vite is invoked THROUGH node, on purpose:
rem    frontend\node_modules\.bin\vite.CMD calls `node` unpinned, so a broken or
rem    wrong node earlier on PATH hijacks it. Passing %NODE_EXE% explicitly keeps
rem    the launcher on the exact interpreter preflight verified.
echo   [2] frontend 127.0.0.1:%FRONTEND_PORT%
pushd "%FE_DIR%"
start "RobotForge frontend" /min cmd /c ""%NODE_EXE%" "%VITE_JS%" --host 127.0.0.1 --port %FRONTEND_PORT% --strictPort > "%FRONTEND_LOG%" 2>&1"
popd

call :wait_port %FRONTEND_PORT% 60
if errorlevel 1 (
  echo.
  echo ===========================================================================
  echo   [FAILED] frontend did not listen on %FRONTEND_PORT% within 60s.
  echo   Log: %FRONTEND_LOG%
  echo   Last lines:
  echo   ---------------------------------------------------------------------
  call :tail_log "%FRONTEND_LOG%"
  echo   ---------------------------------------------------------------------
  echo ===========================================================================
  call :cleanup
  call :pause_if_needed
  exit /b 1
)
echo   [ OK ] frontend is up  ^(log: %FRONTEND_LOG%^)
call :remember_pid %FRONTEND_PORT%

rem  Prove the two halves actually talk to each other.
rem  A LISTENING port is a weak signal: vite can be up while its proxy to the
rem  backend is broken. That shows up as an HTTP 502 whose body is a proxy error
rem  string - and it looks exactly like "the frontend is dead".
rem  This is the check that matters.
set "PROXY_CODE="
curl -s -o "%LOG_DIR%\start-probe.json" -w "%%{http_code}" "http://127.0.0.1:%FRONTEND_PORT%/api/robots" > "%TMP_OUT%" 2>nul
call :capture_read "%TMP_OUT%" PROXY_CODE

echo.
if "%PROXY_CODE%"=="200" (
  echo   [ OK ] proxy  /api through %FRONTEND_PORT% reaches the backend
) else (
  if "%PROXY_CODE%"=="" (
    echo   [WARN] could not run curl, so the proxy was not verified.
  ) else (
    echo   [WARN] HTTP %PROXY_CODE% from /api/robots through the vite proxy.
  )
  echo          Both servers are listening; the page may still show "cannot load".
  echo          Backend log: %BACKEND_LOG%
)

echo.
echo ===========================================================================
echo                      RobotForge is running
echo ===========================================================================
echo.
echo   Open this in your browser:
echo.
echo       http://127.0.0.1:%FRONTEND_PORT%/
echo.
echo   Backend API ^(also reachable through the frontend proxy as /api^):
echo.
echo       http://127.0.0.1:%BACKEND_PORT%/api/robots
echo       http://127.0.0.1:%BACKEND_PORT%/api/health
echo       http://127.0.0.1:%BACKEND_PORT%/docs      ^(interactive API docs^)
echo.
echo   Open port %FRONTEND_PORT%, NOT port %BACKEND_PORT%: the backend serves JSON
echo   only, the 3D view lives on the frontend.
echo.
echo   Controls: left-drag = orbit, wheel = zoom, right-drag = pan.
echo   The "WebSocket: connected" row in the left panel is the live proof that
echo   the Frontend - WS - Runtime - MuJoCo loop is up.
echo.
echo   Logs:
echo       %BACKEND_LOG%
echo       %FRONTEND_LOG%
echo.
echo ===========================================================================
echo   Press Ctrl+C or close this window to STOP the servers.
echo ===========================================================================
echo.

:wait_loop
timeout /t 3 /nobreak >nul 2>nul
call :check_alive
if errorlevel 1 (
  echo.
  echo ===========================================================================
  echo   A server process has exited. See the logs above.
  echo ===========================================================================
  call :cleanup
  call :pause_if_needed
  exit /b 1
)
goto wait_loop

rem ===========================================================================
rem  preflight steps
rem ===========================================================================

:check_python
rem  Resolution order: existing .venv, then `py -3`, then `python`.
rem
rem  Every probe writes to a file rather than using `for /f` over a quoted path
rem  (see the header note), and none of them uses a shared helper that shifts
rem  its arguments: `shift` does NOT change %* in cmd, so the "drop the label
rem  then run the rest" trick silently runs the label as part of the command.
rem  Measured result of getting that wrong: python is handed its own
rem  python.exe as a script path and dies with
rem    SyntaxError: Non-UTF-8 code starting with '\x90' in file ...python.exe
rem  which reads like "the venv is corrupt" but is purely an argument bug.
rem
rem  Setting a "command string" variable (PY_CMD) rather than an exe path is
rem  deliberate: "py -3" is a launcher that FORWARDS its arguments, so it only
rem  works as a command string. Keeping one variable for create/verify/install
rem  holds all three steps on the interpreter we actually probed - if each step
rem  re-resolved, they could drift onto different pythons.
if exist "%VENV_PY%" (
  "%VENV_PY%" -c "import sys" > "%TMP_OUT%" 2>&1
  if not errorlevel 1 (
    set "PY_CMD="%VENV_PY%"""
    set "PY_DESC=venv (.venv)"
    call :read_version "%VENV_PY%" -c "import sys;print(sys.version.split()[0])"
    rem  !PY_VER! not %PY_VER%: the whole if-exist block is parsed in one pass,
    rem  so %PY_VER% would expand to its value from BEFORE read_version ran.
    rem  Inside a parenthesized block, anything a called label sets must be read
    rem  back through delayed expansion.
    echo   python   : !PY_DESC!  !PY_VER!
    exit /b 0
  )
  rem  present but not runnable - typically the base interpreter was moved or
  rem  upgraded underneath it. Recreating is the only repair that works.
  echo   [WARN] .venv\Scripts\python.exe exists but does not run:
  call :tail_log "%TMP_OUT%"
  echo   [WARN] recreating .venv
  rmdir /s /q "%HERE%\.venv" >nul 2>nul
)

py -3 -c "import sys" > "%TMP_OUT%" 2>&1
if not errorlevel 1 (
  set "PY_CMD=py -3"
  set "PY_DESC=system (py -3)"
  call :read_version py -3 -c "import sys;print(sys.version.split()[0])"
  goto make_venv
)

python -c "import sys" > "%TMP_OUT%" 2>&1
if not errorlevel 1 (
  set "PY_CMD=python"
  set "PY_DESC=system (python)"
  call :read_version python -c "import sys;print(sys.version.split()[0])"
  goto make_venv
)

echo   [ERROR] no Python found.
echo           Install Python 3.10+ and make sure "py" or "python" is on PATH:
echo             https://www.python.org/downloads/
echo             ^(tick "Add python.exe to PATH" during setup^)
exit /b 1

:make_venv
echo   python   : !PY_DESC!  !PY_VER!
echo   [INFO] creating virtual environment in .venv ... ^(one time, ~20s^)
%PY_CMD% -m venv "%HERE%\.venv"
if errorlevel 1 (
  echo   [ERROR] "python -m venv" failed. See the output above.
  exit /b 1
)
rem  Trust but verify: a venv can be created and still not run - antivirus, no
rem  write permission, or a half-copied base interpreter.
if not exist "%VENV_PY%" (
  echo   [ERROR] venv created but %VENV_PY% is missing.
  exit /b 1
)
"%VENV_PY%" -c "import sys" > "%TMP_OUT%" 2>&1
if errorlevel 1 (
  echo   [ERROR] venv was created but its python.exe does not run:
  call :tail_log "%TMP_OUT%"
  exit /b 1
)
set "PY_CMD="%VENV_PY%""
set "PY_DESC=venv (.venv)"
echo   [ OK ] venv ready
exit /b 0

rem ---------------------------------------------------------------------------
:check_node
rem  node and npm are resolved INDEPENDENTLY, and that is on purpose.
rem  npm.cmd calls node from its own sibling directory, but a node that came
rem  from somewhere else on PATH is still perfectly usable, and npm-cli.js can
rem  be driven by any node >= 18. Coupling them would reject working setups.
rem  (This machine is exactly that case: node on PATH is the managed
rem   22.22.2-3 build while npm.cmd lives under a different node root.)
set "NODE_EXE="
set "NODE_VER="

if defined ROBOTFORGE_NODE_ROOT if exist "%ROBOTFORGE_NODE_ROOT%\node.exe" (
  set "NODE_EXE=%ROBOTFORGE_NODE_ROOT%\node.exe"
)

if not defined NODE_EXE (
  where node > "%TMP_OUT%" 2>nul
  for /f "usebackq tokens=*" %%p in ("%TMP_OUT%") do (
    if not defined NODE_EXE if exist "%%p" set "NODE_EXE=%%p"
  )
)

if not defined NODE_EXE (
  echo   [ERROR] no Node.js found.
  echo           Install Node.js 18+ ^(20 or 22 recommended^):
  echo             https://nodejs.org/en/download
  echo           Or set ROBOTFORGE_NODE_ROOT to the folder holding node.exe.
  exit /b 1
)

"%NODE_EXE%" -v > "%TMP_OUT%" 2>&1
if errorlevel 1 (
  echo   [ERROR] found "%NODE_EXE%" but it does not run:
  call :tail_log "%TMP_OUT%"
  exit /b 1
)
call :capture_read "%TMP_OUT%" NODE_VER
echo   node     : %NODE_VER%  "%NODE_EXE%"

rem  npm search. Order matters: a complete npm shipping next to node.exe is
rem  always a better bet than whatever PATH happens to expose first, because it
rem  is guaranteed to be the same release line.
set "NPM_KIND="
set "NPM_TARGET="
call :find_npm
if not defined NPM_TARGET (
  echo   [WARN] no npm found next to node.exe or on PATH.
  echo          Frontend dependencies cannot be installed automatically.
  if exist "%VITE_JS%" (
    echo          frontend\node_modules already exists, so the app can still run.
    echo          Do not delete node_modules until npm is available again.
  ) else (
    echo          And frontend\node_modules is missing, so the frontend cannot start.
  )
  exit /b 0
)

if "%NPM_KIND%"=="js" (
  "%NODE_EXE%" "%NPM_TARGET%" --version > "%TMP_OUT%" 2>&1
) else (
  call "%NPM_TARGET%" --version > "%TMP_OUT%" 2>&1
)
if errorlevel 1 (
  echo   [WARN] npm was found at "%NPM_TARGET%" but "--version" failed:
  call :tail_log "%TMP_OUT%"
  set "NPM_TARGET="
  set "NPM_VER=unknown"
  exit /b 0
)
call :capture_read "%TMP_OUT%" NPM_VER
if not defined NPM_VER set "NPM_VER=unknown"
echo   npm      : %NPM_VER%
exit /b 0

rem  Sets NPM_KIND=js   + NPM_TARGET=<path to npm-cli.js>   (preferred: unambiguous)
rem  or   NPM_KIND=cmd  + NPM_TARGET=<path to npm.cmd>
:find_npm
set "NPM_CAND="
if defined ROBOTFORGE_NODE_ROOT (
  if exist "%ROBOTFORGE_NODE_ROOT%\node_modules\npm\bin\npm-cli.js" (
    set "NPM_KIND=js"
    set "NPM_TARGET=%ROBOTFORGE_NODE_ROOT%\node_modules\npm\bin\npm-cli.js"
    exit /b 0
  )
  if exist "%ROBOTFORGE_NODE_ROOT%\npm.cmd" (
    set "NPM_KIND=cmd"
    set "NPM_TARGET=%ROBOTFORGE_NODE_ROOT%\npm.cmd"
    exit /b 0
  )
)

for %%D in ("%NODE_EXE%") do set "NPM_CAND=%%~dpD"
for %%D in ("!NPM_CAND!.") do set "NPM_CAND=%%~fD"
if exist "!NPM_CAND!\node_modules\npm\bin\npm-cli.js" (
  set "NPM_KIND=js"
  set "NPM_TARGET=!NPM_CAND!\node_modules\npm\bin\npm-cli.js"
  exit /b 0
)
if exist "!NPM_CAND!\npm.cmd" (
  set "NPM_KIND=cmd"
  set "NPM_TARGET=!NPM_CAND!\npm.cmd"
  exit /b 0
)

where npm.cmd > "%TMP_OUT%" 2>nul
for /f "usebackq tokens=*" %%p in ("%TMP_OUT%") do (
  if not defined NPM_TARGET if exist "%%p" (
    set "NPM_KIND=cmd"
    set "NPM_TARGET=%%p"
  )
)
exit /b 0

rem ---------------------------------------------------------------------------
:check_ports
rem  Reached only after :already_running found BOTH ports free, so anything
rem  listening here is some other program - not a previous RobotForge launcher.
rem
rem  A taken port must be dealt with rather than worked around: vite ships
rem  strictPort:true, so it EXITS with an error instead of hopping to 5174.
rem  Starting on top of a foreign process would produce a very confusing failure.
rem
rem  --check is a READ-ONLY preflight. Measured defect: --check used to fall
rem  through to the taskkill branch below, so running "start.bat --check" while
rem  an instance was up silently destroyed that instance's backend (PID 16572)
rem  while printing "nothing is started". A diagnostic command must not mutate
rem  the thing it diagnoses. In --check mode we therefore only REPORT and bail.
if "%CHECK_ONLY%"=="1" (
  for %%P in (%BACKEND_PORT% %FRONTEND_PORT%) do (
    set "OWNER="
    call :port_owner %%P
    if defined OWNER (
      echo   [WARN] port %%P is in use by PID !OWNER!
      echo          ^(--check does not free ports; stop that process yourself^)
      set "PORT_BUSY=1"
    )
  )
  if defined PORT_BUSY (
    echo   ports    : NOT free - see the warnings above
    echo.
    echo   Preflight found a port conflict. Nothing was stopped, because
    echo   --check never touches running processes.
    echo ===========================================================================
    echo   [FAILED] --check did not pass. Nothing was started and nothing was
    echo   stopped; the processes holding the ports are untouched.
    echo   Either stop them yourself, or just run "start.bat" - the launcher
    echo   will take the ports back from programs it can identify.
    echo ===========================================================================
    call :pause_if_needed
    exit /b 1
  )
  echo   ports    : %BACKEND_PORT% and %FRONTEND_PORT% are free
  exit /b 0
)
for %%P in (%BACKEND_PORT% %FRONTEND_PORT%) do (
  set "OWNER="
  call :port_owner %%P
  if defined OWNER (
    echo   [WARN] port %%P is used by another program ^(PID !OWNER!^) - stopping it
    taskkill /f /t /pid !OWNER! >nul 2>nul
    call :sleep 1
    set "STILL="
    call :port_owner %%P
    if defined STILL (
      echo   [ERROR] port %%P is still held by PID !STILL! after taskkill.
      echo           Close that program manually, then run start.bat again.
      exit /b 1
    )
    echo   [ OK ] port %%P released
  )
)
echo   ports    : %BACKEND_PORT% and %FRONTEND_PORT% are free
exit /b 0

rem ---------------------------------------------------------------------------
rem  dependency steps
rem ---------------------------------------------------------------------------

:ensure_python_deps
"%VENV_PY%" -c "import fastapi, uvicorn, mujoco, numpy, yaml" >nul 2>nul
if not errorlevel 1 (
  echo   python deps         : OK
  exit /b 0
)

if not "%DO_INSTALL%"=="1" (
  echo   [ERROR] python deps are missing and --no-install was given.
  echo           Re-run without --no-install, or do it by hand:
  echo             "%VENV_PY%" -m pip install -r "%HERE%\requirements.txt"
  exit /b 1
)

echo   python deps         : missing - installing (a few minutes on first run)
echo   ---------------------------------------------------------------------
"%VENV_PY%" -m pip install -r "%HERE%\requirements.txt" > "%PREFLIGHT_LOG%" 2>&1
set "PIP_RC=!errorlevel!"
type "%PREFLIGHT_LOG%"
echo   ---------------------------------------------------------------------
if not "!PIP_RC!"=="0" (
  echo   [ERROR] pip install failed ^(exit !PIP_RC!^).
  echo           Full log: %PREFLIGHT_LOG%
  echo           A proxy or an offline network is the usual cause.
  exit /b 1
)
rem  pip exiting 0 is not proof the imports work (a wheel can install and still
rem  fail to import). Re-run the same import probe that gated this branch.
"%VENV_PY%" -c "import fastapi, uvicorn, mujoco, numpy, yaml" >nul 2>nul
if errorlevel 1 (
  echo   [ERROR] pip reported success but the imports still fail.
  echo           Log: %PREFLIGHT_LOG%
  exit /b 1
)
echo   [ OK ] python deps installed
exit /b 0

rem ---------------------------------------------------------------------------
:ensure_frontend_deps
rem  Two independent gates, and they mean different things:
rem    vite/bin/vite.js exists  -> the package tree is real, not an empty folder
rem    node_modules/.bin/vite*  -> npm finished (it links .bin last, and only
rem                                after every package passed its lifecycle)
rem  Checking only "the folder exists" is the classic false green: npm dies half
rem  way through, node_modules is non-empty, and vite is not in it.
set "FRONTEND_OK=0"
if exist "%VITE_JS%" if exist "%NODE_MODULES%\.bin\vite.cmd" set "FRONTEND_OK=1"
if exist "%VITE_JS%" if exist "%NODE_MODULES%\.bin\vite" set "FRONTEND_OK=1"
if "%FRONTEND_OK%"=="1" (
  echo   frontend deps       : OK ^(vite present^)
  exit /b 0
)

if not "%DO_INSTALL%"=="1" (
  echo   [ERROR] vite is missing from frontend\node_modules and --no-install was given.
  echo           Re-run without --no-install, or do it by hand:
  echo             cd /d "%FE_DIR%" ^&^& npm install
  exit /b 1
)

if not defined NPM_TARGET (
  echo   [ERROR] vite is missing and npm is unavailable, so it cannot be installed.
  echo           Install Node.js ^(which brings npm^), then run start.bat again.
  exit /b 1
)

echo   frontend deps       : vite missing - installing (a few minutes on first run)
echo   ---------------------------------------------------------------------
pushd "%FE_DIR%"
if "%NPM_KIND%"=="js" (
  "%NODE_EXE%" "%NPM_TARGET%" install > "%PREFLIGHT_LOG%" 2>&1
) else (
  call "%NPM_TARGET%" install > "%PREFLIGHT_LOG%" 2>&1
)
set "NPM_RC=!errorlevel!"
popd
type "%PREFLIGHT_LOG%"
echo   ---------------------------------------------------------------------
if not "!NPM_RC!"=="0" (
  echo   [ERROR] "npm install" failed ^(exit !NPM_RC!^).
  echo           Full log: %PREFLIGHT_LOG%
  echo           Usual causes: no network, or a proxy blocking registry.npmjs.org
  exit /b 1
)
if not exist "%VITE_JS%" (
  echo   [ERROR] npm install reported success but frontend\node_modules\vite is still missing.
  echo           Log: %PREFLIGHT_LOG%
  exit /b 1
)
echo   [ OK ] frontend deps installed
exit /b 0

rem ===========================================================================
rem  helpers
rem ===========================================================================

rem  read_version <command string to run>
rem    Runs the command, expecting it to print a bare version string, and stores
rem    the first line of that output in PY_VER.
rem
rem    Takes the command as an argument rather than reading PY_CMD, because the
rem    whole point is to print the version AT THE MOMENT the interpreter is
rem    probed. Reading PY_CMD would re-run whatever was assigned last, which
rem    during the .venv branch is still empty - the result was a blank
rem    "python :" line that looked like the check had been skipped.
rem
rem    Called with an ALREADY-QUOTED path as the first token. `%*` reproduces the
rem    original quoting verbatim, which is the only form that survives here:
rem    the `for /f` failure described in the header is about cmd parsing a quoted
rem    path INSIDE backticks, and `%*` never goes through that path.
:read_version
%* > "%TMP_OUT%" 2>&1
call :capture_read "%TMP_OUT%" PY_VER
if not defined PY_VER set "PY_VER=?"
exit /b 0

rem  capture_read <file> <variable name>
rem    Reads the first line of a file into the named variable, trimmed.
rem    Needed because Windows tools prefix output with a UTF-8 BOM (curl -w is
rem    the one that bites here): the value comes back as "200" carrying three
rem    invisible bytes, and every string comparison against it silently fails
rem    while the value LOOKS correct in every echo.
rem
rem    The BOM is removed by dropping a leading character that is not part of any
rem    value we read back, rather than by comparing against a literal BOM -
rem    embedding raw non-ASCII bytes in this file is forbidden (see the encoding
rem    note at the top), so a literal is not an option.
:capture_read
set "CAP_FILE=%~1"
set "CAP_NAME=%~2"
set "%CAP_NAME%="
for /f "usebackq tokens=*" %%v in ("%CAP_FILE%") do (
  if not defined %CAP_NAME% (
    set "CAP_LINE=%%v"
    for /f "tokens=* delims= " %%t in ("!CAP_LINE!") do set "CAP_LINE=%%t"
    call :strip_bom CAP_LINE
    set "%CAP_NAME%=!CAP_LINE!"
  )
)
exit /b 0

rem  strip_bom <variable name>
rem    Ask whether the leading char is one of the ones that appear in the values
rem    we actually read back (digits, letters, minus, dot, v). If not, drop it -
rem    that removes a BOM in one step and is a no-op for a well-formed value.
:strip_bom
set "SB_CHK=!%~1!"
if not defined SB_CHK exit /b 0
set "SB_HEAD=!SB_CHK:~0,1!"
set "SB_KEEP=0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ-."
if "!SB_KEEP:%SB_HEAD%=!"=="!SB_KEEP!" (
  rem  leading char is not in the allowed set - strip it once
  set "%~1=!SB_CHK:~1!"
)
exit /b 0

rem  port_owner <port>
rem    Sets OWNER to the PID listening on that port, or leaves it undefined.
rem    netstat -ano prints the owning PID in column 5 for a line like
rem      TCP    127.0.0.1:8000    0.0.0.0:0    LISTENING    12345
rem    The regex pins "TCP", the port and "LISTENING" so an outbound connection
rem    that merely mentions the same number can never be mistaken for a listener.
:port_owner
set "OWNER="
for /f "usebackq tokens=5" %%a in (`netstat -ano ^| findstr /r /c:"TCP.*:%~1 .*LISTENING" 2^>nul`) do (
  if not defined OWNER set "OWNER=%%a"
)
exit /b 0

rem  wait_port <port> <seconds>  -> exit /b 1 on timeout
:wait_port
set /a WAIT_N=0
:wait_port_loop
call :port_owner %~1
if defined OWNER exit /b 0
if %WAIT_N% GEQ %~2 exit /b 1
call :sleep 1
set /a WAIT_N+=1
goto wait_port_loop

:remember_pid
rem  Record port+PID pairs so cleanup kills exactly the processes this run made.
call :port_owner %~1
if defined OWNER >> "%PID_FILE%" echo %~1 !OWNER!
exit /b 0

rem  Are both recorded listeners still alive?
:check_alive
if not exist "%PID_FILE%" exit /b 0
for /f "usebackq tokens=1,2" %%p in ("%PID_FILE%") do (
  set "FOUND=0"
  call :owner_of_is %%p %%q
  if "!FOUND!"=="0" exit /b 1
)
exit /b 0

:owner_of_is
set "FOUND=0"
for /f "usebackq tokens=5" %%a in (`netstat -ano ^| findstr /r /c:"TCP.*:%~1 .*LISTENING" 2^>nul`) do (
  if "%%a"=="%~2" set "FOUND=1"
)
exit /b 0

:already_running
rem  exit /b 1 when a previous launcher still owns either port.
rem
rem  Why not a Windows named mutex (the textbook answer)?
rem    Holding one for the whole run needs a helper process alive in parallel,
rem    and the usual VBScript trick does not survive this environment: a helper
rem    started with `start /b` gets reaped when the invoking tool call returns,
rem    so the "mutex" evaporates while the servers are still running and a
rem    second start.bat walks straight through it. A lock that silently stops
rem    working is worse than no lock.
rem
rem  Why not a PID file as the guard?
rem    PIDs are recycled. A stale file makes us either refuse to start a healthy
rem    setup, or - far worse - kill an unrelated process that inherited the PID.
rem    Both failures are silent and rare, the worst combination. The pid file is
rem    still written, but only so :cleanup knows what to kill.
rem
rem  Why not matching the server window titles with `tasklist /v`?
rem    Tested, does NOT work here: those windows are created with `start /min`
rem    from a non-interactive host and their titles never become visible. A guard
rem    that can never fire is worse than no guard - it looks like protection
rem    while providing none. (This version was written that way first; it passed
rem    review and did nothing.)
rem
rem  So: the ports. If either is LISTENING then a launcher is still running,
rem  because this script is the only thing that puts them there. It is also the
rem  same probe the rest of the script already trusts, so there is no new
rem  mechanism to get wrong.
call :port_owner %BACKEND_PORT%
if defined OWNER exit /b 1
call :port_owner %FRONTEND_PORT%
if defined OWNER exit /b 1
exit /b 0

rem  Kill exactly the processes this run created - never by image name.
rem  `taskkill /IM node.exe /F` is the tempting one-liner and it is wrong: it
rem  takes down the user's editor, other dev servers, anything node-based.
:cleanup
if exist "%PID_FILE%" (
  for /f "usebackq tokens=1,2" %%p in ("%PID_FILE%") do taskkill /f /t /pid %%q >nul 2>nul
  del /q "%PID_FILE%" >nul 2>nul
)
rem  /t should have taken the children, but a vite that already lost its shell
rem  can survive. Sweep by the window titles this script set - no other process
rem  on the machine uses those.
taskkill /f /t /fi "WINDOWTITLE eq RobotForge backend*" >nul 2>nul
taskkill /f /t /fi "WINDOWTITLE eq RobotForge frontend*" >nul 2>nul
rem  Terminating a process does not always release the port instantly.
call :sleep 2
exit /b 0

rem  tail_log <file>  - print the last ~20 lines.
rem    `type` alone is useless here: a failed uvicorn prints hundreds of lines of
rem    traceback and the informative part (the final "Error: ...") scrolls away
rem    exactly when the user needs it.
rem    The line count comes from `find /c /v ""` rather than a `for /f` over the
rem    file, because a for /f over a log would execute any percent sign or
rem    exclamation mark sitting in the captured text.
:tail_log
set "TAIL_N="
for /f "usebackq tokens=*" %%n in (`type "%~1" ^| find /c /v "" 2^>nul`) do set "TAIL_N=%%n"
if not defined TAIL_N (
  echo     ^(log is empty or unreadable^)
  exit /b 0
)
set /a TAIL_SKIP=TAIL_N-20
if %TAIL_SKIP% LSS 0 set "TAIL_SKIP=0"
if "%TAIL_SKIP%"=="0" (
  type "%~1"
) else (
  more +%TAIL_SKIP% "%~1"
)
set "TAIL_N="
set "TAIL_SKIP="
exit /b 0

:sleep
rem  `timeout` needs an interactive console and prints "ERROR: Input redirection
rem  is not supported" when stdout is redirected, so it cannot be relied on here.
rem  ping to 127.0.0.1 is the standard silent wait, and it cannot fail.
if "%~1"=="" (ping -n 2 127.0.0.1 >nul 2>nul) else (ping -n %~1 127.0.0.1 >nul 2>nul)
exit /b 0

:pause_if_needed
if not "%PAUSE_ON%"=="1" exit /b 0
echo.
echo   Press any key to close this window . . .
pause >nul
exit /b 0

rem ---------------------------------------------------------------------------
:failed
echo.
echo ===========================================================================
echo   [FAILED] preflight did not pass - nothing was started.
echo   Fix the [ERROR] or [WARN] lines above, then run start.bat again.
echo ===========================================================================
call :pause_if_needed
exit /b 1

rem  Note: there is deliberately no separate check_busy label. An earlier attempt
rem  used "goto check_busy" from inside the if-block in :check_ports, but that
rem  block is reached through a "call", and the goto did not suppress the
rem  caller's generic banner - both messages printed. The --check verdict is
rem  therefore printed in full inside :check_ports and the caller exits early.

:usage
echo.
echo   RobotForge one-click launcher
echo.
echo     start.bat                 start backend + frontend ^(pauses on exit^)
echo     start.bat --no-pause      do not pause at the end ^(scripted runs^)
echo     start.bat --check         preflight only: environment + dependencies
echo     start.bat --no-install    never install, fail if dependencies are missing
echo     start.bat --help          show this text
echo.
rem  Ports are written literally here, not via %FRONTEND_PORT%: --help jumps past
rem  the point where those variables are assigned, so referring to them would
rem  print an empty port and look like a broken script.
echo   Serves:  http://127.0.0.1:5173/   backend on 8000
echo.
call :pause_if_needed
exit /b 0
