#define MyAppName "FF GUI"
#define MyAppVersion "1.0.1"
#define MyAppPublisher "RSIC"
#define MyAppExeName "ff_gui.exe"
[Setup]
AppId={{B55B4A7C-66F4-42E1-9B2B-FF0010000004}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
VersionInfoVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={autopf}\RSIC\FF GUI
DefaultGroupName={#MyAppName}
SetupIconFile=..\app\resources\ff_gui.ico
OutputDir=..\publish\installer
OutputBaseFilename=FF_GUI_Setup_v{#MyAppVersion}
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequired=admin
Compression=lzma
SolidCompression=yes
WizardStyle=modern
CloseApplications=yes
RestartApplications=yes
[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; GroupDescription: "Additional icons"
[Files]
Source: "..\publish\ff_gui_app\ff_gui.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\publish\ff_gui_app\ff_application.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\publish\ff_gui_app\CppDllMightyZap_Ver2010.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\publish\ff_gui_app\ff_config.default.json"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\publish\ff_gui_app\FF_GUI_Usage.md"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\publish\ff_gui_app\vc_redist.x64.exe"; DestDir: "{app}"; Flags: ignoreversion skipifsourcedoesntexist
Source: "..\publish\ff_gui_app\config\*"; DestDir: "{app}\config"; Flags: onlyifdoesntexist recursesubdirs createallsubdirs
[Dirs]
Name: "{commonappdata}\RSIC\FF"; Permissions: users-modify; Flags: uninsneveruninstall
Name: "{commonappdata}\RSIC\FF\position_presets"; Permissions: users-modify; Flags: uninsneveruninstall
Name: "{commonappdata}\RSIC\FF\training_sessions"; Permissions: users-modify; Flags: uninsneveruninstall
Name: "{commonappdata}\RSIC\FF\logs"; Permissions: users-modify; Flags: uninsneveruninstall
Name: "{commonappdata}\RSIC\FF\data"; Permissions: users-modify; Flags: uninsneveruninstall
[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"; Tasks: desktopicon
[Run]
Filename: "{app}\vc_redist.x64.exe"; Parameters: "/install /quiet /norestart"; StatusMsg: "Installing Microsoft Visual C++ runtime"; Flags: waituntilterminated skipifdoesntexist
Filename: "{app}\{#MyAppExeName}"; Description: "Launch FF GUI"; Flags: nowait postinstall skipifsilent
[Code]
procedure CurStepChanged(CurStep: TSetupStep);
var Src,Dest: String;
begin
  if CurStep=ssPostInstall then begin
    { Runtime configuration and positions live outside the program directory. An upgrade must
      only seed ff_config.json on a fresh installation. }
    Src:=ExpandConstant('{app}\ff_config.default.json');
    Dest:=ExpandConstant('{commonappdata}\RSIC\FF\ff_config.json');
    if not FileExists(Dest) then CopyFile(Src,Dest,False);
  end;
end;
