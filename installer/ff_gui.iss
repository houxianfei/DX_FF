#define MyAppName "FF GUI"
#define MyAppVersion "1.0.0"
#define MyAppPublisher "RSIC"
#define MyAppExeName "ff_gui.exe"
[Setup]
AppId={{B55B4A7C-66F4-42E1-9B2B-FF0010000004}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={autopf}\RSIC\FF GUI
DefaultGroupName={#MyAppName}
OutputDir=..\publish\installer
OutputBaseFilename=FF_GUI_Setup_v{#MyAppVersion}
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequired=admin
Compression=lzma
SolidCompression=yes
WizardStyle=modern
CloseApplications=yes
[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; GroupDescription: "Additional icons"
[Files]
Source: "..\publish\ff_gui_app\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
[Dirs]
Name: "{commonappdata}\RSIC\FF"
Name: "{commonappdata}\RSIC\FF\logs"
Name: "{commonappdata}\RSIC\FF\data"
[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; WorkingDir: "{app}"; Tasks: desktopicon
[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "Launch FF GUI"; Flags: nowait postinstall skipifsilent
Filename: "{app}\vc_redist.x64.exe"; Parameters: "/install /quiet /norestart"; StatusMsg: "Installing Microsoft Visual C++ runtime"; Flags: waituntilterminated skipifdoesntexist
[Code]
procedure CurStepChanged(CurStep: TSetupStep);
var Src,Dest: String;
begin
  if CurStep=ssPostInstall then begin
    Src:=ExpandConstant('{app}\ff_config.default.json');
    Dest:=ExpandConstant('{commonappdata}\RSIC\FF\ff_config.json');
    if not FileExists(Dest) then CopyFile(Src,Dest,False);
  end;
end;
